"""Recording reader/summary and labelling logic on synthetic files (no camera)."""
import csv
import json
from pathlib import Path
import sys
import tempfile
import unittest
import zlib

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from gemini_recording import load_depth_m, load_depth_raw, summarize  # noqa: E402
from label_recording import FALL_EVENTS, LabelSession  # noqa: E402
from record_gemini_mac import PLAN, resolve_segment  # noqa: E402
from evaluate_falls import aggregate, falls_and_actions, score  # noqa: E402


def make_recording(root, color_n=90, depth_n=80, gap_at=None, compressed=True):
    root = Path(root)
    t0 = 1_800_000_000_000_000_000
    with (root/'color_timestamps.csv').open('w', newline='') as handle:
        w = csv.writer(handle); w.writerow(['frame', 'host_wall_ns', 'host_monotonic_ns'])
        for i in range(color_n):
            extra = 1_500_000_000 if gap_at is not None and i >= gap_at else 0
            w.writerow([i+1, t0+i*33_333_333+extra, 5_000_000_000+i*33_333_333+extra])
    with (root/'timestamps.csv').open('w', newline='') as handle:
        w = csv.writer(handle); w.writerow(['frame', 'color_ms', 'depth_ms', 'scale_mm', 'host_wall_ns'])
        for i in range(depth_n):
            w.writerow([i+1, 0, 1000+i*33, 1, t0+200_000_000+i*33_333_333])
            depth = np.full((400, 640), 1000+i, np.uint16)
            depth[:50] = 0
            data = depth.tobytes()
            if compressed:
                (root/f'{i+1}.depth.z').write_bytes(zlib.compress(data, 1))
            else:
                (root/f'{i+1}.depth').write_bytes(data)
    return root


class RecordingTests(unittest.TestCase):
    def test_compressed_and_raw_depth_and_summary(self):
        for compressed in (True, False):
            with tempfile.TemporaryDirectory() as tmp:
                root = make_recording(tmp, compressed=compressed)
                self.assertEqual(int(load_depth_raw(root, 3)[200, 10]), 1002)
                row = dict(frame='5', scale_mm='1')
                self.assertAlmostEqual(float(load_depth_m(root, row)[300, 300]), 1.004, places=6)
                s = summarize(root)
                self.assertEqual((s['color_frames'], s['depth_rows'], s['depth_files_missing']), (90, 80, 0))
                self.assertAlmostEqual(s['overlap_seconds'], 79*.033333333, delta=.01)
                self.assertEqual(s['problems'], [])

    def test_gap_missing_and_corrupt_files_are_reported(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_recording(tmp, gap_at=40)
            (root/'7.depth.z').unlink()
            (root/'1.depth.z').write_bytes(zlib.compress(b'short', 1))
            s = summarize(root, check_depth_files=80)
            self.assertGreater(s['color_max_gap_seconds'], 1.)
            self.assertEqual(s['depth_files_missing'], 1)
            self.assertTrue(s['depth_files_bad_in_sample'])
            self.assertEqual(len(s['problems']), 2)


class LabelTests(unittest.TestCase):
    def session(self, n=300):
        wall = [1_800_000_000_000_000_000+i*33_333_333 for i in range(n)]
        mono = [10_000_000_000+i*33_333_333 for i in range(n)]
        return LabelSession('A1', wall, mono)

    def keys(self, session, text):
        for char in text:
            session.key(char)

    def test_fall_sequence_times_relative_to_marker(self):
        s = self.session()
        self.keys(s, 'c'*3+'m')             # marker at frame 30
        self.keys(s, 'c'+'1'+'dd2'+'d3'+'c4'+'c5')
        rows = s.rows()
        self.assertEqual([r['event'] for r in rows], list(FALL_EVENTS))
        self.assertEqual(rows[0]['t_s'], '0.333')       # 10 frames after the marker
        self.assertEqual(rows[0]['frame'], 41)
        self.assertEqual(rows[0]['direction'], 'side_left')
        self.assertEqual(s.problems(), [])

    def test_problems_and_undo(self):
        s = self.session()
        self.keys(s, '2')                    # impact before onset, no marker
        problems = s.problems()
        self.assertTrue(any('marker' in p for p in problems))
        self.assertTrue(any('out of order' in p for p in problems))
        self.keys(s, 'u')
        self.assertEqual(s.labels, [])
        self.keys(s, 'mnb')                  # action_start fast_floor_sit, never closed
        self.assertTrue(any('without end' in p for p in s.problems()))
        self.keys(s, 'ce')
        self.assertEqual(s.problems(), [])

    def test_cycles_and_save_load_round_trip(self):
        s = self.session()
        self.keys(s, 'mrppk')                # direction side_right, person A+B, position 3m
        self.keys(s, 'c1')
        self.assertEqual((s.direction, s.person, s.position), ('side_right', 'A+B', '3m'))
        with tempfile.TemporaryDirectory() as tmp:
            s.save(tmp)
            meta = json.loads(Path(tmp, 'labels-meta.json').read_text())
            self.assertEqual(meta['marker_frame'], 1)
            restored = self.session()
            restored.load(tmp)
            self.assertEqual(restored.rows(), s.rows())
            header = Path(tmp, 'labels.csv').read_text().splitlines()[0]
            self.assertEqual(header, 'segment,t_s,event,person,position,direction,notes,frame,host_wall_ns')

    def test_navigation_is_clamped(self):
        s = self.session(5)
        self.keys(s, 'zzz')
        self.assertEqual(s.frame, 0)
        self.keys(s, 'cc')
        self.assertEqual(s.frame, 4)
        self.assertIsNone(s.key('x'))
        self.assertEqual(s.key('q'), 'quit')


class SegmentNameTests(unittest.TestCase):
    def test_plan_codes_free_names_and_parts(self):
        with tempfile.TemporaryDirectory() as tmp:
            out, seconds = resolve_segment('a1A', tmp, '20261010')
            self.assertEqual((out.name, seconds), ('gemini-20261010-A1a-fall-side-1.9m', 120))
            out.mkdir(); (out/'x').touch()
            self.assertEqual(resolve_segment('A1a', tmp, '20261010')[0].name, 'gemini-20261010-A1a-fall-side-1.9m-part2')
            self.assertEqual(resolve_segment('my-test', tmp, '20261010'), (Path(tmp)/'gemini-20261010-my-test', 120))
            for bad in ('', '../x', '.hidden'):
                with self.assertRaises(ValueError):
                    resolve_segment(bad, tmp, '20261010')
            self.assertTrue(all(0 < s <= 170 for _, s in PLAN.values()))


class EvaluateTests(unittest.TestCase):
    def labels(self):
        rows = []
        for base, direction in ((10., 'side_left'), (40., 'toward')):
            for offset, event in zip((0., .8, 1.1, 5., 7.), ('fall_onset', 'impact', 'lying_start', 'getup_start', 'stand_stable')):
                rows.append(dict(t=base+offset, event=event, direction=direction, person='A', position='1.9m', notes=''))
        rows += [dict(t=70., event='action_start', notes='fast_lie_down', direction='', person='A', position=''),
                 dict(t=72., event='action_end', notes='fast_lie_down', direction='', person='A', position='')]
        return rows

    def test_detection_latency_false_alarm_and_stray(self):
        falls, actions = falls_and_actions(self.labels())
        self.assertEqual((len(falls), len(actions)), (2, 1))
        timeline = [dict(t=t/10, phase=0, basis='2d') for t in range(0, 1000)]
        for r in timeline:
            if 11.0 <= r['t'] <= 14.0: r['phase'] = 1
            if 12.2 <= r['t'] <= 14.0: r['phase'], r['basis'] = 2, '3d'
            if 71.0 <= r['t'] <= 71.5: r['phase'] = 1          # false alarm on the negative
            if 90.0 <= r['t'] <= 90.3: r['phase'] = 1          # outside any label
        rows, stray = score(timeline, falls, actions)
        first, second, action = rows
        self.assertTrue(first['suspected'] and first['confirmed'])
        self.assertEqual((first['suspected_latency_s'], first['confirmed_latency_s'], first['confirmed_basis']), (.2, 1.4, '3d'))
        self.assertFalse(second['suspected'])
        self.assertTrue(action['suspected'] and not action['confirmed'])
        self.assertEqual(stray, [90.0, 90.1, 90.2, 90.3])
        summary = aggregate(rows)
        self.assertEqual((summary['fall_suspected'], summary['fall_confirmed'], summary['false_suspected']), (1, 1, 1))


if __name__ == '__main__':
    unittest.main()
