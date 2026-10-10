"""Recording reader/summary and labelling logic on synthetic files (no camera)."""
import csv
import json
from pathlib import Path
import sys
import tempfile
import unittest
import zlib

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from gemini_recording import load_depth_m, load_depth_raw, pairing_key, summarize  # noqa: E402
from label_recording import FALL_EVENTS, LabelSession  # noqa: E402
from record_gemini_mac import PLAN, camera_setup, notes_text, resolve_segment  # noqa: E402
from gemini_color import CROP, FULL, OUT, apply_charuco, fit_mapping, mode_intrinsics, rb_ratio, shrink  # noqa: E402
from record_gemini_mac import patch_device_json  # noqa: E402
from evaluate_falls import aggregate, falls_and_actions, score  # noqa: E402
from fall_candidates import candidates, intervals  # noqa: E402


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

    def test_pairing_uses_monotonic_time_when_both_streams_have_it(self):
        colors = [dict(host_wall_ns='1', host_monotonic_ns='2')]
        self.assertEqual(pairing_key(colors, [dict(host_wall_ns='1')]), 'host_wall_ns')       # old depth CSV
        self.assertEqual(pairing_key(colors, [dict(host_wall_ns='1', host_monotonic_ns='3')]), 'host_monotonic_ns')
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(summarize(make_recording(tmp))['pairing_clock'], 'host_wall_ns')

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

    def test_candidate_review_jump_delete_confirm(self):
        s = self.session()
        s.marker = 0
        for event, frame in zip(FALL_EVENTS, (40, 50, 60, 120, 150)):
            s.labels.append(dict(segment='A1', event=event, person='A', position='', direction='', notes='auto', frame=frame))
        s.labels.append(dict(segment='A1', event='fall_onset', person='A', position='', direction='', notes='auto', frame=250))
        self.assertTrue(any('not yet confirmed' in p for p in s.problems()))
        self.keys(s, 'l')
        self.assertEqual(s.frame, 40)
        self.keys(s, 'lll')
        self.assertEqual(s.frame, 120)
        self.keys(s, 'rpk')                  # side_right, person B, position 3m
        self.keys(s, 'y')                    # confirms the 40..150 fall, not the lone onset at 250
        group = [label for label in s.labels if label['frame'] <= 150]
        self.assertEqual({(l['notes'], l['direction'], l['person'], l['position']) for l in group},
                         {('', 'side_right', 'B', '3m')})
        self.keys(s, 'c'*12+'x')             # frame 240: deletes the onset 10 frames away
        self.assertEqual(len(s.labels), 5)
        self.assertEqual(s.problems(), [])
        s.frame = 200
        self.keys(s, 'x')                    # nothing within 25 frames
        self.assertEqual(len(s.labels), 5)

    def test_unreviewed_candidates_are_not_ground_truth(self):
        rows = [dict(t=float(i), event=e, notes='auto' if auto else '', direction='', person='A', position='')
                for auto, base in ((True, 0), (False, 20)) for i, e in enumerate(FALL_EVENTS, base)]
        falls, _ = falls_and_actions(rows)
        self.assertEqual([f['fall_onset'] for f in falls], [20.])

    def test_navigation_is_clamped(self):
        s = self.session(5)
        self.keys(s, 'zzz')
        self.assertEqual(s.frame, 0)
        self.keys(s, 'cc')
        self.assertEqual(s.frame, 4)
        self.assertIsNone(s.key('x'))
        self.assertEqual(s.key('q'), 'quit')


class CandidateTests(unittest.TestCase):
    @staticmethod
    def samples(low, upright, end=60.):
        """10 Hz (t, low, upright) from lists of (start, end) spans."""
        out = []
        for i in range(int(end*10)):
            t = i/10
            out.append((t, any(a <= t <= b for a, b in low), any(a <= t <= b for a, b in upright)))
        return out

    def test_low_stretches_before_standing_again_are_one_fall(self):
        # Sit down, lie back, sit up, lie again, then stand: one fall; a later fall is separate.
        s = self.samples(low=[(5, 8), (10, 12), (30, 33)], upright=[(0, 4), (15, 28), (36, 60)])
        found = candidates(s, intervals(s, 1., .6))
        self.assertEqual([(c['impact'], c['lying_start'], c['getup_start']) for c in found], [(5., 5.5, 12.), (30., 30.5, 33.)])
        self.assertEqual([c['stand_stable'] for c in found], [15., 36.])
        for c in found:
            self.assertEqual([c[e] for e in FALL_EVENTS], sorted(c[e] for e in FALL_EVENTS))

    def test_distant_low_stretch_is_not_merged_without_standing(self):
        # Never seen standing in between (e.g. head out of view): far-apart stretches stay separate.
        s = self.samples(low=[(5, 8), (40, 43)], upright=[])
        found = candidates(s, intervals(s, 1., .6))
        self.assertEqual(len(found), 2)
        self.assertLess(found[0]['stand_stable'], found[1]['fall_onset'])


class SegmentNameTests(unittest.TestCase):
    def test_plan_codes_free_names_and_parts(self):
        with tempfile.TemporaryDirectory() as tmp:
            out, seconds = resolve_segment('a1A', tmp, '20261010')
            self.assertEqual((out.name, seconds), ('gemini-20261010-A1a-fall-side-1.9m', 150))
            out.mkdir(); (out/'x').touch()
            self.assertEqual(resolve_segment('A1a', tmp, '20261010')[0].name, 'gemini-20261010-A1a-fall-side-1.9m-part2')
            self.assertEqual(resolve_segment('my-test', tmp, '20261010'), (Path(tmp)/'gemini-20261010-my-test', 120))
            for bad in ('', '../x', '.hidden'):
                with self.assertRaises(ValueError):
                    resolve_segment(bad, tmp, '20261010')
            self.assertTrue(all(0 < s <= 170 for _, s in PLAN.values()))

    def test_camera_setup_is_saved_once_and_filled_into_notes(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertIsNone(camera_setup(tmp, '20261011'))
            self.assertIn('离地高度（cm）：\n', notes_text('x', 'now', None))
            camera_setup(tmp, '20261011', (55, 8.5, -0.5))
            setup = camera_setup(tmp, '20261011')
            self.assertEqual((setup['height_cm'], setup['pitch_deg'], setup['roll_deg']), (55., 8.5, -.5))
            notes = notes_text('x', 'now', setup)
            self.assertIn('离地高度（cm）：55\n', notes)
            self.assertIn('抬头为正）：8.5\n', notes)
            self.assertIsNone(camera_setup(tmp, '20261012'))           # a new day needs a new measurement
            for bad in ((0.5, 8, 0), (55, 80, 0), (55, 8, 30)):            # metres instead of cm, etc.
                with self.assertRaises(ValueError):
                    camera_setup(tmp, '20261011', bad)


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
        self.assertEqual(stray, [90.0])
        summary = aggregate(rows)
        self.assertEqual((summary['fall_suspected'], summary['fall_confirmed'], summary['false_suspected']), (1, 1, 1))

    def test_alarm_that_expired_in_a_gap_counts_again(self):
        falls, actions = falls_and_actions(self.labels())
        timeline = [dict(t=t/10, phase=0, basis='2d', id='a') for t in range(0, 300)]
        timeline += [dict(t=t/10, phase=2, basis='2d', id='a') for t in range(300, 320)]   # raised at 30 s
        # No rows at all from 32 s to 40 s, then the same person is back at phase 2.
        timeline += [dict(t=t/10, phase=2, basis='2d', id='a') for t in range(400, 450)]
        rows, _ = score(timeline, falls, actions)
        second = rows[1]
        self.assertTrue(second['confirmed'] and not second['confirmed_carried_in'])

    def test_alarm_left_over_from_an_earlier_event_is_not_a_detection(self):
        falls, actions = falls_and_actions(self.labels())
        timeline = [dict(t=t/10, phase=0, basis='2d', id='a') for t in range(0, 1000)]
        for r in timeline:
            if 30.0 <= r['t'] <= 45.0: r['phase'] = 2      # raised before the 40 s fall, never cleared
        # A second person seen in alternate frames does not reset it.
        timeline += [dict(t=k/10+.05, phase=0, basis='2d', id='b') for k in range(380, 420)]
        timeline.sort(key=lambda r: r['t'])
        rows, stray = score(timeline, falls, actions)
        second = rows[1]
        self.assertFalse(second['suspected'] or second['confirmed'])
        self.assertTrue(second['suspected_carried_in'] and second['confirmed_carried_in'])
        self.assertEqual(stray, [30.0])


class ColorModeTests(unittest.TestCase):
    FACTORY = dict(width=640, height=480, fx=452.447, fy=452.447, cx=325.75, cy=245.025)

    def test_shrink_and_cast_ratio(self):
        frame = np.zeros((FULL[1], FULL[0], 3), np.uint8)
        frame[:, :, 0], frame[:, :, 2] = 200, 40
        frame[CROP[1]:CROP[1]+4, CROP[0]:CROP[0]+4] = 255          # first output pixel only
        small = shrink(frame)
        self.assertEqual(small.shape, (OUT[1], OUT[0], 3))
        self.assertTrue((small[0, 0] == 255).all() and (small[0, 1] != 255).any())
        self.assertAlmostEqual(rb_ratio(small), .2, places=2)

    def test_intrinsics_follow_the_mapping(self):
        # A point seen at native pixel (u, v) must land at the same place via the shrunk crop.
        mapping = dict(scale=.2501, tx=4.1, ty=.4)
        k = mode_intrinsics(self.FACTORY, mapping)
        for x, y, z in ((.3, -.2, 2.), (-.5, .4, 3.)):
            u = self.FACTORY['fx']*x/z+self.FACTORY['cx']
            v = self.FACTORY['fy']*y/z+self.FACTORY['cy']
            full = ((u-mapping['tx'])/mapping['scale'], (v-mapping['ty'])/mapping['scale'])
            out = ((full[0]-CROP[0])/4, (full[1]-CROP[1])/4)
            self.assertAlmostEqual(k['fx']*x/z+k['cx'], out[0], places=6)
            self.assertAlmostEqual(k['fy']*y/z+k['cy'], out[1], places=6)
        self.assertEqual((k['width'], k['height']), OUT)

    def test_fit_mapping_recovers_a_synthetic_offset(self):
        rng = np.random.default_rng(1)
        full = np.full((FULL[1], FULL[0]), 128, np.uint8)
        for _ in range(400):                                   # textured scene of random shapes
            x, y, r = int(rng.integers(0, FULL[0])), int(rng.integers(0, FULL[1])), int(rng.integers(15, 90))
            cv2.circle(full, (x, y), r, int(rng.integers(0, 256)), -1)
            cv2.rectangle(full, (x, y), (x+r, y+2*r//3), int(rng.integers(0, 256)), -1)
        full = cv2.GaussianBlur(full, (0, 0), 2)
        # native = 0.25 * full + (4, 0.6): shift by 4x the offset, then area-shrink 4x
        shifted = cv2.warpAffine(full, np.float32([[1, 0, 16.], [0, 1, 2.4]]), (4*OUT[0], 4*OUT[1]))
        native = cv2.resize(shifted, OUT, interpolation=cv2.INTER_AREA)
        fit = fit_mapping(native, full)
        self.assertAlmostEqual(fit['scale'], .25, delta=.0005)
        self.assertAlmostEqual(fit['tx'], 4., delta=.3)
        self.assertAlmostEqual(fit['ty'], .6, delta=.3)

    def test_charuco_principal_point_updates_the_offset(self):
        mapping = dict(scale=.25003, tx=4.17, ty=.47, source='measured')
        report = dict(views=15, coverage=dict(cells_4x4=16), delta=dict(fx_pct=.2),
                      tilts=dict(left_right_strong=4, up_down_strong=3, strong_deg=20.),
                      principal_point_only=dict(cx=319., cy=240., std_cx=.8, std_cy=.9))
        updated = apply_charuco(report, self.FACTORY, mapping)
        k = mode_intrinsics(self.FACTORY, updated)
        self.assertAlmostEqual(k['cx'], 319., places=6)
        self.assertAlmostEqual(k['cy'], 240., places=6)
        self.assertAlmostEqual(k['fx'], mode_intrinsics(self.FACTORY, mapping)['fx'], places=9)
        self.assertEqual((updated['source'], updated['previous']['tx']), ('charuco', 4.17))
        for bad in (dict(views=8), dict(coverage=dict(cells_4x4=9)), dict(delta=dict(fx_pct=3.)),
                    dict(tilts=dict(left_right_strong=5, up_down_strong=0)),
                    dict(principal_point_only=dict(cx=319., cy=240., std_cx=2.5, std_cy=.9))):
            with self.assertRaises(RuntimeError):
                apply_charuco(dict(report, **bad), self.FACTORY, mapping)

    def test_device_json_keeps_factory_intrinsics(self):
        with tempfile.TemporaryDirectory() as tmp:
            Path(tmp, 'device.json').write_text(json.dumps(dict(color_intrinsic=self.FACTORY)))
            mapping = dict(scale=.25, tx=4., ty=.6, source='measured')
            patch_device_json(Path(tmp), 'full', mapping)
            device = json.loads(Path(tmp, 'device.json').read_text())
            self.assertEqual(device['factory_color_intrinsic'], self.FACTORY)
            self.assertAlmostEqual(device['color_intrinsic']['cx'], 325.75-4.-4., places=6)
            self.assertEqual(device['color_capture']['mode'], 'full')
            patch_device_json(Path(tmp), 'full', mapping)      # idempotent: never re-derive from derived
            again = json.loads(Path(tmp, 'device.json').read_text())
            self.assertEqual(again['color_intrinsic'], device['color_intrinsic'])


if __name__ == '__main__':
    unittest.main()
