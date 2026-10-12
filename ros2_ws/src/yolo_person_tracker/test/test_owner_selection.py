import unittest
from types import SimpleNamespace
from yolo_person_tracker.owner_selection import OwnerSelection


def det(track, x=10):
    return SimpleNamespace(track_id=track, box=(x, 10, x+40, 90), confidence=.9)


class OwnerSelectionTests(unittest.TestCase):
    def setUp(self):
        self.s = OwnerSelection(); self.t = 10.
        self.frame([det(1, 30)]); self.assertTrue(self.s.lock(now=self.t)[0])

    def frame(self, dets):
        self.t += .1
        self.s.update(dets, 100, self.t)
        self.s.record_frame(round(self.t*1e9), 'optical')

    def reid(self, rows):
        self.s.observe(round(self.t*1e9), 'optical', [(t, p, bool(p), True) for t, p in rows], round(self.t*1e9))

    def bind(self):
        for _ in range(3):
            self.frame([det(1, 30)]); self.reid([('0:1', 'alice')])
        self.assertEqual(self.s.owner_id, 'alice')

    def test_follows_immediately_without_reid(self):
        self.frame([det(1, 30)])
        self.assertEqual(self.s.selected(now=self.t).track_id, 1)  # no ReID authorization needed
        self.reid([('0:1', '')])
        self.assertIsNotNone(self.s.selected(now=self.t))

    def test_veto_needs_repeated_evidence_then_searches_and_reacquires(self):
        self.bind()
        self.frame([det(1, 30), det(2, 60)]); self.reid([('0:1', 'bob')])
        self.assertIsNotNone(self.s.target_id)  # single contrary result is not enough
        self.frame([det(1, 30), det(2, 60)]); self.reid([('0:1', 'bob')])
        self.assertIsNone(self.s.target_id); self.assertTrue(self.s.searching)
        self.assertFalse(self.s.auto_lock_single(now=self.t))
        self.frame([det(1, 30), det(2, 60)]); self.reid([('0:2', 'alice')])
        self.assertIsNone(self.s.target_id)
        self.frame([det(1, 30), det(2, 60)]); self.reid([('0:2', 'alice')])
        self.assertEqual(self.s.target_id, (0, 2)); self.assertFalse(self.s.searching)

    def test_owner_seen_on_another_track_is_a_veto(self):
        self.bind()
        for _ in range(2):
            self.frame([det(1, 30), det(2, 60)]); self.reid([('0:2', 'alice')])
        self.assertIsNone(self.s.target_id)
        self.frame([det(1, 30), det(2, 60)]); self.reid([('0:2', 'alice')])
        self.assertEqual(self.s.target_id, (0, 2))  # re-acquired after the second confirmation

    def test_absence_without_bridge_loses_and_unbound_owner_needs_relock(self):
        for _ in range(12):
            self.frame([det(2, 60)]); self.s.check_absence(self.t, bridging=False)
        self.assertIsNone(self.s.target_id); self.assertFalse(self.s.searching)
        self.assertIn('relock', self.s.detail)

    def test_bridging_keeps_target_alive(self):
        for _ in range(20):
            self.frame([det(2, 60)]); self.s.note_bridging(self.t); self.s.check_absence(self.t, bridging=True)
        self.assertEqual(self.s.target_id, (0, 1))

    def test_unrecorded_or_stale_frames_are_ignored(self):
        self.bind()
        self.frame([det(1, 30)])
        self.s.observe(round(self.t*1e9)+5, 'optical', [('0:1', 'bob', True, True)], round(self.t*1e9)+5)
        self.s.observe(round(self.t*1e9), 'optical', [('0:1', 'bob', True, True)], round((self.t+1)*1e9))
        self.assertEqual(self.s.veto, 0)

    def test_release_forgets_owner(self):
        self.bind(); self.s.release()
        self.assertEqual(self.s.owner_id, ''); self.assertFalse(self.s.searching)


if __name__ == '__main__':
    unittest.main()
