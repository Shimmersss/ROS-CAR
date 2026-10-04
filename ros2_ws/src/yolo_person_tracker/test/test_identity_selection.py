import unittest
from types import SimpleNamespace
from yolo_person_tracker.identity_selection import IdentitySelection


def detection(track):
    return SimpleNamespace(track_id=track, box=(10, 10, 90, 90), confidence=.9)


class IdentityLockTests(unittest.TestCase):
    def setUp(self):
        self.s = IdentitySelection()
        self.t = 10.
        self.frame([1])
        self.s.lock(now=self.t)

    def frame(self, ids):
        self.t += .1
        self.s.update([detection(i) for i in ids], 100, self.t)
        self.s.record_frame(round(self.t*1e9), 'optical')

    def evidence(self, rows=None, valid=True, stamp=None, frame='optical'):
        self.s.observe(round(self.t*1e9) if stamp is None else stamp, frame,
                       rows if rows is not None else [('0:1', 'alice', True, True)],
                       round(self.t*1e9), valid, now=self.t)

    def confirm(self, track=1, person='alice', epoch=0):
        for _ in range(3):
            self.frame([track])
            self.evidence([(f'{epoch}:{track}', person, True, True)])
        self.assertIsNotNone(self.s.selected(self.t))

    def test_initial_and_changed_id_confirmation(self):
        self.assertIsNone(self.s.selected(self.t))
        self.confirm()
        self.frame([2])
        self.assertIsNone(self.s.selected(self.t))
        self.assertEqual(self.s.target_id, (0, 1))
        self.confirm(2)
        self.assertEqual(self.s.target_id, (0, 2))

    def test_wrong_person_and_same_id_switch(self):
        self.confirm()
        self.frame([1]); self.evidence([('0:1', 'bob', True, True)])
        self.assertIsNone(self.s.selected(self.t))
        self.confirm()
        for _ in range(5):
            self.frame([2]); self.evidence([('0:2', 'bob', True, True)])
        self.assertEqual(self.s.target_id, (0, 1))
        self.assertIsNone(self.s.selected(self.t))

    def test_ambiguity_and_low_quality_reset(self):
        self.confirm()
        self.frame([1, 2])
        self.evidence([('0:1', 'alice', True, True), ('0:2', 'alice', True, True)])
        self.assertIsNone(self.s.selected(self.t))
        self.frame([1]); self.evidence([('0:1', '', False, True)])
        self.assertEqual(self.s.count, 0)
        self.confirm()

    def test_release_and_explicit_relock(self):
        self.confirm(); self.s.release()
        self.frame([2]); self.evidence([('0:2', 'alice', True, True)])
        self.assertIsNone(self.s.target_id)
        self.s.lock(now=self.t)
        self.evidence([('0:2', 'bob', True, True)])  # pre-lock frame
        self.assertEqual(self.s.count, 0)
        self.confirm(2, 'bob')
        self.assertEqual(self.s.person_id, 'bob')

    def test_epoch_change_requires_new_evidence(self):
        self.confirm(); self.s.reset_stream()
        self.frame([1]); self.assertIsNone(self.s.selected(self.t))
        self.assertFalse(self.s.auto_lock_single(now=self.t))
        self.confirm(3, epoch=1)
        self.assertEqual(self.s.target_id, (1, 3))

    def test_replays_and_unknown_headers(self):
        self.frame([1]); self.evidence()
        for _ in range(5): self.evidence()
        self.assertEqual(self.s.count, 1)
        self.frame([1]); self.evidence(frame='wrong')
        self.assertEqual(self.s.count, 0)
        self.frame([1]); self.evidence(stamp=round(self.t*1e9)-1)
        self.assertEqual(self.s.count, 0)
        self.confirm()

    def test_stale_invalid_and_absent(self):
        self.confirm()
        self.assertIsNone(self.s.selected(self.t+.6))
        self.frame([1]); self.evidence(valid=False)
        self.assertEqual(self.s.count, 0)
        self.confirm(); self.frame([])
        self.assertIsNone(self.s.selected(self.t))

    def test_delayed_evidence_does_not_refresh_ttl(self):
        self.confirm()
        self.frame([1])
        self.t += .45
        self.s.observe(self.s.latest_stamp, 'optical', [('0:1', 'alice', True, True)],
                       round(self.t*1e9), now=self.t)
        self.assertIsNone(self.s.selected(self.t+.1))

    def test_crossing_and_duplicate_rows(self):
        self.confirm(); self.frame([1, 2])
        rows=[('0:1', 'bob', True, True), ('0:2', 'alice', True, True)]
        self.evidence(rows)
        self.assertIsNone(self.s.selected(self.t))
        for _ in range(2): self.frame([1, 2]); self.evidence(rows)
        self.assertEqual(self.s.selected(self.t).track_id, 2)
        self.frame([1, 2]); self.evidence(rows+rows)
        self.assertIsNone(self.s.selected(self.t))

    def test_bounded_history_and_settings(self):
        for _ in range(100): self.frame([1])
        self.assertEqual(len(self.s.frames), 64)
        with self.assertRaises(ValueError): IdentitySelection(confirm_frames=1)
        with self.assertRaises(ValueError): IdentitySelection(max_age_s=float('nan'))


if __name__ == '__main__': unittest.main()
