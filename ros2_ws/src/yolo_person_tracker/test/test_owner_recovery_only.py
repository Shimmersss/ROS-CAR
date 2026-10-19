"""Appearance recovery after real loss; no model accuracy or motion claims."""
import unittest
from types import SimpleNamespace
from yolo_person_tracker.owner_selection import OwnerSelection


def person(track, x=10):
    return SimpleNamespace(track_id=track, box=(x, 10, x+40, 90), confidence=.9)


class RecoveryOnlyTests(unittest.TestCase):
    def setUp(self):
        self.s = OwnerSelection(reacquire_only_after_loss=True, reacquire_frames=3, reacquire_min_s=.4)
        self.t = 10.
        self.frame([person(1)])
        self.assertTrue(self.s.lock(now=self.t)[0])
        for _ in range(3):
            self.frame([person(1)])
            self.identities([('0:1', 'alice')])
        self.assertEqual(self.s.owner_id, 'alice')

    def frame(self, people, dt=.2):
        self.t += dt
        self.s.update(people, 1000, self.t)
        self.stamp = round(self.t*1e9)
        self.s.record_frame(self.stamp, 'camera')

    def identities(self, rows, **kwargs):
        self.s.observe(self.stamp, 'camera', [(t, p, True, True) for t,p in rows],
                       self.stamp, **kwargs)

    def lost(self):
        self.frame([], dt=1.1)
        self.s.check_absence(self.t, bridging=False)
        self.assertTrue(self.s.searching)
        self.assertIsNone(self.s.target_id)

    def test_appearance_cannot_interrupt_or_replace_continuous_target(self):
        for rows in ([], [('0:1','bob')], [('0:2','alice')])*3:
            self.frame([person(1), person(2, 500)])
            self.identities(rows)
            self.assertEqual(self.s.target_id, (0,1))
            self.assertEqual(self.s.owner_id, 'alice')
        self.assertEqual(self.s.veto, 0)

    def test_lost_owner_returns_at_different_position_after_three_fresh_frames(self):
        self.lost()
        for i in range(3):
            self.frame([person(7, 800)])
            self.identities([('0:7','alice')])
            self.assertEqual(self.s.target_id, (0,7) if i==2 else None)
        self.assertFalse(self.s.searching)

    def test_wrong_person_or_ambiguous_identity_never_recovers(self):
        self.lost()
        for rows in ([('0:7','bob')], [('0:7','alice'),('0:8','alice')], [('0:7','alice'),('0:7','alice')]):
            for _ in range(4):
                self.frame([person(7, 500), person(8, 800)])
                self.identities(rows)
                self.assertIsNone(self.s.target_id)

    def test_missing_or_invalid_evidence_breaks_consecutive_confirmation(self):
        for invalid in (False, True):
            self.s.lose('test')
            for _ in range(2):
                self.frame([person(7, 800)]);self.identities([('0:7','alice')])
            self.frame([person(7, 800)]);self.identities([] if not invalid else [('0:7','alice')],valid=not invalid)
            self.frame([person(7, 800)]);self.identities([('0:7','alice')])
            self.assertIsNone(self.s.target_id)
            self.assertEqual(self.s.found[1],1)

    def test_duplicate_stale_unrecorded_wrong_frame_and_long_gap_do_not_confirm(self):
        self.lost()
        self.frame([person(7, 800)]);self.identities([('0:7','alice')])
        for _ in range(5):self.identities([('0:7','alice')])
        self.assertEqual(self.s.found[1],1)
        self.frame([person(7, 800)],dt=.6);self.identities([('0:7','alice')])
        self.assertEqual(self.s.found[1],1)
        for stamp,frame,now in ((self.stamp+1,'camera',self.stamp+1),
                                (self.stamp+2,'wrong',self.stamp+2),
                                (self.stamp+3,'camera',self.stamp+int(1e9))):
            self.s.observe(stamp,frame,[('0:7','alice',True,True)],now)
            self.assertEqual(self.s.found[1],0)
            self.assertIsNone(self.s.target_id)

    def test_fast_burst_does_not_satisfy_minimum_duration(self):
        self.lost()
        for _ in range(3):
            self.frame([person(7,800)],dt=.05);self.identities([('0:7','alice')])
        self.assertIsNone(self.s.target_id)
        self.frame([person(7,800)],dt=.31);self.identities([('0:7','alice')])
        self.assertEqual(self.s.target_id,(0,7))

    def test_historical_or_current_overlap_does_not_confirm(self):
        self.lost()
        for _ in range(4):
            self.frame([person(7,800),person(8,810)])
            self.identities([('0:7','alice')])
        self.assertIsNone(self.s.target_id)
        # A clean old observation is insufficient if the current frame overlaps.
        self.frame([person(7,800)]);old=self.stamp
        self.frame([person(7,800),person(8,810)])
        self.s.observe(old,'camera',[('0:7','alice',True,True)],self.stamp)
        self.assertIsNone(self.s.target_id)
        # A historically occluded observation stays unusable after people separate.
        old=self.stamp
        self.frame([person(7,800),person(8,500)])
        self.s.observe(old,'camera',[('0:7','alice',True,True)],self.stamp)
        self.assertEqual(self.s.found[1],0)

    def test_release_forgets_owner_and_requires_manual_claim(self):
        self.s.release()
        self.assertEqual(self.s.owner_id,'')
        self.frame([person(7,800)])
        self.identities([('0:7','alice')])
        self.assertIsNone(self.s.target_id)
        self.assertTrue(self.s.lock(now=self.t,track_id=7)[0])
        self.assertEqual(self.s.target_id,(0,7))

    def test_initial_binding_requires_clean_consecutive_evidence(self):
        self.s.release();self.frame([person(1),person(2,20)])
        self.s.lock(now=self.t,track_id=1)
        for _ in range(4):
            self.frame([person(1),person(2,20)]);self.identities([('0:1','bob')])
        self.assertEqual(self.s.owner_id,'')
        for _ in range(2):
            self.frame([person(1)]);self.identities([('0:1','alice')])
        self.frame([person(1)]);self.identities([])
        self.frame([person(1)]);self.identities([('0:1','alice')])
        self.assertEqual(self.s.owner_id,'')

    def test_parameters_reject_invalid_duration_and_policy(self):
        for value in (-1.,float('nan'),float('inf'),True):
            with self.assertRaises(ValueError):OwnerSelection(reacquire_min_s=value)
        with self.assertRaises(ValueError):OwnerSelection(reacquire_only_after_loss=1)


if __name__=='__main__':unittest.main()
