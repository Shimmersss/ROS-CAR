"""Opt-in fall-rule robustness switches (all default off; defaults are covered by test_pose*)."""
import unittest
from test_pose import skeleton
from yolo_person_tracker.pose import PoseConfig, PostureTracker, UNKNOWN, STANDING, LYING, FALLEN
from yolo_person_tracker.pose3d import EnhancedPostureTracker, Pose3DConfig, below_near
from test_pose3d import spatial
import numpy as np

STAND_BOX = (50, 10, 150, 210)
LIE_BOX = (20, 130, 240, 220)
MISSING = tuple((0., 0., 0.) for _ in range(17))   # no usable shoulders/hips


def lying_wobble(center):
    return skeleton(True, center)


class GapPauseTests(unittest.TestCase):
    def run_fall(self, **kw):
        t = PostureTracker(PoseConfig(upright_confirmed=True, **kw))
        for s in (1., 1.2, 1.4): t.update('x', s, STAND_BOX, skeleton())
        out = [t.update('x', 1.6, LIE_BOX, skeleton(True, 180))]
        out.append(t.update('x', 1.8, LIE_BOX, MISSING))          # occluded hips while lying
        for s in (2.0, 2.3, 2.6, 2.9): out.append(t.update('x', s, LIE_BOX, skeleton(True, 180)))
        return out

    def test_default_clears_pending_on_missing_keypoints(self):
        out = self.run_fall()
        self.assertEqual(out[0][:2], (LYING, 1))
        self.assertEqual(out[1][:2], (UNKNOWN, 0))
        self.assertNotEqual(out[-1][:2], (FALLEN, 2))

    def test_pause_keeps_pending_and_does_not_count_the_gap(self):
        out = self.run_fall(gap_pause=True)
        self.assertEqual(out[1][:2], (UNKNOWN, 1))
        self.assertIn('paused', out[1][2])
        self.assertEqual(out[-1][:2], (FALLEN, 2))
        # 0.2 s gap is not counted: pending 1.6 -> confirm needs evidence until >= 2.8 s
        self.assertNotEqual(out[-2][:2], (FALLEN, 2))

    def test_long_gap_still_clears(self):
        t = PostureTracker(PoseConfig(upright_confirmed=True, gap_pause=True))
        for s in (1., 1.2, 1.4): t.update('x', s, STAND_BOX, skeleton())
        t.update('x', 1.6, LIE_BOX, skeleton(True, 180))
        for s in (1.8, 2.0, 2.2, 2.4): t.update('x', s, LIE_BOX, MISSING)   # > max_gap_s missing
        self.assertEqual(t.update('x', 2.6, LIE_BOX, skeleton(True, 180))[:2], (LYING, 0))


class ConfirmLyingOnlyTests(unittest.TestCase):
    def test_small_rise_while_lying_no_longer_cancels(self):
        for flag, expected in ((False, 0), (True, 2)):
            t = PostureTracker(PoseConfig(upright_confirmed=True, confirm_lying_only=flag))
            for s in (1., 1.2, 1.4): t.update('x', s, STAND_BOX, skeleton())
            t.update('x', 1.6, LIE_BOX, skeleton(True, 180))
            # still horizontal, but the torso centre rises back above the drop threshold
            for s in (1.9, 2.2, 2.5, 2.7): result = t.update('x', s, LIE_BOX, skeleton(True, 140))
            self.assertEqual(result[1], expected)

    def test_getting_up_still_cancels(self):
        t = PostureTracker(PoseConfig(upright_confirmed=True, confirm_lying_only=True))
        for s in (1., 1.2, 1.4): t.update('x', s, STAND_BOX, skeleton())
        t.update('x', 1.6, LIE_BOX, skeleton(True, 180))
        self.assertEqual(t.update('x', 1.9, STAND_BOX, skeleton())[1], 0)


class HandoverTests(unittest.TestCase):
    def tracker(self, handover=1.5):
        return EnhancedPostureTracker(PoseConfig(upright_confirmed=True, handover_s=handover), Pose3DConfig())

    def test_fall_across_an_id_change(self):
        for handover, expected in ((0., 0), (1.5, 2)):
            t = self.tracker(handover)
            for s in (1., 1.2, 1.4):
                t.handover(s, {'0:1': STAND_BOX}); t.update('0:1', s, STAND_BOX, skeleton())
            moved = t.handover(1.8, {'0:7': LIE_BOX})              # ID lost while falling
            self.assertEqual(moved, {'0:7': '0:1'} if handover else {})
            for s in (1.8, 2.1, 2.4, 2.7, 3.0): result = t.update('0:7', s, LIE_BOX, skeleton(True, 180))
            self.assertEqual(result[1], expected)

    def test_no_handover_when_ambiguous_far_or_still_present(self):
        t = self.tracker()
        for s in (1., 1.2):
            t.update('0:1', s, STAND_BOX, skeleton()); t.update('0:2', s, (300, 10, 400, 210), skeleton())
        self.assertEqual(t.handover(1.4, {'0:1': STAND_BOX, '0:7': LIE_BOX}), {})       # old still present
        self.assertEqual(t.handover(1.4, {'0:7': (500, 300, 600, 400)}), {})           # nowhere near
        t2 = self.tracker()
        t2.update('0:1', 1., STAND_BOX, skeleton()); t2.update('0:2', 1., (60, 20, 160, 215), skeleton())
        self.assertEqual(t2.handover(1.3, {'0:7': LIE_BOX}), {})                         # two candidates
        self.assertEqual(self.tracker().handover(1., {'0:7': LIE_BOX}), {})
        t3 = self.tracker(); t3.update('0:1', 1., STAND_BOX, skeleton())
        self.assertEqual(t3.handover(3., {'0:7': LIE_BOX}), {})                         # beyond handover_s

    def test_below_near_geometry(self):
        self.assertTrue(below_near(STAND_BOX, LIE_BOX))
        self.assertFalse(below_near(STAND_BOX, (60, -150, 160, 40)))                     # above
        self.assertFalse(below_near((0, 0, 0, 10), LIE_BOX))                             # degenerate


class KneeFallbackTests(unittest.TestCase):
    def test_upright_torso_without_knees_gives_metric_baseline(self):
        for flag in (False, True):
            t = EnhancedPostureTracker(PoseConfig(upright_confirmed=True),
                                       Pose3DConfig(gravity_confirmed=True, ground_confirmed=True, knee_fallback=flag))
            j = spatial(); j[13] = j[14] = np.nan
            result = t.update('x', 1., STAND_BOX, skeleton(), j)
            self.assertIn('basis=3d' if flag else 'basis=2d', result[2])
            if flag:
                self.assertEqual(result[0], STANDING)
                self.assertIn('knees from 2D', result[2])

    def test_config_validation(self):
        with self.assertRaises(ValueError): PoseConfig(gap_pause=1)
        with self.assertRaises(ValueError): PoseConfig(handover_s=5.)
        with self.assertRaises(ValueError): Pose3DConfig(knee_fallback='yes')


if __name__ == '__main__':
    unittest.main()
