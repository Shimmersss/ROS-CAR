"""Per-frame gravity (gimbal/floor) for the fall rules; static mode is covered by test_pose3d."""
import unittest
import numpy as np
from test_pose import skeleton
from test_pose3d import BOX, spatial
from yolo_person_tracker.pose import PoseConfig, UNKNOWN, STANDING, LYING, FALLEN
from yolo_person_tracker.pose3d import EnhancedPostureTracker, Pose3DConfig

LEVEL = (np.array([0., -1., 0.]), 1.)
LYING_BOX = (20, 130, 240, 220)


def tilt(pitch_deg):
    """Rotation taking level-camera coordinates to a camera pitched up by pitch_deg."""
    a = np.radians(-pitch_deg)
    return np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])


def gravity(pitch_deg=0., roll_deg=0., height=1.):
    t, r = np.radians(pitch_deg), np.radians(roll_deg)
    return np.array([np.sin(r)*np.cos(t), -np.cos(r)*np.cos(t), np.sin(t)]), height


class DynamicGravityTests(unittest.TestCase):
    def tracker(self, three_d=True, upright=True):
        return EnhancedPostureTracker(PoseConfig(upright_confirmed=upright), Pose3DConfig(
            gravity_confirmed=three_d, ground_confirmed=three_d, up_source='floor'))

    def stable2d(self, t, g=LEVEL):
        for s in (1., 1.2, 1.4): t.update('x', s, BOX, skeleton(), None, g)

    def test_level_dynamic_matches_static_3d_fall(self):
        t = self.tracker()
        for s in (1., 1.2, 1.4): t.update('x', s, BOX, skeleton(), spatial(), LEVEL)
        for s in np.arange(1.6, 2.9, .2): result = t.update('x', float(s), BOX, skeleton(), spatial(.35, True), LEVEL)
        self.assertEqual(result[:2], (FALLEN, 2))
        self.assertIn('basis=3d', result[2])

    def test_metric_fall_while_camera_tilts(self):
        # Joints and up rotate together: gravity-frame geometry is unchanged by the gimbal.
        t = self.tracker()
        for i, s in enumerate((1., 1.2, 1.4)):
            m = tilt(5*i); g = (m@LEVEL[0], 1.)
            t.update('x', s, BOX, skeleton(), spatial()@m.T, g)
        for i, s in enumerate(np.arange(1.6, 2.9, .2)):
            m = tilt(10+2*i); g = (m@LEVEL[0], 1.)
            result = t.update('x', float(s), BOX, skeleton(), spatial(.35, True)@m.T, g)
        self.assertEqual(result[:2], (FALLEN, 2))

    def test_missing_gravity_falls_back_to_2d_without_fall(self):
        t = self.tracker()
        self.stable2d(t, None)
        result = t.update('x', 1.6, LYING_BOX, skeleton(True, 180), spatial(.35, True), None)
        self.assertEqual(result[1], 0)
        self.assertIn('basis=2d', result[2]); self.assertIn('gravity=unavailable', result[2])

    def test_camera_rotation_resets_pixel_drop_reference(self):
        # Same image sequence: a static camera raises suspicion, a rotated one must not.
        still = self.tracker(three_d=False); self.stable2d(still)
        self.assertEqual(still.update('x', 1.6, LYING_BOX, skeleton(True, 180), None, LEVEL)[:2], (LYING, 1))
        moved = self.tracker(three_d=False); self.stable2d(moved)
        result = moved.update('x', 1.6, LYING_BOX, skeleton(True, 180), None, gravity(6.))
        self.assertEqual(result[:2], (LYING, 0))
        self.assertIn('camera rotated', result[2])
        small = self.tracker(three_d=False); self.stable2d(small)
        self.assertEqual(small.update('x', 1.6, LYING_BOX, skeleton(True, 180), None, gravity(1.))[:2], (LYING, 1))

    def test_roll_beyond_limit_makes_2d_unknown(self):
        t = self.tracker(three_d=False)
        result = t.update('x', 1., BOX, skeleton(), None, gravity(0., 6.))
        self.assertEqual(result[0], UNKNOWN); self.assertIn('roll', result[2])
        self.assertEqual(t.update('x', 1.2, BOX, skeleton(), None, gravity(0., 2.))[0], STANDING)

    def test_2d_confirmation_still_needs_explicit_upright_flag(self):
        t = self.tracker(three_d=False, upright=False); self.stable2d(t)
        for s in (1.6, 1.9, 2.2, 2.5, 2.7):
            result = t.update('x', s, LYING_BOX, skeleton(True, 180), None, LEVEL)
        self.assertEqual(result[:2], (LYING, 1))
        t = self.tracker(three_d=False); self.stable2d(t)
        for s in (1.6, 1.9, 2.2, 2.5, 2.7):
            result = t.update('x', s, LYING_BOX, skeleton(True, 180), None, LEVEL)
        self.assertEqual(result[:2], (FALLEN, 2))

    def test_invalid_gravity_and_config(self):
        t = self.tracker()
        result = t.update('x', 1., BOX, skeleton(), spatial(), (np.array([0., -2., 0.]), 1.))
        self.assertIn('gravity=unavailable', result[2])
        with self.assertRaises(ValueError): Pose3DConfig(up_source='imu')


if __name__ == '__main__':
    unittest.main()
