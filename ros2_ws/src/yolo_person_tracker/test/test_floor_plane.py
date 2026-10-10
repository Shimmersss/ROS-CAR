import math
import unittest
import numpy as np
from yolo_person_tracker.floor_plane import (FloorConfig, FloorFit, FloorTracker, angle_deg,
                                             ankle_contact, camera_angles, fit_floor)

K = (452.4, 452.4, 325.7, 245.0)


def up_vector(pitch_deg=0., roll_deg=0.):
    """World up in the optical frame for a camera pitched up and rolled (x right, y down, z forward)."""
    t, r = math.radians(pitch_deg), math.radians(roll_deg)
    return np.array([math.sin(r)*math.cos(t), -math.cos(r)*math.cos(t), math.sin(t)])


def scene(height=.25, pitch=0., roll=0., wall_m=None, noise=0., holes=0., table=None, seed=1):
    """Registered depth (metres) of a floor, optional wall facing the camera and a raised table patch."""
    fx, fy, cx, cy = K
    v, u = np.mgrid[0:480, 0:640]
    rays = np.stack(((u-cx)/fx, (v-cy)/fy, np.ones_like(u, float)), axis=-1)
    up = up_vector(pitch, roll)
    depth = np.full((480, 640), np.inf)
    down = rays@up
    with np.errstate(divide='ignore', invalid='ignore'):
        floor = np.where(down < 0, -height/down, np.inf)
    depth = np.minimum(depth, floor)
    if wall_m is not None:
        forward = np.array([0., 0., 1.])-up[2]*up
        forward /= np.linalg.norm(forward)
        with np.errstate(divide='ignore', invalid='ignore'):
            wall = np.where(rays@forward > 0, wall_m/(rays@forward), np.inf)
        depth = np.minimum(depth, wall)
    if table is not None:
        (x1, y1, x2, y2), top_m = table
        with np.errstate(divide='ignore', invalid='ignore'):
            raised = np.where(down < 0, -(height-top_m)/down, np.inf)
        region = np.zeros_like(depth, bool); region[y1:y2, x1:x2] = True
        depth = np.where(region, np.minimum(depth, raised), depth)
    rng = np.random.default_rng(seed)
    depth = depth+rng.normal(0, noise, depth.shape)
    depth[~np.isfinite(depth) | (depth > 8)] = 0.
    if holes:
        depth[rng.random(depth.shape) < holes] = 0.
    return depth.astype(np.float32), up


class FloorPlaneTests(unittest.TestCase):
    def check(self, fit, up, height, angle_tol=.5, height_tol=.01):
        self.assertTrue(fit.valid, fit.reason)
        self.assertLess(angle_deg(fit.up, up), angle_tol)
        self.assertLess(abs(fit.height_m-height), height_tol)

    def test_level_camera(self):
        depth, up = scene()
        self.check(fit_floor(depth, K), up, .25)

    def test_gimbal_pitch_roll_wall_noise_holes(self):
        depth, up = scene(.3, pitch=15., roll=2., wall_m=2.5, noise=.004, holes=.1)
        fit = fit_floor(depth, K)
        self.check(fit, up, .3, height_tol=.015)
        pitch, roll = camera_angles(fit.up)
        self.assertAlmostEqual(pitch, 15., delta=.6)
        self.assertAlmostEqual(roll, 2., delta=.6)
        self.assertLess(fit.rms_m, .01)

    def test_small_floor_share_across_noise_seeds(self):
        # Gimbal looking up: the floor is a small share of the ROI behind a wall.
        for seed in range(1, 11):
            depth, up = scene(.3, pitch=15., roll=2., wall_m=2.5, noise=.004, holes=.1, seed=seed)
            self.check(fit_floor(depth, K), up, .3, angle_tol=.3, height_tol=.01)

    def test_narrow_floor_strip_is_rejected_not_biased(self):
        for seed in range(1, 6):
            depth, _ = scene(.3, pitch=15., roll=2., wall_m=1.8, noise=.008, holes=.2, seed=seed)
            self.assertFalse(fit_floor(depth, K).valid)

    def test_person_box_excludes_raised_surface(self):
        box = (200, 260, 460, 480)
        depth, up = scene(table=(box, .1))
        fit = fit_floor(depth, K, boxes=[box])
        self.check(fit, up, .25)
        self.assertEqual(fit_floor(depth, K, boxes=[(0, 0, 640, 480)]).valid, False)

    def test_wall_only_and_bad_inputs_are_rejected(self):
        depth = np.full((480, 640), 2., np.float32)  # fronto-parallel wall, no floor
        fit = fit_floor(depth, K)
        self.assertFalse(fit.valid)
        self.assertIn('expected up', fit.reason)
        self.assertFalse(fit_floor(np.zeros((480, 640)), K).valid)
        self.assertFalse(fit_floor(scene()[0], (0., 452., 320., 240.)).valid)
        self.assertFalse(fit_floor(scene()[0], K, expected_up=(0., 0., 0.)).valid)
        with self.assertRaises(ValueError): FloorConfig(stride=0)
        with self.assertRaises(ValueError): FloorConfig(max_tilt_deg=95.)
        with self.assertRaises(ValueError): FloorConfig(min_depth_m=5.)

    def test_unstructured_clutter_is_rejected(self):
        depth = np.random.default_rng(3).uniform(.3, 4., (480, 640)).astype(np.float32)
        self.assertFalse(fit_floor(depth, K).valid)

    def test_tracker_requires_consecutive_consistent_fits(self):
        tracker = FloorTracker()
        good = fit_floor(scene()[0], K)
        moved = fit_floor(scene(pitch=8.)[0], K)
        self.assertEqual([tracker.update(t, good)[0] for t in (1., 1.1, 1.2)], [False, False, True])
        self.assertFalse(tracker.update(1.3, moved)[0])                # camera rotated: restart
        self.assertFalse(tracker.update(1.4, FloorFit(False, reason='x'))[0])
        for t in (1.5, 1.6): tracker.update(t, good)
        self.assertTrue(tracker.update(1.7, good)[0])
        self.assertFalse(tracker.update(2.5, good)[0])                 # gap restarts
        self.assertFalse(tracker.update(2.4, good)[0])                 # rewind restarts

    def test_ankle_contact_and_angles(self):
        up = up_vector(10., -3.)
        height, ankle = .3, .08
        foot = np.array([.4, 0., 2.])
        foot = foot-(foot@up+height-ankle)*up                         # put it on the ankle plane
        u = K[0]*foot[0]/foot[2]+K[2]; v = K[1]*foot[1]/foot[2]+K[3]
        np.testing.assert_allclose(ankle_contact(u, v, K, up, height, ankle), foot, atol=1e-9)
        self.assertIsNone(ankle_contact(K[2], 10., K, up_vector(), height, ankle))  # above horizon
        self.assertEqual(tuple(round(a, 6) for a in camera_angles((0., -1., 0.))), (0., 0.))
        pitch, roll = camera_angles(up_vector(12., 0.))
        self.assertAlmostEqual(pitch, 12.); self.assertAlmostEqual(roll, 0.)


if __name__ == '__main__':
    unittest.main()
