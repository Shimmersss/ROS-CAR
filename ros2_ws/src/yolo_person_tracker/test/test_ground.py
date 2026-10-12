"""Synthetic geometry checks for monocular ground localization; not a field accuracy claim."""
import math
import unittest
from types import SimpleNamespace as NS

import numpy as np

from yolo_person_tracker.ground import (
    GroundConfig, GroundTrackFilter, camera_model_from_info, estimate_ground_point,
    rotation_matrix)

FX = FY = 500.
SIZE = (640, 480)
INTRINSICS = (FX, FY, 320., 240.)
OPTICAL = (-.5, .5, -.5, .5)  # optical right/down/forward -> body forward/left/up


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx+ax*bw+ay*bz-az*by, aw*by-ax*bz+ay*bw+az*bx,
            aw*bz+ax*by-ay*bx+az*bw, aw*bw-ax*bx-ay*by-az*bz)


def mount(origin=(.1, 0., .5), pitch_deg=0., yaw_deg=0.):
    """Base<-optical transform; positive pitch looks down, positive yaw looks left."""
    p, y = math.radians(pitch_deg)/2, math.radians(yaw_deg)/2
    q = qmul((0., 0., math.sin(y), math.cos(y)),
             qmul((0., math.sin(p), 0., math.cos(p)), OPTICAL))
    return NS(translation=NS(x=origin[0], y=origin[1], z=origin[2]),
              rotation=NS(x=q[0], y=q[1], z=q[2], w=q[3]))


def project(transform, point):
    r, t = transform.rotation, transform.translation
    rot = rotation_matrix((r.x, r.y, r.z, r.w))
    optical = rot.T @ (np.asarray(point, float)-np.array([t.x, t.y, t.z]))
    return FX*optical[0]/optical[2]+320., FY*optical[1]/optical[2]+240.


def person(transform, foot=(3., .4), height=1.70, posture=1, fall_stage=0,
           ankles=True, hips=True, shoulders=True, box_bottom=None):
    points = [NS(x=math.nan, y=math.nan) for _ in range(17)]
    conf = [0.]*17

    def put(index, xyz):
        u, v = project(transform, xyz)
        points[index] = NS(x=u, y=v)
        conf[index] = .9

    if shoulders:
        for i, side in ((5, .2), (6, -.2)):
            put(i, (foot[0], foot[1]+side, height*.82))
    if hips:
        for i, side in ((11, .1), (12, -.1)):
            put(i, (foot[0], foot[1]+side, height*.53))
    if ankles:
        for i, side in ((15, .1), (16, -.1)):
            put(i, (foot[0], foot[1]+side, .08))
    u, v = project(transform, (foot[0], foot[1], 0.))
    top = project(transform, (foot[0], foot[1], height))[1]
    bottom = v if box_bottom is None else box_bottom
    return NS(keypoints_2d=points, keypoint_confidences=conf, posture=posture,
              fall_stage=fall_stage, box=[u-60., top, u+60., bottom], confidence=.9)


class GroundTest(unittest.TestCase):
    def setUp(self):
        self.t = mount()

    def locate(self, p, config=None, transform=None):
        return estimate_ground_point(p, INTRINSICS, SIZE, transform or self.t, config)

    def assertXY(self, estimate, xy, tolerance):
        self.assertAlmostEqual(estimate.point[0], xy[0], delta=tolerance)
        self.assertAlmostEqual(estimate.point[1], xy[1], delta=tolerance)
        self.assertEqual(estimate.point[2], 0.)

    def test_feet_with_ankle_height_compensation(self):
        estimate, reason = self.locate(person(self.t))
        self.assertIsNone(reason)
        self.assertEqual(estimate.method, 'feet_ankles')
        self.assertXY(estimate, (3., .4), .01)

    def test_installation_pitch_yaw_and_translation(self):
        t = mount(origin=(.2, -.1, .9), pitch_deg=20., yaw_deg=15.)
        # A pitched camera still sees a person who stands on the base-frame ground plane.
        estimate, reason = self.locate(person(t, foot=(2.5, .3)), transform=t)
        self.assertIsNone(reason)
        self.assertXY(estimate, (2.5, .3), .01)

    def test_box_bottom_fallback_disabled(self):
        self.assertIsNone(self.locate(person(self.t, ankles=False))[0])
        with self.assertRaises(ValueError):
            GroundConfig(allow_box_bottom_fallback=True)

    def test_bottom_edge_truncation_uses_height_prior(self):
        t = mount(origin=(0., 0., .5))
        p = person(t, foot=(1.2, 0.), ankles=False, box_bottom=SIZE[1])
        estimate, reason = self.locate(p, GroundConfig(enable_height_prior=True, height_prior_confirmed=True), transform=t)
        self.assertIsNone(reason)
        self.assertEqual(estimate.method, 'height_prior')
        self.assertXY(estimate, (1.2, 0.), .03)
        # A keypoint exactly on the bottom border is also truncated and cannot be feet.
        p.keypoints_2d[15] = NS(x=320., y=float(SIZE[1]))
        p.keypoint_confidences[15] = .9
        self.assertEqual(self.locate(p, GroundConfig(enable_height_prior=True, height_prior_confirmed=True), transform=t)[0].method, 'height_prior')

    def test_height_prior_error_is_reported_not_hidden(self):
        p = person(self.t, height=1.80, ankles=False, box_bottom=SIZE[1])
        estimate, _ = self.locate(p, GroundConfig(enable_height_prior=True, height_prior_confirmed=True))
        self.assertEqual(estimate.method, 'height_prior')
        # Taller than the 1.70 m prior shifts the answer; the declared std must cover it.
        error = math.hypot(estimate.point[0]-3., estimate.point[1]-.4)
        self.assertGreater(error, .05)
        self.assertGreater(estimate.std_m, .5*error)

    def test_height_prior_needs_two_anchors_and_disable_switch(self):
        one = person(self.t, ankles=False, hips=False, box_bottom=SIZE[1])
        one.keypoint_confidences[6] = 0.
        self.assertIn('two', self.locate(one, GroundConfig(enable_height_prior=True, height_prior_confirmed=True))[1])
        p = person(self.t, ankles=False, box_bottom=SIZE[1])
        self.assertIsNone(self.locate(p, GroundConfig(enable_height_prior=False))[0])

    def test_posture_support_assumptions(self):
        sitting = person(self.t, posture=2, ankles=False)
        self.assertIsNone(self.locate(sitting)[0])  # no fixed-height or box-bottom guess
        self.assertEqual(self.locate(person(self.t, posture=2))[0].method, 'feet_ankles')
        for p in (person(self.t, posture=3), person(self.t, posture=4),
                  person(self.t, fall_stage=1)):
            estimate, reason = self.locate(p)
            self.assertIsNone(estimate)
            self.assertIn('lying/fall', reason)

    def test_out_of_range_and_unusable_rays_rejected(self):
        far = person(self.t, foot=(30., 0.))
        self.assertIsNone(self.locate(far)[0])
        # Camera at ankle height has a horizontal ankle ray: no plane intersection.
        strict = GroundConfig(allow_box_bottom_fallback=False, enable_height_prior=False)
        t = mount(origin=(0., 0., .08))
        self.assertIsNone(self.locate(person(t, hips=False, shoulders=False), strict,
                                      transform=t)[0])
        # Camera below the ankle plane cannot see it as ground contact.
        t = mount(origin=(0., 0., .02))
        self.assertIsNone(self.locate(person(t, hips=False, shoulders=False), strict,
                                      transform=t)[0])

    def test_std_grows_with_range(self):
        near, _ = self.locate(person(self.t, foot=(2., 0.)))
        far, _ = self.locate(person(self.t, foot=(6., 0.)))
        self.assertLess(near.std_m, far.std_m)
        self.assertIsNone(self.locate(person(self.t, foot=(6., 0.)),
                                      GroundConfig(max_std_m=.05))[0])

    def test_invalid_keypoint_shape_and_nonfinite_transform(self):
        p = person(self.t)
        p.keypoints_2d = p.keypoints_2d[:5]
        self.assertEqual(self.locate(p)[1], 'invalid COCO17 keypoints')
        bad = mount(origin=(math.nan, 0., .5))
        with self.assertRaises(ValueError):
            self.locate(person(self.t), transform=bad)

    def test_config_validation(self):
        for kwargs in ({'person_height_m': 0.}, {'pixel_std': math.nan},
                       {'shoulder_height_ratio': .5, 'hip_height_ratio': .6},
                       {'min_keypoint_confidence': 1.5}, {'enable_height_prior': 1},
                       {'ground_z_m': math.inf}, {'ankle_height_m': -.1}):
            with self.assertRaises(ValueError, msg=str(kwargs)):
                GroundConfig(**kwargs)

    def test_camera_info_requires_rectified_uncropped(self):
        info = NS(p=[FX, 0., 320., 0., 0., FY, 240., 0., 0., 0., 1., 0.], width=640, height=480,
                  binning_x=0, binning_y=0, roi=NS(x_offset=0, y_offset=0))
        self.assertEqual(camera_model_from_info(info), (INTRINSICS, SIZE))
        for change in ({'width': 0}, {'binning_x': 2}, {'roi': NS(x_offset=8, y_offset=0)},
                       {'p': [FX, 0., 320., 5., 0., FY, 240., 0., 0., 0., 1., 0.]},
                       {'p': [FX, 0., 320., 0., 0., FY, math.nan, 0., 0., 0., 1., 0.]}):
            bad = NS(**{**vars(info), **change})
            with self.assertRaises(ValueError):
                camera_model_from_info(bad)

    def test_filter_uses_per_sample_noise_and_real_age(self):
        precise, noisy = GroundTrackFilter(), GroundTrackFilter()
        for f in (precise, noisy):
            f.update_ground((3., 0., 0.), 10., .05)
        precise.update_ground((3.3, 0., 0.), 10.1, .05)
        noisy.update_ground((3.3, 0., 0.), 10.1, 1.)
        self.assertGreater(precise.state[0], noisy.state[0])
        # Prediction never refreshes the measurement stamp and expires on its own.
        self.assertEqual(noisy.stamp, 10.1)
        self.assertIsNotNone(noisy.hold(10.3, .25))
        self.assertIsNone(noisy.hold(10.6, .25))
        self.assertIsNone(noisy.update_ground((math.nan, 0., 0.), 10.4, .1))


if __name__ == '__main__':
    unittest.main()
