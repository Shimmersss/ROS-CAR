"""Synthetic geometry tests; not camera accuracy or physical person acceptance."""
import unittest
import numpy as np

from yolo_person_tracker.depth import measure
from yolo_person_tracker.fusion import FusionConfig, fuse_depth

BOX = (20, 10, 180, 230)
K = (200, 200, 100, 120)


def skeleton():
    p = np.zeros((17, 3))
    for i, xy in {5: (70, 60), 6: (130, 60), 11: (75, 130),
                  12: (125, 130), 13: (75, 180), 14: (125, 180),
                  15: (75, 215), 16: (125, 215), 9: (40, 115)}.items():
        p[i] = (*xy, .9)
    return p


def patches(points, indices, values=2., background=0.):
    d = np.full((240, 200), background, dtype=float)
    for i, z in zip(indices, np.broadcast_to(values, (len(indices),))):
        x, y = map(int, points[i, :2])
        d[y-3:y+4, x-3:x+4] = z
    return d


class FusionTest(unittest.TestCase):
    def setUp(self):
        self.p = skeleton()

    def fuse(self, d, **kwargs):
        return fuse_depth(d, BOX, self.p, K, **kwargs)

    def test_sparse_shoulders_recover_target_without_filling_missing_joints(self):
        d = patches(self.p, [5, 6])
        self.assertIsNone(measure(d, BOX, K, .08))
        result = self.fuse(d)
        self.assertEqual(result.source, 'pose_anchors')
        np.testing.assert_allclose(result.target, (0., 0., 2.))
        self.assertEqual(np.isfinite(result.joints).all(axis=1).sum(), 2)
        self.assertTrue(np.isnan(result.joints[11]).all())

    def test_wall_does_not_outvote_four_body_anchors(self):
        d = patches(self.p, [5, 6, 11, 12], background=4.)
        self.assertAlmostEqual(measure(d, BOX, K, .08)[2], 4.)
        result = self.fuse(d)
        self.assertAlmostEqual(result.target[2], 2.)
        self.assertTrue(np.isnan(result.joints[9]).all())

    def test_joint_uses_own_depth_and_target_uses_fixed_ray(self):
        d = patches(self.p, [5, 6, 11, 12, 9], [2., 2., 2., 2., 1.7])
        result = self.fuse(d)
        np.testing.assert_allclose(result.target, (0., 0., 2.))
        np.testing.assert_allclose(result.joints[9], (-.51, -.0425, 1.7))

    def test_no_depth_does_not_invent_metric_coordinates(self):
        for z in (0., np.nan, np.inf, 9.):
            result = self.fuse(np.full((240, 200), z))
            self.assertIsNone(result.target)
            self.assertTrue(np.isnan(result.joints).all())

    def test_one_anchor_and_hands_cannot_recover_target(self):
        for indices in ([5], [9]):
            self.assertIsNone(self.fuse(patches(self.p, indices)).target)

    def test_repeated_pixels_do_not_count_as_independent_anchors(self):
        self.p[[6, 11, 12], :2] = self.p[5, :2]
        self.assertIsNone(self.fuse(patches(self.p, [5])).target)

    def test_lower_body_can_recover_when_upper_depth_missing(self):
        result = self.fuse(patches(self.p, [13, 14]))
        self.assertEqual(result.source, 'pose_lower_body')
        self.assertAlmostEqual(result.target[2], 2.)
        self.assertTrue(np.isnan(result.joints[5]).all())

    def test_equally_supported_conflicting_surfaces_rejected(self):
        result = self.fuse(patches(self.p, [5, 6, 11, 12], [2., 2., 4., 4.]))
        self.assertIsNone(result.target)
        self.assertIn('conflicting', result.reason)

    def test_physically_plausible_tilted_torso_keeps_own_joint_depths(self):
        # Same-height shoulder/hip centres separated along optical Z, not a wall 2m away.
        p=np.zeros((17,3));d=np.zeros((480,640));k=(300.,300.,320.,240.)
        for i,x,z in ((5,-.2,2.25),(6,.2,2.25),(11,-.15,1.75),(12,.15,1.75)):
            u,v=300*x/z+320,300*.65/z+240;p[i]=(u,v,.9)
            a,b=int(round(u)),int(round(v));d[b-4:b+5,a-4:a+5]=z
        result=fuse_depth(d,(250.,280.,390.,400.),p,k)
        self.assertEqual(result.source,'pose_3d_anchors')
        self.assertTrue(np.isfinite(result.joints[[5,6,11,12]]).all())
        self.assertAlmostEqual(result.joints[5,2],2.25)
        self.assertAlmostEqual(result.joints[11,2],1.75)
        self.assertTrue(np.isnan(result.joints[15]).all())

    def test_unreliable_confidence_and_mixed_patches_rejected(self):
        d = patches(self.p, [5, 6])
        self.p[6, 2] = .49
        self.assertIsNone(self.fuse(d).target)
        self.p = skeleton()
        d[:, ::2] = np.where(d[:, ::2] > 0., 4., 0.)
        self.assertIsNone(self.fuse(d).target)

    def test_torso_polygon_can_recover_without_joint_depth(self):
        d = np.zeros((240, 200)); d[78:112, 85:115] = 2.
        result = self.fuse(d)
        self.assertEqual(result.source, 'pose_torso')
        self.assertAlmostEqual(result.target[2], 2.)
        self.assertTrue(np.isnan(result.joints).all())

    def test_missing_pose_retains_region_fallback(self):
        result = fuse_depth(np.full((240, 200), 2.), BOX, (), K)
        self.assertEqual(result.source, 'regions')
        np.testing.assert_allclose(result.target, (0., 0., 2.))
        self.assertTrue(np.isnan(result.joints).all())

    def test_overlap_pixels_cannot_supply_either_person(self):
        d = patches(self.p, [5, 6, 11, 12])
        result = self.fuse(d, other_boxes=[BOX])
        self.assertIsNone(result.target)
        self.assertTrue(np.isnan(result.joints).all())
        # Non-overlapping left shoulder/hip still support this person.
        result = self.fuse(d, other_boxes=[(100, 10, 190, 230)])
        self.assertEqual(result.source, 'pose_anchors')
        self.assertTrue(np.isnan(result.joints[[6, 12]]).all())
        self.assertTrue(np.isfinite(result.joints[[5, 11]]).all())

    def test_overlap_without_pose_rejects_ambiguous_region_fallback(self):
        result = fuse_depth(np.full((240, 200), 2.), BOX, (), K,
                            other_boxes=[(100, 10, 190, 230)])
        self.assertIsNone(result.target)

    def test_bad_geometry_and_outside_keypoints(self):
        d = np.full((240, 200), 2.)
        for box in ((300, 0, 400, 100), (20, 20, 10, 10), (0, 0, np.nan, 100)):
            self.assertIsNone(fuse_depth(d, box, self.p, K).target)
        self.assertIsNone(fuse_depth(d, BOX, self.p, (0, 200, 100, 120)).target)
        self.p[[5, 6], :2] = ((5, 60), (195, 60))
        result = self.fuse(patches(self.p, [5, 6]))
        self.assertIsNone(result.target)

    def test_config_validation(self):
        for kwargs in ({'min_anchor_joints': 1}, {'min_anchor_joints': 2.5},
                       {'cluster_abs_m': 0.}, {'cluster_rel': float('nan')},
                       {'torso_scale': 1.1}, {'enabled': 1}):
            with self.assertRaises(ValueError):
                FusionConfig(**kwargs)


if __name__ == '__main__':
    unittest.main()
