import math
import unittest
from types import SimpleNamespace as NS
import numpy as np
from test_ground import mount, person, INTRINSICS, SIZE
from yolo_person_tracker.ground import GroundConfig, GroundEstimate, rotation_matrix
from yolo_person_tracker.unified import UnifiedConfig, UnifiedTrack, candidates


def measured(transform, foot=(2.,.4), indices=(5,6,11,12,15,16)):
    p=person(transform,foot=foot)
    r,t=transform.rotation,transform.translation
    rotation=rotation_matrix((r.x,r.y,r.z,r.w))
    origin=np.array([t.x,t.y,t.z])
    p.keypoints_3d=[NS(x=math.nan,y=math.nan,z=math.nan) for _ in range(17)]
    p.keypoints_3d_valid=[False]*17
    for i,side,height in ((5,.2,1.394),(6,-.2,1.394),(11,.1,.901),(12,-.1,.901),(15,.1,.08),(16,-.1,.08)):
        if i in indices:
            xyz=rotation.T@(np.array([foot[0],foot[1]+side,height])-origin)
            p.keypoints_3d[i]=NS(x=xyz[0],y=xyz[1],z=xyz[2]);p.keypoints_3d_valid[i]=True
    return p


def estimate(x=2.,method='depth_ankles'):
    return GroundEstimate((x,0.,0.),.9,.1,method,'test')


class UnifiedTests(unittest.TestCase):
    def test_same_reference_and_body_projection(self):
        t=mount()
        for indices,method in [((15,16),'depth_ankles'),((5,6),'depth_body_projection')]:
            p=measured(t,indices=indices)
            d,m,hard,_=candidates(p,(INTRINSICS,SIZE),t,GroundConfig(),UnifiedConfig())
            self.assertFalse(hard);self.assertEqual(d.method,method)
            np.testing.assert_allclose(d.point,(2.,.4,0.),atol=1e-6)
            np.testing.assert_allclose(d.point,m.point,atol=1e-6)
            if method=='depth_body_projection':self.assertFalse(p.keypoints_3d_valid[15])

    def test_region_depth_projection_does_not_fill_joints(self):
        t=mount();p=measured(t,indices=())
        p.body_depth_valid=True;p.body_depth_m=1.9
        d,m,hard,_=candidates(p,(INTRINSICS,SIZE),t,GroundConfig(),UnifiedConfig())
        self.assertFalse(hard);self.assertEqual(d.method,'depth_body_projection')
        np.testing.assert_allclose(d.point,(2.,.4,0.),atol=1e-6)
        self.assertFalse(any(p.keypoints_3d_valid))

    def test_far_and_missing_ankles(self):
        t=mount();p=measured(t,foot=(6.,.4))
        d,m,hard,_=candidates(p,(INTRINSICS,SIZE),t,GroundConfig(),UnifiedConfig())
        self.assertIsNone(d);self.assertIsNotNone(m);self.assertFalse(hard)
        p.keypoint_confidences[15]=0.
        self.assertEqual(candidates(p,(INTRINSICS,SIZE),t,GroundConfig(),UnifiedConfig())[:2],(None,None))

    def test_bad_depth_contact_and_lying(self):
        t=mount();p=measured(t)
        p.keypoints_3d[15].y+=1.
        d,m,hard,_=candidates(p,(INTRINSICS,SIZE),t,GroundConfig(),UnifiedConfig())
        self.assertTrue(hard);self.assertIsNone(d)
        p.posture=3
        d,m,hard,_=candidates(p,(INTRINSICS,SIZE),t,GroundConfig(),UnifiedConfig())
        self.assertTrue(hard);self.assertEqual((d,m),(None,None))

    def test_mount_rotation(self):
        t=mount(pitch_deg=12.,yaw_deg=20.)
        p=measured(t)
        d,m,hard,_=candidates(p,(INTRINSICS,SIZE),t,GroundConfig(),UnifiedConfig())
        self.assertFalse(hard);np.testing.assert_allclose(d.point,(2.,.4,0.),atol=1e-6)

    def test_initial_confirmation_and_switch_both_directions(self):
        t=UnifiedTrack()
        for stamp in (1.,1.1):self.assertIsNone(t.update(stamp,estimate(),estimate(method='mono_ankles'))[0])
        self.assertEqual(t.update(1.2,estimate(),estimate(method='mono_ankles'))[0].method,'fused_contact')
        for stamp in (1.3,1.4):self.assertEqual(t.update(stamp,None,estimate(method='mono_ankles'))[0].method,'mono_ankles')
        self.assertEqual(t.update(1.5,None,estimate(method='mono_ankles'))[0].method,'mono_ankles')
        for stamp in (1.6,1.7):self.assertIsNone(t.update(stamp,estimate(),None)[0])
        self.assertEqual(t.update(1.8,estimate(),None)[0].method,'depth_ankles')

    def test_depth_reacquisition_keeps_fresh_mono(self):
        t=UnifiedTrack()
        for s in (1.,1.1,1.2):t.update(s,None,estimate(method='mono_ankles'))
        for s in (1.3,1.4):
            self.assertEqual(t.update(s,estimate(),estimate(method='mono_ankles'))[0].method,'fused_contact')
        self.assertEqual(t.update(1.5,estimate(),estimate(method='mono_ankles'))[0].method,'fused_contact')

    def test_conflict_jump_and_no_stale_prediction(self):
        t=UnifiedTrack()
        for s in (1.,1.1,1.2):t.update(s,estimate(),None)
        self.assertIsNone(t.update(1.3,estimate(),estimate(5.,'mono_ankles'))[0])
        self.assertIsNone(t.update(1.4,None,None)[0])
        self.assertIsNone(t.update(1.5,estimate(6.),None)[0])
        self.assertIsNone(t.update(2.1,estimate(6.),None)[0])
        self.assertIsNone(t.update(2.2,estimate(6.),None)[0])
        self.assertIsNotNone(t.update(2.3,estimate(6.),None)[0])

    def test_rewind_and_gap_reconfirm(self):
        for stamp in (.8,2.):
            t=UnifiedTrack()
            for s in (1.,1.1,1.2):t.update(s,estimate(),None)
            self.assertIsNone(t.update(stamp,estimate(),None)[0])
        with self.assertRaises(ValueError):UnifiedConfig(confirm_frames=1)
