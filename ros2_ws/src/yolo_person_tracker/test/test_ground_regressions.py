"""Counterexamples from the independent review, not field accuracy tests."""
import unittest
from types import SimpleNamespace as NS
from test_ground import mount, person, project, INTRINSICS, SIZE
from yolo_person_tracker.ground import GroundConfig, estimate_ground_point

class GroundRegressions(unittest.TestCase):
    def test_occluded_feet_never_use_box_bottom(self):
        t=mount()
        p=person(t, ankles=False, posture=0, box_bottom=285.)
        self.assertIsNone(estimate_ground_point(p,INTRINSICS,SIZE,t)[0])

    def test_unknown_seated_person_never_uses_standing_height(self):
        t=mount();p=person(t,ankles=False,hips=False,posture=0,box_bottom=480.)
        for i,y in ((5,.6),(6,.2)):
            u,v=project(t,(3.,y,1.0));p.keypoints_2d[i]=NS(x=u,y=v)
        self.assertIsNone(estimate_ground_point(p,INTRINSICS,SIZE,t,
            GroundConfig(enable_height_prior=True, height_prior_confirmed=True))[0])

    def test_conflicting_height_anchors_are_rejected(self):
        t=mount();p=person(t,ankles=False,box_bottom=480.)
        for i,y in ((11,.5),(12,.3)):
            u,v=project(t,(5.,y,1.7*.53));p.keypoints_2d[i]=NS(x=u,y=v)
        cfg=GroundConfig(enable_height_prior=True, height_prior_confirmed=True)
        self.assertIsNone(estimate_ground_point(p,INTRINSICS,SIZE,t,cfg)[0])

    def test_single_ankle_is_not_enough(self):
        t=mount();p=person(t);p.keypoint_confidences[16]=0.
        self.assertIsNone(estimate_ground_point(p,INTRINSICS,SIZE,t)[0])

    def test_conflicting_feet_do_not_fall_back_to_prior(self):
        t=mount();p=person(t)
        u,v=project(t,(5.,.3,.08));p.keypoints_2d[16]=NS(x=u,y=v)
        e,r=estimate_ground_point(p,INTRINSICS,SIZE,t,
            GroundConfig(enable_height_prior=True,height_prior_confirmed=True))
        self.assertIsNone(e);self.assertIn('conflicting',r)

    def test_enable_without_confirmation_does_not_use_prior(self):
        t=mount();p=person(t,ankles=False,box_bottom=480.)
        self.assertIsNone(estimate_ground_point(p,INTRINSICS,SIZE,t,
            GroundConfig(enable_height_prior=True))[0])
