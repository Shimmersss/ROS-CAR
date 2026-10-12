"""Regressions for continuous pixel/metric histories and common-reference fusion."""
import unittest
import numpy as np
from test_pose import skeleton
from test_pose3d import spatial, BOX
from test_ground import mount, INTRINSICS, SIZE
from test_unified import measured, estimate
from yolo_person_tracker.pose import PoseConfig, FALLEN, LYING, STANDING
from yolo_person_tracker.pose3d import Pose3DConfig, EnhancedPostureTracker
from yolo_person_tracker.body_contact import BodyContact, BodyContactConfig, body_anchor
from yolo_person_tracker.ground import GroundConfig, GroundEstimate
from yolo_person_tracker.unified import UnifiedConfig, conservative_fusion, UnifiedTrack


class ContinuousPostureTests(unittest.TestCase):
    def tracker(self):
        return EnhancedPostureTracker(PoseConfig(upright_confirmed=True),
            Pose3DConfig(gravity_confirmed=True,ground_confirmed=True))

    def stable(self,t):
        for s in (1.,1.2,1.4):t.update('a',s,BOX,skeleton(),spatial())

    def test_2d_history_survives_depth_loss_during_fall(self):
        t=self.tracker();self.stable(t)
        # Reliable 3D saw standing. The complete fall then happens without depth.
        for s in np.arange(1.6,2.9,.2):
            result=t.update('a',float(s),(20,130,240,220),skeleton(True,180),None)
        self.assertEqual(result[:2],(FALLEN,2))
        self.assertIn('confirmed_by=2d',result[2])

    def test_depth_flicker_does_not_restart_planar_candidate(self):
        t=self.tracker();self.stable(t)
        for i,s in enumerate(np.arange(1.6,3.,.15)):
            j=spatial(.35,True) if i%2==0 else None
            result=t.update('a',float(s),(20,130,240,220),skeleton(True,180),j)
        self.assertEqual(result[:2],(FALLEN,2))

    def test_metric_pause_does_not_count_missing_time(self):
        t=self.tracker();self.stable(t)
        t.update('a',1.6,BOX,skeleton(),spatial(.35,True))
        t.update('a',1.7,BOX,skeleton(),None)
        result=t.update('a',1.9,BOX,skeleton(),spatial(.35,True))
        self.assertEqual(result[1],1)
        self.assertAlmostEqual(t.metric.tracks['a']['pending'][0],1.8)
        for s in (2.1,2.3,2.5,2.7):
            result=t.update('a',s,BOX,skeleton(),spatial(.35,True))
        self.assertEqual(result[1],1)
        result=t.update('a',2.9,BOX,skeleton(),spatial(.35,True))
        self.assertEqual(result[:2],(FALLEN,2))

    def test_long_depth_gap_drops_metric_candidate_but_not_pixel_history(self):
        t=self.tracker();self.stable(t)
        t.update('a',1.6,BOX,skeleton(),spatial(.35,True))
        for s in (1.8,2.,2.2):t.update('a',s,BOX,skeleton(),None)
        result=t.update('a',2.4,BOX,skeleton(),spatial(.35,True))
        self.assertEqual(result[:2],(LYING,0))
        self.assertGreater(len(t.planar.tracks['a']['history']),1)

    def test_reliable_high_3d_vetoes_perspective_fall(self):
        t=self.tracker();self.stable(t)
        for s in np.arange(1.6,3.,.2):
            result=t.update('a',float(s),(20,130,240,220),skeleton(True,180),spatial(.9,True))
            self.assertNotEqual(result[1],2)
        self.assertFalse(t.planar.tracks['a']['fallen'])

    def test_latched_metric_event_survives_missing_depth(self):
        t=self.tracker();self.stable(t)
        for s in np.arange(1.6,3.,.2):t.update('a',float(s),BOX,skeleton(),spatial(.35,True))
        for s in np.arange(3.,5.5,.2):
            result=t.update('a',float(s),BOX,skeleton(),None)
            self.assertEqual(result[:2],(FALLEN,2))
        for s in np.arange(5.5,7.8,.2):result=t.update('a',float(s),BOX,skeleton(),spatial())
        self.assertEqual(result[:2],(STANDING,0))

    def test_holes_or_scale_switch_alone_cannot_create_event(self):
        t=self.tracker()
        for i,s in enumerate(np.arange(1.,4.,.1)):
            result=t.update('a',float(s),BOX,skeleton(),spatial() if i%2 else None)
            self.assertEqual(result[1],0)
        t.update('a',4.1,BOX,(),None)
        result=t.update('a',4.2,(20,130,240,220),skeleton(True,180),None)
        self.assertEqual(result[:2],(LYING,0))


class BodyFusionTests(unittest.TestCase):
    def test_correlated_fusion_does_not_invent_extra_precision(self):
        a=GroundEstimate((2.,0.,0.),.9,.2,'depth','')
        b=GroundEstimate((2.2,0.,0.),.8,.4,'mono','')
        f=conservative_fusion([a,b])
        self.assertGreater(f.point[0],2.);self.assertLess(f.point[0],2.2)
        self.assertGreaterEqual(f.std_m,.2)
        self.assertAlmostEqual(conservative_fusion([a,a,a]).std_m,a.std_m)

    def paired(self):
        m=BodyContact()
        foot=estimate()
        for s in (1.,1.1,1.2):
            e,_=m.observe(s,np.array([1.9,0.,1.]),foot,.2,0.,1.)
            self.assertIsNone(e)  # Never use a mapping learned on the same frame.
        return m

    def test_current_body_moves_without_ankles_then_expires(self):
        m=self.paired()
        e,_=m.observe(1.3,np.array([2.1,0.,1.]),None,.2,0.,1.)
        self.assertAlmostEqual(e.point[0],2.2)
        later,_=m.observe(1.7,np.array([2.3,0.,1.]),None,.2,0.,1.)
        self.assertAlmostEqual(later.point[0],2.4)
        self.assertGreater(later.std_m,e.std_m)
        expired,_=m.observe(2.,np.array([2.4,0.,1.]),None,.2,0.,1.)
        self.assertIsNone(expired)

    def test_no_unpaired_fallback_or_cross_track_transfer(self):
        self.assertIsNone(BodyContact().observe(1.,np.array([2.,0.,1.]),None,.2,0.,1.)[0])
        m=self.paired()
        self.assertIsNone(m.observe(1.3,None,None,.2,0.,1.)[0])
        self.assertIsNone(m.observe(1.4,np.array([2.,0.,1.]),None,.2,0.,1.)[0])
        m=self.paired()
        self.assertIsNone(m.observe(.8,np.array([2.,0.,1.]),None,.2,0.,1.)[0])

    def test_changed_relation_or_conflict_invalidates_mapping(self):
        m=self.paired()
        self.assertIsNone(m.observe(1.3,np.array([1.,0.,1.]),estimate(),.2,0.,1.)[0])
        self.assertIsNone(m.offset)
        m=self.paired()
        self.assertIsNone(m.observe(1.3,np.array([1.9,0.,1.]),None,.2,0.,1.,hard=True)[0])

    def test_body_anchor_uses_current_depth_and_pose_not_box_center(self):
        t=mount();p=measured(t);p.body_depth_valid=True;p.body_depth_m=1.9
        anchor=body_anchor(p,(INTRINSICS,SIZE),t,GroundConfig(),UnifiedConfig(),BodyContactConfig())
        self.assertIsNotNone(anchor)
        p.box=[10.,10.,600.,470.]
        np.testing.assert_allclose(anchor,body_anchor(p,(INTRINSICS,SIZE),t,GroundConfig(),UnifiedConfig(),BodyContactConfig()))
        p.posture=2
        self.assertIsNone(body_anchor(p,(INTRINSICS,SIZE),t,GroundConfig(),UnifiedConfig(),BodyContactConfig()))

    def test_conflicting_body_or_excessive_uncertainty_is_rejected(self):
        t=UnifiedTrack()
        for s in (1.,1.1,1.2):t.update(s,estimate(),estimate(method='mono_ankles'))
        self.assertIsNone(t.update(1.3,estimate(),None,body=estimate(5.,'body_contact'))[0])
        broad=GroundEstimate((2.,0.,0.),.5,2.,'body_contact','')
        self.assertIsNone(t.update(1.4,None,None,body=broad,max_std=1.)[0])

    def test_fused_stream_keeps_shared_body_source_across_ankle_loss(self):
        t=UnifiedTrack()
        body=estimate(method='body_contact')
        for s in (1.,1.1,1.2):t.update(s,estimate(),estimate(method='mono_ankles'),body=body)
        result,_=t.update(1.3,None,None,body=body)
        self.assertEqual(result.method,'body_contact')
        self.assertIsNone(t.update(1.4,None,None)[0])
