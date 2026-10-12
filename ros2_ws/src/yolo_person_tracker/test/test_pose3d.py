import unittest
import numpy as np
from test_pose import skeleton
from yolo_person_tracker.pose import PoseConfig, UNKNOWN, STANDING, SITTING_CROUCHING, LYING, FALLEN
from yolo_person_tracker.pose3d import EnhancedPostureTracker, Pose3DConfig

BOX=(50,10,150,210)


def spatial(center_height=1.1, lying=False, seated=False):
    xyz=np.full((17,3),np.nan)
    center=np.array([0.,1.-center_height,2.])
    torso=np.array([0.,0.,.5]) if lying else np.array([0.,-.5,0.])
    for i,side in ((5,-.2),(6,.2)):xyz[i]=center+torso/2+[side,0.,0.]
    for i,side in ((11,-.15),(12,.15)):xyz[i]=center-torso/2+[side,0.,0.]
    for i,h in ((13,11),(14,12)):xyz[i]=xyz[h]+([0.,.1,-.4] if seated else [0.,.4,0.])
    return xyz


class Pose3DTests(unittest.TestCase):
    def tracker(self,**kwargs):
        return EnhancedPostureTracker(PoseConfig(upright_confirmed=True),
            Pose3DConfig(gravity_confirmed=True,ground_confirmed=True,**kwargs))
    def update(self,t,s,j,identity='x',points=None):
        return t.update(identity,s,BOX,skeleton() if points is None else points,j)
    def stable(self,t):
        for s in (1.,1.2,1.4):self.update(t,s,spatial())

    def test_3d_standing_sitting_static_lying(self):
        t=self.tracker()
        self.assertEqual(self.update(t,1.,spatial())[0],STANDING)
        self.assertEqual(self.update(t,1.2,spatial(seated=True))[0],SITTING_CROUCHING)
        for s in np.arange(1.4,4.,.2):
            result=self.update(t,float(s),spatial(.35,True),identity='lying')
            self.assertEqual(result[:2],(LYING,0))
            self.assertIn('basis=3d',result[2])

    def test_metric_fall_towards_camera_and_recovery(self):
        t=self.tracker();self.stable(t)
        # 2D remains vertical: measured 3D detects horizontal torso in depth direction.
        for s in np.arange(1.6,2.9,.2):result=self.update(t,float(s),spatial(.35,True))
        self.assertEqual(result[:2],(FALLEN,2))
        for s in np.arange(3.,5.4,.2):result=self.update(t,float(s),spatial())
        self.assertEqual(result[:2],(STANDING,0))

    def test_continuous_rotation_retains_recent_baseline(self):
        t=self.tracker();self.stable(t)
        for s,angle,height in ((1.6,25.,.95),(1.8,48.,.75),(2.,75.,.5),(2.2,90.,.35)):
            j=spatial(height)
            a=np.radians(angle);rot=np.array([[1,0,0],[0,np.cos(a),-np.sin(a)],[0,np.sin(a),np.cos(a)]])
            center=(j[5]+j[6]+j[11]+j[12])/4
            j=(j-center)@rot.T+center
            result=self.update(t,s,j)
        self.assertEqual(result[1],1)
        for s in (2.4,2.6,2.8,3.1):result=self.update(t,s,spatial(.35,True))
        self.assertEqual(result[:2],(FALLEN,2))

    def test_bending_high_torso_not_fall(self):
        t=self.tracker();self.stable(t)
        for s in np.arange(1.6,3.,.2):
            result=self.update(t,float(s),spatial(.9,True))
            self.assertEqual(result[:2],(UNKNOWN,0))

    def test_slow_lying_not_fall(self):
        t=self.tracker();self.stable(t)
        for i,s in enumerate(np.arange(1.6,3.6,.2)):
            result=self.update(t,float(s),spatial(max(.35,1.05-i*.08),True))
            self.assertNotEqual(result[1],2)

    def test_short_depth_gap_preserves_baseline_without_cross_id_transfer(self):
        t=self.tracker();self.stable(t)
        j=spatial();j[5]=np.nan
        result=self.update(t,1.5,j)
        self.assertIn('basis=2d',result[2])
        self.assertEqual(self.update(t,1.6,spatial(.35,True))[:2],(LYING,1))
        self.assertEqual(self.update(t,1.8,spatial(.35,True),'new')[:2],(LYING,0))

    def test_no_calibration_falls_back_and_bad_geometry_rejects(self):
        t=EnhancedPostureTracker(PoseConfig(),Pose3DConfig())
        self.assertIn('basis=2d',self.update(t,1.,spatial())[2])
        t=self.tracker();j=spatial();j[5,0]-=2.
        result=self.update(t,1.,j)
        self.assertEqual(result[:2],(UNKNOWN,0));self.assertIn('invalid3d',result[2])
        j=spatial();j[13,1]+=2.
        self.assertEqual(self.update(t,1.1,j)[0],UNKNOWN)

    def test_missing_knee_depth_falls_back_without_filling(self):
        t=self.tracker();j=spatial();j[13]=np.nan
        result=self.update(t,1.,j)
        self.assertIn('basis=2d',result[2]);self.assertTrue(np.isnan(j[13]).all())

    def test_unconfirmed_ground_no_3d_fall(self):
        t=EnhancedPostureTracker(PoseConfig(upright_confirmed=True),Pose3DConfig(gravity_confirmed=True))
        self.stable(t)
        for s in np.arange(1.6,3.,.2):
            self.assertEqual(self.update(t,float(s),spatial(.35,True))[1],0)

    def test_gap_rewind_and_epoch_do_not_invent_fall(self):
        for stamp,identity in ((2.1,'x'),(.8,'x'),(1.6,'1:x')):
            t=self.tracker();self.stable(t)
            self.assertEqual(self.update(t,stamp,spatial(.35,True),identity)[:2],(LYING,0))

    def test_rotated_gravity_and_bad_params(self):
        t=EnhancedPostureTracker(PoseConfig(),Pose3DConfig(gravity_confirmed=True,ground_confirmed=True,up_x=1.,up_y=0.))
        j=spatial();j[:,:2]=np.stack([-j[:,1],j[:,0]],axis=1)
        self.assertEqual(self.update(t,1.,j)[0],STANDING)
        with self.assertRaises(ValueError):Pose3DConfig(up_y=2.)
        with self.assertRaises(ValueError):Pose3DConfig(ground_confirmed=True)
