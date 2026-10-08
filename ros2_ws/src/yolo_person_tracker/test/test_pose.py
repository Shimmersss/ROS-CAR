import unittest
import numpy as np
from yolo_person_tracker.pose import *
from yolo_person_tracker.registration import Registration


def skeleton(lying=False, center=100., knees=190.):
    p=np.zeros((17,3)); p[:,2]=.9
    p[:,0]=100.; p[:,1]=100.
    if lying:
        p[5,:2]=(55,center-5); p[6,:2]=(65,center-5)
        p[11,:2]=(145,center+5); p[12,:2]=(155,center+5)
    else:
        p[5,:2]=(90,center-25); p[6,:2]=(110,center-25)
        p[11,:2]=(90,center+25); p[12,:2]=(110,center+25)
    p[13,:2]=(90,knees);p[14,:2]=(110,knees)
    return tuple(map(tuple,p))


class PoseTests(unittest.TestCase):
    def tracker(self,confirmed=True):
        return PostureTracker(PoseConfig(upright_confirmed=confirmed))

    def stable(self,t):
        for s in (1.,1.2,1.4): t.update('0:1',s,(50,10,150,210),skeleton())

    def test_postures(self):
        t=self.tracker()
        self.assertEqual(t.update('x',1.,(50,10,150,210),skeleton())[0],STANDING)
        self.assertEqual(t.update('x',1.2,(50,10,150,210),skeleton(knees=140.))[0],SITTING_CROUCHING)
        self.assertEqual(t.update('x',1.4,(50,100,250,200),skeleton(True,150))[0],LYING)

    def test_fall_and_recovery(self):
        t=self.tracker();self.stable(t)
        result=t.update('0:1',1.6,(20,130,240,220),skeleton(True,180))
        self.assertEqual(result[:2],(LYING,1))
        for s in (1.9,2.2,2.5,2.7): result=t.update('0:1',s,(20,130,240,220),skeleton(True,180))
        self.assertEqual(result[:2],(FALLEN,2))
        for s in np.arange(2.9,5.2,.2): result=t.update('0:1',float(s),(50,10,150,210),skeleton())
        self.assertEqual(result[:2],(STANDING,0))

    def test_static_lying_never_fall(self):
        t=self.tracker()
        for s in np.arange(1.,4.,.2):
            self.assertEqual(t.update('x',float(s),(20,130,240,220),skeleton(True,180))[:2],(LYING,0))

    def test_unconfirmed_mount(self):
        t=self.tracker(False);self.stable(t)
        for s in np.arange(1.6,3.,.2): result=t.update('0:1',float(s),(20,130,240,220),skeleton(True,180))
        self.assertEqual(result[:2],(LYING,1))

    def test_missing_id_gap_and_reset(self):
        for mode in ('id','gap','backwards','missing','reset'):
            t=self.tracker();self.stable(t)
            if mode=='missing': t.update('0:1',1.5,(50,10,150,210),())
            if mode=='reset': t.reset()
            stamp=2.1 if mode=='gap' else .8 if mode=='backwards' else 1.6
            identity='0:2' if mode=='id' else '0:1'
            self.assertEqual(t.update(identity,stamp,(20,130,240,220),skeleton(True,180))[:2],(LYING,0),mode)

    def test_slow_descent_and_bending(self):
        t=self.tracker();self.stable(t)
        for s,c in [(1.6,115),(1.8,125),(2.,135),(2.2,145),(2.4,155),(2.6,165)]:
            result=t.update('0:1',s,(20,c-50,240,c+40),skeleton(True,c))
            self.assertNotEqual(result[1],2)
        t=self.tracker();self.stable(t)
        p=np.array(skeleton());p[[5,6],0]-=50
        self.assertEqual(t.update('0:1',1.6,(50,10,150,210),tuple(map(tuple,p)))[0],UNKNOWN)

    def test_depth_holes_background_and_intrinsics(self):
        p=skeleton();z=np.ones((240,320),np.float32)*2
        result=joints3d(z,p,(100.,100.,160.,120.),reference=(0,0,2))
        self.assertTrue(np.isfinite(result).all())
        self.assertAlmostEqual(result[5,0],-1.4)
        self.assertTrue(np.isnan(joints3d(z*0,p,(100.,100.,160.,120.),reference=(0,0,2))).all())
        self.assertTrue(np.isnan(joints3d(z*3,p,(100.,100.,160.,120.),reference=(0,0,2))).all())
        z[72:79,87:94]=np.tile([1.,1.,1.,4.,4.,4.,4.],(7,1))
        self.assertTrue(np.isnan(joints3d(z,p,(100.,100.,160.,120.),reference=(0,0,2))[5]).all())

    def test_bounded_state_and_bad_config(self):
        t=self.tracker()
        for i in range(500):t.update(str(i),1.,(50,10,150,210),skeleton())
        self.assertLessEqual(len(t.tracks),128)
        with self.assertRaises(ValueError): PoseConfig(horizontal_deg=10.)


class RegistrationTests(unittest.TestCase):
    def calibration(self):
        i=dict(width=4,height=3,fx=2.,fy=2.,cx=1.5,cy=1.)
        return dict(color_intrinsic=i,depth_intrinsic=i,color_distortion=[0.]*8,depth_distortion=[0.]*8,
                    depth_to_color_rotation=np.eye(3).reshape(-1).tolist(),depth_to_color_translation_mm=[0.]*3)

    def test_identity_units_holes_and_shape(self):
        r=Registration(self.calibration());color=np.zeros((3,4,3),np.uint8);depth=np.full((3,4),2.,np.float32);depth[0,0]=0
        rect,z=r.apply(color,depth);np.testing.assert_equal(z,depth)
        with self.assertRaises(ValueError):r.apply(color,np.zeros((4,3)))

    def test_actual_translation_and_invalid_rotation(self):
        calibration=self.calibration();calibration['depth_to_color_translation_mm']=[1000.,0.,0.]
        r=Registration(calibration);_,z=r.apply(np.zeros((3,4,3),np.uint8),np.full((3,4),2.,np.float32))
        self.assertTrue((z[:,0]==0).all());self.assertTrue((z[:,1:]==2).all())
        calibration['depth_to_color_rotation']=[0.]*9
        with self.assertRaises(ValueError):Registration(calibration)


if __name__=='__main__':unittest.main()


class JointJumpGateTest(unittest.TestCase):
    @staticmethod
    def joints(z):
        xyz = np.full((17,3), np.nan)
        xyz[:,2] = z
        xyz[:,:2] = 0.
        return xyz

    def test_outlier_never_becomes_reference(self):
        gate = JointJumpGate(.5, 2., .5)
        gate.apply(self.joints(2.), 1.)
        out, rejected = gate.apply(self.joints(3.), 1.033)
        self.assertEqual(rejected, 17)
        self.assertTrue(np.isnan(out).all())
        out, rejected = gate.apply(self.joints(2.02), 1.066)
        self.assertEqual(rejected, 0)
        np.testing.assert_allclose(out[:,2], 2.02)

    def test_reappearing_joint_is_checked_and_reference_expires(self):
        gate = JointJumpGate(.5, 2., .5)
        gate.apply(self.joints(2.), 1.)
        missing = self.joints(2.); missing[15] = np.nan
        gate.apply(missing, 1.1)
        out, rejected = gate.apply(self.joints(3.5), 1.2)
        self.assertEqual(rejected, 17)
        out, rejected = gate.apply(self.joints(3.5), 1.55)
        self.assertEqual(rejected, 16)  # joint 15 expired (0.55 s); the rest are 0.45 s old
        self.assertEqual(out[15,2], 3.5)

    def test_allowance_grows_with_age_and_time_reversal_accepts(self):
        gate = JointJumpGate(.5, 2., .5)
        gate.apply(self.joints(2.), 1.)
        self.assertEqual(gate.apply(self.joints(2.8), 1.2)[1], 0)
        self.assertEqual(gate.apply(self.joints(5.), 1.)[1], 0)
