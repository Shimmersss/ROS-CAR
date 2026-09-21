import math
from types import SimpleNamespace
import unittest
from motion_guard.safety import SafetyConfig, clearance


def scan(value=3.):
    return SimpleNamespace(angle_min=-math.pi,angle_increment=math.pi/180,
                           angle_max=math.pi-math.pi/180,range_min=.15,range_max=15.,ranges=[value]*360)


class SafetyTest(unittest.TestCase):
    def test_clear(self):
        self.assertTrue(clearance(scan(),(0,0,0),.15,.5,SafetyConfig())[0])

    def test_all_sides_and_stopped_request(self):
        for i in (0,90,180,270):
            s=scan();s.ranges[i]=.4
            for v,w in ((.15,0.),(0.,.5),(0.,0.)):
                self.assertFalse(clearance(s,(0,0,0),v,w,SafetyConfig())[0])

    def test_unknown_and_infinite_contract(self):
        for v in (math.nan,-math.inf,0.,16.,math.inf):
            s=scan();s.ranges[180]=v
            self.assertFalse(clearance(s,(0,0,0),.1,0.,SafetyConfig())[0])
        self.assertTrue(clearance(scan(math.inf),(0,0,0),0,0,SafetyConfig(allow_infinite_clear=True))[0])

    def test_bad_geometry(self):
        for key,value in (('angle_increment',0.),('range_max',math.nan),('angle_max',0.)):
            s=scan();setattr(s,key,value)
            self.assertFalse(clearance(s,(0,0,0),0,0,SafetyConfig())[0])

    def test_mount_translation_rotation(self):
        s=scan();s.ranges[180]=1.
        self.assertTrue(clearance(s,(0,0,0),0,0,SafetyConfig())[0])
        self.assertFalse(clearance(s,(1,0,math.pi),0,0,SafetyConfig())[0])

    def test_sensor_blind_zone(self):
        s=scan();s.range_min=.3
        self.assertEqual(clearance(s,(0,0,0),.1,0,SafetyConfig())[:2],
                         (False,'sensor_blind_zone_outside_body'))
        self.assertFalse(clearance(scan(),(0,.1,0),.1,0,SafetyConfig())[0])
        self.assertTrue(clearance(scan(),(.05,0,0),.1,0,SafetyConfig())[0])

    def test_configuration(self):
        for kwargs in ({'length_m':0.},{'deceleration_mps2':math.nan},{'request_timeout_s':-1.},
                       {'max_linear_mps':2.}):
            with self.assertRaises(ValueError): SafetyConfig(**kwargs)

if __name__=='__main__':unittest.main()
