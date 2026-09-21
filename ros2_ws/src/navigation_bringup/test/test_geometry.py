import math
import unittest
from navigation_bringup.geometry import standoff

class Goals(unittest.TestCase):
    def test_standoff(self):
        self.assertEqual(standoff((0,0),(3,0),1),(2,0,0))
        x,y,yaw=standoff((1,1),(1,4),1)
        self.assertAlmostEqual(x,1);self.assertAlmostEqual(y,3);self.assertAlmostEqual(yaw,math.pi/2)
    def test_close_person_hold(self):
        self.assertIsNone(standoff((0,0),(0,0),1))
        self.assertIsNone(standoff((0,0),(.9,0),1))
    def test_invalid(self):
        for person,distance in [((float('nan'),1),1),((1,1),0),((1,1),-1)]:
            with self.assertRaises(ValueError):standoff((0,0),person,distance)
