import unittest
import numpy as np
from yolo_person_tracker.depth import measure


class DepthTest(unittest.TestCase):
    def test_plane_and_units(self):
        xyz = measure(np.full((100,100), 2.), (0,0,100,100), (100,100,50,50))
        self.assertAlmostEqual(xyz[2], 2.)
        self.assertLess(abs(xyz[0]), .02)
        self.assertLess(xyz[1], 0)

    def test_invalid_sparse_and_background(self):
        for value in (0., float('nan'), float('inf'), 9.):
            self.assertIsNone(measure(np.full((100,100), value), (0,0,100,100), (100,100,50,50)))
        d = np.zeros((100,100)); d[40,40] = 2
        self.assertIsNone(measure(d, (0,0,100,100), (100,100,50,50)))
        d[:,::2] = 2; d[:,1::2] = 4
        self.assertIsNone(measure(d, (0,0,100,100), (100,100,50,50)))

    def test_bad_box_and_calibration(self):
        d = np.ones((100,100))
        for box in ((20,20,10,10), (200,200,300,300), (0,0,float('nan'),100)):
            self.assertIsNone(measure(d, box, (100,100,50,50)))
        self.assertIsNone(measure(d, (0,0,100,100), (0,100,50,50)))
