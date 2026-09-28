import unittest
import numpy as np
from yolo_person_tracker.depth import DepthTrackFilter, measure


class DepthTest(unittest.TestCase):
    def test_temporal_filter_smooths_and_resets_large_jump(self):
        f = DepthTrackFilter(alpha=.5, jump_reset_m=.8)
        self.assertEqual(f.update((0., 0., 2.), 1.), (0., 0., 2.))
        self.assertAlmostEqual(f.update((0., 0., 2.4), 2.)[2], 2.2, places=5)
        self.assertEqual(f.update((0., 0., 4.), 3.), (0., 0., 4.))

    def test_temporal_filter_does_not_turn_invalid_into_position(self):
        f = DepthTrackFilter()
        self.assertIsNone(f.update(None, 1.))
        self.assertEqual(f.update((0., 0., 2.), 2.), (0., 0., 2.))
        self.assertIsNone(f.update(None, 3.))

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

    def test_recovers_from_non_torso_body_region(self):
        d = np.full((100, 100), 4.)
        # Simulate a depth hole over the torso while the lower body remains
        # measurable inside the same YOLO detection.
        d[25:61, 30:70] = 0.
        d[60:95, 20:80] = 2.
        xyz = measure(d, (0, 0, 100, 100), (100, 100, 50, 50))
        self.assertIsNotNone(xyz)
        self.assertAlmostEqual(xyz[2], 2.)
