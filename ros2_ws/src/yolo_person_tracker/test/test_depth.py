import unittest
import numpy as np
from yolo_person_tracker.depth import DepthTrackFilter, measure


class DepthTest(unittest.TestCase):
    def test_kalman_reduces_noise_and_tracks_velocity(self):
        rng = np.random.default_rng(42)
        f = DepthTrackFilter()
        errors, raw = [], []
        for i in range(180):
            t = i*.05
            truth = np.array([.12*t, -.03*t, 2.+.2*t])
            point = truth+rng.normal(0, .08, 3)
            estimate = np.array(f.update(point, t))
            if i > 20:
                errors.append(np.sum((estimate-truth)**2))
                raw.append(np.sum((point-truth)**2))
            self.assertGreaterEqual(np.linalg.eigvalsh(f.covariance).min(), -1e-12)
        self.assertLess(np.mean(errors), np.mean(raw)*.7)
        np.testing.assert_allclose(f.state[3:], [.12, -.03, .2], atol=.3)

    def test_resets_jump_time_reversal_and_gap(self):
        f = DepthTrackFilter()
        for t, z in ((1., 2.), (1.1, 4.), (.5, 3.), (4., 5.)):
            self.assertEqual(f.update((0., 0., z), t), (0., 0., z))
            np.testing.assert_array_equal(f.state[3:], 0.)

    def test_prediction_is_bounded_and_never_refreshes_measurement(self):
        f = DepthTrackFilter()
        for i in range(30):
            f.update((i*.01, 0., 2.), i*.05)
        before = f.state.copy()
        stamp = f.stamp
        predicted, age = f.hold(stamp+.2, .25)
        self.assertGreater(predicted[0], before[0])
        self.assertAlmostEqual(age, .2)
        np.testing.assert_array_equal(f.state, before)
        self.assertEqual(f.stamp, stamp)
        self.assertIsNone(f.hold(stamp+.26, .25))
        self.assertIsNone(f.update((0., 0., 2.), float('nan')))
        self.assertEqual(f.stamp, stamp)

    def test_invalid_parameters(self):
        for key in ('measurement_std_m', 'acceleration_std_mps2', 'jump_reset_m', 'reset_gap_s'):
            for value in (0., -1., float('nan'), float('inf')):
                with self.assertRaises(ValueError):
                    DepthTrackFilter(**{key: value})

    def test_temporal_filter_does_not_turn_invalid_into_position(self):
        f = DepthTrackFilter()
        self.assertIsNone(f.update(None, 1.))
        self.assertEqual(f.update((0., 0., 2.), 2.), (0., 0., 2.))
        self.assertIsNone(f.update(None, 3.))

    def test_temporal_filter_has_bounded_position_hold(self):
        f = DepthTrackFilter()
        f.update((0., 0., 2.), 10.)
        self.assertEqual(f.hold(10.2, .25)[0], (0., 0., 2.))
        self.assertIsNone(f.hold(10.3, .25))

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
