import unittest
import numpy as np
from yolo_person_tracker.depth import DepthGate, DepthTrackFilter, measure


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


class DepthGateTest(unittest.TestCase):
    def walk(self, f, rate, seconds=3., speed=1.5, seed=1):
        rng = np.random.default_rng(seed)
        rejected = 0
        for i in range(int(seconds*rate)):
            t = i/rate
            f.update((0., 0., 2.+speed*t+rng.normal(0, .03)), t)
            rejected += f.last_rejected
        return rejected

    def test_normal_walking_is_never_gated_at_5_or_30_hz(self):
        for rate in (5., 30.):
            self.assertEqual(self.walk(DepthTrackFilter(gate=DepthGate()), rate), 0)

    def test_single_outlier_is_rejected_and_never_becomes_reference(self):
        f = DepthTrackFilter(gate=DepthGate())
        for i in range(10):
            f.update((0., 0., 2.9), i*.2)
        stamp = f.stamp
        self.assertIsNone(f.update((0., 0., 1.8), 2.))
        self.assertTrue(f.last_rejected)
        self.assertEqual(f.stamp, stamp)
        held, age = f.hold(2., .25)
        self.assertAlmostEqual(held[2], 2.9, places=2)
        self.assertAlmostEqual(age, .2)
        # The following real measurement is judged against the held state, not the outlier.
        self.assertAlmostEqual(f.update((0., 0., 2.92), 2.2)[2], 2.9, delta=.05)
        self.assertFalse(f.last_rejected)
        self.assertIsNone(f.pending)

    def test_consistent_relocation_is_confirmed_inconsistent_is_not(self):
        f = DepthTrackFilter(gate=DepthGate(), reset_gap_s=5.)
        f.update((0., 0., 2.), 0.)
        times = [.033*i for i in range(1, 12)]
        results = [f.update((0., 0., 3.), t) for t in times]
        accepted = [t for t, r in zip(times, results) if r is not None]
        self.assertTrue(accepted)
        self.assertGreaterEqual(accepted[0], .3)
        # Relocation is a clean re-initialization, not a blend that invents velocity.
        self.assertTrue(all(r[2] == 3. for r in results if r is not None))
        self.assertLess(abs(f.state[5]), .1)
        self.assertAlmostEqual(f.update((0., 0., 3.), 1.)[2], 3., delta=.02)
        g = DepthTrackFilter(gate=DepthGate(), reset_gap_s=5.)
        g.update((0., 0., 2.), 0.)
        for i, z in enumerate((3., 4., 3., 4., 3., 4., 3., 4., 3., 4., 3., 4.)):
            self.assertIsNone(g.update((0., 0., z), .033*(i+1)))
        self.assertEqual(g.state[2], 2.)

    def test_sustained_rejection_beyond_default_gap_never_bypasses_gate(self):
        # Default node timeout (max_age_s=0.5): 2 m accepted, then 3/4 m alternate at 30 Hz.
        f = DepthTrackFilter(gate=DepthGate())
        f.update((0., 0., 2.), 0.)
        for i in range(1, 91):
            self.assertIsNone(f.update((0., 0., 3. if i % 2 else 4.), i/30.), i)
            self.assertTrue(f.last_rejected)
        # The stale state is dropped, not predicted forward or held.
        self.assertIsNone(f.stamp)
        self.assertIsNone(f.hold(3., .25))
        self.assertEqual(f.evidence_stamp, 3.)
        # Even a torso-backed sample at the old position now needs confirmation.
        self.assertIsNone(f.update((0., 0., 2.), 3.+1/30.))
        times = [3.+i/30. for i in range(2, 16)]
        results = [f.update((0., 0., 2.5), t) for t in times]
        accepted = [t for t, r in zip(times, results) if r is not None]
        self.assertTrue(accepted)
        self.assertGreaterEqual(accepted[0]-times[0], .3-1e-9)
        self.assertTrue(all(r == (0., 0., 2.5) for r in results if r is not None))

    def test_consistent_relocation_with_default_gap(self):
        f = DepthTrackFilter(gate=DepthGate())
        f.update((0., 0., 2.), 0.)
        results = [f.update((0., 0., 3.), i/30.) for i in range(1, 16)]
        first = next(i for i, r in enumerate(results, 1) if r is not None)
        self.assertGreaterEqual(first/30.-1/30., .3-1e-9)
        self.assertLess(first/30., .5)
        self.assertEqual(results[first-1], (0., 0., 3.))
        self.assertLess(abs(f.state[5]), .1)

    def test_real_input_gap_still_restarts_directly(self):
        f = DepthTrackFilter(gate=DepthGate())
        f.update((0., 0., 2.), 0.)
        self.assertIsNone(f.update((0., 0., 4.), .1))
        # No input of any kind for > reset_gap: a torso-backed sample starts at once.
        self.assertEqual(f.update((0., 0., 4.), .7), (0., 0., 4.))
        self.assertIsNone(f.update((0., 0., 6.), 1.4, fallback=True))
        self.assertIsNone(f.stamp)

    def test_gap_and_time_reversal_still_reinitialize(self):
        f = DepthTrackFilter(gate=DepthGate())
        f.update((0., 0., 2.), 1.)
        self.assertEqual(f.update((0., 0., 4.), .9), (0., 0., 4.))
        self.assertEqual(f.update((0., 0., 6.), 1.6), (0., 0., 6.))
        self.assertFalse(f.last_rejected)

    def test_fallback_is_gated_tighter_and_weighted_less(self):
        def settled():
            f = DepthTrackFilter(gate=DepthGate())
            for i in range(20):
                f.update((0., 0., 2.), i*.033)
            return f
        t = 20*.033
        self.assertIsNotNone(settled().update((0., 0., 2.3), t))
        fallback = settled()
        self.assertIsNone(fallback.update((0., 0., 2.3), t, fallback=True))
        self.assertTrue(fallback.last_rejected)
        normal = settled().update((0., 0., 2.15), t)[2]
        weak = settled().update((0., 0., 2.15), t, fallback=True)[2]
        self.assertLess(weak-2., normal-2.)

    def test_fallback_alone_cannot_start_a_track_until_confirmed(self):
        f = DepthTrackFilter(gate=DepthGate())
        self.assertIsNone(f.update((0., 0., 2.16), 1., fallback=True))
        self.assertTrue(f.last_rejected)
        self.assertIsNone(f.stamp)
        self.assertEqual(f.evidence_stamp, 1.)
        self.assertIsNone(f.hold(1.1, .25))
        # A torso-backed measurement starts the track immediately and drops the pending fallback.
        self.assertEqual(f.update((0., 0., 1.52), 1.2), (0., 0., 1.52))
        self.assertIsNone(f.pending)
        g = DepthTrackFilter(gate=DepthGate())
        results = [g.update((0., 0., 1.5), 1.+i*.033, fallback=True) for i in range(12)]
        self.assertTrue(all(r is None for r in results[:9]))
        self.assertEqual(results[10], (0., 0., 1.5))
        # A stale track is dropped rather than kept when only a fallback reappears.
        self.assertIsNone(g.update((0., 0., 3.), 3., fallback=True))
        self.assertIsNone(g.stamp)

    def test_legacy_without_gate_ignores_fallback_flag(self):
        f = DepthTrackFilter()
        f.update((0., 0., 2.), 1.)
        self.assertEqual(f.update((0., 0., 4.), 1.1, fallback=True), (0., 0., 4.))
        self.assertFalse(f.last_rejected)

    def test_invalid_gate(self):
        for key in ('base_m', 'speed_mps', 'confirm_s', 'fallback_scale', 'fallback_noise_scale'):
            for value in (0., -1., float('nan'), float('inf')):
                with self.assertRaises(ValueError):
                    DepthGate(**{key: value})
        with self.assertRaises(ValueError):
            DepthGate(fallback_scale=1.5)
        with self.assertRaises(ValueError):
            DepthGate(fallback_noise_scale=.5)
        with self.assertRaises(ValueError):
            DepthTrackFilter(gate=object())
