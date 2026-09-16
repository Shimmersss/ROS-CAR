import math
import unittest

from astra_body_adapter.follow_control import (
    FollowConfig,
    compute_command,
    target_is_usable,
)


class FollowControlTest(unittest.TestCase):
    def setUp(self):
        self.config = FollowConfig(target_distance_m=2.0, distance_deadband_m=0.15)

    def test_default_thirty_centimetre_trigger(self):
        for distance in (.25, .3):
            self.assertEqual(compute_command(distance, 0., True), (0., 0.))
        self.assertGreater(compute_command(.31, 0., True)[0], 0.)
        self.assertAlmostEqual(compute_command(.7, 0., True)[0], .15)

    def test_far_centered_target_moves_forward_at_limited_speed(self):
        linear, angular = compute_command(3.0, 0.0, True, self.config)
        self.assertAlmostEqual(0.15, linear)
        self.assertEqual(0.0, angular)

    def test_target_to_right_commands_negative_yaw(self):
        linear, angular = compute_command(2.0, 0.2, True, self.config)
        self.assertEqual(0.0, linear)
        self.assertAlmostEqual(-0.24, angular)

    def test_large_bearing_turns_in_place(self):
        linear, angular = compute_command(3.0, 0.7, True, self.config)
        self.assertEqual(0.0, linear)
        self.assertAlmostEqual(-0.5, angular)

    def test_close_target_never_commands_reverse(self):
        linear, angular = compute_command(1.0, 0.0, True, self.config)
        self.assertEqual(0.0, linear)
        self.assertEqual(0.0, angular)

    def test_invalid_target_stops(self):
        self.assertEqual(
            (0.0, 0.0),
            compute_command(3.0, 0.0, False, self.config),
        )

    def test_non_finite_target_stops(self):
        self.assertEqual(
            (0.0, 0.0),
            compute_command(math.nan, 0.0, True, self.config),
        )

    def test_non_positive_distance_stops(self):
        self.assertEqual(
            (0.0, 0.0),
            compute_command(0.0, 0.2, True, self.config),
        )

    def test_bearing_deadband_prevents_small_turns(self):
        linear, angular = compute_command(2.0, 0.05, True, self.config)
        self.assertEqual(0.0, linear)
        self.assertEqual(0.0, angular)

    def test_fresh_tracking_target_is_usable_when_enabled(self):
        self.assertTrue(target_is_usable(True, 2, 2, True, 0.2, 0.5))

    def test_disabled_follower_rejects_target(self):
        self.assertFalse(target_is_usable(False, 2, 2, True, 0.2, 0.5))

    def test_non_tracking_status_rejects_target(self):
        self.assertFalse(target_is_usable(True, 3, 2, True, 0.2, 0.5))

    def test_stale_target_is_rejected(self):
        self.assertFalse(target_is_usable(True, 2, 2, True, 0.6, 0.5))

    def test_negative_speed_limit_is_rejected(self):
        with self.assertRaises(ValueError):
            FollowConfig(max_linear_mps=-0.1)

    def test_turn_threshold_must_cover_bearing_deadband(self):
        with self.assertRaises(ValueError):
            FollowConfig(bearing_deadband_rad=0.7, turn_in_place_rad=0.6)


if __name__ == '__main__':
    unittest.main()
