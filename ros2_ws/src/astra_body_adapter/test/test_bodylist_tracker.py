import math
from types import SimpleNamespace
import unittest

from astra_body_adapter.bodylist_node import BodylistTracker, is_akimbo


def vector(x=0.0, y=0.0, z=0.0):
    return SimpleNamespace(x=x, y=y, z=z)


def body(body_id, center=(0.0, 0.0, 2000.0), akimbo=False):
    joints = [SimpleNamespace(worldposition=vector()) for _ in range(19)]
    if akimbo:
        joints[9].worldposition = vector(0.0, 0.0, 0.0)
        joints[4].worldposition = vector(-300.0, 200.0, 2000.0)
        joints[2].worldposition = vector(-300.0, 300.0, 2000.0)
        joints[7].worldposition = vector(300.0, 200.0, 2000.0)
        joints[5].worldposition = vector(300.0, 300.0, 2000.0)
    return SimpleNamespace(
        bodyid=body_id,
        centerofmass=vector(*center),
        joints=joints,
    )


class BodylistTrackerTest(unittest.TestCase):
    def test_waits_for_explicit_lock_gesture(self):
        tracker = BodylistTracker()
        result = tracker.process([body(7)])
        self.assertEqual('SEARCHING', result.status)
        self.assertFalse(result.position_valid)
        self.assertEqual('', result.target_id)

    def test_gesture_locks_and_converts_sdk_mm_to_optical_metres(self):
        target = body(7, center=(300.0, 400.0, 2000.0), akimbo=True)
        self.assertTrue(is_akimbo(target))
        result = BodylistTracker().process([target])
        self.assertEqual('TRACKING', result.status)
        self.assertEqual('7', result.target_id)
        self.assertTrue(result.position_valid)
        self.assertAlmostEqual(0.3, result.x_m)
        self.assertAlmostEqual(-0.4, result.y_m)
        self.assertAlmostEqual(2.0, result.z_m)

    def test_locked_id_is_lost_without_silent_reassignment(self):
        tracker = BodylistTracker()
        tracker.process([body(7, akimbo=True)])
        result = tracker.process([body(8)])
        self.assertEqual('LOST', result.status)
        self.assertEqual('7', result.target_id)
        self.assertFalse(result.position_valid)

    def test_new_explicit_gesture_can_switch_target(self):
        tracker = BodylistTracker()
        tracker.process([body(7, akimbo=True)])
        result = tracker.process([body(8, akimbo=True)])
        self.assertEqual('TRACKING', result.status)
        self.assertEqual('8', result.target_id)

    def test_invalid_center_is_not_a_valid_observation(self):
        result = BodylistTracker().process([
            body(7, center=(0.0, 0.0, math.nan), akimbo=True)
        ])
        self.assertEqual('TRACKING', result.status)
        self.assertFalse(result.position_valid)
        self.assertTrue(math.isnan(result.z_m))


if __name__ == '__main__':
    unittest.main()
