import math
import unittest
import numpy as np
from yolo_person_tracker.height_fall import HeightFallConfig, HeightFallTracker, body_heights

K = (500., 500., 320., 240.)
UP = np.array([0., -1., 0.])      # level camera
CAM_H = .6


def person_depth(top, bottom, z=2., half_width=.25, background=4.):
    """Depth image of a fronto-parallel slab from `bottom` to `top` metres above the floor."""
    depth = np.full((480, 640), background, np.float32)
    v, u = np.mgrid[0:480, 0:640]
    y = (v-K[3])*z/K[1]                       # optical y (down) on the slab
    x = (u-K[2])*z/K[0]
    height = -y+CAM_H
    depth[(abs(x) <= half_width) & (height >= bottom) & (height <= top)] = z
    return depth


def box_of(top, bottom, z=2., half_width=.3):
    v_top = K[3]-(top-CAM_H)*K[1]/z
    v_bottom = K[3]-(bottom-CAM_H)*K[1]/z
    return (K[2]-half_width*K[0]/z, max(0, v_top-5), K[2]+half_width*K[0]/z, min(479, v_bottom+5))


class BodyHeightTests(unittest.TestCase):
    def test_standing_and_lying_slabs(self):
        top, median = body_heights(person_depth(1.2, 0.), box_of(1.2, 0.), K, UP, CAM_H)
        self.assertAlmostEqual(top, 1.14, delta=.03)
        self.assertAlmostEqual(median, .6, delta=.03)
        top, _ = body_heights(person_depth(.35, 0.), box_of(.35, 0.), K, UP, CAM_H)
        self.assertLess(top, .4)

    def test_extent_and_missing_depth(self):
        depth = person_depth(.3, 0., half_width=.8)
        *_, extent = body_heights(depth, box_of(.3, 0., half_width=.85), K, UP, CAM_H, extent=True)
        self.assertGreater(extent, 1.)          # 1.6 m slab minus box margins and 5-95 % tails
        self.assertIsNone(body_heights(np.zeros((480, 640), np.float32), box_of(1.2, 0.), K, UP, CAM_H))
        self.assertIsNone(body_heights(person_depth(1.2, 0.), (300, 200, 301, 201), K, UP, CAM_H))


class HeightFallTrackerTests(unittest.TestCase):
    def feed(self, tracker, samples, ident='0:1', hint=False):
        return [tracker.update(ident, t, h, upright_hint=hint) for t, h in samples]

    def test_drop_confirm_and_recovery(self):
        t = HeightFallTracker()
        self.feed(t, [(1., 1.), (1.2, 1.), (1.4, 1.)])
        out = self.feed(t, [(1.8, .7), (2.1, .3), (2.5, .3), (2.9, .3), (3.2, .3)])
        self.assertEqual([p for p, _ in out], [0, 1, 1, 1, 2])
        out = self.feed(t, [(4., 1.), (5., 1.), (6.1, 1.)])
        self.assertEqual([p for p, _ in out], [2, 2, 0])

    def test_static_low_and_slow_descent_do_not_alarm(self):
        t = HeightFallTracker()
        self.assertEqual(self.feed(t, [(1., .3), (1.5, .3), (2., .3)])[-1][0], 0)
        t = HeightFallTracker()
        samples = [(1., 1.), (1.3, 1.), (1.6, 1.)]+[(1.6+.8*i, 1.-.1*i) for i in range(1, 9)]
        self.assertTrue(all(p == 0 for p, _ in self.feed(t, samples)))       # last upright to low > transition_s

    def test_crouch_veto_and_rising_cancel(self):
        t = HeightFallTracker()
        self.feed(t, [(1., 1.), (1.2, 1.), (1.4, 1.)])
        self.assertEqual(self.feed(t, [(1.8, .4)], hint=True)[0][0], 0)
        t = HeightFallTracker()
        self.feed(t, [(1., 1.), (1.2, 1.), (1.4, 1.)])
        self.assertEqual(self.feed(t, [(1.8, .4), (2.0, .7)])[1][0], 0)

    def test_gap_pauses_timers(self):
        t = HeightFallTracker()
        self.feed(t, [(1., 1.), (1.2, 1.), (1.4, 1.), (1.8, .3)])
        self.assertEqual(self.feed(t, [(2.1, None), (2.4, None)])[-1][0], 1)
        # 0.6 s unmeasured does not count: confirmation needs 1 s of measured low time
        self.assertEqual(self.feed(t, [(2.6, .3)])[0][0], 1)
        self.assertEqual(self.feed(t, [(3.5, .3)])[0][0], 2)

    def test_handover_carries_history(self):
        t = HeightFallTracker()
        boxes = {'0:1': (50., 10., 150., 210.)}
        self.feed(t, [(1., 1.), (1.2, 1.), (1.4, 1.)])
        moved = t.handover(1.8, {'0:7': (20., 130., 240., 220.)}, boxes)
        self.assertEqual(moved, {'0:7': '0:1'})
        self.assertEqual(self.feed(t, [(1.8, .3)], ident='0:7')[0][0], 1)

    def test_config_validation(self):
        with self.assertRaises(ValueError): HeightFallConfig(low_max_m=1.)
        with self.assertRaises(ValueError): HeightFallConfig(confirm_s=0.)
        with self.assertRaises(ValueError): HeightFallConfig(top_percentile=120.)


if __name__ == '__main__':
    unittest.main()
