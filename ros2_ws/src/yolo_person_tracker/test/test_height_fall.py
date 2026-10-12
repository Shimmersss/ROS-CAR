import math
import unittest
import numpy as np
from yolo_person_tracker.height_fall import (HeightFallConfig, HeightFallTracker, at_side_edge, body_heights,
                                             clipped_at_top, skeleton_veto)

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

    def test_head_estimate_is_display_only_and_higher(self):
        depth, box = person_depth(1.2, 0.), box_of(1.2, 0.)
        top, median, head = body_heights(depth, box, K, UP, CAM_H, head=True)
        self.assertEqual((top, median), body_heights(depth, box, K, UP, CAM_H))   # rules unchanged
        self.assertGreaterEqual(head, top)
        *_, extent, head2 = body_heights(depth, box, K, UP, CAM_H, extent=True, head=True)
        self.assertEqual(head2, head)


class HeightFallTrackerTests(unittest.TestCase):
    def feed(self, tracker, samples, ident='0:1', hint=False):
        return [tracker.update(ident, t, h, upright_hint=hint) for t, h in samples]

    def test_drop_confirm_and_recovery(self):
        t = HeightFallTracker(HeightFallConfig(recovery_s=2.))
        self.feed(t, [(1., 1.), (1.2, 1.), (1.4, 1.)])
        out = self.feed(t, [(1.8, .7), (2.1, .3), (2.5, .3), (2.9, .3), (3.2, .3)])
        self.assertEqual([p for p, _ in out], [0, 1, 1, 1, 2])
        out = self.feed(t, [(4., 1.), (5., 1.), (6.1, 1.)])
        self.assertEqual([p for p, _ in out], [2, 2, 0])

    def test_default_recovery_allows_a_repeat_fall(self):
        # 2026-10-09: a second fall 1.6 s after standing up was lost to a 2 s recovery.
        t = HeightFallTracker()
        self.feed(t, [(1., 1.), (1.2, 1.), (1.4, 1.), (1.8, .3), (2.2, .3), (2.9, .3)])
        out = self.feed(t, [(4., 1.), (4.4, 1.), (5.1, 1.), (5.3, 1.), (5.5, 1.), (5.7, 1.), (6., .3)])
        self.assertEqual([p for p, _ in out], [2, 2, 0, 0, 0, 0, 1])

    def test_skeleton_veto_needs_a_tall_box(self):
        self.assertTrue(skeleton_veto(True, (100, 100, 200, 300)))     # crouch: 2:1 box
        self.assertFalse(skeleton_veto(True, (100, 200, 300, 330)))    # lying called standing: wide box
        self.assertFalse(skeleton_veto(False, (100, 100, 200, 300)))
        self.assertTrue(skeleton_veto(True, (100, 200, 300, 330), HeightFallConfig(veto_min_aspect=.5)))

    def test_standing_skeleton_gives_baseline_and_recovery(self):
        # 2026-10-09: a far standing person's top stayed ~0.7 m (head beyond the depth view).
        def feed(t, samples, standing):
            return [t.update('0:1', ts, h, posture_upright=standing)[0] for ts, h in samples]
        t = HeightFallTracker()
        feed(t, [(1., .72), (1.2, .72), (1.4, .72)], False)
        self.assertEqual(feed(t, [(1.8, .3)], False), [0])                    # no skeleton: no baseline
        t = HeightFallTracker()
        feed(t, [(1., .72), (1.2, .72), (1.4, .72)], True)
        self.assertEqual(feed(t, [(1.8, .3), (2.3, .3), (2.9, .3)], False), [1, 1, 2])
        self.assertEqual(feed(t, [(3.5, .7), (4.6, .7)], False)[-1], 2)       # no skeleton: still fallen
        self.assertEqual(feed(t, [(5., .7), (6.1, .7)], True)[-1], 0)
        t = HeightFallTracker()
        feed(t, [(1., .6), (1.2, .6), (1.4, .6)], True)                       # below hint_upright_min_m
        self.assertEqual(feed(t, [(1.8, .2)], False), [0])

    def test_unmeasured_standing_skeleton_recovers(self):
        t = HeightFallTracker()
        self.feed(t, [(1., 1.), (1.2, 1.), (1.4, 1.), (1.8, .3), (2.3, .3), (2.9, .3)])
        out = [t.update('0:1', 3.+.25*k, None, upright_hint=True, posture_upright=True)[0] for k in range(6)]
        self.assertEqual(out, [2, 2, 2, 2, 0, 0])            # 1 s default recovery
        t = HeightFallTracker()                                  # without the skeleton the alarm stays
        self.feed(t, [(1., 1.), (1.2, 1.), (1.4, 1.), (1.8, .3), (2.3, .3), (2.9, .3)])
        self.assertEqual([t.update('0:1', 3.+.25*k, None)[0] for k in range(6)], [2]*6)

    def test_clipped_top_proves_upright_but_not_a_drop(self):
        self.assertTrue(clipped_at_top((10, 5, 200, 400)))
        self.assertFalse(clipped_at_top((10, 40, 200, 400)))
        t = HeightFallTracker()
        for k in range(3):
            t.update('0:1', 1.+.2*k, 1., clipped=True)                 # high clipped tops: baseline
        self.assertEqual(t.update('0:1', 1.8, .4, clipped=True)[0], 0)  # low clipped top: unmeasured
        self.assertEqual(t.update('0:1', 1.9, .4)[0], 1)                # unclipped low top: drop

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

    def test_handover_gap_does_not_count_towards_confirmation(self):
        t = HeightFallTracker()
        boxes = {'0:1': (50., 10., 150., 210.)}
        self.feed(t, [(1., 1.), (1.2, 1.), (1.4, 1.), (1.8, .3)])           # pending from 1.8 s
        self.assertEqual(t.handover(2.6, {'0:7': (20., 130., 240., 220.)}, boxes), {'0:7': '0:1'})
        # 0.8 s with neither track seen: 1 s of measured low time ends at 3.6 s, not 2.8 s.
        self.assertEqual([p for p, _ in self.feed(t, [(2.6, .3), (3.0, .3), (3.5, .3)], ident='0:7')], [1, 1, 1])
        self.assertEqual(self.feed(t, [(3.65, .3)], ident='0:7')[0][0], 2)

    def test_handover_to_a_fresh_track_that_overlapped(self):
        # The new ID appears one frame before the old one is dropped (2026-10-10 A1a fall 2).
        t = HeightFallTracker()
        boxes = {'0:1': (50., 130., 250., 220.)}
        self.feed(t, [(1., 1.), (1.2, 1.), (1.4, 1.), (1.8, .3)])           # pending from 1.8 s
        t.handover(2.0, {'0:1': boxes['0:1'], '0:7': (40., 140., 260., 225.)}, boxes)
        self.feed(t, [(2.0, .3)])
        self.assertEqual(self.feed(t, [(2.0, .3)], ident='0:7')[0][1].split(';')[-1].strip(), 'static low, no observed drop')
        boxes['0:7'] = (40., 140., 260., 225.)
        self.assertEqual(t.handover(2.1, {'0:7': boxes['0:7']}, boxes), {'0:7': '0:1'})
        self.assertEqual(self.feed(t, [(2.1, .3), (2.5, .3)], ident='0:7')[-1][0], 1)
        self.assertEqual(self.feed(t, [(2.95, .3)], ident='0:7')[0][0], 2)   # 0.1 s unseen gap excluded

    def test_track_with_its_own_evidence_is_not_replaced(self):
        t = HeightFallTracker()
        boxes = {'0:1': (50., 130., 250., 220.), '0:7': (40., 10., 260., 225.)}
        self.feed(t, [(1., 1.), (1.2, 1.), (1.4, 1.)])
        self.feed(t, [(1.0, 1.6), (1.2, 1.6), (1.4, 1.6)], ident='0:7')     # standing: has a baseline
        self.assertEqual(t.handover(1.6, {'0:7': boxes['0:7']}, boxes), {})

    def test_lost_hold_hands_over_a_suspected_track_after_a_long_gap(self):
        boxes = {'0:1': (50., 130., 250., 220.)}
        def run(cfg):
            t = HeightFallTracker(cfg)
            self.feed(t, [(1., 1.), (1.2, 1.), (1.4, 1.), (1.8, .3)])       # suspected from 1.8 s, then lost
            moved = t.handover(6.8, {'0:9': (40., 140., 260., 225.)}, boxes, 1.5)
            return moved, [p for p, _ in self.feed(t, [(6.8, .3), (7.5, .3), (7.85, .3)], ident='0:9')]
        self.assertEqual(run(HeightFallConfig()), ({}, [0, 0, 0]))           # 5 s gap: static low only
        # Held: the 5 s unseen never counts; 1 s of measured low after reappearing confirms.
        self.assertEqual(run(HeightFallConfig(lost_hold_s=6.)), ({'0:9': '0:1'}, [1, 1, 2]))

    def test_lost_hold_keeps_the_same_identity_and_leaves_short_gaps_alone(self):
        t = HeightFallTracker(HeightFallConfig(lost_hold_s=6.))
        self.feed(t, [(1., 1.), (1.2, 1.), (1.4, 1.), (1.8, .3)])
        self.assertEqual([p for p, _ in self.feed(t, [(5.0, .3), (5.9, .3), (6.05, .3)])], [1, 1, 2])
        t = HeightFallTracker(HeightFallConfig(lost_hold_s=6.))
        self.feed(t, [(1., 1.), (1.2, 1.), (1.4, 1.), (1.8, .3)])
        self.assertEqual(self.feed(t, [(2.5, .3), (2.85, .3)])[-1][0], 2)  # 0.7 s gap still counts, as before

    def test_sit_hold_keeps_a_fall_through_sitting_up(self):
        drop = [(1., 1.5), (1.2, 1.5), (1.4, 1.5), (1.8, .35)]
        sit_then_lie = drop+[(2.2, .7), (3.0, .7), (3.6, .3), (4.0, .3), (4.65, .3)]
        self.assertEqual([p for p, _ in self.feed(HeightFallTracker(), sit_then_lie)][4:], [0, 0, 0, 0, 0])
        held = [p for p, _ in self.feed(HeightFallTracker(HeightFallConfig(sit_hold_s=3.)), sit_then_lie)]
        self.assertEqual(held[4:], [1, 1, 1, 1, 2])
        # Standing back up cancels; lying only after the window is static low.
        stood = drop+[(2.2, 1.4), (2.4, 1.4), (4.5, .3), (5.0, .3), (5.6, .3)]   # lying only after the baseline expired
        self.assertEqual([p for p, _ in self.feed(HeightFallTracker(HeightFallConfig(sit_hold_s=3.)), stood)][4:],
                         [0, 0, 0, 0, 0])
        late = drop+[(2.2, .7), (4.0, .7), (5.0, .3), (5.5, .3), (6.2, .3)]
        self.assertEqual(self.feed(HeightFallTracker(HeightFallConfig(sit_hold_s=3.)), late)[-1][0], 0)

    def test_side_edge_box_cannot_start_a_fall(self):
        cfg = HeightFallConfig(side_edge_px=8.)
        self.assertTrue(at_side_edge((4., 51., 213., 466.), 640, cfg))
        self.assertTrue(at_side_edge((300., 51., 636., 466.), 640, cfg))
        self.assertFalse(at_side_edge((4., 51., 213., 466.), 640, HeightFallConfig()))   # off by default
        t = HeightFallTracker(cfg)
        self.feed(t, [(1., 1.), (1.2, 1.), (1.4, 1.)])
        self.assertEqual(t.update('0:1', 1.6, .3, side=True)[0], 0)          # leg of someone walking out
        self.assertEqual(t.update('0:1', 1.7, .3)[0], 1)                     # same drop seen in full view
        self.assertEqual(t.update('0:1', 2.0, .3, side=True)[0], 1)          # an ongoing fall is kept

    def test_continuation_is_off_by_default(self):
        t = HeightFallTracker()
        self.feed(t, [(1., 1.3), (1.2, 1.3), (1.4, 1.3), (1.6, .3)])
        self.assertEqual(t.continuable(1.7, {}), [])

    def test_continuation_confirms_a_person_lost_mid_drop(self):
        # 2026-10-10 A3: the box vanished at a 0.56 m top and only a static low ID came back 3.4 s later.
        t = HeightFallTracker(HeightFallConfig(continue_s=6.))
        self.feed(t, [(1., 1.3), (1.2, 1.3), (1.4, 1.3), (1.6, .9), (1.7, .56)])
        self.assertEqual(t.continuable(1.8, {}), ['0:1'])
        self.assertEqual(t.update('0:1', 1.8, .5, continued=True)[0], 1)
        out = [t.update('0:1', 1.8+.1*i, .22, continued=True)[0] for i in range(1, 12)]
        self.assertEqual(out[-1], 2)
        self.assertEqual(t.tracks['0:1']['seen'], 1.7)                          # continuation is not a detection

    def test_continuation_only_counts_a_lying_body_top(self):
        t = HeightFallTracker(HeightFallConfig(continue_s=6.))
        self.feed(t, [(1., 1.3), (1.2, 1.3), (1.4, 1.3), (1.6, .3)])           # suspected
        for i, top in enumerate((1.6, .05, None, 1.6)):                       # wall behind, bare floor, no depth
            self.assertEqual(t.update('0:1', 1.7+.1*i, top, continued=True)[0], 1)
        self.assertIsNotNone(t.tracks['0:1']['pending'])                       # unmeasured: paused, not cancelled
        self.assertEqual(t.tracks['0:1']['top'], .3)

    def test_continuation_limits(self):
        t = HeightFallTracker(HeightFallConfig(continue_s=6.))
        self.feed(t, [(1., 1.3), (1.2, 1.3), (1.4, 1.3)], ident='up')         # lost while upright
        self.feed(t, [(1., 1.3), (1.2, 1.3), (1.4, 1.3), (1.6, .3)])
        self.assertEqual(t.continuable(1.7, {'0:2': (0, 0, 1, 1)}), ['0:1'])
        self.assertEqual(t.continuable(1.7, {'0:1': (0, 0, 1, 1)}), [])        # still detected
        for i in range(1, 5):                                                  # never a lying top: stops after 4 max_gap_s
            t.update('0:1', 1.6+.5*i, 2., continued=True)
        self.assertEqual(t.continuable(3.7, {}), [])
        t2 = HeightFallTracker(HeightFallConfig(continue_s=2.))
        self.feed(t2, [(1., 1.3), (1.2, 1.3), (1.4, 1.3), (1.6, .3)])
        for i in range(1, 21):
            t2.update('0:1', 1.6+.1*i, .3, continued=True)
        self.assertEqual(t2.continuable(3.7, {}), [])                          # continue_s after the last detection

    def test_continued_track_hands_over_to_the_returning_detection(self):
        t = HeightFallTracker(HeightFallConfig(continue_s=6.))
        old = (100., 100., 200., 400.)
        self.feed(t, [(1., 1.3), (1.2, 1.3), (1.4, 1.3), (1.6, .3)])
        for i in range(1, 15):
            t.update('0:1', 1.6+.1*i, .25, continued=True)
        self.assertTrue(t.tracks['0:1']['fallen'])
        moved = t.handover(3.1, {'0:9': (90., 300., 230., 410.)}, {'0:1': old})
        self.assertEqual(moved, {'0:9': '0:1'})
        self.assertTrue(t.tracks['0:9']['fallen'])

    def test_continuation_yields_to_a_detection_for_good(self):
        # 2026-10-09 A2a: the person got up under a new ID next to the continued lying identity.
        t = HeightFallTracker(HeightFallConfig(continue_s=6.))
        self.feed(t, [(1., 1.3), (1.2, 1.3), (1.4, 1.3), (1.6, .3)])
        lying = (100., 300., 300., 400.)
        self.assertFalse(t.yields('0:1', lying, {'0:5': (400., 100., 500., 400.)}))
        self.assertEqual(t.continuable(1.7, {'0:5': (400., 100., 500., 400.)}), ['0:1'])
        self.assertTrue(t.yields('0:1', lying, {'0:6': (150., 150., 260., 420.)}))   # sitting up in place
        self.assertEqual(t.continuable(1.8, {}), [])
        t.update('0:1', 1.9, .3)                                                   # detected again: may continue later
        self.assertEqual(t.continuable(2.0, {}), ['0:1'])

    def test_config_validation(self):
        with self.assertRaises(ValueError): HeightFallConfig(continue_min_m=.6)
        with self.assertRaises(ValueError): HeightFallConfig(continue_s=-1.)
        with self.assertRaises(ValueError): HeightFallConfig(low_max_m=1.)
        with self.assertRaises(ValueError): HeightFallConfig(confirm_s=0.)
        with self.assertRaises(ValueError): HeightFallConfig(top_percentile=120.)


if __name__ == '__main__':
    unittest.main()
