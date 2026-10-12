"""Bridge/arbiter rules with a scripted tracker; no model, not tracking accuracy."""
from types import SimpleNamespace as NS
import unittest
import numpy as np
from yolo_person_tracker.sot import (ArbiterConfig, BridgeArbiter, BridgeConfig, TargetBridge, VitTrack,
                                     crop_image, hann2d, overlaps_anyone)

INTR = (500., 500., 320., 240.)


class Scripted:
    """Stands in for VitTrack: returns queued (x, y, w, h), score."""
    def __init__(self):
        self.queue = []; self.inits = []

    def init(self, image, box):
        self.inits.append(box)

    def update(self, image):
        return self.queue.pop(0)


def depth_with(*people):
    """people: (x1, x2, z) vertical slabs over the full height."""
    d = np.zeros((480, 640), np.float32)
    for x1, x2, z in people:
        d[:, x1:x2] = z
    return d


def det(track, box):
    return NS(track_id=track, box=box)


class CropTests(unittest.TestCase):
    def test_crop_pads_outside_image_and_keeps_square_size(self):
        img = np.full((100, 100, 3), 7, np.uint8)
        crop = crop_image(img, (-10, -10, 30, 30), 2)
        self.assertEqual(crop.shape[:2], (60, 60))
        self.assertTrue((crop[0, 0] == 0).all() and (crop[40, 40] == 7).all())

    def test_hanning_window_symmetric_and_peaked_in_centre(self):
        w = hann2d()
        np.testing.assert_allclose(w, w.T, atol=1e-6); np.testing.assert_allclose(w, w[::-1, ::-1], atol=1e-6)
        self.assertGreater(w[8, 8], w[0, 0])

    def test_model_must_be_local_and_hashed(self):
        with self.assertRaises(ValueError):
            VitTrack('/nonexistent/vittrack.onnx')


class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.tracker = Scripted(); self.bridge = TargetBridge(self.tracker, BridgeConfig(max_bridge_s=1.))
        self.img = np.zeros((480, 640, 3), np.uint8)

    def test_bridges_until_score_drops(self):
        self.bridge.refresh(self.img, None, INTR, (100, 50, 200, 400), 1.)
        self.tracker.queue = [((102, 50, 100, 350), .8), ((104, 50, 100, 350), .2)]
        self.assertTrue(self.bridge.step(self.img, None, INTR, 1.1).valid)
        r = self.bridge.step(self.img, None, INTR, 1.2)
        self.assertFalse(r.valid); self.assertIn('score', r.reason); self.assertFalse(self.bridge.active)

    def test_bridge_age_is_limited_since_last_clean_refresh(self):
        self.bridge.refresh(self.img, None, INTR, (100, 50, 200, 400), 1.)
        self.tracker.queue = [((100, 50, 100, 350), .9)]*3
        self.assertTrue(self.bridge.step(self.img, None, INTR, 1.9).valid)
        self.assertIn('too long', self.bridge.step(self.img, None, INTR, 2.1).reason)

    def test_depth_contradiction_stops_after_consecutive_rejects(self):
        far = depth_with((100, 200, 3.)); near = depth_with((100, 200, 3.), (300, 400, 1.5))
        self.bridge.refresh(self.img, far, INTR, (100, 50, 200, 400), 1.)
        self.tracker.queue = [((300, 50, 100, 350), .9)]*2  # box jumps onto a nearer person
        self.assertTrue(self.bridge.step(self.img, near, INTR, 1.05).valid)
        r = self.bridge.step(self.img, near, INTR, 1.1)
        self.assertFalse(r.valid); self.assertIn('depth', r.reason)

    def test_refresh_in_same_frame_as_step_keeps_bridge(self):
        self.bridge.refresh(self.img, None, INTR, (100, 50, 200, 400), 1.)
        self.tracker.queue = [((100, 50, 100, 350), .9)]
        self.bridge.step(self.img, None, INTR, 1.1)
        self.assertTrue(self.bridge.refresh(self.img, None, INTR, (100, 50, 200, 400), 1.1).valid)
        self.assertTrue(self.bridge.active)
        self.assertFalse(self.bridge.refresh(self.img, None, INTR, (100, 50, 200, 400), 1.).valid)


class ArbiterTests(unittest.TestCase):
    def setUp(self):
        self.tracker = Scripted()
        self.arb = BridgeArbiter(TargetBridge(self.tracker), ArbiterConfig(conflict_frames=3, handover_frames=2))
        self.img = np.zeros((480, 640, 3), np.uint8); self.t = 1.

    def frame(self, target, cands, bridged=None):
        self.t += .1
        if bridged is not None:
            x1, y1, x2, y2 = bridged; self.tracker.queue.append(((x1, y1, x2-x1, y2-y1), .9))
        return self.arb.update(self.img, None, INTR, self.t, target, cands)

    def start(self):
        a = det(4, (100, 50, 200, 400)); b = det(7, (400, 50, 500, 400))
        self.assertEqual(self.frame(a, [a, b]).reason, 'bridge started')
        return a, b

    def test_agreement_refreshes_template_only_when_clean(self):
        a, b = self.start()
        self.assertEqual(self.frame(a, [a, b], a.box).action, 'agree'); self.assertEqual(len(self.tracker.inits), 2)
        touching = det(7, (170, 50, 270, 400))
        self.assertEqual(self.frame(a, [a, touching], a.box).action, 'agree'); self.assertEqual(len(self.tracker.inits), 2)

    def test_swap_is_reported_as_lost_never_as_a_switch(self):
        a, b = self.start()
        swapped_a, swapped_b = det(4, b.box), det(7, a.box)  # ByteTrack exchanged the numbers
        for i in range(3):
            d = self.frame(swapped_a, [swapped_a, swapped_b], a.box)
        self.assertEqual(d.action, 'lost'); self.assertFalse(self.arb.bridge.active)

    def test_reacquisition_only_onto_bridge_confirmed_track(self):
        a, b = self.start()
        self.assertEqual(self.frame(None, [b], (102, 50, 202, 400)).action, 'fill')
        new = det(12, (104, 50, 204, 400))
        self.assertEqual(self.frame(None, [b, new], (104, 50, 204, 400)).action, 'fill')
        d = self.frame(None, [b, new], (104, 50, 204, 400))
        self.assertEqual((d.action, d.track_id), ('handover', 12))

    def test_never_hands_over_to_a_track_seen_beside_the_target(self):
        a, b = self.start()
        d = self.frame(None, [b], b.box)  # target vanishes behind b; bridge slides onto b
        self.assertEqual((d.action, d.box), ('conflict', b.box))
        self.frame(None, [b], b.box)
        self.assertEqual(self.frame(None, [b], b.box).action, 'lost')

    def test_unwatched_overlap_taints_the_track(self):
        a, b = self.start()
        self.arb.bridge.stop('test')
        overlapped = det(7, (150, 50, 250, 400))
        self.assertEqual(self.frame(a, [a, overlapped]).action, 'none')
        self.assertEqual(self.frame(a, [a, b]).action, 'lost')
        self.assertEqual(self.frame(a, [a, b]).reason, 'bridge started')  # after an explicit relock

    def test_overlap_rule_matches_reid_crop_gate(self):
        self.assertFalse(overlaps_anyone((0, 0, 100, 100), [(95, 0, 195, 100)]))
        self.assertTrue(overlaps_anyone((0, 0, 100, 100), [(85, 0, 185, 100)]))


if __name__ == '__main__':
    unittest.main()


class MaskedRefreshTests(unittest.TestCase):
    def person(self, track, x):
        kp = [(x+20., 20., .9)]*17
        for i, (dx, dy) in {5: (5, 30), 6: (35, 30), 11: (8, 70), 12: (32, 70)}.items():
            kp[i] = (x+dx, dy, .9)
        return NS(track_id=track, box=(x, 5., x+40., 95.), keypoints=tuple(kp))

    def test_overlapped_target_refreshed_from_masked_image_only_when_enabled(self):
        img = np.full((100, 120, 3), 200, np.uint8)
        for enabled, expected in ((False, 1), (True, 2)):
            tracker = Scripted(); arb = BridgeArbiter(TargetBridge(tracker), ArbiterConfig(mask_refresh=enabled))
            a, b = self.person(4, 10.), self.person(7, 70.)
            arb.update(img, None, INTR, 1., a, [a, b])  # clean start
            near = self.person(7, 40.)  # neighbour overlaps the target's right side
            tracker.queue.append(((10, 5, 40, 90), .9))
            self.assertEqual(arb.update(img, None, INTR, 1.1, a, [a, near]).action, 'agree')
            self.assertEqual(len(tracker.inits), expected)
