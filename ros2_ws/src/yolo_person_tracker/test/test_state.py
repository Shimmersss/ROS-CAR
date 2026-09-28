import unittest
from yolo_person_tracker.backend import Detection
from yolo_person_tracker.state import Selection


class SelectionTest(unittest.TestCase):
    def test_lock_loss_hold_and_release(self):
        s = Selection(hold_s=.35)
        a, b = Detection(1,(0,0,20,90),.9), Detection(2,(40,0,60,90),.8)
        s.update([a,b], 100, 10)
        self.assertTrue(s.lock(now=10.1)[0])
        self.assertEqual(s.selected(), b)
        s.update([a], 100, 10.2)
        self.assertIsNotNone(s.selected(now=10.3))
        self.assertEqual(s.target_id, (0,2))
        s.update([b], 100, 10.3)
        self.assertEqual(s.selected(), b)
        s.update([a], 100, 10.7)
        self.assertIsNone(s.selected(now=10.71))
        s.reset_stream()
        s.update([b], 100, 11)
        self.assertIsNone(s.selected())
        self.assertTrue(s.lock(now=11.1)[0])
        self.assertEqual(s.target_id, (1,2))
        s.release()
        self.assertIsNone(s.selected())

    def test_stale_and_empty(self):
        s = Selection()
        self.assertFalse(s.lock(now=1)[0])
        s.update([Detection(1,(0,0,20,90),.9)],100,1)
        self.assertFalse(s.lock(now=2)[0])


class UntrackedSelectionTest(unittest.TestCase):
    def test_untracked_cannot_lock(self):
        selection = Selection()
        selection.update([Detection(None, (10, 0, 90, 100), .8)], 100, 1.)
        self.assertFalse(selection.lock(now=1.)[0])
        self.assertIsNone(selection.selected())

    def test_auto_requires_stable_single_and_reacquires_changed_track(self):
        selection = Selection()
        person = Detection(1, (10, 0, 90, 100), .8)
        for index in range(3):
            selection.update([person], 100, float(index))
            self.assertEqual(selection.auto_lock_single(now=float(index)), index == 2)
        self.assertEqual(selection.target_id, (0, 1))
        replacement = Detection(2, (10, 0, 90, 100), .8)
        for index in range(3, 6):
            selection.update([replacement], 100, float(index))
            self.assertEqual(selection.auto_lock_single(now=float(index)), index == 5)
        self.assertEqual(selection.target_id, (0, 2))

    def test_auto_never_selects_among_multiple_or_stale_people(self):
        selection = Selection()
        tracked = Detection(1, (10, 0, 40, 100), .8)
        untracked = Detection(None, (50, 0, 90, 100), .8)
        for index in range(4):
            selection.update([tracked, untracked], 100, float(index))
            self.assertFalse(selection.auto_lock_single(now=float(index)))
        selection.update([tracked], 100, 5.)
        self.assertFalse(selection.auto_lock_single(now=6.))
        self.assertIsNone(selection.target_id)

    def test_reacquires_changed_track_near_last_box(self):
        selection = Selection(hold_s=.2, reacquire_s=.8, reacquire_center_fraction=.25)
        first = Detection(1, (40, 20, 80, 120), .9)
        selection.update([first], 200, 1.)
        self.assertTrue(selection.lock(now=1.01)[0])
        replacement = Detection(7, (44, 22, 84, 122), .8)
        selection.update([replacement], 200, 1.2)
        self.assertEqual(selection.target_id, (0, 7))
        self.assertIs(selection.selected(), replacement)

    def test_reacquire_rejects_far_person(self):
        selection = Selection(reacquire_s=.8)
        first = Detection(1, (40, 20, 80, 120), .9)
        selection.update([first], 200, 1.)
        self.assertTrue(selection.lock(now=1.01)[0])
        selection.update([Detection(7, (150, 20, 190, 120), .8)], 200, 1.2)
        self.assertEqual(selection.target_id, (0, 1))
