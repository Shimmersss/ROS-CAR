import unittest
from yolo_person_tracker.backend import Detection
from yolo_person_tracker.state import Selection


class SelectionTest(unittest.TestCase):
    def test_lock_loss_and_release(self):
        s = Selection()
        a, b = Detection(1,(0,0,20,90),.9), Detection(2,(40,0,60,90),.8)
        s.update([a,b], 100, 10)
        self.assertTrue(s.lock(now=10.1)[0])
        self.assertEqual(s.selected(), b)
        s.update([a], 100, 10.2)
        self.assertIsNone(s.selected())
        self.assertEqual(s.target_id, (0,2))
        s.update([b], 100, 10.3)
        self.assertEqual(s.selected(), b)
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
