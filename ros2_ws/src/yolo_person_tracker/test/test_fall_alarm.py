import unittest
from types import SimpleNamespace
from yolo_person_tracker.fall_alarm import summarize


def person(ident, stage):
    return SimpleNamespace(track_id=ident, fall_stage=stage, box=(0., 0., 1., 1.))


class FallAlarmTests(unittest.TestCase):
    def test_no_people_or_no_fall(self):
        out = summarize([], [])
        self.assertEqual((out.stage, out.person, out.suspected, out.confirmed), (0, None, 0, 0))
        out = summarize([person('0:1', 0)], [person('0:2', 0)])
        self.assertEqual((out.stage, out.person), (0, None))

    def test_highest_stage_and_counts(self):
        out = summarize([person('0:1', 1), person('0:3', 0)], [person('0:2', 2), person('0:4', 1)])
        self.assertEqual((out.stage, out.person.track_id, out.continued), (2, '0:2', True))
        self.assertEqual((out.suspected, out.confirmed), (2, 1))

    def test_tie_prefers_the_detected_person(self):
        out = summarize([person('0:9', 2)], [person('0:7', 2)])
        self.assertEqual((out.person.track_id, out.continued), ('0:9', False))
        out = summarize([person('0:5', 1), person('0:6', 1)], [])
        self.assertEqual(out.person.track_id, '0:5')


if __name__ == '__main__':
    unittest.main()
