import math
import unittest
from astra_body_adapter.follow_control import observation_is_fresh


class ObservationTest(unittest.TestCase):
    def test_fresh_rgbd(self):
        self.assertTrue(observation_is_fresh(10.,9.9,9.8,.1,.5,'red_object','red_object'))

    def test_republished_old_observation_stops(self):
        self.assertFalse(observation_is_fresh(10.,10.,8.,0.,.5,'red_object','red_object'))

    def test_missing_source_or_invalid_age_stops(self):
        for observed, age in ((0.,math.nan),(9.9,math.nan),(10.1,0.),(9.9,.6)):
            self.assertFalse(observation_is_fresh(10.,10.,observed,age,.5,'red_object','red_object'))
        self.assertFalse(observation_is_fresh(10.,10.,9.9,.1,.5,'yolo','red_object'))

    def test_astra_legacy_only_for_explicit_astra(self):
        self.assertTrue(observation_is_fresh(10.,9.9,0.,math.nan,.5,'astra','astra'))
        self.assertFalse(observation_is_fresh(10.,9.9,0.,math.nan,.5,'astra','red_object'))


if __name__ == '__main__': unittest.main()
