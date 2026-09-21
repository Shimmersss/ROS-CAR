import copy
from types import SimpleNamespace as NS
import unittest

from yolo_person_tracker.input_contract import validate_pair


class InputContractTest(unittest.TestCase):
    def setUp(self):
        self.color = NS(width=640, height=480, encoding='rgb8',
                        header=NS(frame_id='color_optical', stamp=NS(sec=100, nanosec=0)))
        self.depth = copy.deepcopy(self.color)
        self.depth.encoding = '16UC1'
        self.info = copy.deepcopy(self.color)
        self.info.p = [500., 0., 320., 0., 0., 500., 240., 0., 0., 0., 1., 0.]
        self.info.binning_x = self.info.binning_y = 0
        self.info.roi = NS(x_offset=0, y_offset=0)

    def check(self):
        return validate_pair(self.color, self.depth, self.info, 100.1, .5, .06)

    def test_valid(self):
        self.assertEqual(self.check(), (500., 500., 320., 240.))
        self.depth.encoding = '32FC1'
        self.check()

    def test_projection_not_just_focal_length(self):
        original = self.info.p[:]
        for index, value in ((1, 2.), (4, 2.), (3, .1), (7, .1),
                             (8, float('nan')), (9, .2), (10, 0.), (11, 1.)):
            with self.subTest(index=index):
                self.info.p = original[:]
                self.info.p[index] = value
                with self.assertRaisesRegex(ValueError, 'projection'):
                    self.check()

    def test_stamp_and_skew(self):
        for seconds in (0, 98, 101):
            self.depth.header.stamp.sec = seconds
            with self.assertRaisesRegex(ValueError, 'timestamp'):
                self.check()
        self.depth.header.stamp.sec = 100
        self.depth.header.stamp.nanosec = 80000000
        with self.assertRaisesRegex(ValueError, 'sync tolerance'):
            self.check()

    def test_metadata_mismatch(self):
        for obj, key, value, error in (
                (self.depth.header, 'frame_id', 'depth_optical', 'optical frame'),
                (self.depth, 'width', 320, 'dimensions'),
                (self.color, 'height', 0, 'dimensions'),
                (self.info, 'binning_x', 2, 'Binned'),
                (self.info.roi, 'x_offset', 1, 'cropped'),
                (self.color, 'encoding', 'mono8', 'RGB/BGR'),
                (self.depth, 'encoding', 'mono16', 'Depth must')):
            old = getattr(obj, key)
            setattr(obj, key, value)
            with self.assertRaisesRegex(ValueError, error):
                self.check()
            setattr(obj, key, old)
        self.info = None
        with self.assertRaisesRegex(ValueError, 'Waiting'):
            self.check()
