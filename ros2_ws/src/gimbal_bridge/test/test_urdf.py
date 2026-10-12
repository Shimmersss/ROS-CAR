import copy
import os
import unittest
import xml.etree.ElementTree as ET
from gimbal_bridge.urdf import build_urdf, load_mount, validate_mount

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'gimbal_mount.yaml')


class UrdfTests(unittest.TestCase):
    def test_default_mount_is_unconfirmed_placeholder(self):
        self.assertFalse(load_mount(CONFIG)['confirmed'])

    def test_tree_axes_and_limits(self):
        robot = ET.fromstring(build_urdf(load_mount(CONFIG)))
        joints = {j.get('name'): j for j in robot.findall('joint')}
        self.assertEqual(joints['pan_joint'].find('parent').get('link'), 'base_footprint')
        self.assertEqual(joints['pan_joint'].find('axis').get('xyz'), '0 0 1')
        self.assertEqual(joints['tilt_joint'].find('axis').get('xyz'), '0 1 0')
        self.assertEqual(joints['camera_mount_joint'].find('child').get('link'), 'camera_link')
        self.assertEqual(float(joints['pan_joint'].find('limit').get('upper')), 2.967)
        self.assertNotIn('camera_color_optical_frame', [l.get('name') for l in robot.findall('link')])

    def test_rejects_bad_values(self):
        mount = load_mount(CONFIG)
        for path, value in ((('confirmed',), 'yes'), (('base_to_pan', 'xyz'), [0., float('nan'), 0.]),
                            (('pan_limit_rad',), 4.), (('tilt_lower_rad',), 1.), (('camera_frame',), ''),
                            (('tilt_to_imu', 'rpy'), [0., 0.])):
            bad = copy.deepcopy(mount)
            target = bad
            for key in path[:-1]: target = target[key]
            target[path[-1]] = value
            with self.assertRaises(ValueError, msg=str(path)):
                validate_mount(bad)
