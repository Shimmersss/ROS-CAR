"""ROS synthetic selection gate regressions; no hardware or accuracy claim."""
import copy
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tests'))
sys.path.insert(0, str(ROOT / 'ros2_ws/src/yolo_person_tracker/test'))

import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point
from person_interfaces.msg import PersonStateArray, TargetState
from test_ground_runtime import to_state, static_tf, FRAME
from test_ground import mount
from test_unified import measured
from yolo_person_tracker.ground_node import GroundLocalizer
from yolo_person_tracker.unified_node import UnifiedLocalizer


def run(cls):
    node = cls(namespace='review_' + cls.NODE_NAME, parameter_overrides=[
        Parameter('extrinsics_calibrated', value=True),
        Parameter('ground_plane_confirmed', value=True),
        Parameter('reid_lock_enabled', value=False),
    ])
    try:
        transform = mount()
        node.buffer.set_transform_static(static_tf(transform), 'synthetic_fixture')
        info = CameraInfo()
        info.width, info.height = 640, 480
        info.header.frame_id = FRAME
        info.p = [500., 0., 320., 0., 0., 500., 240., 0., 0., 0., 1., 0.]
        node.on_info(info)
        # Three consistent frames satisfy the unified estimator's source confirmation.
        for _ in range(3):
            fixture = measured(transform, foot=(2., .4), indices=(15, 16))
            person = to_state(fixture, '0:7')
            person.keypoints_3d = [Point(x=float(p.x), y=float(p.y), z=float(p.z))
                                  for p in fixture.keypoints_3d]
            person.keypoints_3d_valid = list(fixture.keypoints_3d_valid)
            people = PersonStateArray(valid=True)
            people.header.frame_id = FRAME
            people.header.stamp = node.get_clock().now().to_msg()
            people.persons = [person]
            node.on_persons(people)
            target = TargetState(target_id='0:7', source='yolo', status=TargetState.TRACKING)
            target.header.stamp = copy.deepcopy(people.header.stamp)
            node.on_target(target)
            time.sleep(.03)
        assert node.latest.valid and node.latest.persons[0].valid
        assert node.tick().position_valid, 'fixture must first produce a valid selected position'
        # This is the exact state node.py publishes while BridgeArbiter pauses an ID conflict:
        # LOST, selected ID retained, and position_valid=False; all-person geometry remains valid.
        target.status = TargetState.LOST
        target.position_valid = False
        target.detail = 'Paused: single-object tracker disagrees with the selected track (possible ID swap)'
        target.header.stamp = node.get_clock().now().to_msg()
        node.on_target(target)
        result = node.tick()
        assert result.status == TargetState.LOST and not result.position_valid
        assert node.latest.valid and node.latest.persons[0].valid
        # Reject paused, stale, future, simulated and wrong-source selections in both modes.
        for identity in (False, True):
            node.cfg['reid_lock_enabled'] = identity
            for status, age, simulated, source in (
                    (TargetState.LOST, 0., False, 'yolo'),
                    (TargetState.TRACKING, 2., False, 'yolo'),
                    (TargetState.TRACKING, -2., False, 'yolo'),
                    (TargetState.TRACKING, 0., True, 'yolo'),
                    (TargetState.TRACKING, 0., False, 'demo')):
                target.status, target.is_simulated, target.source = status, simulated, source
                target.header.stamp = rclpy.time.Time(nanoseconds=int(
                    node.get_clock().now().nanoseconds-age*1e9)).to_msg()
                node.on_target(target)
                assert not node.tick().position_valid, (cls, identity, status, age, simulated, source)
            target.status, target.is_simulated, target.source = TargetState.TRACKING, False, 'yolo'
            target.header.stamp = node.get_clock().now().to_msg()
            node.on_target(target)
            # Valid fresh 2D selection is enough: independent ground geometry supplies depth.
            assert not target.position_valid and node.tick().position_valid
        print('PASS ' + cls.NODE_NAME + ': loss/freshness/simulation/source gate and recovery')
    finally:
        node.destroy_node()


if __name__ == '__main__':
    rclpy.init()
    try:
        run(GroundLocalizer)
        run(UnifiedLocalizer)
    finally:
        rclpy.shutdown()
