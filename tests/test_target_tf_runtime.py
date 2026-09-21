"""Synthetic TF acceptance, not physical camera calibration."""
import math
import time

import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import TransformStamped
from person_interfaces.msg import TargetState
from yolo_person_tracker.target_tf import TargetTransform


def make_node(calibrated):
    return TargetTransform(namespace='calibrated' if calibrated else 'uncalibrated', parameter_overrides=[
        Parameter('extrinsics_calibrated', value=calibrated),
        Parameter('publish_mount_tf', value=False),
        Parameter('max_age_s', value=.4)])


def main():
    rclpy.init()
    node = make_node(True)
    uncalibrated = make_node(False)

    def sample():
        msg = TargetState()
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.observation_stamp = node.get_clock().now().to_msg()
        msg.header.stamp = msg.observation_stamp
        msg.source = 'yolo'
        msg.target_id = '0:7'
        msg.status = TargetState.TRACKING
        msg.position_valid = True
        msg.position.x, msg.position.y, msg.position.z = .2, .3, 2.
        return msg

    def transform(parent, child, xyz, quat, stamp=None):
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = parent, child
        t.header.stamp = stamp or node.get_clock().now().to_msg()
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = quat
        return t

    try:
        msg = sample()
        uncalibrated.on_state(msg)
        result = uncalibrated.tick()
        assert not result.position_valid and 'UNCONFIRMED' in result.detail
        node.on_state(msg)
        assert not node.tick().position_valid  # Missing TF cannot become valid.
        # Optical right/down/forward -> body forward/left/up, then installation translation.
        node.buffer.set_transform_static(transform('camera_link', 'camera_color_optical_frame',
                                                   (0.,0.,0.), (-.5,.5,-.5,.5)), 'test')
        node.buffer.set_transform_static(transform('base_link', 'camera_link',
                                                   (.1,0.,.5), (0.,0.,0.,1.)), 'test')
        node.on_state(sample())
        result = node.tick()
        assert result.position_valid, result.detail
        assert result.header.frame_id == 'base_link'
        assert abs(result.position.x-2.1)<1e-6
        assert abs(result.position.y+.2)<1e-6
        assert abs(result.position.z-.2)<1e-6
        assert abs(result.bearing_rad-math.atan2(-.2,2.1))<1e-6
        assert abs(result.horizontal_distance_m-math.hypot(2.1,.2))<1e-6
        # Include an installed yaw rotation, not only the optical-axis convention.
        node.buffer.set_transform_static(transform('base_link', 'camera_link',
            (.1,0.,.5), (0.,0.,math.sqrt(.5),math.sqrt(.5))), 'test')
        node.on_state(sample())
        result = node.tick()
        assert result.position_valid and abs(result.position.x-.3)<1e-6
        assert abs(result.position.y-2.)<1e-6 and abs(result.position.z-.2)<1e-6
        msg = sample(); msg.header.frame_id = 'wrong_frame'; node.on_state(msg)
        assert not node.tick().position_valid
        msg = sample(); msg.position.x = math.nan; node.on_state(msg)
        assert not node.tick().position_valid
        msg = sample(); msg.observation_stamp.sec -= 1; node.on_state(msg)
        assert node.tick().status == TargetState.STALE
        msg = sample(); msg.observation_stamp.sec += 1; node.on_state(msg)
        assert node.tick().status == TargetState.STALE
        node.on_state(sample()); node.received_at -= 1
        assert node.tick().status == TargetState.STALE
        # A dynamic transform only at an older time must not be used as latest fallback.
        node.buffer.clear()
        msg = sample()
        old = node.get_clock().now().to_msg(); old.sec -= 2
        node.buffer.set_transform(transform('base_link', 'camera_color_optical_frame',
                                            (0.,0.,0.), (0.,0.,0.,1.), old), 'test')
        node.on_state(msg)
        assert not node.tick().position_valid
        assert '/cmd_vel' not in dict(node.get_topic_names_and_types())
        print('PASS TF: uncalibrated/missing rejection, rotation+translation, base signs, invalid/stale input, observation-time lookup')
    finally:
        node.destroy_node(); uncalibrated.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
