"""Explicit synthetic observations, only for validating the transport and layout."""
import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from person_interfaces.msg import TargetState


class DemoNode(Node):
    def __init__(self):
        super().__init__('perception_demo')
        descriptor = ParameterDescriptor(read_only=True)
        self.declare_parameter('publish_rate_hz', 10.0, descriptor)
        self.declare_parameter('frame_id', 'demo_camera_optical_frame', descriptor)
        rate = self.get_parameter('publish_rate_hz').value
        if not math.isfinite(rate) or not 1.0 <= rate <= 60.0:
            raise ValueError('publish_rate_hz must be between 1 and 60')
        self.frame_id = self.get_parameter('frame_id').value
        if not self.frame_id:
            raise ValueError('frame_id must not be empty')
        self.start = self.get_clock().now()
        self.publisher = self.create_publisher(TargetState, 'target_state', 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_state)
        self.get_logger().warning('SYNTHETIC DEMO ONLY: no camera, no person, no motor commands.')

    def publish_state(self):
        now = self.get_clock().now()
        t = max(0.0, (now - self.start).nanoseconds / 1e9)
        msg = TargetState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.source = 'demo'
        msg.is_simulated = True
        # A 12-second cycle explicitly includes a target-lost period.
        visible = t % 12.0 < 9.0
        msg.status = TargetState.TRACKING if visible else TargetState.LOST
        msg.detail = 'SYNTHETIC observation' if visible else 'SYNTHETIC target lost'
        msg.target_id = 'demo-person-1'
        msg.position_valid = visible
        if visible:
            msg.observation_stamp = now.to_msg()
            msg.position.x = 0.35 * math.sin(t / 2.0)
            msg.position.y = 0.0
            msg.position.z = 1.8 + 0.2 * math.cos(t / 3.0)
            msg.horizontal_distance_m = math.hypot(msg.position.x, msg.position.z)
            msg.bearing_rad = math.atan2(msg.position.x, msg.position.z)
            msg.measurement_age_s = 0.0
            msg.confidence = 1.0
        else:
            msg.position.x = msg.position.y = msg.position.z = math.nan
            msg.horizontal_distance_m = msg.bearing_rad = math.nan
            msg.measurement_age_s = msg.confidence = math.nan
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DemoNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
