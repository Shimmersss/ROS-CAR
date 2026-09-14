"""Readiness-only scaffold. Does not open a camera or claim real detections."""
import math

import rclpy
from rclpy.node import Node
from person_interfaces.msg import TargetState


class ReadinessNode(Node):
    def __init__(self):
        super().__init__('yolo_person_tracker')
        self.publisher = self.create_publisher(TargetState, 'target_state', 10)
        self.timer = self.create_timer(1.0, self.publish_state)
        self.get_logger().warning('YOLO + ByteTrack + aligned depth pipeline not connected.')

    def publish_state(self):
        msg = TargetState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = 'yolo'
        msg.status = TargetState.NOT_READY
        msg.detail = 'YOLO + ByteTrack + aligned depth pipeline not connected.'
        msg.is_simulated = False
        msg.position_valid = False
        msg.position.x = msg.position.y = msg.position.z = math.nan
        msg.horizontal_distance_m = msg.bearing_rad = math.nan
        msg.measurement_age_s = msg.confidence = math.nan
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ReadinessNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
