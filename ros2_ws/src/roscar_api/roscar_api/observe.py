"""Read-only sensor subscriptions; compatible with reliable and best-effort sources."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import LaserScan


def run(kind):
    rclpy.init()
    node = Node('roscar_' + kind + '_example')
    if kind == 'detections':
        def show(msg):
            node.get_logger().info(str([(d.id, d.results[0].hypothesis.class_id,
                d.results[0].hypothesis.score, d.bbox.center.position.x,
                d.bbox.center.position.y, d.bbox.size_x, d.bbox.size_y) for d in msg.detections]))
        node.create_subscription(Detection2DArray, '/perception/detections', show, qos_profile_sensor_data)
    else:
        node.create_subscription(LaserScan, '/scan', lambda m: node.get_logger().info(
            f'{m.header.frame_id}: {len(m.ranges)} rays; range [{m.range_min}, {m.range_max}] m'),
            qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


def detections_main(): run('detections')
def radar_main(): run('radar')
