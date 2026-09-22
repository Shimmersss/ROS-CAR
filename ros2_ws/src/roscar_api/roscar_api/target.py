"""Explicit central-track lock/release; never arms vehicle motion."""
import argparse
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['lock', 'release'])
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Node('roscar_target_example')
    try:
        client = node.create_client(Trigger, '/perception/' + args.action + '_target')
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError('Target service unavailable (YOLO route required)')
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        if not future.done(): raise RuntimeError('Target service timed out')
        result = future.result()
        node.get_logger().info(f'success={result.success}: {result.message}')
        if not result.success: raise RuntimeError(result.message)
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
