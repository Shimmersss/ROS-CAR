"""Explicit central-track lock/release; never arms vehicle motion."""
import argparse
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from person_interfaces.srv import ClaimOwner


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['lock', 'claim', 'release'])
    parser.add_argument('--id', default='', help='claim a visible epoch:track_id; empty selects the central track')
    args, ros_args = parser.parse_known_args()
    if args.id and args.action != 'claim':
        parser.error('--id requires claim')
    rclpy.init(args=ros_args)
    node = Node('roscar_target_example')
    try:
        kind = ClaimOwner if args.action == 'claim' else Trigger
        endpoint = '/perception/claim_owner' if args.action == 'claim' else '/perception/' + args.action + '_target'
        client = node.create_client(kind, endpoint)
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError('Target service unavailable (YOLO route required)')
        request = ClaimOwner.Request(target_id=args.id) if args.action == 'claim' else Trigger.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        if not future.done(): raise RuntimeError('Target service timed out')
        result = future.result()
        node.get_logger().info(f'success={result.success}: {result.message}')
        if not result.success: raise RuntimeError(result.message)
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
