"""List every live ROS endpoint and its native type, without starting any module."""
import argparse
import json
import math
import time

import rclpy
from rclpy.action import get_action_names_and_types
from rclpy.node import Node


def discover(node):
    return {
        'topics': dict(node.get_topic_names_and_types()),
        'services': dict(node.get_service_names_and_types()),
        'actions': dict(get_action_names_and_types(node)),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wait', type=float, default=2., help='DDS discovery time, seconds (0, 30]')
    parser.add_argument('--json', action='store_true')
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.wait) or not 0 < args.wait <= 30:
        parser.error('--wait must be finite and in (0, 30]')
    rclpy.init(args=ros_args)
    node = Node('roscar_interface_list')
    try:
        end = time.monotonic()+args.wait
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        result = discover(node)
        if args.json:
            print(json.dumps(result, ensure_ascii=False, sort_keys=True, indent=2))
        else:
            for kind, endpoints in result.items():
                for name, types in sorted(endpoints.items()):
                    print(kind.rstrip('s'), name, ', '.join(types))
    finally:
        node.destroy_node()
        rclpy.shutdown()
