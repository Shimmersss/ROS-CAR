#!/usr/bin/env python3
"""Send one fresh mode request to the running dispatcher (no direct motion)."""
import argparse
import time
import uuid
import rclpy
from rclpy.node import Node
from execution_mode_interfaces.msg import ModeCommand


def main():
    p = argparse.ArgumentParser()
    p.add_argument('operation', choices=['ENTER', 'RETAKE', 'REPEAT', 'EXIT'])
    p.add_argument('--mode', default='tongue')
    args, ros_args = p.parse_known_args()
    rclpy.init(args=ros_args)
    node = Node('execution_mode_cli')
    pub = node.create_publisher(ModeCommand, '/execution_mode/request', 10)
    try:
        until = time.monotonic()+3.
        while not pub.get_subscription_count() and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        if not pub.get_subscription_count():
            raise SystemExit('没有模式调度器订阅者，请检查 ROS_DOMAIN_ID 和启动状态')
        msg = ModeCommand()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.request_id, msg.mode_id, msg.operation = uuid.uuid4().hex, args.mode, args.operation
        pub.publish(msg)
        end = time.monotonic()+.3
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.03)
        print('请求已发送：', args.mode, args.operation, msg.request_id)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
