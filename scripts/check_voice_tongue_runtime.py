#!/usr/bin/env python3
"""Read-only runtime snapshot; does not publish commands or save camera/audio data."""
import argparse
import json
import time
from collections import Counter

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from roscar_interfaces.msg import AudioFrame, VoiceCommandResult


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=10)
    args = parser.parse_args()
    rclpy.init()
    node = Node('voice_tongue_readonly_probe')
    counts, latest, peaks = Counter(), {}, Counter()
    def receive(key, transform):
        def callback(message):
            counts[key] += 1
            latest[key] = transform(message)
        return callback
    def velocity(message, key):
        value = max(abs(message.linear.x), abs(message.linear.y), abs(message.angular.z))
        peaks[key] = max(peaks[key], value)
        return value
    specifications = [
        ('audio', AudioFrame, '/voice/audio', lambda m: dict(rate=m.sample_rate, channels=m.channels, bytes=len(m.pcm_s16le))),
        ('camera', Image, '/camera/color/image_raw', lambda m: dict(width=m.width, height=m.height, encoding=m.encoding)),
        ('odom', Odometry, '/odom', lambda m: velocity(m.twist.twist, 'odom')),
        ('cmd_vel', Twist, '/cmd_vel', lambda m: velocity(m, 'cmd_vel')),
        ('health', Bool, '/voice/emergency_healthy', lambda m: m.data),
        ('control', String, '/control/state', lambda m: json.loads(m.data)),
        ('mode', String, '/execution_mode/state', lambda m: json.loads(m.data)),
        ('tongue', String, '/tongue/state', lambda m: json.loads(m.data)),
        ('asr', String, '/voice/asr_text', lambda m: m.data),
        ('wake', String, '/voice_words', lambda m: m.data),
        ('response', VoiceCommandResult, '/voice/command_result', lambda m: dict(success=m.success, message=m.message)),
    ]
    for key, kind, topic, transform in specifications:
        node.create_subscription(kind, topic, receive(key, transform), qos_profile_sensor_data)
    start = time.monotonic()
    while time.monotonic()-start < args.seconds:
        rclpy.spin_once(node, timeout_sec=.1)
    report = dict(seconds=round(time.monotonic()-start, 2), counts=dict(counts), latest=latest,
                  peak_velocity=dict(peaks), cmd_vel_publishers=node.count_publishers('/cmd_vel'),
                  nodes=sorted(node.get_node_names()))
    print(json.dumps(report, ensure_ascii=False, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
