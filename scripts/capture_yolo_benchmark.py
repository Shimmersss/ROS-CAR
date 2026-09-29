#!/usr/bin/env python3
"""Read-only capture of bounded BGR frames for the backend benchmark."""
import argparse
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/camera/color/image_raw')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--frames', type=int, default=30)
    args = parser.parse_args()
    if not 10 <= args.frames <= 100:
        parser.error('--frames must be 10..100')
    if args.output.exists():
        parser.error('output already exists')
    rclpy.init()
    node = rclpy.create_node('yolo_benchmark_capture')
    bridge = CvBridge()
    frames, stamps = [], []
    previous = -float('inf')

    def receive(msg):
        nonlocal previous
        now = time.monotonic()
        if now-previous < .2 or len(frames) >= args.frames:
            return
        frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
        if frames and frame.shape != frames[0].shape:
            return
        frames.append(frame.copy())
        stamps.append(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
        previous = now

    node.create_subscription(Image, args.topic, receive, qos_profile_sensor_data)
    deadline = time.monotonic()+30
    try:
        while len(frames) < args.frames and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.2)
        if len(frames) != args.frames:
            raise RuntimeError(f'Only received {len(frames)}/{args.frames} frames')
        np.savez_compressed(args.output, bgr=np.stack(frames), stamps=np.asarray(stamps))
        print(f'Saved {len(frames)} real camera frames to {args.output}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
