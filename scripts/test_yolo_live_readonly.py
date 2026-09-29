#!/usr/bin/env python3
"""Validate the real YOLO RGB-D node in a separate namespace; no control nodes."""
import argparse
from collections import Counter
import json
from pathlib import Path
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from person_interfaces.msg import TargetState
from vision_msgs.msg import Detection2DArray
from yolo_person_tracker.node import TrackerNode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=20.)
    args = parser.parse_args()
    if not 1 <= args.duration <= 120:
        parser.error('duration must be 1..120 seconds')
    rclpy.init()
    node = TrackerNode(namespace='/validation_yolo_20260929', parameter_overrides=[
        Parameter('model_path', value=args.model), Parameter('device', value='0'),
        Parameter('depth_registered', value=True), Parameter('auto_lock_single', value=True),
        Parameter('color_topic', value='/camera/color/image_raw'),
        Parameter('depth_topic', value='/camera/depth/image_raw'),
        Parameter('camera_info_topic', value='/camera/color/camera_info'),
        Parameter('visualization_fps', value=0.0)])
    states, detections, valid_positions = Counter(), [], []
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def state(msg):
        states[str(msg.status)] += 1
        if msg.position_valid:
            valid_positions.append([msg.position.x, msg.position.y, msg.position.z, msg.measurement_age_s])

    node.create_subscription(TargetState, 'target_state', state, 10)
    node.create_subscription(Detection2DArray, 'detections', lambda msg: detections.append(len(msg.detections)), 10)
    started = time.monotonic()
    try:
        while time.monotonic()-started < args.duration:
            executor.spin_once(timeout_sec=.05)
        report = dict(model=args.model, namespace=node.get_namespace(),
                      duration_s=time.monotonic()-started, states=dict(states),
                      detection_messages=len(detections), person_counts=detections,
                      valid_positions=valid_positions, error=node.error,
                      kalman_measurement_std_m=node.cfg['kalman_measurement_std_m'],
                      kalman_acceleration_std_mps2=node.cfg['kalman_acceleration_std_mps2'],
                      scope='live camera and real model; separate namespace, no motion')
        args.output.write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ('person_counts','valid_positions')}, indent=2))
        if not detections or node.error:
            raise RuntimeError('No successful real-model detection messages or final node error')
    finally:
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
