"""Read-only, bounded RGB-D diagnostics. Does not load YOLO or publish commands."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from .input_contract import stamp_seconds, validate_pair


class Stats:
    def __init__(self):
        self.count, self.total, self.low, self.high = 0, 0., math.inf, -math.inf

    def add(self, value):
        self.count += 1
        self.total += value
        self.low, self.high = min(self.low, value), max(self.high, value)

    def report(self):
        return (dict(count=self.count, min=self.low, mean=self.total/self.count, max=self.high)
                if self.count else dict(count=0, min=None, mean=None, max=None))


class InputProbe(Node):
    def __init__(self, color_topic, depth_topic, camera_info_topic, max_age=.5, sync_slop=.06):
        super().__init__('rgbd_input_probe')
        self.max_age, self.sync_slop = max_age, sync_slop
        self.started = time.monotonic()
        self.info = None
        self.accepted_at = -math.inf
        self.accepted_stamp = 0.
        self.counts, self.errors = Counter(), Counter()
        self.latest, self.received = {}, {}
        self.metrics = {key: Stats() for key in
                        ('pair_skew_s', 'color_age_s', 'depth_age_s', 'depth_valid_fraction')}
        self.bridge = CvBridge()
        self.topics = dict(color=color_topic, depth=depth_topic, camera_info=camera_info_topic)
        self.color = message_filters.Subscriber(self, Image, color_topic, qos_profile=qos_profile_sensor_data)
        self.depth = message_filters.Subscriber(self, Image, depth_topic, qos_profile=qos_profile_sensor_data)
        self.color.registerCallback(lambda msg: self.observe('color', msg))
        self.depth.registerCallback(lambda msg: self.observe('depth', msg))
        self.create_subscription(CameraInfo, camera_info_topic, self.on_info, qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color, self.depth], queue_size=5, slop=sync_slop)
        self.sync.registerCallback(self.on_pair)

    def observe(self, kind, msg):
        self.counts[kind] += 1
        self.received[kind] = time.monotonic()
        self.latest[kind] = dict(width=msg.width, height=msg.height,
                                 frame_id=msg.header.frame_id, stamp_s=stamp_seconds(msg.header.stamp))
        if kind != 'camera_info':
            self.latest[kind]['encoding'] = msg.encoding
            self.metrics[kind+'_age_s'].add(
                self.get_clock().now().nanoseconds*1e-9-stamp_seconds(msg.header.stamp))

    def on_info(self, msg):
        self.info = msg
        self.observe('camera_info', msg)
        self.latest['camera_info'].update(p=list(msg.p), k=list(msg.k), d=list(msg.d),
                                          distortion_model=msg.distortion_model)

    def on_pair(self, color, depth):
        self.counts['pairs'] += 1
        now = self.get_clock().now().nanoseconds*1e-9
        self.metrics['pair_skew_s'].add(abs(stamp_seconds(color.header.stamp)-stamp_seconds(depth.header.stamp)))
        try:
            validate_pair(color, depth, self.info, now, self.max_age, self.sync_slop)
            self.bridge.imgmsg_to_cv2(color, desired_encoding='bgr8')
            metres = self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough').astype(np.float32)
            if depth.encoding == '16UC1':
                metres *= .001
            valid = np.isfinite(metres) & (metres >= .2) & (metres <= 8.)
            self.metrics['depth_valid_fraction'].add(float(np.mean(valid)))
            self.counts['accepted_pairs'] += 1
            self.accepted_at = time.monotonic()
            self.accepted_stamp = min(stamp_seconds(color.header.stamp), stamp_seconds(depth.header.stamp))
        except (ValueError, TypeError, cv2.error, CvBridgeError) as exc:
            self.errors[str(exc)] += 1

    def report(self):
        elapsed = max(time.monotonic()-self.started, 1e-6)
        missing = [kind for kind in self.topics if not self.counts[kind]]
        stale = [kind for kind in ('color', 'depth')
                 if kind in self.received and time.monotonic()-self.received[kind] > self.max_age]
        fresh_pair = (time.monotonic()-self.accepted_at <= self.max_age
                      and 0 <= self.get_clock().now().nanoseconds*1e-9-self.accepted_stamp <= self.max_age)
        metadata_pass = bool(fresh_pair and not self.errors and not missing and not stale)
        return dict(
            metadata_pass=metadata_pass, fresh_accepted_pair=fresh_pair, registration_verified=False,
            note='Metadata/decode check only. Physical alignment, rectification and sensor capture timing require separate verification.',
            elapsed_s=elapsed, topics=self.topics, counts=dict(self.counts),
            received_hz={kind: self.counts[kind]/elapsed for kind in self.topics},
            missing_streams=missing, stale_streams=stale, rejected_reasons=dict(self.errors),
            latest_stamp_skew_s=(abs(self.latest['color']['stamp_s']-self.latest['depth']['stamp_s'])
                                 if 'color' in self.latest and 'depth' in self.latest else None),
            latest=self.latest, metrics={key: value.report() for key, value in self.metrics.items()},
            cmd_vel_topic_observed='/cmd_vel' in dict(self.get_topic_names_and_types()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--color-topic', default='/camera/color/image_rect')
    parser.add_argument('--depth-topic', default='/camera/aligned_depth_to_color/image_raw')
    parser.add_argument('--camera-info-topic', default='/camera/color/camera_info')
    parser.add_argument('--duration', type=float, default=15.)
    parser.add_argument('--max-age', type=float, default=.5)
    parser.add_argument('--sync-slop', type=float, default=.06)
    parser.add_argument('--output', type=Path, help='Optional JSON report; no image data is saved')
    args = parser.parse_args()
    if not (math.isfinite(args.duration) and args.duration > 0
            and math.isfinite(args.max_age) and 0 < args.sync_slop < args.max_age):
        parser.error('Require duration > 0 and 0 < sync-slop < max-age, all finite')
    rclpy.init(args=[])
    node = InputProbe(args.color_topic, args.depth_topic, args.camera_info_topic,
                      args.max_age, args.sync_slop)
    try:
        end = time.monotonic()+args.duration
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=min(.1, max(0., end-time.monotonic())))
        report = node.report()
        # Invalid calibration may contain NaN; keep the JSON standards compliant.
        def clean(value):
            if isinstance(value, float) and not math.isfinite(value):
                return None
            if isinstance(value, dict):
                return {k: clean(v) for k, v in value.items()}
            if isinstance(value, list):
                return [clean(v) for v in value]
            return value
        output = json.dumps(clean(report), ensure_ascii=False, indent=2, allow_nan=False)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(output+'\n', encoding='utf-8')
        print(output)
        return 0 if report['metadata_pass'] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
