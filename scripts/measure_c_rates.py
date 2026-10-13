#!/usr/bin/env python3
"""Read-only C frame-rate probe for the Jetson: small topics plus /proc CPU, never decodes images.

Per-stage rates come from CameraInfo-sized topics published once per frame (driver color/depth,
rectified CameraInfo from registration), the Pose node's RuntimeMetrics and TargetState ages.
Process CPU is sampled from /proc. Diagnostic only: it publishes nothing and is not an acceptance.
  source install/setup.bash && ROS_DOMAIN_ID=183 python3 scripts/measure_c_rates.py --duration 20
"""
import argparse
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from person_interfaces.msg import RuntimeMetrics, TargetState

# Matched against each argv element's basename, so wrapper shells and this probe never count.
PROCESSES = ('pose_tracker', 'gemini_registration', 'astra_camera_node', 'component_container',
             'ground_localizer', 'unified_localizer', 'reid_observer', 'foxglove_bridge')


def summary(values):
    values = [v for v in values if math.isfinite(v)]
    if not values:
        return None
    values.sort()
    return dict(mean=round(sum(values)/len(values), 2), p50=round(values[len(values)//2], 2),
                p95=round(values[min(len(values)-1, int(len(values)*.95))], 2), n=len(values))


def cpu_ticks():
    """{pid: (name, utime+stime ticks)} for processes whose command line names a C stage."""
    result = {}
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                argv = f.read().decode(errors='replace').split('\0')
            if argv and os.path.basename(argv[0]) in ('bash', 'sh'):
                continue
            names = {os.path.basename(arg) for arg in argv}
            name = next((p for p in PROCESSES if p in names), None)
            if name is None:
                continue
            with open(f'/proc/{pid}/stat') as f:
                fields = f.read().rsplit(')', 1)[1].split()
            result[pid] = (name, int(fields[11])+int(fields[12]))
        except (OSError, IndexError, ValueError):
            continue
    return result


class RateProbe(Node):
    def __init__(self, topics):
        super().__init__('c_rate_probe')
        self.stamps = {name: [] for name in topics}
        for name, topic in topics.items():
            self.create_subscription(CameraInfo, topic, lambda m, n=name: self.stamps[n].append(time.monotonic()),
                                     qos_profile_sensor_data)
        self.metrics = []
        self.ages = []
        self.create_subscription(RuntimeMetrics, '/perception/performance', self.metrics.append, 10)
        self.create_subscription(TargetState, '/perception/target_state', self.on_target, 10)

    def on_target(self, msg):
        if msg.position_valid:
            self.ages.append(msg.measurement_age_s*1000.)


def rate(stamps, duration):
    return round(len(stamps)/duration, 2) if stamps else 0.


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--duration', type=float, default=20.)
    parser.add_argument('--color-info', default='/camera/color/camera_info')
    parser.add_argument('--depth-info', default='/camera/depth/camera_info')
    parser.add_argument('--rect-info', default='/camera/color/camera_info_rect')
    args = parser.parse_args()
    if not 2 <= args.duration <= 600:
        parser.error('--duration must be within [2, 600] s')
    rclpy.init()
    probe = RateProbe(dict(driver_color=args.color_info, driver_depth=args.depth_info,
                           registered=args.rect_info))
    tick = os.sysconf('SC_CLK_TCK')
    try:
        end = time.monotonic()+1.
        while time.monotonic() < end:  # discovery warm-up, not counted
            rclpy.spin_once(probe, timeout_sec=.05)
        for values in probe.stamps.values():
            values.clear()
        probe.metrics.clear(); probe.ages.clear()
        before, start = cpu_ticks(), time.monotonic()
        while time.monotonic()-start < args.duration:
            rclpy.spin_once(probe, timeout_sec=.05)
        elapsed, after = time.monotonic()-start, cpu_ticks()
        cpu = {}
        for pid, (name, ticks) in after.items():
            if pid in before:
                cpu[name] = round(cpu.get(name, 0.)+(ticks-before[pid][1])/tick/elapsed*100., 1)
        result = dict(
            duration_s=round(elapsed, 2),
            rates_hz={name: rate(values, elapsed) for name, values in probe.stamps.items()},
            pose_input_fps=summary([m.input_fps for m in probe.metrics]),
            pose_processing_ms=summary([m.processing_ms for m in probe.metrics]),
            pose_observation_age_ms=summary([m.observation_age_ms for m in probe.metrics]),
            target_measurement_age_ms=summary(probe.ages),
            process_cpu_percent=cpu,
            note='diagnostic only; 100% = one core; processing_ms includes in-process registration when enabled')
        print(json.dumps(result, ensure_ascii=False, indent=2))
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
