"""Bounded per-window metrics; no per-frame ROS publication or logging."""
from collections import deque
import math
import time
from person_interfaces.msg import RuntimeMetrics


class Performance:
    FIELDS = ('processing', 'rgbd', 'observation_age', 'control_latency')

    def __init__(self, node, topic, source):
        self.node, self.source = node, source
        node.declare_parameter('performance_enabled', True)
        self.enabled = bool(node.get_parameter('performance_enabled').value)
        self.inputs = self.outputs = 0
        self.samples = {name: deque(maxlen=4096) for name in self.FIELDS}
        self.started = time.monotonic()
        if self.enabled:
            self.publisher = node.create_publisher(RuntimeMetrics, topic, 2)
            self.timer = node.create_timer(1., self.publish)

    def record(self, name, milliseconds):
        if self.enabled and math.isfinite(milliseconds) and milliseconds >= 0:
            self.samples[name].append(float(milliseconds))

    def observation_age(self, stamp):
        seconds = stamp.sec + stamp.nanosec*1e-9
        if seconds > 0:
            return (self.node.get_clock().now().nanoseconds*1e-9 - seconds)*1000
        return math.nan

    def publish(self):
        now = time.monotonic()
        duration = now-self.started
        if duration <= 0:
            return
        msg = RuntimeMetrics()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.source, msg.window_s = self.source, duration
        msg.input_count, msg.output_count = self.inputs, self.outputs
        msg.input_fps, msg.output_fps = self.inputs/duration, self.outputs/duration
        for name, values in self.samples.items():
            ordered = sorted(values)
            mean = sum(ordered)/len(ordered) if ordered else math.nan
            p95 = ordered[math.ceil(.95*len(ordered))-1] if ordered else math.nan
            setattr(msg, name+'_ms', mean)
            setattr(msg, name+'_p95_ms', p95)
            values.clear()
        self.inputs = self.outputs = 0
        self.started = now
        self.publisher.publish(msg)
