"""Observe native LaserScan health without filtering it or authorizing motion."""
import json
import math
import time
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class RadarHealth(Node):
    def __init__(self, **kwargs):
        super().__init__('radar_health', **kwargs)
        self.declare_parameter('scan_topic', '/scan', ParameterDescriptor(read_only=True))
        self.declare_parameter('frame_id', 'laser', ParameterDescriptor(read_only=True))
        self.declare_parameter('max_age_s', 1.0, ParameterDescriptor(read_only=True))
        self.max_age = float(self.get_parameter('max_age_s').value)
        if not math.isfinite(self.max_age) or self.max_age <= 0:
            raise ValueError('max_age_s must be positive and finite')
        self.frame = self.get_parameter('frame_id').value
        self.latest = None
        self.arrivals = deque(maxlen=100)
        self.pub = self.create_publisher(String, '/radar/status', 10)
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value,
                                 self.on_scan, qos_profile_sensor_data)
        self.create_timer(.2, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_scan(self, msg):
        self.latest = msg
        self.arrivals.append(time.monotonic())

    def tick(self):
        result = dict(status='WAITING', frame_id=self.frame, hz=0., valid_points=0,
                      nearest_m=None, observation_age_s=None, detail='No LaserScan received')
        msg = self.latest
        if msg is not None:
            now = time.monotonic()
            span = self.arrivals[-1]-self.arrivals[0]
            result['hz'] = (len(self.arrivals)-1)/span if span > 0 and now-self.arrivals[-1] <= self.max_age else 0.
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            age = self.get_clock().now().nanoseconds*1e-9-stamp
            result['observation_age_s'] = age
            if stamp <= 0 or not 0 <= age <= self.max_age or now-self.arrivals[-1] > self.max_age:
                result.update(status='STALE', detail='Missing/future/expired timestamp or input stopped')
            elif (msg.header.frame_id != self.frame or not msg.ranges
                  or not all(math.isfinite(v) for v in (msg.angle_min, msg.angle_max,
                             msg.angle_increment, msg.range_min, msg.range_max))
                  or msg.angle_increment <= 0 or not 0 <= msg.range_min < msg.range_max
                  or abs(msg.angle_max-msg.angle_min-(len(msg.ranges)-1)*msg.angle_increment) > 2*msg.angle_increment):
                result.update(status='INVALID', detail='Unexpected frame or malformed scan geometry')
            else:
                values = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
                result.update(status='OK' if values else 'NO_RETURNS', valid_points=len(values),
                              nearest_m=min(values) if values else None,
                              detail='Native scan only; not a calibrated obstacle-clearance decision')
        self.pub.publish(String(data=json.dumps(result, allow_nan=False)))
        return result


def main(args=None):
    rclpy.init(args=args)
    node = RadarHealth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
