"""Test-only ROS time driven by monotonic time, immune to VM wall-clock steps.

Use on nodes constructed with use_sim_time=True. Real arrival/worker ages still
use time.monotonic in production code. No deployment clock behavior is changed.
"""
import time
from rclpy.time import Time


class MonotonicRosClock:
    def __init__(self, node):
        if not node.get_parameter('use_sim_time').value:
            raise ValueError('Test clock requires use_sim_time=True')
        self.clock = node.get_clock()
        self.started = time.monotonic_ns()
        self.offset_ns = 1_000_000_000
        self.tick()

    def tick(self):
        self.clock.set_ros_time_override(Time(
            nanoseconds=self.offset_ns + time.monotonic_ns() - self.started))

    def spin_once(self, executor, timeout_sec=.005):
        self.tick()
        executor.spin_once(timeout_sec=timeout_sec)
