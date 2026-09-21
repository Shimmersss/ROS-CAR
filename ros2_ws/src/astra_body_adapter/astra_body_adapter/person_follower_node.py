#!/usr/bin/env python3

import math
import time

from geometry_msgs.msg import TwistStamped
from person_interfaces.msg import TargetState
import signal
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from .performance import Performance
from .follow_control import FollowConfig, compute_command, target_is_usable, observation_is_fresh


class PersonFollowerNode(Node):
    def __init__(self, **kwargs):
        super().__init__('person_follower', **kwargs)
        defaults = FollowConfig()
        self.declare_parameter('base_frame', 'base_link', ParameterDescriptor(read_only=True))
        if not self.get_parameter('base_frame').value:raise ValueError('base_frame must not be empty')
        self.declare_parameter('enabled', False)
        self.declare_parameter('expected_source', 'astra')
        self.declare_parameter('target_distance_m', defaults.target_distance_m)
        self.declare_parameter(
            'distance_deadband_m', defaults.distance_deadband_m)
        self.declare_parameter(
            'bearing_deadband_rad', defaults.bearing_deadband_rad)
        self.declare_parameter('turn_in_place_rad', defaults.turn_in_place_rad)
        self.declare_parameter('max_linear_mps', defaults.max_linear_mps)
        self.declare_parameter('max_angular_rps', defaults.max_angular_rps)
        self.declare_parameter('distance_gain', defaults.distance_gain)
        self.declare_parameter('bearing_gain', defaults.bearing_gain)
        self.declare_parameter('message_timeout_s', 0.5)

        self.config = FollowConfig(
            target_distance_m=self.get_parameter(
                'target_distance_m').value,
            distance_deadband_m=self.get_parameter(
                'distance_deadband_m').value,
            bearing_deadband_rad=self.get_parameter(
                'bearing_deadband_rad').value,
            turn_in_place_rad=self.get_parameter('turn_in_place_rad').value,
            max_linear_mps=self.get_parameter('max_linear_mps').value,
            max_angular_rps=self.get_parameter('max_angular_rps').value,
            distance_gain=self.get_parameter('distance_gain').value,
            bearing_gain=self.get_parameter('bearing_gain').value,
        )
        self.message_timeout_s = float(
            self.get_parameter('message_timeout_s').value)
        if (not math.isfinite(self.message_timeout_s)
                or self.message_timeout_s <= 0.0):
            raise ValueError('message_timeout_s must be positive and finite')

        self.performance = Performance(self, '/control/performance', 'follower')
        self.latest_target = None
        self.received_at = None
        self.publisher = self.create_publisher(TwistStamped, '/control/cmd_vel_request', 1)
        self.subscription = self.create_subscription(
            TargetState,
            '/perception/target_state',
            self._target_callback,
            10,
        )
        self.timer = self.create_timer(0.05, self._control_tick)
        self.get_logger().info(
            'person follower ready; enabled=false until explicitly armed')

    def _target_callback(self, msg):
        if self.performance.enabled:
            self.performance.inputs += 1
        self.latest_target = msg
        self.received_at = time.monotonic()

    def _control_tick(self):
        now = time.monotonic()
        age_s = (
            math.inf if self.received_at is None
            else now - self.received_at
        )
        target = self.latest_target
        enabled = bool(self.get_parameter('enabled').value)
        usable = target is not None and target_is_usable(
            enabled,
            target.status,
            TargetState.TRACKING,
            target.position_valid,
            age_s,
            self.message_timeout_s,
        )

        if usable:
            stamp = lambda value: value.sec + value.nanosec*1e-9
            usable = (not target.is_simulated
                      and target.source in ('astra', 'yolo', 'red_object')
                      and self.count_publishers('/perception/target_state') == 1
                      and observation_is_fresh(
                          self.get_clock().now().nanoseconds*1e-9,
                          stamp(target.header.stamp), stamp(target.observation_stamp),
                          target.measurement_age_s, self.message_timeout_s,
                          target.source, self.get_parameter('expected_source').value))

        linear = angular = 0.0
        if usable:
            linear, angular = compute_command(
                target.horizontal_distance_m,
                target.bearing_rad,
                True,
                self.config,
            )
        self._publish(linear, angular)
        if usable:
            self.performance.record('control_latency', self.performance.observation_age(target.observation_stamp))

    def _publish(self, linear, angular):
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = self.get_parameter('base_frame').value
        command.twist.linear.x = float(linear)
        command.twist.angular.z = float(angular)
        self.publisher.publish(command)
        if self.performance.enabled:
            self.performance.outputs += 1

    def stop(self):
        self._publish(0.0, 0.0)


def main(args=None):
    def terminate(signum, frame):
        # ros2 launch may forward a second signal after the process group received
        # the first one. Do not interrupt final zero publication or ROS cleanup.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, terminate)
    signal.signal(signal.SIGTERM, terminate)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = PersonFollowerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
