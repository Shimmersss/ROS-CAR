#!/usr/bin/env python3

import math
import json
from std_msgs.msg import String
import time

from geometry_msgs.msg import TwistStamped
from person_interfaces.msg import TargetState, FallAlarm
from std_srvs.srv import Trigger
from .fall_pause import FallPause
import signal
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
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
        self.declare_parameter('ackermann', defaults.ackermann)
        self.declare_parameter('min_turn_radius_m', defaults.min_turn_radius_m)
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
            ackermann=self.get_parameter('ackermann').value,
            min_turn_radius_m=self.get_parameter('min_turn_radius_m').value,
        )
        self.message_timeout_s = float(
            self.get_parameter('message_timeout_s').value)
        if (not math.isfinite(self.message_timeout_s)
                or self.message_timeout_s <= 0.0):
            raise ValueError('message_timeout_s must be positive and finite')

        self.pause_on_fall = self.declare_parameter('pause_on_fall', False).value
        self.fall_pause = FallPause(self.declare_parameter('fall_resume_s', 3.).value,
                                   self.declare_parameter('fall_alarm_timeout_s', .5).value)
        self.resume_requested = False
        if self.pause_on_fall:
            self.create_subscription(FallAlarm, '/perception/fall_alarm', self._on_fall, 10)
            self.create_service(Trigger, '/demo/resume_follow', self._resume_follow)
        self.performance = Performance(self, '/control/performance', 'follower')
        self._authority = {}
        self._authority_at = -math.inf
        self.create_subscription(String, '/control/state', self._on_authority, 10)
        self.latest_target = None
        self.received_at = None
        self.publisher = self.create_publisher(TwistStamped, '/control/cmd_vel_request', 1)
        self.state_publisher = self.create_publisher(String, '/control/follow_state', 10)
        self._state_reason, self._state_at = None, -math.inf
        self.subscription = self.create_subscription(
            TargetState,
            '/perception/target_state',
            self._target_callback,
            10,
        )
        # Keep request/zero heartbeats running when ROS time pauses or steps.
        # Target freshness still checks both real arrival age and ROS stamps.
        self.timer = self.create_timer(0.05, self._control_tick,
                                       clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            'person follower ready; nonzero requests require fresh ARMED_FOLLOW authority')

    def _on_fall(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
        self.fall_pause.observe(message.valid, message.stage, stamp, time.monotonic(),
                                self.get_clock().now().nanoseconds*1e-9)
        if self.fall_pause.paused:
            if message.stage != FallAlarm.NONE:
                self.resume_requested = False
            self.stop()

    def _resume_follow(self, request, response):
        now, ros_now = time.monotonic(), self.get_clock().now().nanoseconds*1e-9
        response.success = self.fall_pause.fresh(now, ros_now) and self.fall_pause.stage == 0
        self.resume_requested = response.success
        response.message = ('Resume requested; fresh target and existing FOLLOW authority still required'
                            if response.success else 'Fresh NONE fall evidence required')
        return response

    def _on_authority(self, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                raise ValueError('Invalid authority state')
            self._authority, self._authority_at = value, time.monotonic()
        except (ValueError, TypeError):
            self._authority, self._authority_at = {}, -math.inf

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
        authorized = (now - self._authority_at < .3
                      and self._authority.get('authority_state') == 'ARMED_FOLLOW'
                      and self._authority.get('command_mode') == 'FOLLOW')
        fall_blocked = False
        if self.pause_on_fall:
            ros_now = self.get_clock().now().nanoseconds*1e-9
            if self.resume_requested and usable:
                self.fall_pause.resume(now, ros_now)
                self.resume_requested = False
            self.fall_pause.update(now, ros_now, can_resume=usable)
            fall_blocked = self.fall_pause.paused or not self.fall_pause.fresh(now, ros_now)
        if usable and authorized and not fall_blocked:
            linear, angular = compute_command(
                target.horizontal_distance_m,
                target.bearing_rad,
                True,
                self.config,
            )
        self._publish(linear, angular)
        reason = ('target_invalid_or_stale' if not usable else
                  'not_authorized' if not authorized else
                  'fall_paused' if fall_blocked else
                  'within_follow_distance' if target.horizontal_distance_m <= (
                      self.config.target_distance_m+self.config.distance_deadband_m) else
                  'outside_forward_half_plane' if self.config.ackermann and abs(target.bearing_rad) >= math.pi/2 else
                  'following')
        if reason != self._state_reason or now-self._state_at >= 1.:
            self._state_reason, self._state_at = reason, now
            self.state_publisher.publish(String(data=json.dumps(dict(
                reason=reason, target_id=target.target_id if target is not None else '',
                distance_m=target.horizontal_distance_m if usable else None,
                bearing_rad=target.bearing_rad if usable else None,
                target_distance_m=self.config.target_distance_m,
                ackermann=self.config.ackermann, linear_mps=linear, angular_rps=angular))))
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
    stopping = []

    def terminate(signum, frame):
        # ros2 launch may forward a second signal after the process group received
        # the first one. Do not interrupt final zero publication or ROS cleanup.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        stopping.append(signum)
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
    except RuntimeError:
        # A stop signal landing inside rclpy's take_message surfaces as RuntimeError, not
        # KeyboardInterrupt. Treat it as the requested stop; any other RuntimeError is real.
        if not stopping:
            raise
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
