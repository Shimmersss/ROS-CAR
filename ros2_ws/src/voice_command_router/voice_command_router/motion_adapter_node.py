"""Convert semantic requests to chassis velocity, gated by global authority."""
from dataclasses import fields
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from roscar_interfaces.msg import MotionRequest as MotionRequestMsg, VoiceCommandResult
from .motion import MotionRequest, ChassisCapabilities, adapt_motion
from .validation import CommandValidationError


class MotionAdapter(Node):
    def __init__(self, **kwargs):
        super().__init__('chassis_motion_adapter', **kwargs)
        for field in fields(ChassisCapabilities):
            self.declare_parameter(field.name, getattr(ChassisCapabilities(), field.name))
        self.capabilities = ChassisCapabilities(**{f.name: self.get_parameter(f.name).value
                                                   for f in fields(ChassisCapabilities)})
        self.state, self.state_at = {}, -float('inf')
        self.velocity, self.until, self.epoch = (0., 0.), 0., None
        self.seen = {}
        self.pub = self.create_publisher(TwistStamped, '/chassis/cmd_vel', 1)
        self.results = self.create_publisher(VoiceCommandResult, '/voice/command_result', 10)
        self.tts = self.create_publisher(String, '/voice/tts_text', 10)
        self.create_subscription(String, '/control/state', self.on_state, 10)
        self.create_subscription(MotionRequestMsg, '/voice/motion_request', self.on_request, 10)
        self.timer = self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_state(self, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                raise ValueError('Invalid authority state')
            self.state, self.state_at = value, time.monotonic()
        except (TypeError, ValueError):
            self.state, self.state_at = {}, -float('inf')
            self.velocity = (0., 0.)

    def authorized(self, now):
        return (now - self.state_at < .3 and self.state.get('authority_state') == 'ARMED_MANUAL'
                and self.state.get('command_mode') == 'EXTERNAL')

    def on_request(self, message):
        now = time.monotonic()
        result = VoiceCommandResult()
        result.stamp, result.request_id, result.action = self.get_clock().now().to_msg(), message.request_id, message.action
        try:
            stamp = message.stamp.sec + message.stamp.nanosec * 1e-9
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            self.seen = {key: at for key, at in self.seen.items() if now - at < 60}
            if not message.request_id or message.request_id in self.seen or stamp <= 0 or not 0 <= age <= 2:
                raise CommandValidationError('运动意图已过期或重复')
            self.seen[message.request_id] = now
            if not self.authorized(now) or message.authority_epoch != self.state.get('authority_epoch'):
                raise CommandValidationError('请先完成手动模式授权')
            request = MotionRequest(message.action, message.direction, message.style,
                                    message.speed if message.speed_specified else None,
                                    message.duration_s if message.duration_specified else None)
            drive = adapt_motion(request, self.capabilities)
            self.velocity, self.until, self.epoch = (drive.linear_mps, drive.angular_rps), now + drive.duration_s, message.authority_epoch
            result.success, result.message = True, '已接受运动请求'
        except (ValueError, TypeError) as exc:
            self.velocity, self.until = (0., 0.), 0.
            result.success, result.message = False, str(exc)
        result.command_mode = self.state.get('command_mode', '')
        result.guard_mode = self.state.get('mode', '')
        self.results.publish(result)
        self.tts.publish(String(data=result.message))

    def tick(self):
        now = time.monotonic()
        if (not self.authorized(now) or self.state.get('authority_epoch') != self.epoch or now >= self.until):
            self.velocity = (0., 0.)
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.twist.linear.x, message.twist.angular.z = self.velocity
        # Always publish fresh zero for the manual arming handshake. Guard
        # selects its source and discards these messages outside EXTERNAL.
        self.pub.publish(message)

    def destroy_node(self):
        self.velocity = (0., 0.)
        self.until = 0.
        self.tick()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotionAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
