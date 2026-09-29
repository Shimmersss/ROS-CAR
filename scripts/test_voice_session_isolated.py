#!/usr/bin/env python3
"""Exercise voice authorization with fake guard services in an isolated ROS domain."""

import json
import os
import time

if os.environ.get('ROS_DOMAIN_ID') in (None, '', '182') or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise SystemExit('必须在非正式 ROS_DOMAIN_ID 且 ROS_LOCALHOST_ONLY=1 下运行')

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from roscar_interfaces.msg import VoiceCommandResult
from roscar_interfaces.srv import SetControlMode
from voice_command_router.router_node import VoiceCommandRouter


class FakeGuard(Node):
    def __init__(self):
        super().__init__('fake_voice_guard')
        self.command_mode = 'IDLE'
        self.mode = 'STANDBY'
        self.arm_calls = self.stop_calls = self.disarm_calls = 0
        self.confirm_arm = True
        self.velocities = []
        self.results = []
        self.sessions = []
        self.state_pub = self.create_publisher(String, '/control/state', 10)
        self.wake_pub = self.create_publisher(String, '/voice_words', 10)
        self.asr_pub = self.create_publisher(String, '/voice/asr_text', 10)
        self.create_service(SetControlMode, '/control/set_mode', self._set_mode)
        self.create_service(Trigger, '/control/arm', self._arm)
        self.create_service(Trigger, '/control/stop', self._stop)
        self.create_service(Trigger, '/control/disarm', self._disarm)
        self.create_subscription(TwistStamped, '/chassis/cmd_vel',
                                 lambda msg: self.velocities.append(
                                     (msg.twist.linear.x, msg.twist.angular.z)), 10)
        self.create_subscription(VoiceCommandResult, '/voice/command_result',
                                 lambda msg: self.results.append(msg), 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/voice/session_active',
                                 lambda msg: self.sessions.append(msg.data), qos)
        self.create_timer(0.05, self._publish_state)

    def _publish_state(self):
        self.state_pub.publish(String(data=json.dumps({
            'command_mode': self.command_mode, 'mode': self.mode, 'ready': True,
        })))

    def _set_mode(self, request, response):
        self.command_mode = request.mode
        response.success = True
        response.message = 'mode set'
        return response

    def _arm(self, request, response):
        self.arm_calls += 1
        if self.confirm_arm:
            self.mode = 'ARMED'
        response.success = True
        response.message = 'armed'
        return response

    def _stop(self, request, response):
        self.stop_calls += 1
        self.mode = 'STANDBY'
        response.success = True
        response.message = 'stopped'
        return response

    def _disarm(self, request, response):
        self.disarm_calls += 1
        self.mode = 'STANDBY'
        response.success = True
        response.message = 'disarmed'
        return response


def main():
    rclpy.init()
    router = VoiceCommandRouter()
    fake = FakeGuard()
    executor = SingleThreadedExecutor()
    executor.add_node(router)
    executor.add_node(fake)

    def pump(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)

    def send(text):
        fake.asr_pub.publish(String(data=text))
        pump(0.1)

    try:
        pump(0.5)
        fake.wake_pub.publish(String(data='小车唤醒'))
        pump(0.2)
        assert fake.sessions[-1] is True
        send('前进')
        pump(1.0)
        assert fake.arm_calls == 1, fake.arm_calls
        assert any(r.action == 'DRIVE' and r.success for r in fake.results)
        assert any(linear > 0 for linear, _ in fake.velocities)

        previous_stop_calls = fake.stop_calls
        fake.wake_pub.publish(String(data='小车唤醒'))
        pump(0.2)
        assert fake.sessions[-1] is True
        assert fake.stop_calls == previous_stop_calls

        send('停一下')
        pump(0.3)
        assert fake.stop_calls > previous_stop_calls
        assert fake.sessions[-1] is True
        send('左转')
        pump(1.0)
        assert fake.arm_calls == 2
        assert any(angular > 0 for _, angular in fake.velocities)

        send('急停')
        pump(0.3)
        assert fake.disarm_calls >= 1
        assert fake.sessions[-1] is False
        previous_arm_calls = fake.arm_calls
        send('前进')
        pump(0.3)
        assert fake.arm_calls == previous_arm_calls

        fake.confirm_arm = False
        fake.wake_pub.publish(String(data='小车唤醒'))
        pump(0.2)
        previous_velocity_count = len(fake.velocities)
        send('前进')
        pump(2.0)
        assert fake.arm_calls == previous_arm_calls + 1
        assert not any(linear > 0 or angular != 0
                       for linear, angular in fake.velocities[previous_velocity_count:])
        assert any(r.action == 'DRIVE' and not r.success and '未确认' in r.message
                   for r in fake.results)
        router.set_parameters([Parameter('session_timeout_s', value=0.5)])
        fake.wake_pub.publish(String(data='小车唤醒'))
        pump(0.8)
        assert fake.sessions[-1] is False
        print('PASS: 自动授权、重复唤醒、停止后重授权、急停与超时退出、会话外拒绝、授权状态未确认停车')
    finally:
        executor.remove_node(fake)
        executor.remove_node(router)
        fake.destroy_node()
        router.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
