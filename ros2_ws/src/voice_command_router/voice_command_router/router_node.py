"""Translate validated tool calls into narrowly scoped hardware topics."""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

from .validation import CommandValidationError, parse_buzz_command


class VoiceCommandRouter(Node):
    def __init__(self):
        super().__init__('voice_command_router')
        self.declare_parameter('input_topic', '/voice/tool_call')
        self.declare_parameter('buzzer_topic', '/hardware/buzzer_duration_ms')
        self._buzzer_pub = self.create_publisher(
            Int32, self.get_parameter('buzzer_topic').value, 10)
        self._result_pub = self.create_publisher(String, '/voice/tool_result', 10)
        self._subscription = self.create_subscription(
            String, self.get_parameter('input_topic').value, self._on_tool_call, 10)

    def _publish_result(self, request_id, success, detail):
        result = String()
        result.data = json.dumps({
            'request_id': request_id,
            'success': bool(success),
            'detail': detail,
        }, ensure_ascii=False)
        self._result_pub.publish(result)

    def _on_tool_call(self, message):
        request_id = ''
        try:
            request_id, duration_ms = parse_buzz_command(message.data)
            command = Int32()
            command.data = duration_ms
            self._buzzer_pub.publish(command)
            self._publish_result(request_id, True, '蜂鸣器命令已通过白名单校验并发布')
            self.get_logger().info(f'发布蜂鸣器命令: {duration_ms} ms')
        except CommandValidationError as exc:
            self._publish_result(request_id, False, str(exc))
            self.get_logger().warning(f'拒绝工具调用: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandRouter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
