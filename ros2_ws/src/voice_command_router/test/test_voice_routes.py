"""A direct typed voice command cannot reach the retained buzzer adapter."""

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('roscar_interfaces.msg')

import rclpy
from roscar_interfaces.msg import VoiceCommand
from voice_command_router.router_node import VoiceCommandRouter


def test_typed_buzz_is_rejected_without_buzzer_publisher():
    rclpy.init()
    node = VoiceCommandRouter()
    try:
        assert not hasattr(node, '_buzzer_pub')
        results = []
        node._publish_result = lambda command, success, message: results.append((success, message))
        command = VoiceCommand()
        command.stamp = node.get_clock().now().to_msg()
        command.request_id = 'rejected-buzz'
        command.action = 'BUZZ'
        command.duration_s = 0.3
        node._on_command(command)
        assert len(results) == 1
        assert results[0][0] is False
        assert '未知语音动作' in results[0][1]
    finally:
        node.destroy_node()
        rclpy.shutdown()
