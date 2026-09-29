"""The active language-model tool list must not expose the retained buzzer code."""

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('roscar_interfaces.msg')

from deepseek_ros2.chat_node import DeepSeekChatNode


def test_control_tools_remain_but_buzz_is_unavailable():
    names = {item['function']['name'] for item in DeepSeekChatNode._tools()}
    assert names == {'drive', 'set_control_mode', 'arm', 'stop', 'query_status'}
    with pytest.raises(RuntimeError, match='拒绝未知工具'):
        DeepSeekChatNode._handle_tool_calls(object(), {
            'tool_calls': [{'function': {'name': 'buzz', 'arguments': '{"duration_ms":300}'}}],
        })
