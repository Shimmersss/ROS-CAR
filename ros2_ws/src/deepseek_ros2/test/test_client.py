import pytest

from deepseek_ros2.client import (
    DeepSeekError,
    extract_assistant_message,
    extract_assistant_text,
)


def test_extract_assistant_text():
    payload = {'choices': [{'message': {'content': '  你好  '}}]}
    assert extract_assistant_text(payload) == '你好'


def test_extract_assistant_text_rejects_malformed_response():
    with pytest.raises(DeepSeekError):
        extract_assistant_text({'choices': []})


def test_extract_tool_call_message():
    payload = {'choices': [{'message': {
        'content': None,
        'tool_calls': [{'function': {'name': 'buzz', 'arguments': '{}'}}],
    }}]}
    assert extract_assistant_message(payload)['tool_calls'][0]['function']['name'] == 'buzz'
