import json

import pytest

from voice_command_router.validation import CommandValidationError, parse_buzz_command


def test_accepts_bounded_buzz_command():
    raw = json.dumps({
        'request_id': 'request-1',
        'name': 'buzz',
        'arguments': {'duration_ms': 300},
    })
    assert parse_buzz_command(raw) == ('request-1', 300)


@pytest.mark.parametrize('duration', [0, 99, 2001, True, '300'])
def test_rejects_unsafe_duration(duration):
    raw = json.dumps({
        'request_id': 'request-1',
        'name': 'buzz',
        'arguments': {'duration_ms': duration},
    })
    with pytest.raises(CommandValidationError):
        parse_buzz_command(raw)


def test_rejects_unknown_tool():
    raw = json.dumps({
        'request_id': 'request-1',
        'name': 'cmd_vel',
        'arguments': {},
    })
    with pytest.raises(CommandValidationError):
        parse_buzz_command(raw)
