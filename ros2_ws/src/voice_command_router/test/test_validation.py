import json

import pytest

from voice_command_router.validation import (CommandValidationError,
    parse_buzz_command, parse_local_text, validate_drive)


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


@pytest.mark.parametrize('text, expected', [
    ('前进', ('DRIVE', 0.08, 0.0)),
    ('后退一秒', ('DRIVE', -0.08, 0.0)),
    ('左转', ('DRIVE', 0.0, 0.25)),
    ('右转速度0.4两秒', ('DRIVE', 0.0, -0.4)),
    ('后退0.1米每秒', ('DRIVE', -0.1, 0.0)),
])
def test_local_parser(text, expected):
    command = parse_local_text(text)
    assert command['action'] == expected[0]
    assert command['linear_mps'] == pytest.approx(expected[1])
    assert command['angular_rps'] == pytest.approx(expected[2])


def test_local_parser_modes_and_fallback():
    assert parse_local_text('开始遥控')['action'] == 'ARM'
    assert parse_local_text('开始跟随') == {'action': 'SET_MODE', 'mode': 'FOLLOW'}
    assert parse_local_text('导航去客厅')['query'] == 'unsupported_navigation'
    assert parse_local_text('介绍一下你自己') is None


@pytest.mark.parametrize('text, action, duration', [
    ('往前走', 'DRIVE', 2.0),
    ('请小车向前走一点', 'DRIVE', 1.0),
    ('再左转一点', 'DRIVE', 1.0),
    ('停一下', 'STOP', None),
    ('急停', 'DISARM', None),
    ('退出语音控制', 'DISARM', None),
    ('好的，小车停。', 'STOP', None),
])
def test_simple_session_phrases(text, action, duration):
    parsed = parse_local_text(text)
    assert parsed['action'] == action
    if duration is not None:
        assert parsed['duration_s'] == duration


@pytest.mark.parametrize('text', ['不要前进', '能不能左转', '前进还是后退'])
def test_ambiguous_motion_is_not_executed(text):
    with pytest.raises(CommandValidationError):
        parse_local_text(text)


def test_drive_validation_allows_bounded_reverse():
    assert validate_drive(-0.2, 0.0, 2.0).linear_mps == -0.2
    with pytest.raises(CommandValidationError):
        validate_drive(-0.201, 0.0, 1.0)
    with pytest.raises(CommandValidationError):
        validate_drive(0.0, 0.0, 1.0)
