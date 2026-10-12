import math
import pytest
from voice_command_router.validation import parse_intent, CommandValidationError
from voice_command_router.motion import MotionRequest, ChassisCapabilities, adapt_motion
from voice_command_router.session import VoiceSession


@pytest.mark.parametrize('text,direction', [('前进一下', 'FORWARD'), ('往后一点', 'REVERSE'), ('稍微左转', 'LEFT'), ('右转一下', 'RIGHT')])
def test_micro_is_semantic_not_twist(text, direction):
    intent = parse_intent(text)
    assert intent['direction'] == direction and intent['style'] == 'MICRO'
    assert 'linear_mps' not in intent and intent['speed'] is None


@pytest.mark.parametrize('text,action', [('急停', 'ESTOP'), ('不用了', 'END_SESSION'), ('退出语音', 'END_SESSION'), ('休眠', 'END_SESSION'), ('解除紧急制动', 'RESET_ESTOP'), ('复位故障', 'RESET_FAULT')])
def test_session_and_reset_commands(text, action):
    assert parse_intent(text)['action'] == action


@pytest.mark.parametrize('direction,sign', [('LEFT', 1), ('RIGHT', -1)])
def test_ackermann_turn_moves_on_curve(direction, sign):
    result = adapt_motion(MotionRequest('TURN', direction), ChassisCapabilities(turn_calibrated=True))
    assert result.linear_mps == .08 and result.angular_rps == sign * .2
    assert result.linear_mps / abs(result.angular_rps) >= .4 - 1e-9


@pytest.mark.parametrize('motion_request', [MotionRequest('TURN', 'LEFT'), MotionRequest('MOVE', 'FORWARD', 'MICRO'), MotionRequest('TURN', 'LEFT', 'IN_PLACE')])
def test_unvalidated_or_unsupported_capability_fails(motion_request):
    with pytest.raises(CommandValidationError):
        adapt_motion(motion_request, ChassisCapabilities())


def test_micro_parameters_are_bounded():
    caps = ChassisCapabilities(micro_calibrated=True, turn_calibrated=True)
    result = adapt_motion(MotionRequest('MOVE', 'REVERSE', 'MICRO'), caps)
    assert (result.linear_mps, result.duration_s) == (-.05, .5)
    for request in (MotionRequest('MOVE', 'FORWARD', 'MICRO', .1), MotionRequest('MOVE', 'FORWARD', 'MICRO', duration_s=1), MotionRequest('TURN', 'LEFT', speed=.5)):
        with pytest.raises(CommandValidationError):
            adapt_motion(request, caps)


@pytest.mark.parametrize('value', [0, -1, True, '0.1', math.nan, math.inf])
@pytest.mark.parametrize('slot', ['speed', 'duration_s'])
def test_malformed_numeric_slot_never_becomes_velocity(value, slot):
    with pytest.raises(CommandValidationError):
        adapt_motion(MotionRequest('MOVE', 'FORWARD', **{slot: value}), ChassisCapabilities())


def test_differential_capability_is_explicit():
    result = adapt_motion(MotionRequest('TURN', 'LEFT', 'IN_PLACE'), ChassisCapabilities(chassis='differential'))
    assert result.linear_mps == 0 and result.angular_rps > 0


def test_wake_refreshes_window_and_exit_closes_it():
    session = VoiceSession(8)
    assert not session.active(0)
    session.wake(0)
    session.wake(4)
    assert session.active(11.99) and not session.active(12)
    session.success(11)
    assert session.active(18)
    session.close()
    assert not session.active(18)


@pytest.mark.parametrize('window', [0, 4.9, 10.1, math.nan, math.inf])
def test_unbounded_session_rejected(window):
    with pytest.raises(ValueError):
        VoiceSession(window)
