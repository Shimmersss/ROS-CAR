import json
import random

import pytest

from motion_guard.authority import AuthorityError, MotionAuthority


def test_default_and_explicit_arm():
    a = MotionAuthority()
    assert (a.state, a.command_mode, a.armed) == ('DISARMED', 'IDLE', False)
    with pytest.raises(AuthorityError):
        a.arm(True, True)
    for mode, state in [('EXTERNAL', 'ARMED_MANUAL'), ('FOLLOW', 'ARMED_FOLLOW')]:
        a.select_mode(mode)
        for ready, enabled in [(False, True), (True, False)]:
            with pytest.raises(AuthorityError):
                a.arm(ready, enabled)
        a.arm(True, True)
        assert a.state == state
        a.stop()
        assert a.state == 'DISARMED' and a.command_mode == 'IDLE'


def test_estop_latches_across_commands_stop_fault_and_restart(tmp_path):
    path = tmp_path / 'estop.json'
    a = MotionAuthority(path)
    a.select_mode('EXTERNAL')
    a.arm(True, True)
    a.estop()
    epoch = a.epoch
    a.stop()
    a.fault('audio_failed')
    assert a.state == 'ESTOP' and not a.armed and a.epoch > epoch
    for operation in [lambda: a.arm(True, True), lambda: a.select_mode('FOLLOW'),
                      lambda: a.reset_estop(False), lambda: a.reset_fault(True)]:
        with pytest.raises(AuthorityError):
            operation()
    restarted = MotionAuthority(path)
    assert restarted.state == 'ESTOP'
    restarted.reset_estop(True)
    assert restarted.state == 'DISARMED' and not restarted.armed
    assert MotionAuthority(path).state == 'DISARMED'
    restarted.select_mode('FOLLOW')
    restarted.arm(True, True)
    assert restarted.state == 'ARMED_FOLLOW'


def test_fault_requires_reset_and_new_mode():
    a = MotionAuthority()
    a.select_mode('FOLLOW')
    a.arm(True, True)
    a.fault('stale_target')
    for operation in [lambda: a.arm(True, True), lambda: a.select_mode('EXTERNAL'),
                      lambda: a.reset_fault(False)]:
        with pytest.raises(AuthorityError):
            operation()
    a.stop()
    assert a.state == 'FAULT'
    a.reset_fault(True)
    assert a.command_mode == 'IDLE' and not a.armed


@pytest.mark.parametrize('contents', ['bad json', '{}', '{"estop": 0}', '{"estop": true}'])
def test_invalid_or_latched_storage_fails_closed(tmp_path, contents):
    path = tmp_path / 'latch.json'
    path.write_text(contents)
    assert MotionAuthority(path).state == 'ESTOP'


def test_persistence_failure_stays_estopped(tmp_path):
    path = tmp_path / 'directory'
    path.mkdir()
    a = MotionAuthority(path)
    with pytest.raises(AuthorityError):
        a.reset_estop(True)
    assert a.state == 'ESTOP' and not a.armed


def test_random_events_never_clear_latch_or_arm_without_request():
    rng = random.Random(42)
    a = MotionAuthority()
    for _ in range(2000):
        operation = rng.choice(['select', 'arm', 'stop', 'fault', 'estop', 'reset_estop', 'reset_fault'])
        before = a.state
        try:
            if operation == 'select':
                a.select_mode(rng.choice(['IDLE', 'FOLLOW', 'EXTERNAL']))
            elif operation == 'arm':
                a.arm(rng.choice([True, False]), rng.choice([True, False]))
            elif operation == 'stop':
                a.stop()
            elif operation == 'fault':
                a.fault('test')
            elif operation == 'estop':
                a.estop()
            elif operation == 'reset_estop':
                a.reset_estop(True)
            else:
                a.reset_fault(True)
        except AuthorityError:
            pass
        if a.armed and before != a.state:
            assert operation == 'arm' and before == 'DISARMED'
        if before == 'ESTOP' and operation != 'reset_estop':
            assert a.state == 'ESTOP'


def test_restart_does_not_reuse_authorization_epoch(tmp_path):
    first = MotionAuthority(tmp_path / 'latch.json')
    first.select_mode('EXTERNAL')
    second = MotionAuthority(tmp_path / 'latch.json')
    second.select_mode('EXTERNAL')
    assert first.epoch != second.epoch
    assert 0 <= second.epoch < 2**64
