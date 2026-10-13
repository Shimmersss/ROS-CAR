from roscar_api.phases import PhaseTransitions, OWNER_TEXT


def test_owner_announces_transitions_once_and_allows_new_owner():
    phases = PhaseTransitions()
    sequence = [('UNCLAIMED', 'ready'), ('TRACKING', 'claimed'), ('LOST', 'lost'),
                ('TRACKING', 'returned'), ('LOST', 'lost'), ('UNCLAIMED', 'expired'),
                ('TRACKING', 'claimed')]
    for number, (state, event) in enumerate(sequence):
        assert phases.owner(state, event, number) == OWNER_TEXT[event]
        assert phases.owner(state, event, number) is None
    assert all(len(text) <= 14 for text in OWNER_TEXT.values())


def test_follow_does_not_repeat_during_target_recovery_and_stop_is_announced():
    phases = PhaseTransitions()
    armed = dict(command_mode='FOLLOW', authority_state='ARMED_FOLLOW')
    assert phases.control(armed) == '开始跟随'
    assert phases.control(armed) is None
    assert phases.control(dict(command_mode='FOLLOW', authority_state='FAULT', last_fault='target_invalid')) is None
    assert phases.control(armed) is None
    assert phases.control(dict(command_mode='IDLE', authority_state='DISARMED')) == '跟随已暂停'
    assert phases.control(armed) == '开始跟随'
    assert phases.control(dict(command_mode='IDLE', authority_state='ESTOP')) == '已紧急制动'
