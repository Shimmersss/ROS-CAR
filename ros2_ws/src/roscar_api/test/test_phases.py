from roscar_api.phases import PhaseTransitions, OWNER_TEXT


def test_owner_announces_transitions_once_and_allows_new_owner():
    phases = PhaseTransitions(owner_loss_delay_s=0)
    sequence = [('UNCLAIMED', 'ready'), ('TRACKING', 'claimed'), ('LOST', 'lost'),
                ('TRACKING', 'returned'), ('LOST', 'lost'), ('UNCLAIMED', 'expired'),
                ('TRACKING', 'claimed')]
    for number, (state, event) in enumerate(sequence):
        assert phases.owner(state, event, number) == OWNER_TEXT[event]
        assert phases.owner(state, event, number) is None
    assert all(len(text) <= 14 for text in OWNER_TEXT.values())


def test_brief_owner_loss_and_return_are_silent_but_sustained_loss_is_announced():
    phases = PhaseTransitions()
    assert phases.owner('TRACKING', 'claimed', 1, now=0) == OWNER_TEXT['claimed']
    assert phases.owner('LOST', 'lost', 2, now=1) is None
    assert phases.owner('LOST', 'lost', 2, now=1.49) is None
    assert phases.owner('TRACKING', 'returned', 3, now=1.49) is None
    assert phases.owner('LOST', 'lost', 4, now=2) is None
    assert phases.owner('LOST', 'lost', 4, now=2.5) == OWNER_TEXT['lost']
    assert phases.owner('LOST', 'lost', 4, now=3) is None
    assert phases.owner('TRACKING', 'returned', 5, now=3.1) == OWNER_TEXT['returned']
    assert phases.owner('TRACKING', 'returned', 5, now=3.2) is None


def test_expiry_and_release_are_immediate_and_reset_loss_announcement():
    phases = PhaseTransitions()
    assert phases.owner('LOST', 'lost', 1, now=0) is None
    assert phases.owner('LOST', 'lost', 1, now=.5) == OWNER_TEXT['lost']
    assert phases.owner('UNCLAIMED', 'expired', 2, now=10) == OWNER_TEXT['expired']
    assert phases.owner('TRACKING', 'claimed', 3, now=10.1) == OWNER_TEXT['claimed']
    assert phases.owner('LOST', 'lost', 4, now=11) is None
    assert phases.owner('UNCLAIMED', 'released', 5, now=11.1) == OWNER_TEXT['released']
    assert phases.owner('LOST', 'lost', 6, now=12) is None
    assert phases.owner('LOST', 'lost', 6, now=12.49) is None
    assert phases.owner('LOST', 'lost', 6, now=12.5) == OWNER_TEXT['lost']


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


def test_fall_stage_edges_not_ids_or_invalid_gaps_control_repeated_speech():
    from roscar_api.phases import FallTransitions,FALL_TEXT
    falls=FallTransitions()
    assert falls.update(True,1,now=0)==FALL_TEXT[1]
    assert falls.update(True,1,now=.1) is None
    assert falls.update(True,2,now=.2)==FALL_TEXT[2]
    assert falls.update(False,0,now=4) is None
    assert falls.update(True,2,now=5) is None
    assert falls.update(True,1,now=6) is None
    assert falls.update(True,0,now=7) is None
    assert falls.update(True,0,now=8.9) is None
    assert falls.update(True,2,now=8.95) is None  # brief zero was not recovery
    falls.update(True,0,now=10)
    falls.update(False,0,now=11)
    falls.update(True,0,now=12)
    assert falls.update(True,2,now=13) is None
    falls.update(True,0,now=14)
    falls.update(True,0,now=16)
    assert falls.update(True,2,now=16.1)==FALL_TEXT[2]
