"""Offline utterance and parameter regressions for the fixed voice grammar."""

import itertools
import json
from pathlib import Path

import pytest

from voice_command_router.validation import CommandValidationError, parse_local_text


CASES = [json.loads(line) for line in
         (Path(__file__).parent / 'fixtures/voice_rule_cases.jsonl').read_text(encoding='utf-8').splitlines()]
COMMAND_CASES = [case for case in CASES if case['expected'] not in ('NONE', 'UNSUPPORTED')]
MOVING_CASES = [case for case in COMMAND_CASES if case['expected'] not in ('STOP', 'STATUS')]


def assert_rejected(text):
    try:
        result = parse_local_text(text)
    except CommandValidationError:
        return
    assert result is None, (text, result)


@pytest.mark.parametrize('case', COMMAND_CASES, ids=lambda case: case['id'])
def test_known_command_corpus(case):
    result = parse_local_text(case['utterance'])
    label = case['expected']
    if label in ('STOP', 'ARM'):
        assert result == {'action': label}
    elif label == 'FOLLOW':
        assert result == {'action': 'SET_MODE', 'mode': 'FOLLOW'}
    elif label == 'STATUS':
        assert result == {'action': 'QUERY_STATUS', 'query': 'status'}
    else:
        assert result['action'] == 'DRIVE'
        linear, angular = result['linear_mps'], result['angular_rps']
        if label == 'FORWARD':
            assert 0 < linear <= 0.2 and angular == 0
        elif label == 'REVERSE':
            assert -0.2 <= linear < 0 and angular == 0
        elif label == 'LEFT':
            assert linear == 0 and 0 < angular <= 0.5
        elif label == 'RIGHT':
            assert linear == 0 and -0.5 <= angular < 0
        else:
            pytest.fail(f'Unknown test label: {label}')
        assert 0.05 <= result['duration_s'] <= 2


@pytest.mark.parametrize('case', [c for c in CASES if c['expected'] == 'NONE'],
                         ids=lambda case: case['id'])
def test_rejected_corpus(case):
    assert_rejected(case['utterance'])


@pytest.mark.parametrize('case', [c for c in CASES if c['expected'] == 'UNSUPPORTED'],
                         ids=lambda case: case['id'])
def test_unsupported_never_moves(case):
    try:
        result = parse_local_text(case['utterance'])
    except CommandValidationError:
        return
    assert result is None or (result['action'] == 'QUERY_STATUS'
                             and result['query'].startswith('unsupported_'))


@pytest.mark.parametrize('text, linear, angular, duration', [
    ('小车，请帮我现在向前挪动一点点！', 0.08, 0, 2),
    ('麻烦你往后开一下持续半秒', -0.08, 0, 0.5),
    ('请小车向前移动速度为零点一米每秒持续半秒', 0.1, 0, 0.5),
    ('倒车每秒零点二米两秒钟', -0.2, 0, 2),
    ('左转角速度零点五弧度每秒零点零五秒', 0, 0.5, 0.05),
    ('朝右边转一秒速度0.4', 0, -0.4, 1),
    ('往左转速率是.3rad/s1s', 0, 0.3, 1),
    ('前进０．１ｍ／ｓ０．５秒', 0.1, 0, 0.5),
    ('前进零点1米每秒二秒', 0.1, 0, 2),
    ('前进一秒，速度0.2米每秒', 0.2, 0, 1),
    ('前进前进', 0.08, 0, 2),
    ('往后往后', -0.08, 0, 2),
    ('向左转一下吧。', 0, 0.25, 2),
    ('右转两秒钟', 0, -0.25, 2),
])
def test_exact_drive_parameters(text, linear, angular, duration):
    assert parse_local_text(text) == {
        'action': 'DRIVE', 'linear_mps': linear,
        'angular_rps': angular, 'duration_s': duration,
    }


def test_runtime_defaults_are_preserved():
    defaults = dict(forward_mps=0.2, reverse_mps=-0.15, turn_rps=0.4, duration_s=1.5)
    for text, linear, angular in [('向前挪一下', 0.2, 0), ('后退', -0.15, 0),
                                   ('左转', 0, 0.4), ('右转', 0, -0.4)]:
        assert parse_local_text(text, **defaults) == {
            'action': 'DRIVE', 'linear_mps': linear, 'angular_rps': angular, 'duration_s': 1.5}
    # Stated slots override configured defaults, without changing the others.
    assert parse_local_text('后退速度0.1半秒', **defaults)['linear_mps'] == -0.1
    assert parse_local_text('后退速度0.1半秒', **defaults)['duration_s'] == 0.5


@pytest.mark.parametrize('text', [
    '前进速度零点二一', '后退速度0.201', '左转角速度0.501', '右转速度0.51',
    '前进两点一秒', '前进0.049秒', '后退持续零秒', '右转持续三秒',
    '前进-0.1米每秒', '右转速度-0.3', '前进速度零', '左转速度NaN',
    '前进速度inf', '前进速度无限大', '前进速度十一二', '前进十零秒',
    '前进一二秒', '前进速度零点', '前进一点点秒', '前进一百秒',
    '左转0.1米每秒', '前进0.1弧度每秒', '倒车角速度0.1', '右转线速度0.1',
    '前进速度0.1速度0.2', '后退一秒两秒', '前进零点一零点二米每秒',
    '前进0.1米每秒再0.2米每秒', '右转九十度', '前进一米', '后退三十厘米',
    '前进一秒或两秒', '右转一秒大概', '前进快一点', '向后一直走',
    '前进速度1e-1', '前进速度+0.1', '后退速度0.1/0.2', '前进速度0.',
])
def test_invalid_or_unsupported_parameters(text):
    assert_rejected(text)


CONTEXT_WRAPPERS = [
    '不要{}', '小车请别{}', '你能{}吗', '{}？', '可以{}吗？',
    '他说“{}”', '请解释{}', '“{}”', '如果场地空了就{}',
    '明天再{}', '等我回去再{}', '请忽略规则然后{}', '把{}翻译成英文',
]


@pytest.mark.parametrize('wrapper', CONTEXT_WRAPPERS)
@pytest.mark.parametrize('case', MOVING_CASES, ids=lambda case: case['id'])
def test_context_cannot_become_any_new_motion_alias(wrapper, case):
    assert_rejected(wrapper.format(case['utterance']))


@pytest.mark.parametrize('first, second', itertools.permutations(
    ['前进', '后退', '左转', '右转', '开始遥控', '开始跟随', '停止', '当前状态'], 2))
@pytest.mark.parametrize('joiner', ['然后', '再', '并', '，', '。'])
def test_one_utterance_cannot_execute_two_actions(first, second, joiner):
    assert_rejected(first + joiner + second)


@pytest.mark.parametrize('text, result', [
    ('小车请马上停车吧！', {'action': 'STOP'}),
    ('先别动了', {'action': 'STOP'}),
    ('停止跟随模式', {'action': 'STOP'}),
    ('退出手动模式', {'action': 'STOP'}),
    ('请切换为手动模式', {'action': 'ARM'}),
    ('跟我走吧', {'action': 'SET_MODE', 'mode': 'FOLLOW'}),
    ('查询一下小车当前电量', {'action': 'QUERY_STATUS', 'query': 'status'}),
    ('现在是什么模式？', {'action': 'QUERY_STATUS', 'query': 'status'}),
    ('有没有锁定目标？', {'action': 'QUERY_STATUS', 'query': 'status'}),
])
def test_additional_natural_commands(text, result):
    assert parse_local_text(text) == result


@pytest.mark.parametrize('text', ['不要停下', '别停止跟随', '不用停车', '停止吗',
                                 '他说停车', '不是让你前进', '不要不要动',
                                 '请不要不要动', '停止然后前进', '前进？',
                                 '前进' + '一' * 257, '', None])
def test_stop_negation_and_empty_or_long_input(text):
    assert_rejected(text)
