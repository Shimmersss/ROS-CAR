"""Dependency-free validation for untrusted model and voice commands."""

import json
import math
import re
from dataclasses import dataclass


class CommandValidationError(ValueError):
    pass


@dataclass(frozen=True)
class DriveCommand:
    linear_mps: float
    angular_rps: float
    duration_s: float


MAX_FORWARD_MPS = 0.2
MAX_REVERSE_MPS = 0.2
MAX_ANGULAR_RPS = 0.5
MAX_DURATION_S = 2.0


def validate_drive(linear_mps, angular_rps, duration_s):
    values = (linear_mps, angular_rps, duration_s)
    if not all(isinstance(value, (int, float)) and not isinstance(value, bool)
               and math.isfinite(float(value)) for value in values):
        raise CommandValidationError('运动参数必须是有限数字')
    linear_mps = float(linear_mps)
    angular_rps = float(angular_rps)
    duration_s = float(duration_s)
    if not -MAX_REVERSE_MPS <= linear_mps <= MAX_FORWARD_MPS:
        raise CommandValidationError('线速度超出 -0.2 到 0.2 m/s 范围')
    if abs(angular_rps) > MAX_ANGULAR_RPS:
        raise CommandValidationError('角速度绝对值不能超过 0.5 rad/s')
    if not 0.05 <= duration_s <= MAX_DURATION_S:
        raise CommandValidationError('持续时间必须在 0.05 到 2 秒之间')
    if linear_mps == 0.0 and angular_rps == 0.0:
        raise CommandValidationError('运动命令不能同时为零，请使用停止命令')
    return DriveCommand(linear_mps, angular_rps, duration_s)


def validate_mode(mode):
    if mode not in {'IDLE', 'EXTERNAL', 'FOLLOW'}:
        raise CommandValidationError('控制模式必须是 IDLE、EXTERNAL 或 FOLLOW')
    return mode


_ZH_DIGITS = {'零': 0, '〇': 0, '一': 1, '二': 2, '两': 2, '三': 3,
              '四': 4, '五': 5, '六': 6, '七': 7, '八': 8, '九': 9}


def _number(value):
    if value is None:
        return None
    value = value.strip()
    try:
        return float(value)
    except ValueError:
        if '点' in value:
            integer, fraction = value.split('点', 1)
            if fraction and all(x in _ZH_DIGITS for x in fraction):
                return _number(integer) + float('0.' + ''.join(str(_ZH_DIGITS[x]) for x in fraction))
        if value in _ZH_DIGITS:
            return float(_ZH_DIGITS[value])
        if value.startswith('十'):
            return float(10 + (_ZH_DIGITS.get(value[1:], 0) if len(value) > 1 else 0))
        if '十' in value:
            left, right = value.split('十', 1)
            return float(_ZH_DIGITS.get(left, 1) * 10 + _ZH_DIGITS.get(right, 0))
    return None


def _extract(pattern, text):
    match = re.search(pattern, text, re.IGNORECASE)
    return _number(match.group(1)) if match else None


def parse_local_text(text, *, forward_mps=0.08, reverse_mps=-0.08,
                     turn_rps=0.25, duration_s=2.0):
    """Parse common Chinese driving phrases without network access.

    Returns a small action dictionary or ``None`` for ordinary conversation.
    A recognized but ambiguous command raises CommandValidationError.
    """
    normalized = re.sub(r'[\s，。！？!?、,]', '', str(text or '')).lower()
    normalized = re.sub(r'^(?:好的|请|小车|帮我|再|继续)+', '', normalized)
    common_phrases = {
        '往前走': '前进', '向前走': '前进',
        '往后走': '后退', '向后走': '后退',
        '往前走一点': '前进一秒', '向前走一点': '前进一秒',
        '前进一点': '前进一秒', '后退一点': '后退一秒',
        '左转一点': '左转一秒', '右转一点': '右转一秒',
    }
    normalized = common_phrases.get(normalized, normalized)
    aliases = {
        '停止': {'action': 'STOP'}, '停': {'action': 'STOP'},
        '停下': {'action': 'STOP'}, '停一下': {'action': 'STOP'},
        '不要动': {'action': 'STOP'},
        '急停': {'action': 'DISARM'}, '退出遥控': {'action': 'DISARM'},
        '取消遥控': {'action': 'DISARM'},
        '退出控制': {'action': 'DISARM'}, '退出语音控制': {'action': 'DISARM'},
        '开始遥控': {'action': 'ARM'}, '手动驾驶': {'action': 'ARM'},
        '手动控制': {'action': 'ARM'}, '进入手动': {'action': 'ARM'},
        '遥控模式': {'action': 'ARM'}, '进入遥控': {'action': 'ARM'},
        '开启遥控': {'action': 'ARM'}, '遥控': {'action': 'ARM'},
        '开始跟随': {'action': 'SET_MODE', 'mode': 'FOLLOW'},
        '跟随我': {'action': 'SET_MODE', 'mode': 'FOLLOW'},
        '进入跟随': {'action': 'SET_MODE', 'mode': 'FOLLOW'},
    }
    if normalized in aliases:
        return aliases[normalized]
    if normalized in ('电量', '电池', '当前状态', '控制状态', '是否在跟随'):
        return {'action': 'QUERY_STATUS', 'query': 'status'}
    if any(word in normalized for word in ('导航', '去客厅', '去门口', '目的地')):
        return {'action': 'QUERY_STATUS', 'query': 'unsupported_navigation'}
    directions = {'前进':'forward', '向前':'forward', '往前':'forward', '前':'forward',
                  '后退':'reverse', '倒车':'reverse', '倒退':'reverse',
                  '向后':'reverse', '往后':'reverse', '后':'reverse',
                  '左转':'left', '向左':'left', '左拐':'left', '左':'left',
                  '右转':'right', '向右':'right', '右拐':'right', '右':'right'}
    # A short pause can leave two repetitions in one ASR utterance. Treat
    # exact repeated direction words as one command; unrelated long text is
    # still rejected below instead of guessing a vehicle action.
    for token in sorted(directions, key=len, reverse=True):
        normalized = re.sub(rf'(?:{re.escape(token)}){{2,}}$', token, normalized)
    number = r'(?:[0-9]+(?:\.[0-9]+)?|[零〇一二两三四五六七八九十]+(?:点[零〇一二两三四五六七八九]+)?)'
    match = re.fullmatch('('+'|'.join(directions)+')'+
        r'(?:(?:速度|速率)('+number+r')|('+number+r')(?:米每秒|米/秒|m/s))?'+
        r'(?:('+number+r')(?:秒|s))?', normalized)
    if not match:
        if any(word in normalized for word in (*directions, '遥控', '跟随', '停止')):
            raise CommandValidationError('指令包含否定、疑问或不明确的参数，请明确说明一个动作')
        return None
    direction = directions[match[1]]
    speed = _number(match[2] or match[3])
    duration = _number(match[4]) if match[4] else float(duration_s)
    if direction == 'forward':
        linear, angular = (speed if speed is not None else forward_mps), 0.0
    elif direction == 'reverse':
        linear, angular = -(speed if speed is not None else abs(reverse_mps)), 0.0
    elif direction == 'left':
        linear, angular = 0.0, speed if speed is not None else abs(turn_rps)
    else:
        linear, angular = 0.0, -(speed if speed is not None else abs(turn_rps))
    command = validate_drive(linear, angular, duration)
    return {'action': 'DRIVE', 'linear_mps': command.linear_mps,
            'angular_rps': command.angular_rps, 'duration_s': command.duration_s}


def parse_buzz_command(raw):
    try:
        command = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise CommandValidationError('工具调用不是有效 JSON') from exc
    if not isinstance(command, dict):
        raise CommandValidationError('工具调用必须是 JSON 对象')
    if set(command) - {'request_id', 'name', 'arguments'}:
        raise CommandValidationError('工具调用包含未知字段')
    if command.get('name') != 'buzz':
        raise CommandValidationError('工具不在本地白名单中')
    request_id = command.get('request_id')
    if not isinstance(request_id, str) or not request_id or len(request_id) > 128:
        raise CommandValidationError('request_id 无效')
    arguments = command.get('arguments')
    if not isinstance(arguments, dict) or set(arguments) - {'duration_ms'}:
        raise CommandValidationError('buzz 参数无效')
    duration_ms = arguments.get('duration_ms', 300)
    if isinstance(duration_ms, bool) or not isinstance(duration_ms, int):
        raise CommandValidationError('duration_ms 必须是整数')
    if not 100 <= duration_ms <= 2000:
        raise CommandValidationError('duration_ms 必须在 100 到 2000 之间')
    return request_id, duration_ms
