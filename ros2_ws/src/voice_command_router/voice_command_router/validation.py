"""Dependency-free validation for untrusted model and voice commands."""

import json
import math
import re
import unicodedata
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
    """Decode only well-formed unsigned numbers; never guess missing digits."""
    if re.fullmatch(r'(?:[0-9]+(?:\.[0-9]+)?|\.[0-9]+)', value):
        return float(value)
    if value == '半':
        return 0.5
    integer, separator, fraction = value.partition('点')
    if integer in _ZH_DIGITS:
        result = float(_ZH_DIGITS[integer])
    elif re.fullmatch(r'[一二两三四五六七八九]?十[一二三四五六七八九]?', integer):
        tens, ones = integer.split('十')
        result = float(_ZH_DIGITS.get(tens, 1) * 10 + _ZH_DIGITS.get(ones, 0))
    else:
        raise CommandValidationError('数字格式不明确，请说零点一或一秒等完整数字')
    if separator:
        if not fraction or any(x not in _ZH_DIGITS and x not in '0123456789' for x in fraction):
            raise CommandValidationError('小数格式不明确')
        digits = ''.join(str(_ZH_DIGITS[x]) if x in _ZH_DIGITS else x for x in fraction)
        result += float('0.' + digits)
    return result


_NUMBER = r'(?:[0-9]+(?:\.[0-9]+)?|\.[0-9]+|[零〇一二两三四五六七八九十]+(?:点[零〇一二两三四五六七八九0-9]+)?)'
_DURATION = rf'(?:持续|维持)?(?P<duration>{_NUMBER}|半)(?:秒钟|秒|s)'
_PREFIX = re.compile(r'^(?:麻烦你|麻烦|请你|请|帮我|小车|现在|马上|立即|先)')
_SOFT_SUFFIX = r'(?:一下|一点|一点点)?'
_DIRECTIONS = {
    'forward': r'(?:前进|(?:向|往|朝)前(?:面|方)?(?:短暂)?(?:移动|挪动|挪|走|行驶|开动|开)?|前)',
    'reverse': r'(?:后退|倒车|倒退|(?:向|往|朝)后(?:面|方)?(?:短暂)?(?:移动|挪动|挪|走|行驶|开动|开|倒)?|后)',
    'left': r'(?:左转|左拐|(?:向|往|朝)左(?:边|侧)?(?:短暂)?(?:转动|转弯|转|拐弯|拐|调整方向)?|左)',
    'right': r'(?:右转|右拐|(?:向|往|朝)右(?:边|侧)?(?:短暂)?(?:转动|转弯|转|拐弯|拐|调整方向)?|右)',
}
_STOP = re.compile(
    r'(?:停止|急停|停下|停下来|停住|停车|停一下|停一停|不要动|别动|别动了|不要动了|'
    r'(?:退出|取消|结束|停止)(?:(?:遥控|跟随|手动(?:驾驶|控制)?)(?:模式)?|当前动作))'
    r'(?:小车)?(?:吧)?')
_ARM = re.compile(
    r'(?:(?:开始|进入|开启|启动|切换(?:到|为|成)?|切到)?(?:遥控|手动(?:驾驶|控制|操控)?)(?:模式|状态|授权)?|'
    r'把小车(?:切换到|切到)(?:遥控|手动驾驶)模式|'
    r'接下来让我手动控制|我现在要接管驾驶)(?:吧)?')
_FOLLOW = re.compile(
    r'(?:(?:开始|进入|开启|启动|切换(?:到|为|成)?|切到)?(?:人体)?跟随(?:模式)?|'
    r'(?:开始)?跟随我|(?:开始)?跟着我(?:走)?|跟我走|跟上我|开始跟在我后面|'
    r'接下来跟随说话的人|开始跟踪并跟随我)(?:吧)?')
_STATUS = re.compile(
    r'(?:(?:查询|查看|报告|告诉我)(?:一下)?(?:小车|车辆)?(?:现在的|目前|当前|剩余)?'
    r'(?:电量|电池电压|控制模式|状态)|电量|电池|当前状态|控制状态|'
    r'(?:当前)?是否在跟随|(?:现在)?锁定了目标吗|有没有锁定目标|是否锁定目标|'
    r'(?:小车)?(?:现在)?(?:是|处于)什么模式|'
    r'(?:现在的)?控制模式|(?:剩余)?电量(?:是多少|多少|还有多少))')
# Questions about the state are explicitly handled above. Other context must
# survive normalization and cannot be stripped into an executable command.
_CONTEXT = re.compile(
    r'[?？"\'“”‘’:：]|不要|别|勿|不想|无需|不用|不能|可以|能不能|能否|是否|吗|么|'
    r'如果|假如|假设|要是|以后|明天|下周|等.+再|待.+再|有空|考虑|'
    r'他说|说了|说过|有人|口令是|示例|举例|文章|报告中|引用|解释|翻译|朗读|'
    r'忽略|规则|系统|管理员|白名单|提示词|返回|输出|伪造')


def _drive_pattern(direction):
    # A turn takes rad/s, translation takes m/s. Bare "速度" keeps the
    # historical direction-dependent unit, but explicit wrong units fail.
    linear = direction in ('forward', 'reverse')
    unit = r'(?:米每秒|米/秒|m/s)' if linear else r'(?:弧度每秒|弧度/秒|rad/s)'
    noun = '米' if linear else '弧度'
    label = r'(?:速度|速率|线速度)' if linear else r'(?:速度|速率|角速度)'
    speed = (rf'(?:以)?(?:{label}(?:为|是)?(?P<speed>{_NUMBER})(?:{unit})?|'
             rf'(?P<unit_speed>{_NUMBER}){unit}|每秒(?P<per_second>{_NUMBER}){noun})'
             r'(?:的速度)?')
    # Two anchored alternatives allow either slot order, with each slot at
    # most once. No suffix search or partial action extraction is performed.
    head = _DIRECTIONS[direction] + _SOFT_SUFFIX
    return (
        re.compile(head + rf'(?:{speed})?(?:{_DURATION})?' + _SOFT_SUFFIX + r'(?:吧)?'),
        re.compile(head + rf'{_DURATION}{speed}' + _SOFT_SUFFIX + r'(?:吧)?'),
    )


_DRIVE_PATTERNS = {key: _drive_pattern(key) for key in _DIRECTIONS}


def parse_local_text(text, *, forward_mps=0.08, reverse_mps=-0.08,
                     turn_rps=0.25, duration_s=2.0):
    """Parse common Chinese driving phrases without network access.

    Returns a small action dictionary or ``None`` for ordinary conversation.
    A recognized but ambiguous command raises CommandValidationError.
    """
    normalized = unicodedata.normalize('NFKC', str(text or '')).lower()
    if len(normalized) > 256:
        raise CommandValidationError('指令过长，请一次只说一个动作')
    normalized = re.sub(r'[\s，。、,!！]', '', normalized)
    normalized = re.sub(r'(?<=[^0-9])\.$', '', normalized)
    # Prefixes can occur together ("小车，请帮我..."); only this closed
    # vocabulary is removed. Quotes and question marks are never discarded.
    while True:
        stripped = _PREFIX.sub('', normalized, count=1)
        if stripped == normalized:
            break
        normalized = stripped
    if _STOP.fullmatch(normalized):
        return {'action': 'STOP'}
    if _STATUS.fullmatch(normalized.rstrip('?？')):
        return {'action': 'QUERY_STATUS', 'query': 'status'}
    if _CONTEXT.search(normalized):
        raise CommandValidationError('指令包含否定、疑问或转述，请直接说明一个动作')
    if _ARM.fullmatch(normalized):
        return {'action': 'ARM'}
    if _FOLLOW.fullmatch(normalized):
        return {'action': 'SET_MODE', 'mode': 'FOLLOW'}
    if re.fullmatch(r'(?:导航)?(?:去|到|前往)(?:客厅|门口|厨房|充电点)(?:等我)?', normalized):
        return {'action': 'QUERY_STATUS', 'query': 'unsupported_navigation'}
    # Keep the old exact repeated-direction convention; never collapse a
    # repeated suffix inside a larger, possibly negated utterance.
    repeated = re.fullmatch(r'(前进|后退|倒车|倒退|左转|右转|向前|往前|向后|往后|向左|左拐|向右|右拐|前|后|左|右)\1+', normalized)
    if repeated:
        normalized = repeated[1]
    for direction, patterns in _DRIVE_PATTERNS.items():
        match = next((m for pattern in patterns if (m := pattern.fullmatch(normalized))), None)
        if match:
            break
    else:
        if re.search(r'前|后|左|右|遥控|手动|跟随|跟着|停止|停车|倒车|导航', normalized):
            raise CommandValidationError('指令包含否定、疑问或不明确的参数，请明确说明一个动作')
        return None
    slots = match.groupdict()
    speed_text = slots['speed'] or slots['unit_speed'] or slots['per_second']
    speed = _number(speed_text) if speed_text is not None else None
    duration = _number(slots['duration']) if slots['duration'] else duration_s
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
