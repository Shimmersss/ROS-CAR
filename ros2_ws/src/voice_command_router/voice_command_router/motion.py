"""Chassis-independent motion intent and an explicit capability adapter."""
from dataclasses import dataclass
import math
from .validation import CommandValidationError, validate_drive


@dataclass(frozen=True)
class MotionRequest:
    action: str
    direction: str
    style: str = 'NORMAL'
    speed: float | None = None
    duration_s: float | None = None


@dataclass(frozen=True)
class ChassisCapabilities:
    chassis: str = 'ackermann'
    turn_calibrated: bool = False
    micro_calibrated: bool = False
    forward_mps: float = 0.2
    reverse_mps: float = 0.2
    turn_linear_mps: float = 0.08
    turn_rps: float = 0.2
    min_turn_radius_m: float = 0.4
    duration_s: float = 2.0
    micro_mps: float = 0.05
    micro_turn_rps: float = 0.1
    micro_duration_s: float = 0.5

    def __post_init__(self):
        if self.chassis not in ('ackermann', 'differential'):
            raise CommandValidationError('未知底盘能力')
        if type(self.turn_calibrated) is not bool or type(self.micro_calibrated) is not bool:
            raise CommandValidationError('标定开关必须是布尔值')
        for name, value in vars(self).items():
            if name not in ('chassis', 'turn_calibrated', 'micro_calibrated'):
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                    raise CommandValidationError('底盘能力参数必须为有限正数')
        if (max(self.forward_mps, self.reverse_mps, self.turn_linear_mps, self.micro_mps) > 0.2
                or max(self.turn_rps, self.micro_turn_rps) > 0.5
                or not 0.05 <= self.duration_s <= 2
                or not 0.4 <= self.micro_duration_s <= 0.7):
            raise CommandValidationError('底盘能力参数超出软件边界')


def adapt_motion(request, capabilities):
    c, r = capabilities, request
    for value in (r.speed, r.duration_s) if isinstance(r, MotionRequest) else ():
        if value is not None and (isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0):
            raise CommandValidationError('显式运动参数必须为有限正数')
    if not isinstance(r, MotionRequest) or r.action not in ('MOVE', 'TURN'):
        raise CommandValidationError('无效运动意图')
    if r.style not in ('NORMAL', 'MICRO', 'IN_PLACE'):
        raise CommandValidationError('未知运动风格')
    if r.action == 'MOVE' and (r.direction not in ('FORWARD', 'REVERSE') or r.style == 'IN_PLACE'):
        raise CommandValidationError('直行方向或风格无效')
    if r.action == 'TURN' and r.direction not in ('LEFT', 'RIGHT'):
        raise CommandValidationError('转向方向无效')
    if r.style == 'IN_PLACE' and c.chassis == 'ackermann':
        raise CommandValidationError('当前底盘无法原地转向')
    if r.action == 'TURN' and c.chassis == 'ackermann' and not c.turn_calibrated:
        raise CommandValidationError('阿克曼转向参数尚未完成实车验收')
    micro = r.style == 'MICRO'
    if micro and not c.micro_calibrated:
        raise CommandValidationError('微动参数尚未完成实车验收')
    duration = r.duration_s if r.duration_s is not None else c.micro_duration_s if micro else c.duration_s
    if r.action == 'MOVE':
        speed = r.speed if r.speed is not None else c.micro_mps if micro else c.forward_mps if r.direction == 'FORWARD' else c.reverse_mps
        linear, angular = speed * (1 if r.direction == 'FORWARD' else -1), 0.
    else:
        speed = r.speed if r.speed is not None else c.micro_turn_rps if micro else c.turn_rps
        linear = (c.micro_mps if micro else c.turn_linear_mps) if c.chassis == 'ackermann' else 0.
        angular = speed * (1 if r.direction == 'LEFT' else -1)
        if c.chassis == 'ackermann' and speed > 0 and linear / speed < c.min_turn_radius_m - 1e-9:
            raise CommandValidationError('请求转弯半径小于已配置底盘能力')
    if r.speed is not None and (isinstance(r.speed, bool) or not isinstance(r.speed, (int, float)) or r.speed <= 0):
        raise CommandValidationError('速度必须是正数')
    if micro and (duration > 0.7 or (abs(linear) > c.micro_mps or abs(angular) > c.micro_turn_rps)):
        raise CommandValidationError('微动显式参数超过微动边界')
    return validate_drive(linear, angular, duration)
