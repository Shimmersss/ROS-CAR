"""Demo only: relative distance/angle motion, closed loop on odometry. No ROS imports.

The ROS node feeds odometry poses and publishes the returned velocity as a /chassis/cmd_vel
request; motion_guard stays the only /cmd_vel publisher and the final authority.
"""
from dataclasses import dataclass
import math

MOVE, TURN = 'MOVE', 'TURN'


class GoalRejected(ValueError):
    pass


@dataclass(frozen=True)
class Limits:
    # Software envelope; the guard's loaded safety config may be stricter and wins.
    max_distance_m: float = 2.0
    max_angle_deg: float = 180.0
    default_mps: float = 0.15
    max_mps: float = 0.2
    min_mps: float = 0.04             # below this the chassis may not move at all
    default_yaw_rps: float = 0.2       # Ackermann cap: turn_linear_mps / min_turn_radius_m = 0.25
    max_yaw_rps: float = 0.5
    min_yaw_rps: float = 0.08
    ackermann: bool = True
    turn_linear_mps: float = 0.1      # creep speed while arcing on Ackermann
    min_turn_radius_m: float = 0.4
    distance_tolerance_m: float = 0.03
    angle_tolerance_deg: float = 3.0
    slowdown_m: float = 0.15          # linear ramp-down zone before the target
    slowdown_deg: float = 20.0
    timeout_margin_s: float = 3.0
    timeout_factor: float = 2.0

    def __post_init__(self):
        for name, value in vars(self).items():
            if name == 'ackermann':
                if type(value) is not bool:
                    raise ValueError('ackermann must be bool')
            elif isinstance(value, bool) or not isinstance(value, (int, float)) \
                    or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be a finite positive number')
        if not self.min_mps <= self.default_mps <= self.max_mps:
            raise ValueError('need min_mps <= default_mps <= max_mps')
        if not self.min_yaw_rps <= self.default_yaw_rps <= self.max_yaw_rps:
            raise ValueError('need min_yaw_rps <= default_yaw_rps <= max_yaw_rps')


@dataclass(frozen=True)
class Goal:
    kind: str
    amount: float        # signed: m for MOVE, rad for TURN
    speed: float         # |m/s| for MOVE, |rad/s| for TURN
    reverse: bool        # TURN on Ackermann: arc backwards
    timeout_s: float


def make_goal(kind, distance_m=0., angle_deg=0., speed=0., timeout_s=0., reverse=False, limits=Limits()):
    """Validate a request; raise GoalRejected with a human readable (Chinese) reason."""
    values = (distance_m, angle_deg, speed, timeout_s)
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
        raise GoalRejected('参数必须为有限数值')
    if speed < 0 or timeout_s < 0:
        raise GoalRejected('速度与超时不能为负')
    if kind == MOVE:
        if angle_deg != 0:
            raise GoalRejected('MOVE 不接受角度')
        if abs(distance_m) < limits.distance_tolerance_m:
            raise GoalRejected('距离过小')
        if abs(distance_m) > limits.max_distance_m:
            raise GoalRejected(f'距离超过上限 {limits.max_distance_m} m')
        v = speed or limits.default_mps
        if not limits.min_mps <= v <= limits.max_mps:
            raise GoalRejected(f'线速度须在 {limits.min_mps}–{limits.max_mps} m/s')
        amount, nominal = float(distance_m), abs(distance_m) / v
    elif kind == TURN:
        if distance_m != 0:
            raise GoalRejected('TURN 不接受距离，后退转弯请设置 reverse')
        if abs(angle_deg) < limits.angle_tolerance_deg:
            raise GoalRejected('角度过小')
        if abs(angle_deg) > limits.max_angle_deg:
            raise GoalRejected(f'角度超过上限 {limits.max_angle_deg}°')
        v = speed or limits.default_yaw_rps
        if not limits.min_yaw_rps <= v <= limits.max_yaw_rps:
            raise GoalRejected(f'角速度须在 {limits.min_yaw_rps}–{limits.max_yaw_rps} rad/s')
        if limits.ackermann and limits.turn_linear_mps / v < limits.min_turn_radius_m - 1e-9:
            raise GoalRejected('转弯半径小于底盘最小半径，请降低角速度')
        amount, nominal = math.radians(angle_deg), abs(math.radians(angle_deg)) / v
    else:
        raise GoalRejected('kind 必须是 MOVE 或 TURN')
    timeout = timeout_s or nominal * limits.timeout_factor + limits.timeout_margin_s
    return Goal(kind, amount, float(v), bool(reverse), float(timeout))


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2. * (w * z + x * y), 1. - 2. * (y * y + z * z))


class Progress:
    """Signed travel along the start heading and unwrapped yaw change since start."""

    def __init__(self, x, y, yaw):
        self.x0, self.y0, self.yaw0 = x, y, yaw
        self.last_yaw, self.turned = yaw, 0.
        self.travelled = 0.

    def update(self, x, y, yaw):
        dx, dy = x - self.x0, y - self.y0
        self.travelled = dx * math.cos(self.yaw0) + dy * math.sin(self.yaw0)
        self.turned += wrap_angle(yaw - self.last_yaw)
        self.last_yaw = yaw
        return self.travelled, self.turned


def remaining(goal, progress):
    """Remaining amount (m or rad) in the goal direction; <= 0 means reached or overshot."""
    done = progress.travelled if goal.kind == MOVE else progress.turned
    return math.copysign(1., goal.amount) * (goal.amount - done)


def reached(goal, progress, limits=Limits()):
    tolerance = limits.distance_tolerance_m if goal.kind == MOVE else math.radians(limits.angle_tolerance_deg)
    return remaining(goal, progress) <= tolerance


def command(goal, progress, limits=Limits()):
    """(linear m/s, angular rad/s) request for this control tick; zero once reached."""
    left = remaining(goal, progress)
    if reached(goal, progress, limits):
        return 0., 0.
    sign = math.copysign(1., goal.amount)
    if goal.kind == MOVE:
        scale = min(1., left / limits.slowdown_m)
        return sign * max(limits.min_mps, goal.speed * scale), 0.
    scale = min(1., left / math.radians(limits.slowdown_deg))
    yaw_rate = max(limits.min_yaw_rps, goal.speed * scale)
    if not limits.ackermann:
        return 0., sign * yaw_rate
    # Ackermann: yaw follows v/R, so keep the creep speed and steer by the requested yaw rate,
    # never tighter than the minimum radius. angular.z is body yaw rate (+ left) for either
    # travel direction; whether the chassis firmware maps negative v the same way is a field check.
    linear = limits.turn_linear_mps * (-1. if goal.reverse else 1.)
    yaw_rate = min(yaw_rate, limits.turn_linear_mps / limits.min_turn_radius_m)
    angular = sign * yaw_rate
    return linear, angular
