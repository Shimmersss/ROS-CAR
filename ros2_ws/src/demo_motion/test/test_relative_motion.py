import math
import pytest
from demo_motion.relative_motion import (Limits, GoalRejected, Progress, MOVE, TURN, command,
                                         make_goal, reached, remaining, yaw_from_quaternion)


def test_move_forward_and_backward_are_signed():
    forward = make_goal(MOVE, distance_m=1.0)
    backward = make_goal(MOVE, distance_m=-0.5, speed=0.1)
    assert forward.amount == 1.0 and forward.speed == Limits().default_mps
    assert backward.amount == -0.5 and backward.speed == 0.1
    assert forward.timeout_s == pytest.approx(1.0 / 0.15 * 2 + 3)


@pytest.mark.parametrize('kwargs', [
    dict(kind=MOVE, distance_m=3.0), dict(kind=MOVE, distance_m=0.01), dict(kind=MOVE, distance_m=1, speed=0.5),
    dict(kind=MOVE, distance_m=1, angle_deg=5), dict(kind=TURN, angle_deg=270), dict(kind=TURN, angle_deg=1),
    dict(kind=TURN, angle_deg=90, distance_m=1), dict(kind='SPIN', angle_deg=90),
    dict(kind=MOVE, distance_m=float('nan')), dict(kind=MOVE, distance_m=1, speed=-0.1),
])
def test_invalid_goals_rejected(kwargs):
    with pytest.raises(GoalRejected):
        make_goal(**kwargs)


def test_ackermann_turn_radius_enforced():
    with pytest.raises(GoalRejected):
        make_goal(TURN, angle_deg=90, speed=0.5)   # 0.1 / 0.5 = 0.2 m < 0.4 m
    goal = make_goal(TURN, angle_deg=-90, speed=0.25)
    assert goal.amount == pytest.approx(-math.pi / 2)


def test_progress_projects_onto_start_heading_and_unwraps_yaw():
    p = Progress(1.0, 1.0, math.pi / 2)
    p.update(1.0, 1.5, math.pi / 2)
    assert p.travelled == pytest.approx(0.5)
    p = Progress(0., 0., 3.0)
    for yaw in (3.1, -3.1, -2.9):  # crosses +-pi
        p.update(0., 0., yaw)
    assert p.turned == pytest.approx(-2.9 - 3.0 + 2 * math.pi)


def test_move_command_ramps_down_and_stops():
    goal = make_goal(MOVE, distance_m=-1.0)
    p = Progress(0., 0., 0.)
    linear, angular = command(goal, p)
    assert linear == pytest.approx(-0.15) and angular == 0
    p.update(-0.92, 0., 0.)
    assert -0.15 < command(goal, p)[0] < 0
    p.update(-0.98, 0., 0.)
    assert reached(goal, p) and command(goal, p) == (0., 0.)


def test_ackermann_turn_creeps_and_steers_by_sign():
    left = make_goal(TURN, angle_deg=90)
    right_back = make_goal(TURN, angle_deg=-45, reverse=True)
    p = Progress(0., 0., 0.)
    assert command(left, p) == pytest.approx((0.1, 0.2))
    assert command(right_back, p) == pytest.approx((-0.1, -0.2))
    fast = make_goal(TURN, angle_deg=90, speed=0.25)
    assert command(fast, p) == pytest.approx((0.1, 0.25))       # v / R_min
    p.update(0.3, 0.3, math.radians(88))
    assert reached(left, p) and remaining(left, p) < math.radians(3)


def test_differential_turns_in_place():
    limits = Limits(ackermann=False)
    goal = make_goal(TURN, angle_deg=30, limits=limits)
    assert command(goal, Progress(0., 0., 0.), limits) == pytest.approx((0., 0.2))


def test_yaw_from_quaternion():
    assert yaw_from_quaternion(0., 0., math.sin(0.4), math.cos(0.4)) == pytest.approx(0.8)
