"""Isolated node regressions: no motor publisher or real GPIO output is used."""
import copy
import math
from types import SimpleNamespace
import rclpy
from rclpy.parameter import Parameter
from person_interfaces.msg import TargetState
from std_msgs.msg import Int32
from astra_body_adapter.person_follower_node import PersonFollowerNode
from voice_command_router.buzzer_gpio_node import BuzzerGpioNode

rclpy.init()
n = PersonFollowerNode()
n.set_parameters([Parameter('enabled', value=True)])
# Isolated callback test; ROS publisher discovery is tested separately.
n.count_publishers = lambda topic: 1
commands = []
n._publish = lambda v, w: commands.append((v, w))
def target(source='yolo'):
    m = TargetState()
    m.source = source
    m.status = m.TRACKING
    m.position_valid = True
    m.horizontal_distance_m = 3.
    m.bearing_rad = .2
    m.header.stamp = n.get_clock().now().to_msg()
    m.observation_stamp = copy.deepcopy(m.header.stamp)
    m.measurement_age_s = 0.
    return m
def check(m, moving):
    n.set_parameters([Parameter('expected_source', value=m.source if m.source in ('astra', 'yolo', 'red_object') else 'yolo')])
    n._target_callback(m)
    n._control_tick()
    assert (commands[-1] != (0., 0.)) == moving, commands[-1]
check(target(), True)
check(target('red_object'), True)
for source in ('astra', 'yolo', 'red_object'):
    check(target(source), True)
    n.set_parameters([Parameter('expected_source', value='unknown')])
    n._control_tick(); assert commands[-1] == (0., 0.)
    n.set_parameters([Parameter('expected_source', value=source)])
    n.count_publishers = lambda topic: 2
    n._control_tick(); assert commands[-1] == (0., 0.)
    n.count_publishers = lambda topic: 1
for source in ('demo', 'unknown'):
    check(target(source), False)
m = target(); m.is_simulated = True; check(m, False)
m = target(); m.header.stamp.sec -= 5; check(m, False)
m = target(); m.observation_stamp = n.get_clock().now().to_msg(); m.observation_stamp.sec -= 5; check(m, False)
m = target(); m.observation_stamp.sec += 5; check(m, False)
for age in (math.nan, -1., 100.):
    m = target(); m.measurement_age_s = age; check(m, False)
m = target('astra'); m.observation_stamp.sec = m.observation_stamp.nanosec = 0
m.measurement_age_s = math.nan; check(m, True)
m.header.stamp.sec -= 5; check(m, False)
n._target_callback(target()); n.received_at -= 5; n._control_tick(); assert commands[-1] == (0., 0.)
n.destroy_node()
b = BuzzerGpioNode()
writes = []
b._gpio = SimpleNamespace(HIGH=1, LOW=0, output=lambda p,v: writes.append(v), cleanup=lambda p: writes.append('cleanup'))
b._configured_pin = 7
b._on_command(Int32(data=100)); assert 1 not in writes
b.set_parameters([Parameter('enabled', value=True)])
b._on_command(Int32(data=2000)); assert writes[-1] == 1
b.set_parameters([Parameter('enabled', value=False)])
b._tick(); assert writes[-1] == 0 and b._deadline is None
writes.clear(); b._on_command(Int32(data=100)); assert 1 not in writes
b.set_parameters([Parameter('enabled', value=True)])
b._on_command(Int32(data=100)); b._deadline = 0.; b._tick(); assert writes[-1] == 0
b._on_command(Int32(data=2000)); b.destroy_node(); assert writes[-2:] == [0, 'cleanup']
rclpy.shutdown()
print('PASS control source/freshness/A compatibility and GPIO disable/expiry/shutdown')
