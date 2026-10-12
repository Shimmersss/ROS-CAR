"""motion_guard target_grace_s and target_auto_recover in FOLLOW: real node, synthetic request/target, no hardware or chassis driver."""
import copy
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from person_interfaces.msg import TargetState
from motion_guard.node import MotionGuard

rclpy.init(); probe = Node('grace_probe'); ex = SingleThreadedExecutor(); ex.add_node(probe)
BASE = dict(motion_enabled=True, geometry_confirmed=True, stopping_model_confirmed=True,
            mount_calibrated=True, radar_required=False)
req = probe.create_publisher(TwistStamped, '/control/cmd_vel_request', 1)
targets = probe.create_publisher(TargetState, '/perception/target_state', 1)
vel = []; states = []
probe.create_subscription(Twist, '/cmd_vel', vel.append, 1)
probe.create_subscription(String, '/control/state', lambda m: states.append(json.loads(m.data)), 10)
flags = dict(lost=False, request=True)


def pump(duration):
    end = time.monotonic()+duration; last = 0
    while time.monotonic() < end:
        now = time.monotonic()
        if now-last > .025:
            last = now; stamp = probe.get_clock().now().to_msg()
            if flags['request']:
                m = TwistStamped(); m.header.stamp = copy.deepcopy(stamp); m.header.frame_id = 'base_link'
                m.twist.linear.x = .1; req.publish(m)
            m = TargetState(); m.header.stamp = copy.deepcopy(stamp); m.header.frame_id = 'astra_depth_optical_frame'
            m.source = 'astra'; m.status = m.LOST if flags['lost'] else m.TRACKING
            m.position_valid = not flags['lost']; m.position.z = 3.; m.horizontal_distance_m = 3.; m.bearing_rad = 0.
            m.measurement_age_s = math.nan; targets.publish(m)
        ex.spin_once(timeout_sec=.005)


def arm():
    client = probe.create_client(Trigger, '/control/arm'); assert client.wait_for_service(timeout_sec=2)
    f = client.call_async(Trigger.Request()); end = time.monotonic()+2
    while not f.done() and time.monotonic() < end: pump(.02)
    probe.destroy_client(client)
    assert f.result().success, f.result().message


def run(grace, recover=False):
    params = dict(BASE, target_grace_s=grace, target_auto_recover=recover)
    guard = MotionGuard(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
    ex.add_node(guard); flags.update(lost=False, request=True)
    pump(.4); arm(); pump(.3); assert vel[-1].linear.x > 0
    return guard


try:
    # Grace off: one lost burst faults immediately, exactly as before.
    g = run(0.); flags['lost'] = True; pump(.15)
    assert states[-1]['mode'] == 'FAULT' and states[-1]['last_fault'] == 'target_invalid', states[-1]
    ex.remove_node(g); g.destroy_node()
    # Grace 0.6 s: a 0.3 s gap stays ARMED, decays (never exceeds the last command), then resumes.
    g = run(.6); vel.clear(); flags['lost'] = True; pump(.3)
    assert states[-1]['mode'] == 'ARMED' and states[-1]['reason'].startswith('target_grace:'), states[-1]
    gap = [v.linear.x for v in vel]
    assert gap and max(gap) <= .1+1e-9 and gap[-1] < .1, gap
    flags['lost'] = False; pump(.3)
    assert states[-1]['mode'] == 'ARMED' and states[-1]['ready'] and vel[-1].linear.x > 0, states[-1]
    # A gap longer than the grace still faults and stays stopped after the target returns.
    flags['lost'] = True; pump(.9)
    assert states[-1]['mode'] == 'FAULT' and states[-1]['last_fault'] == 'target_invalid', states[-1]
    flags['lost'] = False; pump(.3)
    assert states[-1]['mode'] == 'FAULT' and all(v.linear.x == 0 for v in vel[-3:])
    ex.remove_node(g); g.destroy_node()
    # Non-target failures get no grace: a stale request faults immediately.
    g = run(.6); flags['request'] = False; pump(.35)
    assert states[-1]['mode'] == 'FAULT' and states[-1]['last_fault'] == 'request_timeout', states[-1]
    ex.remove_node(g); g.destroy_node()
    # Auto-recover: a target-only FAULT re-arms once the target is valid again (Route A behaviour).
    g = run(.6, recover=True); flags['lost'] = True; pump(.9)
    assert states[-1]['mode'] == 'FAULT' and states[-1]['last_fault'] == 'target_invalid', states[-1]
    assert all(v.linear.x == 0 for v in vel[-3:])
    flags['lost'] = False; pump(.3)
    assert states[-1]['mode'] == 'ARMED' and states[-1]['ready'] and vel[-1].linear.x > 0, states[-1]
    # Operator stop is not a fault: auto-recover never overrides it (auto_arm is off here).
    client = probe.create_client(Trigger, '/control/stop'); assert client.wait_for_service(timeout_sec=2)
    f = client.call_async(Trigger.Request()); end = time.monotonic()+2
    while not f.done() and time.monotonic() < end: pump(.02)
    probe.destroy_client(client); pump(.3)
    assert states[-1]['mode'] == 'STANDBY' and all(v.linear.x == 0 for v in vel[-3:]), states[-1]
    ex.remove_node(g); g.destroy_node()
    # Auto-recover leaves every other fault latched: a stale request stays FAULT after it returns.
    g = run(.6, recover=True); flags['request'] = False; pump(.35)
    assert states[-1]['mode'] == 'FAULT' and states[-1]['last_fault'] == 'request_timeout', states[-1]
    flags['request'] = True; pump(.4)
    assert states[-1]['mode'] == 'FAULT' and all(v.linear.x == 0 for v in vel[-3:]), states[-1]
    ex.remove_node(g); g.destroy_node()
    try:
        MotionGuard(parameter_overrides=[Parameter('target_grace_s', value=1.5)])
        raise AssertionError('grace above 1 s must be rejected')
    except ValueError:
        pass
    print('PASS motion_guard target grace: off=immediate fault, 0.3 s gap decays and resumes, '
          '0.9 s gap faults and latches, request failure no grace, limit enforced; auto-recover re-arms '
          'only target faults, not stop or request faults (synthetic)')
finally:
    probe.destroy_node(); rclpy.shutdown()
