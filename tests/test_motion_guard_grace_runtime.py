"""motion_guard follow-entry options (auto_arm, target_grace_s, target_auto_recover) on the MotionAuthority
model: real node, synthetic request/target, no hardware or chassis driver."""
import copy
import json
import math
import tempfile
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from person_interfaces.msg import TargetState
from motion_guard.node import MotionGuard

rclpy.init(); probe = Node('grace_probe'); ex = SingleThreadedExecutor(); ex.add_node(probe)
LATCH = str(Path(tempfile.mkdtemp())/'estop.json')
BASE = dict(motion_enabled=True, geometry_confirmed=True, stopping_model_confirmed=True,
            mount_calibrated=True, radar_required=False, emergency_kws_required=False,
            estop_latch_path=LATCH, command_mode='FOLLOW', auto_arm=True)
req = probe.create_publisher(TwistStamped, '/control/cmd_vel_request', 1)
targets = probe.create_publisher(TargetState, '/perception/target_state', 1)
health = probe.create_publisher(Bool, '/voice/emergency_healthy', 10)
vel = []; states = []
probe.create_subscription(Twist, '/cmd_vel', vel.append, 1)
probe.create_subscription(String, '/control/state', lambda m: states.append(json.loads(m.data)), 10)
flags = dict(lost=False, request=True, healthy=True)


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
            if flags['healthy'] is not None:
                health.publish(Bool(data=flags['healthy']))
        ex.spin_once(timeout_sec=.005)


def call(name):
    client = probe.create_client(Trigger, name); assert client.wait_for_service(timeout_sec=2)
    f = client.call_async(Trigger.Request()); end = time.monotonic()+2
    while not f.done() and time.monotonic() < end: pump(.02)
    probe.destroy_client(client)
    return f.result()


def run(grace, recover=False, **extra):
    params = dict(BASE, target_grace_s=grace, target_auto_recover=recover, **extra)
    guard = MotionGuard(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
    ex.add_node(guard); flags.update(lost=False, request=True, healthy=True)
    pump(.5)  # auto_arm: no explicit authorization call
    assert states[-1]['mode'] == 'ARMED' and states[-1]['authority_state'] == 'ARMED_FOLLOW', states[-1]
    assert vel[-1].linear.x > 0
    return guard


def done(g):
    ex.remove_node(g); g.destroy_node()


def rejected(**overrides):
    try:
        MotionGuard(parameter_overrides=[Parameter(k, value=v) for k, v in dict(BASE, **overrides).items()])
    except ValueError:
        return True
    return False


try:
    # Grace off: one lost burst faults immediately and stays latched without auto-recover.
    g = run(0.); flags['lost'] = True; pump(.15)
    assert states[-1]['mode'] == 'FAULT' and states[-1]['last_fault'] == 'target_invalid', states[-1]
    flags['lost'] = False; pump(.3)
    assert states[-1]['mode'] == 'FAULT' and all(v.linear.x == 0 for v in vel[-3:])
    done(g)
    # Grace 0.6 s: a 0.3 s gap stays armed, decays (never exceeds the last command), then resumes.
    g = run(.6); vel.clear(); flags['lost'] = True; pump(.3)
    assert states[-1]['mode'] == 'ARMED' and states[-1]['reason'].startswith('target_grace:'), states[-1]
    gap = [v.linear.x for v in vel]
    assert gap and max(gap) <= .1+1e-9 and gap[-1] < .1, gap
    flags['lost'] = False; pump(.3)
    assert states[-1]['mode'] == 'ARMED' and states[-1]['ready'] and vel[-1].linear.x > 0, states[-1]
    # A gap longer than the grace faults; auto_arm never re-arms a FAULT.
    flags['lost'] = True; pump(.9); flags['lost'] = False; pump(.3)
    assert states[-1]['mode'] == 'FAULT' and all(v.linear.x == 0 for v in vel[-3:]), states[-1]
    done(g)
    # Non-target failures get no grace: a stale request faults immediately.
    g = run(.6); flags['request'] = False; pump(.35)
    assert states[-1]['mode'] == 'FAULT' and states[-1]['last_fault'] == 'request_timeout', states[-1]
    done(g)
    # Auto-recover: a target-only FAULT re-arms once the target is valid again (Route A behaviour).
    g = run(.6, recover=True); flags['lost'] = True; pump(.9)
    assert states[-1]['mode'] == 'FAULT' and states[-1]['last_fault'] == 'target_invalid', states[-1]
    assert all(v.linear.x == 0 for v in vel[-3:])
    flags['lost'] = False; pump(.3)
    assert states[-1]['authority_state'] == 'ARMED_FOLLOW' and vel[-1].linear.x > 0, states[-1]
    # STOP returns to IDLE: neither auto_arm nor auto-recover undoes it.
    assert call('/control/stop').success; pump(.4)
    assert states[-1]['command_mode'] == 'IDLE' and states[-1]['mode'] == 'STANDBY', states[-1]
    assert all(v.linear.x == 0 for v in vel[-3:])
    done(g)
    # Auto-recover leaves every other fault latched: a stale request stays FAULT after it returns.
    g = run(.6, recover=True); flags['request'] = False; pump(.35)
    assert states[-1]['mode'] == 'FAULT' and states[-1]['last_fault'] == 'request_timeout', states[-1]
    flags['request'] = True; pump(.4)
    assert states[-1]['mode'] == 'FAULT' and all(v.linear.x == 0 for v in vel[-3:]), states[-1]
    # ESTOP latches and is never recovered by follow options.
    assert call('/control/estop').success; pump(.3)
    assert states[-1]['authority_state'] == 'ESTOP' and all(v.linear.x == 0 for v in vel[-3:]), states[-1]
    done(g)
    # A recovered target fault must not let a later emergency-channel fault recover by itself.
    Path(LATCH).unlink(missing_ok=True)  # clear the ESTOP latched above
    g = run(.6, recover=True, emergency_kws_required=True); flags['lost'] = True; pump(.9)
    assert states[-1]['mode'] == 'FAULT' and states[-1]['last_fault'] == 'target_invalid', states[-1]
    flags['lost'] = False; pump(.3)
    assert states[-1]['authority_state'] == 'ARMED_FOLLOW', states[-1]
    flags['healthy'] = False; pump(.15)
    assert states[-1]['mode'] == 'FAULT' and states[-1]['last_fault'] == 'emergency_channel_unhealthy', states[-1]
    flags['healthy'] = True; pump(.4)
    assert states[-1]['mode'] == 'FAULT' and all(v.linear.x == 0 for v in vel[-3:]), states[-1]
    done(g)
    Path(LATCH).unlink(missing_ok=True)
    # A second fault while ALREADY waiting for a target must supersede the recoverable latch.
    for health_value in (False, None):
        g = run(0., recover=True, emergency_kws_required=True, emergency_health_timeout_s=.3)
        flags['lost'] = True; pump(.15)
        assert g.authority.reason == 'target_invalid'
        flags['healthy'] = health_value; pump(.45)
        assert g.authority.reason == 'emergency_channel_unhealthy', states[-1]
        epoch = g.authority.epoch
        flags.update(lost=False, healthy=True); pump(.35)
        assert g.authority.state == 'FAULT' and all(v.linear.x == 0 for v in vel[-3:])
        assert g.authority.epoch == epoch, 'A preserved latch must not churn epochs'
        assert call('/control/reset_fault').success
        pump(.2)
        assert g.authority.state == 'DISARMED' and g.command_mode == 'IDLE'
        done(g)
    g = run(0., recover=True); flags['lost'] = True; pump(.15)
    flags['request'] = False; pump(.35)
    assert g.authority.reason == 'request_timeout'
    flags.update(lost=False, request=True); pump(.3)
    assert g.authority.state == 'FAULT' and all(v.linear.x == 0 for v in vel[-3:])
    done(g)
    assert rejected(target_grace_s=1.5), 'grace above 1 s must be rejected'
    assert rejected(command_mode='IDLE'), 'auto_arm requires a FOLLOW startup'
    assert rejected(motion_enabled=False), 'auto_arm requires motion_enabled'
    assert rejected(command_mode='EXTERNAL', auto_arm=False), 'startup EXTERNAL is not allowed'
    print('PASS motion_guard follow options on MotionAuthority: auto_arm from FOLLOW, grace decays/resumes, '
          'long gap faults, request failure no grace, auto-recover only target faults (not a later emergency-channel fault), STOP/ESTOP not undone, '
          'invalid configurations rejected (synthetic)')
finally:
    probe.destroy_node(); rclpy.shutdown()
