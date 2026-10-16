"""Real guard/executor with synthetic odometry, never a physical acceptance test."""
import json
import math
import os
import tempfile
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from demo_interfaces.action import MoveRelative
from demo_interfaces.srv import MoveDistance, TurnAngle
from demo_motion.relative_motion_node import RelativeMotion
from motion_guard.node import MotionGuard
from rclpy.time import Time

assert os.environ.get('ROS_DOMAIN_ID') == '214', 'Use isolated ROS_DOMAIN_ID=214'
rclpy.init()
probe = Node('demo_runtime_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
temporary = tempfile.TemporaryDirectory()
params = dict(use_sim_time=True, motion_enabled=True, geometry_confirmed=True, mount_calibrated=True,
              stopping_model_confirmed=True, emergency_kws_required=False,
              radar_required=False, estop_latch_path=temporary.name+'/latch.json')
guard = MotionGuard(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
demo = RelativeMotion(parameter_overrides=[Parameter('use_sim_time', value=True)])
executor = MultiThreadedExecutor(num_threads=8)
for node in (probe, guard, demo):
    executor.add_node(node)
state = {}; velocity = [0., 0.]; pose = [0., 0., 0.]
flags = dict(odom=True, stale=False)

def on_state(msg):
    state.clear(); state.update(json.loads(msg.data))

def on_velocity(msg):
    velocity[:] = msg.linear.x, msg.angular.z

probe.create_subscription(String, '/control/state', on_state, 10)
probe.create_subscription(Twist, '/cmd_vel', on_velocity, 10)
odom = probe.create_publisher(Odometry, '/odom', 10)
clock_started = time.monotonic_ns()
clock_stop = threading.Event()
def clock_tick():
    while not clock_stop.is_set():
        stamp = Time(nanoseconds=1_000_000_000 + time.monotonic_ns()-clock_started)
        for node in (guard, demo, probe):
            node.get_clock().set_ros_time_override(stamp)
        clock_stop.wait(.001)
clock_thread = threading.Thread(target=clock_tick, daemon=True)
clock_thread.start()
last = time.monotonic()

def integrate():
    global last
    now = time.monotonic(); dt = min(now-last, .05); last = now
    v, w = velocity
    pose[0] += v*math.cos(pose[2])*dt; pose[1] += v*math.sin(pose[2])*dt
    pose[2] += w*dt
    if flags['odom']:
        msg = Odometry(); msg.header.frame_id = 'odom'; msg.child_frame_id = 'base_link'
        msg.header.stamp = probe.get_clock().now().to_msg()
        if flags['stale']: msg.header.stamp.sec -= 2
        msg.pose.pose.position.x, msg.pose.pose.position.y = pose[:2]
        msg.pose.pose.orientation.z = math.sin(pose[2]/2)
        msg.pose.pose.orientation.w = math.cos(pose[2]/2)
        odom.publish(msg)

probe.create_timer(.02, integrate)
thread = threading.Thread(target=executor.spin, daemon=True); thread.start()
client = ActionClient(probe, MoveRelative, '/demo/move_relative')

def wait_for(predicate, timeout=5.):
    deadline = time.monotonic()+timeout
    while not predicate():
        assert time.monotonic() < deadline, state
        time.sleep(.01)

def future_result(future, timeout=5.):
    wait_for(future.done, timeout)
    return future.result()

def send(kind='MOVE', distance=.5, angle=0.):
    g = MoveRelative.Goal(); g.kind=kind; g.distance_m=distance; g.angle_deg=angle
    handle = future_result(client.send_goal_async(g)); assert handle.accepted
    return handle

def result(handle, status='SUCCEEDED'):
    value = future_result(handle.get_result_async(), 30.).result
    assert value.status == status, value
    wait_for(lambda: state.get('command_mode') == 'IDLE' and state.get('authority_state') in ('DISARMED', 'ESTOP'))
    wait_for(lambda: velocity == [0., 0.])
    print(status, value.travelled_m, value.turned_deg, flush=True)
    return value

def trigger(name):
    c = probe.create_client(Trigger, name)
    assert c.wait_for_service(timeout_sec=2.)
    response = future_result(c.call_async(Trigger.Request()))
    assert response.success, response
    probe.destroy_client(c)

try:
    assert client.wait_for_server(timeout_sec=3.)
    wait_for(lambda: demo.odom_fresh() and demo.state_fresh())
    for kind, distance, angle in [('MOVE', .5, 0.), ('MOVE', -.3, 0.),
                                   ('TURN', 0., 90.), ('TURN', 0., -45.)]:
        value = result(send(kind, distance, angle))
        if kind == 'MOVE': assert abs(value.travelled_m-distance) <= .035
        else: assert abs(value.turned_deg-angle) <= 3.1
    handle = send(); wait_for(lambda: abs(velocity[0]) > .01)
    g = MoveRelative.Goal(); g.kind='MOVE'; g.distance_m=.1
    assert not future_result(client.send_goal_async(g)).accepted
    future_result(handle.cancel_goal_async()); result(handle, 'CANCELED')
    handle = send(); wait_for(lambda: abs(velocity[0]) > .01)
    trigger('/demo/motion_cancel'); result(handle, 'CANCELED')
    for service_type, name, fields in [
            (MoveDistance, '/demo/move_distance', {'distance_m': .1}),
            (TurnAngle, '/demo/turn_angle', {'angle_deg': 10.})]:
        service = probe.create_client(service_type, name)
        assert service.wait_for_service(timeout_sec=2.)
        request = service_type.Request()
        for key, value in fields.items(): setattr(request, key, value)
        response = future_result(service.call_async(request), 10.)
        assert response.success, response
        probe.destroy_client(service)
    original_call = demo.call
    def stop_ack_failure(client, request, timeout_s=3.):
        response = original_call(client, request, timeout_s)
        if client is demo.stop:
            response.success = False; response.message = 'injected stop acknowledgement failure'
        return response
    demo.call = stop_ack_failure
    result(send(distance=.1), 'ABORTED')
    demo.call = original_call
    for fault in ('odom', 'stale', 'stop', 'estop'):
        handle = send(); wait_for(lambda: abs(velocity[0]) > .01)
        if fault == 'odom': flags['odom'] = False
        elif fault == 'stale': flags['stale'] = True
        else: trigger('/control/'+fault)
        wait_for(lambda: velocity == [0., 0.], 1.)
        result(handle, 'ABORTED')
        flags.update(odom=True, stale=False)
        wait_for(demo.odom_fresh)
    demo.timer.cancel()
    conflict = probe.create_publisher(TwistStamped, '/chassis/cmd_vel', 1)
    wait_for(lambda: demo.count_publishers('/chassis/cmd_vel') == 2)
    try:
        demo.publish_request()
        raise AssertionError('publisher conflict not rejected')
    except RuntimeError as exc:
        assert 'Multiple' in str(exc)
    assert demo.velocity == (0., 0.)
    probe.destroy_publisher(conflict)
    print('PASS: publisher conflict, four motions, busy rejection, cancel, odom loss/staleness, STOP and ESTOP', flush=True)
finally:
    executor.shutdown(timeout_sec=5.)
    thread.join(timeout=5.)
    for node in (demo, guard, probe): node.destroy_node()
    clock_stop.set(); clock_thread.join()
    rclpy.shutdown(); temporary.cleanup()
