"""Isolated synthetic ROS: a busy guard still receives requests; no robot or motion."""
import json
import os
import tempfile
import threading
import time
from pathlib import Path

import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped, Twist
from std_msgs.msg import String
from person_interfaces.msg import TargetState
from roscar_interfaces.srv import AuthorizeMotion
from motion_guard.node import MotionGuard

assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
assert os.environ.get('ROS_DOMAIN_ID') == '222'


def trial(concurrent):
    with tempfile.TemporaryDirectory() as temp:
        guard = MotionGuard(parameter_overrides=[Parameter(k, value=v) for k, v in dict(
            motion_enabled=True, command_mode='FOLLOW', radar_required=False,
            geometry_confirmed=True, mount_calibrated=True, stopping_model_confirmed=True,
            emergency_kws_required=False, expected_source='yolo',
            target_auto_recover=True, estop_latch_path=str(Path(temp)/'estop.json')).items()])
        sender = rclpy.create_node('synthetic_request_source')
        requests = sender.create_publisher(TwistStamped, '/control/cmd_vel_request', 1)
        targets = sender.create_publisher(TargetState, '/perception/target_state', 1)
        states, velocities = [], []
        sender.create_subscription(String, '/control/state', lambda m: states.append(json.loads(m.data)), 20)
        sender.create_subscription(Twist, '/cmd_vel', lambda m: velocities.append((m.linear.x, m.angular.z)), 20)
        executor = MultiThreadedExecutor(num_threads=2) if concurrent else SingleThreadedExecutor()
        executor.add_node(guard)
        running, send_requests = threading.Event(), threading.Event()
        running.set(); send_requests.set()
        def publish():
            while running.is_set():
                stamp = sender.get_clock().now().to_msg()
                if send_requests.is_set():
                    msg = TwistStamped(); msg.header.stamp = stamp; msg.header.frame_id = 'base_link'
                    requests.publish(msg)
                target = TargetState(status=TargetState.TRACKING, position_valid=True, source='yolo',
                                     horizontal_distance_m=1.3, bearing_rad=0., measurement_age_s=0.)
                target.header.frame_id = guard.cfg['target_frame']
                target.header.stamp = target.observation_stamp = stamp
                target.position.x = target.position.y = 0.; target.position.z = 1.3
                targets.publish(target)
                time.sleep(.025)
        worker = threading.Thread(target=executor.spin)
        publisher = threading.Thread(target=publish)
        worker.start(); publisher.start()
        def wait(predicate, timeout=4.):
            end = time.monotonic()+timeout
            while time.monotonic() < end:
                rclpy.spin_once(sender, timeout_sec=.02)
                if predicate(): return
            raise AssertionError(states[-3:])
        try:
            wait(lambda: states and states[-1]['ready'])
            client = sender.create_client(AuthorizeMotion, '/control/authorize')
            assert client.wait_for_service(timeout_sec=2)
            # A newer zero request cannot make a validated nonzero snapshot
            # eligible for authorization.
            checked = guard.check
            guard.check = lambda: (True, 'synthetic_snapshot', .1, 0.)
            try:
                future = client.call_async(AuthorizeMotion.Request(
                    mode='FOLLOW', authority_epoch=states[-1]['authority_epoch']))
                wait(lambda: future.done())
                assert not future.result().success, 'nonzero validated request must not arm'
            finally:
                guard.check = checked
            future = client.call_async(AuthorizeMotion.Request(mode='FOLLOW', authority_epoch=states[-1]['authority_epoch']))
            wait(lambda: future.done())
            assert future.result().success, future.result()
            wait(lambda: states[-1]['authority_state'] == 'ARMED_FOLLOW')
            # Delay a serialized graph check beyond the unchanged 200 ms watchdog.
            original = guard.count_publishers
            delayed = threading.Event()
            def slow_graph(topic):
                if not delayed.is_set():
                    delayed.set(); time.sleep(.27)
                return original(topic)
            guard.count_publishers = slow_graph
            wait(delayed.is_set)
            wait(lambda: states and (states[-1]['authority_state'] == 'FAULT'
                                    or len(states) > 25), timeout=3.)
            # Collect states after the delayed check, not a pre-delay ready state.
            end = time.monotonic()+.6
            while time.monotonic() < end: rclpy.spin_once(sender, timeout_sec=.02)
            if concurrent:
                assert states[-1]['authority_state'] == 'ARMED_FOLLOW', states[-3:]
                assert not any(s['last_fault'] == 'request_timeout' for s in states)
                send_requests.clear()
                wait(lambda: states[-1]['last_fault'] == 'request_timeout')
                send_requests.set()
                end = time.monotonic()+.35
                while time.monotonic() < end: rclpy.spin_once(sender, timeout_sec=.02)
                assert states[-1]['authority_state'] == 'FAULT', 'request faults must stay latched'
            else:
                assert states[-1]['last_fault'] == 'request_timeout', states[-3:]
            assert velocities and all(v == (0., 0.) for v in velocities)
        finally:
            running.clear(); publisher.join(); executor.shutdown(); worker.join()
            guard.destroy_node(); sender.destroy_node()


rclpy.init()
try:
    trial(False)
    trial(True)
    print('PASS: serialized executor reproduces false request timeout; concurrent input survives 270ms check; true request loss still latches; all final velocities zero')
finally:
    rclpy.shutdown()
