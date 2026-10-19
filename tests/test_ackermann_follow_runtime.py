"""Installed follower+guard launch, synthetic targets only; no chassis or camera."""
import json
import math
import os
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from person_interfaces.msg import TargetState
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger

assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
assert os.environ.get('ROS_DOMAIN_ID') == '225'


def main():
    rclpy.init(); node = Node('ackermann_follow_fixture')
    pub = node.create_publisher(TargetState, '/perception/target_state', 10)
    commands, states, following = [], [], []
    node.create_subscription(Twist, '/cmd_vel', commands.append, 10)
    node.create_subscription(String, '/control/state', lambda m: states.append(json.loads(m.data)), 10)
    node.create_subscription(String, '/control/follow_state', lambda m: following.append(json.loads(m.data)), 10)
    temp = tempfile.TemporaryDirectory(); log = tempfile.TemporaryFile(mode='w+')
    proc = subprocess.Popen(['ros2', 'launch', 'motion_guard', 'follow.launch.py',
        'motion_enabled:=true', 'auto_arm:=true', 'target_auto_recover:=true',
        'geometry_confirmed:=true', 'mount_calibrated:=true', 'stopping_model_confirmed:=true',
        'emergency_kws_required:=false', 'expected_source:=yolo', 'target_frame:=camera_optical',
        'target_distance_m:=1.3', 'max_linear_mps:=0.2', 'distance_gain:=1.0', 'bearing_gain:=1.6',
        'ackermann:=true', 'min_turn_radius_m:=0.4', 'target_grace_s:=0.6'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
        env=dict(os.environ, XDG_STATE_HOME=temp.name))

    def pump(seconds, distance=2., bearing=0., valid=True):
        end = time.monotonic()+seconds; last = 0
        while time.monotonic() < end:
            assert proc.poll() is None
            if time.monotonic()-last > .03:
                m = TargetState(); m.header.stamp = node.get_clock().now().to_msg()
                m.observation_stamp = m.header.stamp; m.header.frame_id = 'camera_optical'
                m.source = 'yolo'; m.status = TargetState.TRACKING; m.position_valid = valid
                m.position.x = distance*math.sin(bearing); m.position.z = distance*math.cos(bearing)
                m.horizontal_distance_m = distance; m.bearing_rad = bearing; m.measurement_age_s = 0.
                pub.publish(m); last = time.monotonic()
            rclpy.spin_once(node, timeout_sec=.01)
    try:
        # Jetson cold imports and DDS discovery finish asynchronously.
        deadline = time.monotonic()+12.
        while time.monotonic() < deadline:
            pump(.1)
            if commands and abs(commands[-1].linear.x-.2) < 1e-6: break
        assert commands and abs(commands[-1].linear.x-.2) < 1e-6, (states[-3:], following[-3:])
        for bearing in (.7, -.7):
            pump(.7, 2., bearing)
            m = commands[-1]
            assert m.linear.x > 0 and m.angular.z*bearing < 0
            assert abs(m.angular.z) <= m.linear.x/.4+1e-6
        pump(.7, 1., .3)
        assert commands[-1].linear.x == commands[-1].angular.z == 0.
        assert following[-1]['reason'] == 'within_follow_distance'
        pump(.7, 2., 1.7)
        assert commands[-1].linear.x == commands[-1].angular.z == 0.
        pump(.8, valid=False)
        assert commands[-1].linear.x == commands[-1].angular.z == 0.
        assert states[-1]['authority_state'] == 'FAULT'
        pump(.7); assert states[-1]['authority_state'] == 'ARMED_FOLLOW', (states[-3:], following[-3:])
        assert commands[-1].linear.x > 0
        client = node.create_client(Trigger, '/control/stop')
        assert client.wait_for_service(timeout_sec=2)
        future = client.call_async(Trigger.Request())
        end = time.monotonic()+2
        while not future.done() and time.monotonic() < end: pump(.02)
        assert future.done() and future.result().success
        pump(.5)
        assert states[-1]['authority_state'] == 'DISARMED' and states[-1]['command_mode'] == 'IDLE'
        assert commands[-1].linear.x == commands[-1].angular.z == 0.
        assert all(0 <= m.linear.x <= .2 and abs(m.angular.z) <= min(.5, m.linear.x/.4)+1e-6 for m in commands)
        assert node.count_publishers('/cmd_vel') == 1
        print('PASS installed Ackermann follow: forward, left/right arcs, close/behind stop, curvature bound, invalid target/target recovery, STOP remains idle, explanatory state; synthetic only')
    except Exception:
        log.seek(0); print(log.read()); raise
    finally:
        os.killpg(proc.pid, signal.SIGINT)
        try: proc.wait(timeout=8)
        except subprocess.TimeoutExpired: os.killpg(proc.pid, signal.SIGKILL); proc.wait()
        log.close(); temp.cleanup(); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
