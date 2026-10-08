"""Gimbal bridge end to end with the MCU simulator on a pty, robot_state_publisher and TF.
Synthetic only: no servo, encoder, IMU or USB-serial hardware is involved."""
import math
import os
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException
import yaml
from ament_index_python.packages import get_package_share_directory
from gimbal_interfaces.msg import GimbalCommand, GimbalStatus
from gimbal_bridge.bridge_node import GimbalBridge
from gimbal_bridge.mock_mcu import MockConfig, PtyMock
from gimbal_bridge.urdf import build_urdf, load_mount

MOUNT = os.path.join(get_package_share_directory('gimbal_bridge'), 'config', 'gimbal_mount.yaml')


def yaw_pitch(q):
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    pitch = math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x))))
    return yaw, pitch


class Harness:
    def __init__(self, mode='encoder', corrupt_every=0, base_rate='odom'):
        self.mock = PtyMock(MockConfig(mode=mode), corrupt_every=corrupt_every).start()
        self.bridge = GimbalBridge(namespace='gt', parameter_overrides=[
            Parameter('port', value=self.mock.slave_path), Parameter('base_yaw_rate_source', value=base_rate)])
        self.node = rclpy.create_node('gimbal_probe', namespace='gt')
        self.status, self.joints = [], []
        self.node.create_subscription(GimbalStatus, 'gimbal/status', self.status.append, 50)
        self.node.create_subscription(JointState, 'gimbal/joint_states',
                                      lambda m: self.joints.append((time.monotonic(), self.node.get_clock().now(), m)), 50)
        self.command_pub = self.node.create_publisher(GimbalCommand, 'gimbal/command', 10)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node)
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(self.bridge); self.ex.add_node(self.node)
        params = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
        yaml.safe_dump({'robot_state_publisher': {'ros__parameters': {
            'robot_description': build_urdf(load_mount(MOUNT))}}}, params)
        params.close()
        # Run the binary directly: `ros2 run` does not forward SIGTERM to it.
        self.rsp = subprocess.Popen(['/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher',
                                     '--ros-args', '--params-file', params.name,
                                     '-r', 'joint_states:=/gt/gimbal/joint_states'],
                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def spin(self, seconds, command=None):
        end, last = time.monotonic()+seconds, 0.
        while time.monotonic() < end:
            if command is not None and time.monotonic()-last >= .05:
                command.header.stamp = self.node.get_clock().now().to_msg()
                self.command_pub.publish(command); last = time.monotonic()
            self.ex.spin_once(timeout_sec=.005)

    def wait(self, condition, seconds, command=None, message=''):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            self.spin(.05, command)
            if condition():
                return
        raise AssertionError(message or 'condition not reached')

    def last_joint(self, name, settled=0.):
        """Newest sample of `name`; settled>0 skips samples younger than that (TF still in flight)."""
        now = time.monotonic()
        for received, _, msg in reversed(self.joints):
            if name in msg.name and now-received >= settled:
                return msg, msg.position[msg.name.index(name)]
        return None, None

    def settled_tf(self):
        msg, _ = self.last_joint('pan_joint', .1)
        if msg is None or not self.buffer.can_transform('base_footprint', 'camera_link', Time.from_msg(msg.header.stamp)):
            return None
        return self.camera_tf(msg.header.stamp)

    def camera_tf(self, stamp):
        t = self.buffer.lookup_transform('base_footprint', 'camera_link', Time.from_msg(stamp))
        return yaw_pitch(t.transform.rotation)

    def close(self):
        self.rsp.terminate(); self.rsp.wait(timeout=5)
        self.ex.shutdown(); self.bridge.destroy_node(); self.node.destroy_node(); self.mock.close()


def command(mode, pan=math.nan, tilt=math.nan, rate=0.):
    return GimbalCommand(mode=mode, pan_target_rad=pan, tilt_target_rad=tilt, pan_rate_ff_radps=rate)


def encoder_checks():
    h = Harness(corrupt_every=97)
    try:
        h.wait(lambda: h.status and h.status[-1].time_synced and h.last_joint('pan_joint')[0], 5.,
               message='bridge never synchronised')
        h.spin(4.)   # the simulated MCU counter wraps 3 s after start
        # Judge stamp latency only over a window without a host clock step (the VM steps its
        # wall clock; the bridge re-anchors at the next sync and reports clock_steps).
        for _ in range(4):
            steps, count = h.bridge.sync.steps, len(h.joints)
            h.spin(1.5)
            if h.bridge.sync.steps == steps:
                break
        else:
            raise AssertionError('host clock kept stepping; cannot judge stamp latency')
        lags = [(recv-Time.from_msg(m.header.stamp)).nanoseconds*1e-9 for _, recv, m in h.joints[count+40:]]
        assert len(lags) > 100 and all(-.002 <= lag <= .03 for lag in lags), ('joint stamps must be MCU sample times', min(lags), max(lags))
        stamps = [Time.from_msg(m.header.stamp).nanoseconds for _, _, m in h.joints]
        assert all(b > a for a, b in zip(stamps, stamps[1:])), 'joint stamps must increase across the MCU wrap'
        msg, pan = h.last_joint('pan_joint')
        assert abs(pan-.3) < .005, pan     # absolute encoder: valid at power-up
        h.wait(lambda: h.settled_tf() is not None and abs(h.settled_tf()[0]-.3) < .01, 3.,
               message='TF base_footprint->camera_link missing')
        target = command(GimbalCommand.POSITION, 1., .2)
        h.wait(lambda: abs(h.last_joint('pan_joint')[1]-1.) < .01 and abs(h.last_joint('tilt_joint')[1]-.2) < .01,
               4., target, 'pan/tilt did not reach the command')
        h.spin(.3, target)
        yaw, pitch = h.settled_tf()
        assert abs(yaw-1.) < .01 and abs(pitch-.2) < .01, (yaw, pitch)
        h.spin(1.)   # no host command: bridge keeps a HOLD heartbeat, MCU does not time out
        s = h.status[-1]
        assert s.connected and not s.cmd_timeout and abs(s.pan_rad-1.) < .02, s
        assert s.crc_errors > 0 and s.pan_valid, s
        client = h.node.create_client(Trigger, 'gimbal/set_pan_zero')
        assert client.wait_for_service(timeout_sec=2.)
        future = client.call_async(Trigger.Request())
        h.wait(future.done, 2., message='set_pan_zero no response')
        assert future.result().success, future.result()
        h.wait(lambda: abs(h.last_joint('pan_joint')[1]) < .005, 1., message='zero not applied')
        h.mock.mcu.magnet_ok = False
        h.wait(lambda: not h.status[-1].pan_valid, 1., message='magnet loss not reported')
        count = len(h.joints); h.spin(.3)
        assert all('pan_joint' not in m.name and 'tilt_joint' in m.name for _, _, m in h.joints[count:]), 'pan withheld, tilt kept'
        latest = h.joints[-1][2].header.stamp
        try:
            h.camera_tf(latest)
        except TransformException:
            pass
        else:
            raise AssertionError('camera TF must not extrapolate a withheld pan angle')
        h.mock.close()
        h.wait(lambda: not h.status[-1].connected, 1., message='port loss not reported')
        count = len(h.joints); h.spin(.3)
        assert len(h.joints) == count, 'no joint states without an MCU'
        assert not any(n == '/cmd_vel' for n, _ in h.node.get_topic_names_and_types())
    finally:
        h.close()


def gyro_checks():
    h = Harness(mode='gyro', base_rate='zero')
    try:
        h.wait(lambda: h.status and h.status[-1].time_synced, 5., message='gyro bridge never synchronised')
        h.spin(.5, command(GimbalCommand.POSITION, .5, 0.))
        assert not h.status[-1].pan_valid and h.last_joint('pan_joint')[0] is None, 'unhomed gyro pan must be withheld'
        h.wait(lambda: h.status[-1].pan_valid and h.status[-1].homed, 4., command(GimbalCommand.HOME),
               'homing never completed')
        h.wait(lambda: h.last_joint('pan_joint')[0] is not None and abs(h.last_joint('pan_joint')[1]-.5) < .02,
               4., command(GimbalCommand.POSITION, .5, 0.), 'homed gyro pan did not reach the command')
        assert h.status[-1].ms_since_home < 0xFFFFFFFF
    finally:
        h.close()


def launch_checks():
    env = dict(os.environ)
    link = '/tmp/roscar_gimbal_launch_test'
    def launch(*args):
        return subprocess.Popen(['ros2', 'launch', 'gimbal_bridge', 'gimbal.launch.py', *args],
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
    bad = launch('allow_unconfirmed_mount:=true', 'port:=/dev/null')
    out = bad.communicate(timeout=30)[0]
    assert bad.returncode != 0 and 'simulator only' in out, out[-500:]
    node = rclpy.create_node('gimbal_launch_probe')
    buffer = Buffer(); TransformListener(buffer, node)
    ex = SingleThreadedExecutor(); ex.add_node(node)
    for args, expect_tf in ((('mock:=true', f'mock_link:={link}'), False),
                            (('mock:=true', f'mock_link:={link}', 'allow_unconfirmed_mount:=true'), True)):
        buffer.clear()
        proc = launch(*args)
        try:
            end, seen = time.monotonic()+8., False
            while time.monotonic() < end and not seen:
                ex.spin_once(timeout_sec=.05)
                seen = buffer.can_transform('base_footprint', 'camera_link', Time())
            assert seen == expect_tf, (args, seen)
        finally:
            proc.send_signal(signal.SIGINT)
            try: proc.wait(timeout=15)
            except subprocess.TimeoutExpired: proc.kill(); proc.wait()
    ex.shutdown(); node.destroy_node()


def main():
    rclpy.init()
    try:
        encoder_checks()
        gyro_checks()
        launch_checks()
        print('PASS gimbal ROS: pty serial, CRC resync, MCU clock wrap/sync stamps, URDF TF follows commands, '
              'HOLD heartbeat, zero service, magnet loss/port loss withdraw TF, gyro homing gate, launch mount gate')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
