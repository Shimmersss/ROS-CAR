#!/usr/bin/env python3
"""Real guard and epoch-bound services; synthetic requests only, no car."""
import json
from pathlib import Path
import tempfile
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from roscar_interfaces.srv import SetControlMode, AuthorizeMotion
from execution_mode_interfaces.srv import TaskHold
from motion_guard.node import MotionGuard


def main():
    rclpy.init()
    probe = Node('hold_test')
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    params = dict(motion_enabled=True, geometry_confirmed=True, mount_calibrated=True,
                  stopping_model_confirmed=True, emergency_kws_required=False,
                  estop_latch_path=str(Path(tempfile.mkdtemp())/'estop.json'))
    guard = MotionGuard(parameter_overrides=[Parameter(k, value=v) for k,v in params.items()])
    executor.add_node(guard)
    pub = probe.create_publisher(TwistStamped, '/chassis/cmd_vel', 1)
    velocities, states = [], []
    probe.create_subscription(Twist, '/cmd_vel', velocities.append, 10)
    probe.create_subscription(String, '/control/state', lambda m:states.append(json.loads(m.data)), 10)
    clients = {key:probe.create_client(cls, topic) for key,cls,topic in (
        ('hold', TaskHold, '/control/task_hold'), ('mode', SetControlMode, '/control/set_mode'),
        ('authorize', AuthorizeMotion, '/control/authorize'), ('stop', Trigger, '/control/stop'),
        ('arm', Trigger, '/control/arm'), ('estop', Trigger, '/control/estop'))}
    velocity = 0.

    def pump(seconds):
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            msg = TwistStamped()
            msg.header.stamp = probe.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'; msg.twist.linear.x = velocity
            pub.publish(msg)
            executor.spin_once(timeout_sec=.01)

    def call(name, request):
        client = clients[name]
        assert client.wait_for_service(timeout_sec=2.)
        future = client.call_async(request)
        end = time.monotonic()+2.
        while not future.done() and time.monotonic()<end: pump(.02)
        assert future.done()
        return future.result()

    def zeros():
        pump(.2)
        assert states[-1]['command_mode']=='IDLE'
        assert all(v.linear.x==0. and v.angular.z==0. for v in velocities[-3:])

    def authorize():
        nonlocal velocity
        velocity = 0.
        mode = call('mode', SetControlMode.Request(mode='EXTERNAL'))
        assert mode.success
        pump(.2)
        ticket = AuthorizeMotion.Request(mode='EXTERNAL', authority_epoch=mode.authority_epoch)
        assert call('authorize', ticket).success
        velocity = .1; pump(.2)
        assert guard.authority.state=='ARMED_MANUAL' and velocities[-1].linear.x==.1
        return ticket

    try:
        pump(.4); zeros()
        assert guard.authority.state=='DISARMED' and not call('arm', Trigger.Request()).success
        ticket = authorize()
        token = call('hold', TaskHold.Request(operation='ACQUIRE', owner='tongue', session_id='s1', lease_sec=3.))
        assert token.success and token.token; zeros()
        assert not call('mode', SetControlMode.Request(mode='FOLLOW')).success
        assert not call('authorize', ticket).success
        assert not call('hold', TaskHold.Request(operation='ACQUIRE', owner='other', session_id='s2', lease_sec=3.)).success
        assert not call('hold', TaskHold.Request(operation='RELEASE', token='wrong')).success
        assert call('hold', TaskHold.Request(operation='RENEW', token=token.token, lease_sec=3.)).success
        assert call('stop', Trigger.Request()).success; zeros()
        assert guard.task_hold.token==token.token
        assert call('hold', TaskHold.Request(operation='RELEASE', token=token.token)).success; zeros()
        assert not guard.task_hold.token and guard.authority.state=='DISARMED'
        assert not call('authorize', ticket).success
        authorize()
        lease = call('hold', TaskHold.Request(operation='ACQUIRE', owner='tongue', session_id='s3', lease_sec=.5))
        assert lease.success; pump(.7); zeros()
        assert not guard.task_hold.token and guard.authority.state=='DISARMED'
        assert not call('hold', TaskHold.Request(operation='RENEW', token=lease.token, lease_sec=3.)).success
        token = call('hold', TaskHold.Request(operation='ACQUIRE', owner='tongue', session_id='s4', lease_sec=3.))
        assert token.success
        assert call('estop', Trigger.Request()).success
        assert call('hold', TaskHold.Request(operation='RELEASE', token=token.token)).success
        zeros(); assert guard.authority.state=='ESTOP'
        assert not call('hold', TaskHold.Request(operation='ACQUIRE', owner='tongue', session_id='s5', lease_sec=3.)).success
        print('PASS: epoch arm, lease exclusivity/token/renewal, old-ticket rejection, release/expiry stay DISARMED IDLE, ESTOP persists and blocks mode entry; simulated inputs only')
    finally:
        executor.remove_node(guard); executor.remove_node(probe)
        guard.destroy_node(); probe.destroy_node(); executor.shutdown(); rclpy.shutdown()


if __name__=='__main__':
    main()
