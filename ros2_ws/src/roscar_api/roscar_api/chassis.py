"""Send zero until an operator explicitly arms EXTERNAL; stop on exit/fault."""
import argparse
import json
import math
import signal
import time
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from roscar_interfaces.srv import SetControlMode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--speed', type=float, default=0.05)
    parser.add_argument('--turn', type=float, default=0.0)
    parser.add_argument('--duration', type=float, default=2.0)
    args, ros_args = parser.parse_known_args()
    if not (all(map(math.isfinite, (args.speed,args.turn,args.duration)))
            and 0 <= args.speed <= .15 and abs(args.turn) <= .5 and 0 < args.duration <= 30):
        parser.error('Require speed 0..0.15, turn -0.5..0.5, duration (0,30]')
    def terminate(signum, frame):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, terminate)
    signal.signal(signal.SIGTERM, terminate)
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('roscar_chassis_example')
    publisher = node.create_publisher(TwistStamped, '/chassis/cmd_vel', 1)
    state = {}; seen_at = [0.]
    def receive(msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict): return
            state.clear(); state.update(value); seen_at[0] = time.monotonic()
        except (ValueError, TypeError): pass
    node.create_subscription(String, '/control/state', receive, 10)
    mode = node.create_client(SetControlMode, '/control/set_mode')
    stop = node.create_client(Trigger, '/control/stop')
    velocity = [0.,0.]
    def publish():
        msg = TwistStamped(); msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x, msg.twist.angular.z = velocity
        publisher.publish(msg)
    timer = node.create_timer(.05, publish)
    def call(client, request):
        if not client.wait_for_service(timeout_sec=3): raise RuntimeError('Service unavailable: '+client.srv_name)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        if not future.done(): raise RuntimeError('Service timed out: '+client.srv_name)
        result = future.result()
        node.get_logger().info(f'{client.srv_name}: success={result.success}, {result.message}')
        if not result.success: raise RuntimeError(result.message)
    try:
        request = SetControlMode.Request(); request.mode = 'EXTERNAL'; call(mode, request)
        node.get_logger().info('Sending zeros. Operator: ros2 service call /control/arm std_srvs/srv/Trigger "{}"')
        started = None
        deadline = time.monotonic()+60
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.02)
            now = time.monotonic()
            fresh = now-seen_at[0] < .3
            armed = fresh and state.get('command_mode') == 'EXTERNAL' and state.get('mode') == 'ARMED'
            if armed:
                if started is None: started = now
                velocity[:] = [args.speed,args.turn]
                if now-started >= args.duration: break
            else:
                velocity[:] = [0.,0.]
                if started is not None: raise RuntimeError('Authorization lost; explicit restart required')
                if now > deadline: raise RuntimeError('Timed out waiting for explicit arm')
                if fresh and (state.get('mode') == 'FAULT' or state.get('command_mode') != 'EXTERNAL'):
                    raise RuntimeError('Guard fault or mode changed')
    except KeyboardInterrupt:
        pass
    finally:
        velocity[:] = [0.,0.]
        publish()
        try: call(stop, Trigger.Request())
        except Exception as exc: node.get_logger().error(f'Stop service failed: {exc}; zero sent, request watchdog remains')
        timer.cancel(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
