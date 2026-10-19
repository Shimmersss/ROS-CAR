"""Demo only: /demo/move_relative action plus /demo/move_distance and /demo/turn_angle services.

Each goal is one explicit operator request. With authorize_per_goal (default true) the node
switches motion_guard to EXTERNAL, streams fresh zeros, authorizes with the returned epoch,
drives closed loop on /odom, then brakes and calls /control/stop (back to IDLE, authorization
revoked). motion_guard remains the only /cmd_vel publisher; this node only requests on
/chassis/cmd_vel and must be that topic's only publisher (disable the voice motion adapter).

Runtime coverage and remaining field checks: see
docs/demo/Codex任务清单.md.
"""
from dataclasses import fields
import json
import math
import threading
import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from roscar_interfaces.srv import AuthorizeMotion, SetControlMode
from demo_interfaces.action import MoveRelative
from demo_interfaces.srv import MoveDistance, TurnAngle
from .relative_motion import (GoalRejected, Limits, MOVE, TURN, Progress, command, make_goal,
                              reached, remaining, yaw_from_quaternion)

PERIOD_S = 0.05


class RelativeMotion(Node):
    def __init__(self, **kwargs):
        super().__init__('demo_relative_motion', **kwargs)
        for f in fields(Limits):
            self.declare_parameter(f.name, getattr(Limits(), f.name))
        self.limits = Limits(**{f.name: self.get_parameter(f.name).value for f in fields(Limits)})
        self.authorize_per_goal = self.declare_parameter('authorize_per_goal', True).value
        self.odom_timeout_s = self.declare_parameter('odom_timeout_s', 0.3).value
        self.state_timeout_s = self.declare_parameter('state_timeout_s', 0.3).value
        self.zero_lead_s = self.declare_parameter('zero_lead_s', 0.3).value
        self.brake_s = self.declare_parameter('brake_s', 0.5).value
        for name in ('odom_timeout_s', 'state_timeout_s', 'zero_lead_s', 'brake_s'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        self.pose_stamp = -math.inf
        self.publisher_conflict = False
        self.lock = threading.Lock()
        self.busy = False
        self.cancel_requested = False
        self.velocity = (0., 0.)
        self.pose, self.pose_at = None, -math.inf
        self.state, self.state_at = {}, -math.inf
        group = ReentrantCallbackGroup()
        self.pub = self.create_publisher(TwistStamped, '/chassis/cmd_vel', 1)
        self.status_pub = self.create_publisher(String, '/demo/motion_state', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10, callback_group=group)
        self.create_subscription(String, '/control/state', self.on_state, 10, callback_group=group)
        self.set_mode = self.create_client(SetControlMode, '/control/set_mode', callback_group=group)
        self.authorize = self.create_client(AuthorizeMotion, '/control/authorize', callback_group=group)
        self.stop = self.create_client(Trigger, '/control/stop', callback_group=group)
        self.action = ActionServer(self, MoveRelative, '/demo/move_relative', self.execute_action,
                                   goal_callback=self.on_goal, cancel_callback=self.on_cancel,
                                   callback_group=group)
        self.create_service(MoveDistance, '/demo/move_distance', self.on_move_distance, callback_group=group)
        self.create_service(TurnAngle, '/demo/turn_angle', self.on_turn_angle, callback_group=group)
        self.create_service(Trigger, '/demo/motion_cancel', self.on_cancel_service, callback_group=group)
        # Fresh requests every tick (zero unless a goal is driving) for the guard's handshake.
        self.timer = self.create_timer(PERIOD_S, self.publish_request, callback_group=group)

    # --- inputs -------------------------------------------------------------------------------
    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        values = (p.x, p.y, q.x, q.y, q.z, q.w)
        norm = sum(v*v for v in (q.x, q.y, q.z, q.w))
        if (not all(map(math.isfinite, values)) or not 0.99 <= norm <= 1.01
                or stamp <= self.pose_stamp or not 0 <= now - stamp <= self.odom_timeout_s):
            self.get_logger().warning(f'Rejected odom: age={now-stamp:.6f}, delta={stamp-self.pose_stamp:.6f}, norm={norm:.6f}', throttle_duration_sec=1.)
            self.pose_at = -math.inf
            return
        self.pose_stamp = stamp
        self.pose = (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        self.pose_at = time.monotonic()

    def on_state(self, msg):
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict):
                self.state, self.state_at = value, time.monotonic()
        except (TypeError, ValueError):
            self.state, self.state_at = {}, -math.inf

    def odom_fresh(self):
        return (self.pose is not None and time.monotonic() - self.pose_at < self.odom_timeout_s
                and 0 <= self.get_clock().now().nanoseconds * 1e-9 - self.pose_stamp <= self.odom_timeout_s)

    def state_fresh(self):
        return time.monotonic() - self.state_at < self.state_timeout_s

    def armed_external(self, epoch=None):
        s = self.state
        return (self.state_fresh() and s.get('command_mode') == 'EXTERNAL'
                and s.get('authority_state') == 'ARMED_MANUAL'
                and (epoch is None or s.get('authority_epoch') == epoch))

    def publish_request(self):
        if self.count_publishers('/chassis/cmd_vel') > 1:
            self.publisher_conflict = True
            self.velocity = (0., 0.)
            raise RuntimeError('Multiple /chassis/cmd_vel publishers; disable motion_adapter')
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x, msg.twist.angular.z = self.velocity
        self.pub.publish(msg)

    def report(self, **fields_):
        self.status_pub.publish(String(data=json.dumps(fields_, ensure_ascii=False)))

    # --- service helper (runs inside a reentrant callback; never spin here) -------------------
    def call(self, client, request, timeout_s=3.0):
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError(f'服务不可用: {client.srv_name}')
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout_s):
            raise RuntimeError(f'服务超时: {client.srv_name}')
        return future.result()

    # --- goal intake --------------------------------------------------------------------------
    def on_goal(self, request):
        with self.lock:
            if self.busy:
                return GoalResponse.REJECT
            self.busy, self.cancel_requested = True, False
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        self.cancel_requested = True
        return CancelResponse.ACCEPT

    def on_cancel_service(self, request, response):
        self.cancel_requested = True
        response.success, response.message = True, 'cancel requested' if self.busy else 'idle'
        return response

    def execute_action(self, goal_handle):
        g = goal_handle.request
        def feedback(phase, travelled, turned, left):
            fb = MoveRelative.Feedback()
            fb.phase, fb.travelled_m, fb.turned_deg, fb.remaining = phase, travelled, turned, left
            goal_handle.publish_feedback(fb)
        outcome = self.run(g.kind, g.distance_m, g.angle_deg, g.speed, g.timeout_s,
                           reverse=g.kind == TURN and g.distance_m < 0, feedback=feedback,
                           is_canceled=lambda: goal_handle.is_cancel_requested, reserved=True)
        result = MoveRelative.Result()
        (result.success, result.status, result.message,
         result.travelled_m, result.turned_deg) = outcome
        if result.status == 'CANCELED' and goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def on_move_distance(self, request, response):
        (response.success, response.status, response.message,
         response.travelled_m, _) = self.run(MOVE, request.distance_m, 0., request.speed_mps, 0.)
        return response

    def on_turn_angle(self, request, response):
        (response.success, response.status, response.message,
         response.travelled_m, response.turned_deg) = self.run(
            TURN, 0., request.angle_deg, request.yaw_rate_rps, 0., reverse=request.reverse)
        return response

    # --- execution ----------------------------------------------------------------------------
    def run(self, kind, distance_m, angle_deg, speed, timeout_s, reverse=False,
            feedback=lambda *a: None, is_canceled=lambda: False, reserved=False):
        """Returns (success, status, message, travelled_m, turned_deg)."""
        with self.lock:
            if self.busy and not reserved:
                return False, 'REJECTED', '已有相对运动在执行', 0., 0.
            if not reserved:
                self.busy, self.cancel_requested = True, False
        progress = None
        authorized_here = False
        try:
            if not math.isfinite(distance_m):
                raise GoalRejected('距离必须为有限数值')
            # TURN with distance_m < 0 is the action's way to request a backward arc.
            goal = make_goal(kind, 0. if kind == TURN else distance_m, angle_deg, speed, timeout_s,
                             reverse, self.limits)
            if self.publisher_conflict or self.count_publishers('/chassis/cmd_vel') != 1:
                raise GoalRejected('请求话题发布者冲突')
            if not self.odom_fresh():
                raise GoalRejected('没有新鲜 /odom，拒绝执行')
            if not self.state_fresh():
                raise GoalRejected('没有新鲜 /control/state，拒绝执行')
            if self.state.get('authority_state') in ('ESTOP', 'FAULT'):
                raise GoalRejected(f"guard 处于 {self.state.get('authority_state')}，需先人工复位")
            epoch = None
            if self.authorize_per_goal:
                feedback('AUTHORIZING', 0., 0., abs(distance_m or angle_deg))
                authorized_here = True  # set_mode already changes the guard: always stop after
                epoch = self.authorize_external(is_canceled)
            elif not self.armed_external():
                raise GoalRejected('guard 未处于 EXTERNAL/ARMED_MANUAL')
            else:
                epoch = self.state.get('authority_epoch')
                authorized_here = True
            progress = Progress(*self.pose)
            started = time.monotonic()
            while True:
                if self.cancel_requested or is_canceled():
                    return self.finish(progress, False, 'CANCELED', '已取消')
                if self.publisher_conflict:
                    return self.finish(progress, False, 'ABORTED', '请求发布者冲突')
                if not self.odom_fresh():
                    return self.finish(progress, False, 'ABORTED', '/odom 中断')
                if not self.armed_external(epoch):
                    return self.finish(progress, False, 'ABORTED',
                                       f"授权丢失: {self.state.get('reason') or self.state.get('authority_state')}")
                if time.monotonic() - started > goal.timeout_s:
                    return self.finish(progress, False, 'TIMEOUT', '超时未到达')
                progress.update(*self.pose)
                if reached(goal, progress, self.limits):
                    return self.finish(progress, True, 'SUCCEEDED', '已到达')
                self.velocity = command(goal, progress, self.limits)
                left = remaining(goal, progress)
                feedback('MOVING', progress.travelled, math.degrees(progress.turned),
                         max(0., left if kind == MOVE else math.degrees(left)))
                self.report(phase='MOVING', kind=kind, travelled_m=progress.travelled,
                            turned_deg=math.degrees(progress.turned))
                time.sleep(PERIOD_S)
        except GoalRejected as exc:
            if self.cancel_requested or is_canceled():
                return self.finish(progress, False, 'CANCELED', str(exc))
            return False, 'REJECTED', str(exc), 0., 0.
        except Exception as exc:  # service failures etc.: stop and report
            return self.finish(progress, False, 'ABORTED', str(exc))
        finally:
            self.velocity = (0., 0.)
            if authorized_here:
                time.sleep(self.brake_s)
                try:
                    stopped = self.call(self.stop, Trigger.Request())
                    if not stopped.success:
                        raise RuntimeError(stopped.message)
                except Exception as exc:
                    self.get_logger().error(f'/control/stop failed: {exc}; zero requests continue')
                    self.busy = False
                    self.report(phase='STOP_FAILED')
                    return self.finish(progress, False, 'ABORTED', f'停车确认失败: {exc}')
            self.busy = False
            self.report(phase='IDLE')

    def finish(self, progress, success, status, message):
        self.velocity = (0., 0.)
        travelled = progress.travelled if progress else 0.
        turned = math.degrees(progress.turned) if progress else 0.
        self.get_logger().info(f'{status}: {message} (travelled {travelled:.3f} m, turned {turned:.1f} deg)')
        return success, status, message, travelled, turned

    def authorize_external(self, is_canceled=lambda: False):
        request = SetControlMode.Request()
        request.mode = 'EXTERNAL'
        mode = self.call(self.set_mode, request)
        if not mode.success:
            raise GoalRejected(f'切换 EXTERNAL 失败: {mode.message}')
        time.sleep(self.zero_lead_s)  # guard must see fresh zeros stamped after the switch
        if self.cancel_requested or is_canceled():
            raise GoalRejected('授权期间已取消')
        auth = AuthorizeMotion.Request()
        auth.authority_epoch, auth.mode = mode.authority_epoch, 'EXTERNAL'
        result = self.call(self.authorize, auth)
        if not result.success:
            raise GoalRejected(f'授权失败: {result.message}')
        deadline = time.monotonic() + 1.0
        while not self.armed_external(mode.authority_epoch):
            if time.monotonic() > deadline:
                raise GoalRejected(f"授权后未进入 ARMED_MANUAL: {self.state.get('reason', '')}")
            time.sleep(PERIOD_S)
        return mode.authority_epoch

    def destroy_node(self):
        self.velocity = (0., 0.)
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        if rclpy.ok(context=self.context):
            self.pub.publish(msg)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RelativeMotion()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
