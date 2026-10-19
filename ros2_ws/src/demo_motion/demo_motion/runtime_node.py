"""Resident demo facade: one heavy mode, one velocity gateway, voice stays resident."""
import json
import threading
import time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from roscar_interfaces.msg import VoiceCommand
from roscar_interfaces.srv import SetControlMode, AuthorizeMotion
from person_interfaces.srv import ClaimOwner
from demo_interfaces.srv import SetRuntimeMode, CaptureFace
from capture_task_interfaces.action import CaptureTask
from capture_task_interfaces.srv import RunCapture, RunInspection, CancelCapture
from capture_task.api_logic import request_goal, service_timeout
from capture_task.catalog import Catalog
from ament_index_python.packages import get_package_share_directory
from .runtime import ModuleOwner, CAPTURE_MODES, validate_mode, manual_command


class RuntimeManager(Node):
    def __init__(self, **kwargs):
        super().__init__('demo_runtime_manager', **kwargs)
        self.root = self.declare_parameter('project_root', '').value
        self.module = ModuleOwner(self.root)
        self.mode, self.phase, self.detail = 'IDLE', 'BOOTING', ''
        self.control, self.control_at = {}, 0.
        self.capture_state, self.capture_at = {}, 0.
        self.sequence = 0
        self.gate = threading.RLock()
        self.operation = threading.Lock()
        self.closing = False
        self.capture_handle = None
        self.capture_cancel_future = None
        self.capture_task_id = ''
        self.manual, self.manual_at = None, 0.
        self.manual_since_ros = float('inf')
        self.stop_pending = False
        self.group = ReentrantCallbackGroup()
        self.set_control = self.create_client(SetControlMode, '/control/set_mode', callback_group=self.group)
        self.arm = self.create_client(AuthorizeMotion, '/control/authorize', callback_group=self.group)
        self.stop_control = self.create_client(Trigger, '/control/stop', callback_group=self.group)
        self.claim = self.create_client(ClaimOwner, '/perception/claim_owner', callback_group=self.group)
        self.capture = ActionClient(self, CaptureTask, '/capture_task/run', callback_group=self.group)
        self.catalog = Catalog.load(str(Path(get_package_share_directory('capture_task'))/'config/tasks.yaml'))
        self.follow_pub = self.create_publisher(TwistStamped, '/control/cmd_vel_request', 1)
        self.external_pub = self.create_publisher(TwistStamped, '/chassis/cmd_vel', 1)
        self.state_pub = self.create_publisher(String, '/robot/state', 10)
        for topic, callback, typ in [('/control/state', self.on_control, String),
                                     ('/capture_task/state', self.on_capture, String),
                                     ('/demo/follow_request', self.on_follow, TwistStamped),
                                     ('/demo/relative_request', self.on_relative, TwistStamped),
                                     ('/robot/cmd_vel', self.on_manual, TwistStamped),
                                     ('/voice/command', self.on_voice, VoiceCommand)]:
            self.create_subscription(typ, topic, callback, 10, callback_group=self.group)
        self.create_service(SetRuntimeMode, '/robot/set_mode', self.set_mode, callback_group=self.group)
        self.create_service(Trigger, '/robot/stop', self.stop, callback_group=self.group)
        self.create_service(Trigger, '/follow/start', self.follow_start, callback_group=self.group)
        self.create_service(Trigger, '/follow/stop', self.stop, callback_group=self.group)
        self.create_service(CaptureFace, '/face/capture', self.collect_face, callback_group=self.group)
        self.create_service(RunCapture, '/capture/face', self.face, callback_group=self.group)
        for prefix in ('/capture/cancel', '/inspect/cancel'):
            self.create_service(CancelCapture, prefix, self.cancel_capture, callback_group=self.group)
        for kind in ('face', 'pcb', 'helmet', 'tongue'):
            self.create_service(RunInspection, '/inspect/'+kind,
                                lambda req, res, kind=kind: self.inspect(kind, req, res), callback_group=self.group)
        self.timer = self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.monitor = self.create_timer(1., self.watch, callback_group=self.group,
                                         clock=Clock(clock_type=ClockType.STEADY_TIME))
        # Never pick up a module or authorization left over from a previous core process.
        threading.Thread(target=self.boot, daemon=True).start()

    @staticmethod
    def decode(message):
        try:
            value = json.loads(message.data)
            return value if isinstance(value, dict) else {}
        except (ValueError, TypeError):
            return {}

    def on_control(self, message):
        self.control, self.control_at = self.decode(message), time.monotonic()
        if (self.phase == 'ACTIVE' and self.mode in ('FOLLOW', 'ROS_CONTROL') and
                self.control.get('command_mode') == 'IDLE'):
            self.request_stop('control_stopped')

    def on_capture(self, message):
        self.capture_state, self.capture_at = self.decode(message), time.monotonic()

    def check(self, ticket):
        if self.closing or ticket != self.sequence or not rclpy.ok(context=self.context):
            raise RuntimeError('cancelled')

    def wait(self, future, ticket, seconds=3.):
        end = time.monotonic()+seconds
        while not future.done():
            self.check(ticket)
            if time.monotonic() >= end:
                raise RuntimeError('interface_timeout')
            time.sleep(.02)
        self.check(ticket)
        return future.result()

    def call(self, client, request, ticket):
        if not client.service_is_ready():
            raise RuntimeError('interface_unavailable:'+client.srv_name)
        # Dispatch and STOP use the same gate. An obsolete operation cannot send a late arm.
        with self.gate:
            self.check(ticket)
            future = client.call_async(request)
        return self.wait(future, ticket)

    def cancel_owned(self):
        handle = self.capture_handle
        if (handle is not None and handle.accepted and handle.status not in (4, 5, 6)
                and self.capture_cancel_future is None):
            self.capture_cancel_future = handle.cancel_goal_async()

    def ticket(self):
        with self.gate:
            self.sequence += 1
            self.manual = None
            try:
                if self.stop_control.service_is_ready():
                    self.stop_control.call_async(Trigger.Request())
                self.cancel_owned()
            except Exception:
                if not self.closing:
                    raise
            return self.sequence

    def until(self, condition, ticket, seconds, reason):
        end = time.monotonic()+seconds
        while not condition():
            self.check(ticket)
            if time.monotonic() >= end:
                raise RuntimeError(reason() if callable(reason) else reason)
            time.sleep(.05)
        self.check(ticket)

    def clean(self, ticket):
        self.phase = 'STOPPING'
        self.manual = None
        result = self.call(self.stop_control, Trigger.Request(), ticket)
        if not result.success:
            raise RuntimeError('stop_failed')
        self.module.stop()
        # DDS discovery and a forcibly terminated capture lease need time to drain.
        self.until(lambda: time.monotonic()-self.control_at < .5 and
                   self.control.get('command_mode') == 'IDLE' and
                   not self.control.get('task_hold', {}).get('active') and
                   self.count_publishers('/camera/color/image_raw') == 0 and
                   self.count_publishers('/perception/person_states') == 0,
                   ticket, 30., lambda: 'old_resources_not_released:'+json.dumps(dict(
                       control_age=round(time.monotonic()-self.control_at, 2),
                       control_mode=self.control.get('command_mode'),
                       hold=self.control.get('task_hold', {}),
                       camera=self.count_publishers('/camera/color/image_raw'),
                       persons=self.count_publishers('/perception/person_states'))))
        self.mode, self.phase, self.detail = 'IDLE', 'IDLE', ''

    def activate(self, mode, target_id, ticket):
        validate_mode(mode, target_id)
        if (mode in CAPTURE_MODES and self.mode == mode and self.phase == 'ACTIVE' and
                self.module.active() and time.monotonic()-self.capture_at < 1. and
                not self.capture_state.get('active')):
            self.check(ticket)
            return
        self.clean(ticket)
        if mode == 'IDLE':
            return
        self.mode, self.phase = mode, 'STARTING'
        if mode in CAPTURE_MODES:
            self.capture_state, self.capture_at = {}, 0.
        if mode != 'ROS_CONTROL':
            self.module.start(mode)
        if mode in ('FOLLOW', 'PERCEPTION'):
            self.until(lambda: self.count_publishers('/perception/target_state') == 1 and
                       self.claim.service_is_ready(), ticket, 50., 'perception_start_timeout')
            if mode == 'FOLLOW':
                end = time.monotonic()+20.
                while True:
                    result = self.call(self.claim, ClaimOwner.Request(target_id=target_id), ticket)
                    if result.success:
                        break
                    if time.monotonic() >= end:
                        raise RuntimeError('owner_claim_failed:'+result.message)
                    time.sleep(.2)
        elif mode in CAPTURE_MODES:
            self.until(lambda: self.capture.server_is_ready() and
                       time.monotonic()-self.capture_at < 1. and
                       self.capture_state.get('camera_ok') and self.capture_state.get('tts_ok') and
                       self.capture_state.get('task_mode') == ('face' if mode=='FACE_UPLOAD' else mode.lower()) and
                       self.capture_state.get('pipeline_mode') == ('collect' if mode=='FACE' else 'upload' if mode=='FACE_UPLOAD' else 'infer'),
                       ticket, 30., 'capture_start_timeout')
        elif mode == 'RELATIVE':
            self.until(lambda: self.count_publishers('/demo/relative_request') == 1,
                       ticket, 10., 'relative_start_timeout')
        if mode in ('FOLLOW', 'ROS_CONTROL'):
            control_mode = 'FOLLOW' if mode == 'FOLLOW' else 'EXTERNAL'
            result = self.call(self.set_control, SetControlMode.Request(mode=control_mode), ticket)
            if not result.success:
                raise RuntimeError(result.message)
            epoch = result.authority_epoch
            self.until(lambda: time.monotonic()-self.control_at < .5 and
                       self.control.get('authority_epoch') == epoch and self.control.get('ready'),
                       ticket, 8., 'motion_not_ready:'+self.control.get('reason', 'unknown'))
            result = self.call(self.arm, AuthorizeMotion.Request(mode=control_mode, authority_epoch=epoch), ticket)
            if not result.success:
                raise RuntimeError('authorize_failed:'+result.message)
            self.until(lambda: self.control.get('authority_epoch') == epoch and
                       self.control.get('authority_state') in ('ARMED_FOLLOW', 'ARMED_MANUAL'),
                       ticket, 2., 'authorization_not_observed')
        with self.gate:
            self.check(ticket)
            self.manual_since_ros = self.get_clock().now().nanoseconds*1e-9
            self.phase, self.detail = 'ACTIVE', ('capture_only;model_not_integrated' if mode == 'FACE' else 'ready')

    def transition(self, mode, target_id=''):
        validate_mode(mode, target_id)
        if self.phase == 'BOOTING':
            raise RuntimeError('booting')
        ticket = self.ticket()
        if not self.operation.acquire(timeout=20.):
            raise RuntimeError('previous_operation_still_stopping')
        try:
            self.check(ticket)
            self.activate(mode, target_id, ticket)
        except Exception as exc:
            self.fail_cleanup(ticket)
            self.detail = str(exc)
            raise
        finally:
            self.operation.release()

    def fail_cleanup(self, ticket):
        # Stale operation never cleans up resources belonging to a newer operation;
        # the operation lock is still held here, so stop is always safe.
        self.phase = 'STOPPING'
        with self.gate:
            if self.stop_control.service_is_ready():
                self.stop_control.call_async(Trigger.Request())
        try:
            if self.capture_handle is not None and self.capture_handle.accepted:
                self.cancel_owned()
                drained = self.capture_handle.get_result_async()
                end = time.monotonic()+2.
                while (not drained.done() or self.capture_cancel_future is not None and
                       not self.capture_cancel_future.done()) and time.monotonic() < end:
                    time.sleep(.02)
            self.module.stop()
            self.mode, self.phase = 'IDLE', 'IDLE'
        except Exception:
            self.phase = 'ERROR'

    def set_mode(self, request, response):
        try:
            self.transition(request.mode, request.target_id)
            response.success, response.detail = True, self.detail or 'stopped'
        except Exception as exc:
            response.detail = str(exc)
        response.mode = self.mode
        return response

    def follow_start(self, request, response):
        try:
            self.transition('FOLLOW')
            response.success, response.message = True, 'following'
        except Exception as exc:
            response.message = str(exc)
        return response

    def stop(self, request, response):
        try:
            self.transition('IDLE')
            response.success, response.message = True, 'stopped'
        except Exception as exc:
            response.message = str(exc)
        return response

    def on_voice(self, message):
        if message.action.upper() in ('STOP', 'DISARM', 'ESTOP', 'EXIT'):
            stamp = message.stamp.sec+message.stamp.nanosec*1e-9
            if 0 <= self.get_clock().now().nanoseconds*1e-9-stamp <= 1.:
                self.request_stop('voice_stop')

    def request_stop(self, reason):
        with self.gate:
            if self.stop_pending or self.phase in ('IDLE', 'BOOTING'):
                return
            self.stop_pending = True
            self.ticket()  # Immediate zero/STOP and cancellation before waiting for operation lock.
        def finish():
            try:
                self.transition('IDLE')
                self.detail = reason
            except Exception as exc:
                self.detail = str(exc)
            finally:
                self.stop_pending = False
        threading.Thread(target=finish, daemon=True).start()

    def on_follow(self, message):
        if self.mode == 'FOLLOW' and self.phase == 'ACTIVE':
            if self.count_publishers('/demo/follow_request') != 1:
                self.request_stop('follow_publisher_conflict')
                return
            self.follow_pub.publish(message)  # Preserve producer stamp; no stale-command refresh.

    def on_relative(self, message):
        if self.mode == 'RELATIVE' and self.phase == 'ACTIVE':
            if self.count_publishers('/demo/relative_request') != 1:
                self.request_stop('relative_publisher_conflict')
                return
            self.external_pub.publish(message)

    def on_manual(self, message):
        if (self.mode == 'ROS_CONTROL' and self.phase == 'ACTIVE' and
                self.control.get('authority_state') == 'ARMED_MANUAL'):
            stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
            if stamp < self.manual_since_ros:
                return
            if self.count_publishers('/robot/cmd_vel') != 1:
                self.request_stop('manual_publisher_conflict')
                return
            self.manual, self.manual_at = message, time.monotonic()

    def zero(self):
        message = TwistStamped()
        message.header.frame_id = 'base_link'
        message.header.stamp = self.get_clock().now().to_msg()
        return message

    def tick(self):
        if not (self.mode == 'FOLLOW' and self.phase == 'ACTIVE'):
            self.follow_pub.publish(self.zero())
        if not (self.mode == 'RELATIVE' and self.phase == 'ACTIVE'):
            output = self.zero()
            message = self.manual
            if (self.mode == 'ROS_CONTROL' and self.phase == 'ACTIVE' and message is not None
                    and time.monotonic()-self.manual_at <= .2):
                t = message.twist
                age = self.get_clock().now().nanoseconds*1e-9-(message.header.stamp.sec+message.header.stamp.nanosec*1e-9)
                output.twist.linear.x, output.twist.angular.z = manual_command(
                    (t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z), age, message.header.frame_id)
            self.external_pub.publish(output)
        self.state_pub.publish(String(data=json.dumps(dict(mode=self.mode, phase=self.phase,
                                    detail=self.detail, generation=self.sequence, recording=False))))

    def watch(self):
        if self.phase == 'ACTIVE':
            if self.mode != 'ROS_CONTROL' and not self.module.active():
                self.request_stop('module_exited')
            elif time.monotonic()-self.control_at > 1.:
                self.request_stop('control_state_stale')

    def capture_call(self, kind, request, response, inspection, collect=False):
        response.task_id = request.task_id
        if kind == 'face' and inspection:
            response.status, response.detail = 'unavailable', 'model_not_integrated;use_/face/capture'
            return response
        try:
            goal = request_goal(kind, request, self.catalog)
        except ValueError as exc:
            response.status, response.detail = 'invalid', str(exc)
            return response
        if self.phase == 'BOOTING':
            response.status, response.detail = 'unavailable', 'booting'
            return response
        ticket = self.ticket()
        if not self.operation.acquire(timeout=20.):
            response.status, response.detail = 'busy', 'previous_operation_still_stopping'
            return response
        try:
            self.capture_task_id = goal['task_id']
            self.check(ticket)
            self.activate('FACE' if collect else 'FACE_UPLOAD' if kind=='face' else kind.upper(), '', ticket)
            pending = self.capture.send_goal_async(CaptureTask.Goal(**goal))
            def accepted(done):
                handle = done.result()
                if handle.accepted and (ticket != self.sequence or self.closing):
                    handle.cancel_goal_async()
            pending.add_done_callback(accepted)
            self.capture_handle = self.wait(pending, ticket)
            if not self.capture_handle.accepted:
                raise RuntimeError('capture_rejected')
            result = self.wait(self.capture_handle.get_result_async(), ticket,
                               service_timeout(request.timeout_sec, self.catalog.defaults)).result
            self.capture_handle = None  # Terminal goal: no late cancellation RPC during cleanup.
            response.task_id = goal['task_id']
            response.capture_id = result.capture_id
            response.status, response.detail = result.status, result.detail
            response.success = result.status == ('captured' if collect else 'analyzed' if inspection else 'uploaded')
            if collect:
                response.image = result.original_image
                response.detail = 'capture_only;model_not_integrated'+(';' + result.detail if result.detail and result.detail != 'model_not_integrated' else '')
            elif inspection:
                response.result_json, response.annotated_image = result.result_json, result.annotated_image
            else:
                response.upload_json = result.upload_json
                response.detail = 'capture_only;model_not_integrated'+(';' + result.detail if result.detail and result.detail != 'model_not_integrated' else '')
            # Report cleanup failure instead of an apparently successful inspection.
            self.clean(ticket)
        except Exception as exc:
            response.success = False
            if collect:
                response.image = type(response.image)()
            elif inspection:
                response.annotated_image = type(response.annotated_image)()
                response.result_json = ''
            response.status = 'cancelled' if str(exc)=='cancelled' else 'timeout' if str(exc)=='interface_timeout' else 'failed'
            response.detail = str(exc)
            if kind == 'face':
                response.detail = 'model_not_integrated;'+response.detail
            self.cancel_owned()
            self.fail_cleanup(ticket)
            self.detail = response.detail
        finally:
            self.capture_handle = None
            self.capture_cancel_future = None
            self.capture_task_id = ''
            self.operation.release()
        return response

    def cancel_capture(self, request, response):
        with self.gate:
            if not self.capture_task_id:
                response.detail = 'not_active'
            elif request.task_id != self.capture_task_id:
                response.detail = 'not_owner'
            else:
                self.request_stop('capture_cancelled')
                response.success, response.detail = True, 'cancel_requested'
        return response

    def collect_face(self, request, response):
        return self.capture_call('face', request, response, False, collect=True)

    def face(self, request, response):
        return self.capture_call('face', request, response, False)

    def inspect(self, kind, request, response):
        return self.capture_call(kind, request, response, True)

    def boot(self):
        ticket = self.ticket()
        with self.operation:
            try:
                self.until(lambda: self.stop_control.service_is_ready(), ticket, 30., 'guard_unavailable')
                self.clean(ticket)
            except Exception as exc:
                self.phase, self.detail = 'ERROR', str(exc)

    def close(self):
        self.closing = True
        self.ticket()
        # systemd BindsTo provides the final cgroup cleanup even after process failure.
        self.module.stop()


def main(args=None):
    rclpy.init(args=args)
    node = RuntimeManager()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close()
        executor.shutdown(timeout_sec=5.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
