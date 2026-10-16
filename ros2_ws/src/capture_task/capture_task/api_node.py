"""Optional ROS services forwarding only to the existing capture action.

Owned-goal cleanup follows roscar_api/speech.py; no camera, TTS or HTTP clients.
"""
from dataclasses import dataclass, field
from pathlib import Path
import threading
import time
import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String
from capture_task_interfaces.action import CaptureTask
from capture_task_interfaces.srv import RunCapture, RunInspection, CancelCapture
from .api_logic import request_goal, service_timeout, response_fields, action_result, rejection_result
from .catalog import Catalog
from .protocol import loads
from .state import fresh, EXECUTOR_MAX_AGE


@dataclass
class Call:
    task_id: str
    started: float
    deadline: float
    cancel: threading.Event = field(default_factory=threading.Event)
    abandoned: threading.Event = field(default_factory=threading.Event)
    handle: object = None
    pending: object = None


class CallCancelled(Exception):
    pass


class CaptureApi(Node):
    def __init__(self, **kwargs):
        super().__init__('capture_api', **kwargs)
        self.declare_parameter('catalog', str(Path(get_package_share_directory('capture_task'))/'config/tasks.yaml'),
                               ParameterDescriptor(read_only=True))
        self.declare_parameter('pipeline_mode', 'upload', ParameterDescriptor(read_only=True))
        self.pipeline_mode = self.get_parameter('pipeline_mode').value
        if self.pipeline_mode not in ('upload', 'infer'):
            raise ValueError('invalid_pipeline_mode')
        self.inspection = self.pipeline_mode == 'infer'
        self.prefix = '/inspect/' if self.inspection else '/capture/'
        self.catalog = Catalog.load(self.get_parameter('catalog').value)
        if 'cancel' in self.catalog.tasks:
            raise ValueError('capture_task_type_conflicts_with_cancel_service')
        self._busy = threading.Lock()
        self._lock = threading.RLock()
        self.closing = False
        self.job = None
        self.rejections = {}
        self.executor_state = {}
        self.executor_at = 0.
        group = ReentrantCallbackGroup()
        self.client = ActionClient(self, CaptureTask, '/capture_task/run', callback_group=group)
        self.create_subscription(String, '/capture_task/result', self.on_rejection, 10, callback_group=group)
        self.create_subscription(String, '/capture_task/state', self.on_state, 10, callback_group=group)
        self.capture_services = [self.create_service(RunInspection if self.inspection else RunCapture, self.prefix+kind,
                        lambda request,response,kind=kind: self.run_capture(kind,request,response), callback_group=group)
                         for kind in self.catalog.tasks]
        self.create_service(CancelCapture, self.prefix+'cancel', self.cancel_capture, callback_group=group)

    def on_rejection(self, message):
        try:
            data = loads(message.data)
            if data.get('status') == 'rejected':
                with self._lock:
                    self.rejections[data['task_id']] = (time.monotonic(), data['detail'])
                    if len(self.rejections) > 64:
                        del self.rejections[next(iter(self.rejections))]
        except (ValueError, KeyError):
            pass

    def on_state(self, message):
        with self._lock:
            try:
                self.executor_state = loads(message.data)
                self.executor_at = time.monotonic()
            except ValueError:
                self.executor_state = {};self.executor_at = 0.

    def fill(self, response, fields):
        if self.inspection:
            fields = dict(fields)
            fields.pop('upload_json', None)
            fields['success'] = fields.get('status') == 'analyzed'
        for key, value in fields.items():
            if hasattr(response, key):
                setattr(response, key, value)
        return response

    def check(self, job):
        if self.closing or not rclpy.ok(context=self.context) or (job.cancel.is_set() and (job.handle is None or not job.handle.accepted)):
            raise CallCancelled()
        if time.monotonic() >= job.deadline:
            raise TimeoutError()

    def wait(self, future, job):
        while not future.done():
            self.check(job)
            time.sleep(.01)
        self.check(job)
        return future.result()

    @staticmethod
    def cancel_handle(handle):
        try:
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
        except Exception:
            pass  # ROS may be closing; never fall back to global cancellation.

    @classmethod
    def cancel_when_accepted(cls, future):
        try:
            cls.cancel_handle(future.result())
        except Exception:
            pass

    def abandon(self, job):
        job.abandoned.set()
        self.cancel_handle(job.handle)
        if job.pending is not None:
            # Also cover cancellation between the original done callback and
            # assigning the handle in the waiting service callback.
            job.pending.add_done_callback(self.cancel_when_accepted)

    def rejection(self, job):
        end = min(job.deadline, time.monotonic()+.2)
        while True:
            self.check(job)
            with self._lock:
                item = self.rejections.get(job.task_id)
            if item and item[0] > job.started:
                return rejection_result(job.task_id, item[1])
            if time.monotonic() >= end:
                return rejection_result(job.task_id)
            job.cancel.wait(.01)

    def run_capture(self, kind, request, response):
        started = time.monotonic()
        try:
            goal_fields = request_goal(kind, request, self.catalog)
        except ValueError as exc:
            return self.fill(response, response_fields(request.task_id, 'invalid', str(exc)))
        task_id = goal_fields['task_id']
        if not self._busy.acquire(blocking=False):
            return self.fill(response, response_fields(task_id, 'busy', 'busy'))
        job = Call(task_id, started, started+service_timeout(request.timeout_sec,self.catalog.defaults))
        with self._lock:
            self.job = job
        try:
            with self._lock:
                mode=self.executor_state.get('pipeline_mode') if fresh(self.executor_at,EXECUTOR_MAX_AGE) else None
            if mode is not None and mode!=self.pipeline_mode:
                return self.fill(response,response_fields(task_id,'unavailable','pipeline_mode_mismatch'))
            # Check readiness in short intervals so shutdown and cancel stay responsive.
            until = min(job.deadline, time.monotonic()+2.)
            while not self.client.server_is_ready():
                self.check(job)
                if time.monotonic() >= until:
                    return self.fill(response, response_fields(task_id, 'unavailable', 'executor_unavailable'))
                self.client.wait_for_server(timeout_sec=min(.05,max(0.,until-time.monotonic())))
            self.check(job)
            job.pending = self.client.send_goal_async(CaptureTask.Goal(**goal_fields))
            def accepted(done):
                try:
                    handle = done.result()
                    if job.abandoned.is_set() or job.cancel.is_set() or self.closing:
                        self.cancel_handle(handle)
                except Exception:
                    pass
            job.pending.add_done_callback(accepted)
            job.handle = self.wait(job.pending, job)
            if not job.handle.accepted:
                return self.fill(response, self.rejection(job))
            result = self.wait(job.handle.get_result_async(), job).result
            fields = action_result(task_id, result)
            if self.inspection:
                fields.update(result_json=result.result_json, annotated_image=result.annotated_image)
            return self.fill(response, fields)
        except CallCancelled:
            self.abandon(job)
            return self.fill(response, response_fields(task_id, 'cancelled', 'cancel_requested'))
        except TimeoutError:
            self.abandon(job)
            return self.fill(response, response_fields(task_id, 'timeout', 'service_timeout'))
        except Exception:
            self.abandon(job)
            return self.fill(response, response_fields(task_id, 'failed', 'action_unavailable'))
        finally:
            with self._lock:
                if self.job is job:
                    self.job = None
            self._busy.release()

    def cancel_capture(self, request, response):
        with self._lock:
            job = self.job
            if job is not None:
                if request.task_id != job.task_id:
                    response.detail = 'not_owner'
                else:
                    job.cancel.set();self.abandon(job)
                    response.success = True;response.detail = 'cancel_requested'
            elif fresh(self.executor_at,EXECUTOR_MAX_AGE) and self.executor_state.get('active'):
                response.detail = 'not_owner'
            else:
                response.detail = 'not_active'
        return response

    def stop(self):
        with self._lock:
            self.closing = True
            if self.job is not None:
                self.job.cancel.set();self.abandon(self.job)

    def destroy_node(self):
        self.stop();self.client.destroy()
        return super().destroy_node()


def main(args=None):
    from .runtime import run
    run(CaptureApi, args, threads=6)
