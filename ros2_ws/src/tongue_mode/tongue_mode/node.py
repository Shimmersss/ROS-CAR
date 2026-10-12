"""RunMode implementation: persistent stationary session with bounded capture jobs."""
import json
import math
import os
from pathlib import Path
import queue
import re
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace
import rclpy
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from execution_mode_interfaces.action import RunMode, SpeakText
from execution_mode_interfaces.msg import ModeCommand
from execution_mode_interfaces.srv import TaskHold
from .capture import FrameBuffer
from .cloud import CloudClient, Cancelled
from .flow import DetectionFlow
from .results import AnalysisError, spoken_result


class SafetyError(AnalysisError):
    pass


class RetakeRequested(Cancelled):
    pass


class TongueMode(Node):
    def __init__(self):
        super().__init__('tongue_mode')
        defaults = dict(server_url='http://172.19.128.171:8096',
                        expected_pipeline_id='cloud_r50_384_cvAC10_passthrough_debug',
                        expected_pipeline_digest='c5b9b1e31f73515b44044569b66e64d49861880441273a1fb86cd84774d6a494',
                        camera_topic='/camera/color/image_raw', camera_profile='astra_rgb_tongue_v1',
                        safety_profile='robot', prepare_sec=2., task_timeout_sec=150.,
                        speech_timeout_sec=30., frame_timeout_sec=2., max_frame_age_sec=.5,
                        jpeg_quality=95, manager_timeout_sec=6., hold_lease_sec=3.,
                        save_images=False, storage_dir='~/.local/state/roscar/tongue')
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.cfg = {key:self.get_parameter(key).value for key in defaults}
        if self.cfg['safety_profile'] not in ('bench', 'robot'):
            raise ValueError('safety_profile must be bench or robot')
        if not 80 <= self.cfg['jpeg_quality'] <= 100 or not .5 <= self.cfg['hold_lease_sec'] <= 10.:
            raise ValueError('Invalid quality/lease parameters')
        for key in ('task_timeout_sec', 'speech_timeout_sec', 'frame_timeout_sec', 'max_frame_age_sec', 'manager_timeout_sec'):
            if not math.isfinite(self.cfg[key]) or self.cfg[key] <= 0:
                raise ValueError('Invalid time limit: ' + key)
        if not 0 <= self.cfg['prepare_sec'] <= 30. or not re.fullmatch(r'[0-9a-f]{64}', self.cfg['expected_pipeline_digest']):
            raise ValueError('Invalid preparation time or pipeline digest')
        self.bridge = CvBridge()
        self.frames = FrameBuffer()
        self.cloud = CloudClient(self.cfg['server_url'], bearer=os.environ.get('TONGUE_API_TOKEN', ''))
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='tongue-http')
        self.network_future = None
        self.ctx = None
        self.reserved = False
        self.lock = threading.RLock()
        self.last_event = None
        self.control_state, self.control_at = {}, 0.
        self.odom, self.odom_at = None, 0.
        self.result_pub = self.create_publisher(String, '/tongue/result', 10)
        self.state_pub = self.create_publisher(String, '/tongue/state', 10)
        self.create_subscription(Image, self.cfg['camera_topic'], self.image, qos_profile_sensor_data)
        self.create_subscription(String, '/control/state', self.state, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 2)
        self.create_subscription(ModeCommand, '/execution_mode/control', self.control, 10)
        self.create_subscription(String, '/execution_mode/heartbeat', self.heartbeat, 10)
        self.clients_group = ReentrantCallbackGroup()
        self.hold = self.create_client(TaskHold, '/control/task_hold', callback_group=self.clients_group)
        self.speech = ActionClient(self, SpeakText, '/voice/speak', callback_group=self.clients_group)
        self.server = ActionServer(self, RunMode, '/modes/tongue/run', self.execute,
                                   goal_callback=self.goal, cancel_callback=self.cancel,
                                   callback_group=ReentrantCallbackGroup())
        self.create_timer(.25, self.timer, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def image(self, msg):
        try:
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            age = self.get_clock().now().nanoseconds*1e-9-stamp
            if msg.encoding not in ('rgb8', 'bgr8', 'rgba8', 'bgra8') or stamp <= 0 or not -.05 <= age <= self.cfg['max_frame_age_sec']:
                return
            # cv_bridge does the BGR/RGB conversion; the cloud receives RGB JPEG.
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            self.frames.put(rgb, stamp)
        except Exception as exc:
            self.get_logger().warning('RGB decode failed: ' + type(exc).__name__)

    def state(self, msg):
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict):
                self.control_state, self.control_at = value, time.monotonic()
        except ValueError:
            pass

    def on_odom(self, msg):
        self.odom, self.odom_at = msg, time.monotonic()

    def heartbeat(self, msg):
        try:
            value = json.loads(msg.data)
            ctx = self.ctx
            if ctx and value.get('mode_id') == 'tongue' and value.get('session_id') == ctx.session:
                ctx.manager_at = time.monotonic()
        except (ValueError, AttributeError):
            pass

    def goal(self, request):
        with self.lock:
            if self.reserved or request.mode_id != 'tongue' or not re.fullmatch(r'[A-Za-z0-9_-]{1,80}', request.session_id) or not request.request_id:
                return GoalResponse.REJECT
            self.reserved = True
            return GoalResponse.ACCEPT

    def interrupt(self, ctx, exit_mode=False):
        if exit_mode:
            ctx.cancel.set()
        ctx.task_cancel.set()
        self.cloud.cancel()
        if ctx.speech_handle:
            try:
                ctx.speech_handle.cancel_goal_async()
            except Exception:
                pass

    def cancel(self, goal):
        ctx = self.ctx
        if ctx and ctx.goal == goal:
            self.interrupt(ctx, True)
        return CancelResponse.ACCEPT

    def control(self, msg):
        ctx = self.ctx
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        age = self.get_clock().now().nanoseconds*1e-9-stamp
        if not ctx or msg.session_id != ctx.session or msg.mode_id != 'tongue' or stamp <= 0 or not 0 <= age <= 2.:
            return
        if msg.operation == 'EXIT':
            self.interrupt(ctx, True)
        elif msg.operation == 'RETAKE':
            if ctx.phase != 'READY':
                ctx.retake = True
                self.interrupt(ctx)
            else:
                self.enqueue(ctx, 'RETAKE')
        elif msg.operation == 'REPEAT' and ctx.phase == 'READY':
            self.enqueue(ctx, 'REPEAT')

    @staticmethod
    def enqueue(ctx, operation):
        try:
            ctx.commands.put_nowait(operation)
        except queue.Full:
            pass

    def feedback(self, ctx):
        if not rclpy.ok(context=self.context):
            return
        text = json.dumps(ctx.last_result, ensure_ascii=False) if ctx.last_result else ''
        if ctx.goal.is_active:
            ctx.goal.publish_feedback(RunMode.Feedback(phase=ctx.phase, detail=ctx.detail, result_json=text))
        self.state_pub.publish(String(data=json.dumps(dict(mode_id='tongue', session_id=ctx.session,
                                   phase=ctx.phase, detail=ctx.detail), ensure_ascii=False)))
        event_key = (ctx.session, ctx.phase, ctx.detail, ctx.capture_generation)
        if event_key != self.last_event:
            self.last_event = event_key
            try:
                root = Path(self.cfg['storage_dir']).expanduser()
                root.mkdir(parents=True, exist_ok=True)
                with (root/'events.jsonl').open('a', encoding='utf-8') as f:
                    f.write(json.dumps(dict(session_id=ctx.session, phase=ctx.phase, detail=ctx.detail,
                                           capture_generation=ctx.capture_generation, monotonic_sec=time.monotonic()),
                                       ensure_ascii=False)+'\n')
            except OSError as exc:
                self.get_logger().error('Mode event log failed: ' + type(exc).__name__)

    def timer(self):
        ctx = self.ctx
        if not ctx:
            return
        self.feedback(ctx)
        if ctx.token and not ctx.cancel.is_set() and time.monotonic()-ctx.renew_at >= .5 and not ctx.renew_pending:
            ctx.renew_pending = True
            request = TaskHold.Request(operation='RENEW', token=ctx.token, lease_sec=self.cfg['hold_lease_sec'])
            future = self.hold.call_async(request)
            def done(f):
                ctx.renew_pending = False
                try:
                    if not f.result().success:
                        ctx.safety_error = '停车保持续租失败'
                    else:
                        ctx.renew_at = time.monotonic()
                except Exception:
                    ctx.safety_error = '停车保持服务失效'
            future.add_done_callback(done)

    def check(self, ctx, task=True):
        if not rclpy.ok() or ctx.cancel.is_set() or ctx.goal.is_cancel_requested:
            raise Cancelled()
        if time.monotonic()-ctx.manager_at > self.cfg['manager_timeout_sec']:
            raise SafetyError('执行模式调度器心跳超时')
        if ctx.safety_error:
            raise SafetyError(ctx.safety_error)
        if task and ctx.task_cancel.is_set():
            raise RetakeRequested()
        if task and ctx.deadline and time.monotonic() > ctx.deadline:
            self.cloud.cancel()
            raise AnalysisError('检测任务超过时限')
        if ctx.monitor:
            self.verify_stationary(ctx)

    def verify_stationary(self, ctx):
        now, state = time.monotonic(), self.control_state
        if self.count_publishers('/cmd_vel') != 1:
            raise SafetyError('检测到缺失或重复的最终速度发布者')
        hold = state.get('task_hold', {})
        if now-self.control_at > .5 or state.get('command_mode') != 'IDLE' or not hold.get('active') or hold.get('session_id') != ctx.session:
            raise SafetyError('控制状态陈旧、停车保持失效或小车模式改变')
        if state.get('authority_state') != 'DISARMED':
            raise SafetyError('检测期间发生急停、故障或意外运动授权')
        if state.get('emergency_healthy') is not True:
            raise SafetyError('急停识别通道失效')
        if self.cfg['safety_profile'] == 'bench':
            if state.get('motion_enabled') is not False:
                raise SafetyError('台架配置要求 motion_enabled=false')
        else:
            if self.odom is None or now-self.odom_at > .5:
                raise SafetyError('缺少新鲜底盘速度反馈')
            stamp = self.odom.header.stamp.sec + self.odom.header.stamp.nanosec * 1e-9
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            if stamp <= 0 or not 0 <= age <= .5:
                raise SafetyError('底盘速度反馈时间戳失效')
            t = self.odom.twist.twist
            values = (t.linear.x, t.linear.y, t.angular.z)
            if not all(math.isfinite(x) for x in values) or abs(values[0]) > .01 or abs(values[1]) > .01 or abs(values[2]) > .02:
                raise SafetyError('底盘没有保持静止')

    def wait_future(self, ctx, future, timeout, task=True):
        deadline = time.monotonic()+timeout
        while not future.done():
            self.check(ctx, task)
            if time.monotonic() > deadline:
                raise AnalysisError('等待接口响应超时')
            time.sleep(.025)
        self.check(ctx, task)
        return future.result()

    def acquire(self, ctx):
        if not self.hold.service_is_ready():
            raise SafetyError('停车保持服务尚未就绪')
        req = TaskHold.Request(operation='ACQUIRE', owner='tongue', session_id=ctx.session,
                               lease_sec=self.cfg['hold_lease_sec'])
        response = self.wait_future(ctx, self.hold.call_async(req), 2., task=False)
        if not response.success:
            raise SafetyError(response.detail)
        ctx.token, ctx.renew_at = response.token, time.monotonic()
        end, stopped_at = time.monotonic()+3., None
        while time.monotonic() < end:
            self.check(ctx, task=False)
            try:
                self.verify_stationary(ctx)
                stopped_at = stopped_at or time.monotonic()
                if time.monotonic()-stopped_at >= .5:
                    ctx.monitor = True
                    return
            except SafetyError:
                stopped_at = None
            time.sleep(.025)
        raise SafetyError('无法确认停车和新鲜速度反馈')

    def stage(self, phase, body=None):
        ctx = self.ctx
        ctx.phase, ctx.detail = phase, ''
        if body:
            ctx.last_result = body
        self.feedback(ctx)

    def speak(self, text):
        ctx = self.ctx
        self.check(ctx)
        if not self.speech.server_is_ready():
            raise AnalysisError('带完成确认的语音播报服务不可用')
        goal = SpeakText.Goal(utterance_id=ctx.session+'-'+str(time.monotonic_ns()), text=text)
        accepted = self.speech.send_goal_async(goal)
        generation = ctx.capture_generation
        def cancel_late(f):
            try:
                handle = f.result()
                if handle.accepted and (self.ctx is not ctx or ctx.cancel.is_set() or ctx.task_cancel.is_set() or ctx.capture_generation != generation):
                    handle.cancel_goal_async()
            except Exception:
                pass
        accepted.add_done_callback(cancel_late)
        handle = self.wait_future(ctx, accepted, 2.)
        if not handle.accepted:
            raise AnalysisError('语音播报队列已满或请求被拒绝')
        ctx.speech_handle = handle
        try:
            result = self.wait_future(ctx, handle.get_result_async(), self.cfg['speech_timeout_sec'])
            if result.result.status != 'completed':
                raise AnalysisError('语音未成功播放：' + result.result.detail)
        finally:
            handle.cancel_goal_async()
            ctx.speech_handle = None

    def wait(self, seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            self.check(self.ctx)
            time.sleep(.025)

    def capture(self, after, cancel):
        ros_after = self.get_clock().now().nanoseconds * 1e-9
        return self.frames.capture(after, cancel, self.cfg['frame_timeout_sec'],
                                   self.cfg['max_frame_age_sec'], lambda:self.check(self.ctx), ros_after=ros_after)

    def network(self, function):
        ctx = self.ctx
        if self.network_future and not self.network_future.done():
            raise AnalysisError('上一请求正在清理，请稍后重拍')
        self.network_future = self.pool.submit(function)
        try:
            return self.wait_future(ctx, self.network_future, max(.1, ctx.deadline-time.monotonic()))
        except Exception:
            self.cloud.cancel()
            raise

    def ready(self):
        ctx = self.ctx
        if self.count_publishers(self.cfg['camera_topic']) != 1:
            raise AnalysisError('需要唯一的原始彩色相机发布者')
        cancel, deadline = ctx.task_cancel, min(ctx.deadline, time.monotonic()+5.)
        return self.network(lambda:self.cloud.ready(cancel, deadline,
                     self.cfg['expected_pipeline_id'], self.cfg['expected_pipeline_digest']))

    def analyze(self, jpeg, ids, capture_time):
        ctx = self.ctx
        cancel, deadline = ctx.task_cancel, ctx.deadline
        return self.network(lambda:self.cloud.analyze(jpeg, ids, self.cfg['camera_profile'],
                                                       capture_time, cancel, deadline))

    def save(self, record, jpeg):
        self.check(self.ctx)
        root = Path(self.cfg['storage_dir']).expanduser() / record['ids']['capture_id']
        root.mkdir(parents=True, exist_ok=True)
        record['safety_profile'] = self.cfg['safety_profile']
        path = root/'result.json'
        temporary = root/'result.json.tmp'
        temporary.write_text(json.dumps(record, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        temporary.replace(path)
        if self.cfg['save_images']:
            (root/'capture.jpg').write_bytes(jpeg)
        self.result_pub.publish(String(data=json.dumps(record, ensure_ascii=False)))

    def run_capture(self, ctx, first=False):
        ctx.last_result = None
        ctx.task_cancel = threading.Event()
        ctx.capture_generation += 1
        ctx.deadline = time.monotonic()+self.cfg['task_timeout_sec']
        try:
            DetectionFlow(self, self.cfg).run(ctx.session, ctx.task_cancel, first)
        except RetakeRequested:
            raise
        except SafetyError:
            raise
        except Cancelled:
            if ctx.retake and not ctx.cancel.is_set() and not ctx.goal.is_cancel_requested:
                raise RetakeRequested()
            raise
        except Exception as exc:
            ctx.last_result = None
            ctx.detail = str(exc)[:200]
            ctx.phase = 'CAPTURE_FAILED'
            self.feedback(ctx)
            ctx.deadline = None
            # Failure to speak is recorded, never treated as successful playback.
            try:
                self.speak('这次检测没有完成，请检查相机或网络，然后说重新拍摄。')
            except (Cancelled, SafetyError):
                raise
            except Exception:
                pass
        finally:
            ctx.deadline = None

    def execute(self, goal):
        ctx = SimpleNamespace(goal=goal, session=goal.request.session_id, cancel=threading.Event(),
                              task_cancel=threading.Event(), commands=queue.Queue(maxsize=1),
                              token='', renew_at=0., renew_pending=False, manager_at=time.monotonic(),
                              safety_error='', monitor=False, speech_handle=None, phase='ACQUIRE_HOLD',
                              detail='', last_result=None, deadline=None, retake=False, capture_generation=0)
        self.ctx = ctx
        status, detail = 'exited', ''
        try:
            self.acquire(ctx)
            capture, first = True, True
            while rclpy.ok():
                self.check(ctx, task=False)
                if capture:
                    try:
                        self.run_capture(ctx, first)
                    except RetakeRequested:
                        ctx.retake = False
                        ctx.phase = 'CLEANUP'
                        # Finish cancelling the old socket before submitting the
                        # new capture. No queued/unbounded HTTP jobs on retake.
                        end = time.monotonic()+3.5
                        while self.network_future and not self.network_future.done() and time.monotonic() < end:
                            self.check(ctx, task=False)
                            time.sleep(.025)
                        if self.network_future and not self.network_future.done():
                            raise AnalysisError('旧网络请求未能及时清理')
                        first = False
                        continue
                    first, capture = False, False
                ctx.task_cancel.clear()
                ctx.phase = 'READY'
                self.feedback(ctx)
                try:
                    operation = ctx.commands.get(timeout=.1)
                except queue.Empty:
                    continue
                if operation == 'RETAKE':
                    capture = True
                elif operation == 'REPEAT':
                    self.speak(spoken_result(ctx.last_result) if ctx.last_result and ctx.last_result.get('status') in ('ok', 'uncertain')
                               else '当前没有可重播的成功结果，请说重新拍摄。')
            raise Cancelled()
        except Cancelled:
            status, detail = 'cancelled', 'Mode cancelled; vehicle remains IDLE'
        except Exception as exc:
            status, detail = 'failed', str(exc)[:200]
        finally:
            self.interrupt(ctx, True)
            ctx.phase, ctx.detail = status.upper(), detail
            self.feedback(ctx)
            if ctx.token and rclpy.ok():
                future = self.hold.call_async(TaskHold.Request(operation='RELEASE', token=ctx.token))
                end = time.monotonic()+1.
                while not future.done() and time.monotonic() < end:
                    time.sleep(.025)
            with self.lock:
                self.ctx = None
                self.reserved = False
        if goal.is_cancel_requested:
            goal.canceled()
        elif status == 'failed':
            goal.abort()
        else:
            goal.succeed()
        return RunMode.Result(status=status, detail=detail,
                              last_result_json=json.dumps(ctx.last_result, ensure_ascii=False) if ctx.last_result else '')

    def stop(self):
        if self.ctx:
            self.interrupt(self.ctx, True)
        self.cloud.cancel()
        self.pool.shutdown(wait=False, cancel_futures=True)


def main(args=None):
    rclpy.init(args=args)
    node = TongueMode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.stop()
        executor.shutdown(timeout_sec=3.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
