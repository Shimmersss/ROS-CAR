"""ROS adapter for independent capture jobs. Never publishes vehicle commands.

Speech/lease lifecycle adapted from tongue_mode/node.py (2026-10-12).
"""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import threading
import time
import uuid
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from capture_task_interfaces.action import CaptureTask
from execution_mode_interfaces.action import SpeakText
from execution_mode_interfaces.srv import TaskHold
from .catalog import Catalog, number
from .cloud_client import CloudClient, CloudError, Cancelled, token_from_env
from .flow import CaptureFlow, failure_result
from .frames import FrameBuffer
from .protocol import identifier, loads


@dataclass
class Job:
    request: object
    task: dict
    received: float
    deadline: float
    cancel: threading.Event = field(default_factory=threading.Event)
    upload_cancel: threading.Event = field(default_factory=threading.Event)
    finished: threading.Event = field(default_factory=threading.Event)
    goal: object = None
    speech: object = None
    token: str = ''
    phase: str = 'ACCEPTED'
    detail: str = ''
    monitor: bool = False
    renew_at: float = 0.
    renew_pending: bool = False
    safety_error: str = ''
    records: list = field(default_factory=list)
    pending_images: list = field(default_factory=list)


class CaptureExecutor(Node):
    def __init__(self, **kwargs):
        super().__init__('capture_task_executor', **kwargs)
        defaults = dict(catalog=str(Path(get_package_share_directory('capture_task'))/'config/tasks.yaml'),
                        camera_topic='/camera/color/image_raw',camera_profile='raw_rgb_v1',safety_profile='bench',
                        cloud_url='http://127.0.0.1:8090',robot_id='roscar-01',storage_dir='artifacts/capture-task',
                        save_images=False,token_file='',hold_lease_sec=3.)
        for key,value in defaults.items():
            self.declare_parameter(key,value,ParameterDescriptor(read_only=True))
        self.cfg={key:self.get_parameter(key).value for key in defaults}
        self.catalog=Catalog.load(self.cfg['catalog'])
        identifier(self.cfg['robot_id'])
        if self.cfg['safety_profile'] not in ('bench','robot'):
            raise ValueError('invalid_safety_profile')
        number(self.cfg['hold_lease_sec'],1.,10.)
        if not self.cfg['camera_topic'].startswith('/') or not self.cfg['camera_profile'].isascii() or not self.cfg['camera_profile'].replace('_','').replace('-','').isalnum():
            raise ValueError('invalid_camera_config')
        self.robot=self.cfg['safety_profile']=='robot'
        self.cloud=CloudClient(self.cfg['cloud_url'],token_from_env(self.cfg['token_file']))
        self.pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='capture_upload')
        self.lock=threading.RLock(); self.io_lock=threading.Lock()
        self.job=None; self.closing=False
        self.mode={};self.mode_at=0.;self.control={};self.control_at=0.;self.odom=None;self.odom_at=0.
        self.frames=FrameBuffer();self.cv=CvBridge();self.group=ReentrantCallbackGroup()
        self.speech=ActionClient(self,SpeakText,'/voice/speak',callback_group=self.group)
        self.hold=self.create_client(TaskHold,'/control/task_hold',callback_group=self.group)
        self.create_subscription(Image,self.cfg['camera_topic'],self.on_image,qos_profile_sensor_data,callback_group=self.group)
        self.create_subscription(String,'/execution_mode/state',self.on_mode,10,callback_group=self.group)
        self.create_subscription(String,'/control/state',self.on_control,10,callback_group=self.group)
        self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data,callback_group=self.group)
        self.state_pub=self.create_publisher(String,'/capture_task/state',10)
        self.result_pub=self.create_publisher(String,'/capture_task/result',10)
        self.server=ActionServer(self,CaptureTask,'/capture_task/run',self.execute,
                                goal_callback=self.admit,cancel_callback=self.on_cancel,callback_group=self.group)
        self.timer=self.create_timer(.25,self.tick,callback_group=self.group,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_mode(self,message):
        try:
            self.mode=loads(message.data);self.mode_at=time.monotonic()
        except ValueError:
            self.mode={'active':True};self.mode_at=time.monotonic()

    def on_control(self,message):
        try:
            self.control=loads(message.data);self.control_at=time.monotonic()
        except ValueError:
            self.control={};self.control_at=0.

    def on_odom(self,message):
        self.odom=message;self.odom_at=time.monotonic()

    def on_image(self,message):
        try:
            if message.encoding not in ('rgb8','bgr8','rgba8','bgra8') or not 0 < message.width*message.height <= 40000000:
                return
            rgb=self.cv.imgmsg_to_cv2(message,desired_encoding='rgb8')
            self.frames.put(rgb,message.header.stamp.sec+message.header.stamp.nanosec*1e-9)
        except (ValueError,TypeError,RuntimeError):
            pass

    def rejection(self,request,reason):
        # Rejected ROS actions have no result transport; publish the reason here.
        data=dict(status='rejected',detail=reason,task_id=request.task_id[:64],task_type=request.task_type[:32],capture_id='',records=[])
        self.result_pub.publish(String(data=json.dumps(data)))
        return GoalResponse.REJECT

    def admit(self,request):
        with self.lock:
            if self.closing or self.job:
                return self.rejection(request,'busy')
            try:
                identifier(request.task_id)
                if request.source not in ('cloud','local_test'):
                    raise ValueError()
                params=loads(request.params_json or '{}')
                task=self.catalog.resolve(request.task_type,params)
                number(request.ttl_sec,0,600)
            except ValueError:
                return self.rejection(request,'invalid')
            if self.mode.get('active') is True:
                return self.rejection(request,'voice_mode_active')
            now=time.monotonic();ttl=request.ttl_sec or self.catalog.defaults['task_timeout_sec']
            self.job=Job(request,task,now,now+ttl)
            return GoalResponse.ACCEPT

    def on_cancel(self,goal):
        with self.lock:
            if self.job and self.job.goal is goal:
                self.job.cancel.set();self.job.upload_cancel.set();self.cloud.cancel()
                if self.job.speech:
                    self.job.speech.cancel_goal_async()
        return CancelResponse.ACCEPT

    def tick(self):
        job=self.job
        state=dict(active=job is not None,task_id=job.request.task_id if job else '',
                   task_type=job.request.task_type if job else '',phase=job.phase if job else 'IDLE',detail=job.detail if job else '',
                   camera_ok=self.count_publishers(self.cfg['camera_topic'])==1,tts_ok=self.speech.server_is_ready())
        self.state_pub.publish(String(data=json.dumps(state)))
        if job and job.token and not job.finished.is_set() and not job.renew_pending and time.monotonic()-job.renew_at>=.5:
            job.renew_pending=True
            future=self.hold.call_async(TaskHold.Request(operation='RENEW',token=job.token,lease_sec=self.cfg['hold_lease_sec']))
            def renewed(done):
                job.renew_pending=False
                try:
                    if not done.result().success:
                        job.safety_error='hold_renew_failed'
                    else:
                        job.renew_at=time.monotonic()
                except Exception:
                    job.safety_error='hold_service_failed'
            future.add_done_callback(renewed)

    def stage(self,phase,detail=''):
        job=self.job;job.phase,job.detail=phase,detail
        if job.goal and job.goal.is_active:
            job.goal.publish_feedback(CaptureTask.Feedback(phase=phase,detail=detail))
        self.write_event(dict(task_id=job.request.task_id,phase=phase,detail=detail,monotonic_sec=time.monotonic()))

    def write_event(self,data):
        root=Path(self.cfg['storage_dir']).expanduser();root.mkdir(parents=True,exist_ok=True)
        with self.io_lock, (root/'events.jsonl').open('a',encoding='utf-8') as handle:
            handle.write(json.dumps(data,ensure_ascii=False,allow_nan=False)+'\n')

    def check(self):
        job=self.job
        if self.closing or job.cancel.is_set() or job.goal and job.goal.is_cancel_requested or not rclpy.ok(context=self.context):
            self.cloud.cancel();raise Cancelled()
        if time.monotonic()>=job.deadline:
            self.cloud.cancel();raise ValueError('task_timeout')
        if job.safety_error:
            raise ValueError(job.safety_error)
        if self.mode.get('active') is True:
            raise ValueError('voice_mode_active')
        if job.monitor:
            self.verify_stationary()
        if not self.robot and self.count_publishers('/cmd_vel'):
            raise ValueError('bench_velocity_publisher')

    def ready(self):
        self.check()
        if self.count_publishers(self.cfg['camera_topic']) != 1:
            raise ValueError('camera_publisher_count')
        if not self.speech.server_is_ready():
            raise ValueError('tts_unavailable')
        if self.robot:
            publishers=self.get_publishers_info_by_topic('/cmd_vel')
            if len(publishers)!=1 or publishers[0].node_name!='motion_guard':
                raise ValueError('robot_guard_publisher_count')

    def future(self,future,timeout):
        end=min(self.job.deadline,time.monotonic()+timeout)
        while not future.done():
            self.check()
            if time.monotonic()>=end:
                raise ValueError('interface_timeout')
            time.sleep(.01)
        self.check()
        return future.result()

    def release_token(self,token):
        if token:
            return self.hold.call_async(TaskHold.Request(operation='RELEASE',token=token))
        return None

    def acquire(self):
        job=self.job
        if not self.hold.service_is_ready():
            raise ValueError('hold_failed')
        sent=self.hold.call_async(TaskHold.Request(operation='ACQUIRE',owner='capture_task',session_id=job.request.task_id,lease_sec=self.cfg['hold_lease_sec']))
        # A successful late ACQUIRE must release its token, even on timeout/shutdown.
        abandoned=threading.Event()
        def late(done):
            try:
                response=done.result()
                if response.success and (abandoned.is_set() or job.finished.is_set() or job.cancel.is_set()):
                    self.release_token(response.token)
            except Exception:
                pass
        sent.add_done_callback(late)
        try:
            response=self.future(sent,2.)
        except Exception:
            abandoned.set()
            if sent.done():
                late(sent)
            raise
        if not response.success:
            raise ValueError('hold_failed')
        job.token=response.token;job.renew_at=time.monotonic()
        end=min(job.deadline,time.monotonic()+3.);stable=None
        while time.monotonic()<end:
            self.check()
            try:
                self.verify_stationary()
                stable=stable or time.monotonic()
                if time.monotonic()-stable>=.5:
                    job.monitor=True;return
            except ValueError:
                stable=None
            time.sleep(.02)
        raise ValueError('stationary_unconfirmed')

    def verify_stationary(self):
        now=time.monotonic();state=self.control;job=self.job
        hold=state.get('task_hold',{})
        if self.count_publishers('/cmd_vel')!=1 or now-self.control_at>.5 or state.get('command_mode')!='IDLE' or state.get('authority_state') not in ('DISARMED','ESTOP'):
            raise ValueError('stationary_control_invalid')
        if hold.get('active') is not True or hold.get('owner')!='capture_task' or hold.get('session_id')!=job.request.task_id:
            raise ValueError('hold_lost')
        if state.get('emergency_healthy') is not True:
            raise ValueError('emergency_unhealthy')
        odom=self.odom
        if odom is None or now-self.odom_at>.5:
            raise ValueError('odom_stale')
        stamp=odom.header.stamp.sec+odom.header.stamp.nanosec*1e-9
        age=self.ros_now()-stamp
        twist=odom.twist.twist
        values=(twist.linear.x,twist.linear.y,twist.angular.z)
        if stamp<=0 or not 0<=age<=.5 or not all(math.isfinite(v) for v in values) or abs(values[0])>.01 or abs(values[1])>.01 or abs(values[2])>.02:
            raise ValueError('odom_not_stationary')

    def speak(self,text):
        deadline=min(self.job.deadline,time.monotonic()+self.catalog.defaults['speech_timeout_sec'])
        self.speak_goal(text,self.check,deadline)

    def failure_notice_skipped(self,reason):
        self.write_event(dict(task_id=self.job.request.task_id,phase='FAILED',
                              detail='fail_announce_skipped:'+reason))

    def speak_failure(self,detail):
        # Failure notification has its own budget; task deadline/safety errors
        # must not suppress the safe configured message. Never call check here.
        if detail.startswith('tts_'):
            self.failure_notice_skipped(detail)
            return
        job=self.job
        def external_cancel_only():
            if self.closing or not rclpy.ok(context=self.context) or job.cancel.is_set() or job.goal and job.goal.is_cancel_requested:
                raise Cancelled()
        external_cancel_only()
        deadline=time.monotonic()+min(self.catalog.defaults['speech_timeout_sec'],15.)
        self.speak_goal(self.catalog.defaults['fail_announce'],external_cancel_only,deadline)

    def speak_goal(self,text,check,deadline):
        job=self.job;check()
        if not self.speech.server_is_ready():
            raise ValueError('tts_unavailable')
        def await_speech(future,end):
            while not future.done():
                check()
                if time.monotonic()>=end:
                    raise ValueError('tts_timeout')
                time.sleep(.01)
            check()
            if time.monotonic()>=end:
                raise ValueError('tts_timeout')
            return future.result()
        sent=self.speech.send_goal_async(SpeakText.Goal(utterance_id='capture-'+uuid.uuid4().hex,text=text))
        abandoned=threading.Event()
        def late(done):
            try:
                handle=done.result()
                if handle.accepted and (abandoned.is_set() or job.finished.is_set() or job.cancel.is_set() or self.closing):
                    handle.cancel_goal_async()
            except Exception:
                pass
        sent.add_done_callback(late)
        try:
            handle=await_speech(sent,min(deadline,time.monotonic()+2.))
        except Exception:
            abandoned.set()
            if sent.done():
                late(sent)
            raise
        if not handle.accepted:
            raise ValueError('tts_rejected')
        job.speech=handle
        try:
            response=await_speech(handle.get_result_async(),deadline)
            if response.result.status!='completed':
                raise ValueError('tts_not_completed')
        finally:
            handle.cancel_goal_async();job.speech=None

    def wait(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            self.check();time.sleep(min(.02,end-time.monotonic()))
        self.check()

    def ros_now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def window(self):
        return time.monotonic(),self.ros_now()

    def capture(self,after,ros_after):
        self.ready()  # Recheck publisher multiplicity during bursts.
        return self.frames.capture(after,ros_after,self.job.cancel,self.catalog.defaults['frame_timeout_sec'],
                                   self.catalog.defaults['max_frame_age_sec'],self.check,self.ros_now)

    def upload(self,image,record):
        metadata={'X-Robot-Id':self.cfg['robot_id'],'X-Task-Type':record['task_type'],
                  'X-Frame-Index':str(record['frame_index']),'X-Capture-Time':record['capture_time'],
                  'X-Camera-Profile':self.cfg['camera_profile'],
                  'X-Quality':json.dumps(record['quality'],allow_nan=False)}
        sent=self.pool.submit(self.cloud.upload,record['task_id'],record['capture_id'],image,metadata,self.job.upload_cancel,self.job.deadline)
        try:
            return self.future(sent,max(.1,self.job.deadline-time.monotonic()))
        except Exception:
            self.job.upload_cancel.set();self.cloud.cancel()
            # Bounded HTTP timeout ensures worker drains before another job can upload.
            try:
                sent.result(timeout=3.5)
            except Exception:
                pass
            raise

    def record(self,record,image):
        self.check()
        root=Path(self.cfg['storage_dir']).expanduser()/record['capture_id'];root.mkdir(parents=True,exist_ok=True)
        temporary=root/'result.json.tmp';temporary.write_text(json.dumps(record,ensure_ascii=False,allow_nan=False),encoding='utf-8')
        temporary.replace(root/'result.json')
        if self.cfg['save_images']:
            self.job.pending_images.append((root/'image.jpg',image))
        self.job.records.append(record)

    def execute(self,goal):
        job=self.job;job.goal=goal
        status='failed';detail='';records=[]
        try:
            self.stage('ACCEPTED')
            records=CaptureFlow().run(self,dict(task_id=job.request.task_id,task_type=job.request.task_type),job.task,self.catalog.defaults)
            self.check()
            for image_path,image in job.pending_images:
                self.check();image_path.write_bytes(image)
            status='uploaded';detail=job.detail
        except Cancelled:
            status='cancelled';detail='cancelled'
        except Exception as exc:
            detail=str(exc) if isinstance(exc,(ValueError,CloudError)) else 'executor_'+type(exc).__name__
            # Only configured safe text may be spoken; arbitrary error strings never go to TTS.
            try:
                self.stage('FAILED',detail)
            except Exception:
                pass
            failed=failure_result(self,detail)
            status,detail=failed['status'],failed['detail']
        finally:
            job.finished.set();self.cloud.cancel()
            if job.speech:
                job.speech.cancel_goal_async()
            try:
                released=self.release_token(job.token)
            except Exception:
                released=None
                status='failed';detail='hold_release_unconfirmed'
            if released:
                until=time.monotonic()+2.
                while not released.done() and time.monotonic()<until:
                    time.sleep(.01)
                try:
                    release_ok=released.done() and released.result().success
                except Exception:
                    release_ok=False
                if not release_ok:
                    status='failed';detail='hold_release_unconfirmed'
            if status!='uploaded':
                for image_path,_ in job.pending_images:
                    image_path.unlink(missing_ok=True)
            job.pending_images.clear()
            data=dict(status=status,detail=detail,task_id=job.request.task_id,task_type=job.request.task_type,
                      capture_id=job.records[0]['capture_id'] if job.records else '',records=list(job.records))
            try:
                if job.phase!=status.upper() or job.detail!=detail:
                    self.stage(status.upper(),detail)
                self.write_event(data)
            except OSError:
                status='failed';data.update(status=status,detail='record_write_failed')
            self.result_pub.publish(String(data=json.dumps(data,ensure_ascii=False)))
            if goal.is_cancel_requested:
                goal.canceled()
            elif status=='uploaded':
                goal.succeed()
            else:
                goal.abort()
            result=CaptureTask.Result(status=data['status'],detail=data['detail'],capture_id=data['capture_id'],
                                      upload_json=json.dumps([r['receipt'] for r in job.records]))
            with self.lock:
                self.job=None
        return result

    def stop(self):
        self.closing=True
        if self.job:
            self.job.cancel.set()
            self.job.upload_cancel.set()
        self.cloud.cancel()

    def destroy_node(self):
        self.stop();self.pool.shutdown(wait=True,cancel_futures=True)
        self.server.destroy();self.speech.destroy()
        return super().destroy_node()


def main(args=None):
    from .runtime import run
    run(CaptureExecutor,args,threads=6)
