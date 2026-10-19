"""Actual ROS manager + guard, synthetic resources; never a hardware test."""
import json
import os
import tempfile
import threading
import time
from pathlib import Path
from unittest.mock import patch
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped, Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from person_interfaces.msg import TargetState
from person_interfaces.srv import ClaimOwner
from demo_interfaces.srv import SetRuntimeMode, CaptureFace
from capture_task_interfaces.srv import RunInspection, RunCapture
from capture_task_interfaces.action import CaptureTask
from motion_guard.node import MotionGuard
from demo_motion.runtime_node import RuntimeManager

assert os.environ.get('ROS_DOMAIN_ID') == '228' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
rclpy.init()
executor=MultiThreadedExecutor(num_threads=8)
probe=Node('runtime_probe');executor.add_node(probe)
tmp=tempfile.TemporaryDirectory()
class Module:
    current=None
    def __init__(self,root):self.node=None;self.server=None;self.started=[];self.running=False;Module.current=self
    def active(self):return self.running
    def stop(self):
        self.running=False
        if self.server:
            executor.remove_node(self.node)
            time.sleep(.2)  # Let completed action callbacks leave the executor before fixture teardown.
            self.server.destroy();self.server=None
        if self.node:
            executor.remove_node(self.node);self.node.destroy_node();self.node=None
    def start(self,mode):
        assert not self.running
        self.running=True;self.started.append(mode)
        if mode in ('FOLLOW','PERCEPTION'):
            n=self.node=Node('synthetic_perception');executor.add_node(n)
            pub=n.create_publisher(TargetState,'/perception/target_state',10)
            n.create_service(ClaimOwner,'/perception/claim_owner',lambda req,res:claim(req,res))
            def publish():
                t=TargetState();t.header.stamp=n.get_clock().now().to_msg();t.header.frame_id='camera_color_optical_frame'
                t.observation_stamp=t.header.stamp;t.status=t.TRACKING;t.position_valid=True;t.source='yolo';t.target_id='0:7'
                t.position.z=3.;t.horizontal_distance_m=3.;t.measurement_age_s=0.;pub.publish(t)
            n.create_timer(.03,publish)
        elif mode in ('FACE','FACE_UPLOAD','PCB','HELMET','TONGUE'):
            n=self.node=Node('synthetic_capture');executor.add_node(n)
            state=n.create_publisher(String,'/capture_task/state',10)
            camera=n.create_publisher(Image,'/camera/color/image_raw',1)
            n.create_timer(.05,lambda:state.publish(String(data=json.dumps(dict(camera_ok=True,tts_ok=True,active=False,task_mode='face' if mode=='FACE_UPLOAD' else mode.lower(),pipeline_mode='collect' if mode=='FACE' else 'upload' if mode=='FACE_UPLOAD' else 'infer')))))
            def execute(handle):
                if handle.request.task_id=='slow-test':
                    end=time.monotonic()+20
                    while not handle.is_cancel_requested and time.monotonic()<end:time.sleep(.02)
                if handle.is_cancel_requested:
                    handle.canceled();return CaptureTask.Result(status='cancelled')
                handle.succeed()
                return CaptureTask.Result(status='captured' if mode=='FACE' else 'uploaded' if mode=='FACE_UPLOAD' else 'analyzed',capture_id='synthetic-only',upload_json='[]')
            self.server=ActionServer(n,CaptureTask,'/capture_task/run',execute,
                cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=ReentrantCallbackGroup())

def claim(req,res):res.success=True;return res
params=lambda **kw:[Parameter(k,value=v) for k,v in kw.items()]
guard=MotionGuard(parameter_overrides=params(motion_enabled=True,geometry_confirmed=True,mount_calibrated=True,
 stopping_model_confirmed=True,emergency_kws_required=False,radar_required=False,expected_source='yolo',
 target_frame='camera_color_optical_frame',command_mode='IDLE',estop_latch_path=tmp.name+'/latch.json'))
executor.add_node(guard)
with patch('demo_motion.runtime_node.ModuleOwner',Module):
 manager=RuntimeManager(parameter_overrides=params(project_root=str(Path(__file__).parents[1])))
executor.add_node(manager)
thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
vel=[];probe.create_subscription(Twist,'/cmd_vel',vel.append,10)
follow=probe.create_publisher(TwistStamped,'/demo/follow_request',10)
manual=probe.create_publisher(TwistStamped,'/robot/cmd_vel',10)
stop_flag=threading.Event()
def follow_loop():
 while not stop_flag.wait(.03):
  m=TwistStamped();m.header.stamp=probe.get_clock().now().to_msg();m.header.frame_id='base_link';m.twist.linear.x=.15;follow.publish(m)
producer=threading.Thread(target=follow_loop,daemon=True);producer.start()
def until(test,timeout=8):
 end=time.monotonic()+timeout
 while not test():
  assert time.monotonic()<end,(manager.phase,manager.mode,manager.detail,manager.control)
  time.sleep(.03)
def call(name,typ,request):
 client=probe.create_client(typ,name);assert client.wait_for_service(timeout_sec=3)
 f=client.call_async(request);until(f.done,30);result=f.result();probe.destroy_client(client);return result
try:
 until(lambda:manager.phase=='IDLE')
 assert not Module.current.started and guard.authority.state=='DISARMED'
 # Gate buffers only zeros through authorization, even if follower already requests motion.
 result=call('/robot/set_mode',SetRuntimeMode,SetRuntimeMode.Request(mode='FOLLOW'))
 assert result.success,result.detail
 until(lambda:vel and vel[-1].linear.x>0)
 assert manager.mode=='FOLLOW'
 old=TwistStamped();old.header.stamp=probe.get_clock().now().to_msg();old.header.frame_id='base_link';old.twist.linear.x=.2
 result=call('/robot/set_mode',SetRuntimeMode,SetRuntimeMode.Request(mode='ROS_CONTROL'))
 assert result.success,result.detail
 assert not Module.current.active()
 manual.publish(old);time.sleep(.1);assert vel[-1].linear.x==0.,'pre-authorization input must not be replayed'
 m=TwistStamped();m.header.stamp=probe.get_clock().now().to_msg();m.header.frame_id='base_link';m.twist.linear.x=.1;m.twist.angular.z=1.
 manual.publish(m);until(lambda:vel[-1].linear.x>0)
 assert abs(vel[-1].angular.z)<=.25
 time.sleep(.4);assert vel[-1].linear.x==0.,'lost teleop input must brake'
 # No face inference, no heavy-mode start and no guessed model output.
 before=list(Module.current.started)
 result=call('/inspect/face',RunInspection,RunInspection.Request())
 assert not result.success and 'model_not_integrated' in result.detail
 assert Module.current.started==before
 result=call('/capture/face',RunCapture,RunCapture.Request(prepare_sec=0.,timeout_sec=5.))
 assert result.success and result.status=='uploaded' and 'model_not_integrated' in result.detail
 assert manager.mode=='IDLE' and not Module.current.active()
 result=call('/face/capture',CaptureFace,CaptureFace.Request(prepare_sec=0.,timeout_sec=5.))
 assert result.success and result.status=='captured' and 'model_not_integrated' in result.detail
 assert manager.mode=='IDLE' and not Module.current.active()
 capture_client=probe.create_client(RunInspection,'/inspect/pcb')
 pending=capture_client.call_async(RunInspection.Request(task_id='slow-test',prepare_sec=0.,timeout_sec=30.))
 until(lambda:manager.capture_handle is not None and manager.capture_handle.accepted)
 result=call('/robot/set_mode',SetRuntimeMode,SetRuntimeMode.Request(mode='PERCEPTION'))
 assert result.success,result.detail
 until(pending.done);assert not pending.result().success
 assert Module.current.started[-2:]==['PCB','PERCEPTION']
 result=call('/robot/stop',Trigger,Trigger.Request());assert result.success,result.message
 until(lambda:manager.phase=='IDLE');assert guard.authority.state=='DISARMED'
 # STOP preempts a FOLLOW startup that cannot find a person and must never arm later.
 Module.current.start=lambda mode: setattr(Module.current,'running',True)
 client=probe.create_client(SetRuntimeMode,'/robot/set_mode');f=client.call_async(SetRuntimeMode.Request(mode='FOLLOW'))
 until(lambda:manager.phase=='STARTING')
 result=call('/robot/stop',Trigger,Trigger.Request());assert result.success,result.message
 until(f.done);assert not f.result().success
 assert guard.authority.state=='DISARMED' and manager.mode=='IDLE'
 print('PASS: cold IDLE, zero handshake, FOLLOW -> ROS_CONTROL, stale teleop stop, face unavailable, startup cancellation')
finally:
 stop_flag.set();manager.close();executor.shutdown(timeout_sec=3);thread.join(timeout=3)
 for n in (manager,guard,probe):n.destroy_node()
 rclpy.shutdown();tmp.cleanup()
