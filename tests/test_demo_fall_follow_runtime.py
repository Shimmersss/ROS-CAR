"""Real follower + guard + owner node; synthetic inputs, no hardware or learned model."""
import json
import math
import os
import tempfile
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist, TwistStamped
from person_interfaces.msg import FallAlarm, TargetState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from astra_body_adapter.person_follower_node import PersonFollowerNode
from motion_guard.node import MotionGuard
from yolo_person_tracker.node import TrackerNode
from monotonic_ros_clock import MonotonicRosClock

assert os.environ.get('ROS_DOMAIN_ID') == '215'
class Backend:
    def infer(self,image):return []
    def reset(self):pass

rclpy.init()
params=lambda **kw:[Parameter(k,value=v) for k,v in dict(use_sim_time=True,**kw).items()]
probe=Node('demo_fall_probe',parameter_overrides=params())
clock=MonotonicRosClock(probe)
tmp=tempfile.TemporaryDirectory()
guard=MotionGuard(parameter_overrides=params(motion_enabled=True,geometry_confirmed=True,
    mount_calibrated=True,stopping_model_confirmed=True,emergency_kws_required=False,
    radar_required=False,command_mode='FOLLOW',auto_arm=True,target_auto_recover=True,
    target_grace_s=0.,expected_source='yolo',target_frame='camera_color_optical_frame',
    estop_latch_path=tmp.name+'/latch.json'))
follower=PersonFollowerNode(parameter_overrides=params(enabled=True,expected_source='yolo',pause_on_fall=True))
owner=TrackerNode(backend=Backend(),namespace='owner_test',parameter_overrides=params(
    owner_hold_on_fall=True,owner_lost_timeout_s=5.,owner_continuity_reset_s=2.))
ex=SingleThreadedExecutor()
for node in (probe,guard,follower,owner):ex.add_node(node)
target=probe.create_publisher(TargetState,'/perception/target_state',10)
fall=probe.create_publisher(FallAlarm,'/perception/fall_alarm',10)
owner_fall=probe.create_publisher(FallAlarm,'/owner_test/fall_alarm',10)
vel=[];requests=[];states=[]
probe.create_subscription(Twist,'/cmd_vel',vel.append,10)
probe.create_subscription(TwistStamped,'/control/cmd_vel_request',requests.append,10)
probe.create_subscription(String,'/control/state',lambda m:states.append(json.loads(m.data)),10)
flags=dict(stage=0,alarm=True,valid=True,target=True,stale=False)
last_send=0.

def pump(seconds):
    global last_send
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        clock.tick()
        for node in (guard,follower,owner):node.get_clock().set_ros_time_override(clock.clock.now())
        now=time.monotonic()
        if now-last_send>.025:
            last_send=now
            t=TargetState();t.header.stamp=probe.get_clock().now().to_msg();t.header.frame_id='camera_color_optical_frame'
            t.observation_stamp=t.header.stamp;t.measurement_age_s=0.;t.source='yolo';t.target_id='0:7'
            t.status=t.TRACKING if flags['target'] else t.LOST;t.position_valid=flags['target']
            t.position.z=3.;t.horizontal_distance_m=3.;t.bearing_rad=0.;target.publish(t)
            if flags['alarm']:
                a=FallAlarm();a.header=t.header;a.valid=flags['valid'];a.stage=flags['stage']
                if flags['stale']:a.header.stamp.sec-=1
                fall.publish(a);owner_fall.publish(a)
        ex.spin_once(timeout_sec=.002)

def zeros():
    assert vel and all(v.linear.x==0. and v.angular.z==0. for v in vel[-3:]),vel[-3:]
    assert requests and all(v.twist.linear.x==0. for v in requests[-3:])

def call(name):
    c=probe.create_client(Trigger,name);assert c.wait_for_service(timeout_sec=2.)
    f=c.call_async(Trigger.Request());end=time.monotonic()+2.
    while not f.done() and time.monotonic()<end:pump(.02)
    assert f.done();response=f.result();probe.destroy_client(c);return response

try:
    pump(.7);assert vel[-1].linear.x>0.
    owner.owner.claim();pump(.3)
    owner_lost_before=owner.owner.lost_for(time.monotonic())
    flags['stage']=1;pump(.2);zeros()
    assert not call('/demo/resume_follow').success
    flags['stage']=2;pump(5.4);zeros()
    assert owner.owner.state=='LOST' and owner.owner.lost_for(time.monotonic())<owner_lost_before+.2
    assert not owner.owner.continuity_reset_done
    flags['target']=False;pump(.3);zeros()
    flags['target']=True;pump(.3);zeros() # guard target recovery must not override fall pause
    flags['alarm']=False;pump(.7);zeros()
    flags.update(alarm=True,stage=0,valid=False);pump(.4);zeros()
    flags.update(valid=True,stale=True);pump(.4);zeros()
    flags['stale']=False;pump(2.8);zeros()
    pump(.4);assert vel[-1].linear.x>0.
    assert owner.owner.state=='LOST' # hold released, not expired immediately
    remaining=owner.owner.timeout_s-owner.owner.lost_for(time.monotonic())
    pump(remaining+.15);assert owner.owner.state=='UNCLAIMED'
    owner.owner.claim();flags['stage']=2;pump(.2)
    assert owner.owner.held_at is not None
    assert call('/owner_test/release_target').success
    pump(.2);assert owner.owner.state=='UNCLAIMED' and not owner.owner_fall_pause.paused
    # Manual release of a pause requires fresh NONE; it never grants authority.
    flags['stage']=1;pump(.2);zeros()
    flags['stage']=0;pump(.15)
    assert call('/demo/resume_follow').success
    pump(.2);assert vel[-1].linear.x>0.
    flags['stage']=2;pump(.2);zeros()
    assert call('/control/estop').success
    flags['stage']=0;pump(.2);assert call('/demo/resume_follow').success
    pump(.3);zeros();assert states[-1]['authority_state']=='ESTOP'
    assert probe.count_publishers('/cmd_vel')==1
    print('PASS fall pause: suspect/confirm, lost target recovery, stale/invalid alarms, continuous NONE, manual resume, ESTOP; owner held >5 s then expires after remaining duration')
finally:
    ex.shutdown()
    for node in (owner,follower,guard,probe):node.destroy_node()
    rclpy.shutdown();tmp.cleanup()
