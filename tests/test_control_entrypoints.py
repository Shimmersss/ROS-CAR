"""Exercise shipped entrypoints, custom frames, and SIGTERM final zero commands."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import yaml
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_prefix,get_package_share_directory
from geometry_msgs.msg import Twist,TwistStamped,TransformStamped
from sensor_msgs.msg import LaserScan
from person_interfaces.msg import TargetState
from std_srvs.srv import Trigger
from rcl_interfaces.srv import GetParameters
from tf2_ros import StaticTransformBroadcaster

rclpy.init();n=Node('control_entrypoint_probe');requests=[];vel=[]
target=n.create_publisher(TargetState,'/perception/target_state',1)
scan=n.create_publisher(LaserScan,'/scan',1)
n.create_subscription(TwistStamped,'/control/cmd_vel_request',requests.append,10)
n.create_subscription(Twist,'/cmd_vel',vel.append,10)
request=None;processes=[];logs=[]
b=StaticTransformBroadcaster(n);tr=TransformStamped();tr.header.frame_id='base_footprint';tr.child_frame_id='laser';tr.transform.rotation.w=1.;b.sendTransform(tr)
def feed():
    stamp=n.get_clock().now().to_msg()
    t=TargetState();t.header.stamp=stamp;t.header.frame_id='astra_depth_optical_frame';t.source='astra';t.target_id='test';t.status=t.TRACKING;t.position_valid=True;t.position.z=3.;t.horizontal_distance_m=3.;t.measurement_age_s=math.nan;target.publish(t)
    s=LaserScan();s.header.stamp=stamp;s.header.frame_id='laser';s.angle_min=-math.pi;s.angle_increment=math.pi/180;s.angle_max=s.angle_min+359*s.angle_increment;s.range_min=.15;s.range_max=15.;s.ranges=[3.]*360;scan.publish(s)
    if request is not None:
        m=TwistStamped();m.header.stamp=stamp;m.header.frame_id='base_footprint';m.twist.linear.x=.1;request.publish(m)
n.create_timer(.025,feed)
def wait(fn,timeout=8):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        rclpy.spin_once(n,timeout_sec=.02)
        if fn():return
    raise AssertionError('Timed out')
def spawn(cmd):
    log=tempfile.TemporaryFile(mode='w+');logs.append(log)
    p=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p);return p

def executable(pkg,name):return str(Path(get_package_prefix(pkg))/'lib'/pkg/name)
def term(p):
    p.send_signal(signal.SIGTERM);wait(lambda:p.poll() is not None)
    assert p.returncode==0,p.returncode
    until=time.monotonic()+.2;wait(lambda:time.monotonic()>until)
try:
    p=spawn([executable('astra_body_adapter','person_follower'),'--ros-args','-p','enabled:=true','-p','base_frame:=base_footprint'])
    wait(lambda:requests and requests[-1].twist.linear.x>0)
    assert requests[-1].header.frame_id=='base_footprint'
    mark=len(requests);term(p)
    assert len(requests)>mark and requests[-1].twist.linear.x==0
    wait(lambda:n.count_publishers('/control/cmd_vel_request')==0)
    request=n.create_publisher(TwistStamped,'/control/cmd_vel_request',1)
    args=[executable('motion_guard','guard'),'--ros-args']
    for param in ('motion_enabled:=true','geometry_confirmed:=true','mount_calibrated:=true','stopping_model_confirmed:=true','base_frame:=base_footprint'):args+=['-p',param]
    p=spawn(args);client=n.create_client(Trigger,'/control/arm');wait(client.service_is_ready)
    armed=False
    for _ in range(8):
        future=client.call_async(Trigger.Request());wait(future.done)
        if future.result().success:armed=True;break
        until=time.monotonic()+.3;wait(lambda:time.monotonic()>until)
    assert armed
    wait(lambda:vel and vel[-1].linear.x>0)
    mark=len(vel);term(p)
    assert len(vel)>mark and vel[-1].linear.x==0 and vel[-1].angular.z==0
    n.destroy_publisher(request);request=None
    wait(lambda:n.count_publishers('/cmd_vel')==0)
    config=yaml.safe_load((Path(get_package_share_directory('motion_guard'))/'config/safety.yaml').read_text())
    config['/**']['ros__parameters']['base_frame']='base_footprint'
    with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml') as cfg:
        yaml.safe_dump(config,cfg);cfg.flush()
        p=spawn(['ros2','launch','motion_guard','follow.launch.py','safety_config:='+cfg.name,'expected_source:=red_object','target_frame:=camera_color_optical_frame','performance_enabled:=false'])
        get=n.create_client(GetParameters,'/motion_guard/get_parameters');wait(get.service_is_ready)
        query=GetParameters.Request();query.names=['base_frame','target_frame','expected_source']
        future=get.call_async(query);wait(future.done)
        assert [v.string_value for v in future.result().values]==['base_footprint','camera_color_optical_frame','red_object']
        wait(lambda:n.count_publishers('/control/performance')==0)
        mark=len(requests);wait(lambda:len(requests)>mark)
        assert requests[-1].header.frame_id=='base_footprint'
        os.killpg(p.pid,signal.SIGINT);wait(lambda:p.poll() is not None)
    print('PASS custom-frame guarded launch and SIGTERM final-zero shutdown for follower/guard',flush=True)
finally:
    for p in processes:
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGKILL);p.wait()
    errors=[]
    for log in logs:
        log.seek(0);content=log.read()
        if 'Traceback' in content:errors.append(content)
        log.close()
    n.destroy_node();rclpy.shutdown()
    assert not errors, "\n".join(errors)
