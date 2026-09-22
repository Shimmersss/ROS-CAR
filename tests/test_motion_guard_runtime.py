"""ROS gate fault injection: real services, TF, publishers; no hardware."""
import copy
import json
import math
import os
import pty
import struct
import subprocess
import tempfile
import threading
import signal
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TwistStamped, TransformStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster, Buffer
from person_interfaces.msg import TargetState
from motion_guard.node import MotionGuard
from roscar_interfaces.srv import SetControlMode

rclpy.init(); probe=Node('guard_test'); ex=SingleThreadedExecutor(); ex.add_node(probe)
# Real C++ chassis driver on a PTY; recorded frames never reach hardware.
master,slave=pty.openpty();os.set_blocking(master,False)
driver_log=tempfile.TemporaryFile(mode='w+')
driver=subprocess.Popen(['ros2','run','turn_on_wheeltec_robot','wheeltec_robot_node',
    '--ros-args','-p','usart_port_name:='+os.ttyname(slave),'-p','car_mode:=mini_mec'],
    stdout=driver_log,stderr=subprocess.STDOUT,start_new_session=True)
wire_frames=[];wire_stop=threading.Event()
def xor(data):
    out=0
    for b in data:out^=b
    return out
def wire_loop():
    payload=bytes([0x7b,0])+struct.pack('>10h',0,0,0,0,0,16384,0,0,0,12000)
    feedback=payload+bytes([xor(payload),0x7d]);buf=bytearray()
    while not wire_stop.is_set():
        os.write(master,feedback)
        try:buf.extend(os.read(master,65536))
        except BlockingIOError:pass
        while len(buf)>=11:
            if buf[0]!=0x7b or buf[10]!=0x7d or xor(buf[:9])!=buf[9]:
                del buf[0];continue
            wire_frames.append(struct.unpack('>hhh',buf[3:9]));del buf[:11]
        time.sleep(.02)
wire_thread=threading.Thread(target=wire_loop,daemon=True);wire_thread.start()
params=[Parameter(k,value=v) for k,v in dict(motion_enabled=True,geometry_confirmed=True,stopping_model_confirmed=True,mount_calibrated=True).items()]
guard=MotionGuard(parameter_overrides=params);ex.add_node(guard)
req=probe.create_publisher(TwistStamped,'/control/cmd_vel_request',1)
scans=probe.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
targets=probe.create_publisher(TargetState,'/perception/target_state',1)
vel=[];states=[]
probe.create_subscription(Twist,'/cmd_vel',vel.append,1)
probe.create_subscription(String,'/control/state',lambda m:states.append(json.loads(m.data)),10)
broadcaster=StaticTransformBroadcaster(probe)
tr=TransformStamped();tr.header.frame_id='base_link';tr.child_frame_id='laser';tr.transform.rotation.w=1.
broadcaster.sendTransform(tr)
arm=probe.create_client(Trigger,'/control/arm');stop=probe.create_client(Trigger,'/control/stop')
flags=dict(scan=True,target=True,request=True,obstacle=False,simulated=False,old_request=False,old_target=False,lost=False)
def pump(duration):
    end=time.monotonic()+duration; last=0
    while time.monotonic()<end:
        now=time.monotonic()
        if now-last>.025:
            last=now;stamp=probe.get_clock().now().to_msg()
            if flags['request']:
                m=TwistStamped();m.header.stamp=copy.deepcopy(stamp);m.header.frame_id='base_link'
                m.twist.linear.x=.1
                if flags['old_request']:m.header.stamp.sec-=3
                req.publish(m)
            if flags['target']:
                m=TargetState();m.header.stamp=copy.deepcopy(stamp);m.header.frame_id='astra_depth_optical_frame'
                m.source='astra';m.status=m.LOST if flags['lost'] else m.TRACKING
                m.position_valid=True;m.position.z=3.;m.horizontal_distance_m=3.;m.bearing_rad=0.
                m.measurement_age_s=math.nan;m.is_simulated=flags['simulated']
                if flags['old_target']:m.header.stamp.sec-=3
                targets.publish(m)
            if flags['scan']:
                m=LaserScan();m.header.stamp=stamp;m.header.frame_id='laser'
                m.angle_min=-math.pi;m.angle_increment=math.pi/180;m.angle_max=math.pi-m.angle_increment
                m.range_min=.15;m.range_max=15.;m.ranges=[3.]*360
                if flags['obstacle']:m.ranges[180]=.3
                scans.publish(m)
        ex.spin_once(timeout_sec=.005)
def call(client):
    assert client.wait_for_service(timeout_sec=2)
    f=client.call_async(Trigger.Request());end=time.monotonic()+2
    while not f.done() and time.monotonic()<end:pump(.02)
    assert f.done();return f.result()
def stopped():
    assert vel and all(v.linear.x==0 and v.angular.z==0 for v in vel[-3:]),vel[-3:]
    assert wire_frames and wire_frames[-1]==(0,0,0),wire_frames[-3:]
def enable():
    pump(.35);result=call(arm);assert result.success,result.message
    pump(.3);assert vel[-1].linear.x>0
    assert any(v[0]==100 for v in wire_frames[-5:]),wire_frames[-5:]
try:
    pump(1.5);assert states[-1]['mode']=='STANDBY';stopped()
    for key in ('obstacle','simulated','old_request','old_target','lost'):
        enable();flags[key]=True;pump(.25)
        assert states[-1]['mode']=='FAULT',(key,states[-1]);stopped()
        assert not call(arm).success
        flags[key]=False;pump(.35);assert states[-1]['mode']=='FAULT';stopped()
    for key in ('scan','request','target'):
        enable();flags[key]=False;pump(.7);assert states[-1]['mode']=='FAULT';stopped()
        flags[key]=True;pump(.35);stopped()
    enable();extra=probe.create_publisher(TwistStamped,'/control/cmd_vel_request',1)
    pump(.4);assert states[-1]['mode']=='FAULT';stopped();probe.destroy_publisher(extra)
    enable();extra=probe.create_publisher(Twist,'/cmd_vel',1)
    pump(.4);assert states[-1]['mode']=='FAULT';stopped();probe.destroy_publisher(extra)
    enable();old_buffer=guard.buffer;guard.buffer=Buffer();pump(.25);assert states[-1]['mode']=='FAULT';stopped()
    guard.buffer=old_buffer
    broadcaster.sendTransform(tr);pump(.3);stopped()
    enable();assert call(stop).success;pump(.2);assert states[-1]['mode']=='STANDBY';stopped()
    # Same real driver PTY, now independently controlled without any visual target.
    mode_client=probe.create_client(SetControlMode,'/control/set_mode')
    assert mode_client.wait_for_service(timeout_sec=2)
    mode_request=SetControlMode.Request();mode_request.mode='EXTERNAL'
    future=mode_client.call_async(mode_request)
    while not future.done():pump(.02)
    assert future.result().success
    pump(.15);stopped()
    flags['target']=False
    follow_req=req
    req=probe.create_publisher(TwistStamped,'/chassis/cmd_vel',1)
    enable()
    flags['obstacle']=True;pump(.25);assert states[-1]['mode']=='FAULT';stopped()
    flags['obstacle']=False;enable()
    assert call(stop).success;pump(.2);stopped()
    probe.destroy_publisher(req);req=follow_req;flags['target']=True
    ex.remove_node(guard);guard.destroy_node()
    guard=MotionGuard(parameter_overrides=params);ex.add_node(guard)
    broadcaster.sendTransform(tr);pump(.5);assert states[-1]['mode']=='STANDBY';stopped()
    ex.remove_node(guard);guard.destroy_node()
    guard=MotionGuard();ex.add_node(guard);pump(.5)
    assert states[-1]['mode']=='PERCEPTION_ONLY';assert not call(arm).success;stopped()
    print('PASS gate ROS + real chassis PTY: arm/stop, source/replay/loss, scan/request/target timeout, duplicate requests, TF loss, fault latch and restart disarmed')
finally:
    ex.remove_node(guard);guard.destroy_node();ex.remove_node(probe);probe.destroy_node();rclpy.shutdown()
    os.killpg(driver.pid,signal.SIGINT)
    try:driver.wait(timeout=4)
    except subprocess.TimeoutExpired:os.killpg(driver.pid,signal.SIGKILL);driver.wait()
    wire_stop.set();wire_thread.join(timeout=2)
    os.close(master);os.close(slave);driver_log.close()
