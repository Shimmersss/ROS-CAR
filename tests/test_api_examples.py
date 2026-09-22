"""Installed launch/CLI examples on synthetic data in an isolated ROS domain."""
import os
import json
import math
import signal
import subprocess
import tempfile
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Header
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster
from vision_msgs.msg import Detection2DArray
from astra_body_adapter.detections import detection_array
from motion_guard.node import MotionGuard

rclpy.init(); p=Node('api_examples_test');ex=SingleThreadedExecutor();ex.add_node(p)
processes=[];logs=[];guard=None
states=[];vel=[]
p.create_subscription(String,'/control/state',lambda m:states.append(json.loads(m.data)),10)
p.create_subscription(Twist,'/cmd_vel',vel.append,10)
scan=p.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
dets=p.create_publisher(Detection2DArray,'/perception/detections',10)
tf=StaticTransformBroadcaster(p)
tr=TransformStamped();tr.header.frame_id='base_link';tr.child_frame_id='laser';tr.transform.rotation.w=1.
def pump(duration):
    until=time.monotonic()+duration;last=0
    while time.monotonic()<until:
        if time.monotonic()-last>.025:
            last=time.monotonic()
            msg=LaserScan();msg.header.stamp=p.get_clock().now().to_msg();msg.header.frame_id='laser'
            msg.angle_min=-math.pi;msg.angle_increment=math.pi/180;msg.angle_max=math.pi-msg.angle_increment
            msg.range_min=.15;msg.range_max=15.;msg.ranges=[3.]*360;scan.publish(msg)
            header=Header();header.stamp=msg.header.stamp;header.frame_id='camera_optical'
            dets.publish(detection_array(header,[((2,3,20,40),'person',.8,'0:7')]))
        ex.spin_once(timeout_sec=.003)
def start(command):
    log=tempfile.TemporaryFile(mode='w+');logs.append(log)
    proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    processes.append(proc);return proc,log
def output(log):
    log.seek(0);return log.read()
def stop(proc):
    if proc.poll() is None:
        os.killpg(proc.pid,signal.SIGINT)
        end=time.monotonic()+5
        while proc.poll() is None and time.monotonic()<end:pump(.03)
        if proc.poll() is None:os.killpg(proc.pid,signal.SIGKILL);proc.wait();raise AssertionError('Process did not exit')
try:
    launched,log=start(['ros2','launch','roscar_api','api.launch.py'])
    pump(2)
    assert states and states[-1]['command_mode']=='IDLE' and states[-1]['mode']=='PERCEPTION_ONLY',output(log)
    assert all(v.linear.x==0 for v in vel)
    assert not any(name in p.get_node_names() for name in ('person_follower','wheeltec_robot','roscar_n10p'))
    stop(launched);pump(.3)
    guard=MotionGuard(parameter_overrides=[Parameter(k,value=v) for k,v in dict(command_mode='IDLE',
        motion_enabled=True,geometry_confirmed=True,stopping_model_confirmed=True,mount_calibrated=True).items()])
    ex.add_node(guard);tf.sendTransform(tr)
    chassis,log=start(['ros2','run','roscar_api','chassis','--duration','.4'])
    pump(1.3);assert chassis.poll() is None,output(log)
    assert guard.command_mode=='EXTERNAL' and guard.mode=='STANDBY'
    assert all(v.linear.x==0 for v in vel[-5:])
    arm=p.create_client(Trigger,'/control/arm');assert arm.wait_for_service(timeout_sec=2)
    future=arm.call_async(Trigger.Request())
    while not future.done():pump(.01)
    assert future.result().success,future.result().message
    pump(1.2);assert chassis.poll()==0,output(log)
    assert any(v.linear.x==.05 for v in vel)
    assert guard.mode=='STANDBY' and vel[-1].linear.x==0
    print('PASS installed default API launch; chassis example zero/explicit arm/velocity/final stop')
    readers=[]
    for name in ('detections','radar'):
        readers.append(start(['ros2','run','roscar_api',name]))
    pump(1.3)
    for proc,log in readers:
        stop(proc);assert ('person' in output(log) or '360 rays' in output(log)),output(log)
    def success(req,res):res.success=True;res.message='synthetic target service';return res
    services=[p.create_service(Trigger,'/perception/'+name+'_target',success) for name in ('lock','release')]
    for name in ('lock','release'):
        proc,log=start(['ros2','run','roscar_api','target',name]);pump(1.2)
        assert proc.poll()==0,output(log)
    print('PASS installed detection/radar subscriptions and lock/release service example')
finally:
    for proc in processes:
        if proc.poll() is None:
            os.killpg(proc.pid,signal.SIGKILL);proc.wait()
    for log in logs:log.close()
    if guard is not None:ex.remove_node(guard);guard.destroy_node()
    p.destroy_node();rclpy.shutdown()
