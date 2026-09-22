"""Public mode service, unselected sources and invalid external input; synthetic ROS only."""
import copy
import json
import math
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
from roscar_interfaces.srv import SetControlMode
from motion_guard.node import MotionGuard

rclpy.init()
p = Node('api_test'); ex=SingleThreadedExecutor(); ex.add_node(p)
g=MotionGuard(parameter_overrides=[Parameter(k,value=v) for k,v in dict(
    command_mode='IDLE',motion_enabled=True,geometry_confirmed=True,
    stopping_model_confirmed=True,mount_calibrated=True).items()]); ex.add_node(g)
ext=p.create_publisher(TwistStamped,'/chassis/cmd_vel',1)
follow=p.create_publisher(TwistStamped,'/control/cmd_vel_request',1)
scan=p.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
vel=[]; states=[]
p.create_subscription(Twist,'/cmd_vel',vel.append,10)
p.create_subscription(String,'/control/state',lambda m:states.append(json.loads(m.data)),10)
tf=StaticTransformBroadcaster(p); t=TransformStamped();t.header.frame_id='base_link';t.child_frame_id='laser';t.transform.rotation.w=1.;tf.sendTransform(t)
flags=dict(send=True,obstacle=False,old=False,field=None,value=0.,frame='base_link')
mode=p.create_client(SetControlMode,'/control/set_mode');arm=p.create_client(Trigger,'/control/arm')
def pump(duration):
    end=time.monotonic()+duration;last=0
    while time.monotonic()<end:
        if time.monotonic()-last>.025:
            last=time.monotonic(); stamp=p.get_clock().now().to_msg()
            m=TwistStamped();m.header.stamp=copy.deepcopy(stamp);m.header.frame_id=flags['frame'];m.twist.linear.x=.1
            if flags['old']:m.header.stamp.sec-=10
            if flags['field']:
                group,field=flags['field'].split('.');setattr(getattr(m.twist,group),field,flags['value'])
            if flags['send']:ext.publish(m)
            m.twist.linear.x=.14; follow.publish(m)
            s=LaserScan();s.header.stamp=stamp;s.header.frame_id='laser';s.angle_min=-math.pi;s.angle_increment=math.pi/180;s.angle_max=math.pi-s.angle_increment
            s.range_min=.15;s.range_max=15.;s.ranges=[3.]*360
            if flags['obstacle']:s.ranges[180]=.3
            scan.publish(s)
        ex.spin_once(timeout_sec=.003)
def call(client,req):
    assert client.wait_for_service(timeout_sec=2)
    f=client.call_async(req);end=time.monotonic()+2
    while not f.done() and time.monotonic()<end:pump(.01)
    assert f.done();return f.result()
def set_mode(value):
    q=SetControlMode.Request();q.mode=value;return call(mode,q)
def stopped():
    pump(.15);assert all(v.linear.x==0 and v.angular.z==0 for v in vel[-2:]),states[-1]
def enable():
    pump(.3);r=call(arm,Trigger.Request());assert r.success,r.message
    pump(.15);assert vel[-1].linear.x==.1,states[-1]
try:
    pump(.6);assert states[-1]['command_mode']=='IDLE';stopped()
    assert not call(arm,Trigger.Request()).success
    assert not set_mode('external').success;assert g.command_mode=='IDLE'
    assert set_mode('EXTERNAL').success;enable() # No target publisher exists.
    assert not set_mode('INVALID').success;assert g.mode=='ARMED'
    assert set_mode('EXTERNAL').success;stopped();assert g.mode=='STANDBY'
    flags['send']=False
    assert set_mode('FOLLOW').success;stopped();assert g.request is not None
    assert not call(arm,Trigger.Request()).success # Missing target still blocks FOLLOW.
    assert set_mode('EXTERNAL').success;stopped();assert g.request is None
    assert not call(arm,Trigger.Request()).success # Unselected FOLLOW cannot fill request.
    flags['send']=True; flags['old']=True;pump(.2);assert g.request is None
    flags['old']=False;enable()
    for field,value in [('linear.x',-.01),('linear.x',.16),('linear.y',.01),('linear.z',.01),('angular.x',.01),('angular.y',.01),('angular.z',.51),('angular.z',math.nan),('linear.x',math.inf)]:
        flags['field']=field;flags['value']=value;pump(.15);assert g.mode=='FAULT';stopped()
        assert not call(arm,Trigger.Request()).success
        flags['field']=None;pump(.2);assert g.mode=='FAULT';enable()
    flags['frame']='camera_optical';pump(.2);assert g.mode=='FAULT';stopped()
    flags['frame']='base_link';enable()
    flags['send']=False;pump(.35);assert g.mode=='FAULT';stopped()
    flags['send']=True;enable()
    flags['obstacle']=True;pump(.2);assert g.mode=='FAULT';stopped()
    flags['obstacle']=False;enable()
    duplicate=p.create_publisher(TwistStamped,'/chassis/cmd_vel',1);pump(.3);assert g.mode=='FAULT';stopped()
    p.destroy_publisher(duplicate);enable()
    saved=g.buffer;g.buffer=Buffer();pump(.2);assert g.mode=='FAULT';stopped();g.buffer=saved;enable()
    assert set_mode('IDLE').success;stopped();assert g.request is None
    print('PASS API modes: no-target EXTERNAL, FOLLOW rejection, switching stop/disarm/flush, invalid modes, late requests, unselected sources, velocity validation, watchdog, obstacle, TF, duplicate source and explicit re-arm')
finally:
    ex.remove_node(g);g.destroy_node();p.destroy_node();rclpy.shutdown()
