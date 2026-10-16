"""Real pose node with synthetic slab depth, detections and odometry; no camera/model."""
import copy
import os
import time
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image
from nav_msgs.msg import Odometry
from person_interfaces.msg import PersonStateArray
from yolo_person_tracker.pose_node import PoseTrackerNode
from yolo_person_tracker.backend import Detection
from monotonic_ros_clock import MonotonicRosClock
from test_height_continue_runtime import Backend, F, CX, CY, box, slab

assert os.environ.get('ROS_DOMAIN_ID')=='216'
rclpy.init()
values=dict(use_sim_time=True,depth_registered=True,max_age_s=1.,height_fall_enabled=True,
    height_continue_s=6.,height_continue_requires_stationary=True,pose_handover_s=1.5,
    pose3d_up_source='static',pose3d_up_x=0.,pose3d_up_y=-1.,pose3d_up_z=0.,
    pose3d_camera_height_m=1.,pose3d_gravity_confirmed=True,pose3d_ground_confirmed=True)
node=PoseTrackerNode(backend=Backend(),namespace='stationary_test',parameter_overrides=[Parameter(k,value=v) for k,v in values.items()])
ex=SingleThreadedExecutor();ex.add_node(node);clock=MonotonicRosClock(node)
people=[];continued=[]
node.create_subscription(PersonStateArray,'person_states',people.append,10)
node.create_subscription(PersonStateArray,'person_states_continued',continued.append,10)
cp=node.create_publisher(Image,node.cfg['color_topic'],10)
dp=node.create_publisher(Image,node.cfg['depth_topic'],10)
ip=node.create_publisher(CameraInfo,node.cfg['camera_info_topic'],10)
op=node.create_publisher(Odometry,'/odom',10)
flags=dict(odom=True,v=0.,w=0.,stale=False)
last_odom=0.

def spin(seconds=.02):
    global last_odom
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        clock.tick();now=time.monotonic()
        if flags['odom'] and now-last_odom>.02:
            last_odom=now
            o=Odometry();o.header.stamp=node.get_clock().now().to_msg()
            if flags['stale']:o.header.stamp.sec-=1
            o.twist.twist.linear.x=flags['v'];o.twist.twist.angular.z=flags['w'];op.publish(o)
        ex.spin_once(timeout_sec=.002)

def send(top,detections):
    Backend.detections=detections
    info=CameraInfo();info.width=640;info.height=480;info.header.frame_id='camera_color_optical_frame'
    info.p=[F,0.,CX,0.,0.,F,CY,0.,0.,0.,1.,0.];ip.publish(info);spin()
    c=node.bridge.cv2_to_imgmsg(np.zeros((480,640,3),np.uint8),'bgr8');c.header.frame_id=info.header.frame_id
    c.header.stamp=node.get_clock().now().to_msg()
    d=node.bridge.cv2_to_imgmsg(slab(top),'16UC1');d.header=copy.deepcopy(c.header);cp.publish(c);dp.publish(d)
    end=time.monotonic()+.7
    while time.monotonic()<end:
        spin(.005)
        p=next((p for p in reversed(people) if p.header==c.header),None)
        out=next((v for v in reversed(continued) if v.header==c.header),None)
        if p is not None and out is not None:return p,out
    raise AssertionError((node.error,people[-1:],continued[-1:]))

person=lambda top:Detection(7,box(top),.9,tuple((0.,0.,0.) for _ in range(17)))
def repeat(top,det,duration):
    end=time.monotonic()+duration
    while time.monotonic()<end:result=send(top,det)
    return result

try:
    flags['odom']=False;spin(.2)
    p,c=send(1.6,[person(1.6)]);assert not c.valid and not node.continuation_boxes
    flags['odom']=True;spin(.2)
    p,c=send(1.6,[person(1.6)]);assert not c.valid # less than .5 seconds
    repeat(1.6,[person(1.6)],.6)
    send(.7,[person(.7)])
    p,c=send(.3,[]);assert not p.persons and c.valid and len(c.persons)==1
    p,c=repeat(.3,[],1.4);assert c.persons[0].fall_stage==2
    # Motion immediately withdraws; a stopped camera must never resurrect a pre-motion box.
    flags['v']=.03;spin(.12);assert not continued[-1].valid and not node.continuation_boxes
    p,c=send(.3,[]);assert not c.valid and not c.persons
    flags['v']=0.;repeat(.3,[],.7)
    p,c=send(.3,[]);assert c.valid and not c.persons
    send(.3,[person(.3)]);p,c=send(.3,[]);assert c.persons
    flags['w']=.06;spin(.1);assert not continued[-1].valid and not node.continuation_boxes
    flags['w']=0.;repeat(.3,[person(.3)],.7);send(.3,[])
    flags['odom']=False;spin(.4);assert not continued[-1].valid and not node.continuation_boxes
    flags['odom']=True;repeat(.3,[],.7);p,c=send(.3,[]);assert not c.persons
    repeat(.3,[person(.3)],.2);send(.3,[])
    flags['stale']=True;spin(.1);assert not continued[-1].valid and not node.continuation_boxes
    assert node.count_publishers('/cmd_vel')==0
    print('PASS stationary continuation: missing odom, .5 s dwell, real continuation/confirmation, translation/rotation/stale/stream loss withdrawal, no old-box resurrection; zero motion publishers')
finally:
    ex.shutdown();node.destroy_node();rclpy.shutdown()
