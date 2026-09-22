"""Synchronized red RGB-D list semantics, no hardware or RGB-only extension."""
import copy
import math
import time
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray
from red_object_tracker.node import RedTrackerNode
rclpy.init();node=RedTrackerNode(parameter_overrides=[Parameter('depth_registered',value=True)])
ex=SingleThreadedExecutor();ex.add_node(node);received=[]
node.create_subscription(Detection2DArray,'detections',received.append,10)
def spin():
    until=time.monotonic()+.12
    while time.monotonic()<until:ex.spin_once(timeout_sec=.005)
def send(empty=False,old=False):
    info=CameraInfo();info.width=info.height=100;info.header.frame_id='camera_optical'
    info.p=[100.,0.,50.,0.,0.,100.,50.,0.,0.,0.,1.,0.];node.on_info(info)
    pixels=np.zeros((100,100,3),np.uint8)
    if not empty:pixels[10:40,10:40,2]=255;pixels[60:80,60:80,2]=255
    color=node.bridge.cv2_to_imgmsg(pixels,'bgr8');color.header.frame_id='camera_optical';color.header.stamp=node.get_clock().now().to_msg()
    if old:color.header.stamp.sec-=3
    depth=node.bridge.cv2_to_imgmsg(np.full((100,100),1500,np.uint16),'16UC1');depth.header=copy.deepcopy(color.header)
    node.on_pair(color,depth);spin();return color
try:
    spin();color=send();assert len(received[-1].detections)==2
    assert received[-1].header==color.header
    assert all(d.id=='' and math.isnan(d.results[0].hypothesis.score) for d in received[-1].detections)
    send();send()
    assert sum(bool(d.id) for d in received[-1].detections)==1
    assert all(d.results[0].hypothesis.class_id=='red_object' for d in received[-1].detections)
    assert node.snapshot[3] is not None
    send(empty=True);assert received[-1].detections==[]
    count=len(received);send(old=True);assert len(received)==count
    spin();assert len(received)==count
    print('PASS red RGB-D: all candidates, original header, NaN score, confirmed ID only, empty vs failure/stall')
finally:
    node.destroy_node();rclpy.shutdown()
