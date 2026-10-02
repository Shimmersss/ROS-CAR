"""Real ROS registration node with synthetic calibration/images; not camera acceptance."""
import copy
import json
from pathlib import Path
import tempfile
import time
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image, CameraInfo
from yolo_person_tracker.gemini_registration import GeminiRegistration


def main():
    rclpy.init()
    with tempfile.TemporaryDirectory() as folder:
        intrinsic=dict(width=4,height=3,fx=2.,fy=2.,cx=1.5,cy=1.)
        data=dict(serial='synthetic-fixture',color_intrinsic=intrinsic,depth_intrinsic=intrinsic,
                  color_distortion=[0.]*8,depth_distortion=[0.]*8,depth_to_color_rotation=np.eye(3).reshape(-1).tolist(),depth_to_color_translation_mm=[0.]*3)
        path=Path(folder)/'calibration.json';path.write_text(json.dumps(data))
        node=GeminiRegistration(parameter_overrides=[Parameter('calibration_file',value=str(path)),Parameter('expected_serial',value=data['serial'])])
        ex=SingleThreadedExecutor();ex.add_node(node)
        colors=[];depths=[];infos=[]
        node.create_subscription(Image,'/camera/color/image_rect',colors.append,10)
        node.create_subscription(Image,'/camera/aligned_depth_to_color/image_raw',depths.append,10)
        node.create_subscription(CameraInfo,'/camera/color/camera_info_rect',infos.append,10)
        cp=node.create_publisher(Image,'/camera/color/image_raw',10);dp=node.create_publisher(Image,'/camera/depth/image_raw',10)
        def spin(duration=.2):
            end=time.monotonic()+duration
            while time.monotonic()<end:ex.spin_once(timeout_sec=.01)
        def send(bad_frame=False,old=False,bad_size=False):
            c=node.bridge.cv2_to_imgmsg(np.full((3,4,3),42,np.uint8),'bgr8');c.header.frame_id='camera_color_optical_frame';c.header.stamp=node.get_clock().now().to_msg()
            if old:c.header.stamp.sec-=2
            d=node.bridge.cv2_to_imgmsg(np.full((4 if bad_size else 3,4),2000,np.uint16),'16UC1');d.header=copy.deepcopy(c.header);d.header.frame_id='bad' if bad_frame else 'camera_depth_optical_frame'
            ns=d.header.stamp.sec*10**9+d.header.stamp.nanosec-20000000;d.header.stamp.sec,d.header.stamp.nanosec=divmod(ns,10**9)
            cp.publish(c);dp.publish(d);spin();return c,d
        try:
            spin(.3);c,d=send()
            assert colors and depths and infos
            assert colors[-1].header==c.header
            assert depths[-1].header.stamp==d.header.stamp and depths[-1].header.frame_id==c.header.frame_id
            assert infos[-1].p[0]==2. and infos[-1].p[5]==2. and infos[-1].width==4
            np.testing.assert_allclose(node.bridge.imgmsg_to_cv2(depths[-1]),2.)
            count=len(depths)
            send(bad_frame=True);send(old=True);send(bad_size=True)
            assert len(depths)==count
            send();assert len(depths)>count
            print('PASS Gemini ROS explicit projection, metre units, original timestamps, native-frame/age/size rejection (synthetic)')
        finally:ex.shutdown();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
