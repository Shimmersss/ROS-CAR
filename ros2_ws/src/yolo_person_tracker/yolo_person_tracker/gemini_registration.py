"""ROS adapter for raw Gemini streams; no registration claim based on frame names."""
import copy
import json
from pathlib import Path
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from .registration import Registration
from .input_contract import stamp_seconds


class GeminiRegistration(Node):
    def __init__(self, **kwargs):
        super().__init__('gemini_registration', **kwargs)
        for name,value in dict(calibration_file='',expected_serial='',sync_slop_s=.06,max_age_s=.5,
                               color_frame='camera_color_optical_frame',depth_frame='camera_depth_optical_frame').items():
            self.declare_parameter(name,value)
        self.cfg={p:self.get_parameter(p).value for p in ('calibration_file','expected_serial','sync_slop_s','max_age_s','color_frame','depth_frame')}
        if not 0<self.cfg['sync_slop_s']<self.cfg['max_age_s']:
            raise ValueError('Invalid RGB-D timing bounds')
        calibration=json.loads(Path(self.cfg['calibration_file']).read_text())
        if not self.cfg['expected_serial'] or calibration['serial']!=self.cfg['expected_serial']:
            raise ValueError('Calibration serial must match explicitly selected Gemini')
        self.registration=Registration(calibration); self.bridge=CvBridge()
        self.color_pub=self.create_publisher(Image,'/camera/color/image_rect',2)
        self.depth_pub=self.create_publisher(Image,'/camera/aligned_depth_to_color/image_raw',2)
        self.info_pub=self.create_publisher(CameraInfo,'/camera/color/camera_info_rect',10)
        self.color=message_filters.Subscriber(self,Image,'/camera/color/image_raw',qos_profile=qos_profile_sensor_data)
        self.depth=message_filters.Subscriber(self,Image,'/camera/depth/image_raw',qos_profile=qos_profile_sensor_data)
        self.sync=message_filters.ApproximateTimeSynchronizer([self.color,self.depth],5,self.cfg['sync_slop_s'])
        self.sync.registerCallback(self.on_pair)
        self.last_error=''

    def on_pair(self,color,depth):
        try:
            now=self.get_clock().now().nanoseconds*1e-9
            for msg in (color,depth):
                stamp=stamp_seconds(msg.header.stamp)
                if stamp<=0 or not 0<=now-stamp<=self.cfg['max_age_s']:
                    raise ValueError('Expired, zero or future raw timestamp')
            if abs(stamp_seconds(color.header.stamp)-stamp_seconds(depth.header.stamp))>self.cfg['sync_slop_s']:
                raise ValueError('RGB-D skew exceeds tolerance')
            if color.header.frame_id!=self.cfg['color_frame'] or depth.header.frame_id!=self.cfg['depth_frame']:
                raise ValueError('Unexpected native optical frames')
            if depth.encoding not in ('16UC1','32FC1'):
                raise ValueError('Unsupported depth unit/encoding')
            rgb=self.bridge.imgmsg_to_cv2(color,'bgr8')
            z=self.bridge.imgmsg_to_cv2(depth,'passthrough').astype(np.float32)
            if depth.encoding=='16UC1': z*=.001
            rect,aligned=self.registration.apply(rgb,z)
            c=self.bridge.cv2_to_imgmsg(rect,'bgr8'); c.header=copy.deepcopy(color.header)
            d=self.bridge.cv2_to_imgmsg(aligned,'32FC1'); d.header=copy.deepcopy(depth.header)
            d.header.frame_id=color.header.frame_id  # Actual transformed geometry, original depth time.
            info=CameraInfo(); info.header=copy.deepcopy(color.header)
            info.width=color.width; info.height=color.height; info.distortion_model='plumb_bob'; info.d=[0.]*5
            k=self.registration.kc; info.k=list(map(float,k.reshape(-1))); info.r=list(map(float,np.eye(3).reshape(-1)))
            info.p=list(map(float,np.column_stack((k,np.zeros(3))).reshape(-1)))
            self.info_pub.publish(info); self.color_pub.publish(c); self.depth_pub.publish(d)
            self.last_error=''
        except Exception as exc:
            error=str(exc)
            if error!=self.last_error: self.get_logger().error(error)
            self.last_error=error


def main(args=None):
    rclpy.init(args=args); node=GeminiRegistration()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
