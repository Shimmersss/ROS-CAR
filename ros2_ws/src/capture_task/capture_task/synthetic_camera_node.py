"""Synthetic chessboard for explicit isolated software tests, never a camera fallback."""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data


class SyntheticCamera(Node):
    def __init__(self,**kwargs):
        super().__init__('capture_synthetic_camera',**kwargs)
        self.declare_parameter('test_only',False)
        if not self.get_parameter('test_only').value:
            raise ValueError('synthetic_camera_requires_test_only_true')
        y,x=np.indices((1080,1920))
        gray=np.where((x//24+y//24)%2,200,60).astype(np.uint8)
        self.image=np.repeat(gray[:,:,None],3,axis=2);self.cv=CvBridge()
        self.publisher=self.create_publisher(Image,'/camera/color/image_raw',qos_profile_sensor_data)
        self.timer=self.create_timer(.0667,self.publish)

    def publish(self):
        msg=self.cv.cv2_to_imgmsg(self.image,encoding='rgb8')
        msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='synthetic_rgb_optical_frame'
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args);node=SyntheticCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
