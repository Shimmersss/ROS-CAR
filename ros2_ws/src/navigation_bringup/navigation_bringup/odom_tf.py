"""Timestamped planar wheel odometry TF. Do not use alongside another odom TF owner."""
import math
import signal
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTF(Node):
    def __init__(self,**kwargs):
        super().__init__('wheel_odom_tf',**kwargs)
        for key,value in dict(odom_frame='odom',base_frame='base_footprint',timeout_s=.5).items():
            self.declare_parameter(key,value,ParameterDescriptor(read_only=True))
        self.broadcaster=TransformBroadcaster(self)
        self.create_subscription(Odometry,'/odom',self.on_odom,1)

    def on_odom(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        age=self.get_clock().now().nanoseconds*1e-9-stamp
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        if (self.count_publishers('/odom')!=1 or stamp<=0
                or not 0<=age<=self.get_parameter('timeout_s').value
                or msg.header.frame_id!=self.get_parameter('odom_frame').value
                or msg.child_frame_id!=self.get_parameter('base_frame').value
                or not all(math.isfinite(v) for v in (p.x,p.y,q.x,q.y,q.z,q.w))
                or abs(sum(v*v for v in (q.x,q.y,q.z,q.w))-1)>1e-3):
            return
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        tr=TransformStamped();tr.header=msg.header;tr.child_frame_id=msg.child_frame_id
        tr.transform.translation.x=p.x;tr.transform.translation.y=p.y
        # Vendor odometry's position.z contains heading. Never translate by it.
        tr.transform.translation.z=0.
        tr.transform.rotation.z=math.sin(yaw/2);tr.transform.rotation.w=math.cos(yaw/2)
        self.broadcaster.sendTransform(tr)


def main(args=None):
    def terminate(signum, frame):
        # ros2 launch may forward a second signal after the process group received
        # the first one. Do not interrupt final zero publication or ROS cleanup.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, terminate)
    signal.signal(signal.SIGTERM, terminate)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO);n=OdomTF()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
