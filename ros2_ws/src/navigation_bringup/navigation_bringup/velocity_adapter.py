"""Humble Twist -> stamped request. Nav2 never owns the final cmd_vel topic."""
import copy
import json
import time
import signal
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist,TwistStamped
from std_msgs.msg import String


class VelocityAdapter(Node):
    def __init__(self,**kwargs):
        super().__init__('nav_velocity_adapter',**kwargs)
        self.declare_parameter('base_frame','base_footprint',ParameterDescriptor(read_only=True))
        self.command=None;self.command_at=-float('inf');self.status=None;self.status_at=-float('inf')
        self.allowed_since=None
        self.pub=self.create_publisher(TwistStamped,'/control/cmd_vel_request',1)
        self.create_subscription(Twist,'/navigation/cmd_vel_raw',self.on_command,1)
        self.create_subscription(String,'/navigation/state',self.on_state,1)
        self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_command(self,msg):
        m=TwistStamped();m.header.stamp=self.get_clock().now().to_msg()
        m.header.frame_id=self.get_parameter('base_frame').value;m.twist=copy.deepcopy(msg)
        self.command=m;self.command_at=time.monotonic()

    def on_state(self,msg):
        try:
            value=json.loads(msg.data)
            if (not isinstance(value,dict) or not isinstance(value.get('allow_motion'),bool)
                    or not isinstance(value.get('fault'),bool)
                    or type(value.get('stamp_ns')) is not int):
                raise ValueError('Malformed navigation state')
            stamp=int(value['stamp_ns']);age=(self.get_clock().now().nanoseconds-stamp)*1e-9
            if stamp<=0 or not 0<=age<=.3:raise ValueError('Stale navigation state')
        except (ValueError,TypeError,KeyError,OverflowError):
            self.status=None;return
        self.status=value;self.status_at=time.monotonic()
        if not value['allow_motion']:
            self.command=None;self.allowed_since=None
        elif self.allowed_since is None:
            self.allowed_since=time.monotonic();self.command=None

    def tick(self):
        now=time.monotonic()
        result=TwistStamped();result.header.frame_id=self.get_parameter('base_frame').value
        status=self.status
        healthy=(status is not None and now-self.status_at<=.3
                 and self.count_publishers('/navigation/state')==1)
        if healthy and not status.get('fault',False):
            if not status['allow_motion']:
                result.header.stamp=self.get_clock().now().to_msg()
            elif (self.command is not None and now-self.command_at<=.2
                  and self.count_publishers('/navigation/cmd_vel_raw')==1):
                result=copy.deepcopy(self.command)  # Never refresh an old command's stamp.
            elif self.command is None and self.allowed_since is not None and now-self.allowed_since<2.:
                result.header.stamp=self.get_clock().now().to_msg()  # Initial planning grace, zero only.
        # Missing/fault status or expired command -> zero with unknown stamp: guard latches FAULT.
        self.pub.publish(result)


def main(args=None):
    def terminate(signum, frame):
        # ros2 launch may forward a second signal after the process group received
        # the first one. Do not interrupt final zero publication or ROS cleanup.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, terminate)
    signal.signal(signal.SIGTERM, terminate)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO);n=VelocityAdapter()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
