"""Only this node publishes final velocity. Faults require explicit re-arming."""
import json
import math
import time
from dataclasses import fields
import signal
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException
from person_interfaces.msg import TargetState
from roscar_interfaces.srv import SetControlMode
from astra_body_adapter.follow_control import observation_is_fresh
from .safety import SafetyConfig, clearance


def seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


class MotionGuard(Node):
    def __init__(self, **kwargs):
        super().__init__('motion_guard', **kwargs)
        defaults = dict(command_mode='FOLLOW', motion_enabled=False, geometry_confirmed=False, stopping_model_confirmed=False, mount_calibrated=False,
                        base_frame='base_link', scan_frame='laser', expected_source='astra',
                        target_frame='astra_depth_optical_frame', request_topic='/control/cmd_vel_request',
                        scan_topic='/scan', target_topic='/perception/target_state')
        defaults.update(vars(SafetyConfig()))
        for key, value in defaults.items():
            self.declare_parameter(key, value, ParameterDescriptor(read_only=True))
        self.cfg = {key:self.get_parameter(key).value for key in defaults}
        self.safety = SafetyConfig(**{f.name:self.cfg[f.name] for f in fields(SafetyConfig)})
        if self.cfg['expected_source'] not in ('astra','yolo','red_object'):
            raise ValueError('Select a real supported target source')
        if not all(self.cfg[k] for k in ('base_frame','scan_frame','target_frame')):
            raise ValueError('Explicit nonempty frames required')
        if self.cfg['base_frame'] == self.cfg['scan_frame']:
            raise ValueError('Use distinct laser/base frames and a calibrated mount')
        if self.cfg['request_topic'] in ('/chassis/cmd_vel', '/cmd_vel'):
            raise ValueError('Follow request topic must be distinct from external and output topics')
        self.mode = 'STANDBY' if self.cfg['motion_enabled'] else 'PERCEPTION_ONLY'
        self.command_mode = self.cfg['command_mode']
        if self.command_mode not in ('IDLE', 'EXTERNAL', 'FOLLOW'):
            raise ValueError('Invalid command_mode')
        self.mode_since = self.get_clock().now().nanoseconds*1e-9
        self.reason = 'not_armed'
        self.last_fault = ''
        self.request = self.scan = self.target = None
        self.received = {}
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer,self)
        self.pub = self.create_publisher(Twist,'/cmd_vel',1)
        self.status_pub = self.create_publisher(String,'/control/state',10)
        self.create_subscription(TwistStamped,self.cfg['request_topic'],self.on_request,1)
        self.create_subscription(TwistStamped,'/chassis/cmd_vel',self.on_external,1)
        self.create_service(SetControlMode,'/control/set_mode',self.set_mode)
        self.create_subscription(LaserScan,self.cfg['scan_topic'],self.on_scan,qos_profile_sensor_data)
        self.create_subscription(TargetState,self.cfg['target_topic'],self.on_target,1)
        self.create_service(Trigger,'/control/arm',self.arm)
        self.create_service(Trigger,'/control/stop',self.stop_service)
        self.create_service(Trigger,'/control/disarm',self.stop_service)
        # Watchdogs keep running if /clock pauses.
        self.timer = self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def set_mode(self, request, response):
        if request.mode not in ('IDLE', 'EXTERNAL', 'FOLLOW'):
            response.success = False
            response.message = 'Expected IDLE, EXTERNAL or FOLLOW'
            return response
        self.stop_service(None, Trigger.Response())
        self.command_mode = request.mode
        self.mode_since = self.get_clock().now().nanoseconds*1e-9
        self.request = None
        self.received.pop('request', None)
        response.success = True
        response.message = 'Stopped and disarmed; send fresh request then explicitly arm'
        return response

    def accept_request(self, msg, source):
        if self.command_mode == source and seconds(msg.header.stamp) > self.mode_since:
            self.request=msg; self.received['request']=time.monotonic()

    def on_request(self,msg):
        self.accept_request(msg, 'FOLLOW')

    def on_external(self,msg):
        self.accept_request(msg, 'EXTERNAL')

    def on_scan(self,msg):
        self.scan=msg; self.received['scan']=time.monotonic()

    def on_target(self,msg):
        self.target=msg; self.received['target']=time.monotonic()

    def check(self):
        if self.command_mode == 'IDLE':
            return False,'idle',0.,0.
        if self.command_mode == 'EXTERNAL' and self.cfg['base_frame'] != 'base_link':
            return False,'external_requires_base_link',0.,0.
        if not all(self.cfg[k] for k in ('geometry_confirmed','mount_calibrated','stopping_model_confirmed')):
            return False,'unconfirmed_geometry_or_mount',0.,0.
        topics = [self.cfg['scan_topic'], '/cmd_vel',
                  '/chassis/cmd_vel' if self.command_mode == 'EXTERNAL' else self.cfg['request_topic']]
        if self.command_mode == 'FOLLOW':
            topics.append(self.cfg['target_topic'])
        for topic in topics:
            if self.count_publishers(topic) != 1:
                return False,'missing_or_multiple_publishers:'+topic,0.,0.
        now_ros=self.get_clock().now().nanoseconds*1e-9
        inputs = [
            ('request',self.request,self.safety.request_timeout_s),
            ('scan',self.scan,self.safety.scan_timeout_s),
        ]
        if self.command_mode == 'FOLLOW':
            inputs.append(('target',self.target,self.safety.target_timeout_s))
        for key, msg, timeout in inputs:
            if msg is None or time.monotonic()-self.received.get(key,-math.inf)>timeout:
                return False,key+'_timeout',0.,0.
            stamp=seconds(msg.header.stamp)
            if stamp<=0 or not 0 <= now_ros-stamp <= timeout:
                return False,key+'_timestamp',0.,0.
        target=self.target
        if self.command_mode == 'FOLLOW' and (target.is_simulated or not target.position_valid or target.status!=TargetState.TRACKING
                or target.header.frame_id != self.cfg['target_frame']
                or not all(math.isfinite(x) for x in (target.position.x,target.position.y,target.position.z,
                                                     target.horizontal_distance_m,target.bearing_rad))
                or not observation_is_fresh(now_ros,seconds(target.header.stamp),
                    seconds(target.observation_stamp),target.measurement_age_s,
                    self.safety.target_timeout_s,target.source,self.cfg['expected_source'])):
            return False,'target_invalid',0.,0.
        t=self.request.twist
        values=(t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z)
        if (self.request.header.frame_id != self.cfg['base_frame'] or not all(map(math.isfinite,values))
                or any(x!=0 for x in (t.linear.y,t.linear.z,t.angular.x,t.angular.y))
                or not 0 <= t.linear.x <= self.safety.max_linear_mps
                or abs(t.angular.z)>self.safety.max_angular_rps):
            return False,'invalid_velocity_request',0.,0.
        if self.scan.header.frame_id != self.cfg['scan_frame']:
            return False,'unexpected_scan_frame',0.,0.
        try:
            tr=self.buffer.lookup_transform(self.cfg['base_frame'],self.cfg['scan_frame'],
                                             Time.from_msg(self.scan.header.stamp)).transform
            q=tr.rotation; xyz=tr.translation
            if (not all(math.isfinite(v) for v in (q.x,q.y,q.z,q.w,xyz.x,xyz.y,xyz.z))
                    or abs(q.x)>1e-4 or abs(q.y)>1e-4
                    or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>1e-4):
                return False,'nonplanar_or_invalid_tf',0.,0.
            yaw=2*math.atan2(q.z,q.w)
            clear,reason=clearance(self.scan,(xyz.x,xyz.y,yaw),t.linear.x,t.angular.z,self.safety)
            return clear,reason,t.linear.x,t.angular.z
        except TransformException:
            return False,'tf_unavailable_at_scan_time',0.,0.

    def arm(self,request,response):
        del request
        if not self.cfg['motion_enabled']:
            response.success=False; response.message='motion_enabled=false; perception only'
            return response
        ready,reason,_,_=self.check()
        response.success=ready; response.message=reason
        if ready:
            self.mode='ARMED'; self.reason='explicitly_armed'
        else:
            self.reason=reason
        return response

    def stop_service(self,request,response):
        del request
        self.mode='STANDBY' if self.cfg['motion_enabled'] else 'PERCEPTION_ONLY'
        self.reason='operator_stop'
        self.publish(0.,0.)
        response.success=True; response.message='Stopped; explicit arm required to resume'
        return response

    def publish(self,v,w):
        msg=Twist(); msg.linear.x=float(v); msg.angular.z=float(w); self.pub.publish(msg)

    def tick(self):
        ready,reason,v,w=self.check()
        if self.mode=='ARMED' and not ready:
            self.mode='FAULT'; self.last_fault=reason
        self.reason=reason
        self.publish(v if self.mode=='ARMED' and ready else 0.,
                     w if self.mode=='ARMED' and ready else 0.)
        self.status_pub.publish(String(data=json.dumps(dict(mode=self.mode,command_mode=self.command_mode,ready=ready,
             reason=reason,last_fault=self.last_fault,motion_enabled=self.cfg['motion_enabled']))))

    def destroy_node(self):
        self.timer.cancel(); self.publish(0.,0.)
        return super().destroy_node()


def main(args=None):
    def terminate(signum, frame):
        # ros2 launch may forward a second signal after the process group received
        # the first one. Do not interrupt final zero publication or ROS cleanup.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, terminate)
    signal.signal(signal.SIGTERM, terminate)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO); node=MotionGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
