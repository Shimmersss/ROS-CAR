"""Only this node publishes final velocity. Faults require explicit re-arming.

Follow-only options for an unattended follow entry (all default off):
- command_mode:=FOLLOW selects FOLLOW at startup (a persisted ESTOP still keeps it IDLE).
- auto_arm arms from DISARMED in FOLLOW once every check passes, like the Route A follower.
  STOP returns to IDLE, so a stop is never undone by auto_arm.
- target_grace_s (<= 1 s): a briefly invalid or late target keeps the vehicle armed while the last
  command decays to zero; a longer gap is a FAULT. Zero output when radar_required.
- target_auto_recover: a FAULT caused only by the target re-arms once every check passes again.
ESTOP, emergency-channel and every other fault stay latched and need the explicit reset.
"""
import json
import math
import os
from pathlib import Path
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
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException
from person_interfaces.msg import TargetState
from roscar_interfaces.srv import SetControlMode, AuthorizeMotion
from execution_mode_interfaces.srv import TaskHold
from astra_body_adapter.follow_control import observation_is_fresh
from .safety import SafetyConfig, clearance
from .authority import MotionAuthority, AuthorityError
from .task_hold import StationaryLease

TARGET_REASONS = ('target_timeout', 'target_timestamp', 'target_invalid')


def seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


class MotionGuard(Node):
    def __init__(self, **kwargs):
        super().__init__('motion_guard', **kwargs)
        defaults = dict(command_mode='IDLE', motion_enabled=False, geometry_confirmed=False, stopping_model_confirmed=False, mount_calibrated=False,
                        base_frame='base_link', scan_frame='laser', expected_source='astra',
                        target_frame='astra_depth_optical_frame', request_topic='/control/cmd_vel_request',
                        scan_topic='/scan', target_topic='/perception/target_state',
                        radar_required=False, auto_arm=False, target_grace_s=0., target_auto_recover=False,
                        emergency_kws_required=True,
                        emergency_health_timeout_s=0.5, allow_legacy_arm=False, estop_latch_path=str(Path(
                            os.environ.get('XDG_STATE_HOME', str(Path.home() / '.local/state'))) / 'roscar/estop.json'))
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
        grace = self.cfg['target_grace_s']
        if isinstance(grace, bool) or not isinstance(grace, (int, float)) or not 0 <= grace <= 1.:
            raise ValueError('target_grace_s must be within [0, 1] seconds')
        if self.cfg['command_mode'] not in ('IDLE', 'FOLLOW'):
            raise ValueError('Startup command_mode must be IDLE or FOLLOW')
        follow_only = ('auto_arm', 'target_auto_recover')
        if any(self.cfg[k] for k in follow_only) and (self.cfg['command_mode'] != 'FOLLOW' or not self.cfg['motion_enabled']):
            raise ValueError('auto_arm/target_auto_recover require command_mode=FOLLOW and motion_enabled')
        if not math.isfinite(self.cfg['emergency_health_timeout_s']) or not 0.1 <= self.cfg['emergency_health_timeout_s'] <= 0.5:
            raise ValueError('Emergency health timeout must be 0.1..0.5 seconds')
        self.authority = MotionAuthority(self.cfg['estop_latch_path'])
        if self.cfg['command_mode'] == 'FOLLOW':
            try:
                self.authority.select_mode('FOLLOW')
            except AuthorityError as exc:  # persisted ESTOP: stay IDLE until an explicit reset
                self.get_logger().warning('Startup FOLLOW not selected: '+str(exc))
        self.last_ok_at = -math.inf  # last tick that was armed and ready (monotonic)
        self.last_cmd = (0., 0.)
        self._emergency_ok = False
        self._emergency_at = -math.inf
        self.mode_since = self.get_clock().now().nanoseconds*1e-9
        self.reason = 'not_armed'
        self.last_fault = ''
        self.task_hold = StationaryLease()
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
        self.create_service(AuthorizeMotion,'/control/authorize',self.authorize)
        self.create_service(Trigger,'/control/stop',self.stop_service)
        self.create_service(Trigger,'/control/disarm',self.stop_service)
        self.create_service(TaskHold,'/control/task_hold',self.on_task_hold)
        self.create_service(Trigger,'/control/estop',self.estop_service)
        self.create_service(Trigger,'/control/reset_estop',self.reset_estop)
        self.create_service(Trigger,'/control/reset_fault',self.reset_fault)
        self.create_subscription(Bool,'/control/emergency_stop',self.on_estop,10)
        self.create_subscription(Bool,'/voice/emergency_healthy',self.on_emergency_health,10)
        # Watchdogs keep running if /clock pauses.
        self.timer = self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    @property
    def command_mode(self):
        return self.authority.command_mode

    @property
    def mode(self):
        if self.authority.armed:
            return 'ARMED'  # retained /control/state compatibility field
        if self.authority.state == 'DISARMED':
            return 'STANDBY' if self.cfg['motion_enabled'] else 'PERCEPTION_ONLY'
        return self.authority.state

    def clear_requests(self):
        self.request = None
        self.received.pop('request', None)
        self.mode_since = self.get_clock().now().nanoseconds * 1e-9

    def set_mode(self, request, response):
        self.publish(0., 0.)
        if self.task_hold.active(time.monotonic()) and request.mode != 'IDLE':
            response.success, response.message = False, 'Execution mode holds the vehicle stationary'
            return response
        try:
            self.authority.select_mode(request.mode)
            self.clear_requests()
            response.success, response.message = True, 'Disarmed; fresh zero request and explicit arm required'
            response.authority_epoch = self.authority.epoch
        except AuthorityError as exc:
            if self.authority.armed:
                self.authority.stop()
                self.clear_requests()
            response.success, response.message = False, str(exc)
        return response

    def emergency_healthy(self):
        return (not self.cfg['emergency_kws_required'] or
                (self._emergency_ok and time.monotonic() - self._emergency_at <=
                 self.cfg['emergency_health_timeout_s']))

    def on_emergency_health(self, message):
        self._emergency_ok = bool(message.data)
        self._emergency_at = time.monotonic()
        if not self._emergency_ok and self.authority.armed:
            self.authority.fault('emergency_channel_unhealthy')
            self.last_fault = 'emergency_channel_unhealthy'
            self.publish(0., 0.)

    def on_estop(self, message):
        if message.data:
            self.estop_service(None, Trigger.Response())

    def estop_service(self, request, response):
        del request
        self.publish(0., 0.)
        try:
            self.authority.estop()
            response.success, response.message = True, 'ESTOP latched; explicit reset then re-arm required'
        except AuthorityError as exc:
            response.success, response.message = False, str(exc)
        self.clear_requests()
        return response

    def reset_estop(self, request, response):
        del request
        self.publish(0., 0.)
        try:
            self.authority.reset_estop(self.emergency_healthy())
            self.clear_requests()
            response.success, response.message = True, 'ESTOP cleared; DISARMED, select mode and re-arm'
        except AuthorityError as exc:
            response.success, response.message = False, str(exc)
        return response

    def reset_fault(self, request, response):
        del request
        self.publish(0., 0.)
        try:
            ready = self.emergency_healthy() if self.command_mode == 'IDLE' else self.check()[0]
            self.authority.reset_fault(ready)
            self.clear_requests()
            response.success, response.message = True, 'FAULT cleared; DISARMED'
        except (AuthorityError, ValueError) as exc:
            response.success, response.message = False, str(exc)
        return response

    def accept_request(self, msg, source):
        if (self.authority.state != 'ESTOP' and self.command_mode == source
                and seconds(msg.header.stamp) > self.mode_since):
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
        if self.task_hold.active(time.monotonic()):
            return False, 'task_hold', 0., 0.
        if not self.emergency_healthy():
            return False, 'emergency_channel_unhealthy', 0., 0.
        if self.command_mode == 'IDLE':
            return False,'idle',0.,0.
        if self.command_mode == 'EXTERNAL' and self.cfg['base_frame'] != 'base_link':
            return False,'external_requires_base_link',0.,0.
        if not all(self.cfg[k] for k in ('geometry_confirmed','mount_calibrated','stopping_model_confirmed')):
            return False,'unconfirmed_geometry_or_mount',0.,0.
        topics = ['/cmd_vel',
                  '/chassis/cmd_vel' if self.command_mode == 'EXTERNAL' else self.cfg['request_topic']]
        if self.cfg['radar_required']:
            topics.insert(0, self.cfg['scan_topic'])
        if self.command_mode == 'FOLLOW':
            topics.append(self.cfg['target_topic'])
        for topic in topics:
            if self.count_publishers(topic) != 1:
                return False,'missing_or_multiple_publishers:'+topic,0.,0.
        now_ros=self.get_clock().now().nanoseconds*1e-9
        inputs = [('request',self.request,self.safety.request_timeout_s)]
        if self.cfg['radar_required']:
            inputs.append(('scan',self.scan,self.safety.scan_timeout_s))
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
                or not -self.safety.max_reverse_mps <= t.linear.x <= self.safety.max_linear_mps
                or abs(t.angular.z)>self.safety.max_angular_rps):
            return False,'invalid_velocity_request',0.,0.
        if not self.cfg['radar_required']:
            return True,'ready_without_radar',t.linear.x,t.angular.z
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

    def authorize(self, request, response):
        if (request.authority_epoch != self.authority.epoch or request.mode != self.command_mode):
            response.success, response.message = False, 'Stale authorization epoch/mode'
            return response
        return self._arm(response)

    def arm(self, request, response):
        del request
        if not self.cfg['allow_legacy_arm']:
            response.success, response.message = False, 'Use epoch-bound /control/authorize'
            return response
        return self._arm(response)

    def _arm(self, response):
        if self.task_hold.active(time.monotonic()):
            response.success, response.message = False, 'Execution mode holds the vehicle stationary'
            self.publish(0., 0.)
            return response
        try:
            ready, reason, _, _ = self.check()
            # Arming must never launch a buffered nonzero command.
            zero = self.request is not None and all(v == 0 for v in (
                self.request.twist.linear.x, self.request.twist.angular.z))
            self.authority.arm(ready and zero, self.cfg['motion_enabled'])
            response.success, response.message = True, 'Explicitly armed at zero velocity'
        except (AuthorityError, ValueError) as exc:
            self.publish(0., 0.)
            response.success, response.message = False, str(exc)
        return response

    def stop_service(self, request, response):
        del request
        self.authority.stop()  # STOP cannot clear an ESTOP or FAULT latch.
        self.clear_requests()
        self.reason = self.authority.reason
        self.publish(0., 0.)
        response.success, response.message = True, 'Stopped and disarmed'
        return response

    def publish(self,v,w):
        msg=Twist(); msg.linear.x=float(v); msg.angular.z=float(w); self.pub.publish(msg)

    def on_task_hold(self, request, response):
        now = time.monotonic()
        try:
            if request.operation == 'ACQUIRE':
                if self.authority.state in ('ESTOP', 'FAULT') or not self.emergency_healthy():
                    raise ValueError('Reset fault/ESTOP and restore emergency channel before entering a mode')
                response.token = self.task_hold.acquire(request.owner, request.session_id, request.lease_sec, now)
                self.stop_service(None, Trigger.Response())
            elif request.operation == 'RENEW':
                self.task_hold.renew(request.token, request.lease_sec, now)
                response.token = request.token
            elif request.operation == 'RELEASE':
                self.task_hold.release(request.token)
                self.stop_service(None, Trigger.Response())
            else:
                raise ValueError('Expected ACQUIRE, RENEW or RELEASE')
            response.success, response.detail = True, 'Vehicle remains IDLE; explicit new authorization required'
        except ValueError as exc:
            response.success, response.detail = False, str(exc)
        return response

    def tick(self):
        if self.task_hold.expire(time.monotonic()):
            self.stop_service(None, Trigger.Response())
        try:
            ready, reason, v, w = self.check()
        except Exception as exc:
            ready, reason, v, w = False, 'guard_exception:' + type(exc).__name__, 0., 0.
        now = time.monotonic()
        follow = self.command_mode == 'FOLLOW' and self.cfg['motion_enabled']
        if follow and ready and self.cfg['auto_arm'] and self.authority.state == 'DISARMED':
            self.authority.arm(True, True)
            self.reason, self.last_ok_at, self.last_cmd = 'auto_armed', -math.inf, (0., 0.)
        if (follow and ready and self.cfg['target_auto_recover'] and self.authority.state == 'FAULT'
                and self.authority.reason in TARGET_REASONS):
            # Only a target loss recovers by itself; gate on the authority's current fault, not a past one.
            # The fresh target passed every check this tick.
            self.authority.recover_follow()
            self.last_ok_at, self.last_cmd = -math.inf, (0., 0.)
            self.get_logger().info('Target valid again after '+self.last_fault+'; re-armed')
        grace = self.cfg['target_grace_s']
        in_grace = (self.authority.armed and not ready and self.command_mode == 'FOLLOW' and grace > 0
                    and reason in TARGET_REASONS and 0 <= now-self.last_ok_at <= grace)
        if in_grace:
            # Brief target gap: stay armed and decay the last command; never extend it.
            k = 0. if self.cfg['radar_required'] else max(0., 1.-(now-self.last_ok_at)/grace)
            v, w = self.last_cmd[0]*k, self.last_cmd[1]*k
            reason = 'target_grace:'+reason
        elif self.authority.armed and not ready:
            self.authority.fault(reason)
            self.last_fault = reason
        if self.authority.armed and ready:
            self.last_ok_at, self.last_cmd = now, (v, w)
        self.reason = reason
        allowed = self.authority.armed and (ready or in_grace) and self.cfg['motion_enabled']
        self.publish(v if allowed else 0., w if allowed else 0.)
        self.status_pub.publish(String(data=json.dumps(dict(
            mode=self.mode, authority_state=self.authority.state,
            authority_epoch=self.authority.epoch, command_mode=self.command_mode,
            ready=ready, reason=reason, last_fault=self.last_fault,
            motion_enabled=self.cfg['motion_enabled'], emergency_healthy=self.emergency_healthy(),
            task_hold=dict(active=self.task_hold.active(time.monotonic()), owner=self.task_hold.owner,
                           session_id=self.task_hold.session_id)))))

    def destroy_node(self):
        self.timer.cancel()
        if rclpy.ok(context=self.context):
            self.publish(0.,0.)
        return super().destroy_node()


def main(args=None):
    stopping = []

    def terminate(signum, frame):
        # ros2 launch may forward a second signal after the process group received
        # the first one. Do not interrupt final zero publication or ROS cleanup.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        stopping.append(signum)
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, terminate)
    signal.signal(signal.SIGTERM, terminate)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO); node=MotionGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        # A stop signal landing inside rclpy's take_message surfaces as RuntimeError, not
        # KeyboardInterrupt. Treat it as the requested stop; any other RuntimeError is real.
        if not stopping:
            raise
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
