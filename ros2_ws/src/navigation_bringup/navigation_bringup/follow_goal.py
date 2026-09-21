"""Convert a fresh selected person to serialized, cancelable Nav2 standoff goals."""
import json
import math
import time
import signal
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.time import Time
from rclpy.action import ActionClient
from rcl_interfaces.msg import ParameterDescriptor
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped,PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer,TransformListener,TransformException
from tf2_geometry_msgs import do_transform_point
from person_interfaces.msg import TargetState
from astra_body_adapter.follow_control import observation_is_fresh
from .geometry import standoff


class FollowGoal(Node):
    def __init__(self,**kwargs):
        super().__init__('navigation_follow_goal',**kwargs)
        defaults=dict(camera_calibrated=False,odometry_confirmed=False,allow_unstamped_astra=False,
                      expected_source='astra',target_frame='astra_depth_optical_frame',
                      base_frame='base_footprint',map_frame='map',odom_frame='odom',
                      follow_distance_m=1.,target_timeout_s=.5,odom_timeout_s=.5,
                      goal_update_s=1.,goal_shift_m=.3,tf_wait_timeout_s=.15)
        for k,v in defaults.items():self.declare_parameter(k,v,ParameterDescriptor(read_only=True))
        self.cfg={k:self.get_parameter(k).value for k in defaults}
        for k in ('follow_distance_m','target_timeout_s','odom_timeout_s','goal_update_s','goal_shift_m','tf_wait_timeout_s'):
            if not math.isfinite(self.cfg[k]) or self.cfg[k]<=0:raise ValueError(k+' must be positive and finite')
        if self.cfg['tf_wait_timeout_s']>self.cfg['target_timeout_s']:
            raise ValueError('TF wait must not exceed target freshness timeout')
        if self.cfg['expected_source'] not in ('astra','yolo','red_object'):raise ValueError('Invalid target source')
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.client=ActionClient(self,NavigateToPose,'navigate_to_pose')
        self.pub=self.create_publisher(String,'/navigation/state',10)
        self.goal_pub=self.create_publisher(PoseStamped,'/navigation/follow_goal',10)
        self.target=self.odom=self.guard=None
        self.target_at=self.odom_at=self.guard_at=-math.inf
        self.tf_wait_since=None
        self.active=False;self.fault=False;self.reason='not_started';self.locked_id=None
        self.goal_handle=None;self.pending=None;self.result_future=None;self.canceling=False
        self.last_goal=None;self.sent_at=-math.inf;self.distance_remaining=None
        self.create_subscription(TargetState,'/perception/target_state',self.on_target,1)
        self.create_subscription(Odometry,'/odom',self.on_odom,1)
        self.create_subscription(String,'/control/state',self.on_guard,1)
        self.create_service(Trigger,'/navigation/start_follow',self.start)
        self.create_service(Trigger,'/navigation/stop_follow',self.stop)
        self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_target(self,m):self.target=m;self.target_at=time.monotonic()
    def on_odom(self,m):self.odom=m;self.odom_at=time.monotonic()
    def on_guard(self,m):
        try:
            data=json.loads(m.data)
            if not isinstance(data,dict):
                self.guard=None;return
        except ValueError:
            self.guard=None;return
        self.guard=data;self.guard_at=time.monotonic()

    def guard_armed(self):
        return (self.guard is not None and time.monotonic()-self.guard_at<=.3
                and self.guard.get('mode')=='ARMED' and self.count_publishers('/control/state')==1)

    def ready_goal(self):
        c=self.cfg;now=self.get_clock().now().nanoseconds*1e-9
        if not c['camera_calibrated'] or not c['odometry_confirmed']:raise ValueError('calibration_unconfirmed')
        if self.count_publishers('/perception/target_state')!=1 or self.count_publishers('/odom')!=1:
            raise ValueError('missing_or_multiple_inputs')
        m=self.target;o=self.odom
        if (m is None or time.monotonic()-self.target_at>c['target_timeout_s']
                or m.is_simulated or not m.position_valid or m.status!=TargetState.TRACKING
                or not m.target_id or m.header.frame_id!=c['target_frame']):raise ValueError('target_unavailable')
        sec=lambda t:t.sec+t.nanosec*1e-9
        if not observation_is_fresh(now,sec(m.header.stamp),sec(m.observation_stamp),m.measurement_age_s,
                                    c['target_timeout_s'],m.source,c['expected_source']):raise ValueError('target_stale')
        observed=m.observation_stamp
        if sec(observed)==0:
            if not c['allow_unstamped_astra']:raise ValueError('source_observation_time_unknown')
            observed=m.header.stamp  # Explicit legacy approximation, never a silent fallback.
        if (o is None or time.monotonic()-self.odom_at>c['odom_timeout_s']
                or sec(o.header.stamp)<=0 or not 0<=now-sec(o.header.stamp)<=c['odom_timeout_s']
                or o.header.frame_id!=c['odom_frame'] or o.child_frame_id!=c['base_frame']):
            raise ValueError('odometry_stale_or_frame_mismatch')
        if not self.client.server_is_ready():raise ValueError('nav2_action_unavailable')
        stamp=Time.from_msg(observed)
        person_transform=self.buffer.lookup_transform(c['map_frame'],m.header.frame_id,stamp)
        robot_transform=self.buffer.lookup_transform(c['map_frame'],c['base_frame'],stamp)
        # Current localization must also be present; old observation TF alone is insufficient.
        latest=self.buffer.lookup_transform(c['map_frame'],c['base_frame'],Time())
        latest_age=now-sec(latest.header.stamp)
        if sec(latest.header.stamp)<=0 or not 0<=latest_age<=c['odom_timeout_s']:
            raise ValueError('localization_tf_stale')
        p=PointStamped();p.header.frame_id=m.header.frame_id;p.header.stamp=observed;p.point=m.position
        pt=do_transform_point(p,person_transform).point
        r=robot_transform.transform.translation
        result=standoff((r.x,r.y),(pt.x,pt.y),c['follow_distance_m'])
        if result is None:return None
        x,y,yaw=result
        goal=PoseStamped();goal.header.frame_id=c['map_frame'];goal.header.stamp=self.get_clock().now().to_msg()
        goal.pose.position.x=x;goal.pose.position.y=y
        goal.pose.orientation.z=math.sin(yaw/2);goal.pose.orientation.w=math.cos(yaw/2)
        return goal

    def cancel(self):
        if self.goal_handle is not None and not self.canceling:
            self.canceling=True
            self.goal_handle.cancel_goal_async()  # Wait for terminal result before sending another goal.

    def fail(self,reason):
        self.active=False;self.fault=True;self.reason=reason;self.cancel()

    def start(self,request,response):
        del request
        try:
            self.ready_goal()
            if not self.guard_armed():raise ValueError('arm_motion_guard_first')
            if self.pending is not None or self.goal_handle is not None:raise ValueError('previous_goal_still_finishing')
        except (ValueError,TransformException) as exc:
            response.success=False;response.message=str(exc);return response
        self.active=True;self.fault=False;self.reason='started';self.last_goal=None
        self.locked_id=self.target.target_id
        response.success=True;response.message='Following enabled; guarded Nav2 goals only';return response

    def stop(self,request,response):
        del request
        self.active=False;self.fault=False;self.reason='operator_stop';self.cancel()
        response.success=True;response.message='Navigation stopped; restart requires explicit service';return response

    def accepted(self,future):
        self.pending=None
        try:self.goal_handle=future.result()
        except Exception as exc:self.fail('goal_send_failed:'+str(exc));return
        if not self.goal_handle.accepted:
            self.goal_handle=None;self.fail('goal_rejected');return
        self.result_future=self.goal_handle.get_result_async();self.result_future.add_done_callback(self.finished)
        if not self.active:self.cancel()

    def finished(self,future):
        canceled=self.canceling
        self.goal_handle=None;self.result_future=None;self.canceling=False
        try:status=future.result().status
        except Exception as exc:self.fail('goal_result_failed:'+str(exc));return
        if not canceled and self.active and status!=GoalStatus.STATUS_SUCCEEDED:
            self.fail('nav2_goal_failed:'+str(status))

    def feedback(self,message):
        value=float(message.feedback.distance_remaining)
        self.distance_remaining=value if math.isfinite(value) else None

    def tick(self):
        goal=None;ready=False
        try:
            goal=self.ready_goal();ready=True;self.tf_wait_since=None
        except TransformException as exc:
            # RGB-D may arrive before odom TF for that observation. Zero output while
            # waiting, never substitute a latest-time transform for the observation.
            if self.tf_wait_since is None:self.tf_wait_since=time.monotonic()
            if self.active and time.monotonic()-self.tf_wait_since>=self.cfg['tf_wait_timeout_s']:
                self.fail('tf_unavailable:'+str(exc))
            elif not self.fault:self.reason='waiting_for_observation_tf'
        except ValueError as exc:
            self.tf_wait_since=None
            if self.active:self.fail(str(exc))
            elif not self.fault:self.reason=str(exc)
        if self.active and self.target is not None and self.target.target_id!=self.locked_id:
            self.fail('selected_target_changed')
        if self.active and not self.guard_armed():self.fail('motion_guard_not_armed')
        if self.active and ready:
            if goal is None:
                self.reason='within_follow_distance';self.last_goal=None;self.cancel()
            else:
                self.goal_pub.publish(goal)
                changed=(self.last_goal is None or math.hypot(
                    goal.pose.position.x-self.last_goal.pose.position.x,
                    goal.pose.position.y-self.last_goal.pose.position.y)>=self.cfg['goal_shift_m'])
                if changed and time.monotonic()-self.sent_at>=self.cfg['goal_update_s']:
                    if self.goal_handle is not None:self.reason='updating_goal';self.cancel()
                    elif self.pending is None:
                        request=NavigateToPose.Goal();request.pose=goal
                        self.last_goal=goal;self.sent_at=time.monotonic();self.reason='navigating'
                        self.pending=self.client.send_goal_async(request,feedback_callback=self.feedback)
                        self.pending.add_done_callback(self.accepted)
        allow=(self.active and ready and goal is not None and self.goal_handle is not None
               and not self.canceling and self.guard_armed())
        self.pub.publish(String(data=json.dumps(dict(stamp_ns=self.get_clock().now().nanoseconds,
            active=self.active,fault=self.fault,ready=ready,allow_motion=allow,reason=self.reason,
            distance_remaining=self.distance_remaining))))

    def destroy_node(self):
        self.active=False;self.cancel()
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
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO);n=FollowGoal()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
