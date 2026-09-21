"""ROS bridge fault injection with synthetic TF/targets and a controllable Nav2 server."""
import json
import math
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer,CancelResponse
from geometry_msgs.msg import TransformStamped,Twist,TwistStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from person_interfaces.msg import TargetState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster,Buffer,TransformListener,TransformException
from navigation_bringup.follow_goal import FollowGoal
from navigation_bringup.velocity_adapter import VelocityAdapter
from navigation_bringup.odom_tf import OdomTF

rclpy.init();n=Node('navigation_bridge_fixture')
params={'camera_calibrated':True,'odometry_confirmed':True,'expected_source':'yolo','target_frame':'camera'}
f=FollowGoal(parameter_overrides=[Parameter(k,value=v) for k,v in params.items()]);a=VelocityAdapter();o=OdomTF()
e=MultiThreadedExecutor(num_threads=5)
for node in (n,f,a,o):e.add_node(node)
pub=n.create_publisher(TargetState,'/perception/target_state',10)
odom=n.create_publisher(Odometry,'/odom',10)
control=n.create_publisher(String,'/control/state',10)
raw=n.create_publisher(Twist,'/navigation/cmd_vel_raw',10)
tf=TransformBroadcaster(n);st=StaticTransformBroadcaster(n)
t=TransformStamped();t.header.frame_id='base_footprint';t.child_frame_id='camera';t.transform.rotation.w=1.;st.sendTransform(t)
buf=Buffer();listener=TransformListener(buf,n)
states=[];requests=[];goals=[];canceled=[]
n.create_subscription(String,'/navigation/state',lambda m:states.append(json.loads(m.data)),10)
n.create_subscription(TwistStamped,'/control/cmd_vel_request',lambda m:requests.append(m),10)
valid=True;send_raw=True;send_odom=True;target_id="7";person_x=3.

def execute(handle):
    goals.append(handle.request)
    while rclpy.ok():
        if handle.is_cancel_requested:
            canceled.append(True);handle.canceled();return NavigateToPose.Result()
        time.sleep(.02)
    return NavigateToPose.Result()
server=ActionServer(n,NavigateToPose,'navigate_to_pose',execute_callback=execute,
                    cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=ReentrantCallbackGroup())

def feed():
    now=n.get_clock().now();stamp=now.to_msg()
    if send_odom:
        msg=Odometry();msg.header.stamp=stamp;msg.header.frame_id='odom';msg.child_frame_id='base_footprint'
        msg.pose.pose.orientation.w=1.;msg.pose.pose.position.z=1.2;odom.publish(msg)
    t=TransformStamped();t.header.stamp=stamp;t.header.frame_id='map';t.child_frame_id='odom';t.transform.rotation.w=1.;tf.sendTransform(t)
    m=TargetState();m.header.stamp=stamp;m.header.frame_id='camera';m.source='yolo';m.target_id=target_id;m.status=m.TRACKING if valid else m.LOST;m.position_valid=valid
    m.observation_stamp=rclpy.time.Time(nanoseconds=now.nanoseconds-100_000_000).to_msg();m.measurement_age_s=.1;m.position.x=person_x;m.position.z=1.;pub.publish(m)
    control.publish(String(data=json.dumps({'mode':'ARMED'})))
    if send_raw:
        v=Twist();v.linear.x=.1;raw.publish(v)
n.create_timer(.05,feed)
thread=threading.Thread(target=e.spin,daemon=True);thread.start()

def wait(fn,timeout=10):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if fn():return
        time.sleep(.03)
    raise AssertionError('Timeout: '+str(states[-3:]))
def call(path):
    client=n.create_client(Trigger,path);assert client.wait_for_service(timeout_sec=5)
    future=client.call_async(Trigger.Request());wait(future.done);result=future.result();n.destroy_client(client);return result
try:
    wait(lambda:states and states[-1]['ready'])
    assert buf.lookup_transform('odom','base_footprint',rclpy.time.Time()).transform.translation.z==0
    assert call('/navigation/start_follow').success
    wait(lambda:requests and requests[-1].twist.linear.x>.05)
    assert abs(goals[-1].pose.pose.position.x-2.)<.01
    print('PASS observation-time goal transform, standoff, planar odom TF, authorized raw velocity',flush=True)
    lookup=f.buffer.lookup_transform
    def unavailable(*args,**kwargs):raise TransformException('injected delayed transform')
    f.buffer.lookup_transform=unavailable
    wait(lambda:states[-1]['reason']=='waiting_for_observation_tf' and requests[-1].twist.linear.x==0,timeout=.14)
    assert states[-1]['active'] and not states[-1]['fault']
    f.buffer.lookup_transform=lookup
    wait(lambda:requests[-1].twist.linear.x>.05)
    print('PASS short TF arrival delay zeros velocity without latching a false fault',flush=True)
    person_x=.5
    wait(lambda:states[-1]['reason']=='within_follow_distance' and requests[-1].twist.linear.x==0 and len(canceled)>=1)
    person_x=3.
    wait(lambda:requests[-1].twist.linear.x>.05 and len(goals)>=2)
    valid=False
    wait(lambda:states[-1]['fault'] and canceled and requests[-1].twist.linear.x==0)
    valid=True;time.sleep(.5);assert not states[-1]['active']
    assert call('/navigation/stop_follow').success
    wait(lambda:states[-1]['ready']);assert call('/navigation/start_follow').success
    wait(lambda:requests[-1].twist.linear.x>.05)
    send_raw=False
    time.sleep(2.3)
    assert requests[-1].twist.linear.x==0 and requests[-1].header.stamp.sec==0
    send_raw=True;send_odom=False
    wait(lambda:states[-1]['fault'])
    print('PASS close-target hold/resume, target loss cancellation, no fault auto-resume, stale command/odometry stop',flush=True)
    send_odom=True
    assert call('/navigation/stop_follow').success
    wait(lambda:states[-1]['ready']);assert call('/navigation/start_follow').success
    wait(lambda:requests[-1].twist.linear.x>.05)
    target_id='8'
    wait(lambda:states[-1]['fault'] and states[-1]['reason']=='selected_target_changed')
    print('PASS target ID change requires explicit restart',flush=True)
    assert call('/navigation/stop_follow').success
    wait(lambda:states[-1]['ready']);assert call('/navigation/start_follow').success
    wait(lambda:requests[-1].twist.linear.x>.05)
    f.buffer.lookup_transform=unavailable
    wait(lambda:states[-1]['fault'] and states[-1]['reason'].startswith('tf_unavailable:'))
    f.buffer.lookup_transform=lookup
    print('PASS persistent TF loss still latches a fault',flush=True)
finally:
    valid=False
    call('/navigation/stop_follow');time.sleep(.2)
    server.destroy();e.shutdown(timeout_sec=5)
    for node in (n,f,a,o):node.destroy_node()
    rclpy.shutdown();thread.join(timeout=3)
