"""Actual Humble SLAM + Nav2 processes with synthetic room/odometry, no hardware."""
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from geometry_msgs.msg import TransformStamped,Twist,PoseStamped,PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid,Odometry
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import ComputePathToPose,NavigateToPose
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster

rclpy.init();n=Node('navigation_stack_fixture')
q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
odom=n.create_publisher(Odometry,'/odom',10);scan=n.create_publisher(LaserScan,'/scan',10)
map_pub=n.create_publisher(OccupancyGrid,'/map',q)
tf=TransformBroadcaster(n);static=StaticTransformBroadcaster(n)
laser=TransformStamped();laser.header.frame_id='base_footprint';laser.child_frame_id='laser';laser.transform.rotation.w=1.
static.sendTransform(laser)
seen_maps=[];raw=[];final=[]
n.create_subscription(OccupancyGrid,'/map',lambda m:seen_maps.append(m),q)
n.create_subscription(Twist,'/navigation/cmd_vel_raw',lambda m:raw.append(m),10)
n.create_subscription(Twist,'/cmd_vel',lambda m:final.append(m),10)
mode='mapping';procs=[];started=time.monotonic()

def feed():
    stamp=n.get_clock().now().to_msg()
    o=Odometry();o.header.stamp=stamp;o.header.frame_id='odom';o.child_frame_id='base_footprint';o.pose.pose.orientation.w=1.
    o.pose.pose.position.x=min(1.,(time.monotonic()-started)*.05) if mode=='mapping' else 0.
    odom.publish(o)
    t=TransformStamped();t.header=o.header;t.child_frame_id='base_footprint';t.transform.rotation.w=1.;t.transform.translation.x=o.pose.pose.position.x;tf.sendTransform(t)
    s=LaserScan();s.header.stamp=stamp;s.header.frame_id='laser';s.angle_min=-math.pi;s.angle_increment=2*math.pi/360;s.angle_max=s.angle_min+359*s.angle_increment;s.range_min=.05;s.range_max=10.;s.scan_time=.1
    s.ranges=[float(5/max(abs(math.cos(s.angle_min+i*s.angle_increment)),abs(math.sin(s.angle_min+i*s.angle_increment)))) for i in range(360)];scan.publish(s)
    if mode=='external':
        t=TransformStamped();t.header.stamp=stamp;t.header.frame_id='map';t.child_frame_id='odom';t.transform.rotation.w=1.;tf.sendTransform(t)

n.create_timer(.05,feed)
def wait(predicate,seconds=30):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        rclpy.spin_once(n,timeout_sec=.03)
        if predicate():return
    raise AssertionError('Timed out: '+str(predicate)+' map samples='+str([(len(m.data),min(m.data,default=-2),max(m.data,default=-2)) for m in seen_maps[-3:]]))
def launch(args,name):
    log=open('/tmp/'+name+'.log','w');p=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);procs.append((p,log));return p

def stop(p):
    os.killpg(p.pid,signal.SIGINT)
    try:p.wait(timeout=10)
    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()

try:
    from ament_index_python.packages import get_package_share_directory
    params=str(Path(get_package_share_directory('navigation_bringup'))/'config/slam.yaml')
    p=launch(['ros2','run','slam_toolbox','async_slam_toolbox_node','--ros-args','--params-file',params],'slam-test')
    wait(lambda:any(len(m.data)>0 and 100 in m.data for m in seen_maps),40)
    print('PASS actual SLAM Toolbox generated occupied/free room map',flush=True)
    stop(p);mode='external'
    m=OccupancyGrid();m.header.frame_id='map';m.header.stamp=n.get_clock().now().to_msg();m.info.resolution=.05;m.info.width=240;m.info.height=240;m.info.origin.position.x=-6.;m.info.origin.position.y=-6.;m.info.origin.orientation.w=1.
    data=[]
    for y in range(240):
        for x in range(240):
            wx=-6+(x+.5)*.05;wy=-6+(y+.5)*.05
            data.append(100 if x in (0,239) or y in (0,239) or (1.3<wx<1.7 and abs(wy)<.6) else 0)
    m.data=data;map_pub.publish(m)
    p=launch(['ros2','launch','navigation_bringup','navigation.launch.py','mode:=external','publish_odom_tf:=false'],'nav2-test')
    planner=ActionClient(n,ComputePathToPose,'compute_path_to_pose')
    wait(lambda:planner.server_is_ready(),50)
    # Server discovery precedes lifecycle activation; wait for costmaps to settle.
    until=time.monotonic()+6;wait(lambda:time.monotonic()>until,10)
    goal=ComputePathToPose.Goal();goal.goal.header.frame_id='map';goal.goal.header.stamp=n.get_clock().now().to_msg();goal.goal.pose.position.x=3.5;goal.goal.pose.orientation.w=1.;goal.planner_id='GridBased'
    f=planner.send_goal_async(goal);wait(f.done);h=f.result();assert h.accepted
    f=h.get_result_async();wait(f.done);result=f.result();assert result.status==4,result
    poses=result.result.path.poses;assert len(poses)>20
    assert max(abs(p.pose.position.y) for p in poses)>.9,'Planner did not detour around wall'
    print('PASS actual NavFn global path detours around obstacle, poses='+str(len(poses)),flush=True)
    nav=ActionClient(n,NavigateToPose,'navigate_to_pose');wait(nav.server_is_ready)
    g=NavigateToPose.Goal();g.pose=goal.goal
    f=nav.send_goal_async(g);wait(f.done);h=f.result();assert h.accepted
    wait(lambda:any(abs(v.linear.x)+abs(v.angular.z)>.001 for v in raw),20)
    assert not any(abs(v.linear.x)+abs(v.angular.z)>.00001 for v in final),'Unexpected final motion'
    f=h.cancel_goal_async();wait(f.done)
    f=h.get_result_async();wait(f.done)
    client=n.create_client(Trigger,'/navigation/start_follow');wait(client.service_is_ready)
    f=client.call_async(Trigger.Request());wait(f.done);assert not f.result().success
    print('PASS actual BT/DWB commands use isolated raw topic; uncalibrated follow rejected; final velocity zero',flush=True)
    stop(p);mode='localization';n.destroy_publisher(map_pub)
    import yaml
    Path('/tmp/navigation-map.pgm').write_bytes(b'P5\n240 240\n255\n'+bytes(0 if v==100 else 254 for v in data))
    Path('/tmp/navigation-map.yaml').write_text(yaml.safe_dump(dict(image='navigation-map.pgm',resolution=.05,origin=[-6.,-6.,0.],negate=0,occupied_thresh=.65,free_thresh=.25)))
    poses=[]
    n.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',lambda m:poses.append(m),10)
    initial=n.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
    def initialize():
        if poses:return
        m=PoseWithCovarianceStamped();m.header.frame_id='map';m.header.stamp=n.get_clock().now().to_msg();m.pose.pose.orientation.w=1.
        m.pose.covariance[0]=.25;m.pose.covariance[7]=.25;m.pose.covariance[35]=.1;initial.publish(m)
    timer=n.create_timer(1.,initialize)
    p=launch(['ros2','launch','navigation_bringup','navigation.launch.py','mode:=localization','map:=/tmp/navigation-map.yaml','publish_odom_tf:=false'],'amcl-test')
    wait(lambda:len(poses)>0,40)
    assert all(math.isfinite(v) for v in (poses[-1].pose.pose.position.x,poses[-1].pose.pose.position.y))
    print('PASS actual map_server + AMCL activation and initial-pose localization output',flush=True)

finally:
    for p,log in reversed(procs):
        if p.poll() is None:stop(p)
        log.close()
    n.destroy_node();rclpy.shutdown()
