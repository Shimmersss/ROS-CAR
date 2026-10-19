"""Paused synthetic ROS clock: request heartbeat continues, stale target becomes zero."""
import json
import os
import time
import rclpy
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from rclpy.time import Time
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from person_interfaces.msg import TargetState
from astra_body_adapter.person_follower_node import PersonFollowerNode

assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
assert os.environ.get('ROS_DOMAIN_ID') == '227'
rclpy.init()
node = PersonFollowerNode(parameter_overrides=[Parameter(k,value=v) for k,v in dict(
    use_sim_time=True, enabled=True, expected_source='yolo', ackermann=True,
    target_distance_m=1.3, max_linear_mps=.2).items()])
node.get_clock().set_ros_time_override(Time(seconds=10))
probe = rclpy.create_node('paused_clock_probe'); ex = SingleThreadedExecutor()
ex.add_node(node); ex.add_node(probe)
targets = probe.create_publisher(TargetState,'/perception/target_state',10)
authority = probe.create_publisher(String,'/control/state',10)
requests=[];probe.create_subscription(TwistStamped,'/control/cmd_vel_request',requests.append,20)
try:
    deadline=time.monotonic()+3
    while targets.get_subscription_count()<1 and time.monotonic()<deadline:ex.spin_once(timeout_sec=.01)
    target=TargetState(status=TargetState.TRACKING,position_valid=True,source='yolo',horizontal_distance_m=2.,bearing_rad=0.,measurement_age_s=0.)
    target.header.stamp=target.observation_stamp=Time(seconds=10).to_msg()
    targets.publish(target)
    start=time.monotonic();last=0
    while time.monotonic()-start<1.1:
        now=time.monotonic()
        if now-last>.03:
            authority.publish(String(data=json.dumps(dict(authority_state='ARMED_FOLLOW',command_mode='FOLLOW'))));last=now
        ex.spin_once(timeout_sec=.005)
    assert len(requests)>=12, len(requests)
    assert any(m.twist.linear.x>0 for m in requests), 'fresh target should initially follow'
    assert all(m.twist.linear.x==m.twist.angular.z==0 for m in requests[-4:]), 'old target must expire on real arrival age'
    assert all(m.header.stamp.sec==10 for m in requests), 'ROS clock must remain paused throughout'
    assert probe.count_publishers('/cmd_vel')==0
    print('PASS paused ROS time: steady requests continue, fresh target initially follows, real-age stale target withdraws to zero; no final velocity publisher')
finally:
    ex.shutdown();node.destroy_node();probe.destroy_node();rclpy.shutdown()
