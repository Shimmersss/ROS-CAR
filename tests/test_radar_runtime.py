"""Synthetic ROS scans validate health; no device or motion nodes."""
import copy
import json
import math
import subprocess
import time
from pathlib import Path
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from perception_bringup.radar_health import RadarHealth
from perception_bringup.radar_config import driver_spec, mount_args

c = yaml.safe_load((Path(get_package_share_directory('perception_bringup'))/'config/radar.yaml').read_text())
assert driver_spec(c)[2]['lidar_model'] == 'N10Plus'
assert mount_args(c) is None
bad = dict(c, publish_mount_tf=True)
try:
    mount_args(bad)
    raise AssertionError('Uncalibrated mount accepted')
except ValueError:
    pass
assert '--child-frame-id' in mount_args(dict(bad, mount_calibrated=True))
try:
    driver_spec(dict(c,model='bad'))
    raise AssertionError('Unsupported model accepted')
except ValueError:
    pass
rclpy.init()
n = RadarHealth()
pub = n.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
assert n.tick()['status'] == 'WAITING'
m = LaserScan()
m.header.frame_id='laser'; m.angle_min=-1.; m.angle_max=1.; m.angle_increment=.5
m.range_min=.15; m.range_max=15.; m.ranges=[math.inf,2.,1.,math.nan,3.]
end=time.monotonic()+2
while time.monotonic()<end:
    m.header.stamp=n.get_clock().now().to_msg(); pub.publish(m); rclpy.spin_once(n,timeout_sec=.05)
r=n.tick(); assert r['status']=='OK' and r['nearest_m']==1. and r['valid_points']==3, r
m=copy.deepcopy(m); m.header.stamp.sec-=5; n.on_scan(m); assert n.tick()['status']=='STALE'
m.header.stamp=n.get_clock().now().to_msg(); m.header.frame_id='wrong'; n.on_scan(m); assert n.tick()['status']=='INVALID'
m.header.frame_id='laser'; m.ranges=[math.inf]*5; n.on_scan(m); assert n.tick()['status']=='NO_RETURNS'
assert '/cmd_vel' not in dict(n.get_topic_names_and_types())
n.destroy_node()
# Health watchdog must still publish when simulated ROS time is paused at zero.
from rclpy.parameter import Parameter
from std_msgs.msg import String
n=RadarHealth(parameter_overrides=[Parameter('use_sim_time',value=True)])
statuses=[];n.create_subscription(String,'/radar/status',statuses.append,10)
end=time.monotonic()+1.2
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
assert len(statuses)>=3, 'Health timer stopped with /clock'
n.destroy_node(); rclpy.shutdown()
# Launch succeeds without vendor packages when only observing an existing scan.
p=subprocess.Popen(['ros2','launch','perception_bringup','radar.launch.py','start_driver:=false'])
try:
    time.sleep(2); assert p.poll() is None
finally:
    p.send_signal(2)
    p.wait(timeout=10)
print('PASS radar: ROS scan transport, valid/empty/stale/malformed status, uncalibrated TF rejection, launch, no cmd_vel')
