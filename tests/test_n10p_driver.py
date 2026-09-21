"""Actual N10Plus driver reads synthetic 108-byte packets from a PTY."""
import math
import os
import pty
import signal
import struct
import subprocess
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String

master, slave = pty.openpty()
port = os.ttyname(slave)
stop = threading.Event()
def feed():
    while not stop.is_set():
        for angle in range(0,36000,1000):
            if stop.is_set(): return
            b=bytearray(108); b[0:2]=b'\xa5\x5a'
            struct.pack_into('>H',b,5,angle)
            for i in range(32):
                struct.pack_into('>HB',b,7+i*3,1000 if 4500 <= angle <= 12500 else 2000,100)
            struct.pack_into('>H',b,105,angle+990)
            b[107]=sum(b[:107])&255
            os.write(master,b)
            time.sleep(.003)
rclpy.init(); n=Node('n10p_pty_test'); scans=[]; clouds=[]; states=[]
n.create_subscription(LaserScan,'/scan',scans.append,qos_profile_sensor_data)
n.create_subscription(PointCloud2,'/radar/points',clouds.append,qos_profile_sensor_data)
n.create_subscription(String,'/radar/status',states.append,10)
log=open('/tmp/n10p-driver.log','w+')
p=subprocess.Popen(['ros2','launch','perception_bringup','radar.launch.py','serial_port:='+port],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
t=threading.Thread(target=feed,daemon=True); t.start()
try:
    deadline=time.monotonic()+20
    while time.monotonic()<deadline and not (len(scans)>=3 and clouds and states):
        rclpy.spin_once(n,timeout_sec=.1)
        if p.poll() is not None: break
    log.flush()
    if not (scans and clouds):
        log.seek(0); raise AssertionError(log.read()[-8000:])
    values=[v for v in scans[-1].ranges if math.isfinite(v)]
    assert values and min(values) < 1.01 and max(values) > 1.99, values[:10]
    near_angles=[scans[-1].angle_min+i*scans[-1].angle_increment
                 for i,v in enumerate(scans[-1].ranges) if math.isfinite(v) and v<1.1]
    assert near_angles and all(0 < a < math.pi for a in near_angles), near_angles[:10]
    assert scans[-1].header.frame_id=='laser' and clouds[-1].header.frame_id=='laser'
    assert '/cmd_vel' not in dict(n.get_topic_names_and_types())
    print('PASS real N10Plus driver: PTY packets -> /scan + /radar/points, 1m left / 2m elsewhere, consistent laser frame; no hardware/motion')
finally:
    os.killpg(p.pid,signal.SIGINT)
    # Feed until launch has shut down; vendor serial reader can otherwise block.
    try: p.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid,signal.SIGKILL); p.wait()
    stop.set(); t.join(timeout=2)
    os.close(master); os.close(slave); log.close(); n.destroy_node(); rclpy.shutdown()
