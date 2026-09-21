"""Play a real bag through the shipped script and assert commands stay isolated."""
import os
import subprocess
import tempfile
import time
from pathlib import Path
import rosbag2_py
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from geometry_msgs.msg import Twist

with tempfile.TemporaryDirectory() as tmp:
    bag=str(Path(tmp)/'bag')
    writer=rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=bag,storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
    for name in ('/cmd_vel','/navigation/cmd_vel_raw','/unexpected_velocity'):
        writer.create_topic(rosbag2_py.TopicMetadata(name=name,type='geometry_msgs/msg/Twist',serialization_format='cdr'))
    message=Twist();message.linear.x=.1
    start=time.time_ns()
    for i in range(40):
        for name in ('/cmd_vel','/navigation/cmd_vel_raw','/unexpected_velocity'):
            writer.write(name,serialize_message(message),start+i*100000000)
    del writer
    rclpy.init();n=Node('replay_test');out=[];nav_out=[]
    n.create_subscription(Twist,'/replay/cmd_vel',out.append,10)
    n.create_subscription(Twist,'/replay/nav_cmd_vel_raw',nav_out.append,10)
    script=Path(__file__).resolve().parents[1]/'scripts/replay_follow.sh'
    log=tempfile.TemporaryFile(mode='w+')
    p=subprocess.Popen(['bash',str(script),bag],stdout=log,stderr=subprocess.STDOUT)
    try:
        end=time.monotonic()+10
        while time.monotonic()<end and p.poll() is None:
            rclpy.spin_once(n,timeout_sec=.05)
            topics=dict(n.get_topic_names_and_types())
            assert '/cmd_vel' not in topics and '/navigation/cmd_vel_raw' not in topics and '/unexpected_velocity' not in topics,topics
        log.seek(0)
        assert p.poll()==0 and out and nav_out,log.read()
        assert all(m.linear.x==.1 for m in out)
        print('PASS real bag replay: /cmd_vel -> /replay/cmd_vel; Nav2 raw velocity also remapped; unlisted topics excluded')
    finally:
        if p.poll() is None:p.terminate();p.wait(timeout=5)
        n.destroy_node();rclpy.shutdown();log.close()
