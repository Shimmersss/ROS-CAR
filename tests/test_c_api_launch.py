"""Public API accepts C; missing weights stay NOT_READY and the default guard stays idle."""
import os
import signal
import subprocess
import tempfile
import time
import json
import rclpy
from rclpy.node import Node
from person_interfaces.msg import TargetState, PersonStateArray
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def main():
    rclpy.init();node=Node('c_api_probe');states=[];persons=[];guards=[];vel=[]
    node.create_subscription(TargetState,'/perception/target_state',states.append,10)
    node.create_subscription(PersonStateArray,'/perception/person_states',persons.append,10)
    node.create_subscription(String,'/control/state',lambda m:guards.append(json.loads(m.data)),10)
    node.create_subscription(Twist,'/cmd_vel',vel.append,10)
    with tempfile.TemporaryFile(mode='w+') as log:
        proc=subprocess.Popen(['ros2','launch','roscar_api','api.launch.py','with_perception:=true','route:=yolo_pose'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            end=time.monotonic()+10
            while time.monotonic()<end and not (states and persons and guards and vel):
                assert proc.poll() is None
                rclpy.spin_once(node,timeout_sec=.05)
            assert states and persons and guards and vel
            assert all(s.status==TargetState.NOT_READY and s.source=='yolo' and not s.position_valid for s in states)
            assert all(not p.valid and not p.persons for p in persons)
            assert guards[-1]['mode']=='PERCEPTION_ONLY' and guards[-1]['command_mode']=='IDLE'
            assert all(v.linear.x==0 and v.angular.z==0 for v in vel)
            assert 'wheeltec_robot' not in node.get_node_names()
            print('PASS public API C route, missing model rejection, invalid-pose heartbeat, default IDLE/zero output')
        except Exception:
            log.seek(0);print(log.read());raise
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=6)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=3)
            node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
