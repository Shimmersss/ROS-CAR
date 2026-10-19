"""Installed follow launch .5 m/s and higher gains, synthetic targets only."""
import json,math,os,signal,subprocess,tempfile,time
import rclpy
from rclpy.node import Node
from person_interfaces.msg import TargetState
from geometry_msgs.msg import Twist
from std_msgs.msg import String

def main():
    rclpy.init();n=Node('fast_follow_fixture');pub=n.create_publisher(TargetState,'/perception/target_state',10)
    commands=[];states=[]
    n.create_subscription(Twist,'/cmd_vel',commands.append,10)
    n.create_subscription(String,'/control/state',lambda m:states.append(json.loads(m.data)),10)
    log=tempfile.TemporaryFile(mode='w+');proc=subprocess.Popen(['ros2','launch','motion_guard','follow.launch.py',
        'motion_enabled:=true','auto_arm:=true','target_auto_recover:=true','geometry_confirmed:=true','mount_calibrated:=true',
        'stopping_model_confirmed:=true','emergency_kws_required:=false','expected_source:=yolo','target_frame:=camera_optical',
        'target_distance_m:=1.3','max_linear_mps:=0.5','distance_gain:=1.0','bearing_gain:=1.6','target_grace_s:=0.6'],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    def pump(seconds,distance=2.,bearing=0.,valid=True):
        end=time.monotonic()+seconds;last=0
        while time.monotonic()<end:
            assert proc.poll() is None
            if time.monotonic()-last>.04:
                m=TargetState();m.header.stamp=n.get_clock().now().to_msg();m.observation_stamp=m.header.stamp
                m.header.frame_id='camera_optical';m.source='yolo';m.status=TargetState.TRACKING;m.position_valid=valid
                m.position.x=distance*math.sin(bearing);m.position.z=distance*math.cos(bearing)
                m.horizontal_distance_m=distance;m.bearing_rad=bearing;m.measurement_age_s=0.
                pub.publish(m);last=time.monotonic()
            rclpy.spin_once(n,timeout_sec=.01)
    try:
        deadline=time.monotonic()+12.
        while time.monotonic()<deadline and not any(m.linear.x==.5 for m in commands):pump(.1)
        assert any(m.linear.x==.5 for m in commands),states[-3:]
        pump(.5,1.4);assert abs(commands[-1].linear.x-.1)<1e-6
        pump(.5,1.4,.2);assert abs(commands[-1].angular.z+.32)<1e-6
        assert all(0<=m.linear.x<=.5 and abs(m.angular.z)<=.5 for m in commands)
        pump(.8,valid=False);assert commands[-1].linear.x==commands[-1].angular.z==0.
        assert states[-1]['authority_state']=='FAULT'
        pump(.5);assert states[-1]['authority_state']=='ARMED_FOLLOW'
        assert n.count_publishers('/cmd_vel')==1
        print('PASS installed fast follow: .5 cap, 1.0/1.6 gains, invalid target stops at original .6 grace, target-only recovery, unique guard (synthetic)')
    except Exception:
        log.seek(0);print(log.read());raise
    finally:
        os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        log.close();n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
