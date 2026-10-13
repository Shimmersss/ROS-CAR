"""Real ROS fixed-mount fall profile on synthetic depth/boxes; no human fall acceptance."""
import copy
from pathlib import Path
import time
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import CameraInfo
from person_interfaces.msg import FallAlarm
from yolo_person_tracker.pose_node import PoseTrackerNode
from yolo_person_tracker.backend import Detection

class Backend:
    box=(100,290,420,430)
    def infer(self,image):return [Detection(7,self.box,.9,tuple((0.,0.,0.) for _ in range(17)))]
    def reset(self):pass

def main():
    root=Path(__file__).resolve().parents[1]
    cfg=yaml.safe_load((root/'ros2_ws/src/perception_bringup/config/pose_fixed_27cm.yaml').read_text())['/**']['ros__parameters']
    assert cfg['height_continue_s']==0., 'moving camera cannot retain an old pixel box'
    up=np.array([cfg['pose3d_up_'+axis] for axis in 'xyz'])
    np.testing.assert_allclose(np.linalg.norm(up),1.,atol=1e-12)
    assert up[2]>0 and up[1]<0 and cfg['pose3d_camera_height_m']==.27
    rclpy.init();b=Backend()
    n=PoseTrackerNode(backend=b,namespace='/perception',parameter_overrides=[
        Parameter(k,value=v) for k,v in dict(cfg,depth_registered=True,visualization_fps=0.).items()])
    io=Node('fixed_camera_fall_io');alarms=[]
    io.create_subscription(FallAlarm,'/perception/fall_alarm',alarms.append,10)
    ex=SingleThreadedExecutor();ex.add_node(n);ex.add_node(io)
    info=CameraInfo();info.width=640;info.height=480;info.header.frame_id='camera_color_optical_frame'
    info.p=[500.,0.,320.,0.,0.,500.,240.,0.,0.,0.,1.,0.];n.on_info(info)
    def pump(seconds):
        end=time.monotonic()+seconds;last=-float('inf')
        while time.monotonic()<end:
            if time.monotonic()-last>=.1:
                last=time.monotonic();c=n.bridge.cv2_to_imgmsg(np.zeros((480,640,3),np.uint8),'bgr8')
                c.header.frame_id=info.header.frame_id;c.header.stamp=n.get_clock().now().to_msg()
                d=n.bridge.cv2_to_imgmsg(np.full((480,640),1.5,np.float32),'32FC1');d.header=copy.deepcopy(c.header)
                n.on_pair(c,d)
            ex.spin_once(timeout_sec=.01)
    try:
        pump(1.5)
        assert alarms and all(m.stage==0 for m in alarms), 'static lying must not invent a fall'
        b.box=(230,20,380,300);pump(1.5)
        assert alarms[-1].stage==0
        b.box=(100,290,420,430);pump(1.5)
        assert any(m.stage==1 for m in alarms), 'fixed height drop not detected'
        assert alarms[-1].stage==2 and alarms[-1].geometry_confirmed, alarms[-1].detail
        assert io.count_publishers('/cmd_vel')==0
        print('PASS 27cm/pitch15.1/roll0.5 profile: static lying silent; rapid measured height drop suspected then confirmed; no motion')
    finally:
        ex.shutdown();n.destroy_node();io.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
