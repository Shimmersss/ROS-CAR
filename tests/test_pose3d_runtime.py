"""Synthetic RGB-D -> sampled joints -> 3D posture ROS integration."""
import copy
import time
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image
from person_interfaces.msg import PersonStateArray
from yolo_person_tracker.backend import Detection
from yolo_person_tracker.pose_node import PoseTrackerNode


class Backend:
    detections=[]
    def infer(self,image):return self.detections
    def reset(self):pass


def main():
    rclpy.init();backend=Backend()
    node=PoseTrackerNode(backend=backend,namespace='spatial',parameter_overrides=[
        Parameter('depth_registered',value=True),Parameter('max_age_s',value=1.),
        Parameter('pose3d_gravity_confirmed',value=True),Parameter('pose3d_ground_confirmed',value=True)])
    ex=SingleThreadedExecutor();ex.add_node(node);people=[]
    node.create_subscription(PersonStateArray,'person_states',people.append,10)
    cp=node.create_publisher(Image,node.cfg['color_topic'],10)
    dp=node.create_publisher(Image,node.cfg['depth_topic'],10)
    ip=node.create_publisher(CameraInfo,node.cfg['camera_info_topic'],10)
    def spin(seconds=.1):
        end=time.monotonic()+seconds
        while time.monotonic()<end:ex.spin_once(timeout_sec=.005)
    def send(height=1.1,angle=0.,holes=False):
        center=np.array([0.,1.-height,2.]);a=np.radians(angle)
        torso=np.array([0.,-.5*np.cos(a),.5*np.sin(a)])
        xyz={5:center+torso/2+[-.2,0.,0.],6:center+torso/2+[.2,0.,0.],
             11:center-torso/2+[-.15,0.,0.],12:center-torso/2+[.15,0.,0.]}
        xyz[13]=xyz[11]+[0.,.4,0.];xyz[14]=xyz[12]+[0.,.4,0.]
        p=np.zeros((17,3));depth=np.zeros((480,640),np.uint16)
        for i,v in xyz.items():
            u=300*v[0]/v[2]+320;w=300*v[1]/v[2]+240;p[i]=(u,w,.9)
            x,y=int(round(u)),int(round(w))
            if not holes:depth[y-4:y+5,x-4:x+5]=round(v[2]*1000)
        valid=p[:,2]>.5;bounds=(float(p[valid,0].min()-10),float(p[valid,1].min()-10),
                               float(p[valid,0].max()+10),float(p[valid,1].max()+10))
        backend.detections=[Detection(7,bounds,.9,tuple(map(tuple,p)))]
        info=CameraInfo();info.width=640;info.height=480;info.header.frame_id='camera_color_optical_frame'
        info.p=[300.,0.,320.,0.,0.,300.,240.,0.,0.,0.,1.,0.];ip.publish(info);spin(.01)
        c=node.bridge.cv2_to_imgmsg(np.zeros((480,640,3),np.uint8),'bgr8');c.header.frame_id=info.header.frame_id;c.header.stamp=node.get_clock().now().to_msg()
        d=node.bridge.cv2_to_imgmsg(depth,'16UC1');d.header=copy.deepcopy(c.header);cp.publish(c);dp.publish(d)
        end=time.monotonic()+.7
        while time.monotonic()<end and (not people or people[-1].header!=c.header):ex.spin_once(timeout_sec=.005)
        assert people and people[-1].header==c.header
        spin(.10)
        return people[-1].persons[0]
    try:
        spin(.3)
        for _ in range(6):p=send()
        assert p.posture==p.STANDING and 'basis=3d' in p.detail,p
        for height,angle in ((.95,25),(.75,48),(.5,75),(.35,90)):p=send(height,angle)
        assert p.fall_stage==p.FALL_SUSPECTED,p
        p=send(.35,90,holes=True)
        assert p.fall_stage==p.FALL_SUSPECTED and 'basis=2d' in p.detail,p
        for _ in range(12):p=send(.35,90)
        assert p.posture==p.FALLEN and p.fall_stage==p.FALL_CONFIRMED,p
        assert all(p.keypoints_3d_valid[i] for i in (5,6,11,12)),p
        p=send(.35,90,holes=True)
        assert 'basis=2d' in p.detail and not any(p.keypoints_3d_valid),p
        spin(1.1);assert not people[-1].valid
        assert not any(n=='/cmd_vel' for n,_ in node.get_topic_names_and_types())
        print('PASS 3D posture ROS: real depth sampling of synthetic planes, continuous rotation/descent, confirmed fall, holes/fallback/stale')
    finally:
        ex.shutdown();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
