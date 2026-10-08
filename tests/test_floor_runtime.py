"""Depth-fitted floor: C pose node publishes it and uses it as per-frame gravity; unified
localizer uses it for ankle rays. Synthetic geometry only; no physical accuracy claim."""
import copy
import math
import time
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image
from person_interfaces.msg import FloorPlane, PersonStateArray, PersonGroundArray, TargetState
from yolo_person_tracker.backend import Detection
from yolo_person_tracker.pose_node import PoseTrackerNode
from yolo_person_tracker.unified_node import UnifiedLocalizer
from monotonic_ros_clock import MonotonicRosClock
from test_ground_runtime import to_state, static_tf, FRAME
from test_unified import measured
from test_ground import mount


class Backend:
    detections=[]
    def infer(self,image):return self.detections
    def reset(self):pass


def floor_depth(camera_height=1.):
    """Level camera, f=300: rows below the horizon see the floor (millimetres)."""
    v=np.arange(480,dtype=float)[:,None]-240.
    with np.errstate(divide='ignore'):
        z=np.where(v>0,camera_height*300./v,0.)
    z=np.repeat(z,640,axis=1)
    z[z>6.]=0.
    return np.round(z*1000).astype(np.uint16)


def pose_node_checks():
    backend=Backend()
    node=PoseTrackerNode(backend=backend,namespace='floor_pose',parameter_overrides=[
        Parameter('use_sim_time',value=True),
        Parameter('depth_registered',value=True),Parameter('max_age_s',value=1.),
        Parameter('floor_fit_enabled',value=True),Parameter('pose3d_up_source',value='floor'),
        Parameter('pose3d_gravity_confirmed',value=True),Parameter('pose3d_ground_confirmed',value=True)])
    ex=SingleThreadedExecutor();ex.add_node(node);people=[];floors=[]
    clock=MonotonicRosClock(node)
    node.create_subscription(PersonStateArray,'person_states',people.append,10)
    node.create_subscription(FloorPlane,'floor_plane',floors.append,10)
    cp=node.create_publisher(Image,node.cfg['color_topic'],10)
    dp=node.create_publisher(Image,node.cfg['depth_topic'],10)
    ip=node.create_publisher(CameraInfo,node.cfg['camera_info_topic'],10)
    def spin(seconds=.1):
        end=time.monotonic()+seconds
        while time.monotonic()<end:clock.spin_once(ex)
    def send(height=1.1,angle=0.,floor=True):
        center=np.array([0.,1.-height,2.]);a=np.radians(angle)
        torso=np.array([0.,-.5*np.cos(a),.5*np.sin(a)])
        xyz={5:center+torso/2+[-.2,0.,0.],6:center+torso/2+[.2,0.,0.],
             11:center-torso/2+[-.15,0.,0.],12:center-torso/2+[.15,0.,0.]}
        xyz[13]=xyz[11]+[0.,.4,0.];xyz[14]=xyz[12]+[0.,.4,0.]
        p=np.zeros((17,3));depth=floor_depth() if floor else np.zeros((480,640),np.uint16)
        for i,v in xyz.items():
            u=300*v[0]/v[2]+320;w=300*v[1]/v[2]+240;p[i]=(u,w,.9)
            x,y=int(round(u)),int(round(w))
            depth[y-4:y+5,x-4:x+5]=round(v[2]*1000)
        valid=p[:,2]>.5;bounds=(float(p[valid,0].min()-10),float(p[valid,1].min()-10),
                               float(p[valid,0].max()+10),float(p[valid,1].max()+10))
        backend.detections=[Detection(7,bounds,.9,tuple(map(tuple,p)))]
        info=CameraInfo();info.width=640;info.height=480;info.header.frame_id='camera_color_optical_frame'
        info.p=[300.,0.,320.,0.,0.,300.,240.,0.,0.,0.,1.,0.];ip.publish(info);spin(.01)
        c=node.bridge.cv2_to_imgmsg(np.zeros((480,640,3),np.uint8),'bgr8');c.header.frame_id=info.header.frame_id;c.header.stamp=node.get_clock().now().to_msg()
        d=node.bridge.cv2_to_imgmsg(depth,'16UC1');d.header=copy.deepcopy(c.header);cp.publish(c);dp.publish(d)
        end=time.monotonic()+.7
        while time.monotonic()<end and (not people or people[-1].header!=c.header):clock.spin_once(ex)
        assert people and people[-1].header==c.header
        spin(.10)
        assert floors and floors[-1].header==c.header,'floor must carry the colour frame header'
        return people[-1].persons[0],floors[-1]
    try:
        spin(.3)
        for _ in range(5):p,f=send()
        assert f.valid and f.stable,f
        assert math.degrees(math.acos(min(1.,-f.up.y)))<1. and abs(f.height_m-1.)<.02,f
        assert abs(f.pitch_up_deg)<1. and abs(f.roll_deg)<1.,f
        assert p.posture==p.STANDING and 'basis=3d' in p.detail and 'roll=' in p.detail,p
        for height,angle in ((.95,25),(.75,48),(.5,75),(.35,90)):p,_=send(height,angle)
        assert p.fall_stage==p.FALL_SUSPECTED,p
        for _ in range(12):p,_=send(.35,90)
        assert p.posture==p.FALLEN and p.fall_stage==p.FALL_CONFIRMED,p
        # No floor in view: no stale gravity is reused; the latched event is kept.
        p,f=send(.35,90,floor=False)
        assert not f.valid and not f.stable,f
        assert 'gravity=unavailable' in p.detail and p.fall_stage==p.FALL_CONFIRMED,p
        p,f=send(floor=True)
        assert f.valid and not f.stable and 'gravity=unavailable' in p.detail,(f,p)
        assert not any(n=='/cmd_vel' for n,_ in node.get_topic_names_and_types())
    finally:
        ex.shutdown();node.destroy_node()


def unified_checks():
    true,calibrated=mount(pitch_deg=10.),mount(pitch_deg=12.)
    nodes={}
    for name,enabled in (('floor',True),('plain',False)):
        n=UnifiedLocalizer(namespace=f'floor_unified_{name}',parameter_overrides=[
            Parameter('extrinsics_calibrated',value=True),Parameter('ground_plane_confirmed',value=True),
            Parameter('max_age_s',value=.5),Parameter('floor_plane_enabled',value=enabled)])
        n.buffer.set_transform_static(static_tf(calibrated),'synthetic_fixture')
        info=CameraInfo();info.width=640;info.height=480;info.header.frame_id=FRAME
        info.p=[500.,0.,320.,0.,0.,500.,240.,0.,0.,0.,1.,0.];n.on_info(info)
        nodes[name]=n
    outputs={k:[] for k in nodes}
    ex=SingleThreadedExecutor()
    for k,n in nodes.items():
        ex.add_node(n);n.create_subscription(PersonGroundArray,'person_positions',outputs[k].append,10)
    r=true.rotation
    from yolo_person_tracker.ground import rotation_matrix
    up=rotation_matrix((r.x,r.y,r.z,r.w)).T@np.array([0.,0.,1.])
    def spin(seconds=.06):
        end=time.monotonic()+seconds
        while time.monotonic()<end:ex.spin_once(timeout_sec=.005)
    def send(with_floor=True,stale_floor=False):
        fixture=measured(true,foot=(2.5,.4),indices=())
        p=to_state(fixture,'0:7');p.body_depth_valid=False;p.body_depth_m=math.nan
        stamp=nodes['floor'].get_clock().now().to_msg()
        msg=PersonStateArray(valid=True);msg.header.frame_id=FRAME;msg.header.stamp=stamp;msg.persons=[p]
        floor=FloorPlane(valid=True,stable=True,height_m=float(true.translation.z),rms_m=.003)
        floor.header=copy.deepcopy(msg.header);floor.up.x,floor.up.y,floor.up.z=map(float,up)
        if stale_floor:floor.header.stamp.nanosec=(floor.header.stamp.nanosec+1000)%1000000000
        if with_floor:nodes['floor'].on_floor(floor)
        for k,n in nodes.items():
            n.on_persons(msg)
            target=TargetState(target_id='0:7',status=TargetState.TRACKING,source='yolo')
            target.header.stamp=copy.deepcopy(stamp);n.on_target(target);n.tick()
        spin()
        return {k:outputs[k][-1].persons[0] for k in nodes}
    try:
        spin(.2)
        for _ in range(3):result=send()
        assert result['floor'].valid and result['floor'].method=='mono_ankles_floor',result['floor']
        assert math.dist((result['floor'].position.x,result['floor'].position.y),(2.5,.4))<.05,result['floor']
        assert result['plain'].valid and result['plain'].method=='mono_ankles'
        assert abs(result['plain'].position.x-2.5)>.3,'calibrated plane carries the 2 degree pitch error'
        # A floor from another frame stamp is never substituted.
        result=send(stale_floor=True)
        # (The calibrated plane is 0.4 m off, so the track may also be held for reacquisition.)
        assert result['floor'].method!='mono_ankles_floor' and 'floor: disabled or unavailable' in result['floor'].detail,result['floor']
    finally:
        ex.shutdown()
        for n in nodes.values():n.destroy_node()


def main():
    rclpy.init()
    try:
        pose_node_checks()
        unified_checks()
        print('PASS floor ROS: RANSAC floor topic/header/stability, floor gravity 3D fall, no stale gravity, '
              'unified measured-floor ankles, exact-stamp matching')
    finally:
        rclpy.shutdown()


if __name__=='__main__':main()
