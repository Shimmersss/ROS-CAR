"""C pose node: a person lost mid-fall is measured in place on person_states_continued
(height_continue_s), confirmed there, never shown on person_states, and handed back to the
returning detection; fall_alarm summarises both topics per frame without a second rising edge.
Synthetic slab depth with static geometry; no physical accuracy claim."""
import copy
from types import SimpleNamespace
import time
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image
from person_interfaces.msg import FallAlarm, PersonStateArray, PersonState
from yolo_person_tracker.backend import Detection
from yolo_person_tracker.pose_node import PoseTrackerNode
from monotonic_ros_clock import MonotonicRosClock

F, CX, CY, Z = 300., 320., 240., 2.     # camera 1 m above the floor, level, person 2 m ahead


class Backend:
    detections=[]
    def infer(self,image):return self.detections
    def reset(self):pass


def v_of(height):
    return CY+F*(1.-height)/Z


def slab(top):
    """Depth (mm) of a 0.5 m wide fronto-parallel body from the floor up to `top` metres."""
    depth=np.zeros((480,640),np.uint16)
    depth[int(v_of(top)):int(v_of(0.)),int(CX-.25*F/Z):int(CX+.25*F/Z)]=int(Z*1000)
    return depth


def box(top):
    return (CX-.3*F/Z,v_of(top)-5,CX+.3*F/Z,v_of(0.)+5)


def make_node(namespace,continue_s,confirm_max=0.,ground=True):
    return PoseTrackerNode(backend=Backend(),namespace=namespace,parameter_overrides=[
        Parameter('use_sim_time',value=True),Parameter('depth_registered',value=True),Parameter('max_age_s',value=1.),
        Parameter('height_fall_enabled',value=True),Parameter('height_confirm_max_m',value=confirm_max),Parameter('height_continue_s',value=continue_s),
        Parameter('pose_handover_s',value=1.5),Parameter('pose3d_up_source',value='static'),
        Parameter('pose3d_up_x',value=0.),Parameter('pose3d_up_y',value=-1.),Parameter('pose3d_up_z',value=0.),
        Parameter('pose3d_camera_height_m',value=1.),
        Parameter('pose3d_gravity_confirmed',value=True),Parameter('pose3d_ground_confirmed',value=ground)])


def continuation_checks():
    node=make_node('continue_pose',6.)
    ex=SingleThreadedExecutor();ex.add_node(node);people=[];continued=[];alarms=[]
    clock=MonotonicRosClock(node)
    node.create_subscription(PersonStateArray,'person_states',people.append,10)
    node.create_subscription(PersonStateArray,'person_states_continued',continued.append,10)
    node.create_subscription(FallAlarm,'fall_alarm',alarms.append,10)
    cp=node.create_publisher(Image,node.cfg['color_topic'],10)
    dp=node.create_publisher(Image,node.cfg['depth_topic'],10)
    ip=node.create_publisher(CameraInfo,node.cfg['camera_info_topic'],10)
    def spin(seconds=.1):
        end=time.monotonic()+seconds
        while time.monotonic()<end:clock.spin_once(ex)
    def send(top,detections):
        Backend.detections=detections
        info=CameraInfo();info.width=640;info.height=480;info.header.frame_id='camera_color_optical_frame'
        info.p=[F,0.,CX,0.,0.,F,CY,0.,0.,0.,1.,0.];ip.publish(info);spin(.01)
        c=node.bridge.cv2_to_imgmsg(np.zeros((480,640,3),np.uint8),'bgr8');c.header.frame_id=info.header.frame_id
        c.header.stamp=node.get_clock().now().to_msg()
        d=node.bridge.cv2_to_imgmsg(slab(top),'16UC1');d.header=copy.deepcopy(c.header);cp.publish(c);dp.publish(d)
        end=time.monotonic()+.7
        while time.monotonic()<end and not (people and people[-1].header==c.header and continued
                                            and continued[-1].header==c.header and alarms and alarms[-1].header==c.header):
            clock.spin_once(ex)
        assert people and people[-1].header==c.header and continued and continued[-1].header==c.header
        assert alarms and alarms[-1].header==c.header and alarms[-1].valid,alarms[-1:]
        return people[-1],continued[-1]
    keypoints=tuple((0.,0.,0.) for _ in range(17))   # no skeleton: the height cue alone
    person=lambda ident,top:Detection(ident,box(top),.9,keypoints)
    def stand(ident):
        start=time.monotonic()                           # upright baseline needs stable_s (0.3 s)
        while time.monotonic()-start<.6:p,c=send(1.6,[person(ident,1.6)])
        return p,c
    try:
        spin(.3)
        p,c=stand(7)
        assert len(p.persons)==1 and 'height_fall=0' in p.persons[0].detail and not c.persons,(p,c)
        a=alarms[-1]
        assert a.stage==a.NONE and a.track_id=='' and a.geometry_confirmed and a.detail=='no fall',a
        p,c=send(.7,[person(7,.7)])                      # mid-drop: still above the low threshold
        assert p.persons[0].fall_stage==0 and not c.persons,(p,c)
        # Lost while lying (feet towards a low camera): measured in place on its own topic only.
        p,c=send(.3,[])
        assert not p.persons and len(c.persons)==1,(p,c)
        lost=c.persons[0]
        assert lost.track_id.endswith(':7') and lost.fall_stage==lost.FALL_SUSPECTED and lost.confidence==0.,lost
        assert 'continued (no detection)' in lost.detail and not any(lost.keypoints_3d_valid),lost
        a=alarms[-1]
        assert a.stage==a.SUSPECTED and a.continued and a.track_id==lost.track_id and a.suspected_count==1,a
        first_alarm=len(alarms)
        start=time.monotonic()
        for _ in range(40):                                   # confirm_s of sustained low top
            p,c=send(.3,[])
            assert not p.persons and len(c.persons)==1,(p,c)
            if c.persons[0].fall_stage==2 or time.monotonic()-start>3.:break
        lost=c.persons[0]
        assert lost.fall_stage==lost.FALL_CONFIRMED and lost.posture==lost.FALLEN,lost
        assert time.monotonic()-start>=.9,'confirmation needs ~1 s of continued low evidence'
        # The person is detected again under a new ID: the confirmed fall moves back to person_states.
        p,c=send(.3,[person(9,.3)])
        assert not c.persons and len(p.persons)==1,(p,c)
        back=p.persons[0]
        assert back.track_id.endswith(':9') and back.fall_stage==back.FALL_CONFIRMED,back
        a=alarms[-1]
        assert a.stage==a.CONFIRMED and not a.continued and a.track_id==back.track_id and a.confirmed_count==1,a
        stages=[x.stage for x in alarms[first_alarm-1:]]
        assert all(b>=a_ for a_,b in zip(stages,stages[1:])),f'alarm must not drop and rise again: {stages}'
        # Something taller than a lying body in the last box (wall, legs): unmeasured, not a fall.
        stand(11)
        send(.7,[person(11,.7)])
        p,c=send(1.6,[])
        assert len(c.persons)==1 and c.persons[0].fall_stage==0 and 'unmeasured' in c.persons[0].detail,c
        assert not any(n=='/cmd_vel' for n,_ in node.get_topic_names_and_types())
    finally:
        ex.shutdown();node.destroy_node()


def default_checks():
    node=make_node('continue_off',0.)
    try:
        assert node.continued_pub is None
        assert not node.get_publishers_info_by_topic('/continue_off/person_states_continued')
        assert node.get_publishers_info_by_topic('/continue_off/fall_alarm')   # skeleton falls too
    finally:
        node.destroy_node()
    try:
        PoseTrackerNode(backend=Backend(),namespace='continue_bad',parameter_overrides=[
            Parameter('height_continue_s',value=6.)])
    except ValueError as error:
        assert 'height_fall_enabled' in str(error),error
    else:
        raise AssertionError('height_continue_s without height_fall_enabled must be rejected')


def confirmation_parameter_checks():
    # Actual ROS parameter wiring + node depth measurement and geometry cap. Synthetic
    # slab input, no camera, detector, vehicle or physical confidence claim.
    for ceiling, ground, expected in ((0.,True,0),(.75,True,2),(.75,False,1)):
        node=make_node('confirmation_check',0.,ceiling,ground)
        try:
            assert node.height_tracker.cfg.confirm_max_m==ceiling
            for stamp,top in ((1.,1.4),(1.2,1.4),(1.4,1.4),(1.8,.4),
                              (2.,.7),(2.3,.7),(2.6,.7),(2.9,.7)):
                node.snapshot=(SimpleNamespace(width=640),slab(top).astype(float)*.001,None,(F,F,CX,CY))
                person=PersonState()
                node.apply_height_fall(person,'0:1',stamp,box(top),None)
            assert person.fall_stage==expected,person
        finally:
            node.destroy_node()
    print('PASS confirmation hysteresis ROS parameters/depth path: default unchanged, explicit limit, ground confirmation cap')


def main():
    rclpy.init()
    try:
        default_checks()
        confirmation_parameter_checks()
        continuation_checks()
        print('PASS height continuation ROS: lost-mid-fall person measured in place on person_states_continued, '
              'confirmed there, absent from person_states, handed back to the returning ID; fall_alarm continuous across handover; off by default')
    finally:
        rclpy.shutdown()


if __name__=='__main__':main()
