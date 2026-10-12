"""Synthetic ROS C integration; does not claim real camera/person fall acceptance."""
import copy
import sys
import time
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray, Marker
from person_interfaces.msg import PersonStateArray, TargetState, RuntimeMetrics
from yolo_person_tracker.backend import Detection
from yolo_person_tracker.pose_node import PoseTrackerNode


class Backend:
    def __init__(self):
        p=np.full((17,3),30.,float);p[:,2]=.9
        p[5,:2]=(40,20);p[6,:2]=(60,20);p[11,:2]=(40,50);p[12,:2]=(60,50)
        p[13,:2]=(40,80);p[14,:2]=(60,80)
        self.detections=[Detection(7,(10,5,90,95),.9,tuple(map(tuple,p)))]
        self.fail=False
    def reset(self):pass
    def infer(self,image):
        if self.fail:raise RuntimeError('synthetic pose inference failure')
        return self.detections


def main():
    fusion_enabled = '--fusion-off' not in sys.argv
    rclpy.init();backend=Backend()
    node=PoseTrackerNode(backend=backend,namespace='perception',parameter_overrides=[
        Parameter('depth_registered',value=True),Parameter('max_age_s',value=1.),
        Parameter('pose_upright_confirmed',value=True),Parameter('visualization_scale',value=1.),
        Parameter('fusion_enabled',value=fusion_enabled)])
    ex=SingleThreadedExecutor();ex.add_node(node)
    persons=[];states=[];markers=[];metrics=[]
    node.create_subscription(PersonStateArray,'person_states',persons.append,10)
    node.create_subscription(TargetState,'target_state',states.append,10)
    node.create_subscription(MarkerArray,'skeleton_markers',markers.append,10)
    node.create_subscription(RuntimeMetrics,'performance',metrics.append,10)
    cp=node.create_publisher(Image,node.cfg['color_topic'],10)
    dp=node.create_publisher(Image,node.cfg['depth_topic'],10)
    ip=node.create_publisher(CameraInfo,node.cfg['camera_info_topic'],10)
    def spin(seconds=.2):
        end=time.monotonic()+seconds
        while time.monotonic()<end:ex.spin_once(timeout_sec=.01)
    def send(value=2000):
        info=CameraInfo();info.width=info.height=100;info.header.frame_id='camera_color_optical_frame'
        info.p=[100.,0.,50.,0.,0.,100.,50.,0.,0.,0.,1.,0.];ip.publish(info);spin(.03)
        c=node.bridge.cv2_to_imgmsg(np.zeros((100,100,3),np.uint8),'bgr8');c.header.frame_id=info.header.frame_id;c.header.stamp=node.get_clock().now().to_msg()
        pixels = value if isinstance(value, np.ndarray) else np.full((100,100),value,np.uint16)
        d=node.bridge.cv2_to_imgmsg(pixels,'16UC1');d.header=copy.deepcopy(c.header)
        cp.publish(c);dp.publish(d);spin();return c
    def call(name):
        client=node.create_client(Trigger,name);assert client.wait_for_service(timeout_sec=1)
        future=client.call_async(Trigger.Request());end=time.monotonic()+2
        while not future.done() and time.monotonic()<end:ex.spin_once(timeout_sec=.01)
        assert future.done();response=future.result();node.destroy_client(client);return response
    try:
        spin(.3);c=send()
        assert persons[-1].valid and persons[-1].header==c.header
        p=persons[-1].persons[0];assert p.track_id=='0:7' and p.posture==p.STANDING
        assert p.body_depth_valid and abs(p.body_depth_m-2.)<1e-6
        assert all(p.keypoints_3d_valid) and p.keypoints_3d[5].z==2.
        assert markers[-1].markers[0].action==Marker.ADD
        assert call('lock_target').success;spin();assert states[-1].position_valid
        if fusion_enabled:
            # Uncorroborated lower-body depth 1 m nearer: gated, withheld, never the target.
            send();legs=np.zeros((100,100),np.uint16)
            legs[77:84,37:44]=1000;legs[77:84,57:64]=1000
            send(legs);p=persons[-1].persons[0]
            assert 'depth=pose_lower_body' in p.detail and 'withheld by temporal depth gate' in p.detail, p.detail
            assert not p.body_depth_valid and np.isnan(p.body_depth_m)
            assert not states[-1].position_valid or abs(states[-1].position.z-2.)<.1, states[-1]
            sparse=np.zeros((100,100),np.uint16)
            sparse[17:24,37:44]=2000;sparse[17:24,57:64]=2000
            c=send(sparse)
            p=persons[-1].persons[0]
            assert states[-1].position_valid, states[-1]
            assert states[-1].observation_stamp==c.header.stamp, states[-1]
            assert sum(p.keypoints_3d_valid)==2 and 'depth=pose_anchors' in p.detail
            assert not p.keypoints_3d_valid[11] and np.isnan(p.keypoints_3d[11].z)
            assert persons[-1].header==c.header
            send(0)
            assert not any(persons[-1].persons[0].keypoints_3d_valid)
            assert not persons[-1].persons[0].body_depth_valid and np.isnan(persons[-1].persons[0].body_depth_m)
            assert 'depth=invalid' in persons[-1].persons[0].detail
            # The original short position hold must expire, never refresh on holes.
            spin(.35);assert not states[-1].position_valid
            # Box-region depth keeps an existing track valid without keypoints; it may
            # not start a track alone (unit-tested), so refresh the track first.
            send()
        backend.detections=[Detection(7,(10,5,90,95),.9,())];send()
        assert persons[-1].persons[0].posture==0 and states[-1].position_valid
        assert all(not x for x in persons[-1].persons[0].keypoints_3d_valid)
        assert any(m.action==Marker.DELETE for a in markers for m in a.markers)
        assert call('release_target').success
        backend.detections=Backend().detections;send(0)
        assert persons[-1].valid and not any(persons[-1].persons[0].keypoints_3d_valid)
        spin(1.2);assert not persons[-1].valid and not node.postures.tracks
        assert metrics and metrics[-1].source=='yolo_pose'
        send();old_epoch=node.selection.epoch;backend.fail=True;send();assert not persons[-1].valid
        backend.fail=False;send();assert node.selection.epoch>old_epoch
        assert persons[-1].persons[0].track_id==f'{node.selection.epoch}:7'
        assert node.cfg['model_task']=='pose'
        assert not any(name=='/cmd_vel' for name,_ in node.get_topic_names_and_types())
        print(f'C ROS fusion={fusion_enabled}/pose/3D/invalid/stale/epoch/lock/metrics integration PASS (synthetic inputs)')
    finally:
        ex.shutdown();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
