"""Synthetic ROS identity-lock contract; not real-person ReID accuracy acceptance."""
import copy
import time
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from person_interfaces.msg import PersonIdentity, PersonIdentityArray, TargetState
from yolo_person_tracker.pose_node import PoseTrackerNode
from yolo_person_tracker.backend import Detection
from test_pose_runtime import Backend


def main():
    rclpy.init(); backend=Backend()
    node=PoseTrackerNode(backend=backend, namespace='perception', parameter_overrides=[
        Parameter('depth_registered', value=True), Parameter('max_age_s', value=1.),
        Parameter('reid_lock_enabled', value=True), Parameter('auto_lock_single', value=True)])
    executor=SingleThreadedExecutor();executor.add_node(node)
    states=[]
    node.create_subscription(TargetState, 'target_state', states.append, 10)
    cp=node.create_publisher(Image,node.cfg['color_topic'],10)
    dp=node.create_publisher(Image,node.cfg['depth_topic'],10)
    ip=node.create_publisher(CameraInfo,node.cfg['camera_info_topic'],10)
    rp=node.create_publisher(PersonIdentityArray,'person_identities',10)
    keypoints=backend.detections[0].keypoints
    def spin(seconds=.09):
        end=time.monotonic()+seconds
        while time.monotonic()<end:executor.spin_once(timeout_sec=.005)
    def send(track=7, depth=2000, person='alice', verified=True, evidence=True):
        backend.detections=[] if track is None else [Detection(track,(10,5,90,95),.9,keypoints)]
        info=CameraInfo();info.width=info.height=100;info.header.frame_id='camera_color_optical_frame'
        info.p=[100.,0.,50.,0.,0.,100.,50.,0.,0.,0.,1.,0.];ip.publish(info);spin(.02)
        c=node.bridge.cv2_to_imgmsg(np.zeros((100,100,3),np.uint8),'bgr8')
        c.header.frame_id=info.header.frame_id;c.header.stamp=node.get_clock().now().to_msg()
        d=node.bridge.cv2_to_imgmsg(np.full((100,100),depth,np.uint16),'16UC1');d.header=copy.deepcopy(c.header)
        cp.publish(c);dp.publish(d)
        stamp_ns=c.header.stamp.sec*1000000000+c.header.stamp.nanosec
        end=time.monotonic()+.7
        while stamp_ns not in node.selection.frames and time.monotonic()<end:
            executor.spin_once(timeout_sec=.005)
        assert stamp_ns in node.selection.frames, 'Pose frame not processed within test deadline'
        msg=PersonIdentityArray(header=copy.deepcopy(c.header),enabled=True,valid=True)
        if track is not None:
            msg.persons=[PersonIdentity(track_id=f'{node.selection.epoch}:{track}',person_id=person,
                                       verified=verified,visible=True)]
        if evidence:
            rp.publish(msg)
            end=time.monotonic()+.3
            while node.selection.last_evidence < stamp_ns and time.monotonic()<end:
                executor.spin_once(timeout_sec=.005)
            spin(.06)
        print('lock probe', track, person, node.selection.epoch, node.selection.count,
              node.selection.pending, states[-1].target_id, states[-1].position_valid, flush=True)
        return msg
    def call(name):
        client=node.create_client(Trigger,name);assert client.wait_for_service(timeout_sec=1)
        future=client.call_async(Trigger.Request());end=time.monotonic()+2
        while not future.done() and time.monotonic()<end:executor.spin_once(timeout_sec=.01)
        assert future.done() and future.result().success
        node.destroy_client(client);spin()
    def confirm(track, depth=2000, person='alice'):
        for _ in range(3):send(track,depth,person)
    try:
        spin(.3);send(evidence=False);call('lock_target')
        assert not states[-1].position_valid
        confirm(7)
        assert states[-1].position_valid and states[-1].target_id=='0:7'
        assert node.selection.person_id=='alice'
        send(None)
        assert states[-1].status==TargetState.LOST and not states[-1].position_valid
        send(8)
        assert node.selection.target_id==(0,7) and not states[-1].position_valid
        send(8);send(8,3000)
        assert states[-1].position_valid and states[-1].target_id=='0:8'
        # Contradicting the same ByteTrack ID revokes position immediately.
        send(8,person='bob')
        assert not states[-1].position_valid
        confirm(8)
        assert states[-1].position_valid
        # No inherited depth when the new person's current depth is unavailable.
        send(None);confirm(9,0)
        assert states[-1].target_id=='0:9' and not states[-1].position_valid
        send(9);assert states[-1].position_valid
        send(9,verified=False);assert not states[-1].position_valid
        confirm(9);assert states[-1].position_valid
        spin(.55);assert not states[-1].position_valid
        # Recovery needs three distinct fresh observations again.
        send(9);assert not states[-1].position_valid
        send(9);send(9);assert states[-1].position_valid
        call('release_target');confirm(10)
        assert states[-1].status==TargetState.SEARCHING and not states[-1].position_valid
        call('lock_target');confirm(10,person='bob')
        assert node.selection.person_id=='bob' and states[-1].position_valid
        # Stream gap advances epoch; retained identity may recover only with new evidence.
        spin(1.1);send(11,person='bob')
        assert not states[-1].position_valid and node.selection.epoch>0
        send(11,person='bob');send(11,person='bob')
        assert states[-1].position_valid and states[-1].target_id.endswith(':11'), (states[-1], vars(node.selection))
        assert not any(n=='/cmd_vel' for n,_ in node.get_topic_names_and_types())
        print('C identity lock ROS PASS: confirmation/change/contradiction/depth/age/release/epoch (synthetic)')
    finally:
        executor.shutdown();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
