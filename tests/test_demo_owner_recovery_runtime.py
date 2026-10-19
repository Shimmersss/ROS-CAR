"""Demo owner policy on real ROS nodes with synthetic detections/ReID; no hardware/motion."""
import copy
import time
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo
from person_interfaces.msg import PersonIdentityArray, PersonIdentity
from person_interfaces.srv import ClaimOwner
from std_srvs.srv import Trigger
from yolo_person_tracker.pose_node import PoseTrackerNode
from yolo_person_tracker.backend import Detection
from monotonic_ros_clock import MonotonicRosClock


class Backend:
    detections = [Detection(7, (10, 5, 70, 95), .9)]
    def infer(self, image): return self.detections
    def reset(self): pass


class Tracker:
    score = .9
    def init(self, image, box): self.box = box
    def update(self, image): return self.box, self.score


rclpy.init(); backend=Backend(); tracker=Tracker()
node=PoseTrackerNode(backend=backend,sot_tracker=tracker,parameter_overrides=[Parameter(k,value=v) for k,v in dict(
    use_sim_time=True,depth_registered=True,sot_enabled=True,reid_reacquire_enabled=True,
    reid_reacquire_only_after_loss=True,reid_reacquire_confirm_frames=3,reid_reacquire_min_s=.4,
    auto_lock_single=False,owner_continuity_reset_s=2.,owner_lost_timeout_s=5.,visualization_fps=0.).items()])
clock=MonotonicRosClock(node)
info=CameraInfo(width=200,height=100);info.header.frame_id='camera_optical'
info.p=[100.,0.,100.,0.,0.,100.,50.,0.,0.,0.,1.,0.];node.on_info(info)


def step(identity=None, delay=.03):
    clock.tick()
    color=node.bridge.cv2_to_imgmsg(np.zeros((100,200,3),np.uint8),'bgr8')
    color.header.frame_id=info.header.frame_id;color.header.stamp=node.get_clock().now().to_msg()
    depth=node.bridge.cv2_to_imgmsg(np.full((100,200),2.,np.float32),'32FC1');depth.header=copy.deepcopy(color.header)
    node.on_pair(color,depth)
    end=time.monotonic()+2
    while node.future is not None and not node.future.done() and time.monotonic()<end:time.sleep(.001)
    clock.tick();node.tick()
    if identity and node.snapshot:
        node.on_identities(PersonIdentityArray(header=copy.deepcopy(node.snapshot[0].header),enabled=True,valid=True,
            persons=[PersonIdentity(track_id=f'0:{backend.detections[0].track_id}',person_id=identity,verified=True,visible=True)]))
        clock.tick();node.tick()
    time.sleep(delay)


def claim():
    response=node.claim_owner(ClaimOwner.Request(),ClaimOwner.Response())
    assert response.success,response.message


try:
    for _ in range(4):step()
    assert node.owner.state=='UNCLAIMED' and node.selection.target_id is None
    claim()
    for _ in range(3):step('alice',.21)
    assert node.selection.owner_id=='alice'
    for _ in range(4):step('bob')
    assert node.selection.target_id==(0,7) and node.owner.state=='TRACKING', 'normal following must ignore appearance jitter'
    backend.detections=[];tracker.score=0.;step();lost_at=node.owner.lost_at
    while not node.owner.continuity_reset_done and time.monotonic()-lost_at<2.5:step()
    assert node.selection.searching and node.selection.owner_id=='alice' and node.selection.target_id is None
    # Return across the image, outside the previous local tracker box.
    backend.detections=[Detection(9,(120,5,180,95),.9)]
    for _ in range(3):step('bob')
    assert node.selection.target_id is None
    for i in range(3):
        step('alice',.21)
        assert node.selection.target_id==((0,9) if i==2 else None)
    assert node.owner.state=='TRACKING' and node.owner.event=='returned'
    backend.detections=[];tracker.score=0.;step();lost_at=node.owner.lost_at
    while node.owner.state!='UNCLAIMED' and time.monotonic()-lost_at<5.6:step()
    assert node.owner.state=='UNCLAIMED' and not node.selection.owner_id
    backend.detections=[Detection(10,(10,5,70,95),.9)]
    for _ in range(8):step('stranger')
    assert node.owner.state=='UNCLAIMED' and node.selection.target_id is None, 'expired owner must not auto-claim a stranger'
    tracker.score=.9;claim();step()
    assert node.selection.target_id==(0,10)
    assert node.release_target(Trigger.Request(),Trigger.Response()).success
    assert not node.selection.owner_id and not node.selection.searching
    assert node.count_publishers('/cmd_vel')==0
    print('PASS demo recovery-only: explicit claim, ReID noise cannot interrupt following, cross-image same-owner return after 3 fresh frames/0.4s, stranger rejected, real 2/5s lifecycle, no auto-claim after expiry, release clears owner; no motion publisher')
finally:
    node.destroy_node();rclpy.shutdown()
