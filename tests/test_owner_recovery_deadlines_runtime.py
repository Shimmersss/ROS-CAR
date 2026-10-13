"""Real 2/5 s owner deadlines with synthetic RGB-D/ReID; no identity accuracy or motion."""
import copy
import time
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import Trigger
from person_interfaces.msg import PersonIdentityArray, PersonIdentity
from yolo_person_tracker.pose_node import PoseTrackerNode
from yolo_person_tracker.reid_node import ReIDNode
from yolo_person_tracker.backend import Detection

class Backend:
    detections = [Detection(7, (10, 5, 70, 95), .9)]
    def infer(self, image): return self.detections
    def reset(self): pass
class Tracker:
    score = .9
    def init(self, image, box): self.box = box
    def update(self, image): return self.box, self.score

def main():
    rclpy.init(); b=Backend(); tracker=Tracker()
    n=PoseTrackerNode(backend=b,sot_tracker=tracker,parameter_overrides=[Parameter(k,value=v) for k,v in dict(
        depth_registered=True,sot_enabled=True,reid_reacquire_enabled=True,
        owner_lost_timeout_s=5.,owner_continuity_reset_s=2.,auto_lock_single=True,visualization_fps=0.).items()])
    observer=ReIDNode(backend=object(),parameter_overrides=[Parameter('forget_released_owner',value=True)])
    info=CameraInfo(); info.width=info.height=100;info.header.frame_id='camera_optical'
    info.p=[100.,0.,50.,0.,0.,100.,50.,0.,0.,0.,1.,0.];n.on_info(info)
    def step(identity=None):
        c=n.bridge.cv2_to_imgmsg(np.zeros((100,100,3),np.uint8),'bgr8');c.header.frame_id=info.header.frame_id;c.header.stamp=n.get_clock().now().to_msg()
        d=n.bridge.cv2_to_imgmsg(np.full((100,100),2.,np.float32),'32FC1');d.header=copy.deepcopy(c.header)
        n.on_pair(c,d)
        end=time.monotonic()+2
        while n.future is not None and not n.future.done() and time.monotonic()<end:time.sleep(.001)
        n.tick()
        if identity and n.snapshot:
            header=copy.deepcopy(n.snapshot[0].header)
            n.on_identities(PersonIdentityArray(header=header,enabled=True,valid=True,persons=[
                PersonIdentity(track_id=f'0:{b.detections[0].track_id}',person_id=identity,verified=True,visible=True)]))
            n.tick()
        time.sleep(.03)
    try:
        for _ in range(4):step()
        assert n.owner.state=='TRACKING'; n.selection.owner_id='alice'
        b.detections=[];tracker.score=0.;step()
        assert n.owner.state=='LOST';start=n.owner.lost_at; generation=n.selection_generation
        while time.monotonic()-start<1.85:step()
        assert n.selection.target_id is not None and not n.owner.continuity_reset_done
        while not n.owner.continuity_reset_done and time.monotonic()-start<2.5:step()
        assert n.selection.target_id is None and n.selection.owner_id=='alice' and n.selection.searching
        assert n.selection_generation==generation+1 and n.owner.lost_at==start
        # A new track cannot auto-claim while the original owner is reserved.
        b.detections=[Detection(8,(10,5,70,95),.9)];tracker.score=.9
        for _ in range(4):step()
        assert n.owner.state=='LOST' and n.selection.target_id is None
        step('alice');step('alice')
        assert n.owner.state=='TRACKING' and n.owner.event=='returned' and n.selection.target_id==(0,8)
        # Second loss: no owner match, original five-second deadline wins.
        b.detections=[];tracker.score=0.;step(); start=n.owner.lost_at
        while time.monotonic()-start<4.85:step()
        assert n.owner.state=='LOST' and n.selection.owner_id=='alice'
        while n.owner.state!='UNCLAIMED' and time.monotonic()-start<5.5:step()
        assert n.owner.event=='expired' and not n.selection.owner_id and not n.selection.frames
        assert not n.auto_lock_suppressed and n.owner.lost_at is None
        b.detections=[Detection(9,(10,5,70,95),.9)];tracker.score=.9
        step();step();assert n.owner.state=='UNCLAIMED'
        step();assert n.owner.state=='TRACKING' and n.selection.target_id==(0,9)
        # ReID observer removes the old gallery and invalidates an old worker generation.
        m=observer.manager;feature=np.array([1.,0.,0.])
        for stamp in (1.,1.2,1.4): rows=m.update(stamp,{'0:8':feature})
        old=rows[0].person_id;assert old
        observer.on_owner_focus(PersonIdentity(person_id=old,track_id='0:8'));observer.owner_hint()
        generation=observer.generation
        observer.on_owner_focus(PersonIdentity())
        assert old not in m.galleries and not m.pinned and observer.generation==generation+1
        assert n.count_publishers('/cmd_vel')==0
        print('PASS real 2/5 s: continuity-only reset, same-owner ReID return, no premature claim, expiry/clear/gallery invalidation, new single-person claim, no motion')
    finally:
        observer.destroy_node();n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
