"""Continuous camera frames during a blocked worker preserve identity; real gaps reset it."""
import copy
import threading
import time
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo
from person_interfaces.msg import TargetState
from std_srvs.srv import Trigger
from yolo_person_tracker.node import TrackerNode
from yolo_person_tracker.backend import Detection

class Backend:
    def __init__(self):
        self.gate=threading.Event();self.gate.set();self.entered=threading.Event()
    def infer(self,image):
        self.entered.set();assert self.gate.wait(3)
        return [Detection(7,(0,0,100,100),.9)]
    def reset(self):pass

def main():
    rclpy.init();b=Backend();n=TrackerNode(backend=b,parameter_overrides=[
        Parameter('depth_registered',value=True),Parameter('max_age_s',value=.3),
        Parameter('visualization_fps',value=0.)])
    info=CameraInfo();info.width=info.height=100;info.header.frame_id='camera_optical'
    info.p=[100.,0.,50.,0.,0.,100.,50.,0.,0.,0.,1.,0.];n.on_info(info)
    def send():
        c=n.bridge.cv2_to_imgmsg(np.zeros((100,100,3),np.uint8),'bgr8')
        c.header.frame_id='camera_optical';c.header.stamp=n.get_clock().now().to_msg()
        d=n.bridge.cv2_to_imgmsg(np.ones((100,100),np.float32),'32FC1');d.header=copy.deepcopy(c.header)
        n.on_pair(c,d)
    def finish():
        deadline=time.monotonic()+2
        while n.future is not None and not n.future.done() and time.monotonic()<deadline:time.sleep(.005)
        assert n.future is None or n.future.done();n.tick()
    try:
        send();finish();assert n.lock_target(Trigger.Request(),Trigger.Response()).success
        epoch=n.selection.epoch;target=n.selection.target_id
        b.gate.clear();b.entered.clear();send();assert b.entered.wait(1)
        for _ in range(12):time.sleep(.05);send();n.tick()
        assert not getattr(n, "stream_reset_pending", False) and n.selection.epoch==epoch
        b.gate.set();finish()
        assert not n.fresh_snapshot(), 'old worker result must remain stale'
        send();finish()
        assert n.selection.epoch==epoch and n.selection.target_id==target and n.selection.selected() is not None
        # An actual camera interruption still invalidates the old epoch.
        time.sleep(.35);send();finish()
        assert n.selection.epoch==epoch+1 and n.selection.selected() is None
        # A discontinuity noticed during inference must discard that old result.
        b.gate.clear();b.entered.clear();send();assert b.entered.wait(1)
        time.sleep(.35);send();assert n.stream_reset_pending
        b.gate.set();finish();assert n.snapshot is None
        send();finish();assert n.selection.epoch==epoch+2
        print('PASS continuous skipped RGB-D preserves identity; stale result invalid; genuine gaps/reset during worker withdraw old result')
    finally:
        b.gate.set();n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
