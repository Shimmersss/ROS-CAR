"""Single-object bridge in the real TrackerNode with injected detections and a scripted tracker.
Synthetic; NOT VitTrack accuracy, camera or vehicle validation."""
import copy
import time
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from person_interfaces.msg import TargetState
from yolo_person_tracker.backend import Detection
from yolo_person_tracker.node import TrackerNode
from yolo_person_tracker.pose_node import PoseTrackerNode
from yolo_person_tracker.identity_selection import IdentitySelection
from yolo_person_tracker.owner_selection import OwnerSelection
from person_interfaces.msg import PersonIdentity

A, B = (5, 5, 45, 95), (55, 5, 95, 95)


class FakeBackend:
    def __init__(self):
        self.detections = []

    def reset(self):
        pass

    def infer(self, image):
        return list(self.detections)


class ScriptedTracker:
    def __init__(self):
        self.box = A; self.score = .9; self.inits = 0

    def init(self, image, box):
        self.inits += 1

    def update(self, image):
        x1, y1, x2, y2 = self.box
        return (x1, y1, x2-x1, y2-y1), self.score


def main():
    rclpy.init()
    backend, tracker = FakeBackend(), ScriptedTracker()
    node = TrackerNode(backend=backend, sot_tracker=tracker, parameter_overrides=[
        Parameter('depth_registered', value=True), Parameter('max_age_s', value=1.0),
        Parameter('sot_enabled', value=True), Parameter('position_hold_s', value=0.0),
        Parameter('track_hold_s', value=0.0)])
    executor = SingleThreadedExecutor(); executor.add_node(node)
    states = []
    node.create_subscription(TargetState, 'target_state', states.append, 10)
    color_pub = node.create_publisher(Image, node.cfg['color_topic'], 10)
    depth_pub = node.create_publisher(Image, node.cfg['depth_topic'], 10)
    info_pub = node.create_publisher(CameraInfo, node.cfg['camera_info_topic'], 10)

    def spin(duration=.15):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)

    def call(name):
        client = node.create_client(Trigger, name); assert client.wait_for_service(timeout_sec=1)
        future = client.call_async(Trigger.Request()); end = time.monotonic()+2
        while not future.done() and time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)
        return future.result()

    def send(dets, box=None):
        backend.detections = dets
        if box is not None:
            tracker.box = box
        info = CameraInfo(); info.width = info.height = 100; info.header.frame_id = 'camera_optical'
        info.p = [100., 0., 50., 0., 0., 100., 50., 0., 0., 0., 1., 0.]; info_pub.publish(info); spin(.03)
        color = node.bridge.cv2_to_imgmsg(np.zeros((100, 100, 3), np.uint8), 'bgr8')
        color.header.frame_id = 'camera_optical'; color.header.stamp = node.get_clock().now().to_msg()
        depth = node.bridge.cv2_to_imgmsg(np.full((100, 100), 2000, np.uint16), '16UC1')
        depth.header = copy.deepcopy(color.header)
        color_pub.publish(color); depth_pub.publish(depth); spin(.2)
        return states[-1]

    try:
        assert node.selection.nearest_reacquire is False
        both = [Detection(7, A, .9), Detection(9, B, .9)]
        send(both); assert call('lock_target').success; spin()
        assert send(both).status == TargetState.TRACKING and states[-1].target_id == '0:7'
        inits = tracker.inits
        assert send(both, A).status == TargetState.TRACKING and tracker.inits > inits  # agreement refreshes
        # ByteTrack exchanges the numbers; the bridge stays on the original person -> released, not switched.
        swapped = [Detection(7, B, .9), Detection(9, A, .9)]
        for i in range(3):
            last = send(swapped, A)
            if i < 2:  # conflict frames pause output immediately, before the release
                assert last.status == TargetState.LOST and last.detail.startswith('Paused'), last.detail
                assert not last.position_valid
        assert last.status == TargetState.SEARCHING and 'possible ID swap' in last.detail, last.detail
        assert send(swapped).status == TargetState.SEARCHING  # no automatic relock or switch
        # Fresh lock; target then goes undetected and comes back under a new track number.
        send(both, A); assert call('lock_target').success; spin(); send(both, A)
        bridged = send([Detection(9, B, .9)], A)
        assert bridged.status == TargetState.TRACKING and bridged.detail.startswith('Bridged'), bridged.detail
        assert bridged.position_valid and abs(bridged.position.z-2) < .1
        back = [Detection(9, B, .9), Detection(12, A, .9)]
        send(back, A); handed = send(back, A)
        assert handed.status == TargetState.TRACKING and handed.target_id == '0:12', handed.target_id
        # Target 12 vanishes and the bridge slides onto 9, seen beside it: never a position or a switch.
        # (Here the 3D depth gate stops the bridge on the 1 m lateral jump before the conflict count.)
        send(back, A)
        for _ in range(3):
            near = send([Detection(9, B, .9)], B)
            assert near.target_id != '0:9' and not near.position_valid, near
        assert near.status != TargetState.TRACKING, near.detail
        assert node.selection.hold_s == 0.  # bridge replaces the stale held box
        combined = PoseTrackerNode(backend=FakeBackend(), sot_tracker=ScriptedTracker(), parameter_overrides=[
            Parameter('depth_registered', value=True), Parameter('sot_enabled', value=True),
            Parameter('reid_lock_enabled', value=True)])
        assert isinstance(combined.selection, IdentitySelection) and combined.selection.carry_s == 3.
        assert combined.arbiter.cfg.mask_refresh
        combined.destroy_node()
        owner = PoseTrackerNode(backend=FakeBackend(), sot_tracker=ScriptedTracker(), parameter_overrides=[
            Parameter('depth_registered', value=True), Parameter('sot_enabled', value=True),
            Parameter('reid_reacquire_enabled', value=True)])
        assert isinstance(owner.selection, OwnerSelection) and owner.selection.hold_s == 0.
        focus = []
        owner.create_subscription(PersonIdentity, 'owner_focus', focus.append, 10)
        executor.add_node(owner); spin(.3); executor.remove_node(owner)
        assert focus and focus[-1].state == 'IDLE' and not focus[-1].person_id
        owner.destroy_node()
        try:
            PoseTrackerNode(backend=FakeBackend(), parameter_overrides=[Parameter('reid_reacquire_enabled', value=True)])
            raise AssertionError('reid_reacquire_enabled without sot_enabled must be rejected')
        except ValueError:
            pass
        print('PASS SOT bridge ROS: refresh on agreement, swap release, bridged depth, confirmed handover, '
              'no co-visible handover, no stale hold, ReID lock + bridge carry wiring, owner re-acquire wiring (synthetic)')
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
