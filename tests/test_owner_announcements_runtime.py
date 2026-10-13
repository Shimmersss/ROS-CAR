"""Real ROS services/topics and 10 s timer; synthetic RGB-D and speech acknowledgements."""
import copy
import json
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from person_interfaces.msg import OwnerState, TargetState
from person_interfaces.srv import ClaimOwner
from roscar_interfaces.srv import SayText
from yolo_person_tracker.node import TrackerNode
from yolo_person_tracker.backend import Detection
from roscar_api.announcements import PhaseAnnouncements
from roscar_api.interfaces import discover
from monotonic_ros_clock import MonotonicRosClock


class Backend:
    people = [Detection(7, (10, 5, 90, 95), .9)]
    def infer(self, image):
        return self.people
    def reset(self):
        pass


def main():
    rclpy.init()
    sim = [Parameter('use_sim_time', value=True)]
    backend = Backend()
    tracker = TrackerNode(namespace='/perception', backend=backend, parameter_overrides=sim+[
        Parameter('depth_registered', value=True), Parameter('track_hold_s', value=0.),
        Parameter('reacquire_s', value=0.), Parameter('visualization_fps', value=0.)])
    announce = PhaseAnnouncements(parameter_overrides=sim)
    io = Node('owner_test_io', parameter_overrides=sim)
    nodes = [io, tracker, announce]
    clocks = [MonotonicRosClock(n) for n in nodes]
    executor = SingleThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    owners, targets, spoken, events = [], [], [], []
    io.create_subscription(OwnerState, '/perception/owner_state', owners.append, 10)
    io.create_subscription(TargetState, '/perception/target_state', targets.append, 10)
    io.create_subscription(String, '/system/phase', lambda m: events.append(json.loads(m.data)), 10)
    def say(request, response):
        spoken.append(request.text)
        response.success, response.status = True, 'completed'
        return response
    io.create_service(SayText, '/voice/say', say)
    color = io.create_publisher(Image, tracker.cfg['color_topic'], 10)
    depth = io.create_publisher(Image, tracker.cfg['depth_topic'], 10)
    info = io.create_publisher(CameraInfo, tracker.cfg['camera_info_topic'], 10)
    control = io.create_publisher(String, '/control/state', 10)
    claim = io.create_client(ClaimOwner, '/perception/claim_owner')
    release = io.create_client(Trigger, '/perception/release_target')
    last_input = [0.]
    def step():
        clocks[0].tick()
        for clock in clocks[1:]:
            clock.clock.set_ros_time_override(clocks[0].clock.now())
        now = time.monotonic()
        if now-last_input[0] >= .08:
            last_input[0] = now
            camera = CameraInfo(width=100, height=100)
            camera.header.frame_id = 'camera_color_optical_frame'
            camera.p = [100.,0.,50.,0.,0.,100.,50.,0.,0.,0.,1.,0.]
            info.publish(camera)
            rgb = tracker.bridge.cv2_to_imgmsg(np.zeros((100,100,3), np.uint8), 'bgr8')
            rgb.header.frame_id = camera.header.frame_id
            rgb.header.stamp = io.get_clock().now().to_msg()
            dep = tracker.bridge.cv2_to_imgmsg(np.full((100,100), 2000, np.uint16), '16UC1')
            dep.header = copy.deepcopy(rgb.header)
            color.publish(rgb); depth.publish(dep)
        executor.spin_once(timeout_sec=.005)
    def wait(predicate, detail, timeout=3.):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            step()
            if predicate():
                return
        raise AssertionError(detail)
    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            step()
    def call(client, request):
        wait(client.service_is_ready, 'service unavailable')
        future = client.call_async(request)
        wait(future.done, 'service timeout')
        return future.result()
    try:
        wait(lambda: targets and targets[-1].status == TargetState.SEARCHING, 'tracker not ready')
        assert tracker.owner.timeout_s == 10.
        assert not call(claim, ClaimOwner.Request(target_id='bad')).success
        assert not call(claim, ClaimOwner.Request(target_id='99:7')).success
        assert not call(claim, ClaimOwner.Request(target_id='0:999')).success
        assert call(claim, ClaimOwner.Request(target_id='0:7')).success
        wait(lambda: '已认领主人' in spoken, 'claim not announced')
        control.publish(String(data=json.dumps(dict(command_mode='FOLLOW', authority_state='ARMED_FOLLOW'))))
        wait(lambda: '开始跟随' in spoken, 'follow not announced')
        backend.people = []
        wait(lambda: owners and owners[-1].state == 'LOST', 'loss not published')
        wait(lambda: '主人丢失，等待返回' in spoken, 'loss not announced')
        spin(.2)
        backend.people = [Detection(7, (10,5,90,95), .9)]
        wait(lambda: '主人已返回' in spoken, 'return within deadline not announced')
        control.publish(String(data=json.dumps(dict(command_mode='FOLLOW', authority_state='FAULT', last_fault='target_invalid'))))
        spin(.1)
        control.publish(String(data=json.dumps(dict(command_mode='FOLLOW', authority_state='ARMED_FOLLOW'))))
        spin(.1)
        assert spoken.count('开始跟随') == 1
        # Another visible person cannot replace the owner during the full real 10 s window.
        tracker.cfg['auto_lock_single'] = True
        backend.people = [Detection(9, (10,5,90,95), .9)]
        wait(lambda: tracker.owner.state == 'LOST', 'second loss missing')
        lost_at = tracker.owner.lost_at
        wait(lambda: time.monotonic()-lost_at >= 9.8, 'clock stalled', timeout=11.)
        assert tracker.owner.state == 'LOST' and tracker.selection.target_id == (0,7)
        wait(lambda: owners[-1].event == 'expired', 'owner did not expire', timeout=1.)
        assert time.monotonic()-lost_at >= 10.
        assert owners[-1].can_claim and not owners[-1].target_id and not owners[-1].person_id
        assert tracker.selection.target_id is None and not targets[-1].position_valid
        wait(lambda: '主人未返回，可重新认领' in spoken, 'expiry not announced')
        # The explicitly enabled single-person policy must resume only after expiry.
        wait(lambda: owners[-1].target_id == '0:9' and owners[-1].state == 'TRACKING', 'new owner not selected')
        wait(lambda: spoken.count('已认领主人') == 2, 'new owner not announced')
        assert call(release, Trigger.Request()).success
        wait(lambda: owners[-1].can_claim and '已解除认领' in spoken, 'release not exposed')
        spin(.5)
        assert owners[-1].can_claim and tracker.selection.target_id is None, 'manual release must suppress auto claim'
        graph = discover(io)
        assert graph['services']['/perception/claim_owner'] == ['person_interfaces/srv/ClaimOwner']
        assert graph['topics']['/perception/owner_state'] == ['person_interfaces/msg/OwnerState']
        assert spoken.count('主人未返回，可重新认领') == 1
        assert spoken.count('主人已返回') == 1
        assert not any(name == '/cmd_vel' for name in graph['topics'])
        print('PASS real ROS: explicit claim, return, full 10 s expiry, new claim, release, SayText dedup, API discovery')
    finally:
        executor.shutdown()
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
