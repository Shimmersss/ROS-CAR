"""Synthetic RGB-D and injected detections; this is NOT camera/model validation."""
import copy
import json
import subprocess
import sys
import tempfile
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger
from person_interfaces.msg import TargetState
from yolo_person_tracker.backend import Detection
from yolo_person_tracker.node import TrackerNode
from yolo_person_tracker.input_probe import InputProbe
from yolo_person_tracker.target_tf import TargetTransform


class FakeBackend:
    def __init__(self):
        self.detections = [Detection(7, (10, 5, 90, 95), .9)]
        self.fail = False
        self.resets = 0

    def reset(self):
        self.resets += 1

    def infer(self, image):
        if self.fail:
            raise RuntimeError('intentional test failure')
        return self.detections


def main():
    rclpy.init()
    backend = FakeBackend()
    node = TrackerNode(backend=backend, parameter_overrides=[
        Parameter('depth_registered', value=True), Parameter('max_age_s', value=1.0)])
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    probe = InputProbe(node.cfg['color_topic'], node.cfg['depth_topic'],
                       node.cfg['camera_info_topic'], max_age=1.)
    executor.add_node(probe)
    transform_node = TargetTransform()
    executor.add_node(transform_node)
    base_states = []
    node.create_subscription(TargetState, 'target_state_base', base_states.append, 10)
    states = []
    markers = []
    images = []
    node.create_subscription(Image, 'detections_image', images.append, 10)
    node.create_subscription(TargetState, 'target_state', states.append, 10)
    node.create_subscription(Marker, 'target_marker', markers.append, 10)
    color_pub = node.create_publisher(Image, node.cfg['color_topic'],10)
    depth_pub = node.create_publisher(Image, node.cfg['depth_topic'],10)
    info_pub = node.create_publisher(CameraInfo, node.cfg['camera_info_topic'],10)

    def spin(duration=.15):
        end = time.monotonic()+duration
        while time.monotonic()<end:
            executor.spin_once(timeout_sec=.01)

    clients = {name: node.create_client(Trigger, name) for name in ('lock_target', 'release_target')}

    def call(name):
        assert clients[name].wait_for_service(timeout_sec=1)
        future = clients[name].call_async(Trigger.Request())
        end = time.monotonic()+2
        while not future.done() and time.monotonic()<end:
            executor.spin_once(timeout_sec=.01)
        assert future.done()
        return future.result()

    def send(encoding='16UC1', value=2000, frame='camera_optical', age=0, skew_ns=0):
        info = CameraInfo()
        info.width = info.height = 100
        info.header.frame_id = 'camera_optical'
        info.p = [100.,0.,50.,0.,0.,100.,50.,0.,0.,0.,1.,0.]
        info_pub.publish(info)
        spin(.03)
        color = node.bridge.cv2_to_imgmsg(np.zeros((100,100,3), np.uint8), 'bgr8')
        color.header.frame_id = 'camera_optical'
        color.header.stamp = node.get_clock().now().to_msg()
        color.header.stamp.sec -= age
        depth = node.bridge.cv2_to_imgmsg(np.full((100,100), value, np.uint16 if encoding=='16UC1' else np.float32), encoding)
        depth.header = copy.deepcopy(color.header)
        depth.header.frame_id = frame
        stamp_ns = depth.header.stamp.sec*1000000000 + depth.header.stamp.nanosec + skew_ns
        depth.header.stamp.sec, depth.header.stamp.nanosec = divmod(stamp_ns, 1000000000)
        color_pub.publish(color); depth_pub.publish(depth)
        spin(.2)
        return states[-1]

    try:
        spin(.3)
        assert states[-1].status == TargetState.STALE
        assert set(probe.report()['missing_streams']) == {'color', 'depth', 'camera_info'}
        assert not probe.report()['metadata_pass']
        assert send().status == TargetState.SEARCHING
        report = probe.report()
        assert report['metadata_pass'] and not report['registration_verified']
        assert report['metrics']['depth_valid_fraction']['mean'] == 1.
        assert not report['cmd_vel_topic_observed']
        assert call('lock_target').success
        spin()
        assert states[-1].status == TargetState.TRACKING and states[-1].position_valid
        assert abs(states[-1].position.z-2) < 1e-5
        assert base_states and not base_states[-1].position_valid
        assert 'UNCONFIRMED' in base_states[-1].detail
        assert transform_node.broadcaster is None  # No placeholder overwrites the live TF tree.
        assert markers[-1].action == Marker.ADD
        assert images and images[-1].header.frame_id == 'camera_optical'
        assert states[-1].observation_stamp.sec > 0
        assert not states[-1].is_simulated  # Injected test backend; never a deployable mode.
        assert abs(send('32FC1',1.5).position.z-1.5) < 1e-5
        backend.detections = [Detection(8,(10,5,90,95),.9)]
        assert send().status == TargetState.LOST
        assert states[-1].target_id == '0:7' and not states[-1].position_valid
        backend.detections = [Detection(7,(10,5,90,95),.9)]
        assert send(value=0).status == TargetState.TRACKING
        assert not states[-1].position_valid
        assert send(frame='raw_depth').status == TargetState.NOT_READY
        assert not states[-1].position_valid
        assert any('optical frame' in key for key in probe.report()['rejected_reasons'])
        assert not probe.report()['metadata_pass']
        assert send(age=3).status == TargetState.NOT_READY
        send(); spin(1.1)
        assert states[-1].status == TargetState.STALE
        assert markers[-1].action == Marker.DELETE
        assert set(probe.report()['stale_streams']) == {'color', 'depth'}
        assert not call('lock_target').success
        assert send().status == TargetState.LOST  # reset epoch prevents ID reuse
        assert backend.resets > 0
        assert call('lock_target').success
        spin()
        assert states[-1].position_valid
        backend.fail = True
        assert send().status == TargetState.NOT_READY
        assert not states[-1].position_valid
        backend.fail = False
        assert send().status == TargetState.LOST
        call('release_target')
        assert send().status == TargetState.SEARCHING
        pairs = probe.counts['pairs']
        send(skew_ns=-80000000)
        assert probe.counts['pairs'] == pairs  # Both streams arrive, but do not synchronize.
        assert abs(probe.report()['latest_stamp_skew_s']-.08) < 1e-5
        with tempfile.TemporaryDirectory() as folder:
            report_path = Path(folder)/'probe.json'
            result = subprocess.run([
                sys.executable, '/workspace/scripts/check_rgbd_input.py', '--duration', '.2',
                '--color-topic', '/absent/color', '--depth-topic', '/absent/depth',
                '--camera-info-topic', '/absent/info', '--output', str(report_path)],
                capture_output=True, text=True, timeout=10)
            assert result.returncode == 1, result.stderr
            report = json.loads(report_path.read_text())
            assert not report['metadata_pass'] and len(report['missing_streams']) == 3
            assert json.loads(result.stdout) == report
        assert '/cmd_vel' not in dict(node.get_topic_names_and_types())
        print('PASS synthetic B: selection, depth units, loss, stale, registration rejection, inference failure, ID epoch; read-only RGB-D probe; no cmd_vel')
    finally:
        executor.remove_node(transform_node)
        transform_node.destroy_node()
        executor.remove_node(probe)
        probe.destroy_node()
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
