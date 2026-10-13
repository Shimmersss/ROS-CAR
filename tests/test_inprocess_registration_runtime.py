"""In-process Gemini registration vs the standalone node on synthetic frames; NOT camera acceptance."""
import copy
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from person_interfaces.msg import TargetState
from yolo_person_tracker.backend import Detection
from yolo_person_tracker.gemini_registration import GeminiRegistration
from yolo_person_tracker.node import TrackerNode

CALIBRATION = str(Path(__file__).resolve().parents[1]
                  / 'ros2_ws/src/perception_bringup/config/gemini_AY2755200PW.json')
SERIAL = 'AY2755200PW'
RAW_COLOR, RAW_DEPTH = '/camera/color/image_raw', '/camera/depth/image_raw'
INFO_RECT = '/camera/color/camera_info_rect'


class FakeBackend:
    def __init__(self):
        self.images = []

    def reset(self):
        pass

    def infer(self, image):
        self.images.append(image)
        return [Detection(7, (220, 100, 420, 420), .9)]


def scene():
    rng = np.random.default_rng(3)
    color = rng.integers(0, 255, (480, 640, 3), dtype=np.uint8)
    depth = np.full((400, 640), 2600, np.uint16)  # background wall, millimetres
    depth[80:360, 240:420] = 1500  # nearer person-sized block
    depth[200:210, 300:310] = 0  # invalid hole
    return color, depth


def run(inprocess, frames=8, color_frame='camera_color_optical_frame'):
    bridge = CvBridge()
    backend = FakeBackend()
    params = [Parameter('depth_registered', value=True), Parameter('max_age_s', value=1.0),
              Parameter('auto_lock_single', value=True), Parameter('camera_info_topic', value=INFO_RECT),
              Parameter('visualization_fps', value=0.)]
    nodes = []
    if inprocess:
        params += [Parameter('color_topic', value=RAW_COLOR), Parameter('depth_topic', value=RAW_DEPTH),
                   Parameter('registration_calibration_file', value=CALIBRATION),
                   Parameter('registration_expected_serial', value=SERIAL)]
    else:
        params += [Parameter('color_topic', value='/camera/color/image_rect'),
                   Parameter('depth_topic', value='/camera/aligned_depth_to_color/image_raw')]
        nodes.append(GeminiRegistration(parameter_overrides=[
            Parameter('calibration_file', value=CALIBRATION), Parameter('expected_serial', value=SERIAL)]))
    tracker = TrackerNode(backend=backend, parameter_overrides=params)
    nodes.append(tracker)
    ex = SingleThreadedExecutor()
    for node in nodes:
        ex.add_node(node)
    states, infos, rectified = [], [], []
    tracker.create_subscription(TargetState, 'target_state', states.append, 10)
    tracker.create_subscription(CameraInfo, INFO_RECT, infos.append, 10)
    tracker.create_subscription(Image, '/camera/color/image_rect', rectified.append, 10)
    cp = tracker.create_publisher(Image, RAW_COLOR, 10)
    dp = tracker.create_publisher(Image, RAW_DEPTH, 10)

    def spin(duration):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            ex.spin_once(timeout_sec=.01)

    color, depth = scene()
    try:
        spin(.3)
        for _ in range(frames):
            c = bridge.cv2_to_imgmsg(color, 'bgr8')
            c.header.frame_id = color_frame
            c.header.stamp = tracker.get_clock().now().to_msg()
            d = bridge.cv2_to_imgmsg(depth, '16UC1')
            d.header = copy.deepcopy(c.header)
            d.header.frame_id = 'camera_depth_optical_frame'
            cp.publish(c); dp.publish(d)
            spin(.25)
        spin(.2)
        return states, infos, backend, rectified
    finally:
        ex.shutdown()
        for node in nodes:
            node.destroy_node()


def main():
    rclpy.init()
    try:
        ref_states, ref_infos, ref_backend, ref_rgb = run(inprocess=False)
        states, infos, backend, rectified = run(inprocess=True)
        ref = [s for s in ref_states if s.status == TargetState.TRACKING and s.position_valid]
        got = [s for s in states if s.status == TargetState.TRACKING and s.position_valid]
        assert ref and got, (len(ref), len(got), states[-1].detail if states else None)
        a, b = ref[-1].position, got[-1].position
        # 32FC1 standalone output is the same float32 depth as the in-process array.
        np.testing.assert_allclose([b.x, b.y, b.z], [a.x, a.y, a.z], atol=1e-6)
        assert 1.3 < b.z < 1.7, b.z
        # The tracker infers on the rectified image, identical to the standalone node's output.
        np.testing.assert_array_equal(backend.images[-1], ref_backend.images[-1])
        assert rectified, 'ReID input missing with visualization disabled'
        np.testing.assert_array_equal(CvBridge().imgmsg_to_cv2(rectified[-1]), backend.images[-1])
        assert rectified[-1].header.stamp == infos[-1].header.stamp
        assert rectified[-1].header.frame_id == infos[-1].header.frame_id
        assert infos and list(infos[-1].p) == list(ref_infos[-1].p) and infos[-1].width == 640
        assert infos[-1].header.frame_id == 'camera_color_optical_frame'
        # Unexpected native frame: no inference, no CameraInfo, NOT_READY with the reason.
        states, infos, backend, rectified = run(inprocess=True, frames=3, color_frame='bad_frame')
        assert not backend.images and not infos and not rectified
        assert states[-1].status == TargetState.NOT_READY and 'native optical frames' in states[-1].detail
        try:
            TrackerNode(backend=FakeBackend(), parameter_overrides=[
                Parameter('registration_calibration_file', value=CALIBRATION),
                Parameter('registration_expected_serial', value='WRONG')])
        except ValueError as exc:
            assert 'serial' in str(exc)
        else:
            raise AssertionError('serial mismatch must refuse to start')
        print('PASS in-process Gemini registration matches the standalone node (position, rectified image, '
              'CameraInfo); native-frame and serial rejection (synthetic)')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
