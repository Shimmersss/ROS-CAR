"""Unverified registration still permits 2D video, never target locking or 3D position."""
import copy
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image, CompressedImage
from std_srvs.srv import Trigger
from person_interfaces.msg import TargetState
from yolo_person_tracker.backend import Detection
from yolo_person_tracker.node import TrackerNode


class FakeBackend:
    def infer(self, image):
        return [Detection(7, (10, 5, 90, 95), .9)]

    def reset(self):
        pass


def main():
    rclpy.init()
    node = TrackerNode(backend=FakeBackend(), parameter_overrides=[
        Parameter('depth_registered', value=False), Parameter('max_age_s', value=1.0)])
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    states, images, previews, detection_previews = [], [], [], []
    node.create_subscription(TargetState, 'target_state', states.append, 10)
    node.create_subscription(Image, 'detections_image', images.append, 10)
    node.create_subscription(CompressedImage, 'color_preview/compressed', previews.append, 10)
    node.create_subscription(CompressedImage, 'detections_preview/compressed', detection_previews.append, 10)
    try:
        info = CameraInfo()
        info.width = info.height = 100
        info.header.frame_id = 'camera_optical'
        info.p = [100., 0., 50., 0., 0., 100., 50., 0., 0., 0., 1., 0.]
        node.on_info(info)
        color = node.bridge.cv2_to_imgmsg(np.zeros((100, 100, 3), np.uint8), 'bgr8')
        color.header.frame_id = 'camera_optical'
        color.header.stamp = node.get_clock().now().to_msg()
        depth = node.bridge.cv2_to_imgmsg(np.full((100, 100), 2000, np.uint16), '16UC1')
        depth.header = copy.deepcopy(color.header)
        node.on_pair(color, depth)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and not detection_previews:
            executor.spin_once(timeout_sec=.02)
        assert images and images[-1].header == color.header
        assert previews and detection_previews
        assert previews[-1].format == detection_previews[-1].format == 'jpeg'
        assert len(previews[-1].data) > 0 and len(detection_previews[-1].data) > 0
        assert states and states[-1].status == TargetState.NOT_READY
        assert not states[-1].position_valid and np.isnan(states[-1].position.z)
        assert not node.lock_target(Trigger.Request(), Trigger.Response()).success
        assert node.positions == {}
        print('PASS B unregistered: detection image live; target lock and 3D position disabled')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
