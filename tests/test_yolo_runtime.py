"""Synthetic RGB-D and injected detections; this is NOT camera/model validation."""
import copy
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
    states = []
    markers = []
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

    def send(encoding='16UC1', value=2000, frame='camera_optical', age=0):
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
        color_pub.publish(color); depth_pub.publish(depth)
        spin(.2)
        return states[-1]

    try:
        spin(.3)
        assert states[-1].status == TargetState.STALE
        assert send().status == TargetState.SEARCHING
        assert call('lock_target').success
        spin()
        assert states[-1].status == TargetState.TRACKING and states[-1].position_valid
        assert abs(states[-1].position.z-2) < 1e-5
        assert markers[-1].action == Marker.ADD
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
        assert send(age=3).status == TargetState.NOT_READY
        send(); spin(1.1)
        assert states[-1].status == TargetState.STALE
        assert markers[-1].action == Marker.DELETE
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
        assert '/cmd_vel' not in dict(node.get_topic_names_and_types())
        print('PASS synthetic B: selection, depth units, loss, stale, registration rejection, inference failure, ID epoch; no cmd_vel')
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
