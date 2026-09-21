"""A deliberately blocked backend must not block release service or state timers."""
import threading
import time
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import Trigger
from person_interfaces.msg import TargetState
from yolo_person_tracker.backend import Detection
from yolo_person_tracker.node import TrackerNode


class SlowBackend:
    def __init__(self):
        self.entered, self.release = threading.Event(), threading.Event()
        self.calls = 0

    def infer(self, image):
        self.calls += 1
        self.entered.set()
        assert self.release.wait(3), 'Test did not unblock inference'
        return [Detection(7, (0,0,100,100), .9)]

    def reset(self):
        pass


def main():
    rclpy.init()
    backend = SlowBackend()
    node = TrackerNode(backend=backend, parameter_overrides=[
        Parameter('depth_registered', value=True), Parameter('max_age_s', value=.3)])
    client_node = rclpy.create_node('concurrency_test_client')
    states = []
    client_node.create_subscription(TargetState, 'target_state', states.append, 10)
    client = client_node.create_client(Trigger, 'release_target')
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node); executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    try:
        assert client.wait_for_service(timeout_sec=2)
        info = CameraInfo(); info.width = info.height = 100
        info.header.frame_id = 'camera_optical'
        info.p = [100.,0.,50.,0.,0.,100.,50.,0.,0.,0.,1.,0.]
        node.on_info(info)
        color = node.bridge.cv2_to_imgmsg(np.zeros((100,100,3),np.uint8),'bgr8')
        color.header.frame_id = 'camera_optical'
        color.header.stamp = node.get_clock().now().to_msg()
        depth = node.bridge.cv2_to_imgmsg(np.ones((100,100),np.float32),'32FC1')
        depth.header = color.header
        node.on_pair(color,depth)
        assert backend.entered.wait(1)
        node.on_pair(color,depth)  # Must drop, not queue or start a second tracker update.
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic()+1
        while not future.done() and time.monotonic()<deadline:
            time.sleep(.01)
        assert future.done() and future.result().success
        assert not backend.release.is_set() and backend.calls == 1
        time.sleep(.4)
        assert len(states) >= 3, 'State timer stalled behind inference'
        backend.release.set()
        deadline = time.monotonic()+1
        while time.monotonic()<deadline:
            with node.state_lock:
                done = node.future is None
            if done:
                break
            time.sleep(.01)
        assert done
        time.sleep(.1)
        assert states[-1].status == TargetState.STALE and not states[-1].position_valid
        assert backend.calls == 1
        print('PASS concurrent B: services/timer responsive during inference, one backend task, stale completion rejected')
    finally:
        backend.release.set()
        executor.shutdown(); thread.join()
        node.destroy_node(); client_node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
