"""Actual ROS executor, controlled worker; no camera/model/motion validation."""
import copy
import json
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image
from yolo_person_tracker.node import TrackerNode


class Backend:
    def __init__(self):
        self.ready = threading.Event()
        self.finished = None
        self.error = False

    def reset(self):
        pass

    def infer(self, _image):
        assert self.ready.wait(3), 'test worker not released'
        self.finished = time.monotonic()
        if self.error:
            raise ValueError('controlled inference failure')
        return []


rclpy.init()
results = []
try:
    for enabled in (False, True):
        backend = Backend()
        node = TrackerNode(backend=backend, parameter_overrides=[
            Parameter('inference_completion_wakeup', value=enabled),
            Parameter('visualization_fps', value=0.0)])
        node.timer.cancel()  # No polling fallback: completion must wake the executor.
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        info = CameraInfo(width=32, height=32)
        info.header.frame_id = 'test_optical'
        info.p = [100., 0., 16., 0., 0., 100., 16., 0., 0., 0., 1., 0.]
        node.on_info(info)

        def submit(age=0.):
            backend.ready.clear()
            color = Image(width=32, height=32, encoding='bgr8', step=96, data=bytes(3072))
            color.header.frame_id = 'test_optical'
            color.header.stamp = node.get_clock().now().to_msg()
            depth = Image(width=32, height=32, encoding='16UC1', step=64, data=bytes(2048))
            depth.header = copy.deepcopy(color.header)
            node.on_pair(color, depth)
            assert node.future is not None
            if age:
                # Hold the actual worker beyond max_age_s, with no timer tick.
                time.sleep(age)
            backend.ready.set()

        def spin(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=.01)

        submit()
        spin(.2)
        if not enabled:
            assert node.future is not None and node.future.done()
            node.tick()
        else:
            assert node.future is None, 'completion must be consumed without timer polling'
        assert node.snapshot is not None and node.fresh_snapshot()
        results.append(dict(enabled=enabled, consumed_without_timer=enabled))
        if enabled:
            submit(.6)
            spin(.15)
            assert node.future is None and not node.fresh_snapshot(), 'late completion stays stale'
            backend.error = True
            submit()
            spin(.15)
            assert node.future is None and node.snapshot is None and 'controlled inference failure' in node.error
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
    print(json.dumps(results))
    print('PASS: executor wakeup, unchanged stale-result rejection and worker failure withdrawal')
finally:
    rclpy.shutdown()
