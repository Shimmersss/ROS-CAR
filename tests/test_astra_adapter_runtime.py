"""Synthetic ROS integration check; run in an isolated ROS domain, no camera."""
import math
import os
import signal
import subprocess
import time

import rclpy
from bodyreader_msg.msg import Body, Bodylist
from person_interfaces.msg import TargetState
from visualization_msgs.msg import Marker


def main():
    rclpy.init()
    node = rclpy.create_node('synthetic_astra_probe')
    messages = []
    markers = []
    sub = node.create_subscription(TargetState, '/perception/target_state', messages.append, 10)
    marker_sub = node.create_subscription(
        Marker, '/perception/target_marker', markers.append, 10)
    pub = node.create_publisher(Bodylist, '/bodylist', 10)
    process = subprocess.Popen([
        'ros2', 'run', 'astra_body_adapter', 'bodylist_adapter',
        '--ros-args', '-r', '__ns:=/perception',
    ], start_new_session=True)

    def wait_for(status, msg=None):
        messages.clear()
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline:
            if msg is not None:
                pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.1)
            if messages and messages[-1].status == status:
                return messages[-1]
        raise AssertionError(f'missing status {status}')

    def wait_for_marker(action):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if markers and markers[-1].action == action:
                return markers[-1]
        raise AssertionError(f'missing marker action {action}')

    try:
        wait_for(TargetState.STALE)
        empty = Bodylist()
        wait_for(TargetState.SEARCHING, empty)
        body = Body()
        body.bodyid = 7
        body.centerofmass.x = 100.0
        body.centerofmass.y = 200.0
        body.centerofmass.z = 2000.0
        for shoulder, hand in [(2, 4), (5, 7)]:
            body.joints[shoulder].worldposition.y = 300.0
            body.joints[hand].worldposition.y = 150.0
        msg = Bodylist()
        msg.count = 1
        msg.bodies[0] = body
        result = wait_for(TargetState.TRACKING, msg)
        assert result.position_valid and result.target_id == '7'
        assert abs(result.position.z - 2.0) < 1e-6
        assert abs(result.position.y + 0.2) < 1e-6
        assert result.observation_stamp.sec == result.observation_stamp.nanosec == 0
        assert math.isnan(result.measurement_age_s)
        marker = wait_for_marker(Marker.ADD)
        assert abs(marker.pose.position.z - 2.0) < 1e-6
        lost = wait_for(TargetState.LOST, empty)
        assert not lost.position_valid and math.isnan(lost.position.z)
        wait_for_marker(Marker.DELETE)
        wait_for(TargetState.STALE)
        assert '/cmd_vel' not in dict(node.get_topic_names_and_types())
        print('PASS synthetic ROS adapter: state, units, Marker ADD/DELETE, unknown timestamp, no cmd_vel')
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        node.destroy_subscription(sub)
        node.destroy_subscription(marker_sub)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
