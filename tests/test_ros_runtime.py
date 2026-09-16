"""Integration checks against built ROS packages, without hardware or a GPU."""
import math
import os
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from person_interfaces.msg import TargetState


def check_route(route):
    rclpy.init()
    probe = Node('roscar_test_probe')
    received = []
    subscription = probe.create_subscription(
        TargetState, '/perception/target_state', received.append, 10)
    with tempfile.TemporaryFile(mode='w+') as output:
        process = subprocess.Popen(
            ['ros2', 'launch', 'perception_bringup', 'perception.launch.py',
             f'route:={route}', 'with_foxglove:=false'],
            stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + (16 if route == 'demo' else 8)
            while time.monotonic() < deadline:
                assert process.poll() is None, f'{route} launch exited early'
                rclpy.spin_once(probe, timeout_sec=0.1)
                if route != 'demo' and len(received) >= 2:
                    break
                if route == 'demo' and any(m.status == TargetState.LOST for m in received):
                    break
            assert received, f'{route}: no messages received'
            assert all(m.source == ('red_object' if route == 'red' else route) for m in received)
            assert all(m.header.stamp.sec > 0 for m in received)
            assert '/cmd_vel' not in dict(probe.get_topic_names_and_types())
            if route != 'demo':
                expected = (TargetState.STALE if route == 'astra'
                            else TargetState.NOT_READY)
                assert all(m.status == expected for m in received)
                assert all(not m.position_valid and not m.is_simulated for m in received)
                assert all(math.isnan(m.horizontal_distance_m) for m in received)
                assert all(m.observation_stamp.sec == 0 for m in received)
            else:
                assert all(m.is_simulated for m in received)
                visible = [m for m in received if m.status == TargetState.TRACKING]
                lost = [m for m in received if m.status == TargetState.LOST]
                assert visible and lost, 'demo must expose both visible and lost states'
                for msg in visible:
                    assert msg.position_valid and msg.target_id == 'demo-person-1'
                    assert msg.header.frame_id == 'demo_camera_optical_frame'
                    assert msg.observation_stamp == msg.header.stamp
                    assert math.isclose(msg.horizontal_distance_m,
                                        math.hypot(msg.position.x, msg.position.z), rel_tol=1e-6)
                    assert math.isclose(msg.bearing_rad,
                                        math.atan2(msg.position.x, msg.position.z), abs_tol=1e-6)
                assert all(not m.position_valid and math.isnan(m.position.z) for m in lost)
                assert all(m.observation_stamp.sec == 0 for m in lost)
            print(f'PASS route={route}: {len(received)} messages; validity and source checked', flush=True)
        except Exception:
            output.seek(0)
            print(output.read())
            raise
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=6)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
            probe.destroy_subscription(subscription)
            probe.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    for selected in ['astra', 'yolo', 'red', 'demo']:
        check_route(selected)
    invalid = subprocess.run(
        ['ros2', 'launch', 'perception_bringup', 'perception.launch.py', 'route:=invalid'],
        capture_output=True, text=True, timeout=10)
    assert invalid.returncode != 0, 'Unknown routes must fail instead of selecting a fallback'
    print('PASS invalid route rejected. All ROS runtime checks passed.')
