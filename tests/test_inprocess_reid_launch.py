"""Launch contract for shared rectified RGB; no camera/model/physical motion is started."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters


def main():
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '182'), 'Use an isolated test domain'
    root = Path(__file__).resolve().parents[1]
    rclpy.init()
    node = Node('inprocess_reid_launch_probe')
    with tempfile.TemporaryFile(mode='w+') as log:
        proc = subprocess.Popen([
            'ros2', 'launch', 'perception_bringup', 'perception.launch.py',
            'route:=yolo_pose', 'reid_enabled:=true', 'with_foxglove:=false', 'with_radar:=false',
            'with_auxiliary:=false',
            'color_topic:=/camera/color/image_raw', 'depth_topic:=/camera/depth/image_raw',
            'registration_expected_serial:=AY2755200PW',
            'registration_calibration_file:=' + str(root / 'ros2_ws/src/perception_bringup/config/gemini_AY2755200PW.json'),
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            for name, expected in [('yolo_person_tracker', '/camera/color/image_raw'),
                                   ('reid_observer', '/camera/color/image_rect')]:
                client = node.create_client(GetParameters, f'/perception/{name}/get_parameters')
                assert client.wait_for_service(timeout_sec=15), name
                future = client.call_async(GetParameters.Request(names=['color_topic']))
                rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                assert future.done() and future.result().values[0].string_value == expected
            end = time.monotonic() + 3
            while node.count_publishers('/camera/color/image_rect') != 1 and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.05)
            assert proc.poll() is None
            assert node.count_publishers('/camera/color/image_rect') == 1
            assert node.count_publishers('/cmd_vel') == 0
            assert not {'ground_localizer', 'unified_localizer', 'target_transform'} & set(node.get_node_names())
            print('PASS in-process + ReID launch: native RGB input, rectified ReID input, unique image publisher, no motion')
        except Exception:
            log.seek(0)
            print(log.read())
            raise
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=6)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait(timeout=3)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
