#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${ROSCAR_ROS_IMAGE:-ros:humble-ros-base-jammy}"
mkdir -p "$ROOT/artifacts"
docker build --platform linux/arm64 --build-arg "ROS_IMAGE=$IMAGE" \
  -f "$ROOT/deploy/humble-test.Dockerfile" -t roscar-humble-test "$ROOT/deploy"
docker run --rm --platform linux/arm64 -e ROS_DOMAIN_ID=182 \
  -v "$ROOT/ros2_ws/src:/workspace/ros2_ws/src:ro" \
  -v "$ROOT/scripts:/workspace/scripts:ro" \
  -v "$ROOT/tests:/workspace/tests:ro" \
  roscar-humble-test bash -c '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    colcon build --base-paths src --event-handlers console_direct+
    source install/setup.bash
    bash /workspace/scripts/build_chassis.sh
    source chassis_install/setup.bash
    python3 /workspace/tests/test_vendor_frames.py /workspace/ros2_ws/src/chassis_vendor/turn_on_wheeltec_robot/src/wheeltec_robot.cpp
    python3 -m unittest discover -s src/motion_guard/test -v
    python3 -m unittest discover -s src/navigation_bringup/test -v
    ROS_DOMAIN_ID=169 python3 /workspace/tests/test_control_entrypoints.py
    ROS_DOMAIN_ID=173 python3 /workspace/tests/test_motion_guard_runtime.py
    ROS_DOMAIN_ID=174 ROS_LOCALHOST_ONLY=1 python3 /workspace/tests/test_follow_replay.py
    python3 -m unittest discover -s src/red_object_tracker/test -v
    ROS_DOMAIN_ID=176 python3 /workspace/tests/test_performance_runtime.py
    ROS_DOMAIN_ID=177 python3 /workspace/tests/test_red_video_runtime.py
    ROS_DOMAIN_ID=178 python3 /workspace/tests/test_route_a_launch.py
    ROS_DOMAIN_ID=179 python3 /workspace/tests/test_red_serial_runtime.py
    PYTHONPATH=/workspace/ros2_ws/src/astra_body_adapter \
      python3 -m unittest discover \
      -s /workspace/ros2_ws/src/astra_body_adapter/test -v
    PYTHONPATH=/workspace/ros2_ws/src/yolo_person_tracker python3 -m unittest discover -s src/yolo_person_tracker/test -v
    ROS_DOMAIN_ID=177 python3 /workspace/tests/test_review_safety.py
    python3 /workspace/tests/test_route_a_lifecycle.py
    ROS_DOMAIN_ID=178 python3 /workspace/tests/test_target_tf_runtime.py
    ROS_DOMAIN_ID=179 python3 /workspace/tests/test_yolo_concurrency.py
    ROS_DOMAIN_ID=180 python3 /workspace/tests/test_yolo_runtime.py
    ROS_DOMAIN_ID=181 python3 /workspace/tests/test_astra_adapter_runtime.py
    python3 -m pytest -q src/xfyun_speech/test src/deepseek_ros2/test src/voice_command_router/test
    python3 /workspace/tests/test_ros_runtime.py
  ' 2>&1 | tee "$ROOT/artifacts/humble-test.log"
