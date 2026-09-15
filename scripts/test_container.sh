#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${ROSCAR_ROS_IMAGE:-ros:humble-ros-base-jammy}"
mkdir -p "$ROOT/artifacts"
docker build --platform linux/arm64 --build-arg "ROS_IMAGE=$IMAGE" \
  -f "$ROOT/deploy/humble-test.Dockerfile" -t roscar-humble-test "$ROOT/deploy"
docker run --rm --platform linux/arm64 -e ROS_DOMAIN_ID=182 \
  -v "$ROOT/ros2_ws/src:/workspace/ros2_ws/src:ro" \
  -v "$ROOT/tests:/workspace/tests:ro" \
  roscar-humble-test bash -c '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    colcon build --base-paths src --event-handlers console_direct+
    source install/setup.bash
    PYTHONPATH=/workspace/ros2_ws/src/astra_body_adapter \
      python3 -m unittest discover \
      -s /workspace/ros2_ws/src/astra_body_adapter/test -v
    ROS_DOMAIN_ID=181 python3 /workspace/tests/test_astra_adapter_runtime.py
    python3 -m pytest -q src/xfyun_speech/test src/deepseek_ros2/test src/voice_command_router/test
    python3 /workspace/tests/test_ros_runtime.py
  ' 2>&1 | tee "$ROOT/artifacts/humble-test.log"
