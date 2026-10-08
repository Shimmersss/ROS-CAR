#!/usr/bin/env bash
# Gimbal bridge: protocol/sync/simulator unit tests and pty+URDF+TF runtime checks (no hardware).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$ROOT/artifacts"
docker run --rm --platform linux/arm64 -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=204 \
  -v "$ROOT/ros2_ws/src:/workspace/ros2_ws/src:ro" -v "$ROOT/tests:/workspace/tests:ro" \
  "${ROSCAR_C_TEST_IMAGE:-roscar-humble-test}" bash -c '
    set -e
    source /opt/ros/humble/setup.bash
    colcon build --base-paths src --packages-up-to gimbal_bridge --event-handlers console_direct+
    source install/setup.bash
    PYTHONPATH=src/gimbal_bridge python3 -m unittest discover -s src/gimbal_bridge/test -v
    python3 /workspace/tests/test_gimbal_runtime.py
  ' 2>&1 | tee "${ROSCAR_GIMBAL_TEST_LOG:-$ROOT/artifacts/gimbal-test.log}"
