#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
docker build --platform linux/arm64 -f "$ROOT/deploy/humble-test.Dockerfile" -t roscar-humble-test "$ROOT/deploy"
docker build --platform linux/arm64 -f "$ROOT/deploy/radar-test.Dockerfile" -t roscar-radar-test "$ROOT/deploy"
docker run --rm --platform linux/arm64 -e ROS_DOMAIN_ID=175 -e ROS_LOCALHOST_ONLY=1 \
  -v "$ROOT/ros2_ws/src:/workspace/ros2_ws/src:ro" \
  -v "$ROOT/scripts:/workspace/scripts:ro" -v "$ROOT/tests:/workspace/tests:ro" \
  roscar-radar-test bash -c '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    colcon build --base-paths src
    source install/setup.bash
    bash /workspace/scripts/build_radar.sh
    source radar_install/setup.bash
    python3 /workspace/tests/test_radar_runtime.py
    python3 /workspace/tests/test_n10p_driver.py
    python3 /workspace/tests/test_ros_runtime.py
  '
