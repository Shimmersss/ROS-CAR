#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$ROOT/artifacts"
docker build --platform linux/arm64 -f "$ROOT/deploy/humble-test.Dockerfile" -t roscar-humble-test "$ROOT/deploy"
docker build --platform linux/arm64 -f "$ROOT/deploy/navigation-test.Dockerfile" -t roscar-navigation-test "$ROOT/deploy"
docker run --rm --platform linux/arm64 -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=171 \
  -v "$ROOT/ros2_ws/src:/workspace/ros2_ws/src:ro" \
  -v "$ROOT/scripts:/workspace/scripts:ro" -v "$ROOT/tests:/workspace/tests:ro" \
  roscar-navigation-test bash -c '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    colcon build --base-paths src --event-handlers console_direct+
    source install/setup.bash
    python3 -m unittest discover -s src/navigation_bringup/test -v
    python3 /workspace/tests/test_navigation_stack.py || { cat /tmp/slam-test.log /tmp/nav2-test.log /tmp/amcl-test.log; exit 1; }
    ROS_DOMAIN_ID=172 python3 /workspace/tests/test_navigation_bridges.py
  ' 2>&1 | tee "$ROOT/artifacts/navigation-test.log"
