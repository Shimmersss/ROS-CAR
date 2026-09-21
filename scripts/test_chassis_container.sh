#!/usr/bin/env bash
# Build a disposable copy; keep the real workspace COLCON_IGNORE in place.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
docker build --platform linux/arm64 -f "$ROOT/deploy/chassis-test.Dockerfile" -t roscar-chassis-test "$ROOT/deploy"
docker run --rm --platform linux/arm64 \
  -v "$ROOT/ros2_ws/src/chassis_vendor:/source:ro" \
  -v "$ROOT/tests:/workspace/tests:ro" \
  roscar-chassis-test bash -c '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    mkdir -p /tmp/chassis/src
    cp -a /source/serial /source/wheeltec_robot_msg /source/turn_on_wheeltec_robot /tmp/chassis/src/
    find /tmp/chassis/src -name COLCON_IGNORE -delete
    cd /tmp/chassis
    colcon build --base-paths src --cmake-args -DBUILD_TESTING=OFF
    python3 /workspace/tests/test_vendor_frames.py /source/turn_on_wheeltec_robot/src/wheeltec_robot.cpp
  '
