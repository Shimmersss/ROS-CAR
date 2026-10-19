#!/usr/bin/env bash
# Synthetic ROS only; uses an existing Humble image, no hardware or deployment.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$ROOT/artifacts/demo-validation"
docker run --rm -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=214 \
  -v "$ROOT:/workspace:ro" -w /tmp/demo-build \
  "${ROSCAR_C_TEST_IMAGE:-roscar-humble-test}" bash -c '
    set -e
    source /opt/ros/humble/setup.bash
    colcon build --base-paths /workspace/ros2_ws/src --packages-up-to demo_motion motion_guard yolo_person_tracker
    source install/setup.bash
    PYTHONPATH=/workspace/ros2_ws/src/demo_motion python3 -m pytest -q -p no:cacheprovider /workspace/ros2_ws/src/demo_motion/test
    python3 /workspace/tests/test_demo_launch.py
    python3 /workspace/tests/test_demo_relative_motion_runtime.py
    python3 /workspace/tests/test_demo_entrypoints.py
    python3 /workspace/tests/test_c_entrypoints.py
    python3 -m pytest -q -p no:cacheprovider /workspace/ros2_ws/src/astra_body_adapter/test /workspace/ros2_ws/src/yolo_person_tracker/test /workspace/ros2_ws/src/motion_guard/test
    ROS_DOMAIN_ID=215 python3 /workspace/tests/test_demo_fall_follow_runtime.py
    ROS_DOMAIN_ID=216 python3 /workspace/tests/test_demo_stationary_runtime.py
    ROS_DOMAIN_ID=216 python3 /workspace/tests/test_height_continue_runtime.py
    ROS_DOMAIN_ID=216 python3 /workspace/tests/test_pose_runtime.py
    ROS_DOMAIN_ID=217 python3 /workspace/tests/test_owner_recovery_deadlines_runtime.py
    ROS_DOMAIN_ID=218 python3 /workspace/tests/test_demo_owner_recovery_runtime.py
    ROS_DOMAIN_ID=219 python3 /workspace/tests/test_reid_output_gap_runtime.py
  ' 2>&1 | tee "$ROOT/artifacts/demo-validation/acceptance.log"
