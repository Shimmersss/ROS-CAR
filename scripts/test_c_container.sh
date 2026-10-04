#!/usr/bin/env bash
# Scoped C acceptance plus affected B/A/voice/launch contracts. Full legacy suite: test_container.sh.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$ROOT/artifacts"
docker run --rm --platform linux/arm64 -e ROS_LOCALHOST_ONLY=1 \
  -v "$ROOT/ros2_ws/src:/workspace/ros2_ws/src:ro" \
  -v "$ROOT/scripts:/workspace/scripts:ro" -v "$ROOT/tests:/workspace/tests:ro" \
  "${ROSCAR_C_TEST_IMAGE:-roscar-humble-test}" bash -c '
    set -e
    source /opt/ros/humble/setup.bash
    colcon build --base-paths src --event-handlers console_direct+
    source install/setup.bash
    PYTHONPATH=src/yolo_person_tracker python3 -m unittest discover -s src/yolo_person_tracker/test -v
    ROS_DOMAIN_ID=188 python3 /workspace/tests/test_pose_runtime.py
    ROS_DOMAIN_ID=187 python3 /workspace/tests/test_pose_runtime.py --fusion-off
    ROS_DOMAIN_ID=189 python3 /workspace/tests/test_gemini_registration_runtime.py
    ROS_DOMAIN_ID=190 python3 /workspace/tests/test_yolo_runtime.py
    ROS_DOMAIN_ID=191 python3 /workspace/tests/test_target_tf_runtime.py
    ROS_DOMAIN_ID=192 python3 /workspace/tests/test_yolo_concurrency.py
    ROS_DOMAIN_ID=197 python3 /workspace/tests/test_ground_runtime.py
    ROS_DOMAIN_ID=198 python3 /workspace/tests/test_ground_prior_runtime.py
    ROS_DOMAIN_ID=199 python3 /workspace/tests/test_reid_runtime.py
    ROS_DOMAIN_ID=200 python3 /workspace/tests/test_identity_lock_runtime.py
    ROS_DOMAIN_ID=201 python3 /workspace/tests/test_unified_runtime.py
    ROS_DOMAIN_ID=202 python3 /workspace/tests/test_pose3d_runtime.py
    ROS_DOMAIN_ID=193 python3 /workspace/tests/test_astra_adapter_runtime.py
    python3 -m pytest -q src/xfyun_speech/test src/deepseek_ros2/test src/voice_command_router/test
    ROS_DOMAIN_ID=194 python3 /workspace/tests/test_ros_runtime.py
    ROS_DOMAIN_ID=196 python3 /workspace/tests/test_c_api_launch.py
  ' 2>&1 | tee "${ROSCAR_C_TEST_LOG:-$ROOT/artifacts/c-acceptance.log}"
