#!/usr/bin/env bash
# Run against a built Humble overlay. Synthetic inputs; never a hardware acceptance test.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "${PERCEPTION_INSTALL:-$ROOT/install}/setup.bash"
set -u
export ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1
# Tests launching the default guard must not read or modify a real robot's
# persisted emergency latch, even when their ROS domain is isolated.
FLOW_TEST_STATE_DIR=$(mktemp -d)
export XDG_STATE_HOME="$FLOW_TEST_STATE_DIR"
trap 'rm -rf "$FLOW_TEST_STATE_DIR"' EXIT
cd "$ROOT"
python3 tests/test_entry_paths.py
python3 tests/test_voice_flow_regressions.py
python3 -m pytest -q -p no:cacheprovider tests/test_tongue_logic.py
tests=(test_motion_guard_grace_runtime test_fast_follow_launch_runtime test_selection_gate_runtime test_owner_announcements_runtime test_owner_recovery_deadlines_runtime
       test_gemini_registration_runtime test_inprocess_registration_runtime test_inprocess_reid_launch test_tracker_input_continuity test_fall_announcements_runtime test_fixed_camera_fall_runtime
       test_ground_runtime test_unified_runtime test_ground_prior_runtime test_yolo_runtime
       test_yolo_concurrency test_sot_runtime test_identity_lock_runtime test_pose_runtime
       test_c_api_launch test_api_modules_runtime test_api_control test_api_examples
       test_task_hold_runtime test_tongue_mode_runtime)
for test in "${tests[@]}"; do
  echo "== $test (synthetic ROS)"
  ROS_DOMAIN_ID=185 python3 "tests/$test.py"
done
# This test enforces its dedicated isolated domain.
ROS_DOMAIN_ID=186 python3 tests/test_speech_api_runtime.py
