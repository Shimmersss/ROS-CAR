#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
mkdir -p artifacts
export PYTHONPATH="$ROOT/ros2_ws/src/voice_command_router:$ROOT/ros2_ws/src/astra_body_adapter${PYTHONPATH:+:$PYTHONPATH}"
python3 -m pytest -q tests/test_tongue_logic.py ros2_ws/src/voice_command_router/test \
  ros2_ws/src/motion_guard/test \
  --ignore=ros2_ws/src/voice_command_router/test/test_mode_bridge.py | tee artifacts/tongue-unit.log
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo '纯逻辑测试已执行；ROS 构建和运行测试需要 Ubuntu 22.04 / Humble。' >&2
  exit 2
fi
set +u
source /opt/ros/humble/setup.bash
set -u
colcon --log-base "$ROOT/log" build --base-paths "$ROOT/ros2_ws/src" \
  --packages-up-to tongue_mode --build-base "$ROOT/build" --install-base "$ROOT/install" \
  --event-handlers console_direct+ | tee artifacts/tongue-build.log
set +u
source "$ROOT/install/setup.bash"
set -u
export ROS_DOMAIN_ID=184 ROS_LOCALHOST_ONLY=1
python3 -m pytest -q tests/test_tongue_logic.py ros2_ws/src/voice_command_router/test \
  ros2_ws/src/motion_guard/test ros2_ws/src/xfyun_speech/test \
  ros2_ws/src/offline_voice/test | tee artifacts/tongue-ros-unit.log
python3 tests/test_tongue_mode_runtime.py | tee artifacts/tongue-runtime.log
python3 tests/test_task_hold_runtime.py | tee artifacts/tongue-guard-regression.log
python3 tests/test_api_control.py | tee artifacts/tongue-api-regression.log
