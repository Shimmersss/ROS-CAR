#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
if [[ -f /home/wheeltec/wheeltec_ros2/install/setup.bash ]]; then
  source /home/wheeltec/wheeltec_ros2/install/setup.bash
fi
if [[ -f "$ROOT/install/setup.bash" ]]; then
  source "$ROOT/install/setup.bash"
elif [[ -f "$ROOT/ros2_ws/install/setup.bash" ]]; then
  source "$ROOT/ros2_ws/install/setup.bash"
else
  echo '请先构建此副本的 ROS 包。' >&2
  exit 1
fi
set -u
export ROSCAR_WS="$ROOT/ros2_ws"
OFFLINE_SITE="${ROSCAR_OFFLINE_ROOT:-/home/wheeltec/ROSCAR-offline}/venv/lib/python3.10/site-packages"
if [[ -d "$OFFLINE_SITE/sherpa_onnx" ]]; then
  export PYTHONPATH="$OFFLINE_SITE${PYTHONPATH:+:$PYTHONPATH}"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-182}"
exec ros2 launch tongue_mode tongue_mode.launch.py "$@"
