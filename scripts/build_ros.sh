#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo '需要 Ubuntu / Jetson 上的 ROS 2 Humble，或运行 scripts/test_container.sh。' >&2
  exit 1
fi
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$ROOT/ros2_ws"
colcon build --base-paths src --symlink-install --event-handlers console_direct+ "$@"
