#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Intended for Jetson/Linux ROS environment. No chassis or perception start here.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROOT/ros2_ws/install/setup.bash"
if [[ -f "$ROOT/ros2_ws/radar_install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$ROOT/ros2_ws/radar_install/setup.bash"
fi
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-182}"
exec ros2 launch navigation_bringup navigation.launch.py "$@"
