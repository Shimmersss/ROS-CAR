#!/usr/bin/env bash
# Native Jetson/Linux entry point; no chassis, follower, camera or autostart.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROOT/ros2_ws/radar_install/setup.bash"
# shellcheck disable=SC1091
source "$ROOT/ros2_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-182}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
exec ros2 launch perception_bringup radar.launch.py "$@"
