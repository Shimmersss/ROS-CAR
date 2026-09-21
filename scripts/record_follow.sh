#!/usr/bin/env bash
# Record only bounded task topics; never starts a controller or replay publisher.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROOT/ros2_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-182}"
DEST="${1:-$ROOT/data/recordings/follow-$(date +%Y%m%d-%H%M%S)}"
exec ros2 bag record -o "$DEST" /perception/target_state /scan /tf /tf_static \
  /control/cmd_vel_request /control/state /cmd_vel /odom \
  /map /plan /local_plan /global_costmap/costmap /local_costmap/costmap \
  /navigation/follow_goal /navigation/state /navigation/cmd_vel_raw /amcl_pose
