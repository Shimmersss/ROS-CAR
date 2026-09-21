#!/usr/bin/env bash
# Replay to isolated names; captured final commands can never reach /cmd_vel here.
set -eo pipefail
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
if [[ $# != 1 ]]; then echo '用法：bash scripts/replay_follow.sh BAG_DIRECTORY' >&2; exit 2; fi
export ROS_DOMAIN_ID=174
export ROS_LOCALHOST_ONLY=1
exec ros2 bag play "$1" --topics \
  /perception/target_state /scan /tf /tf_static /control/cmd_vel_request /control/state /cmd_vel /odom \
  /map /plan /local_plan /global_costmap/costmap /local_costmap/costmap \
  /navigation/follow_goal /navigation/state /navigation/cmd_vel_raw /amcl_pose \
  --remap \
  /perception/target_state:=/replay/target_state /scan:=/replay/scan \
  /tf:=/replay/tf /tf_static:=/replay/tf_static \
  /control/cmd_vel_request:=/replay/cmd_vel_request /control/state:=/replay/control_state \
  /cmd_vel:=/replay/cmd_vel /odom:=/replay/odom \
  /map:=/replay/map /plan:=/replay/plan /local_plan:=/replay/local_plan \
  /global_costmap/costmap:=/replay/global_costmap /local_costmap/costmap:=/replay/local_costmap \
  /navigation/follow_goal:=/replay/follow_goal /navigation/state:=/replay/navigation_state \
  /navigation/cmd_vel_raw:=/replay/nav_cmd_vel_raw /amcl_pose:=/replay/amcl_pose
