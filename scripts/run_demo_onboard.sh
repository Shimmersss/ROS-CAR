#!/usr/bin/env bash
# Boot the demo in stationary mode; following is started explicitly after boot.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Load the private configuration through the same path as the trial entry.
set -a
source "${ROSCAR_ROBOT_ENV:-$HOME/.config/roscar/robot.env}"
set +a
export MOTION_ENABLED=false FOLLOW_AUTO_ARM=false FOLLOW_TARGET_AUTO_RECOVER=false
export FOLLOW_TARGET_DISTANCE_M=1.3 FOLLOW_MAX_LINEAR_MPS=0.35
export POSE_CONFIG="$ROOT/ros2_ws/src/perception_bringup/config/pose_fixed_27cm_demo.yaml"
exec bash "$ROOT/scripts/run_demo.sh" "$@"
