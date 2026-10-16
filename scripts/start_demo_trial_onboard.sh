#!/usr/bin/env bash
# Prepared manual entry; does not replace the boot deployment or authorize movement.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if systemctl --user is-active --quiet roscar-robot.service; then
  echo 'Existing robot service is active. Arrange a stationary handover before starting this entry.' >&2
  exit 2
fi
set -a
source "${ROSCAR_ROBOT_ENV:-$HOME/.config/roscar/robot.env}"
set +a
export MOTION_ENABLED=false FOLLOW_AUTO_ARM=false FOLLOW_TARGET_AUTO_RECOVER=false
export FOLLOW_TARGET_DISTANCE_M=1.3 FOLLOW_MAX_LINEAR_MPS=0.35
export POSE_CONFIG="$ROOT/ros2_ws/src/perception_bringup/config/pose_fixed_27cm_demo.yaml"
exec bash "$ROOT/scripts/run_demo.sh" "$@"
