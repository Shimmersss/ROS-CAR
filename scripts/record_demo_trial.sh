#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "${ROSCAR_ROBOT_ENV:-$HOME/.config/roscar/robot.env}" ]]; then
  set -a
  source "${ROSCAR_ROBOT_ENV:-$HOME/.config/roscar/robot.env}"
  set +a
fi
source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-182}"
exec python3 "$ROOT/scripts/record_demo_trial.py" "$@"
