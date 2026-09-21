#!/usr/bin/env bash
set -euo pipefail
if [[ $# != 1 || "$1" != /* ]]; then
  echo 'Usage: save_navigation_map.sh /absolute/output/map_name' >&2; exit 2
fi
for suffix in yaml pgm png; do
  [[ ! -e "$1.$suffix" ]] || { echo "Refusing to overwrite $1.$suffix" >&2; exit 2; }
done
mkdir -p "$(dirname "$1")"
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-182}"
exec ros2 run nav2_map_server map_saver_cli -f "$1" --ros-args -p save_map_timeout:=10.0
