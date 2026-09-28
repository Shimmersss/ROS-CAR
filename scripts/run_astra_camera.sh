#!/usr/bin/env bash
# Start the vendor Astra ROS camera driver separately from bodyreader.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /home/wheeltec/wheeltec_ros2/install/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-182}"
export ROS_LOCALHOST_ONLY=0
COLOR_INFO_FILE="$ROOT/ros2_ws/src/perception_bringup/config/astra_s_provisional_color.yaml"
IR_INFO_FILE="$ROOT/ros2_ws/src/perception_bringup/config/astra_s_provisional_depth.yaml"
[[ -f "$COLOR_INFO_FILE" && -f "$IR_INFO_FILE" ]] || {
  echo '缺少 Astra 临时内参 YAML。' >&2
  exit 1
}
# Use the vendor launch namespace so outputs match /camera/color|depth/image_raw.
# Request registration only after the caller has verified the actual camera setup.
exec ros2 launch astra_camera astra.launch.xml \
  camera_name:=camera enable_color:=true enable_depth:=true enable_ir:=false \
  enable_point_cloud:=false "depth_registration:=${DEPTH_REGISTERED:-false}" \
  "color_info_url:=file://$COLOR_INFO_FILE" "ir_info_url:=file://$IR_INFO_FILE"
