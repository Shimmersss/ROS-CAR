#!/usr/bin/env bash
# Start the vendor Astra ROS camera driver separately from bodyreader.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/wheeltec/wheeltec_ros2/install/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-182}"
export ROS_LOCALHOST_ONLY=0
# Use the vendor launch namespace so outputs match /camera/color|depth/image_raw.
# Request registration only after the caller has verified the actual camera setup.
exec ros2 launch astra_camera astra.launch.xml \
  camera_name:=camera enable_color:=true enable_depth:=true enable_ir:=false \
  enable_point_cloud:=false "depth_registration:=${DEPTH_REGISTERED:-false}"
