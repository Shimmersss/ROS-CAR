#!/usr/bin/env bash
# Start the vendor Astra ROS camera driver separately from bodyreader.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/wheeltec/wheeltec_ros2/install/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-182}"
export ROS_LOCALHOST_ONLY=0
exec ros2 run astra_camera astra_camera_node
