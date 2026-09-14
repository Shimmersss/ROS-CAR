ARG ROS_IMAGE=ros:humble-ros-base-jammy
FROM ${ROS_IMAGE}
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions ros-humble-rosidl-default-generators \
    ros-humble-geometry-msgs ros-humble-rclpy ros-humble-launch-ros \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /workspace/ros2_ws
