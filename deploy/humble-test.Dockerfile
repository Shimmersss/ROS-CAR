ARG ROS_IMAGE=ros:humble-ros-base-jammy
FROM ${ROS_IMAGE}
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    alsa-utils python3-colcon-common-extensions python3-pytest python3-websocket \
    ros-humble-rosidl-default-generators ros-humble-geometry-msgs \
    ros-humble-rclpy ros-humble-launch-ros ros-humble-std-srvs \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /workspace/ros2_ws
