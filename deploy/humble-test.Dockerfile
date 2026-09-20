ARG ROS_IMAGE=ros:humble-ros-base-jammy
FROM ${ROS_IMAGE}
SHELL ["/bin/bash", "-c"]
# Only main/universe contain this image's dependencies. Avoid unused GPU-driver indexes.
RUN sed -i -E '/^deb-src /d; s/ restricted//g; s/ multiverse//g; /^deb .* jammy(-updates|-security|-backports)? *$/d' /etc/apt/sources.list \
    && apt-get update && apt-get install -y --no-install-recommends \
    alsa-utils python3-colcon-common-extensions python3-pytest python3-websocket \
    ros-humble-rosidl-default-generators ros-humble-geometry-msgs \
    python3-numpy python3-opencv ros-humble-cv-bridge ros-humble-message-filters \
    ros-humble-sensor-msgs ros-humble-visualization-msgs ros-humble-tf2-ros-py ros-humble-tf2-geometry-msgs \
    ros-humble-rclpy ros-humble-launch-ros ros-humble-std-srvs \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /workspace/ros2_ws
