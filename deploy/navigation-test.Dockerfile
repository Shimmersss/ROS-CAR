FROM roscar-humble-test
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*
