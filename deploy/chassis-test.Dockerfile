FROM roscar-humble-test
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-turtlesim ros-humble-nav2-msgs ros-humble-ackermann-msgs \
    && rm -rf /var/lib/apt/lists/*
