FROM roscar-humble-test
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-radar-msgs ros-humble-pcl-ros ros-humble-pcl-conversions \
    ros-humble-laser-geometry libpcap-dev libyaml-cpp-dev libboost-thread-dev \
    && rm -rf /var/lib/apt/lists/*
