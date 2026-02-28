# ROS2 master/daemon for Client Raspberry Pi
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros2cli \
    && rm -rf /var/lib/apt/lists/*

# Start daemon then keep container running so systemd sees active
CMD ["sh", "-c", "ros2 daemon start; exec sleep infinity"]
