# RPLidar A1 bridge — publishes sensor_msgs/LaserScan via ros-jazzy-rplidar-ros.
# Pass serial device e.g. --device=/dev/ttyUSB0:/dev/ttyUSB0 when running.
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rplidar-ros \
    ros-jazzy-launch-ros \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY launch/rplidar_a1.launch.py ./launch/

CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && exec ros2 launch /app/launch/rplidar_a1.launch.py"]
