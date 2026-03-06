# robot_localization EKF: fuse /odom + /imu/data -> /odometry/filtered
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-robot-localization \
    ros-jazzy-launch-ros \
    && rm -rf /var/lib/apt/lists/*

COPY launch/ekf.launch.py /app/launch/ekf.launch.py
COPY config/ekf.yaml /app/config/ekf.yaml

ENV ROBOT_LOCALIZATION_EKF_CONFIG=/app/config/ekf.yaml

CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && exec ros2 launch /app/launch/ekf.launch.py"]
