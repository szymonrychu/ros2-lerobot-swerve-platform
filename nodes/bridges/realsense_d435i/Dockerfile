# RealSense D435i bridge — publishes Image, Depth, PointCloud2, IMU via ros-jazzy-realsense2-camera.
# Run with --privileged for USB access (e.g. docker run --rm --network host --ipc host --privileged).
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-realsense2-camera \
    ros-jazzy-launch-ros \
    && rm -rf /var/lib/apt/lists/*

# Enable IMU (accel + gyro) and unite to /camera/imu for robot_localization.
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && exec ros2 launch realsense2_camera rs_launch.py enable_accel:=true enable_gyro:=true unite_imu_method:=linear_interpolation align_depth:=true"]
