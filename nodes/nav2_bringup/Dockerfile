# Nav2 2D navigation stack: reads odom, scan, IMU; publishes cmd_vel.
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

COPY config/nav2_params.yaml /app/config/nav2_params.yaml

# use_localization:=false -> no AMCL (no map required). Use odom + scan only.
# params_file points to our config (odom, scan, frame_ids).
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && exec ros2 launch nav2_bringup bringup_launch.py use_localization:=False params_file:=/app/config/nav2_params.yaml"]
