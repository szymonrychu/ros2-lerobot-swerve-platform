# Nodes

All ROS2 node source code and Dockerfiles live here. Deployment is via Ansible: the repo is cloned on each node and containers are built locally from these paths.

## Layout

- **ros2_master/** — ROS2 daemon (used by server and client).
- **master2master/** — Topic proxy (Client only).
- **bridges/uvc_camera/** — UVC camera bridge.
- **bridges/feetech_servos/** — Feetech servos bridge (configurable namespace; leader/follower example configs).
- **lerobot_teleop/** — Leader → follower teleop node (Client only).
- **filter_node/** — Modular joint command filter (e.g. Kalman) for follower teleop (Client only).
- **test_joint_api/** — REST API for joint updates feeding the same path as master2master (Client only).
- **topic_scraper_api/** — Dynamic ROS topic scraper + HTTP JSON API for runtime diagnostics (Client + Server).
- **bno055_imu/** — BNO055 IMU bridge; publishes `sensor_msgs/Imu` on `/imu/data` with covariance for Nav2 (Client only).
- **bridges/gps_rtk/** — GPS RTK bridge for LC29H-BS (base, Server) and LC29H-DA (rover, Client); publishes `sensor_msgs/NavSatFix`, streams RTCM3 over TCP.
- **bridges/rplidar_a1/** — RPLidar A1 bridge; publishes `sensor_msgs/LaserScan` on `/scan` (Client only).
- **bridges/realsense_d435i/** — RealSense D435i bridge; color, depth, point cloud, unified IMU `/camera/imu` (Client only).
- **swerve_drive_controller/** — Swerve controller: cmd_vel → joint commands, FK/IK, odometry (Client only).
- **static_tf_publisher/** — Static TF base_link → sensor frames (Client only).
- **robot_localization_ekf/** — EKF fuse odom + IMU → `/odometry/filtered` (Client only).
- **nav2_bringup/** — Nav2 2D navigation stack container (Client only).
- **haptic_controller/** — Force-feedback (resistance) and zero-G hold mode for leader gripper; gripper-only pilot (Client only).
- **steamdeck_ui/** — Touch-friendly Electron dashboard for SteamDeck (controller.ros2.lan). Camera preview, sensor/effector graphs, local nav map, GPS map, overlay bar. Python rclpy bridge subscribes to `/controller/*` topics and serves them via local WebSocket. Native app — no Docker.

Shared Python libraries used by multiple nodes live in [../shared/](../shared/). See [AGENTS.md](../AGENTS.md) for conventions (type hints, unit tests, rebuild on source change).
