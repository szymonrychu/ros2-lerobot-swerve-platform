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
- **bno095_imu/** — BNO085/BNO095 IMU bridge; publishes `sensor_msgs/Imu` on `/imu/data` with covariance for Nav2 (Client only).

Shared Python libraries used by multiple nodes live in [../shared/](../shared/). See [AGENTS.md](../AGENTS.md) for conventions (type hints, unit tests, rebuild on source change).
