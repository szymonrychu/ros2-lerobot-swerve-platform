# Hardware bridges (client)

Reusable ROS2 nodes that expose hardware as standard ROS2 topics. Each bridge type is implemented once and instantiated per device via env and Ansible config (one systemd service per instance). Target reader: mid-level Python dev with ROS2.

## Pattern

- **Device:** Set via env or the node config (e.g. `RPLIDAR_SERIAL_PORT=/dev/serial/by-id/...` for `rplidar_a1`) so the service can access the host device.
- **Topic (and other options):** Set via env or the node's config file in `group_vars` (`ros2_nodes` `config`).
- **One node type, many services:** Same node type can run multiple times with different env/device (e.g. `feetech_servos` as `lerobot_leader`, `lerobot_follower` and `base_servos`, each an Ansible-deployed systemd service).

## Bridges

| Bridge        | Directory           | Status   | Env / notes                                      |
|---------------|---------------------|----------|--------------------------------------------------|
| Feetech servos | [feetech_servos/](feetech_servos/README.md) | Implemented (stub) | Config: namespace + joint_names as list of { name, id } (explicit servo ID per joint); example leader/follower configs |
| RealSense D435 | [realsense_d435/](realsense_d435/README.md) | Implemented | Config only: stock `realsense2_camera` (ros-jazzy-realsense2-camera) on the head D435 (USB 3.0, no IMU), `/head/d435/*`; udev rule applied by its node deploy |
| RPLidar A1   | [rplidar_a1/](rplidar_a1/README.md) | Implemented | ros-jazzy-rplidar-ros; `/dev/ttyUSB0`, LaserScan `/scan` |
| IMU (BNO055)  | [bno055_imu/](bno055_imu/README.md) | Implemented | Client only; I2C, Nav2 covariance                |
| GPS-RTK       | [gps_rtk/](gps_rtk/README.md) | Implemented | LC29H-BS (base) on Server, LC29H-DA (rover) on Client; NavSatFix + RTCM3 TCP |

When adding a new bridge: add a directory under `bridges/` with Python (or other) code and a `pyproject.toml`, document it in this README and in a local README, and add the node type and build context to Ansible `group_vars` and the deploy role so it is deployed and run on target nodes.

## Rebuild rule

After editing any source used by a bridge node, re-run the Ansible deploy playbook on the target node so the repo is updated and the uv venv is re-synced (see [CLAUDE.md](../../CLAUDE.md)).
