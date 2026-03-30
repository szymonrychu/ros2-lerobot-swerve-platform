# Hardware bridges (client)

Reusable ROS2 nodes that expose hardware as standard ROS2 topics. Each bridge type is implemented once and instantiated per device via env and Ansible config (one systemd service per instance). Target reader: mid-level Python dev with ROS2.

## Pattern

- **Device:** Set via env (e.g. `UVC_DEVICE=/dev/video0`) so the service can access the host device.
- **Topic (and other options):** Set via env (e.g. `UVC_TOPIC=/camera_0/image_raw`).
- **One node type, many services:** Same node type can run multiple times with different env/device (e.g. `uvc_camera_0` and `uvc_camera_1` via Ansible-deployed systemd services).

## Bridges

| Bridge        | Directory           | Status   | Env / notes                                      |
|---------------|---------------------|----------|--------------------------------------------------|
| UVC camera    | [uvc_camera/](uvc_camera/README.md) | Implemented (stub) | `UVC_DEVICE`, `UVC_TOPIC`; device mapping in compose |
| Feetech servos | [feetech_servos/](feetech_servos/README.md) | Implemented (stub) | Config: namespace + joint_names as list of { name, id } (explicit servo ID per joint); example leader/follower configs |
| RealSense D435i | [realsense_d435i/](realsense_d435i/README.md) | Implemented | ros-jazzy-realsense2-camera; USB 3.0, `--privileged` |
| RPLidar A1   | [rplidar_a1/](rplidar_a1/README.md) | Implemented | ros-jazzy-rplidar-ros; `/dev/ttyUSB0`, LaserScan `/scan` |
| IMU (BNO055)  | [bno055_imu/](bno055_imu/README.md) | Implemented | Client only; I2C, Nav2 covariance                |
| GPS-RTK       | [gps_rtk/](gps_rtk/README.md) | Implemented | LC29H-BS (base) on Server, LC29H-DA (rover) on Client; NavSatFix + RTCM3 TCP |

When adding a new bridge: add a directory under `bridges/` with Python (or other) code and a `pyproject.toml`, document it in this README and in a local README, and add the node type and build context to Ansible `group_vars` and the deploy role so it is deployed and run on target nodes.

## Rebuild rule

After editing any source used by a bridge node, re-run the Ansible deploy playbook on the target node so the repo is updated and the Poetry venv is reinstalled (see [CLAUDE.md](../../CLAUDE.md)).
