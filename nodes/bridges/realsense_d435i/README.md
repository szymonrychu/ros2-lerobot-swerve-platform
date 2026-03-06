# RealSense D435i bridge

ROS2 node that publishes color, depth, point cloud, and (on D435i) IMU from an Intel RealSense D435i over USB 3.0.

## Attribution

Uses [realsense-ros](https://github.com/realsenseai/realsense-ros) (Intel RealSense, Apache 2.0) via the `ros-jazzy-realsense2-camera` apt package.

## Features

- **Topics**: Color (`/camera/color/image_raw`), depth (`/camera/depth/image_rect_raw`), point cloud, camera info; D435i also exposes IMU (gyro/accel).
- **Device**: USB 3.0. The container is run with `--privileged` for reliable USB access; device-specific mounts can be refined later.
- **Launch**: Default `rs_launch.py` (enable_color, enable_depth true). Override with launch args via config or env if needed.

## Build and run

From repo root:

```bash
docker build -t realsense_d435i:latest nodes/bridges/realsense_d435i
docker run --rm --network host --ipc host --privileged realsense_d435i:latest
```

## Deploy

Ansible deploys this node on the client (RPi 5). See [ansible/README.md](../../../ansible/README.md) and `group_vars/client.yml` (node type `realsense_d435i`).
