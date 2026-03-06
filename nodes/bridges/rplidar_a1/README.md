# RPLidar A1 bridge

ROS2 node that publishes `sensor_msgs/LaserScan` on `/scan` from a Slamtec RPLidar A1 over USB serial.

## Attribution

Uses [rplidar_ros](https://github.com/Slamtec/rplidar_ros) (Slamtec, BSD-2-Clause) via the `ros-jazzy-rplidar-ros` apt package. This bridge adds a headless launch file (no RViz) and Docker integration.

## Features

- **Topic**: `sensor_msgs/LaserScan` on `/scan` (default).
- **Device**: USB serial (e.g. `/dev/ttyUSB0`). Map into the container with `--device=/dev/ttyUSB0:/dev/ttyUSB0`.
- **Launch args**: `serial_port`, `serial_baudrate` (115200 for A1), `frame_id` (default `laser`), `angle_compensate`, `scan_mode`, `inverted`.
- **Env**: `RPLIDAR_SERIAL_PORT` overrides default serial port at launch.

## Build and run

From repo root:

```bash
docker build -t rplidar_a1:latest nodes/bridges/rplidar_a1
docker run --rm --network host --device=/dev/ttyUSB0:/dev/ttyUSB0 rplidar_a1:latest
```

For stable device naming, use `/dev/serial/by-id/...` and pass the same path in `extra_args` and optionally set `RPLIDAR_SERIAL_PORT` in node env.

## Deploy

Ansible deploys this node on the client (RPi 5). See [ansible/README.md](../../../ansible/README.md) and `group_vars/client.yml` (node type `rplidar_a1`, `ros2_nodes` entry).
