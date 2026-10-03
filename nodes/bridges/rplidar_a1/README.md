# RPLidar A1 bridge

ROS2 node that publishes `sensor_msgs/LaserScan` on `/scan` from a Slamtec RPLidar A1 over USB serial.

## Attribution

Uses [rplidar_ros](https://github.com/Slamtec/rplidar_ros) (Slamtec, BSD-2-Clause) via the `ros-jazzy-rplidar-ros` apt package. This bridge adds a headless launch file (no RViz).

## Features

- **Topic**: `sensor_msgs/LaserScan` on `/scan` (default).
- **Device**: USB serial (e.g. `/dev/ttyUSB0`). The device path is configured in `group_vars/client.yml` via `extra_args`; with native install the process runs as the ansible_user who has system group membership.
- **Launch args**: `serial_port`, `serial_baudrate` (115200 for A1), `frame_id` (default `laser_frame`), `angle_compensate`, `inverted`.
- **Env**: `RPLIDAR_SERIAL_PORT` overrides the default serial port, `RPLIDAR_FRAME_ID` the default `frame_id`.
- **Frame**: `/scan` is stamped `laser_frame`, the child of the static TF `base_link -> laser_frame` published by `static_tf_publisher` (`group_vars/client.yml`). The two must match, otherwise slam_toolbox, the Nav2 costmaps and the collision monitor cannot transform the scan and drop it. `tests/test_nav_stack_config.py` checks this.

## Build and run

Deploy with `scripts/deploy-nodes.sh client rplidar_a1` from the repo root.

For stable device naming, use `/dev/serial/by-id/...` and pass the same path in `extra_args` and optionally set `RPLIDAR_SERIAL_PORT` in node env.

## Deploy

Ansible deploys this node on the client (RPi 5). See [ansible/README.md](../../../ansible/README.md) and `group_vars/client.yml` (node type `rplidar_a1`, `ros2_nodes` entry).
