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

## Scan watchdog

Ansible starts `scan_supervisor.py` instead of the launch file directly. It runs `ros2 launch launch/rplidar_a1.launch.py` as a child process and subscribes to `/scan`. If no scan arrives within 30 s of start, or scans stop for 5 s, it stops the child (SIGINT, then SIGKILL) and exits with status 1, so systemd (`Restart=on-failure`, 15 s delay) starts a fresh driver. Why: the A1 driver sometimes wedges in its device handshake after a restart. The process keeps running at about 20% CPU but never creates the `/scan` publisher (seen 2026-10-03). The decision logic is in `scan_watch.py` (unit-tested in `tests/test_rplidar_scan_watch.py`).

## Metrics

`scan_supervisor.py` and `scan_watch.py` run with the system `python3` (no venv): `prometheus_client` comes from the apt package `python3-prometheus-client` and `ros2_metrics` from the repo (`shared/ros2_metrics` on `PYTHONPATH`). The port comes from env `METRICS_PORT` (robot: `19106`, served on 127.0.0.1; unset disables metrics), node name `rplidar_a1`.

| Metric | Type | Meaning |
|--------|------|---------|
| `lidar_scans_total` | counter | `/scan` messages received |
| `lidar_scan_rate_hz` | gauge | Scan rate over the last 10 s (unset until two scans arrived) |
| `lidar_scan_gap_seconds_max` | gauge | Longest gap between scans in the last 60 s, including the gap still open (unset before the first scan) |
| `lidar_driver_restarts_total{reason}` | counter | Restarts requested by the supervisor: `startup` (no scan within the grace) or `gap` (scans stopped) |

The supervisor exits on a restart and systemd starts a new process, so the counters restart from zero; a restart is also visible as a change of `robot_node_start_time_seconds`.
