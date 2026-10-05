# rf2o_laser_odometry

Lidar odometry with [rf2o_laser_odometry](https://github.com/MAPIRlab/rf2o_laser_odometry) (MAPIR, "Planar Odometry
from a Radial Laser Scanner", ICRA 2016). It estimates planar motion from consecutive `/scan_filtered` scans and
publishes `nav_msgs/Odometry` on `/odom_rf2o`. The robot uses it as a second translation source next to the wheel
odometry, because the wheels slip or stall on small objects while the lidar does not care.

Client only. Not in the Jazzy apt distribution, so Ansible builds it from source.

## Source

- Repo: `https://github.com/MAPIRlab/rf2o_laser_odometry`, branch `ros2` (the ROS2 port; `ros1` is the other branch).
- Pinned commit: `b38c68e46387b98845ecbfeb6660292f967a00d3` (tip of `ros2`, 2023-04-28). Change it in
  `ros2_node_type_defaults.rf2o_laser_odometry.colcon_source.commit` in `ansible/group_vars/client.yml`; the next
  deploy rebuilds.
- Build: `ansible/roles/ros2_node_deploy/tasks/colcon_source_build.yml` clones into `/opt/ros2-ws/src/rf2o_laser_odometry`
  and runs `colcon build --merge-install --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release` with
  `MAKEFLAGS=-j2` under `systemd-run --scope -p CPUQuota=100% -p IOWeight=10 nice -n 19 ionice -c 3` (the Pi overheats
  otherwise). Only rebuilt when the pinned commit changes (stamp file). See `ansible/README.md`.

## Parameters (`config/rf2o.yaml`)

Verified against upstream `CLaserOdometry2DNode.cpp`.

| Parameter | Value | Why |
|---|---|---|
| `laser_scan_topic` | `/scan_filtered` | Footprint-filtered scan; rf2o skips non-finite ranges. |
| `odom_topic` | `/odom_rf2o` | Raw rf2o output, consumed by `rf2o_odom_relay`. |
| `publish_tf` | `false` | The EKF owns `odom -> base_link`. |
| `base_frame_id` / `odom_frame_id` | `base_link` / `odom` | rf2o looks up `base_link -> laser_frame` (mounted rotated 180 deg) itself. |
| `init_pose_from_topic` | `""` | Start at the origin; a non-empty value makes it wait for a pose topic. |
| `freq` | `10.0` | Processing loop; the RPLidar A1 gives about 7-10 scans/s and rf2o only processes new scans. |

The launch command is `ros2 run rf2o_laser_odometry rf2o_laser_odometry_node --ros-args --params-file config/rf2o.yaml
-r __node:=rf2o_laser_odometry` (no launch file needed).

## What rf2o publishes (and why the EKF does not read it directly)

From the upstream source (`publish()`):

- Pose: `base_link` in the `odom` frame, with a valid yaw quaternion. **Covariance: all zeros** (pose and twist).
  robot_localization floors a zero variance to 1e-9, i.e. near-infinite trust, which would make the lidar override the
  wheels and the gyro even in a featureless corridor.
- Twist: `linear.x = acu_trans(0,2) / dt` is the speed along the **laser** x axis (the lidar is mounted rotated
  180 deg, so its sign is inverted against `base_link`), `linear.y` is hard-coded to `0.0` (no lateral speed for a
  holonomic robot), `angular.z` is the yaw difference over dt.

So the EKF fuses `/odom_rf2o_twist` from [`rf2o_odom_relay`](../rf2o_odom_relay/README.md) instead: the body twist
rebuilt from the pose difference, with a real covariance. See `nodes/robot_localization_ekf/README.md` for the fusion
settings (vx, vy only, Mahalanobis rejection on the wheel odometry only).

## Notes

- rf2o assumes a constant scan size (the RPLidar launch uses `angle_compensate`, and the box filter keeps the size).
- Not built or run in CI (no ROS on the dev machine): the Ansible wiring is covered by `tests/test_nav_stack_config.py`.
