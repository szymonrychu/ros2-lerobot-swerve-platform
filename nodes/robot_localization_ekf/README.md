# Robot localization EKF

Runs `robot_localization`'s `ekf_node` (node name `ekf_filter_node`) to fuse swerve wheel odometry (`/odom`), lidar
odometry (`/odom_rf2o_twist`, from `rf2o_laser_odometry` + `rf2o_odom_relay`) and the BNO055 IMU (`/imu/data`). Publishes `/odometry/filtered` and is the **only** publisher of TF `odom -> base_link`
(the swerve controller runs with `publish_tf: false`). Nav2 (`bt_navigator`, `controller_server`, `velocity_smoother`)
uses `/odometry/filtered`; slam_toolbox publishes `map -> odom` on top of this TF.

## Launch

Ansible starts the repo launch file:

```bash
ros2 launch /opt/ros2-lerobot-swerve-platform/nodes/robot_localization_ekf/launch/ekf.launch.py
```

`launch/ekf.launch.py` loads the parameter file named by env `ROBOT_LOCALIZATION_EKF_CONFIG`
(default `/etc/ros2/robot_localization_ekf/config.yaml`). Ansible writes that file from the `config: |` block of the
`robot_localization_ekf` entry in `ansible/group_vars/client.yml`.

## Config

`config/ekf.yaml` is the repo copy and must stay identical in content to the Ansible block
(`tests/test_nav_stack_config.py` compares them).

| Setting | Value | Why |
|---|---|---|
| `two_d_mode` | `true` | Planar robot; z, roll, pitch are ignored. |
| `world_frame` / `odom_frame` / `base_link_frame` | `odom` / `odom` / `base_link` | Continuous odometry frame; SLAM provides `map -> odom`. |
| `publish_tf` | `true` | EKF owns `odom -> base_link`. |
| `odom0: /odom` | fuses `vx`, `vy`, `vyaw` | Holonomic body velocities from swerve FK; `differential: false`, `relative: false` (velocities are not integrated twice). |
| `odom0_twist_rejection_threshold` | `1.5` | Mahalanobis gate on the wheel twist only. Wheels that stall or spin on a small object still report motion; once the lidar odometry disagrees the wheel twist is dropped. The innovation covariance is about the wheel covariance (`0.01 + slip residual^2`) plus the state covariance, so 1.5 rejects wheel-vs-state disagreements above roughly 0.18 m/s; a value of 3 or more would not trigger below the 0.28 m/s top speed. No rejection on rf2o or the IMU. |
| `odom1: /odom_rf2o_twist` | fuses `vx`, `vy` | Lidar odometry body velocities from `rf2o_odom_relay`; `differential: false`, `relative: false`. Yaw rate is not fused: the IMU gyro (fixed covariance 0.0004) stays the yaw-rate source, rf2o's yaw rate is noisy and degrades when turning in featureless surroundings. |
| `imu0: /imu/data` | fuses `vyaw` only | The BNO055 runs in IMUPLUS mode, so its heading has an arbitrary zero. Only the gyro yaw rate is fused, `imu0_relative: false`. To fuse absolute yaw, enable index 5 and set `imu0_relative: true` (the test enforces this pairing). |
| output | `/odometry/filtered` | robot_localization default topic. |

### Slip resilience

rf2o publishes **zero** twist covariance (robot_localization floors it to 1e-9, i.e. near-infinite trust), a laser-frame x speed (the lidar is mounted rotated 180 deg) and `vy = 0`. The EKF therefore never reads `/odom_rf2o` directly: `rf2o_odom_relay` rebuilds the body twist from the rf2o pose and publishes `/odom_rf2o_twist` with variances 0.02 (vx, vy) and 0.05 (yaw rate). The wheel covariance (`nodes/swerve_drive_controller`) grows with the wheel-consistency residual, so the lidar gains weight as the wheels start to slip, and the Mahalanobis gate drops the wheel twist completely in a full stall.

State vector order for the `*_config` lists: `x y z roll pitch yaw vx vy vz vroll vpitch vyaw ax ay az`.

To add the RealSense IMU, add `imu1: /camera/imu` and `imu1_config` to both the repo file and the Ansible block.
