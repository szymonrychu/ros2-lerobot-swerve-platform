# Robot localization EKF

Runs `robot_localization`'s `ekf_node` (node name `ekf_filter_node`) to fuse swerve wheel odometry (`/odom`) and the
BNO055 IMU (`/imu/data`). Publishes `/odometry/filtered` and is the **only** publisher of TF `odom -> base_link`
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
| `imu0: /imu/data` | fuses `vyaw` only | The BNO055 runs in IMUPLUS mode, so its heading has an arbitrary zero. Only the gyro yaw rate is fused, `imu0_relative: false`. To fuse absolute yaw, enable index 5 and set `imu0_relative: true` (the test enforces this pairing). |
| output | `/odometry/filtered` | robot_localization default topic. |

State vector order for the `*_config` lists: `x y z roll pitch yaw vx vy vz vroll vpitch vyaw ax ay az`.

To add the RealSense IMU, add `imu1: /camera/imu` and `imu1_config` to both the repo file and the Ansible block.
