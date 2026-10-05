# Swerve drive controller

ROS2 node that drives the 4-wheel independent-steering (swerve) platform from `geometry_msgs/Twist` on `/cmd_vel`. It runs inverse kinematics to produce steering positions and in-wheel velocities for the feetech bridge, and forward kinematics on the measured wheel states to publish `nav_msgs/Odometry` plus (optionally) the `odom` -> `base_link` TF.

**Audience:** Mid-level Python dev with ROS2 (rclpy, Twist, Odometry, JointState).

## Platform (see `Platform dimensions.md`)

| Wheel | In-wheel (drive) servo | Steering servo |
|---|---|---|
| front left (`fl`) | 32 | 33 |
| front right (`fr`) | 35 | 34 |
| back left (`rl`) | 39 | 38 |
| back right (`rr`) | 36 | 37 |

- Steering yaw axes: 305 mm long x 266.6 mm wide, so `half_length_m: 0.1525` and `half_width_m: 0.1333`. Wheel radius is 60 mm.
- Steering range is -90..+90 deg around straight ahead (servo limits 1024..3072 steps after `calibrate_servos.py center`).
- In-wheel servos have no angle limit and run in wheel (continuous rotation) mode.
- All 8 servos share the follower-arm serial bus. The `lerobot_follower` feetech bridge serves them as the `swerve_drive` extra group (see `nodes/bridges/feetech_servos/README.md`).

## Topics

- **Subscribes:** `/cmd_vel` (Twist: `linear.x` forward m/s, `linear.y` left m/s, `angular.z` CCW rad/s), `/swerve_drive/joint_states` (JointState from the bridge: steer positions in rad, drive velocities in rad/s).
- **Publishes:** `/swerve_drive/joint_commands` (exactly one JointState per control cycle, see below), `/odom` (Odometry), TF `odom` -> `base_link` (unless `publish_tf: false`).

**Combined command format:** `name` lists all 8 joints in config order. `position[i]` is the steering target (rad) for steer joints and `NaN` for drive joints. `velocity[i]` is the drive angular velocity (rad/s) for drive joints and `NaN` for steer joints.

**Pacing:** an rclpy timer runs the control step at exactly `control_loop_hz` (default 50 Hz). Subscription callbacks only store the latest message and never trigger extra cycles. The per-cycle logic lives in `control.py` (no rclpy imports, unit-tested).

Nothing is published until every steering and drive joint has been reported and the joint states are fresher than `joint_states_timeout_s`. If they go stale, publishing stops and the bridge's velocity watchdog stops the wheels.

## Kinematics (`kinematics.py`)

Body frame: x forward, y left, yaw CCW positive. Wheel `i` sits at `(x_i, y_i)` = (+-Lx, +-Ly).

- **Inverse kinematics:** wheel velocity `v_i = (vx - omega * y_i, vy + omega * x_i)`, so the heading is `atan2(v_i)` and the drive speed is `|v_i| / R`.
- **Steering range fold (`fold_to_steer_range`):** heading `a` at speed `s` is the same as `a + pi` at `-s`. With +-90 deg of steering, every heading has a reachable equivalent. Near +-90 deg the side closer to the current angle is used. A heading up to 0.1 rad past the limit keeps the wheel on its current side (clamped), so sideways driving does not flip the wheels back and forth. Driving backwards keeps the wheels straight and reverses the drive.
- **Common steering side (`choose_common_flip`):** the side is chosen for all moving wheels together, so they stay aligned at the +-90 deg limit. Choosing per wheel near +-90 deg used to leave some wheels at e.g. +88 deg driving forward and others at -88 deg driving in reverse, and single wheels swung 180 deg mid-turn. There are two group options: every wheel unflipped (heading `a`, drive `+s`) or every wheel flipped (heading `a + pi`, drive `-s`). An option is feasible when each moving wheel's candidate is inside the limit. A wheel may also go up to `STEER_LIMIT_HYSTERESIS_RAD` (0.1 rad) past the limit, clamped, on the side it is on now (a centred wheel counts as either side). Of the feasible options, the one with less total steering travel from the current targets wins. The previous option is kept unless the other saves more than `GROUP_FLIP_HYSTERESIS_RAD` (0.2 rad) of total travel, so the wheels do not chatter. The choice is stored in `ControlState.steer_flip` between cycles. When no group option is feasible (e.g. rotating in place, where the headings span more than 180 deg, or a turn whose headings straddle +-90 deg by more than the 0.1 rad tolerance), each wheel falls back to `fold_to_steer_range`. Away from the clamped band, every output still gives the commanded twist through forward kinematics.
- **Stopped wheels hold their heading** instead of swinging back to center (`compute_wheel_commands`).
- **Idle recentering:** after the commanded twist has been zero for `idle_recenter_s` (default 3 s), all steering targets return to 0 rad (straight ahead). Any new motion resets the timer; `idle_recenter_s: 0` disables it.
- **Desaturation:** if any wheel would exceed `max_wheel_angular_velocity_rad_s`, all wheels are scaled by the same factor, so the motion direction is kept.
- **No-propulsion safeguard:** a wheel's drive is zero while its steering error is above `steer_error_threshold_rad`.
- **Forward kinematics:** least-squares solution of the 8 wheel-velocity equations for `(vx, vy, omega)`, integrated with the midpoint heading (`integrate_odometry`). `forward_kinematics_with_residual` also returns the residual `r = ||A x - b|| / sqrt(5)` (m/s; normalised by the square root of the degrees of freedom, 8 - 3 = 5; the leave-one-out 6x3 systems use sqrt(3), so the two are comparable), about 0 when the four wheels agree on one rigid-body twist.
- **Slip handling (`robust_forward_kinematics`):** when `r` exceeds `slip_residual_threshold_mps` (default 0.05), the four leave-one-wheel-out 6x3 least-squares problems are solved; if the best one has a residual below half of `r`, its twist is used (the one slipping or stalled wheel is dropped) and its residual is reported, otherwise the full solution is kept. A wheel spinning on a small object therefore does not corrupt the odometry.
- **Odometry covariance:** the published twist covariance is `var_xy = 0.002 + r^2` and `var_yaw = 0.01 + (r / hypot(lx, ly))^2` (largest module radius), so the EKF trusts the wheels less exactly when they disagree. The small xy floor keeps a full stall (wheels say ~0.25 m/s, lidar says 0) well beyond the EKF's 3-sigma gate. When the measured (not commanded) wheel speeds are all below 0.005 m/s (parked), the fixed values `var_xy = 1e-3`, `var_yaw = 1e-4` are published instead, so a parked robot's heading is pinned and does not drift with the gyro bias. These floors and the EKF `odom0_twist_rejection_threshold` should be tuned from recorded `/odom` vs `/odom_rf2o_twist` data.

## Configuration

YAML config (path via `SWERVE_DRIVE_CONTROLLER_CONFIG` or `/etc/ros2/swerve_drive_controller/config.yaml`):

- `half_length_m`, `half_width_m`, `wheel_radius_m`: Geometry (defaults 0.1525, 0.1333, 0.06).
- `max_steer_angle_rad` (default pi/2), `max_wheel_angular_velocity_rad_s` (default 4.71, which is about 0.28 m/s).
- `slip_residual_threshold_mps` (default 0.05): forward-kinematics residual above which one slipping wheel is dropped (see Kinematics).
- `idle_recenter_s` (default 3.0): steer back to straight ahead after this long without motion (0 disables).
- `cmd_vel_timeout_s` (default 0.5): the twist is zeroed when `/cmd_vel` goes quiet. `joint_states_timeout_s` (default 0.5).
- `joint_names`: 8 names in order fl_drive, fl_steer, fr_drive, fr_steer, rl_drive, rl_steer, rr_drive, rr_steer.
- `cmd_vel_topic`, `joint_states_topic`, `joint_commands_topic`, `odom_topic`, `odom_frame_id`, `base_frame_id`.
- `publish_tf` (default true): set false to publish only `/odom` and leave the `odom` -> `base_link` TF to another node (e.g. a localization stack). Parsed strictly: YAML booleans, integers 0/1, and strings (case-insensitive) `true/yes/on/1` and `false/no/off/0` are accepted; any other value falls back to the default (true) rather than being coerced.
- `control_loop_hz` (default 50), `steer_error_threshold_rad`, `max_steer_angular_velocity_rad_s` (ST3215 is about 4.71 rad/s, for tuning).

## Driving manually

From any host on the client's ROS2 graph (for example on the client itself, with `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`):

```bash
# keyboard teleop (holonomic: hold Shift for strafing keys)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.1 -p turn:=0.3

# or single commands (repeat faster than cmd_vel_timeout_s, here 10 Hz)
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0}, angular: {z: 0.0}}"
```

Health check: `./scripts/swerve_diag.sh`.

## Steering calibration

With the wheel pointing exactly straight ahead (bridge stopped so the bus is free):

```bash
cd nodes/bridges/feetech_servos
poetry run python scripts/calibrate_servos.py center --device /dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A059004-if00 --id 33
# repeat for --id 34, 37, 38
```

## Build and run

Ansible deploys this node on the client as `swerve_controller`. Locally: `cd nodes/swerve_drive_controller && poetry install && poetry run python -m swerve_drive_controller` (with config and ROS2 sourced). Tests: `poetry run pytest tests/ -v`.
