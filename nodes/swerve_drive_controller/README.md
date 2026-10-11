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

- **Subscribes:** `/cmd_vel` (Twist: `linear.x` forward m/s, `linear.y` left m/s, `angular.z` CCW rad/s), `/swerve_drive/joint_states` (JointState from the bridge: steer positions in rad, drive velocities in rad/s), `/imu/data` (sensor_msgs/Imu, sensor-data QoS: BNO055 NDOF fused orientation for the tilt guard).
- **Publishes:** `/swerve_drive/joint_commands` (exactly one JointState per control cycle, see below), `/odom` (Odometry), TF `odom` -> `base_link` (unless `publish_tf: false`), `/tilt_guard/state` (std_msgs/String JSON, reliable + transient_local depth 1, see [Tilt guard](#tilt-guard)), and with `tilt_guard.error_action: torque_off` the bridge `/follower/set_register` (std_msgs/String JSON).

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
- **Odometry covariance:** the published twist covariance is `var_xy = 0.002 + r^2` and `var_yaw = 0.01 + (r / hypot(lx, ly))^2` (largest module radius), so the EKF trusts the wheels less exactly when they disagree. The small xy floor keeps a full stall (wheels say ~0.25 m/s, lidar says 0) well beyond the EKF's 3-sigma gate. When the measured wheel speeds are all below 0.005 m/s and no drive command is active (the latest cmd_vel is expired or inside the deadband), the robot is parked and the fixed values `var_xy = 1e-3`, `var_yaw = 1e-6` are published instead. The EKF weights the parked yaw rate as `var_odom / (var_odom + var_imu)` of the gyro bias; with the IMU yaw-rate variance 4e-4 that is 0.25% of the bias, so the measured -5.83e-4 rad/s BNO055 bias drifts the heading about 0.3 deg/h instead of 26 deg/h (which made slam_toolbox insert rotated scans while docked). An active command uses the normal variances, so a commanded rotation start is not fought by the tight covariance. These floors and the EKF `odom0_twist_rejection_threshold` should be tuned from recorded `/odom` vs `/odom_rf2o_twist` data.

## Tilt guard

`tilt_guard.py` (pure logic, unit-tested) protects the base against driving on a slope that can tip it over.

- **Attitude:** roll and pitch come from the `/imu/data` orientation quaternion (normalised, ZYX Euler with numpy), minus `level_reference_roll_deg` / `level_reference_pitch_deg` (the IMU reading when the base stands level; default roll 0.7 / pitch 1.3 deg, the BNO055 mounting bias measured on 2026-10-11 and the same values as mcp_server `floor_guard.imu_level_reference`. To re-measure: park on a level floor, read `roll_deg`/`pitch_deg` from `/tilt_guard/state` with both references at 0, and copy them into the config). `imu_link` is mounted aligned with `base_link`, so roll is about the x (forward) axis and pitch about the y (left) axis. A non-finite or zero quaternion, or `orientation_covariance[0] == -1` (REP 145: no orientation), is not a sample.
- **Asymmetric limits:** pitch tips the robot over its longer front/rear edge (305 mm vs 266.6 mm), so pitch gets the larger limits: defaults roll warning 8 / error 15 deg, pitch warning 12 / error 20 deg. For example roll 13 deg is a warning, roll 18 deg an error, pitch 13 or 18 deg a warning.
- **Levels:** `ok` (0), `warning` (1): consumers (claude_agent, web UI) get an interrupt, stop automatic (SLAM/Nav2) driving and tell the operator to drive back to level ground manually; the base still accepts `/cmd_vel`. `error` (2): the base drive is inhibited. The inhibition is latched: when an error falls back to `warning` the base stays inhibited (`base_drive_inhibited: true`), and it drives again only when the guard is back at `ok`, i.e. the tilt stayed below the warning recovery threshold `warn - hysteresis_deg` (roll 6 deg, pitch 10 deg by default) for `recovery_hold_s`. A warning entered from `ok` never inhibits.
- **Rise:** a level is entered only after the tilt stayed beyond its limit for `persistence_s` (0.5 s), so a bump (e.g. 0.2 s over a threshold) does not trip it. A tilt straight into the error band goes from ok to error directly.
- **Recovery:** a level clears only after the tilt stayed below `limit - hysteresis_deg` (2 deg) on both axes for `recovery_hold_s` (2 s); the guard then drops to the level those lowered limits give (error can drop straight to ok). Any sample back inside the band restarts the hold.
- **Stale IMU:** the IMU is stale when no valid sample arrived for `imu_timeout_s` (0.5 s). The level is held (an error never clears without fresh IMU data: the base motors stay disabled), pending persistence/recovery timers restart after the gap, the state shows `imu_ok: false`, and the node logs the stale and fresh-again transitions. A missing IMU never raises the level by itself.
- **Error enforcement:** while `base_drive_inhibited`, every received `/cmd_vel` is dropped (counted in `tilt_guard_inhibited_commands_total`), the latest command is cleared so nothing resumes after recovery, and each control cycle commands 0 rad/s on every drive joint with the steering held at its last targets (no idle recentering). `error_action: brake` (default) keeps the drive torque on, so the stalled wheels hold the robot on the slope. `error_action: torque_off` additionally writes `torque_enable 0` for the four drive joints through the bridge `set_register` topic on entering error and `torque_enable 1` when the inhibition clears (back at ok); the robot can then roll freely on a slope, which is why it is not the default.
- **Logging:** entering error is logged at error level, warning at warn level, clearing at info level, each with the state message.

### `/tilt_guard/state` contract

std_msgs/String, JSON object, QoS reliable + transient_local, depth 1 (a late subscriber gets the current state). Published on every level change at once and as a heartbeat at `state_publish_hz` (1 Hz). Nothing is published before the first valid IMU sample (no placeholder state).

| Key | Type | Meaning |
|---|---|---|
| `level` | string | `ok`, `warning` or `error` |
| `level_code` | int | 0, 1, 2 (same as the `tilt_guard_level` gauge) |
| `roll_deg`, `pitch_deg` | float | Last roll / pitch relative to the level reference, deg (signed) |
| `axis` | string or null | `roll` or `pitch`: the axis that caused the current level; null while ok |
| `base_drive_inhibited` | bool | True from entering `error` until back at `ok` (also in a `warning` that follows an error): drive commands are ignored and the wheels are stopped |
| `imu_ok` | bool | False when no valid IMU sample arrived for `imu_timeout_s`; the level is held |
| `since` | float | Epoch seconds at which the current level started (the first IMU sample while the level never changed) |
| `error_action` | string | `brake` or `torque_off` |
| `limits` | object | `roll_warn_deg`, `roll_error_deg`, `pitch_warn_deg`, `pitch_error_deg`, plus `hysteresis_deg` |
| `message` | string | Human-readable text, e.g. `tilt error: roll 18.0 deg beyond the 15.0 deg error limit; base drive motors disabled (brake) until the IMU reads below 6.0 deg roll for 2.0 s` |
| `stamp_ns` | int | ROS clock time of the publish, ns |

The shared contract consumers parse (mcp_server `models.TiltGuardState`, web_ui) is `level`, `roll_deg`, `pitch_deg`, `limits.{roll_warn_deg, roll_error_deg, pitch_warn_deg, pitch_error_deg}`, `since`, `base_drive_inhibited`, `imu_ok`; the other keys are extras.

Example:

```json
{"level": "warning", "level_code": 1, "roll_deg": 1.2, "pitch_deg": 13.4, "axis": "pitch",
 "base_drive_inhibited": false, "imu_ok": true, "since": 1760183996.9, "error_action": "brake",
 "limits": {"roll_warn_deg": 8.0, "roll_error_deg": 15.0, "pitch_warn_deg": 12.0, "pitch_error_deg": 20.0,
            "hysteresis_deg": 2.0},
 "message": "tilt warning: pitch 13.4 deg beyond the 12.0 deg warning limit; stop automatic driving and drive back to level ground manually",
 "stamp_ns": 1760184000000000000}
```

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
- `tilt_guard` (nested block, validated with pydantic; unknown keys or inconsistent limits make the whole config invalid and the node exits instead of running with default safety limits):

| Key | Default | Meaning |
|---|---|---|
| `enabled` | `true` | false never leaves ok (roll/pitch are still measured and published) |
| `imu_topic` | `/imu/data` | sensor_msgs/Imu input |
| `state_topic` | `/tilt_guard/state` | JSON state output |
| `torque_topic` | `/follower/set_register` | bridge set_register topic used by `torque_off` |
| `roll_warn_deg` / `roll_error_deg` | `8.0` / `15.0` | roll limits (shorter axis), warning must be below error |
| `pitch_warn_deg` / `pitch_error_deg` | `12.0` / `20.0` | pitch limits (longer axis) |
| `persistence_s` | `0.5` | tilt beyond a limit this long raises the level, s |
| `hysteresis_deg` | `2.0` | a level clears below `limit - hysteresis_deg`; must be below both warning limits |
| `recovery_hold_s` | `2.0` | tilt below the lowered limit this long clears the level, s |
| `imu_timeout_s` | `0.5` | IMU stale after this long without a valid sample, s |
| `level_reference_roll_deg` / `level_reference_pitch_deg` | `0.7` / `1.3` | IMU roll/pitch when the base is level (within +-30), deg |
| `error_action` | `brake` | `brake` (zero speed, torque on) or `torque_off` (drive torque off, can roll) |
| `state_publish_hz` | `1.0` | heartbeat rate of the state topic, Hz |

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
uv run python scripts/calibrate_servos.py center --device /dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A059004-if00 --id 33
# repeat for --id 34, 37, 38
```

## Metrics

Prometheus exporter on `127.0.0.1:19107/metrics` (ros2_nodes name `swerve_controller`), served by `ros2-metrics`
(`shared/ros2_metrics`) in a daemon thread. Config key `metrics_port` (top level of the config YAML; int, default unset =
exporter disabled; env `METRICS_PORT` is used when the key is absent). The helper also registers `robot_node_info{node}`
and `robot_node_start_time_seconds{node}`. Metric objects live in `metrics.py`.

| Metric | Type | Labels | Meaning |
|---|---|---|---|
| `swerve_cmd_vel_age_seconds` | gauge |  | Seconds since the last cmd_vel, every cycle; unset before the first message. |
| `swerve_joint_states_stale_total` | counter |  | Cycles skipped because the joint states were stale or incomplete (nothing is published then). |
| `swerve_slip_residual` | gauge |  | Wheel-consistency residual of the measured twist, m/s. |
| `swerve_loop_duration_seconds` | histogram |  | Duration of one timer cycle including publishing; buckets 0.2 ms to 100 ms. |
| `swerve_odom_published_total` | counter |  | Odometry messages published. |
| `tilt_guard_level` | gauge |  | Tilt guard level: 0 ok, 1 warning, 2 error (base drive inhibited). |
| `tilt_roll_deg` | gauge |  | Roll relative to the level reference, deg, per valid IMU sample. |
| `tilt_pitch_deg` | gauge |  | Pitch relative to the level reference, deg, per valid IMU sample. |
| `tilt_guard_inhibited_commands_total` | counter |  | cmd_vel messages dropped while the tilt guard inhibits the base drive. |

## Build and run

Ansible deploys this node on the client as `swerve_controller`. Locally: `cd nodes/swerve_drive_controller && uv sync && uv run python -m swerve_drive_controller` (with config and ROS2 sourced). Tests: `uv run pytest tests/ -v`.
