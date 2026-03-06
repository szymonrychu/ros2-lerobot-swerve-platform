# Swerve drive controller

ROS2 node that subscribes to `geometry_msgs/Twist` (cmd_vel), computes inverse kinematics for a symmetric 4-wheel swerve drive, publishes `sensor_msgs/JointState` commands to the swerve feetech bridge, and publishes `nav_msgs/Odometry` (and `odom` → `base_link` TF) from forward kinematics.

**Audience:** Mid-level Python dev with ROS2 (rclpy, Twist, Odometry, JointState).

## Topics

- **Subscribes:** `/cmd_vel` (Twist), `/swerve_drive/joint_states` (JointState from feetech bridge).
- **Publishes:** `/swerve_drive/joint_commands` (JointState), `/odom` (Odometry), TF `odom` → `base_link`.

## Kinematics

- **Inverse kinematics (IK):** Body twist (vx, vy, omega) → per-wheel steering angle and drive angular velocity. Symmetric layout: half-length Lx, half-width Ly, wheel radius R.
- **Forward kinematics (FK):** Current wheel steer angles and drive velocities → body vx, vy, omega. Used for odometry integration.
- **No-propulsion safeguard:** When the difference between current and desired steering angle for a wheel exceeds `steer_error_threshold_rad`, that wheel’s drive command is set to zero until steering catches up, to avoid straining the mechanism.

## Configuration

YAML config (path via `SWERVE_DRIVE_CONTROLLER_CONFIG` or `/etc/ros2/swerve_drive_controller/config.yaml`):

- `half_length_m`, `half_width_m`, `wheel_radius_m`: Geometry (defaults 0.2, 0.15, 0.15).
- `joint_names`: List of 8 names (default: fl_drive, fl_steer, fr_drive, fr_steer, rl_drive, rl_steer, rr_drive, rr_steer).
- `cmd_vel_topic`, `joint_states_topic`, `joint_commands_topic`, `odom_topic`, `odom_frame_id`, `base_frame_id`.
- `control_loop_hz`, `steer_error_threshold_rad`, `max_steer_angular_velocity_rad_s` (ST3215 ~4.71 rad/s for path tuning).
- Optional `imu_offset_xyyaw`, `rplidar_offset_xyyaw` for static TF / docs.

## Servo limits

ST3215: 0.222 s/60° → ~4.71 rad/s max steering rate. Use `max_steer_angular_velocity_rad_s` and Nav2 tuning (max angular velocity, goal tolerance) so the platform does not command impossible motions.

## Build and run

From repo root, Ansible deploys this node on the client. Locally: `cd nodes/swerve_drive_controller && poetry install && poetry run python -m swerve_drive_controller` (with config and ROS2 sourced).
