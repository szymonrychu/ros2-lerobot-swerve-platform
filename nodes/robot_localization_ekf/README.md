# Robot localization EKF

Runs `robot_localization`'s `ekf_node` to fuse wheel odometry (`/odom` from swerve controller) and IMU (`/imu/data` from BNO055). Publishes `/odometry/filtered` and TF `odom` → `base_link`. Nav2 can use `/odometry/filtered` as the odometry source.

**Config:** Mount overrides at `ROBOT_LOCALIZATION_EKF_CONFIG` or use built-in `/app/config/ekf.yaml`. To add RealSense IMU, add `imu1: /camera/imu` and `imu1_config` in the YAML.
