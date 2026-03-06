# Static TF publisher

Publishes static transforms from `base_link` to sensor frames (e.g. `imu_link`, `camera_link`, `laser_frame`) with configurable x, y, z, yaw offsets. Used so Nav2 and robot_localization see sensors in a consistent TF tree.

**Config:** YAML at `STATIC_TF_PUBLISHER_CONFIG` or `/etc/ros2/static_tf_publisher/config.yaml`.

Format:

```yaml
parent_frame: base_link
frames:
  - parent: base_link
    child: imu_link
    x: 0.0
    y: 0.0
    z: 0.0
    yaw: 0.0
  - child: laser_frame
    x: 0.1
    y: 0.0
    yaw: 0.0
```

Or shorthand: `imu_link: [0, 0, 0]`, `laser_frame: [0.1, 0, 0]` (x, y, yaw; z default 0).
