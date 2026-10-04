# laser_filter

Removes RPLidar returns that hit the robot itself, so SLAM, the Nav2 costmaps and the collision monitor do not treat the robot body as an obstacle.

**Audience:** Mid-level Python dev with ROS2.

- **Node:** `laser_filters` `scan_to_scan_filter_chain` (apt `ros-jazzy-laser-filters`), launched by Ansible as `laser_filter` on the client.
- **Topics:** subscribes `/scan` (RPLidar, frame `laser_frame`), publishes `/scan_filtered`.
- **Filter:** `LaserScanBoxFilter` in `base_link` removing everything inside the outer footprint (470 x 386 mm) plus a 1 cm margin: `config/footprint_filter.yaml`. Needs the static TF `base_link -> laser_frame`.
- **Why:** on the robot the lidar sees parts of the platform 0.22-0.24 m from the sensor. slam_toolbox's `min_laser_range` (0.15 m) does not remove them, so without the filter they become occupied cells around the robot and NavFn can only plan to the robot's own position.

Consumers: `slam_toolbox` (`scan_topic`), Nav2 global/local costmap obstacle layers and `collision_monitor` (`nodes/nav2_bringup/config/nav2_params.yaml`).
