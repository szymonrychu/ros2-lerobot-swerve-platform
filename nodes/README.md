# Nodes

All ROS2 node source code lives here. Deployment is via Ansible: the repo is cloned on each target and uv venvs are synced (`uv sync --frozen --no-dev`) from these paths.

## Layout

- **ros2_master/** — ROS2 daemon (used by server and client).
- **master2master/** — Topic proxy (Client only).
- **bridges/uvc_camera/** — UVC camera bridge.
- **bridges/arducam_tof/** - Arducam ToF camera bridge: `/tof/depth/image_raw` (16UC1 mm), `/tof/amplitude/image_raw` (mono16), `/tof/camera_info`; never publishes placeholder frames; deployed disabled (Client only).
- **bridges/feetech_servos/** — Feetech servos bridge (configurable namespace, extra namespaces on the same bus, position or wheel/velocity mode per joint; leader/follower example configs).
- **leader_bridge/** — Leader arm teleop relay: `/leader/joint_states` (server) to `/filter/input_joint_updates` (filter_node leader input); replaces master2master's leader rule, never bypasses filter_node arbitration (Client only).
- **lerobot_teleop/** — Leader → follower teleop node (Client only).
- **filter_node/** — Modular joint command filter (e.g. Kalman) for follower teleop (Client only).
- **test_joint_api/** — REST API for joint updates feeding the same path as master2master (Client only).
- **topic_scraper_api/** — Dynamic ROS topic scraper + HTTP JSON API for runtime diagnostics (Client + Server).
- **bridges/bno055_imu/** — BNO055 IMU bridge; publishes `sensor_msgs/Imu` on `/imu/data` with covariance for Nav2 (Client only).
- **bridges/gps_rtk/** — GPS RTK bridge for LC29H-BS (base, Server) and LC29H-DA (rover, Client); publishes `sensor_msgs/NavSatFix`, streams RTCM3 over TCP.
- **bridges/rplidar_a1/** — RPLidar A1 bridge; publishes `sensor_msgs/LaserScan` on `/scan` (Client only).
- **bridges/realsense_d435i/** — RealSense D435i bridge (retired: the Ansible node is `present: false`, superseded by the Pi camera previews and the Arducam ToF).
- **head_camera_cpp/** - C++ (rclcpp + libcamera) node for the head Camera Module 3 (IMX708, CSI cam0) on the pan/tilt head: `/head_camera/image_raw[/compressed]` at 1152x648 and 30 fps, plus autofocused full-resolution zoom crops over `/head_camera/zoom/{command,image,result}` (JSON request/result, used by mcp_server `get_camera_image` and the web_ui Head tab click-to-zoom). Frame `head_camera_optical_frame`, Prometheus on 19116, background cgroup tier, builds the pinned libcamera fork. See `head_camera_cpp/README.md`.
- **pi_cameras/** - Retired: the node and its directory are gone (the Ansible entry is `present: false`, deploying it uninstalls the unit and `camera_ros` is purged). Superseded by `head_camera_cpp/`; the old front and back camera topics no longer exist.
- **swerve_drive_controller/** — Swerve controller: cmd_vel → steer positions + wheel velocities (IK, +-90 deg steering), odometry from FK with slip-aware wheel dropping and residual-based covariance (Client only).
- **static_tf_publisher/** — Static TF base_link → sensor frames (Client only).
- **robot_localization_ekf/** — EKF fuses swerve `/odom` + lidar `/odom_rf2o_twist` + BNO055 `/imu/data` → `/odometry/filtered` and the odom → base_link TF (Client only).
- **rf2o_laser_odometry/** — rf2o lidar odometry (source-built from a pinned upstream commit) on `/scan_filtered` → `/odom_rf2o`; no TF (Client only).
- **rf2o_odom_relay/** — Re-publishes rf2o as a body-frame twist with a real covariance on `/odom_rf2o_twist` for the EKF (Client only).
- **map_guard/** - SLAM map protection: scores each scan against the map and detects the robot being lifted, carried or rotated by hand; pauses slam_toolbox while disturbed, then resumes or backs up and resets the map (web_ui drops its GPS anchor); notifies the agent on `/robot_events` only while it is busy. Starts in dry run (Client only).
- **map_guard_cpp/** - C++ (rclcpp) port of `map_guard`, side by side: same topics, services, config and metrics port (19110). Ansible node type `map_guard_cpp` is `present: false` until the cut-over; the Python `map_guard` stays installed as the rollback. Built on the Pi from the repo tree (Client only).
- **laser_filter/** — Python lidar self filter removing returns on the robot body (polygon settable at runtime; mcp_server sets the dynamic footprint when the arm is released): `/scan` → `/scan_filtered` for SLAM, costmaps and collision monitor (Client only).
- **slam_toolbox/** — slam_toolbox online async SLAM from `/scan` + odometry TF: publishes `/map` and map → odom; saved posegraph in `/var/lib/ros2/maps` is reloaded at start (Client only).
- **nav2_bringup/** — Nav2 2D navigation stack with repo params: footprint-aware Smac lattice global planner (OMNI, H-shaped footprint) on the SLAM map, MPPI Omni controller for the swerve base (Client only).
- **haptic_controller/** — Force-feedback (resistance) and zero-G hold mode for leader gripper; gripper-only pilot (Client only).
- **steamdeck_ui/** — Touch-friendly Electron dashboard for SteamDeck (controller.ros2.lan). Camera preview, sensor/effector graphs, local nav map, GPS map, overlay bar. Python rclpy bridge subscribes to `/controller/*` topics and serves them via local WebSocket. Native app — no Docker.
- **web_ui/** — FastAPI + React + Three.js browser dashboard replacing steamdeck_ui. Serves config, URDF files, and topic data over HTTP/WebSocket; the Map tab shows the SLAM map, robot pose, global/local plans and sets Nav2 goals. Native app — no Docker.
- **mcp_server/** - Robot MCP server (Streamable HTTP on port 18200, bearer token): robot state, camera images, map summary, Nav2 navigation, drive, arm control via the filter_node autonomy lease, and /arm/home + /arm/set_home services, plus map frontiers (`list_frontiers`, `go_to_frontier`, numbered topdown layer; own numpy/scipy WFD); every camera image becomes a photo POI owned by poi_store (mcp_server writes the JPEG and sends the add; `list_lookups` / `get_lookup_image` / `annotate_lookup` / `delete_photos`) (Client only).
- **poi_store/** - Points/areas of interest on the map (name, note, status): JSON file store with latched `/poi/list`, `/poi/command` and `/poi/result` topics; also owns the camera photo index (photo POIs, latched `/poi/photos` summary, retention); edited by the agent (mcp_server) and the user (web_ui) (Client only).
- **map_manager/** - Named maps: save, list, load and delete bundles of the slam_toolbox pose graph, occupancy grid, POI set and last robot pose under `/var/lib/ros2/maps/named`; a load deserializes with a pose hint, replaces the POIs in poi_store and verifies the scan against the map (`/map_manager/*` topics) (Client only).
- **costmap_zones/** - Nav2 keepout and speed zones from area POIs: rasterises the open areas whose `zone` is `keepout` or `slow` (poi_store `/poi/list`) onto exactly the `/map` grid and publishes latched masks (`/costmap_zones/keepout_mask`: -1 outside, slow value (`slow_zone_mask_value`, default 60, a non-lethal path cost) in slow zones, 100 in keep-outs; `/costmap_zones/speed_mask`: 0 outside, speed percent inside) plus the `nav2_msgs/CostmapFilterInfo` for the KeepoutFilter and SpeedFilter (Client only).
- **claude_agent/** - Claude (Agent SDK, Opus) chat agent: the user types instructions in the web UI, the agent answers and drives the robot through mcp_server's tools; HTTP/WebSocket API on 127.0.0.1:18300, effector calls capped per instruction, OAuth token deployed by Ansible, runs as its own non-root user (Client only).

Shared Python libraries used by multiple nodes live in [../shared/](../shared/). See [CLAUDE.md](../CLAUDE.md) for conventions (type hints, unit tests, rebuild on source change).
