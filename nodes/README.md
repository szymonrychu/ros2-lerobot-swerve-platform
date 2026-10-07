# Nodes

All ROS2 node source code lives here. Deployment is via Ansible: the repo is cloned on each target and Poetry venvs are installed from these paths.

## Layout

- **ros2_master/** — ROS2 daemon (used by server and client).
- **master2master/** — Topic proxy (Client only).
- **bridges/uvc_camera/** — UVC camera bridge.
- **bridges/feetech_servos/** — Feetech servos bridge (configurable namespace, extra namespaces on the same bus, position or wheel/velocity mode per joint; leader/follower example configs).
- **lerobot_teleop/** — Leader → follower teleop node (Client only).
- **filter_node/** — Modular joint command filter (e.g. Kalman) for follower teleop (Client only).
- **test_joint_api/** — REST API for joint updates feeding the same path as master2master (Client only).
- **topic_scraper_api/** — Dynamic ROS topic scraper + HTTP JSON API for runtime diagnostics (Client + Server).
- **bridges/bno055_imu/** — BNO055 IMU bridge; publishes `sensor_msgs/Imu` on `/imu/data` with covariance for Nav2 (Client only).
- **bridges/gps_rtk/** — GPS RTK bridge for LC29H-BS (base, Server) and LC29H-DA (rover, Client); publishes `sensor_msgs/NavSatFix`, streams RTCM3 over TCP.
- **bridges/rplidar_a1/** — RPLidar A1 bridge; publishes `sensor_msgs/LaserScan` on `/scan` (Client only).
- **bridges/realsense_d435i/** — RealSense D435i bridge (retired: the Ansible node is `present: false`, replaced by `overview_camera/`).
- **overview_camera/** — Launch-only node: one Raspberry Pi Camera Module 3 (IMX708, autofocus) on CSI cam0 mounted overhead (camera_ros on a source-built libcamera fork); `/overview_camera/image_raw`, `/overview_camera/image_raw/compressed` (640x480, 15 fps), `camera_info` (Client only).
- **swerve_drive_controller/** — Swerve controller: cmd_vel → steer positions + wheel velocities (IK, +-90 deg steering), odometry from FK with slip-aware wheel dropping and residual-based covariance (Client only).
- **static_tf_publisher/** — Static TF base_link → sensor frames (Client only).
- **robot_localization_ekf/** — EKF fuses swerve `/odom` + lidar `/odom_rf2o_twist` + BNO055 `/imu/data` → `/odometry/filtered` and the odom → base_link TF (Client only).
- **rf2o_laser_odometry/** — rf2o lidar odometry (source-built from a pinned upstream commit) on `/scan_filtered` → `/odom_rf2o`; no TF (Client only).
- **rf2o_odom_relay/** — Re-publishes rf2o as a body-frame twist with a real covariance on `/odom_rf2o_twist` for the EKF (Client only).
- **laser_filter/** — laser_filters box filter removing lidar returns on the robot body: `/scan` → `/scan_filtered` for SLAM, costmaps and collision monitor (Client only).
- **slam_toolbox/** — slam_toolbox online async SLAM from `/scan` + odometry TF: publishes `/map` and map → odom; saved posegraph in `/var/lib/ros2/maps` is reloaded at start (Client only).
- **nav2_bringup/** — Nav2 2D navigation stack with repo params: NavFn global planner on the SLAM map, MPPI Omni controller for the swerve base (Client only).
- **haptic_controller/** — Force-feedback (resistance) and zero-G hold mode for leader gripper; gripper-only pilot (Client only).
- **steamdeck_ui/** — Touch-friendly Electron dashboard for SteamDeck (controller.ros2.lan). Camera preview, sensor/effector graphs, local nav map, GPS map, overlay bar. Python rclpy bridge subscribes to `/controller/*` topics and serves them via local WebSocket. Native app — no Docker.
- **web_ui/** — FastAPI + React + Three.js browser dashboard replacing steamdeck_ui. Serves config, URDF files, and topic data over HTTP/WebSocket; the Map tab shows the SLAM map, robot pose, global/local plans and sets Nav2 goals. Native app — no Docker.
- **mcp_server/** - Robot MCP server (Streamable HTTP on port 18200, bearer token): robot state, camera images, map summary, Nav2 navigation, drive, arm control via the filter_node autonomy lease, and /arm/home + /arm/set_home services (Client only).
- **poi_store/** - Points/areas of interest on the map (name, note, status): JSON file store with latched `/poi/list`, `/poi/command` and `/poi/result` topics; edited by the agent (mcp_server) and the user (web_ui) (Client only).
- **claude_agent/** - Claude (Agent SDK, Opus) chat agent: the user types instructions in the web UI, the agent answers and drives the robot through mcp_server's tools; HTTP/WebSocket API on 127.0.0.1:18300, effector calls capped per instruction, OAuth token deployed by Ansible, runs as its own non-root user (Client only).

Shared Python libraries used by multiple nodes live in [../shared/](../shared/). See [CLAUDE.md](../CLAUDE.md) for conventions (type hints, unit tests, rebuild on source change).
