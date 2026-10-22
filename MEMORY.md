# Key decisions and agent notes for this project

(Record important decisions, conventions, and LLM context here.)

## Preparation phase

* **Tooling**: Python 3 via mise; dependencies via uv (the repo moved off Poetry on 2026-10-08, see "Poetry to uv migration"). Linting and testing setup follows [hikvision-doorbell](https://github.com/szymonrychu/hikvision-doorbell): pre-commit, ruff (replaced flake8/black/isort/autoflake), vulture, pytest, coverage.
* **ROS2 distro**: Jazzy on Ubuntu 24.04. Docker base images use `ros:jazzy-ros-base`; Ansible installs `ros-jazzy-*` packages.
* **Layout**: All nodes live under **`nodes/`**; **`shared/`** holds shared Python libraries. No `client/` or `server/` directories; deployment is via Ansible (clone repo on node, build containers locally).
* **Container registry**: Images are built and pushed to `https://harbor.szymonrichert.pl/containers/<node name>` (e.g. `harbor.szymonrichert.pl/containers/client-ros2-master:latest`). Ansible builds containers locally on each node from the cloned repo; image names use this registry path for consistency.
* **Commits**: After each phase/iteration commit, push to upstream. When only one agent works on the codebase, push only to `main` (no feature branches).
* **Leader/follower arms**: The **feetech_servos** node (configurable via YAML: namespace, joint_names as list of `{ name, id }` per joint—explicit servo ID, no assumption IDs start at 1) publishes `/<namespace>/joint_states` and subscribes `/<namespace>/joint_commands`. Use example configs `config/leader.yaml` and `config/follower.yaml` for leader (Server) and follower (Client). Topic layout remains `/leader/...` and `/follower/...` when so configured. Optional lerobot_teleop node on Client: subscribes `/leader/joint_states` (proxied by master2master), publishes `/follower/joint_commands`. master2master supports `type: JointState` for sensor_msgs/JointState relay.

## Linting and Python nodes

* **Per-node uv**: Each Python node has its own `pyproject.toml` and `uv.lock` (master2master, uvc_camera, feetech_servos, lerobot_teleop). Run `uv sync` and `uv run poe lint` / `uv run poe lint-fix` inside each node directory. From repo root: `uv run poe lint-nodes` or `./scripts/lint-all-nodes.sh` to lint all nodes. Root `poe lint` covers only `tests`, `shared` (no `src` package).
* **Docker**: Node images build from the node's `pyproject.toml` and `uv.lock` (no requirements.txt); use `uv sync --frozen --no-dev`.
* **Pre-commit**: Pre-commit only runs on files known to git. If nothing is tracked yet (e.g. before first commit), run `./scripts/run-pre-commit.sh` — it stages all files and runs all hooks. Then run `pre-commit install` for hooks on every commit.
* **Python conventions**: Prefer established libraries (e.g. Feetech servo library, pydantic for config); prefer file config over CLI; write unit tests. Per senior review: constants at top of file in SCREAMING_SNAKE_CASE; fewer private functions; imports at top, no try/except on imports; docstrings must include in/out variables with their types.
* **Commits**: Create a commit after each phase/iteration that passes; use semantic commit messages (feat:, fix:, docs:, etc.); the agent performs these commits.
* **Ansible and ROS2 nodes**: When ROS2 nodes or their configuration change, update Ansible so it stays in sync. Node list and config live in `group_vars/client.yml` and `group_vars/server.yml` under `ros2_nodes` and `ros2_node_type_defaults`. Schema: each node has `name`, `node_type` (maps to image/config via type defaults), `present` (default true; false triggers uninstall), `enabled` (default true; systemd enable/start vs disable/stop), optional `config` (string, node’s config file content), optional `env` (list of KEY=VAL). Ansible clones the repo from https://github.com/szymonrychu/ros2-lerobot-swerve-platform (revision configurable, default main) to a node directory, builds each node container from the repo locally (build_context per type, e.g. nodes/ros2_master), then installs or uninstalls the systemd unit and config dir per node.
* **Ansible lint and tests**: ansible-lint (run from `ansible/`: `ansible-lint .`) and playbook `--syntax-check` are part of the workflow. From repo root: `uv run poe lint-ansible`, `uv run poe test-ansible`. Config: `ansible/.ansible-lint` (profile, skip_list, warn_list). Pre-commit runs ansible-lint via a local hook that `cd`s into `ansible/` so roles_path resolves. Install ansible-lint (e.g. `pip install ansible-lint` or `pipx install ansible-lint`) for the hook to work.
* **Ansible network and hostname**: Roles `network` (netplan static IP: address, gateway, optional nameservers; detects primary interface) and `hostname` (hostnamectl + /etc/hostname, /etc/hosts). Run during provision when `network_address`, `network_gateway`, and/or `hostname` are set in group_vars or host_vars. ROS2 units: All nodes use `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`; only `master2master` (client) and `ros2-master`/`lerobot_leader` (server) set `ROS_STATIC_PEERS` for cross-host unicast. Every unit and `/etc/profile.d/ros2_dds.sh` get `ros2_dds_env` (`group_vars/all.yml`: `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`); there is no XML profile.

## Poetry to uv migration

* **2026-10-08**: Migrated every Python project from Poetry to uv 0.11.29 and from flake8/black/isort to ruff 0.15.20 (pinned in `mise.toml`). Python 3.12 is the floor (`requires-python >=3.12`) because the RPis run Ubuntu 24.04 with ROS2 Jazzy on the system Python 3.12. Node venvs on the Pis are built by `ros2_node_deploy` with `uv sync --frozen --no-dev` into `python3 -m venv --system-site-packages` venvs (so `rclpy` and apt `python3-*` libs stay visible); stamp `<venv>/.uv-deps`. Each node keeps its own `pyproject.toml` + `uv.lock`. Docs: `ansible/README.md` "Node venvs (uv)".

## Repository upgrade (runtime and deploy)

* **master2master**: Single-process relay (one rclpy node, one MultiThreadedExecutor); no per-thread init/shutdown. Config validation: invalid direction/type raise `ConfigError`; topic names normalized (leading slash). Run from repo root with `mise exec --` when uv is not on PATH (see README).
* **UVC camera bridge**: Real frame capture via OpenCV; publishes `sensor_msgs/Image` (bgr8). Env: `UVC_DEVICE`, `UVC_TOPIC`, `UVC_FRAME_ID`. Dependency: `opencv-python-headless`.
* **Teleop**: Configurable topics via `TELEOP_LEADER_JOINT_STATES_TOPIC`, `TELEOP_FOLLOWER_JOINT_COMMANDS_TOPIC`; explicit QoS profile.
* **Feetech**: Config rejects namespace containing `/` and empty joint names; set_servo_id catches OSError/ValueError/RuntimeError from ListServos.
* **Ansible**: `ros2_node_deploy` validates required vars (node_name, node_image, node_build_context, repo_dest); systemd unit has RestartSec=5, TimeoutStopSec=30, docker stop -t 25. Server node name for leader arm: `lerobot_leader` (group_vars/server.yml).

## Rules alignment and testability (post-review)

* **master2master**: Imports (`signal`, `rclpy`) at top of `proxy.py`; docstring Args fixed for `run_all_relays`. Extra tests: `normalize_topic` edge cases, `load_config` from file, `parse_rule_entry` invalid type.
* **UVC camera**: Config split to `config.py` (no ROS/OpenCV) for testability; `get_config` strips topic/frame_id and falls back to defaults. Tests in `nodes/bridges/uvc_camera/tests/test_config.py`; pytest `pythonpath = ["."]` so `config` resolves.
* **Lerobot teleop**: Config split to `config.py` (no ROS deps); tests in `nodes/lerobot_teleop/tests/test_config.py`; same pythonpath pattern.
* **Feetech**: Tests for `load_config_from_env`; `BridgeConfig` and master2master `TopicRule` docstrings (Attributes). Shared `clamp` tests: equal bounds, at bounds.
* **Node tests**: uvc_camera and lerobot_teleop need `uv sync` before first run; run from node dir with `uv run poe test`.

## Topic scraper debug pipeline and Docker DNS

* **Monitoring retired**: Infrastructure monitoring role/playbook were removed from active Ansible deployment flow. Default runtime diagnostics now use ROS topic-level scraping.
* **topic_scraper_api (client + server)**: New ROS2 node under `nodes/topic_scraper_api` dynamically discovers topics, subscribes/unsubscribes at runtime, and exposes latest samples via HTTP (`/topics`, `/topics/<topic-path>`). Payload includes `received_at_ns`, `header_stamp_ns`, and `sample_seq` for jitter/skew analysis.
* **Collector workflow**: Use `scripts/topic_scraper_collect.py` with repeated `--source name=url` and `--select /topic:jq-filter` to emit merged NDJSON for cross-host dynamic comparisons (leader/follower/haptic loop topics).
* **Docker DNS**: Containers may fail to resolve (e.g. `ports.ubuntu.com`). The **docker** role deploys `/etc/docker/daemon.json` with `"dns": ["8.8.8.8", "8.8.4.4"]` (overridable via `docker_dns_servers`). Existing daemon.json is merged, not overwritten. Docker is restarted after the change.

## ROS2 cross-host relay discovery

* **master2master connectivity**: `ROS2_SERVER_HOST` env alone is not consumed by node code and does not configure DDS discovery by itself. For server↔client relay, use host networking and DDS discovery envs.
* **Working setup (native, current)**: client `master2master` uses `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, `ROS_STATIC_PEERS=192.168.1.33`; server `lerobot_leader` uses `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, `ROS_STATIC_PEERS=192.168.1.34`. Discovery stays on the host via `ros2_dds_env` (no XML profile, no multicast between hosts). (Docker-era SUBNET config is obsolete.)

## Leader-follower smoothness and local DDS transport

* **DDS transport (native units)**: the FastDDS shared-memory transport is off (`FASTDDS_BUILTIN_TRANSPORTS=UDPv4` in `ros2_dds_env`): a SIGKILLed participant's SHM port is reused by its restart while peers write into the deleted segment. The old container `--ipc host` guidance no longer applies (nodes run natively).
* **Follower motion smoothing**: `feetech_servos` supports command interpolation (enabled by default) with per-joint smoothstep trajectories over `command_smoothing_time_s` (default `0.12`). This reduces choppy follower movement by sending gradual goal_position updates with ease-in/ease-out behavior.
* **Jitter suppression for precise motion**: Follower interpolation now supports paced retargeting (`interpolation_target_update_hz`) and target deadband (`command_deadband_steps`) so tiny source jitter does not continuously reset trajectories. Tune alongside `command_smoothing_time_s`.
* **Additional precision smoothing controls**: Follower path includes incoming-target low-pass (`target_lowpass_alpha`) plus commanded step-rate limiting (`max_goal_step_rate`) to reduce micro-jitter and stop-go effects during slow, precise teleop.
* **Continuous moving-setpoint tracking**: Follower interpolation now switches behavior based on source-command velocity. While source is moving (`source_motion_velocity_threshold_steps_s`), bridge uses higher retarget rate (`moving_target_update_hz`) and moving deadband (`moving_command_deadband_steps`, typically `0`) to avoid stop-and-go between mid-motion updates; while source is steady, it uses the normal retarget/deadband values to suppress jitter at rest.
* **Kalman-based smoothing/interpolation**: Follower command path now supports a constant-velocity Kalman tracker per joint (`kalman_enabled`) that estimates position+velocity from incoming setpoints and uses short-horizon prediction (`kalman_prediction_lead_s`) for continuous moving-target interpolation without hold-and-catch-up behavior.
* **Kalman continuous output mode**: In Kalman mode, follower now bypasses interpolation retarget gating and streams a fresh predicted goal every control-loop cycle, with velocity damping (`kalman_velocity_decay_per_s`) and bounded no-measurement prediction window (`kalman_max_prediction_time_s`) to avoid mid-move pauses and prevent long-horizon drift.

## Filter node and test joint API (client)

* **Feetech stripped of filtering**: The feetech_servos bridge no longer performs Kalman or interpolation; it applies incoming `joint_commands` directly to servo `goal_position`. Filtering is handled by a separate node.
* **filter_node (client)**: New node under `nodes/filter_node`. Subscribes to `/filter/input_joint_updates` (JointState), runs a configurable algorithm (e.g. `kalman`), publishes filtered JointState to `/follower/joint_commands`. Algorithm is pluggable via config (`algorithm`, `algorithm_params`). Kalman params: `process_noise_pos`, `process_noise_vel`, `measurement_noise`, `prediction_lead_s`, `velocity_decay_per_s`, `max_prediction_time_s`. **Delay compensation:** Increasing `prediction_lead_s` (e.g. 0.045–0.06 s) makes the output a short-horizon prediction that compensates for leader→client network delay; tune when topic_scraper or logs show skew or oscillations.
* **test_joint_api (client)**: REST API node under `nodes/test_joint_api`. `GET /joint-updates` returns latest posted joint map; `POST /joint-updates` accepts JSON (joint name → radians), publishes to `/filter/input_joint_updates` so updates go through the same path as master2master (filter_node → feetech). Use for testing; physical tests must be gripper-only (joint_5, joint_6).
* **Topic flow**: Server feetech publishes `/leader/joint_states` → master2master relays to `/filter/input_joint_updates`; test_joint_api can also publish to `/filter/input_joint_updates`. filter_node consumes that topic and publishes `/follower/joint_commands` → client feetech.
* **Utility script**: `scripts/joint_api_client.py` — GET or POST joint updates (e.g. `post --joint joint_6 0.15` or `post --file updates.json`). Base URL via `JOINT_API_BASE_URL` or `--base-url`. Documented in script docstring and [tests/README.md](tests/README.md).

## BNO055 IMU (client)

* **bno055_imu** node: Client-only. Reads BNO055 over I2C (default bus 1, `/dev/i2c-1`); publishes `sensor_msgs/Imu` on `/imu/data` (configurable) with orientation (quaternion), angular velocity (rad/s), linear acceleration (m/s²), and configurable covariance matrices for Nav2. Config: `BNO055_IMU_CONFIG` or `/etc/ros2/bno055_imu/config.yaml`; keys: topic, frame_id (default `imu_link`), publish_hz (default 100), i2c_bus, i2c_address (default 0x28), orientation_covariance, angular_velocity_covariance, linear_acceleration_covariance. Driver: adafruit-circuitpython-bno055; on Linux, adafruit-extended-bus is used to select I2C bus by number. Container needs `--device=/dev/i2c-1:/dev/i2c-1` (or the bus in use).

## Haptic controller (force-feedback and zero-G, client)

* **Force feedback currently disabled:** The haptic_controller node is deployed with `enabled: false` and `mode: off`. We will revisit force feedback (resistance / zero-G) in the future.
* **haptic_controller** node: Client-only. Gripper-only pilot for active resistance (force-feedback) and zero-G hold mode. Modes: `off` (no publish), `resistance` (oppose closing using follower load as proxy), `zero_g` (hold current leader pose). Subscribes to leader state (`/filter/input_joint_updates`) and follower state (`/follower/joint_states`); publishes to `/client/haptic_leader_commands`, relayed by master2master (direction **out**) to `/leader/joint_commands`. Follower bridge must set `publish_effort_joints: [joint_5, joint_6]` so `JointState.effort` carries `present_load` at control-loop rate. Leader torque must be enabled when using resistance or zero-G. Default deployment: present but **enabled: false**. Config: `HAPTIC_CONTROLLER_CONFIG` or `/etc/ros2/haptic_controller/config.yaml`; resistance gains: max_stiffness, load_deadband, max_step_per_cycle; watchdog_timeout_s for safety.
* **Leader gripper tuning for haptics**: To avoid leader arm feeling locked or oscillating, apply a gripper-only register profile to the leader servos (IDs 5 and 6). Profile: `nodes/bridges/feetech_servos/leader_gripper_haptic_profile.json` (PID + current only: p_coefficient, d_coefficient, i_coefficient, protection_current, max_torque_limit). Apply on the server with `calibrate_servos.py load-config --device <path> --file leader_gripper_haptic_profile.json`; then restart the leader feetech node. See feetech_servos README "Leader gripper tuning for haptics". At runtime only RAM registers (e.g. torque_enable) are accepted from ROS; EPROM writes from set_register are rejected.

## GPS RTK (LC29H-BS base, LC29H-DA rover)

* **gps_rtk** node: Single codebase, configurable `mode: base` (Server) or `mode: rover` (Client). Base: LC29H(BS) on native UART; configures module (MSM7, ref 1005, NMEA), reads mixed NMEA+RTCM3 stream, publishes `sensor_msgs/NavSatFix` on `/server/gps/fix`, serves RTCM3 on TCP port 5016. Rover: LC29H(DA); connects to base TCP, forwards RTCM3 to serial, reads NMEA, publishes `/client/gps/fix`. Serial handler uses byte-level state machine to split NMEA lines and RTCM3 frames (0xD3 + length + CRC24Q) from proprietary binary on native RPi UART. Config: `GPS_RTK_CONFIG` or `/etc/ros2/gps_rtk/config.yaml`; serial_port (e.g. `/dev/ttyAMA0` on RPi 4 with `dtoverlay=disable-bt`), baud_rate, topic, rtcm_tcp_port (base), rtcm_server_host/port (rover). One-time base survey-in: `scripts/calibrate_rtk_base.py` (run via `scripts/rtk_calibrate.sh` from your computer; uses `send_cmd_wait_for` so responses are matched to commands on a busy serial); completion = non-zero ECEF in status response; then power cycle. Deploy: Ansible adds `gps_rtk_base` (server) and `gps_rtk_rover` (client). Serial device: **server (RPi 4)** uses `/dev/ttyAMA0` (PL011 on GPIO 14/15 when `dtoverlay=disable-bt`); **client (RPi 5)** uses `/dev/ttyAMA0` (GPIO UART with `dtoverlay=uart0-pi5`). Playbooks ensure `enable_uart=1` in `/boot/firmware/config.txt`. Container needs `numpy` (sensor_msgs dependency); if gps_rtk_base restarts in a loop, remove stale container: `docker rm -f ros2-gps_rtk_base` then `systemctl start ros2-gps_rtk_base`.
* **GPS RTK post-calibration verification**: After base survey-in and power cycle, from a host with network access to both RPis: `./scripts/rtk_status.sh` (services + RTCM 5016), `./scripts/rtk_status.sh --fix` or `./scripts/rtk_verify.sh` for one-shot fix status; `./scripts/rtk_verify.sh --watch` to poll; `./scripts/rtk_verify.sh --capture 60` to capture and summarize. Client NavSatFix `status=2` = RTK fix (goal). Requires topic_scraper_api on server and client (port 18100).

## RPLidar A1 and RealSense D435i (client)

* **rplidar_a1** node: Client-only. Uses [rplidar_ros](https://github.com/Slamtec/rplidar_ros) (Slamtec, BSD-2-Clause) via `ros-jazzy-rplidar-ros`. Publishes `sensor_msgs/LaserScan` on `/scan`. Device: `/dev/ttyUSB0` (or `/dev/serial/by-id/...`); container `--device=/dev/ttyUSB0:/dev/ttyUSB0`. Headless launch in `nodes/bridges/rplidar_a1/launch/rplidar_a1.launch.py` (no RViz). Env `RPLIDAR_SERIAL_PORT` overrides default port.
* **realsense_d435i** node: Client-only. Uses [realsense-ros](https://github.com/realsenseai/realsense-ros) (Intel RealSense, Apache 2.0) via `ros-jazzy-realsense2-camera`. Publishes color, depth, point cloud, IMU (D435i). Launch enables accel/gyro and `unite_imu_method:=linear_interpolation` so unified `sensor_msgs/Imu` is on `/camera/imu` for robot_localization. Container runs with `--privileged` for USB 3.0 access.

## Servo bus split (arm alone, base on a second adapter)

* **Decision D4**: arm bus (`lerobot_follower`, IDs 1-6) stays at 100 Hz; base bus (`base_servos`, IDs 32-39) runs 50 Hz, equal to swerve_controller's loop. At 100 Hz the base would not fit while driving.
* **Decision D5**: bus budget test (`nodes/bridges/feetech_servos/tests/test_bus_budget.py`) is anchored on the measured shared-bus 95.7 Hz (10.45 ms cycle, 4.28 ms fake wire time): non-bus remainder 6.17 ms kept fixed. Projections: shared 14 servos 95.7 Hz idle / 66 Hz driving; arm only about 108 Hz capacity; arm + 2-servo head about 104 Hz; base + head about 69 Hz of capacity vs the 50 Hz target. Gate on the robot: `/follower/joint_states` >= 98 Hz (was 95).
* The head (pan/tilt) fits on either bus: base bus with 28 % headroom, or the arm bus (projected 9.61 ms < 10 ms).

## Swerve drive and Nav2 (client)

* **UVC camera**: Deployed with `enabled: false` when camera is unplugged; set `enabled: true` in client.yml when camera is in use.
* **base_servos**: Second feetech_servos instance (`node_name: base_servos`, unit `ros2-base_servos`); namespace `swerve_drive`, 8 joints (fl/fr/rl/rr × drive+steer, IDs 32-39) on its own adapter by `/dev/serial/by-id`, `control_loop_hz: 50`, battery reading off (`lerobot_follower` is the only `/battery_state` publisher), critical tier, metrics 19115. Same bridge code as the arm.
* **swerve_controller**: New node under `nodes/swerve_drive_controller`. Subscribes `/cmd_vel` (Twist), `/swerve_drive/joint_states`; publishes `/swerve_drive/joint_commands`, `/odom` (Odometry), TF odom→base_link. Forward and inverse kinematics for symmetric 4-wheel swerve (configurable half_length_m, half_width_m, wheel_radius_m). No-propulsion safeguard: zero drive when steer error exceeds `steer_error_threshold_rad`. ST3215 steering rate ~4.71 rad/s (0.222 s/60°); tune Nav2 max velocities accordingly.
* **static_tf_publisher**: Publishes static TF from base_link to imu_link, laser_frame (configurable offsets). Config: YAML with `frames` list or shorthand frame_id: [x, y, yaw].
* **robot_localization_ekf**: Container runs robot_localization ekf_node; fuses `/odom` (swerve) and `/imu/data` (BNO055); publishes `/odometry/filtered`. Config deployed as config.yaml (Ansible); env ROBOT_LOCALIZATION_EKF_CONFIG points to it. BNO055 assumed working for full fusion; EKF can run with odom + RealSense IMU if BNO055 unavailable.
* **nav2_bringup**: Container runs Nav2 with `use_localization:=false` (no map/AMCL); reads `/odom`, `/scan`, publishes `/cmd_vel`. Goal via `navigate_to_pose` action. Params in image; tune max velocities for swerve steering rate.
* **Debug script**: `scripts/swerve_goal_relative.py` sends a goal relative to current pose (--dx, --dy, --dtheta in robot frame) via navigate_to_pose action.

* **Frontier exploration (E5, mcp_server)**: frontiers are detected by our own vectorised WFD in numpy/scipy (`frontiers.py`: unknown-and-next-to-free mask plus `scipy.ndimage.label`, 8-connectivity) over `/map`, not by the `frontier_exploration_ros2` C++ library. Why: that library is source-only (colcon build, no apt package, no Python bindings), would need a new Ansible build path and an IPC layer, and brings its own exploration loop, which we discard because Claude ranks the frontiers; numpy and scipy are already pinned and Renovate-managed, no new node, no ROS traffic. A literal Python BFS WFD was rejected (about 100x slower on the Pi, same output). Ranking uses one map-geodesic Dijkstra (`scipy.sparse.csgraph`) from the robot over a downsampled traversability grid; Nav2 `ComputePathToPose` runs only for the single chosen observation pose in `plan_frontier_visit` (per-frontier Smac plans cost up to 2 s each on the busy Pi and preempt the BT planner; skipped while navigating). Ids are kept stable by Hungarian centroid matching; computation is lazy and cached per `/map` version (Pi CPU headroom). Budget: extract + geodesic p95 <= 300 ms, otherwise raise `frontiers.geodesic_downsample`. Front camera is uncalibrated, so "last lookup facing" assumes it looks along the base heading. Details: `nodes/mcp_server/README.md` (Frontiers).

## System optimization (both RPis)

* **system_optimize** role: Debloats Ubuntu 24.04, tunes performance, reduces SD card wear, adds resilience for RPi. Integrated into provision playbooks (`server.yml`, `client.yml`, `site.yml`) and available standalone via `playbooks/optimize.yml`.
* **Debloat**: Removes snapd, cloud-init, ModemManager, open-vm-tools, open-iscsi, multipath-tools, udisks2, apport, pollinate, unattended-upgrades, ubuntu-advantage/pro, secureboot-db; masks avahi-daemon and bluetooth. WiFi (wpa_supplicant) stays enabled — both RPis use WiFi extensively.
* **Performance**: CPU governor `performance`, `vm.swappiness=0` (no swap), `vm.vfs_cache_pressure=50`, `vm.dirty_ratio=10`, `vm.min_free_kbytes=65536`, ROS2 DDS UDP buffers 8 MB, inotify limits raised, WiFi power management disabled for low-latency DDS.
* **SD card wear reduction**: Swap fully disabled (removed), root mounted `noatime` + `commit=600`, dirty writeback 15 s, `/tmp` and `/var/tmp` as tmpfs, journal volatile (RAM-only, `/var/log/journal` removed), apt daily timers disabled, core dumps disabled.
* **RPi-specific**: GPU memory 16 MB (headless), Bluetooth disabled (dtoverlay + service mask), HDMI blanked.
* **Resilience**: Hardware watchdog (`bcm2835_wdt`, 15 s timeout), `kernel.panic=10` auto-reboot, Docker log rotation (10 MB × 3 files).
* **LGTM monitoring stack removed**: Grafana, Loki, Mimir, Alloy containers, images, systemd units, compose files, and data dirs were manually cleaned from both hosts. No monitoring role remains in Ansible.

## DNS and ROS2 hostnames

* **DNS override**: System DNS set via `primary_dns_server` (192.168.1.1) and `secondary_dns_server` (1.1.1.1) in group_vars; `network_nameservers` derived from them for netplan.
* **ROS2 hostnames**: `server.ros2.lan` (192.168.1.33) and `client.ros2.lan` (192.168.1.34). By default `ros2_extend_etc_hosts: false` — rely on primary DNS. Set `ros2_extend_etc_hosts: true` to have the hostname role add `ros2_hosts_entries` to `/etc/hosts` on both Pis.
* **Inventory**: Default `ansible_host` uses hostnames. When running Ansible from a dev machine, add to `/etc/hosts`: `192.168.1.33 server.ros2.lan`, `192.168.1.34 client.ros2.lan`.
* **ROS2 config**: `ROS_STATIC_PEERS`, `ros2_server_host`, `rtcm_server_host`, and script defaults use hostnames (server.ros2.lan / client.ros2.lan).

## SteamDeck UI (controller)

* **controller.ros2.lan** (192.168.1.35): SteamDeck running Ubuntu 24.04; native Electron app (no Docker). Fullscreen 1280×800, B&W/gray aesthetic.
* **Communication stack**: master2master on client RPi acts as the controlled gateway (like nginx). Only `/controller/*`-namespaced topics are relayed externally. SteamDeck bridge uses `ROS_STATIC_PEERS=client.ros2.lan` to discover only what master2master publishes. No rosbridge, no topic_scraper_api — pure ROS2 DDS.
* **Bridge architecture**: Python rclpy process (`bridge/bridge_server.py`) subscribes all config topics via DDS and serves `ws://localhost:9090`. Protocol: `{"type":"topic_data","topic":"...","data":{...}}` inbound; `{"type":"publish","topic":"...","msg_type":"...","data":{...}}` outbound. Image messages are JPEG-compressed (80%) via opencv before WebSocket transport.
* **Ansible inventory**: `[controller]` group added; provisioned via `playbooks/controller.yml` (hostname role + steamdeck_ui role); update-only via `playbooks/deploy_steamdeck_ui.yml`. `steamdeck_ui` role installs ROS2 Jazzy base, Node.js 20, Python bridge deps, Electron system deps, npm ci, deploys config to `/etc/steamdeck-ui/config.yaml`, desktop shortcut.
* **master2master extended**: Added 9 new message types: `imu`, `navsatfix`, `laserscan`, `occupancygrid`, `odometry`, `posestamped`, `image`, `compressedimage`, `twist`. Client m2m now relays `/controller/*` namespace (sensors out, goal_pose in). `ROS_STATIC_PEERS` on client: `server.ros2.lan;controller.ros2.lan`.
* **Future capabilities** (not implemented): SteamDeck controller inputs via node-hid/SDL2 → Twist; SteamDeck internal IMU via /dev/input/iio → Imu for LeRobot arm effector orientation; battery indicator via /sys/class/power_supply; offline OSM tile cache.

## mcp_server guard rules

* **Guard rules live in config.yaml, dry_run first**: motion safety rules (R1-R5: arm outside footprint, carrying without object footprint, arm retake from a human, wrist roll with a low arm, stretched shoulder overreach) are declarative `guard_rules:` in the mcp_server config (`ansible/group_vars/client.yml`), evaluated by the pure engine `nodes/mcp_server/mcp_server/guard_rules.py`, not hard-coded per tool. All ship as `mode: dry_run`; after a week of `mcp_guard_rule_hits_total` counters, flip rules to `enforce` one at a time (procedure in `ansible/README.md`). Recovery tools (stop, cancel_motions, release_control, arm_home, resume_motions) can never be gated.

## Grasp verification (E1)

* **Contact uses the absolute thresholds; load-rise (delta) contact is OFF** (`contact_load_delta: null` on every preset). Robot calibration 2026-10-11: 20 empty closes over the four presets rose 35-124 load units above the free-closing baseline at our closing speeds, so the paper's 20-unit delta (arXiv 2608.23983, measured at 5 mm/s) stopped every close half-open and no threshold separates closing from touching. The code path stays for a future speed-aware detector. `gripper_empty_stall_rad` needs retuning if pads or TPU tips are added.
* **Present_Current read rolled back**: lengthening the shared state SYNC_READ to 15 bytes (`publish_current_joints: [gripper]`) dropped the follower to 93.4 Hz / p99 18 ms (gate 95 Hz; 95.7 Hz / 12 ms before), so `publish_current_joints: []` (back at 95.6 Hz). Follow-up: a gripper-only single read every N cycles; `current_contact_delta_a` stays null until then.
* **Agent-as-judge fusion** (kept, verified on the robot): the band (empty/ambiguous/object from hold load and jaw position) is fused with the agent's `confirm_grasp` verdict on the jaw crop (yes/no/unsure). Held closes return `unverified` plus a crop; only fused `grasped` counts as success, `unverified` means look again or release and retry, `missed` means release_object. An empty close now closes fully: band empty, fused missed. The image alone never proves a grasp.

## Photos are POIs (E11)

- A camera image is a POI of kind `photo` owned by **poi_store**: poi_store is the only writer of the photo index and owns retention (max_records 3000 / 300 MB / 14 days, plus an hourly prune) and deletion (a delete removes the JPEG).
- The JPEG stays a file under `/var/lib/ros2/poi/lookups/`; mcp_server writes it and sends a small `add` with metadata over `/poi/command`. No image bytes on topics (same user and host for poi_store, mcp_server and web_ui).
- The existing append-only `index.jsonl` is the photo index (old lookup lines parse as photos, so the robot's history was kept without a rewrite). poi.json and `/poi/list` never contain photos: at 3000 photos `/poi/list` would be about 2.5 MB republished on every camera call. A latched `/poi/photos` summary (count, bytes, revision) tells clients to refresh; the web UI and agent page the index on disk.
- Photos survive the agent session clear ("review past work"); deleting a linked POI strips its id from the photos.
- mcp_server keeps `record_lookup`, the 32-hex lookup ids and the lookup tools as a facade, so the grasp jaw crop (E1, `full_frame=False`: no FOV), frontier facing (E5) and failure evidence (E8) did not change. New: `delete_photos`; `delete_poi` / `update_poi` take photo ids; each photo carries a `view` (camera position, yaw, pitch, hfov; the uncalibrated front camera uses the base heading and has no pitch or FOV, never invented).
- Rejected: photos inside poi.json (multi-MB rewrite per camera call), a virtual photo kind owned by mcp_server (the web UI could not delete), SQLite (not needed at this size).

## Stereo depth (IMX219 pair, client) - SUPERSEDED 2026-10-18 (removed with the Jetson, see "Jetson removal")

* Historical, superseded: the pair, its sources and its calibration are gone.
* **Two IMX219 on cam0/cam1 replace the IMX708 overview camera and the Arducam ToF** (2026-10-12). Why:
  - The IMX219 lens is fixed, while the IMX708's VCM lens drifts even in manual focus (gravity on the pan/tilt head, temperature, vibration), and any lens shift breaks the stereo calibration.
  - The Arducam ToF had not come up at the time (2026-10-12); this claim is stale: the ToF now works on CSI cam1.
  - The client has only about 0.3-0.45 of 4 cores free. So the pair runs at 480x360 (ISP-downscaled from the 1640x1232 binned mode) with SGBM 3WAY and 48 disparities, about 0.4 core at 5 fps, on the client rather than offloaded to the server.
* `nodes/stereo_depth` publishes depth on the ToF contract (`/stereo/depth/image_raw`: 16UC1 mm, 0 = invalid, plus `/stereo/camera_info`), so the Head tab needs no serializer change. The left camera keeps the `/overview_camera/*` topics.
* No calibration, a pair skew above `sync_slop_s` (2 ms; libcamera SyncMode keeps the sensors in step), a size mismatch or an all-invalid pair means nothing is published (logged and counted). Calibration lives in `/var/lib/ros2/camera_calibration/{stereo_left,stereo_right,stereo}.yaml` and is picked up without a restart.

## Jetson removal (2026-10-18)

* The Jetson coprocessor is removed from the robot; the `jetson/` tree, Jetson roles, playbook and deploy target are deleted.
* `stereo_depth` / `stereo_depth_cpp` sources are deleted; their retired `present: false` entries stay so deploying them uninstalls the units (D7). `jetson_bridge_cpp` is `present: false` with `colcon_purge`; `./scripts/deploy-nodes.sh client jetson_link` removes the client's link state (hosts line, sysctl, nftables).
* The web_ui Head tab shows the Arducam ToF depth + amplitude, with no RGB. `/front_camera` feeds the Arm tab and the mcp_server `front` camera. The Overview tab and stereo calibration are gone.
* The client keeps chrony with plain internet pools.
* The earlier "the ToF never came up" claim is corrected: the ToF works on CSI cam1; ToF frame capture on the Pi 5 kernel is a separate open issue.

## Named maps (map_manager)

- **Bundle** `/var/lib/ros2/maps/named/<name>/`: `map.posegraph` + `map.data` (slam_toolbox `serialize_map`), `occupancy.npz` (the `/map` grid, lossless), `bundle.json` (name, saved_at, note, `last_pose`, the POIs as stored, `poi_revision`, `map_info`). Saved into a `.tmp-` dir and renamed; overwrite needs a flag; delete removes the dir and its POIs. Name `^[A-Za-z0-9][A-Za-z0-9_-]{0,39}$`. Contract in `shared/ros2_common/maps.py`; topics `/map_manager/{command,result,status,localised}`.
- **POIs travel with the map**: load replaces the live POI set with exactly the bundle's (poi_store `replace`); clear is an explicit filtered op. **Photos stay one global store** (capture poses refer to the map active then).
- **Relocalisation** = `deserialize_map` with `START_AT_GIVEN_POSE` and the saved pose, hint refined first by scoring the live scan against the bundle grid (+-1 m, full yaw), new measurements paused. Verified slam_toolbox facts: the async node refuses `LOCALIZE_AT_POSE`; an unreadable file still returns success (check files, confirm by `/pose`); the first scan is processed even while paused; the match is never judged by slam_toolbox, so map_manager does.
- **Mapping mode, not localization mode**: localization needs another executable and a restart and stops extending the map; `START_AT_FIRST_NODE` assumes the saved start; AMCL needs a map server and particle filter this stack does not run.
- **Agreement threshold**: median of >= 5 samples in 5 s of the map_guard metric (shared `scan_agreement.py`) >= `localised_ratio` 0.6 (= map_guard `healthy_ratio`) and not ambiguous gives `localised`; otherwise map_manager stays paused and reports `not_localised` / `ambiguous`; map_guard_cpp holds on `/map_manager/localised = false`.
- **Boot unchanged**: unnamed Save map / Reset map and the `slam.launch.py` resume of `slam_map` are as before. Moved by hand: map_guard asks map_manager to relocalise first and only discards the live map and POIs when that fails.

## Head camera (2026-10-18)

- **Decision**: the head camera is the Camera Module 3 (IMX708, CSI cam0) plus the Arducam ToF (cam1) on the pan/tilt stand. `nodes/head_camera_cpp` (C++, libcamera) owns it and publishes `/head_camera/*` with frame `head_camera_optical_frame` (the node publishes no TF; the pan/tilt chain owns the parent of this frame). Default stream 1152x648 at 30 fps.
- **Zoom contract**: JSON request on `/head_camera/zoom/command` (`request_id`, `u`, `v`, `zoom` 2/4/8, `max_px` 256..1024, `af`), JPEG on `/head_camera/zoom/image` cut from the 4608x2592 still, JSON result on `/head_camera/zoom/result` (statuses ok, af_failed, busy, timeout, camera_error, invalid; `stamp_ns` equals the image stamp for pairing). Defined in `zoom_json.hpp`.
- **pi_cameras retired** (`present: false`, `camera_ros` purged, directory and tests deleted); head_camera_cpp builds the same pinned libcamera fork. The mcp_server `get_camera_image` camera value `front` is kept (it maps to `topics.head_camera`).

## ROS2 topic publishing

* **No placeholder data**: Only real, valid data from sensors or actual state may be published on ROS2 topics. Do not publish zeroed/coerced/synthetic data when reads fail or are invalid. If valid data is unavailable, skip publishing for that cycle. See CLAUDE.md "No Placeholder Data on Topics".

## Driving agent skills, workflows and retrospectives (2026-10-18)

- **Skills as a read-only plugin**: the agent's procedures (survey, sightseeing, planning, approach-and-verify, safe-pick-place, recover-from-failure, poi-categories, retrospective) are Agent SDK skills in the `robot-skills` plugin shipped in the claude_agent node. The checkout is root-owned and the service runs non-root, so the agent cannot edit them; changing a skill means a commit and a deploy.
- **Workflows via start_workflow**: discovery, drive_to_target and pick_place are YAML templates that `start_workflow` expands into the task plan (mandatory steps, skill hints); the plan limits still apply.
- **Retrospectives store + feed**: each job ends with a retrospective saved in a store and served as a feed to the web UI Retros tab; `scripts/agent_retrospectives.py` pulls them to `.logs/retrospectives/`.
- **POI categories as note tags**: categories are tags in the POI note, not a new schema field, so existing POIs and the store stay compatible.
- **Slow zones costed through the keepout mask**: slow zones are written into the keepout mask at a non-lethal value (default 60), so Nav2 path cost avoids them without a new filter; the speed mask still limits speed.
- **New session** only resets the conversation and job state; it never deletes POIs or maps. POI removal is by explicit buttons and tools (filter, mark, delete marked).

## Pan/tilt head (2026-10-18)

- **Decision**: the head is the `head` group (`head_pan` ID 20, `head_tilt` ID 21) on the base bus node `base_servos` (never the arm bridge), with `/head/joint_states` and `/head/joint_commands`. `head_state_publisher` runs robot_state_publisher on `nodes/web_ui/urdf/head.urdf`. Speed/acceleration caps (1.0 rad/s, 3.0 rad/s^2) are written with every goal.
- **Home pose**: pan 0, tilt +30 deg (looking down), not the servo centre 2048. It lives in four places (bridge `startup_position_rad`, mcp_server `head.home_*`, web_ui `head_home_deg`, URDF limits it must sit inside), pinned by `tests/test_head_config_consistency.py` together with ranges (pan +-90, tilt -30/+30 deg), joint names and topics.
- **Lease**: the head is under lease "any" (arm or base); home is commanded on the last release.
- **Signs (R3, confirmed 2026-10-18)**: pan positive = looking left (CCW from above); tilt positive = looking down. The tilt servo is mounted reversed (`inverted: true` on `head_tilt`, found 2026-10-20 after re-centring at 2048 = level), so the convention and URDF axis stay; tilt range is -30..+30 deg (steps 1707..2389).

## Head RGB-D fusion (2026-10-19)

- **Decision**: head ToF depth is registered on the client by `head_rgbd_cpp` into the head camera mapping image (576x324, default output 288x162) and published as exact-stamp pairs (RGB stamp on depth, camera_infos, RGB and cloud). RTAB-Map consumes the pair. The head URDF is the only extrinsic source (`scripts/head_rgbd_calibrate.py urdf` rewrites `tof_joint`); the node never reads the extrinsics file. Focus is locked only while the mapping stream has subscribers (the node subscribes lazily). Methodology and targets: `docs/head-rgbd-fusion-methodology.md`.
## 3D mapping (RTAB-Map on the server)

- **D1**: RTAB-Map runs in one component_container on the Pi 4b (`rtabmap_mapping`), installed from the split `ros-jazzy-rtabmap-*` apt packages, never the `rtabmap_ros` metapackage (pulls rtabmap_viz and Qt); the Pi 5 has no CPU headroom.
- **D2**: one client process, `map3d_uplink`, carries every cross-host map3d stream (lazy throttle/relay plus a compressedDepth republisher) from one contract file, `nodes/rtabmap_mapping/config/map3d_topics.yaml`; it is the third static-peer process, so each stream has one server reader.
- **D7**: RTAB-Map's odometry is slam_toolbox's `map -> base_link` (`odom_frame_id: map`), no own loop closure or scan links and no TF publishing, so its 3D map stays aligned to slam_toolbox's `/map`.
- **D8**: server caps are `rtabmap_mapping` 200% CPU / 900M and `map3d_manager` 20% / 128M, with no feature extraction and a 400-node working memory so the leader arm and the monitoring stack are never starved.
- **D13**: the lidar reaches the 3D map through slam_toolbox's `/map` walls (shown as lidar-derived structure, `source: lidar_2d` for the agent), not through RTAB-Map's fusion of scan and depth points.
- Pointers: `nodes/rtabmap_mapping/README.md`, `nodes/map3d_uplink/README.md`, `nodes/map3d_manager_cpp/README.md`, design `docs/plans/in-progress/rtab-map-rgb-d-3d-mapping-on/design.md`.

## Head ToF 3D obstacles (2026-10-19)

- **Decision**: the head ToF depth becomes obstacles and cliffs through `tof_obstacle_cloud` (rtabmap_util `point_cloud_xyz`, `/tof/points`), the `laser_filter` self filter (`/tof/points_filtered`), an STVL (`spatio_temporal_voxel_layer`) `tof_voxel_layer` in the local costmap only, and two collision_monitor pointcloud sources (`tof_obstacles` into StopBox, `tof_cliff` into CliffStop).
- **Height bands, not obstacles_detection**: floor and overhang are separated by fixed bands in base_link (obstacles 6 cm to 0.6 m, cliffs -1 m to -10 cm; STVL and collision monitor use the same numbers), not by rtabmap's `obstacles_detection`; the head TF comes from the pan/tilt joint states.
- **Lidar layers stay separate**: the lidar `obstacle_layer` keeps its own grid, so ToF clearing never erases lidar cells (U5); slam_toolbox keeps `map -> odom` and no ToF node publishes TF.
- **Fail-safe**: a ToF or head-TF outage makes the collision_monitor sources invalid and stops motion; `tof_obstacles.enabled` and `tof_cliff.enabled` are runtime switches.
- **CPU budget**: the pipeline adds at most 0.15 core mean on the client (new unit plus nav2_bringup and laser_filter deltas), EKF keeps 30 Hz; measured with `scripts/node_cpu_compare.py`.
- Wiring is pinned across files by `tests/test_tof_obstacle_wiring.py`.

## Ground plane from the head ToF (2026-10-20)

- **Decision**: `ground_estimator_cpp` fits the floor plane from the 10 Hz ToF (`/tof/points_filtered`) with an IMU prior and publishes plane-relative `/ground/obstacle_points` and `/ground/drop_points`, plus `/ground/labelled_points`, `/ground/state` and `/ground/warning_areas`.
- **Nav2**: the plane-relative bands feed a second STVL layer (`ground_voxel_layer`) alongside the fixed bands; the fixed bands remain active while the runbook switch-over criteria are checked. The collision monitor keeps the 10 Hz ToF bands and the global costmap stays static + lidar.
- **Obstacle vs warning**: a cluster is an obstacle when its step exceeds one third of the wheel diameter (0.04 m) or its slope exceeds 8 deg (the lower tilt_guard warn limit); anything below both limits is a warning, applied as a costmap_zones slow-down (speed mask + local non-lethal ground cost filter), also for colour-flagged flat objects.
- **mcp_server**: `floor_guard` is opt-in. **CPU budget**: 0.10 core for the node (Pi 5), +0.05 core for nav2_bringup. Runbook: `docs/runbooks/ground-estimation.md`; docs pinned by `tests/test_ground_docs.py`.

## Printable metadata markers (2026-10-19)

- **Decision**: fixed places (base, charger, drop-off, button, switch, tool station) are marked with printed QR Model 2 symbols carrying the P1 text payload `P1/K:<KIND>/N:<NAME>/S:<MM>[/KEY:VAL]`, not AprilTag/ArUco. QR carries the meaning and the true side length itself (no id table), is printable on paper and in two colours on the Bambu AMS, and zxing-cpp plus OpenCV IPPE give a full 6-DoF pose from 24 refined finder corners. AprilTag `tagStandard41h12` via `apriltag_ros` is the documented alternative if field range proves insufficient (it replaces only the decode and pose stages).
- **POIs**: the detector (`nodes/marker_detector`) sends `upsert_marker` to `poi_store`, which keeps one `marker` POI per `marker.key` (`<kind>/<N>`), refines its pose by confidence-weighted mean, replaces it on relocation and never overwrites user name/note/status. Automatic zoom requests are controlled by `zoom.auto_zoom` (default `false`; set `true` to enable).
- **ChArUco spec owned by the generator**: the 6x5, 40 mm board lives in the `charuco` block of `scripts/markers_generate.yaml`; `scripts/head_rgbd_calibrate.yaml` must equal it (pinned by a test). Guide: `docs/markers.md`.

## Head RGB-D calibration on the robot (2026-10-19)

- **Decision**: head RGB-D calibration is captured, computed and applied on the robot from the web UI Calibrate page (`/var/lib/ros2/head_rgbd_calibration/applied`); Apply restarts the head nodes (head_camera_cpp, head_rgbd_cpp, head_state_publisher). Repo files under `nodes/*/config` and `nodes/web_ui/urdf/head.urdf` only seed an empty applied dir. The CLI stays for bags, validation captures and offline calibration.

## Servo torque off and neutral (2026-10-20)

- **Decision**: servo neutral lives in the servo EEPROM and is re-zeroed from the web UI Calibrate page inside the running bridge (torque off, record neutral, re-enable torque + save); follower arm only; an arm re-zero leaves `/var/lib/ros2/arm/calibration_stale.json` (joint_offsets and home stale). Robot runbook: `docs/runbooks/servo-torque-neutral.md`.

## Head remount: ToF z-depth and ground noise (2026-10-21)

- **Decision**: after the pan/tilt mount reprint the SDK depth is radial (not z); arducam_tof converts it to z in arducam_tof with the calibrated K and publishes that K. The head_pan servo is mounted reversed (`inverted: true`, +pan = left, CCW from above). ground_estimator_cpp takes a per-cell (10 cm) temporal median of plane-relative heights before segmentation. The ToF obstacle band (STVL tof_mark and collision monitor tof_obstacles) is 10 cm. The obstacle vs warning rule unchanged (max step 0.04 m, max slope 8 deg). Robot runbook: `docs/runbooks/head-remount-floor-check.md`.

- CPU budget (2026-10-21): client saturated (~3.7 cores); decisions: rclpy EventsExecutor per node with `executor: single_threaded` config rollback, IMU 25 Hz, head camera 15 fps, client Alloy scrape 15 s; measured by `docs/runbooks/client-cpu-budget.md`.

## Ground detection robustness (2026-10-21)

- **Box**: the 5 cm box stays an obstacle under the 1/3-wheel rule (0.04 m); it must be classified stably.
- **Ramp**: `traversability.max_slope_deg` 15 deg (was 8 deg), bounded by the tilt_guard error limits (`roll_error_deg` 15, not the warn limits), because the user calls the ~14 deg ramp (25 cm long, 63 mm high) a safe skew, so a warning.
- **Noise**: a per-cell temporal spread gate (`temporal.max_spread_m`, gated cells unknown), a median (not p90) edge step, robust slope with `slope_known`, per-cell class hysteresis (`hysteresis.*`, 3 of 5 frames to enter, 0.3 s lethal latency), `lethal_min_points` 2, `fit.smoothing` 0.2.
- **Colour**: colour-mismatch clusters are warning-only, never lethal.
- **Costmap**: costmap_zones keeps a cell-keyed store of warning areas and publishes only on change.
- Runbook (static robot, report script over topic_scraper captures): `docs/runbooks/ground-robustness.md`. Details: `nodes/ground_estimator_cpp/README.md` (Robustness).
