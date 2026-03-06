# Key decisions and agent notes for this project

(Record important decisions, conventions, and Cursor/LLM context here.)

**Agents:** Always read [AGENTS.md](AGENTS.md) when resuming work, when joining the project, or when context is summarized; mandatory rules and "when to read" are defined there.

## Preparation phase

* **Tooling**: Python 3 via mise; dependencies via uv and Poetry (ROADMAP had "UVC" — corrected to uv). Linting and testing setup follows [hikvision-doorbell](https://github.com/szymonrychu/hikvision-doorbell): pre-commit, flake8, black, isort, vulture, autoflake, pytest, coverage.
* **ROS2 distro**: TBD (e.g. Humble on 22.04 or Jazzy on 24.04). To be recorded when chosen; Docker base images and Ansible will use the same.
* **Layout**: All nodes live under **`nodes/`**; **`shared/`** holds shared Python libraries. No `client/` or `server/` directories; deployment is via Ansible (clone repo on node, build containers locally).
* **Container registry**: Images are built and pushed to `https://harbor.szymonrichert.pl/containers/<node name>` (e.g. `harbor.szymonrichert.pl/containers/client-ros2-master:latest`). Ansible builds containers locally on each node from the cloned repo; image names use this registry path for consistency.
* **Commits**: After each phase/iteration commit, push to upstream. When only one agent works on the codebase, push only to `main` (no feature branches).
* **Leader/follower arms**: The **feetech_servos** node (configurable via YAML: namespace, joint_names as list of `{ name, id }` per joint—explicit servo ID, no assumption IDs start at 1) publishes `/<namespace>/joint_states` and subscribes `/<namespace>/joint_commands`. Use example configs `config/leader.yaml` and `config/follower.yaml` for leader (Server) and follower (Client). Topic layout remains `/leader/...` and `/follower/...` when so configured. Optional lerobot_teleop node on Client: subscribes `/leader/joint_states` (proxied by master2master), publishes `/follower/joint_commands`. master2master supports `type: JointState` for sensor_msgs/JointState relay.

## Linting and Python nodes

* **Per-node Poetry**: Each Python node has its own `pyproject.toml` and `poetry.lock` (master2master, uvc_camera, feetech_servos, lerobot_teleop). Run `poetry install` and `poetry run poe lint` / `poetry run poe lint-fix` inside each node directory. From repo root: `poetry run poe lint-nodes` or `./scripts/lint-all-nodes.sh` to lint all nodes. Root `poe lint` covers only `tests`, `shared` (no `src` package).
* **Docker**: Node images use Poetry (no requirements.txt); Dockerfiles install poetry in a venv, then `poetry install --no-dev`.
* **Pre-commit**: Pre-commit only runs on files known to git. If nothing is tracked yet (e.g. before first commit), run `./scripts/run-pre-commit.sh` — it stages all files and runs all hooks. Then run `pre-commit install` for hooks on every commit.
* **Python conventions**: Prefer established libraries (e.g. Feetech servo library, pydantic for config); prefer file config over CLI; write unit tests. Per senior review: constants at top of file in SCREAMING_SNAKE_CASE; fewer private functions; imports at top, no try/except on imports; docstrings must include in/out variables with their types.
* **Commits**: Create a commit after each phase/iteration that passes; use semantic commit messages (feat:, fix:, docs:, etc.); the agent performs these commits.
* **Ansible and ROS2 nodes**: When ROS2 nodes or their configuration change, update Ansible so it stays in sync. Node list and config live in `group_vars/client.yml` and `group_vars/server.yml` under `ros2_nodes` and `ros2_node_type_defaults`. Schema: each node has `name`, `node_type` (maps to image/config via type defaults), `present` (default true; false triggers uninstall), `enabled` (default true; systemd enable/start vs disable/stop), optional `config` (string, node’s config file content), optional `env` (list of KEY=VAL). Ansible clones the repo from https://github.com/szymonrychu/ros2-lerobot-swerve-platform (revision configurable, default main) to a node directory, builds each node container from the repo locally (build_context per type, e.g. nodes/ros2_master), then installs or uninstalls the systemd unit and config dir per node.
* **Ansible lint and tests**: ansible-lint (run from `ansible/`: `ansible-lint .`) and playbook `--syntax-check` are part of the workflow. From repo root: `poetry run poe lint-ansible`, `poetry run poe test-ansible`. Config: `ansible/.ansible-lint` (profile, skip_list, warn_list). Pre-commit runs ansible-lint via a local hook that `cd`s into `ansible/` so roles_path resolves. Install ansible-lint (e.g. `pip install ansible-lint` or `pipx install ansible-lint`) for the hook to work.
* **Ansible network and hostname**: Roles `network` (netplan static IP: address, gateway, optional nameservers; detects primary interface) and `hostname` (hostnamectl + /etc/hostname, /etc/hosts). Run during provision when `network_address`, `network_gateway`, and/or `hostname` are set in group_vars or host_vars. ROS2 units: Server ros2_master uses `ROS_LOCALHOST_ONLY=0` (bind all); Server other nodes and all Client nodes use `ROS_LOCALHOST_ONLY=1`. Client master2master receives `ROS2_SERVER_HOST` (set `ros2_server_host` in client.yml) for connecting to Server’s ROS2 master.

## Repository upgrade (runtime and deploy)

* **master2master**: Single-process relay (one rclpy node, one MultiThreadedExecutor); no per-thread init/shutdown. Config validation: invalid direction/type raise `ConfigError`; topic names normalized (leading slash). Run from repo root with `mise exec --` when poetry is not on PATH (see README).
* **UVC camera bridge**: Real frame capture via OpenCV; publishes `sensor_msgs/Image` (bgr8). Env: `UVC_DEVICE`, `UVC_TOPIC`, `UVC_FRAME_ID`. Dependency: `opencv-python-headless`.
* **Teleop**: Configurable topics via `TELEOP_LEADER_JOINT_STATES_TOPIC`, `TELEOP_FOLLOWER_JOINT_COMMANDS_TOPIC`; explicit QoS profile.
* **Feetech**: Config rejects namespace containing `/` and empty joint names; set_servo_id catches OSError/ValueError/RuntimeError from ListServos.
* **Ansible**: `ros2_node_deploy` validates required vars (node_name, node_image, node_build_context, repo_dest); systemd unit has RestartSec=5, TimeoutStopSec=30, docker stop -t 25. Server node name for leader arm: `lerobot_leader` (group_vars/server.yml).

## Rules alignment and testability (post-review)

* **master2master**: Imports (`signal`, `rclpy`) at top of `proxy.py`; docstring Args fixed for `run_all_relays`. Extra tests: `normalize_topic` edge cases, `load_config` from file, `parse_rule_entry` invalid type.
* **UVC camera**: Config split to `config.py` (no ROS/OpenCV) for testability; `get_config` strips topic/frame_id and falls back to defaults. Tests in `nodes/bridges/uvc_camera/tests/test_config.py`; pytest `pythonpath = ["."]` so `config` resolves.
* **Lerobot teleop**: Config split to `config.py` (no ROS deps); tests in `nodes/lerobot_teleop/tests/test_config.py`; same pythonpath pattern.
* **Feetech**: Tests for `load_config_from_env`; `BridgeConfig` and master2master `TopicRule` docstrings (Attributes). Shared `clamp` tests: equal bounds, at bounds.
* **Node tests**: uvc_camera and lerobot_teleop need `poetry install --no-root` (or `poetry lock` then install) before first run; run from node dir with `poetry run poe test`.

## Topic scraper debug pipeline and Docker DNS

* **Monitoring retired**: Infrastructure monitoring role/playbook were removed from active Ansible deployment flow. Default runtime diagnostics now use ROS topic-level scraping.
* **topic_scraper_api (client + server)**: New ROS2 node under `nodes/topic_scraper_api` dynamically discovers topics, subscribes/unsubscribes at runtime, and exposes latest samples via HTTP (`/topics`, `/topics/<topic-path>`). Payload includes `received_at_ns`, `header_stamp_ns`, and `sample_seq` for jitter/skew analysis.
* **Collector workflow**: Use `scripts/topic_scraper_collect.py` with repeated `--source name=url` and `--select /topic:jq-filter` to emit merged NDJSON for cross-host dynamic comparisons (leader/follower/haptic loop topics).
* **Docker DNS**: Containers may fail to resolve (e.g. `ports.ubuntu.com`). The **docker** role deploys `/etc/docker/daemon.json` with `"dns": ["8.8.8.8", "8.8.4.4"]` (overridable via `docker_dns_servers`). Existing daemon.json is merged, not overwritten. Docker is restarted after the change.

## ROS2 cross-host relay discovery

* **master2master connectivity**: `ROS2_SERVER_HOST` env alone is not consumed by node code and does not configure DDS discovery by itself. For server↔client relay, use host networking and DDS discovery envs.
* **Working setup**: client `master2master` runs with `--network host`, `ROS_LOCALHOST_ONLY=0`, `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, `ROS_STATIC_PEERS=<server-ip>`; server `lerobot_leader` runs with `--network host`, `ROS_LOCALHOST_ONLY=0`, `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, `ROS_STATIC_PEERS=<client-ip>`.

## Leader-follower smoothness and local DDS transport

* **Container IPC for ROS2**: On client/server ROS2 containers, `--ipc host` is required with `--network host` to avoid FastDDS shared-memory isolation issues where discovery appears correct but local container-to-container payload delivery is unreliable.
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

* **bno095_imu** node: Client-only. Reads BNO055 over I2C (default bus 1, `/dev/i2c-1`); publishes `sensor_msgs/Imu` on `/imu/data` (configurable) with orientation (quaternion), angular velocity (rad/s), linear acceleration (m/s²), and configurable covariance matrices for Nav2. Config: `BNO095_IMU_CONFIG` or `/etc/ros2/bno095_imu/config.yaml`; keys: topic, frame_id (default `imu_link`), publish_hz (default 100), i2c_bus, i2c_address (default 0x28), orientation_covariance, angular_velocity_covariance, linear_acceleration_covariance. Driver: adafruit-circuitpython-bno055; on Linux, adafruit-extended-bus is used to select I2C bus by number. Container needs `--device=/dev/i2c-1:/dev/i2c-1` (or the bus in use).

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

## Swerve drive and Nav2 (client)

* **UVC camera**: Deployed with `enabled: false` when camera is unplugged; set `enabled: true` in client.yml when camera is in use.
* **swerve_drive_servos**: Second feetech_servos instance; namespace `swerve_drive`, 8 joints (fl/fr/rl/rr × drive+steer), device e.g. `/dev/ttyUSB1`. Same bridge code as follower; config defines joint names and IDs.
* **swerve_controller**: New node under `nodes/swerve_drive_controller`. Subscribes `/cmd_vel` (Twist), `/swerve_drive/joint_states`; publishes `/swerve_drive/joint_commands`, `/odom` (Odometry), TF odom→base_link. Forward and inverse kinematics for symmetric 4-wheel swerve (configurable half_length_m, half_width_m, wheel_radius_m). No-propulsion safeguard: zero drive when steer error exceeds `steer_error_threshold_rad`. ST3215 steering rate ~4.71 rad/s (0.222 s/60°); tune Nav2 max velocities accordingly.
* **static_tf_publisher**: Publishes static TF from base_link to imu_link, laser_frame (configurable offsets). Config: YAML with `frames` list or shorthand frame_id: [x, y, yaw].
* **robot_localization_ekf**: Container runs robot_localization ekf_node; fuses `/odom` (swerve) and `/imu/data` (BNO055); publishes `/odometry/filtered`. Config deployed as config.yaml (Ansible); env ROBOT_LOCALIZATION_EKF_CONFIG points to it. BNO055 assumed working for full fusion; EKF can run with odom + RealSense IMU if BNO055 unavailable.
* **nav2_bringup**: Container runs Nav2 with `use_localization:=false` (no map/AMCL); reads `/odom`, `/scan`, publishes `/cmd_vel`. Goal via `navigate_to_pose` action. Params in image; tune max velocities for swerve steering rate.
* **Debug script**: `scripts/swerve_goal_relative.py` sends a goal relative to current pose (--dx, --dy, --dtheta in robot frame) via navigate_to_pose action.

## System optimization (both RPis)

* **system_optimize** role: Debloats Ubuntu 24.04, tunes performance, reduces SD card wear, adds resilience for RPi. Integrated into provision playbooks (`server.yml`, `client.yml`, `site.yml`) and available standalone via `playbooks/optimize.yml`.
* **Debloat**: Removes snapd, cloud-init, ModemManager, open-vm-tools, open-iscsi, multipath-tools, udisks2, apport, pollinate, unattended-upgrades, ubuntu-advantage/pro, secureboot-db; masks avahi-daemon and bluetooth. WiFi (wpa_supplicant) stays enabled — both RPis use WiFi extensively.
* **Performance**: CPU governor `performance`, `vm.swappiness=0` (no swap), `vm.vfs_cache_pressure=50`, `vm.dirty_ratio=10`, `vm.min_free_kbytes=65536`, ROS2 DDS UDP buffers 8 MB, inotify limits raised, WiFi power management disabled for low-latency DDS.
* **SD card wear reduction**: Swap fully disabled (removed), root mounted `noatime` + `commit=600`, dirty writeback 15 s, `/tmp` and `/var/tmp` as tmpfs, journal volatile (RAM-only, `/var/log/journal` removed), apt daily timers disabled, core dumps disabled.
* **RPi-specific**: GPU memory 16 MB (headless), Bluetooth disabled (dtoverlay + service mask), HDMI blanked.
* **Resilience**: Hardware watchdog (`bcm2835_wdt`, 15 s timeout), `kernel.panic=10` auto-reboot, Docker log rotation (10 MB × 3 files).
* **LGTM monitoring stack removed**: Grafana, Loki, Mimir, Alloy containers, images, systemd units, compose files, and data dirs were manually cleaned from both hosts. No monitoring role remains in Ansible.
