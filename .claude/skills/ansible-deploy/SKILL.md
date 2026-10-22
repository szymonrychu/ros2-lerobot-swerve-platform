---
name: ansible-deploy
description: >
  Use when deploying ROS2 nodes to client/server RPis, redeploying after code changes,
  updating node configuration, or troubleshooting deployment failures.
  MANDATORY: always use this skill before running any Ansible deploy. Never run
  ansible-playbook directly — always use scripts/deploy-nodes.sh.
---

# Ansible Deploy

**Never run `ansible-playbook` directly.** All deploys go through `scripts/deploy-nodes.sh`.
Never SSH manually to restart services — Ansible handles everything.

---

## Quick Reference

**Always log deploy output to `.logs/`** (gitignored) instead of system temp dirs:

```bash
mkdir -p .logs

# Single node
./scripts/deploy-nodes.sh client web_ui 2>&1 | tee .logs/deploy-client.log
./scripts/deploy-nodes.sh server lerobot_leader 2>&1 | tee .logs/deploy-server.log

# Several nodes: ONE ansible run with --tags web_ui,filter_node,bno055_imu (never parallel runs)
./scripts/deploy-nodes.sh client web_ui filter_node bno055_imu 2>&1 | tee .logs/deploy-client.log

# All nodes on a target (includes full verify)
./scripts/deploy-nodes.sh client --all 2>&1 | tee .logs/deploy-client-all.log
./scripts/deploy-nodes.sh server --all 2>&1 | tee .logs/deploy-server-all.log

# Partial runs by phase tag (--tags / --skip-tags pass through with --all)
./scripts/deploy-nodes.sh client --all --tags config,restart   # configs/units/launchers only, then restart
./scripts/deploy-nodes.sh client --all --skip-tags build,verify
```

Phase tags: `sync`, `apt`, `python` (uv), `build` (npm/colcon), `config` (config files, units, launchers),
`boot` (firmware overlays, reboot), `setup` (dirs/tokens/users), `restart`, `verify`, `always`; each node's steps are
also tagged with the node name. Details: `ansible/README.md`, "Deploy tags". Builds, uv syncs and restarts only
happen for what changed (stamps); a no-change deploy restarts nothing. The log ends with a per-task timing recap.

Client deploys first wait (up to 15 min) for an idle robot agent (not busy, quiet 5 min) and fail if it stays in use;
override with `-e ros2_deploy_ignore_agent=true` (appended to the deploy-nodes.sh command).

---

## Playbook Structure

```
ansible/playbooks/
  deploy_nodes_client.yml       # all client nodes (explicit per-node steps, each tagged with the node name)
  deploy_nodes_server.yml       # all server nodes (same)
  tasks/
    repo_sync.yml               # shared: clone/update repo on target
    apt_nodes.yml               # shared: one batched apt install for the nodes of the run
    resolve_and_deploy.yml      # shared: resolve ros2_nodes entry + call role
    start_ros_nodes.yml         # last step: restart queued nodes, start stopped ones, gradually
```

There are no per-node playbooks: `deploy-nodes.sh <target> a b` runs the target playbook once with `--tags a,b`.
The playbook syncs the repo once, deploys the selected nodes, restarts what changed and verifies.

---

## Choosing Between Single, Parallel, and All

| Scenario | Command |
|----------|---------|
| Changed one node's code or config | `deploy-nodes.sh <target> <node>` |
| Changed multiple independent nodes | `deploy-nodes.sh <target> node1 node2 node3` |
| First deploy, major refactor, or unknown scope | `deploy-nodes.sh <target> --all` |
| After Ansible role/config structure changes | `deploy-nodes.sh <target> --all` |
| Server monitoring stack (Mimir + Loki + Grafana at `http://server.ros2.lan/grafana/`, nginx on port 80, Alloy for the server's own metrics and journal; not a ROS node, never part of a node deploy). Deploy it before the client | `deploy-nodes.sh server monitoring` |
| Robot monitoring agent (Alloy pushing metrics to the server's Mimir and the journal to its Loki, nginx on port 80 in front of the web UI with `/grafana/` redirected to the server; removes the robot's old Prometheus + Grafana; not a ROS node) | `deploy-nodes.sh client monitoring` |
| Changed `roles/system_optimize` (sysctl, journald, watchdog, debloat, persistent Wi-Fi power save off; not a ROS node, tagged `never` so `--all` and node tags skip it; restarts no node) | `deploy-nodes.sh client system` and/or `deploy-nodes.sh server system` |
| Changed a node's metrics port or `monitoring_agent_node_targets` (group_vars/client.yml; the robot's Alloy scrapes our nodes' `/metrics`) | the node(s), then `deploy-nodes.sh client monitoring` |
| Changed a dashboard (`roles/monitoring/files/*.json`), the Mimir/Loki/Grafana config or the server's slice | `deploy-nodes.sh server monitoring` |

A node list is one tag-filtered run, so there is nothing to parallelise. Servo nodes (`lerobot_follower`,
`lerobot_leader`) are restarted only when their files changed, together with the other changed nodes at the end.

---

## Node Lists

### Client nodes (RPi5)
| Node | Notes |
|------|-------|
| `fastdds_discovery_server` | Removed (present: false); deploys uninstall the old discovery server |
| `ros2-master` | ROS2 DDS master |
| `master2master` | Cross-host topic bridge |
| `filter_node` | Kalman filter for arm joints |
| `test_joint_api` | REST API for joint testing |
| `topic_scraper_api` | Telemetry / observation API |
| `bno055_imu` | IMU bridge (I2C) |
| `gps_rtk_rover` | GPS RTK rover (UART, reboots Pi if UART overlay changes) |
| `haptic_controller` | Haptic feedback (disabled by default) |
| `gripper_uvc_camera` | USB camera bridge |
| `rplidar_a1` | LiDAR bridge (USB) |
| `realsense_d435i` | Removed (present: false): replaced by `overview_camera` |
| `overview_camera` | Retired (D7): `present: false`, deploying it uninstalls the unit; `pi_camera_boot_config.yml` holds the CSI overlays. no node publishes `/overview_camera/*` any more |
| `head_camera_cpp` | Head Camera Module 3 (IMX708, CSI cam0) on the pan/tilt head; C++ node (`ros2 run head_camera_cpp head_camera_node`) built against the pinned libcamera fork in `/opt/ros2-ws`; publishes `/head_camera/image_raw[/compressed]` (1152x648, 30 fps) and serves zoom crops on `/head_camera/zoom/{command,image,result}`; background cgroup tier, metrics on 19116 |
| `head_rgbd_cpp` | Head RGB-D fusion: registers the ToF depth into the Camera Module 3 mapping frame, publishes `/head_rgbd/*` (aligned depth + RGB, coloured cloud); C++ node (`ros2 run head_rgbd_cpp head_rgbd_node`) built in `/opt/ros2-ws`; needs `head_camera_cpp` mapping and `/etc/ros2/head_rgbd_cpp/tof_calibration.yaml`; background cgroup tier, metrics on 19117. Deploy: `./scripts/deploy-nodes.sh client head_rgbd_cpp` |
| `ground_estimator_cpp` | Ground plane, step and drop-off estimate from the filtered head ToF cloud and the IMU; publishes `/ground/*` for the Nav2 local costmap; C++ node (`ros2 run ground_estimator_cpp ground_estimator_node`) built in `/opt/ros2-ws`; normal cgroup tier, metrics on 19119. Deploy: `./scripts/deploy-nodes.sh client ground_estimator_cpp` |
| `map3d_uplink` | The one client static-peer process carrying the map3d streams to the server and the 3D map back (lazy topic_tools relays + compressedDepth republisher, contract `nodes/rtabmap_mapping/config/map3d_topics.yaml`); background tier; deploy after `head_camera_cpp` and `head_rgbd_cpp` |
| `pi_cameras` | Retired: `present: false`, deploying it uninstalls the unit and purges `camera_ros`; replaced by `head_camera_cpp` (the `Deploy pi_cameras` step stays so the uninstall runs) |
| `arducam_tof` | Arducam ToF on CSI cam1 (`csi_device: 1f00128000.csi`, `dtoverlay=arducam-pivariety,cam1`; needs the camera firmware reflashed for the Pi 5); publishes `/tof/depth/image_raw`, `/tof/amplitude/image_raw`, `/tof/camera_info`; metrics 19112 |
| `stereo_depth` | Retired (D7, sources deleted): `present: false`; deploying it stops and removes the unit. No node publishes `/stereo/*` any more |
| `stereo_depth_cpp` | Retired (D7, sources deleted): `present: false`; deploying it stops and removes the unit. No node publishes `/stereo/*` any more |
| `jetson_bridge_cpp` | Retired: `present: false` with colcon purge, so deploying it removes the unit and the built workspace package. `./scripts/deploy-nodes.sh client jetson_link` removes the client's link state (hosts line, sysctl, nftables) |
| `lerobot_follower` | SO-101 follower arm (feetech, its own USB bus) |
| `base_servos` | Swerve servos 32-39 (feetech, base bus on the second adapter); deploy together with `lerobot_follower` and `swerve_controller` when bus config changes |
| `head_state_publisher` | robot_state_publisher on the pan/tilt head URDF (head TF from `/head/joint_states`) |
| `swerve_controller` | Swerve drive kinematics |
| `static_tf_publisher` | TF frame publisher |
| `rf2o_laser_odometry` | rf2o lidar odometry (source-built colcon workspace `/opt/ros2-ws`, pinned commit; first deploy compiles at lowest priority) |
| `rf2o_odom_relay` | rf2o pose -> body twist with covariance for the EKF |
| `map_guard_cpp` | C++ port of `map_guard`, side by side (`present: false`; never enabled together with `map_guard`, same metrics port 19110); built as a colcon local source from `nodes/map_guard_cpp`; rollback: re-enable `map_guard` |
| `robot_localization_ekf` | EKF odometry fusion |
| `nav2_bringup` | Nav2 navigation stack |
| `web_ui` | Browser dashboard (FastAPI + React) |
| `poi_store` | Points/areas of interest store on the map (`/var/lib/ros2/poi/poi.json`, `/poi/*` topics) |
| `map_manager` | Named maps: save/list/load/delete bundles of the slam_toolbox graph, POIs and last pose (`/var/lib/ros2/maps/named`, `/map_manager/*` topics) |
| `costmap_zones` | Keepout/speed zone masks from area POIs for the Nav2 costmap filters (`/costmap_zones/*` masks, `/costmap_filter_info/*`) |
| `marker_detector` | Decodes printed QR metadata markers in the head camera stream and zoom stills, estimates pose (ToF plane disambiguation), upserts marker POIs in poi_store (`/markers/detections`); metrics port 19118 |
| `claude_agent` | Claude (Agent SDK, Opus) chat agent driving the robot via mcp_server; needs `export CLAUDE_CODE_OAUTH_TOKEN=...` on first deploy (the deploy fails without it unless the robot already has the token) |

### Server nodes (RPi4b)
| Node | Notes |
|------|-------|
| `fastdds_discovery_server` | Removed (present: false); deploys uninstall the old discovery server |
| `ros2-master` | ROS2 DDS master |
| `lerobot_leader` | SO-101 leader arm (feetech servos, USB) |
| `topic_scraper_api` | Telemetry / observation API |
| `gps_rtk_base` | GPS RTK base station (UART) |
| `rtabmap_mapping` | RTAB-Map RGB-D 3D mapping (server): one component_container, split `ros-jazzy-rtabmap-*` apt packages, database `/var/lib/ros2/rtabmap`; `cpu_quota` 200%, `memory_max` 900M; fed by the client `map3d_uplink` |
| `map3d_manager` | 3D map save/load/reset commands and status (server, `map3d_manager_cpp` colcon local sources); `cpu_quota` 20%, `memory_max` 128M; deploy after `rtabmap_mapping` |

---

## Node Config Schema

All node configs live in `ansible/group_vars/client.yml` and `server.yml` under `ros2_nodes`:

```yaml
ros2_nodes:
  - name: filter_node          # service name: ros2-filter_node
    node_type: filter_node     # maps to ros2_node_type_defaults entry
    present: true              # false = remove systemd unit and config dir
    enabled: true              # false = installed but stopped/disabled
    config: |                  # written to /etc/ros2-nodes/<name>/config.yaml
      some_param: value
```

To add/remove/disable a node: edit `group_vars/client.yml` or `server.yml`, then run `--all`.

---

## After Deploy: Verification

```bash
# Check systemd service
ssh client.ros2.lan "systemctl status ros2-<node>"
ssh server.ros2.lan "systemctl status ros2-<node>"

# Check service logs
ssh client.ros2.lan "journalctl -u ros2-<node> --no-pager -n 50"

# Check ROS2 topic flow
ros2 topic hz /controller/follower/joint_states
ros2 topic echo /controller/imu/data --once
```

---

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `is not a ros2_nodes entry` | Node name typo | The script lists the valid names from `group_vars/<target>.yml` |
| uv sync timeout | Transient network on RPi | Re-run the same deploy command |
| Service restart loop | Bad config or missing device | `journalctl -u ros2-<node>` for traceback |
| `systemctl status` failed | Node crash on start | Check `journalctl -u ros2-<node>` |
| Stale SSH control socket | Network dropped mid-run | `rm ~/.ansible/cp/*` and re-run |
| `ansible-lint` failures | New task missing `name:` | Fix and run `uv run poe lint-ansible` |

---

## Lint and Test

```bash
# From repo root
uv run poe lint-ansible      # ansible-lint
uv run poe test-ansible      # lint + syntax-check all playbooks
```
