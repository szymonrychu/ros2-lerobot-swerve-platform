# Repo map: stereo depth / cameras / Ansible (for the Jetson Nano offload)

Repo: /Users/szymonri/Documents/ros2-lerobot-sverve-platform (HEAD 31de0d5 on main, 2026-10-13).
Sources used: CLAUDE.md, MEMORY.md, ROADMAP.md, git log, manifests, grep/read. graphify-out/graph.json is from
2026-04-09 (6 months stale, predates all stereo work) so it was not used. LSP tool was available but not used: the
questions were about topic strings and YAML contracts, which grep answers exactly.

## 0. Headline facts

- The stereo design was FULLY implemented (T1-T6 all on main), then ported to C++ and cut over. No -record.md exists
  for stereo-depth, but each task maps to a commit (section 1).
- Stereo depth has NEVER produced data on the robot: docs/plans/2026-10-13-cpp-nodes-record.md:73 "stereo_depth_cpp is
  idle until the right camera (cam1) probes (hardware) and calibration runs". No stereo calibration exists in the repo
  (docs/calibration/2026-10-10 is the ARM/gripper-camera calibration only, unrelated to stereo).
- Only ONE consumer of depth exists: the web_ui Head tab. Nav2 costmaps use /scan only. mcp_server uses the LEFT camera
  RGB (/overview_camera/image_raw/compressed) as its "front" camera; it does not consume depth.
- Moving both IMX219 to the Jetson removes the source of /overview_camera/* too, which has more consumers than depth
  (web_ui Arm/Head/Overview tabs, mcp_server front camera, photos/POIs, agent). That is the larger blast radius.
- No mention of "Jetson" anywhere in tracked files. ROADMAP.md (46 lines) has nothing on stereo, depth, offload or GPU.
- No chrony anywhere. Only time-sync mention: nodes/leader_bridge/README.md:45 (systemd-timesyncd).

## 1. The stereo-depth plan and what landed

Files: docs/plans/2026-10-13-stereo-depth-{design.md (114 l), plan.md (418 l), tasks.json (6 tasks),
approvals.json (S1, S2 both "approved")}, briefs/stereo-brief.md (CPU/sensor research).

Design (design.md):
- Hardware: two IMX219 on Pi 5 CSI cam0 (left) / cam1 (right) (:5, :30-31); replaced IMX708 overview cam + Arducam ToF.
- Capture: 1640x1232 2x2-binned sensor mode, ISP downscale to 480x360 (:11, :34), fixed 200 ms frame duration = 5 fps
  (:35), libcamera SyncMode server(left)/client(right) with ApproximateTime fallback (:36).
- Algorithm: rectify from stored calibration, OpenCV StereoSGBM MODE_SGBM_3WAY, 48 disparities, block 5, speckle filter,
  uniqueness (:39-40). WLS optional "if fps stays >= 5" (:103-104) - not implemented.
- Outputs (:41-42): /stereo/depth/image_raw sensor_msgs/Image 16UC1 mm 0=invalid ("ToF contract"), /stereo/camera_info,
  optional /stereo/disparity (stereo_msgs/DisparityImage).
- Never publish without fresh synced pair + valid calibration (:43); metrics fps/matcher ms/skew/drops (:44).
- Calibration tool (:46-48): chessboard/ChArUco, stereoCalibrate, writes /var/lib/ros2/camera_calibration/stereo.yaml
  + per-camera intrinsics.
- D2 (:85-90): left camera keeps /overview_camera/* topic names; Head tab tof_depth_topic -> /stereo/depth/image_raw.
- Non-goals (:59-61): "No offload to the server", no point cloud / Nav2 costmap integration, no pan/tilt.
- Anti-goal: background cgroup tier, rate degrades rather than starving robot (:66).
- Brief: client Pi 5 has ~0.3-0.45 of 4 cores idle (brief :121-129); MEMORY.md:157 "about 0.4 core at 5 fps".

Task -> commit (git log):
| Task | What | Commit |
|---|---|---|
| T1 boot overlays + camera_ros SyncMode patch | overview_camera_boot_config.yml, patch | 3687594 |
| T2 two synced CameraNodes + calibration gate | overview_camera launch | ef96e32 |
| T3 stereo_depth node (Python SGBM) | nodes/stereo_depth | 75a834e |
| T4 calibrate + bench tools | stereo_depth.calibrate / .bench | f718f5d |
| T5 Head tab -> /stereo/depth/image_raw | web_ui | 4b08929 (+651450d docs) |
| T6 Ansible node, background cgroup tier, metrics | group_vars/client.yml | 515a353 |
| (cpp-nodes plan T3/T8) rclcpp port + cut-over | nodes/stereo_depth_cpp | 79e1903, d4bb78f, b48d09f, f12cb74 |
Plan docs commit 4fe2d15 (dated 2026-10-12 despite the 10-13 filename). Leftover worktrees st-*/st2-* and cpp-t3 exist
under .claude/worktrees (stale agent branches, all merged-or-superseded; not inspected further).

Earlier history: 4c6bf6c/1d0687e (2026-10-05) first IMX219 stereo via stereo_image_proc; 08bc9b8 (2026-10-07) removed it
for a single IMX708; restored 2026-10-12.

## 2. Current stereo / depth / camera code

### nodes/overview_camera (launch-only, no Python package; camera_ros from source)
- launch/overview_camera.launch.py: one component_container_mt "overview_container" (:75-79) with two
  camera::CameraNode composables (:65-71); logs warnings for uncalibrated sides (:61-63).
- launch/overview_camera_config.py: settings merge (:24, :43), camera selection by libcamera id/index (:62),
  calibration gate projection_is_valid / calibration_problem / camera_info_url (:82, :100, :134), SyncMode param (:202).
- config/params.yaml: calibration_dir /var/lib/ros2/camera_calibration (:12); sensor_mode 1640:1232, 480x360, RGB888
  (=bgr8), FrameDurationLimits [200000,200000], AeExposureMode short (:16-22); left ns /overview_camera, sync server,
  frame overview_camera_optical_frame, jpeg_quality 80 (:25-29); right ns /stereo/right, sync client, frame
  stereo_right_optical_frame (:32-35).
- patches/camera_ros-0001-expose-sync-controls.patch, patches/libcamera-0001-add-package-xml.patch.
- Topics produced: /overview_camera/image_raw (+ /compressed via image_transport), /overview_camera/camera_info,
  /stereo/right/image_raw, /stereo/right/camera_info (README.md:19-22).
- README.md:109-114: no TF to overview_camera_optical_frame published (mount not measured).

### nodes/stereo_depth (Python, kept present+disabled as rollback)
- stereo_depth/config.py: DepthConfig pydantic (extra=forbid) :31-86; defaults :63-86 (topics as above,
  calibration_dir, num_disparities 48, block 5, p1 200, p2 800, uniqueness 10, speckle 100/2, disp12 1, prefilter 31,
  min/max depth 0.3/4.0 m, sync_slop_s 0.002, max_rate_hz 5, matcher_threads 2, frame_id stereo_left_optical_frame,
  metrics_port 19113). Env STEREO_DEPTH_CONFIG, default /etc/ros2/stereo_depth/config.yaml (:13-14).
- stereo_depth/node.py: ApproximateTimeSynchronizer on sensor-data QoS (:98-101), publishers reliable depth 5 (:90-93),
  on_pair (:134), publish (:201-216), Prometheus metrics (:54-64).
- stereo_depth/depth.py: disparity_to_depth_mm Z=f*T/d, uint16 mm, 0 invalid (:15-45); create_matcher SGBM 3WAY (:48-69).
- stereo_depth/calibration.py: files stereo_left.yaml / stereo_right.yaml (ROS camera_info YAML) + stereo.yaml (R, T, Q,
  rms) (:6-10, :27-29); validation incl. baseline from right P[3] == |T| (:168-190); load_stereo_calibration (:395).
- stereo_depth/messages.py: depth_image_msg, camera_info_msg (rectified left P), disparity_msg.
- stereo_depth/calibrate.py (478 l): capture mode = a ROS2 node subscribing both image topics, saves pair PNGs;
  solve mode = calibrateCamera per side + stereoCalibrate(CALIB_FIX_INTRINSIC) + stereoRectify(alpha 0) (:1-12).
  config/calibrate.yaml: chessboard 9x6 25 mm, 30 pairs, rms_limit 0.5 px, output /var/lib/ros2/camera_calibration.
- stereo_depth/bench.py + config/bench.yaml: CPU benchmark.
- tests/: test_config, test_calibration, test_depth, test_messages, test_node, test_calibrate, test_bench (146 pass).

### nodes/stereo_depth_cpp (rclcpp, LIVE entry since cut-over f12cb74)
- include/stereo_depth_cpp/config.hpp:19-58: same keys/defaults, same env var and default path, metrics 19113.
- src/stereo_depth_node.cpp:96-105: same pubs/subs; message_filters ApproximateTime with max interval sync_slop_s.
- src/{pipeline,depth,calibration,config}.cpp, src/bench_main.cpp; scripts/bench_compare.py.
- Deps (package.xml:12-21): ros2_common_cpp, yaml-cpp, prometheus-cpp, libopencv-dev, rclcpp, message_filters,
  sensor_msgs, stereo_msgs, std_msgs. Parity fixtures generated from Python (test/gen_fixtures.py).
- shared/ros2_common_cpp: lax YAML reader, Metrics + ProcessCollector.

### Calibration files
- On-robot path only: /var/lib/ros2/camera_calibration/{stereo_left,stereo_right,stereo}.yaml (+ captures/). Not in
  the repo. Whether any exist on the client is UNKNOWN (record says calibration never ran).
- docs/calibration/2026-10-10/: arm joint offsets, gripper camera mount/intrinsics, sag fit. Not stereo.

### Ansible wiring (all in ansible/group_vars/client.yml unless noted)
- Cgroup tier table :27-32 (critical 1000 / normal 500 / agent 300 / background 100; stereo_depth = background).
- node types: overview_camera :285-353 (native ros2 launch, colcon_source libcamera v0.7.2+rpt20260817 and camera_ros
  8f792e2 + patches into /opt/ros2-ws, normal tier); stereo_depth :357-376 (uv node, background tier);
  stereo_depth_cpp :382-427 (colcon local source shared/ros2_common_cpp + nodes/stereo_depth_cpp, background tier).
- ros2_nodes entries: realsense_d435i present:false :1136; overview_camera enabled :1142-1150 (left/right_camera_id
  empty); stereo_depth present, enabled:false :1154-1186; stereo_depth_cpp enabled :1190-1222 (identical config).
- static_tf_publisher: deliberately no overview_camera frame :1404-1405.
- web_ui tabs: Arm overview_camera_topic :1732; Head rgb_camera_topic + tof_depth_topic /stereo/depth/image_raw +
  tof_range_m 4.0 :1744-1746; Overview camera tab :1750.
- mcp_server topics.front_camera /overview_camera/image_raw/compressed :1787; cameras.front parent base_link,
  intrinsics null, mount null (~:1940-1943).
- monitoring_agent_node_targets :2511; stereo_depth_cpp 127.0.0.1:19113 :2524.
- ansible/playbooks/tasks/overview_camera_boot_config.yml:8-64: config.txt edits (remove imx708/arducam overlays,
  camera_auto_detect=0, dtoverlay=imx219,cam0/cam1, reboot on change). Included from deploy_nodes_client.yml:82-86
  (tags boot, overview_camera).
- ansible/playbooks/deploy_nodes_client.yml: Deploy overview_camera :279-286, stereo_depth :288-295,
  stereo_depth_cpp :299-306; arducam_tof_sdk :89.
- arducam_tof (nodes/bridges/arducam_tof) present, enabled:false (:1110); metrics 19112 not scraped.

### Topic contracts and consumers
| Topic | Type | Producer | Consumers |
|---|---|---|---|
| /overview_camera/image_raw | Image bgr8 480x360 5 fps | overview_camera left | stereo_depth(_cpp) left input, calibrate |
| /overview_camera/image_raw/compressed | CompressedImage | image_transport | web_ui Arm/Head/Overview tabs (client.yml:1732,1744,1750; web_ui/config/default.yaml:66,75,82; web_ui/web_ui/config.py:90; bridge.py:120); mcp_server front camera (mcp_server/config.py:136, ros_iface.py:684) -> get_camera_image, lookups, photos/POIs, look_toward_frontier, approach_tools |
| /overview_camera/camera_info | CameraInfo | left | none found in code (mcp uses its own cameras.front intrinsics, null) |
| /stereo/right/image_raw | Image | overview_camera right | stereo_depth(_cpp), calibrate |
| /stereo/depth/image_raw | Image 16UC1 mm, 0 invalid | stereo_depth_cpp | web_ui Head tab only (HEAD_DEPTH_ROLE bridge.py:213; config.py:270,497,590; msg_serializer.py:202-203,257) |
| /stereo/camera_info | CameraInfo (rectified left P) | stereo_depth_cpp | none found |
| /stereo/disparity | DisparityImage (off) | stereo_depth_cpp | none |
Stale entries: web_ui/web_ui/bridge.py:106-108 TOPIC_TYPE_HINTS "/stereo/left/image_rect", "/stereo/depth/image_rect",
"/stereo/depth/camera_info" and msg_serializer.py:15 RGBD_COLOR_TOPICS - leftovers of the 2026-10-05 pipeline, no
producer today. claude_agent test_prompt.py:128 asserts the prompt does not mention "stereo".
No consumer in nav2 (nav2_params.yaml:218,292,569 observation_sources scan), map_guard, costmap_zones, collision.
Frames: stereo_left_optical_frame (depth header), overview_camera_optical_frame, stereo_right_optical_frame - none
in TF.

## 3. Ansible layout and adding a Jetson

- ansible/ansible.cfg: inventory=inventory, roles_path=roles, fact cache, profile_tasks.
- ansible/inventory is GITIGNORED (.gitignore:38) and contains Wi-Fi credentials; tracked template is
  ansible/inventory.example. Groups: [server] server-rpi4 (server.ros2.lan), [client] client-rpi5 (client.ros2.lan),
  [controller] controller-steamdeck (only in the local inventory, not in .example), [all:vars] ansible_user=szymonri,
  ansible_python_interpreter=/usr/bin/python3, network_wifi_ssid/password.
- group_vars: all.yml (ros2_hosts_entries :14-17 with .33/.34/.35, DDS env :29-31, repo url/dest :37-39, build cgroup,
  deploy state dir, agent idle guard), client.yml (network .34 :3-13), server.yml (.33), controller.yml (.35,
  ros2_extend_etc_hosts true). No host_vars dir.
- Provision playbooks: site.yml (hosts: all -> common, network, hostname, system_optimize; then imports
  deploy_nodes_server/client) - NOTE hosts: all would hit a new jetson group too. playbooks/client.yml, server.yml
  (common, network, hostname, ros2_base, system_optimize); controller.yml (hostname, steamdeck_ui) - the precedent for
  a non-Pi, non-ROS-node host with its own group + playbook; optimize.yml (hosts: all, system_optimize);
  docker_cleanup.yml.
- Roles:
  - common: python3/git, pinctrl and raspi-gpio wrappers, GPIO/I2C udev rule (Pi-specific).
  - network: netplan, primary iface static (eth or wlan), every OTHER ethernet gets dhcp4: true, optional: true
    (templates/netplan.yaml.j2:25-31). The client's primary is Wi-Fi (system_optimize disables Wi-Fi power mgmt;
    no network_interface set), so the Pi's eth0 for a direct Jetson link would today be DHCP with no DHCP server ->
    no address. A static point-to-point address needs a template change or new vars.
  - hostname: hostnamectl, /etc/hosts incl. ros2_hosts_entries when ros2_extend_etc_hosts.
  - system_optimize (OS hardening/cleanup): debloat purge list (snapd, cloud-init, modemmanager, unattended-upgrades...
    defaults/main.yml), masked services, cpu governor performance, sysctl, swap off, noatime, apt timers off, core
    dumps off, tmpfs /tmp, /var/tmp, then Pi-only edits of /boot/firmware/config.txt (gpu_mem, bluetooth, HDMI,
    I2C baud; failed_when: false) and watchdog, volatile journald. Not Jetson-aware (L4T has no /boot/firmware;
    swap-off and governor tasks may conflict with nvpmodel/zram on Jetson).
  - ros2_base: ROS Jazzy apt, uv, client-only RealSense/NodeSource tasks gated on 'client' in group_names.
  - ros2_node_deploy: systemd units (templates ros2-node-native.service.j2, ros2-node-launcher.j2, config.yaml.j2),
    colcon_source_build/package.yml, stop_for_build.yml. ROS-specific (sources /opt/ros/jazzy).
  - ros2_node_verify, monitoring (server), monitoring_agent (Alloy -> server Mimir/Loki, node targets list,
    monitoring.slice cpu weight 20), web_proxy (nginx), steamdeck_ui, docker_cleanup.
  - Cgroups: playbooks/tasks/cgroup_headroom.yml + templates/{system,user}-slice-headroom.conf.j2; per-unit
    cpu_weight/io_weight/memory_low/memory_max/oom_score_adjust/nice in node types.
  - Chrony: none. Time sync is whatever the OS default is (systemd-timesyncd on Ubuntu).
- scripts/deploy-nodes.sh: TARGET must be client|server (:30-40), validates names against
  `^  - name:` lines of group_vars/<target>.yml (:83), non-node targets "monitoring" (:47-50), runs
  playbooks/deploy_nodes_<target>.yml -l <target> (:105). A jetson target needs edits here (bash 3.2 compatible).
- Tests that pin Ansible structure: tests/test_ansible_deploy_speed.py, test_unit_cgroups.py,
  test_node_metrics_wiring.py, test_stereo_depth_config.py, test_overview_camera_config.py,
  test_overview_camera_launch_config.py, test_ci_workflow.py (node dir must be in CI matrix), test_mcp_server_config.py,
  test_mcp_camera_config.py, test_arducam_tof_config.py.

## 4. Verification commands (verbatim) and results 2026-10-14

From CLAUDE.md, pyproject.toml [tool.poe.tasks], .mise.toml, .github/workflows/ci.yml:
- `uv run poe lint` (ruff check/format --check + vulture on tests shared) -> "All checks passed!" / "66 files already
  formatted", exit 0.
- `uv run pytest tests -q` (mise task test; CI) -> "1116 passed in 115.87s (0:01:55)".
- `uv run poe test-ansible` (ansible-lint + --syntax-check of site.yml and 9 playbooks; needs local ansible/inventory)
  -> "Passed: 0 failure(s), 0 warning(s) in 134 files processed of 149 encountered. Profile 'basic' was required, but
  'production' profile passed." exit 0.
- `uv run poe lint-ansible` (= cd ansible && ansible-lint .) - covered by the above.
- `uv run poe lint-nodes` (per-node `uv run --frozen poe lint`, explicit list incl. nodes/stereo_depth) - not run in
  full; ran for stereo_depth only.
- Node: `cd nodes/stereo_depth && uv run --frozen pytest -q` -> "146 passed in 1.38s"; `uv run --frozen poe lint` ->
  "All checks passed!", exit 0. CI per node also runs `uv lock --check`.
- C++: `uv run poe test-cpp` / `uv run poe lint-cpp` loop `scripts/cpp-node-test.sh <pkg>` (docker ros:jazzy-ros-base).
  Ran `scripts/cpp-node-test.sh stereo_depth_cpp` -> "Summary: 84 tests, 0 errors, 0 failures, 0 skipped",
  "cpp-node-test: stereo_depth_cpp PASS".
- mise: `mise run install|lint|fmt|test`. pre-commit: conventional-pre-commit (commit-msg), trailing-whitespace,
  end-of-file-fixer, check-yaml, check-added-large-files, ruff-check --fix, ruff-format, vulture, local ansible-lint
  (scripts/lint-ansible.sh) on ansible/*.yml.
- CI jobs: lint, root tests, lint-ansible (installs ansible/requirements.yml collections first), nodes matrix
  (stereo_depth at ci.yml:71), cpp matrix (stereo_depth_cpp at ci.yml:103).
- Deploy: only `scripts/deploy-nodes.sh <client|server> <node...>` logged to .logs/ (CLAUDE.md).
- Tree state: `git status --short` -> only `?? scratch_AH.bak` (untracked, root). Many stale worktrees in
  .claude/worktrees and .worktrees (rgbd-backend, rgbd-frontend).

## 5. Conventions a new piece must follow
- Python 3 type hints, pydantic config from YAML file (env var points at /etc/ros2/<node>/config.yaml), constants at
  top, no try/except imports, docstrings with types; per-node pyproject + uv.lock, ruff+vulture via poe lint; numpy
  guard; tests first; node dir must be added to CI matrix and lint-nodes list (test_ci_workflow enforces).
- No placeholder data on topics: skip + log + count when no valid data.
- Every node: ros2_nodes entry (name, node_type, present, enabled, config) + node type with cgroup tier, memory
  low/max, metrics target in monitoring_agent_node_targets, Deploy block in deploy_nodes_<host>.yml tagged with name.
- READMEs per node, ansible/README.md, tests/README.md, MEMORY.md for decisions.
- Commit semantic, push to main.

## 6. Blast radius for "IMX219 pair now on the Jetson"
Must change or retire:
- overview_camera node (whole thing: libcamera fork + camera_ros colcon builds, launch, params) - no cameras on Pi now.
- overview_camera_boot_config.yml (imx219 overlays on the Pi; reboot logic) + its include at deploy_nodes_client.yml:82-86
  + tests/test_overview_camera_config.py / test_overview_camera_launch_config.py / test_ansible_deploy_speed.py mapping.
- stereo_depth / stereo_depth_cpp node types + entries + metrics target (client.yml:357-427, 1154-1222, 2524),
  tests/test_stereo_depth_config.py, test_unit_cgroups.py, test_node_metrics_wiring.py.
- calibrate tool (captures via ROS topics on the Pi) and calibration path /var/lib/ros2/camera_calibration (Pi).
Must keep working (contracts the Pi bridge must republish):
- /stereo/depth/image_raw 16UC1 mm 0=invalid + /stereo/camera_info (Head tab).
- /overview_camera/image_raw/compressed (web_ui x3 tabs, mcp_server front camera and everything downstream); possibly
  /overview_camera/image_raw raw if anything else needs it (nothing found besides stereo inputs).
- frame_ids stereo_left_optical_frame / overview_camera_optical_frame (no TF either way today).
Cross-host: client.yml DDS is localhost-only (all.yml:29-31), so a Pi bridge node publishing locally fits; the Jetson
must not join DDS. Monitoring: Alloy pushes to server.ros2.lan (monitoring_agent defaults :22-23) - a Jetson without
internet/LAN route to the server would need the Pi to scrape/relay its metrics.

## 7. Unknowns
- Whether cam1 hardware probe failure was ever resolved, and whether any stereo calibration files exist on the client
  (/var/lib/ros2/camera_calibration). Settles: inspect the client (via repo scripts, not ad-hoc ssh per CLAUDE.md).
- Client's physical eth0 usage today (wired vs Wi-Fi primary): group_vars sets no network_interface; inferred Wi-Fi
  from system_optimize/inventory. Settles: `ip -br addr` on client or the fact cache ansible/.ansible_facts_cache.
- Jetson OS image (JetPack 4.x = Ubuntu 18.04 for Nano B01; ansible_python_interpreter /usr/bin/python3 is 3.6 there,
  which modern ansible-core may not support as a target). Not in repo.
- ansible/README.md and nodes/stereo_depth/README.md not read in full (calibration procedure, cut-over/rollback section).
- graphify graph is stale; not rebuilt.
