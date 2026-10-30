# D435 head RGB-D epic: discovery map, perception / TF / Ansible / calibration slice

Date: 2026-10-27. Read-only discovery on `main` at d0bab034. Companion slice (web_ui, mcp_server, gripper position
estimation) is mapped separately. Tools: Python LSP was available and used (documentSymbol / findReferences on
`shared/ros2_common/head_rgbd_calibration/urdf_text.py`); it does not resolve the launch-file `importlib` loads, so
cross-package call sites were found with `rg`. No C++ language server: C++ was read and grepped.

## 1. RealSense bridge (`nodes/bridges/realsense_d435i`)

- Directory holds only `README.md` (stale: describes D435i IMU, `/camera/imu`, `enable_accel:=true`). No launch file,
  no params file, no code. The node is apt-only: `ros-jazzy-realsense2-camera`, `rs_launch.py`.
- Node type: `ansible/group_vars/client.yml:316-331`
  - `node_launch_command`: `ros2 launch realsense2_camera rs_launch.py enable_accel:=false enable_gyro:=false
    depth_module.depth_profile:=848x480x90 rgb_camera.color_profile:=1280x720x30 align_depth.enable:=true`
    (folded `>-`; byte-identity pinned by `tests/test_ansible_deploy_speed.py:1119-1124`, `FOLDED_LAUNCH_COMMANDS`).
  - `src_paths: [nodes/bridges/realsense_d435i]`, `memory_low: 128M` (estimate, "not running on 2026-10-11"),
    `memory_max: 512M`, normal tier `cpu_weight/io_weight 500`, `oom_score_adjust -500`, `apt_packages:
    [ros-jazzy-realsense2-camera]`. No `config_path`, no `env`, no serial number, no camera_name/namespace, no
    `pointcloud.enable`, no `initial_reset`.
- Instance: `client.yml:1405-1409` `name: realsense_d435i, node_type: realsense_d435i, present: false, enabled: false`
  ("Retired earlier").
- Playbook step exists: `ansible/playbooks/deploy_nodes_client.yml:294-301` (`Deploy realsense_d435i`, tags
  `[realsense_d435i, apt, python, build, config]`), so `scripts/deploy-nodes.sh client realsense_d435i` works once
  present.
- Host prep still deployed by `ansible/roles/ros2_base/tasks/main.yml:122-150`: udev rules
  `/etc/udev/rules.d/99-realsense-d435i.rules` and `/etc/modprobe.d/99-realsense-uvcvideo.conf` (`options uvcvideo
  quirks=640`). **The udev rules match `idProduct 0b3a` (D435i) only. A plain D435 is a different product id (0b07 per
  Intel's librealsense udev rules - external knowledge, verify with `lsusb` on the Pi).** The IIO/HID rules are
  D435i-IMU only.
- History: added 6e5bdee6; aligned depth 0fcdc9b9; namespace fix 4cb499b9 (topics are `/camera/camera/...`, the
  realsense-ros 4.x `camera_namespace/camera_name` default); IIO udev 03f8ace9. **Retired by 4c6bf6cf (2026-10-05)
  "IMX219 stereo pair via source-built libcamera fork and camera_ros, retire RealSense"**; the static TF
  `base_link -> camera_link` (x 0.10, z 0.08) was removed in that commit. The stereo pair was itself retired
  2026-10-18 (MEMORY.md:158, with the Jetson). No commit states a technical fault with the RealSense; it was
  replaced by a different sensor choice.
- Frames the node would publish (realsense-ros 4.x defaults, external knowledge, not in repo): `camera_link`,
  `camera_depth_frame`, `camera_depth_optical_frame`, `camera_color_frame`, `camera_color_optical_frame` (plus
  infra1/2), as static TF rooted at `camera_link` (`publish_tf` default true). The aligned depth carries
  `camera_color_optical_frame`. Nothing in the repo connects `camera_link` to the robot tree today. Topics with the
  launch above: `/camera/camera/color/image_raw` (+ `/compressed` if image_transport plugins installed),
  `/camera/camera/color/camera_info`, `/camera/camera/depth/image_rect_raw`, `/camera/camera/aligned_depth_to_color/
  image_raw` (16UC1 mm) and `.../camera_info`.
- Stale references to update: `README.md:56,103,139,201,232,304`, `MEMORY.md:90-93`, `nodes/README.md:23`,
  `ansible/README.md:468,504`, the bridge README itself. Retirement pins that may collide: `tests/test_jetson_retired.py`
  (allowlist line 672, `.claude/skills/ansible-deploy/SKILL.md` row), `tests/test_mcp_server_config.py:828`
  (`"realsense_camera" not in topics`), `nodes/mcp_server/tests/test_tools.py:448-460` (camera schema has no
  "realsense"), `nodes/claude_agent/tests/test_prompt.py:132`.

## 2. Head sensors today

| Node (instance) | Sensor | Publishes | Frame | Enabled |
|---|---|---|---|---|
| `arducam_tof` (`client.yml:1354-1390`, Python `nodes/bridges/arducam_tof`) | Arducam ToF, CSI cam1, 240x180 | `/tof/depth/image_raw` 16UC1 mm z-depth, `/tof/amplitude/image_raw` mono16, `/tof/camera_info` (topics fixed in `bridge.py:49-51`, reliable depth 5) | `tof_optical_frame` | true |
| `tof_obstacle_cloud` (`client.yml:1391-1395`, type `155-175`; config-only `nodes/tof_obstacle_cloud/config/point_cloud_xyz.yaml`) | rtabmap_util `point_cloud_xyz` | `/tof/points` (decimation 4 -> 60x45, min 0.2 / max 3.0 m, voxel 0.05, radius noise filter, `approx_sync: false`) | `tof_optical_frame` | true |
| `laser_filter` (Python, `nodes/laser_filter/config/footprint_filter.yaml:17-24`) | self filter | `/tof/points` -> `/tof/points_filtered` (footprint polygon, z <= 1.0 m, TF at stamp, sensor_msgs_py) | input frame | true |
| `head_camera_cpp` (`client.yml:1420-1446`) | Camera Module 3 (IMX708), CSI cam0 | `/head_camera/image_raw/compressed` 1152x648 15 fps, `/head_camera/camera_info`, zoom topics, mapping stream `/head_camera/mapping/image_raw/compressed` + `/camera_info` 576x324 2 Hz (focus-locked) | `head_camera_optical_frame`, no TF published | true |
| `head_rgbd_cpp` | ToF + CM3 fusion | `/head_rgbd/{depth/image_raw, depth/camera_info, rgb/image_raw/compressed, rgb/camera_info, points, depth_upsampled/image_raw, depth_upsampled/camera_info}` | RGB optical frame (stamp and frame of RGB) | **false** (`client.yml:1451`) |
| `ground_estimator_cpp` | plane fit | `/ground/{obstacle_points, drop_points, labelled_points, state, warning_areas}` | base_link / map | **false** (`client.yml:2149`) |

Consumers (from `rg` over nodes/, ansible/group_vars, shared, scripts; tests excluded):

- `/tof/points_filtered`: nav2 `tof_voxel_layer` (tof_mark, tof_cliff_mark, tof_clear) and `ground_voxel_layer.ground_clear`
  (`nav2_params.yaml:263-345, 430-435`), collision_monitor `tof_obstacles`, `tof_cliff` (`nav2_params.yaml` ~858-872),
  `ground_estimator_cpp` cloud_topic, web_ui `obstacle_cloud_topic` (`client.yml:2510`), mcp_server `tof_points`
  (`client.yml:2701`, `ros_iface.py:703`, lazy).
- `/tof/points`: laser_filter input, head_rgbd_calibration floor sweep `floor.cloud_topic` (`client.yml:1588`).
- `/tof/depth/image_raw`, `/tof/amplitude/image_raw`, `/tof/camera_info`: head_rgbd_cpp input, head_rgbd_calibration
  capture topics (`client.yml:1519-1523`), scripts/head_rgbd_calibrate.yaml capture topics, web_ui head tab
  (`client.yml:2618-2619`), ground_estimator `guidance.tof_info_topic` (`client.yml:2254`).
- `/head_rgbd/*`: ground_estimator (`points` colour, `depth_upsampled/*`, `rgb/image_raw/compressed` guidance),
  map3d_uplink -> server rtabmap_mapping / map3d_manager (`nodes/rtabmap_mapping/config/map3d_topics.yaml`, contract
  pinned by `tests/test_map3d_topics.py`), marker_detector (`marker_detector/config.py`, `tof_plane.py` reads
  `/head_rgbd/depth/image_raw`), web_ui RGB-D view and 3D layer (`client.yml:2523, 2621-2622`), calibration
  validation topics.
- `/ground/*`: nav2 `ground_voxel_layer` (obstacle/drop points), costmap_zones (`warning_areas`), mcp_server
  (`state`: floor_guard, arm, grasp_tools, monitor, map summary), web_ui ground layer and ground_recorder.
- `/head_camera/*`: marker_detector, mcp_server `front` camera, web_ui. Frames `tof_optical_frame` /
  `head_camera_optical_frame` are hardcoded in `mcp_server/head_kinematics.py`, `rtabmap_mapping/config/map3d_topics.yaml`,
  `marker_detector/tof_plane.py`, `shared/ros2_common/head_rgbd_calibration/{calibrate,extrinsics,floor_fit,
  urdf_handoff,head_arm_fit}.py`, `arducam_tof/config.py`.

## 3. head_rgbd_cpp and ground_estimator_cpp: sensor assumptions

head_rgbd_cpp (`nodes/head_rgbd_cpp/README.md`, `src/head_rgbd_node.cpp`, config in `client.yml:1452-1504`):

- Built for a *separate* depth and RGB sensor: ApproximateTime pairing of ToF depth with a *CompressedImage* RGB
  (raw RGB never subscribed), `sync.max_pair_dt_s` 0.06, tf2 motion compensation through `odom`, own z-buffer
  registration of depth into the RGB camera, guided-filter upsampling (`upsampling.enabled` needs `output.divisor` 2).
- Depth must be 16UC1 (`head_rgbd_node.cpp:26,244`, drop reason `bad_input`); ToF intrinsics come from a *file*
  (`tof_calibration_file`, `camera_calibration_parsers::readCalibration`, `:115-133`), not from the depth camera_info
  topic; ToF image size must equal the file (`size_mismatch`). Fixture `test/data/tof_calibration.yaml` is 240x180.
- Jump-edge "flying pixel" filter (`src/core/tof_filter.cpp`) is ToF-specific; `depth.max_m` 4.0.
- Geometry from TF only (head URDF). Every output set carries the RGB header, which RTAB-Map relies on.
- With a D435, realsense-ros already produces registered depth (`aligned_depth_to_color`) with factory extrinsics,
  so registration + ToF intrinsics file + ApproximateTime pairing become either redundant or must be bypassed. The
  downstream contracts (`/head_rgbd/*` topic names, 16UC1 mm, exact-stamp sets, RGB camera_info, compressed RGB)
  are the seam.

ground_estimator_cpp (`nodes/ground_estimator_cpp/README.md:22-41`, config `client.yml:2146-2280`):

- Main input `cloud_topic` `/tof/points_filtered` (base_link via TF), expected 10 Hz (temporal windows sized in frames:
  `temporal.window_frames` 8, `min_frames` 4, hysteresis in frames; latency docs say "/ 10 Hz").
- Noise model tuned for ToF (cell spread gate `max_spread_m` 0.025, lethal-mark rules built from the 2026-10-21 ToF
  capture, occlusion gate D16 for ToF mixed pixels).
- Guidance: `/head_rgbd/depth_upsampled/*`, `/head_rgbd/rgb/image_raw/compressed`, and `guidance.tof_info_topic`
  `/tof/camera_info` as the "ToF model" for no-return areas (`guidance.no_return.tof_margin_px`).
- Replay tests (`test/test_replay.cpp`, `test/data/replay/{home,box,ramp}.bin`) are recordings of
  `/tof/points_filtered` "decimation 4, 5 cm voxel" (README:510) plus `tof_frames` subsets: a D435 swap invalidates
  them as acceptance evidence (they still run as regression).
- IMU prior `/imu/data` (BNO055); D435 has no IMU, so nothing changes there.

## 4. Nav2 lidar-only state and how to re-enable

Commit 50905ee1 (2026-10-24), file `nodes/nav2_bringup/config/nav2_params.yaml`. Every disabled line carries the
comment `disabled (user 2026-10-24): lidar-only path planning until the RealSense D435 replaces the head ToF;
re-enable/replace here` (pinned >= 7 occurrences by `tests/test_nav_lidar_only.py:87-89`):

| Item | Line | State |
|---|---|---|
| local `ground_cost_filter` (KeepoutFilter) | 224 | `enabled: false` |
| local `tof_voxel_layer` (STVL on `/tof/points_filtered`) | 265 | `enabled: false` |
| local `ground_voxel_layer` (STVL on `/ground/*`, clear on `/tof/points_filtered`) | 363 | `enabled: false` |
| collision_monitor `StopBox.sources_names` | 704-705 | `["scan"]` (was `["scan","tof_obstacles"]`) |
| `CliffStop` polygon | 812 | `enabled: false`, `sources_names: ["tof_cliff"]` kept |
| `tof_obstacles` source | 864 | `enabled: false` |
| `tof_cliff` source | 871 | `enabled: false` |

Re-enable = flip those lines (or re-point their `topic:` to a D435-derived cloud) and update the pinning tests:
`tests/test_nav_lidar_only.py` (asserts no enabled layer/source reads `/tof`, `/ground`, `/head_rgbd`),
`tests/test_nav_tof_collision.py`, `tests/test_nav_tof_voxel_layer.py`, `tests/test_nav_ground_voxel_layer.py`,
`tests/test_nav_footprint.py`, `tests/test_tof_obstacle_wiring.py`. STVL/collision heights, 2 m range and
"inside tof_obstacle_cloud max_depth (3 m)" comments are ToF-derived.

Ansible side (9098dea8, d09c3c3f): `head_rgbd_cpp` and `ground_estimator_cpp` `present: true, enabled: false`;
their metrics targets 19117/19119 were removed from `monitoring_agent_node_targets` (`client.yml:3643`, comment
"add their targets when enabling"). Pinned by `tests/test_head_rgbd_deploy.py::test_rgbd_fusion_and_ground_nodes_are_disabled_not_removed`,
`tests/test_ground_estimator_deploy.py`, `tests/test_cpp_cutover.py:56`, `tests/test_map3d_uplink_config.py`.
`arducam_tof`, `tof_obstacle_cloud`, `head_camera_cpp` remain enabled (agent MCP tools only).

## 5. URDF / TF for the head

- Head URDF: `nodes/web_ui/urdf/head.urdf` (112 lines). Chain: `base_link -> head_mount_joint (fixed, 0.15 0.10 0.31)
  -> head_pan_base -> head_pan (rev z, +-1.5708) -> head_pan_link -> head_tilt (rev y, +-0.5236, origin z 0.04) ->
  head_link -> {head_camera_joint (0.02 -0.025 0) -> head_camera_link -> head_camera_optical_frame,
  tof_joint (0.02 0.025 0) -> tof_link -> tof_optical_frame}`; also `head_post` visual. Header says origins are CAD
  estimates.
- Publisher: `head_state_publisher` = `robot_state_publisher` via
  `nodes/head_state_publisher/launch/head_state_publisher.launch.py` (remaps `/head/joint_states`,
  `/head/robot_description`). `launch/head_description.py` loads
  `shared/ros2_common/head_rgbd_calibration/urdf_text.py::apply_overrides` by path and applies
  `/var/lib/ros2/head_rgbd_calibration/applied/head_chain.json` (joints `HEAD_CHAIN_JOINTS = head_mount_joint,
  head_pan, head_tilt, head_camera_joint`, `urdf_text.py:18`) and `tof_joint.json` (`tof_joint`).
  Node type `client.yml:841-858`, `src_paths` include the URDF and `urdf_text.py`.
- `nodes/web_ui/urdf/robot.urdf` has no head/camera links. `static_tf_publisher` (`client.yml:2048-2073`) publishes only
  `imu_link`, `laser_frame` ("No overhead camera frame is published").
- Who else parses head.urdf / frames: mcp_server `head_kinematics.py` (`head` config `client.yml:2852-2862`, front
  camera mount = zero offset on `head_camera_link`, `client.yml:2877-2895`), head_rgbd_calibration
  `head_urdf_file` (`client.yml:1541`), `urdf_handoff.py` (`CHAIN_JOINTS` head_camera_joint, head_camera_optical_joint,
  tof_joint, tof_optical_joint), scripts/head_rgbd_calibrate.yaml `outputs.urdf`.
- Open question for the plan: whether the D435 sits on the pan/tilt (child of `head_link`) or on a fixed mount
  (child of `base_link` via static_tf_publisher or a URDF fixed joint). Either way a `* -> camera_link` joint is needed
  and realsense-ros' own `camera_link -> camera_*_optical_frame` TF must not be duplicated by the URDF.

## 6. Calibration tooling

- `scripts/head_rgbd_calibrate.py` (123 lines) + `scripts/head_rgbd_calibrate.yaml` + `scripts/head_rgbd_calibration/
  {config,frames,pipeline,validate}.py`: offline CLI, modes `capture, capture-validation, calibrate, urdf, validate`;
  bags over SSH (`capture.host client.ros2.lan`, `.logs/head_rgbd`). Assumes ToF amplitude + depth + `/tof/camera_info`
  and the CM3 mapping stream; ChArUco 6x5 40/30 mm DICT_5X5_100; `selection.tof_upscale` 2 for "240x180 markers";
  limits rgb/tof/stereo RMS. Outputs: `nodes/head_camera_cpp/config/head_camera_calibration.yaml`,
  `nodes/head_rgbd_cpp/config/tof_calibration.yaml`, `nodes/head_rgbd_cpp/config/head_rgbd_extrinsics.yaml`,
  `nodes/web_ui/urdf/head.urdf` (rewrites `tof_joint`). **None of the three YAMLs is committed today**
  (`nodes/head_rgbd_cpp/config` holds only `head_rgbd.yaml`; `nodes/head_camera_cpp/config` does not exist).
- Robot-side service `nodes/head_rgbd_calibration` (HTTP 127.0.0.1:18110, proxied by web_ui Calibrate page; instance
  `client.yml:1506-1780`): sessions under `/var/lib/ros2/head_rgbd_calibration/sessions`, Apply copies
  `head_camera_calibration.yaml`, `tof_calibration.yaml`, `head_rgbd_extrinsics.yaml`, `tof_joint.json` (+ head floor
  fit / head_arm results write `head_chain.json`) into `applied/` then `applied.json`; a systemd path unit
  (`ansible/playbooks/templates/ros2-head-rgbd-calibration-apply.path.j2`) restarts `restart_units`
  (`ros2-head_camera_cpp`, `ros2-head_rgbd_cpp`, `ros2-head_state_publisher`). Floor sweep (`floor_sweep.py`) fits the
  head chain from `/tof/points` planes over a pan/tilt grid. Seeding of `applied/` from committed files:
  `ansible/playbooks/tasks/head_rgbd_calibration_setup.yml:32-51` (force: false).
- Maths library `shared/ros2_common/head_rgbd_calibration/` (calibrate, charuco, intrinsics, extrinsics, floor_fit,
  head_arm_fit, chain, urdf_handoff, urdf_text, frames, poses, views, hints, arm_pose_check). `calibrate.py` fits
  `FITS = ("rgb", "tof", "stereo")` and `detect_tof` on amplitude: RGB-ToF stereo is ToF-specific. Floor fit and
  head_arm fit are sensor-agnostic in principle but hardcode `tof_optical_frame` / `head_camera_optical_frame`.
- Readers of `applied/`: arducam_tof (`tof_calibration_file`), head_camera_cpp (`camera_info_file`), head_rgbd_cpp,
  head_state_publisher, mcp_server (`head.py`, front intrinsics `applied/head_camera_calibration.yaml` 576x324 scaled
  to 1152x648), web_ui `server.py` (`head_applied_calibration_dir`, `client.yml:2440`), frontend `headFloorApi.ts`.
- `scripts/solve_camera_mount.py` (92 lines): generic marker-sample mount solver over
  `shared/ros2_common/camera_geometry.py` (`solve_mount_pose`); prints a mount YAML for the mcp_server `cameras`
  config (`gripper` on `gripper_link`, `front` on `head_camera_link`). Not wired to the head chain; used for the
  gripper camera (other slice).

## 7. Pi resource budget

- `client.yml:18-33`: system.slice MemoryLow = sum of enabled nodes' `memory_low`; tiers critical 1000 / normal 500 /
  agent 300 / background 100; no `cpu_quota` / `memory_high` on robot types (pinned by `tests/test_unit_cgroups.py`).
- `tests/test_unit_cgroups.py:81-82,380-383`: `MAX_MEMORY_LOW_SUM_MIB = 2560` on `CLIENT_RAM_MIB = 8 * 1024`.
  **Current enabled sum (computed with the test's own rule): 2432 MiB.** Enabling `realsense_d435i` (128M) alone
  reaches 2560 exactly; also enabling `head_rgbd_cpp` (48M) and `ground_estimator_cpp` (48M) gives 2656 and fails the
  test unless `arducam_tof` (64M) / `tof_obstacle_cloud` (64M) are disabled or figures re-measured.
  `realsense_d435i` is listed in the NORMAL tier types (`test_unit_cgroups.py:41`).
- `memory_max: 512M` for the RealSense is an estimate; no measurement exists ("no series"). CPU history:
  `tests/test_client_cpu_load.py:1` "load ~13 on 4 cores" (2026-10-08). 848x480x90 depth + 1280x720x30 colour with
  `align_depth.enable:=true` runs the alignment on the Pi CPU (librealsense host-side processing); the 90 fps depth
  profile is far above the 10 Hz the consumers are sized for (laser_filter is Python sensor_msgs_py,
  `tof_obstacle_cloud` sized for 2700 points). No rate/decimation filters are set in the launch command.

## 8. Related epics (open tasks only)

| Plan | Open tasks | Overlap with D435 epic |
|---|---|---|
| in-progress/rgb-d-ground-estimation-and | `ground-final-suite-runbook` [escalated] (runbook `docs/runbooks/ground-estimation.md`, preconditions head_rgbd_cpp) | yes: the ground acceptance this epic restores |
| in-progress/drivable-ground-tof-filterin | 20 open: `ground-live-acceptance`, `ground-voxel-layer-on`, `ground-estimator-live-scene`, `guidance-live-check-and-cpu`, `guidance-replay-acceptance`, `live-report-guidance-metrics`, `tof-amplitude-gate`, `tof-filter-live-check`, `tof-flying-pixel-retune`, `capture-baseline-recordings`, `capture-gated-recordings`, `head-tilt-range-widened`, `calib-docs-and-test-index`, `nav2-goal-across-the-scene`, `stopbox-backing-and-turns`, `retro-fixes-live-check`, `agent-retro-skill-lessons`, `map3d-uplink-and-leader-brid`, `tests-test-named-maps-py-tes` [in-progress]; `collision-source-stall-fix` [blocked] | heavy: ToF-specific tasks (amplitude gate, flying pixel retune, ToF filter live check, recordings) become moot or need re-scoping; ground live acceptance / voxel layer on are the same goal |
| in-progress/head-chain-floor-plane-fit-a | none open (all tasks done; README `stage: build`) | yes: floor fit is the head-chain recalibration path, fed by `/tof/points` |
| in-progress/tof-depth-fix-and-ground-noi | `head-remount-floor-runbook` [blocked] | yes: head remount + recalibration runbook |
| in-progress/tof-3d-obstacle-detection-fo | `final-suite-and-runbook` [escalated] | yes: the nav2 ToF wiring the epic re-points |
| todo/head-rgb-d-fusion-runbook | `rgbd-final-suite-and-runbook` [todo] (files: committed calibration YAMLs, head.urdf, client.yml) | yes: RGB-D fusion runbook for the ToF+CM3 pair |
| done/head-rgb-d-calibration-in-th, done/head-rgb-d-fusion-of-camera | none | background only |

## 9. Verification commands (verbatim from the repo)

- C++: `scripts/cpp-node-test.sh head_rgbd_cpp`, `scripts/cpp-node-test.sh ground_estimator_cpp` (docker,
  ros:jazzy-ros-base, ends with `cpp-node-test: <pkg> PASS|FAIL`); `--lint-only` for ament linters; all:
  `uv run poe test-cpp`, `uv run poe lint-cpp`.
- Python nodes (per node dir, `uv run --frozen pytest tests -q && uv run --frozen poe lint`):
  `nodes/head_rgbd_calibration`, `nodes/bridges/arducam_tof`, `nodes/laser_filter`, `nodes/static_tf_publisher`.
  All nodes lint: `uv run poe lint-nodes`.
- Root: `uv run pytest tests -q` (`.mise.toml` `test`), `uv run poe lint` (ruff check/format + vulture on tests, shared).
- Ansible: `uv run poe lint-ansible`, `uv run poe test-ansible`.
- Deploy: `./scripts/deploy-nodes.sh client <node> 2>&1 | tee .logs/deploy-client.log` (never ansible-playbook).
- Config-only nodes (tof_obstacle_cloud, nav2_bringup, head_state_publisher, realsense_d435i) are covered by root
  tests (`tests/test_tof_obstacle_cloud_config.py`, `test_nav_*`, `test_head_config_consistency.py`, `test_unit_cgroups.py`).

## 10. Tree-state results (run 2026-10-27 on d0bab034)

All green. Logs in the session scratchpad (not committed).

| Check | Command | Result line |
|---|---|---|
| head_rgbd_cpp | `scripts/cpp-node-test.sh head_rgbd_cpp` | `Summary: 129 tests, 0 errors, 0 failures, 0 skipped` / `cpp-node-test: head_rgbd_cpp PASS` |
| ground_estimator_cpp | `scripts/cpp-node-test.sh ground_estimator_cpp` | `Summary: 464 tests, 0 errors, 0 failures, 0 skipped` / `cpp-node-test: ground_estimator_cpp PASS` |
| head_rgbd_calibration | `uv run --frozen pytest tests -q`; `uv run --frozen poe lint` | `407 passed in 105.11s`; `All checks passed!` `39 files already formatted` |
| arducam_tof | same | `87 passed in 0.92s`; `All checks passed!` `6 files already formatted` |
| laser_filter | same | `57 passed in 0.38s`; `All checks passed!` `10 files already formatted` |
| static_tf_publisher | same | `15 passed in 0.05s`; `All checks passed!` `9 files already formatted` |
| root tests | `uv run pytest tests -q` | `2156 passed, 9 skipped in 173.03s` |
| root lint | `uv run poe lint` | `All checks passed!` `167 files already formatted` (vulture clean) |
| ansible | `uv run poe test-ansible` | `Passed: 0 failure(s), 0 warning(s) in 145 files processed of 161 encountered.` + 10 playbook `--syntax-check` OK, exit 0 |

## 11. Unknowns

- D435 USB product id on the Pi (udev rule matches 0b3a only) and whether `ros-jazzy-realsense2-camera` from apt
  supports the D435 firmware present; settle with `lsusb` / `rs-enumerate-devices` on the client (via the Ansible
  skill, not ad-hoc SSH).
- Whether the D435 is on the pan/tilt (child of `head_link`) or fixed; whether the ToF and CM3 stay mounted.
- Actual Pi CPU/memory of the realsense node at 848x480x90 + 1280x720x30 with align_depth (no measurement exists).
- Whether `head_rgbd_cpp` should be kept (fed by D435 aligned depth) or bypassed (D435 publishes `/head_rgbd/*`
  contract directly via remaps / a thin relay); RTAB-Map contract expects 288x162 depth + 576x324 RGB at 2 Hz (D3
  budgets in `map3d_topics.yaml`).
- Compressed colour from realsense-ros needs `image_transport` plugins (`ros-jazzy-compressed-image-transport`) on the
  client; not verified as installed.
- realsense-ros topic/frame names above are from upstream defaults, not verified against the installed version.
