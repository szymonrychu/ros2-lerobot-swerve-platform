# Stereo Depth (IMX219 pair) Implementation Plan

> For agentic workers: execute through `workbench:plan-execute` with
> `docs/plans/2026-10-13-stereo-depth-tasks.json` as `args.tasks`. Each task is test-first: the test is written and
> seen failing before the code that makes it pass. Steps use checkbox syntax.

**Goal:** Two IMX219 cameras (left cam0, right cam1) run synced at 5 fps through the restored camera_ros SyncMode patch,
a restored and adapted `stereo_depth` node turns each synced pair into 16UC1 mm depth on `/stereo/depth/image_raw`, the
Head tab shows it, and every `/overview_camera` consumer keeps working.

**Architecture:**

- The Ansible node `overview_camera` stays (name, unit, launch path, cgroup tier). Its launch grows from one to two
  camera_ros `CameraNode`s in one `component_container_mt` (one CameraManager per process). The left camera keeps
  namespace `/overview_camera`, so `/overview_camera/image_raw`, `/compressed` and `camera_info` keep their names (D2).
  The right camera publishes `/stereo/right/image_raw` and `/stereo/right/camera_info`.
- Both cameras: `sensor_mode 1640:1232`, ISP output 480x360 (configurable), `FrameDurationLimits [200000, 200000]`
  (5 fps), `AeExposureMode short`, `SyncMode` server (left) / client (right), no AfMode. `camera_info_url` is set per side
  only when its calibration file has a valid `P`.
- `nodes/stereo_depth` (new uv node, restored from git and rewritten from "disparity in" to "image pair in"): pairs
  the two `image_raw` topics with `message_filters.ApproximateTimeSynchronizer`, rectifies with maps built once from
  the calibration files, runs StereoSGBM 3-way (48 disparities, block 5), and publishes depth on the ToF contract.
- Calibration and benchmark tools live in the same package (`python3 -m stereo_depth.calibrate`,
  `python3 -m stereo_depth.bench`), config-file driven, so they share the OpenCV venv and the calibration file format.
- Unchanged: web_ui serializer, `tof_*` config keys and labels, mcp_server `front_camera` topic, arducam_tof code
  (stays installed, `enabled: false`), libcamera tag and camera_ros commit.

Fixed interfaces (from the design, not owned by any task):

| Interface | Value |
|---|---|
| Left image / compressed / info | `/overview_camera/image_raw` (bgr8), `/overview_camera/image_raw/compressed`, `/overview_camera/camera_info` |
| Right image / info | `/stereo/right/image_raw` (bgr8), `/stereo/right/camera_info` |
| Depth | `/stereo/depth/image_raw` (`sensor_msgs/Image` 16UC1, mm, 0 = invalid), `/stereo/camera_info` (rectified left `P`, depth header); optional `/stereo/disparity` (`stereo_msgs/DisparityImage`, off by default) |
| Calibration files | `/var/lib/ros2/camera_calibration/stereo_left.yaml`, `stereo_right.yaml` (ROS camera_info YAML with K, D, R, P; `P_right[3] = -fx*B`) and `stereo.yaml` (R, T, Q, image size, RMS) |
| Metrics port | `stereo_depth` 19113 (`metrics_port`) |

**Tech stack:** Python 3.12, uv, pydantic, OpenCV (`opencv-python-headless`), numpy, ROS2 Jazzy (rclpy,
message_filters), Ansible. Tests: `uv run pytest`, `uv run poe lint`, `uv run poe lint-nodes`, `uv run poe test-ansible`.

**Spec:** `docs/plans/2026-10-13-stereo-depth-design.md` (executors read both: the design doc for the why and the
decisions, this plan for the what). Brief: `docs/plans/2026-10-13-stereo-depth-briefs/stereo-brief.md`.

## Project brief

What: a ROS2 lerobot swerve robot platform (Pi 5 client, Pi 4 server) with a browser dashboard and an MCP agent.
How: uv Python and launch-only nodes on Raspberry Pis, deployed by Ansible; cameras through a source-built libcamera fork and camera_ros.
Why: the robot gets a working depth source (the Arducam ToF never came up) at 5 fps within the client's free CPU, with the overview view kept.

## Global constraints

- Tests first (red, then green). Type hints, constants on top, imports at the top, docstrings with types, pydantic
  config, config files over CLI flags.
- No placeholder data on topics: no calibration, no fresh synced pair or a skew above the slop means log, count and
  publish nothing.
- Restore prior code with `git show 08bc9b8^:<path> > <path>` (or `> <new path>` where the plan names a new location),
  then adapt. Never retype restored files from memory.
- Plain hyphens and straight quotes only. Docs (READMEs, `tests/README.md`, `ansible/README.md`) are acceptance
  conditions of the task that changes the code, never a separate task.
- `tests/README.md` must list every root test file and function it touches (a test enforces this for
  `test_overview_camera*.py`).
- No deploys inside tasks; deploy happens after the run (see Execution).
- Model: sonnet, except T1 (camera_ros SyncMode and Ansible build) and T3 (stereo_depth node) on opus.

## Research inputs

| Source | Entry | Lands in |
|---|---|---|
| stereo-brief.md section 0 | prior stereo pipeline in git before 08bc9b8 (SyncMode patch, stereo_camera launch helpers, stereo_depth node, tests) | T1, T2, T3 |
| stereo-brief.md plan input 1 | boot overlays to two IMX219, imx219-removal inverted, reboot only on change | T1 |
| stereo-brief.md plan input 2 | camera_ros SyncMode patch in `colcon_source[].patches` | T1 |
| stereo-brief.md plan input 3 | two CameraNodes in one container, select by libcamera ID, calibration gate | T2 |
| stereo-brief.md plan input 4 | stereo_depth node, 16UC1 mm, drop counters, numpy guard | T3 |
| stereo-brief.md plan input 5 | Ansible node type, cgroup tier, metrics port, lint-nodes | T3 (lint-nodes, CI), T6 (Ansible, tier, metrics) |
| stereo-brief.md plan input 6 | repoint consumers | T5 (D2 keeps the topic names, so only the head depth topic and stale sizes change) |
| stereo-brief.md plan input 7 | stereo calibration tool | T4 (inside the stereo_depth package) |
| stereo-brief.md plan input 8 | on-device benchmark and sync check | T4 (tool), Execution (on-device run after deploy) |
| stereo-brief.md section 1 | ISP downscale from 1640x1232, not the 640x480 crop mode | T2 |
| stereo-brief.md section 2 | SyncMode multicast risk on Wi-Fi down | T2 README, Execution checks |
| stereo-brief.md section 5 | arducam_tof as config/metrics template; `disparity_to_depth_mm` reuse | T3 |
| stereo-brief.md section 6 | ChArUco or chessboard, `stereoCalibrate` + `stereoRectify alpha 0`, RMS gate | T4 |
| design D1 | 480x360, 48 disparities, 5 fps (overrides the brief's 640x480 / 64) | T2, T3, T4 |

## Decision coverage

| Decision | Tasks |
|---|---|
| D1 IMX219 pair on cam0 and cam1, ISP-downscaled 480x360, SGBM 48 disparities on the client, about 5 fps | T1, T2, T3, T4, T6 |
| D2 Depth on the ToF contract; the left camera keeps the overview role | T2, T3, T5 |

## Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|
| `overview-camera-config` | T1 | T2 | `ansible/group_vars/client.yml` |
| `stereo-calibration-files` | T3 | T4 | `nodes/stereo_depth/stereo_depth/calibration.py` |
| `stereo-depth-config` | T3 | T4, T6 | `nodes/stereo_depth/stereo_depth/config.py` |

- `overview-camera-config`: the deployed `overview_camera` entry config keys `left_camera_id` and `right_camera_id`
  (flat keys merged into the launch section), plus the node type resources.
- `stereo-calibration-files`: load, validate and write the camera_info YAMLs and `stereo.yaml`; build rectification
  maps. The launch gate in T2 checks only `P` validity of the same standard ROS format (it runs under system Python and
  cannot import the node package).
- `stereo-depth-config`: pydantic `DepthConfig`, env `STEREO_DEPTH_CONFIG`, default `/etc/ros2/stereo_depth/config.yaml`,
  keys listed in T3.

## Review focus

- T1: the boot tasks reboot only on change and the SyncMode patch applies to camera_ros `8f792e27` unchanged; nothing
  installs apt camera_ros or libcamera.
- T2: left and right never swap silently (ID first, index fallback with a warning); `camera_info_url` is empty unless
  the calibration file has a valid `P`; the left namespace still yields the exact `/overview_camera/...` topic names.
- T3: no publish without valid calibration or a fresh pair within slop; invalid, unmatched and out-of-range pixels are
  0; depth within 2% of f*B/d on a synthetic shift; the 5 Hz cap; numpy and OpenCV declared.
- T4: the writer's output is accepted by both the T3 loader and the T2 gate; refuses to write above the RMS limit.
- T6: the background tier is below every robot tier; the metrics target exists because the node is enabled.

### Task 1: Boot overlays and camera_ros SyncMode build

**Files:**
- Modify: `ansible/playbooks/tasks/overview_camera_boot_config.yml`
- Create: `nodes/overview_camera/patches/camera_ros-0001-expose-sync-controls.patch` (restored)
- Modify: `ansible/group_vars/client.yml`
- Modify: `ansible/README.md`, `nodes/bridges/arducam_tof/README.md`, `.claude/skills/ansible-deploy/SKILL.md`, `tests/README.md`
- Test: `tests/test_overview_camera_config.py`

**Interfaces:**
- Consumes: `git show 08bc9b8^:ansible/playbooks/tasks/stereo_camera_boot_config.yml`,
  `git show 08bc9b8^:nodes/stereo_camera/patches/camera_ros-0001-expose-sync-controls.patch`,
  `git show 08bc9b8^:tests/test_stereo_camera_config.py` (boot and patch tests to port).
- Produces: `overview-camera-config` (entry config keys `left_camera_id: ""`, `right_camera_id: ""`).

**Ownership:**
- owns: overview-camera-config
- uses: none
- dependsOn: none

**Requirement:**
- The boot task file writes `camera_auto_detect=0`, `dtoverlay=imx219,cam0` and `dtoverlay=imx219,cam1`, removes
  `dtoverlay=imx708,cam0` and `dtoverlay=arducam-pivariety,cam1` with `state: absent`, and no task removes imx219 lines.
  Every task is aarch64-only, registers its result, and the client reboots only when one of them changed. File name,
  include and tags (`boot`, `overview_camera`) are unchanged.
- `camera_ros-0001-expose-sync-controls.patch` is restored to `nodes/overview_camera/patches/` and adds
  `IF(rpi::SyncMode)` and `IF(rpi::SyncFrames)`; the camera_ros `colcon_source` entry lists it under `patches`. The
  libcamera tag `v0.7.2+rpt20260817` and camera_ros commit `8f792e27...` are unchanged; no apt camera_ros or libcamera.
- `overview_camera` node type comment says two IMX219 (left cam0, right cam1); memory re-estimated for two cameras
  (MemoryLow 128M, MemoryMax 384M, marked as an estimate); tier stays normal robot (weight 500).
- `overview_camera` ros2_nodes entry config holds exactly `left_camera_id: ""` and `right_camera_id: ""`.
- arducam_tof entry comment and `camera_index` comment say cam1 now carries the right IMX219, the node has no CSI port
  and stays `enabled: false`; its README says the same.
- `tests/test_overview_camera_config.py` pins all of the above (boot tasks inverted, patch listed and present with both
  `IF(...)` lines, entry config keys, memory). `ansible/README.md` (camera boot overlay paragraph, memory table row),
  `SKILL.md` overview_camera row and `tests/README.md` (each changed or new test function) are updated.

**Value:** the robot boots with both IMX219 cameras and a camera_ros build that can sync them.

**Steps:**
- [ ] Restore the patch with `git show 08bc9b8^:nodes/stereo_camera/patches/camera_ros-0001-expose-sync-controls.patch > nodes/overview_camera/patches/camera_ros-0001-expose-sync-controls.patch`.
- [ ] Write the tests (port the boot and patch tests from `git show 08bc9b8^:tests/test_stereo_camera_config.py`).
- [ ] Run them. Expected: FAIL (imx708 overlay present, no camera_ros patch).
- [ ] Implement.
- [ ] Run them. Expected: PASS.
- [ ] Lint: `uv run poe lint && uv run poe test-ansible`.
- [ ] Commit: `feat(overview_camera): IMX219 pair boot overlays and camera_ros SyncMode patch`

**Verify:**
`uv run pytest tests/test_overview_camera_config.py tests/test_ansible_deploy_speed.py -v && uv run poe lint && uv run poe test-ansible`
prints all passed and ansible-lint / syntax-check with no failures.

### Task 2: Two synced CameraNodes in the overview_camera launch

**Files:**
- Modify: `nodes/overview_camera/launch/overview_camera.launch.py`, `nodes/overview_camera/launch/overview_camera_config.py`, `nodes/overview_camera/config/params.yaml`
- Modify: `nodes/overview_camera/README.md`, `nodes/README.md`, `tests/README.md`
- Test: `tests/test_overview_camera_launch_config.py`, `tests/test_overview_camera_config.py`

**Interfaces:**
- Consumes: `overview-camera-config` from T1; `git show 08bc9b8^:nodes/stereo_camera/launch/stereo_camera_config.py`,
  `.../stereo_camera.launch.py`, `.../config/params.yaml`, `git show 08bc9b8^:tests/test_stereo_camera_launch_config.py`.
- Produces: the left and right image topics and the calibration gate (fixed interfaces above).

**Ownership:**
- owns: none
- uses: overview-camera-config
- dependsOn: T1

**Requirement:**
- One `component_container_mt` with intra-process comms holds two `camera::CameraNode`s: left in `/overview_camera`
  (topics exactly as today) and right in `/stereo/right`.
- Each side is selected by its libcamera ID (`left_camera_id`, `right_camera_id`), falling back to index 0 / 1 with a
  logged warning (restored `camera_selector`).
- Both sides: `sensor_mode "1640:1232"`, `width 480`, `height 360` (configurable in params.yaml), `role video`,
  `format RGB888`, `FrameDurationLimits [200000, 200000]`, `AeExposureMode short`; `SyncMode` 1 on left, 2 on right
  (restored `SYNC_MODES` / `enum_value`); no `AfMode`. Left keeps `frame_id overview_camera_optical_frame` and
  `jpeg_quality 80`; right gets `stereo_right_optical_frame`.
- `camera_info_url` is `file:///var/lib/ros2/camera_calibration/stereo_{left,right}.yaml` only when that file has a valid
  12-element `P` with non-zero focal terms (restored `calibration_problem` / `projection_is_valid`); otherwise empty and
  the reason is logged. No image_proc, stereo_image_proc or point cloud stages.
- `test_overview_camera_launch_config.py` and the launch/params/README tests of `test_overview_camera_config.py` pin the
  above (the "no SyncMode / no stereo" assertions are replaced). The overview_camera README documents both cameras,
  topics, sync, the Wi-Fi multicast caveat, `cam -l` for both IDs, and links the calibration procedure in
  `nodes/stereo_depth/README.md`. `nodes/README.md` updates the overview_camera line and adds a `stereo_depth/` line.
  `tests/README.md` lists every changed or new test function.

**Value:** both cameras stream synced at 5 fps while the overview view keeps its topic names.

**Steps:**
- [ ] Restore references to a scratch path outside the repo with `git show 08bc9b8^:nodes/stereo_camera/launch/stereo_camera_config.py > <scratch>/stereo_camera_config.py`, and port the needed helpers into `overview_camera_config.py`.
- [ ] Write the tests. Run them. Expected: FAIL (single camera, no sync, 640x480).
- [ ] Implement. Run them. Expected: PASS.
- [ ] Lint: `uv run poe lint`.
- [ ] Commit: `feat(overview_camera): left and right IMX219 CameraNodes with SyncMode and calibration gate`

**Verify:**
`uv run pytest tests/test_overview_camera_launch_config.py tests/test_overview_camera_config.py -v && uv run poe lint`
prints all passed and lint clean.

### Task 3: stereo_depth node (SGBM on synced pairs, 16UC1 mm)

**Files:**
- Create (restored then adapted): `nodes/stereo_depth/pyproject.toml`, `nodes/stereo_depth/uv.lock`, `nodes/stereo_depth/stereo_depth/{__init__,__main__,config,depth,messages,node}.py`, `nodes/stereo_depth/tests/{conftest,test_config,test_depth,test_messages}.py`, `nodes/stereo_depth/README.md`
- Create: `nodes/stereo_depth/stereo_depth/calibration.py`, `nodes/stereo_depth/tests/test_calibration.py`, `nodes/stereo_depth/tests/test_node.py`
- Modify: `.github/workflows/ci.yml`, `pyproject.toml`, `scripts/lint-all-nodes.sh`, `MEMORY.md`

**Interfaces:**
- Consumes: the fixed image topics and calibration paths; `git show 08bc9b8^:nodes/stereo_depth/<path>` for each file;
  `nodes/bridges/arducam_tof` as the uv / metrics / config template.
- Produces: `stereo-calibration-files`, `stereo-depth-config`, the depth topics and the metrics on port 19113.

**Ownership:**
- owns: stereo-calibration-files, stereo-depth-config
- uses: none
- dependsOn: none
- size: large
- sizeReason: one new node package (pyproject, lock, six modules, five test files, README) plus the three lint/CI registry lines that must land with it (test_ci_workflow fails on a node dir missing from the matrix); splitting the package leaves half a node with no runnable entry point.

**Requirement:**
- uv project like arducam_tof (ruff, vulture, poe lint/test, `ros2-metrics` path dep, `package = false`); declares
  `numpy` and `opencv-python-headless` (numpy guard); `uv.lock` committed; the poetry files are not restored.
- `config.py`: frozen pydantic `DepthConfig`, `extra="forbid"`, loaded from `STEREO_DEPTH_CONFIG` or
  `/etc/ros2/stereo_depth/config.yaml`; keys: `left_image_topic` (`/overview_camera/image_raw`), `right_image_topic`
  (`/stereo/right/image_raw`), `depth_topic`, `camera_info_topic`, `disparity_topic` (optional, publish off by default),
  `calibration_dir`, `num_disparities` (48, multiple of 16), `block_size` (5, odd), `p1`, `p2`, `uniqueness_ratio`,
  `speckle_window_size`, `speckle_range`, `disp12_max_diff`, `pre_filter_cap`, `min_depth_m` (0.3) < `max_depth_m` (4.0),
  `sync_slop_s` (0.002), `max_rate_hz` (5.0), `matcher_threads`, `frame_id`, `metrics_port` (19113).
- `calibration.py`: load and validate `stereo_left.yaml` / `stereo_right.yaml` (ROS camera_info YAML: K, D, R, P,
  size; right `P[3] = -fx*B`) and `stereo.yaml`; write them (used by T4); build `initUndistortRectifyMap` maps once
  (`CV_16SC2`). Missing or invalid files return None with the reason.
- `depth.py`: restored `disparity_to_depth_mm` plus StereoSGBM `MODE_SGBM_3WAY` on grey rectified images, fixed-point
  disparity / 16. A synthetic rectified pair shifted by a known disparity gives depth within 2% of f*B/d; invalid,
  unmatched and out-of-range pixels are 0.
- `node.py`: `message_filters.ApproximateTimeSynchronizer` (slop from config) on the two image topics; rate cap at
  `max_rate_hz`; publishes `/stereo/depth/image_raw` (16UC1 mm, left stamp, `frame_id`) and `/stereo/camera_info`
  (rectified left `P`, same header). Nothing is published without valid calibration, with a size mismatch, or with a
  skew above slop; each case logs (throttled) and increments `stereo_depth_frames_dropped_total{reason}`. Metrics:
  `stereo_depth_frames_published_total`, dropped counter, matcher seconds and pair skew seconds histograms. While
  calibration is missing it re-checks the files periodically, so a new calibration needs no redeploy. No cv_bridge
  (numpy reshape of `Image.data`).
- Tests mock rclpy and message_filters (arducam_tof pattern). README: topics, config keys, metrics, the no-placeholder
  behaviour, the CPU estimate from the brief, and how to run the tests. Registered in `.github/workflows/ci.yml`
  matrix, root `pyproject.toml` `lint-nodes` and `scripts/lint-all-nodes.sh`. `MEMORY.md` records the decision: IMX219
  stereo replaces the IMX708 and the ToF, with the reasons (fixed lens vs VCM drift, ToF never came up, client CPU).

**Value:** the robot gets real depth from the stereo pair, and never fake depth.

**Steps:**
- [ ] Restore each prior file with `git show 08bc9b8^:nodes/stereo_depth/<path> > nodes/stereo_depth/<path>` (skip `poetry.lock`).
- [ ] Rewrite pyproject to uv; `uv lock`.
- [ ] Write the tests (synthetic shifted pair, calibration fixtures, node drops). Run. Expected: FAIL.
- [ ] Implement. Run. Expected: PASS.
- [ ] Lint: `uv run poe lint`, root `uv run poe lint-nodes`.
- [ ] Commit: `feat(stereo_depth): SGBM depth from the synced IMX219 pair on the ToF contract`

**Verify:**
`cd nodes/stereo_depth && uv run pytest tests -v && uv run poe lint && cd ../.. && uv run pytest tests/test_ci_workflow.py -v && uv run poe lint-nodes`
prints all passed and every node lint clean.

### Task 4: Stereo calibration and benchmark tools

**Files:**
- Create: `nodes/stereo_depth/stereo_depth/calibrate.py`, `nodes/stereo_depth/config/calibrate.yaml`, `nodes/stereo_depth/stereo_depth/bench.py`, `nodes/stereo_depth/config/bench.yaml`
- Modify: `nodes/stereo_depth/README.md`
- Test: `nodes/stereo_depth/tests/test_calibrate.py`, `nodes/stereo_depth/tests/test_bench.py`

**Interfaces:**
- Consumes: `stereo-calibration-files` (writer and loader), `stereo-depth-config` (matcher parameters).
- Produces: the calibration files at the fixed paths; a benchmark table printed to stdout.

**Ownership:**
- owns: none
- uses: stereo-calibration-files, stereo-depth-config
- dependsOn: T3

**Requirement:**
- `python3 -m stereo_depth.calibrate`, config from `STEREO_CALIBRATE_CONFIG` or `config/calibrate.yaml` (board type
  chessboard or ChArUco, squares, square and marker size, dictionary, capture count, image dir, output dir, RMS limit).
  Capture mode saves synced pairs from the two image topics; solve mode runs per-camera `calibrateCamera`,
  `stereoCalibrate` (`CALIB_FIX_INTRINSIC`) and `stereoRectify` (alpha 0) and writes the three files through
  `calibration.py`.
- On a synthetic board pair rendered with known K, D, R and T, the recovered baseline is within 1% and RMS under 0.5 px;
  the written files load in T3's loader and pass the T2 `P` gate check (same rule: 12 elements, non-zero focal terms).
  It refuses to write above the configured RMS limit.
- `python3 -m stereo_depth.bench`, config `config/bench.yaml` (sizes, numDisparities, matcher BM / SGBM 3WAY / SGBM,
  threads, runs): times each cell on a synthetic textured pair and prints median ms and cores at 5 fps. A test runs a
  one-cell grid and checks the table shape.
- README: the calibration procedure (mount, print board, capture, solve, check RMS, restart not needed), the bench
  command, and per-node test list including the new tests.

**Value:** the user can calibrate the mounted pair and measure the real Pi cost without writing code.

**Steps:**
- [ ] Write the tests. Run. Expected: FAIL (modules missing).
- [ ] Implement. Run. Expected: PASS.
- [ ] Lint: `uv run poe lint`.
- [ ] Commit: `feat(stereo_depth): config-driven stereo calibration and benchmark tools`

**Verify:**
`cd nodes/stereo_depth && uv run pytest tests -v && uv run poe lint` prints all passed and lint clean.

### Task 5: Head tab depth source and consumer checks

**Files:**
- Modify: `ansible/group_vars/client.yml`, `nodes/web_ui/config/default.yaml`, `nodes/web_ui/README.md`, `nodes/mcp_server/mcp_server/tools.py`, `nodes/mcp_server/README.md`, `tests/README.md`
- Test: `tests/test_mcp_server_config.py`, `nodes/web_ui/tests/test_map3d_backend.py`

**Interfaces:**
- Consumes: the fixed depth topic and the unchanged `/overview_camera` topics.
- Produces: the deployed and default head tab on `/stereo/depth/image_raw`.

**Ownership:**
- owns: none
- uses: none
- dependsOn: T2

**Requirement:**
- Head tab (client.yml and web_ui `default.yaml`): `tof_depth_topic: /stereo/depth/image_raw`, `tof_range_m: 4.0`,
  `tof_amplitude_topic` removed (no stereo amplitude); comments say the depth comes from stereo_depth. Arm
  `overview_camera_topic`, head `rgb_camera_topic`, the Overview tab and mcp_server `front_camera` stay
  `/overview_camera/image_raw/compressed`.
- Every `/overview_camera` consumer keeps working: web_ui arm, head and overview tabs, mcp_server `get_camera_image`,
  camera lookups and photo POIs (via `topics.front_camera`), and the agent through mcp_server. The pinned tests
  (`test_web_ui_tab_set`, `test_mcp_server_front_camera_is_the_compressed_overview_camera`, `test_map3d_backend` head
  defaults) assert the new depth topic, no amplitude, and the unchanged overview topics.
- mcp_server tool descriptions no longer claim the front camera is 640x480 (no hardcoded size); web_ui and mcp_server
  READMEs state 480x360 at 5 Hz and stereo depth on the Head tab. `tests/README.md` rows updated.

**Value:** the Head tab shows stereo depth and nothing that used the overview camera breaks.

**Steps:**
- [ ] Update the pinned tests. Run. Expected: FAIL (still `/tof/depth/image_raw`).
- [ ] Implement. Run. Expected: PASS.
- [ ] Lint: `uv run poe lint`, web_ui and mcp_server `uv run poe lint`.
- [ ] Commit: `feat(web_ui): head tab depth from stereo_depth; overview consumers unchanged`

**Verify:**
`uv run pytest tests/test_mcp_server_config.py -v && (cd nodes/web_ui && uv run pytest tests -q && uv run poe lint) && (cd nodes/mcp_server && uv run pytest tests -q && uv run poe lint)`
prints all passed and lint clean.

### Task 6: stereo_depth Ansible, background cgroup tier and metrics wiring

**Files:**
- Modify: `ansible/group_vars/client.yml`, `ansible/playbooks/deploy_nodes_client.yml`, `ansible/README.md`, `.claude/skills/ansible-deploy/SKILL.md`, `tests/README.md`
- Test: `tests/test_stereo_depth_config.py` (new), `tests/test_unit_cgroups.py`, `tests/test_node_metrics_wiring.py`

**Interfaces:**
- Consumes: `stereo-depth-config` (env var, config path, keys, `metrics_port`); `git show 08bc9b8^:ansible/playbooks/nodes/client/stereo_depth.yml` as reference (per-node playbooks are gone; use a `Deploy stereo_depth` block).
- Produces: the deployed, enabled `stereo_depth` unit.

**Ownership:**
- owns: none
- uses: stereo-depth-config
- dependsOn: T3, T4, T5

**Requirement:**
- Node type `stereo_depth`: native, `python3 -m stereo_depth`, `node_src_dir nodes/stereo_depth`, `src_shared: true`,
  `config_path /etc/ros2/stereo_depth`, env `STEREO_DEPTH_CONFIG=...`, apt `ros-jazzy-message-filters` and
  `ros-jazzy-stereo-msgs` if not already pulled in, memory estimate (MemoryLow 96M, MemoryMax 384M, marked estimate).
- A new background tier: `cpu_weight 100`, `io_weight 100`, `oom_score_adjust 0`, below every robot and agent tier;
  `stereo_depth` is its only member. `tests/test_unit_cgroups.py` gains the tier.
- ros2_nodes entry `stereo_depth` present and enabled, config with every `DepthConfig` default made explicit and
  `metrics_port: 19113`. `monitoring_agent_node_targets` has `127.0.0.1:19113`; `test_node_metrics_wiring.py` lists the
  port and the shared node type.
- `deploy_nodes_client.yml` has the tagged `Deploy stereo_depth` block after `overview_camera`, and the boot include's
  display name says IMX219 pair.
- `tests/test_stereo_depth_config.py` pins node type, entry, block and tags. `ansible/README.md` (tiers table, memory
  table, metrics port table), `SKILL.md` (stereo_depth row) and `tests/README.md` (new file, changed tests, and a
  "Per-node tests (stereo_depth)" section listing every test file under `nodes/stereo_depth/tests/`) are updated.

**Value:** one deploy command installs the depth node at a priority that never starves the robot.

**Steps:**
- [ ] Write the tests. Run. Expected: FAIL (no stereo_depth node type).
- [ ] Implement. Run. Expected: PASS.
- [ ] Lint: `uv run poe lint && uv run poe test-ansible`.
- [ ] Commit: `feat(ansible): deploy stereo_depth in a background cgroup tier with metrics`

**Verify:**
`uv run pytest tests/test_stereo_depth_config.py tests/test_unit_cgroups.py tests/test_node_metrics_wiring.py tests/test_ansible_deploy_speed.py -v && uv run pytest tests -q && uv run poe lint && uv run poe test-ansible`
prints all passed and ansible-lint / syntax-check clean.

## Execution

- Run name: `stereo-depth`.
- Model per task: sonnet unless stated; T1 and T3 on opus (camera_ros SyncMode and the Ansible build; the stereo_depth
  node).
- Order: T1 -> T2 -> T5 -> T6 and T3 -> T4 -> T6. T1/T2/T5/T6 are chained only because they share
  `ansible/group_vars/client.yml` and `tests/README.md`; T3 starts at once.
- Lenses run: research-code-internal (stereo-brief.md and git before 08bc9b8), research-docs (libcamera SyncMode, IMX219
  modes); research-code-external and research-papers skipped (standard OpenCV SGBM).
- Deploy after the run, gated on the user: the boot overlay change reboots the client and only works with both IMX219
  mounted, so ask first. Then `./scripts/deploy-nodes.sh client overview_camera stereo_depth web_ui mcp_server` with
  output in `.logs/` (mcp_server redeploy kills a running agent session: check first).
- After deploy and calibration: `python3 -m stereo_depth.bench` on the client (record the table in the stereo_depth
  README), topic_scraper skew of the two `image_raw` header stamps p99 under 1 ms with Wi-Fi up and down, and the
  achieved rate of `/stereo/depth/image_raw` at or near 5 Hz.
- Lint: `node /Users/szymonri/Documents/claude-infra-plugin/workflows/validate-tasks.mjs docs/plans/2026-10-13-stereo-depth-tasks.json --plan docs/plans/2026-10-13-stereo-depth-plan.md --approvals docs/plans/2026-10-13-stereo-depth-approvals.json --design docs/plans/2026-10-13-stereo-depth-design.md`
- Tasks JSON: `docs/plans/2026-10-13-stereo-depth-tasks.json`.
- Approvals: `docs/plans/2026-10-13-stereo-depth-approvals.json`.
