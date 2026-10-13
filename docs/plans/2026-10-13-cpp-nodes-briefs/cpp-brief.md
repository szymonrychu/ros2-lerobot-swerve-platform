# C++ (rclcpp) rewrites of the heaviest nodes - research brief

Main HEAD at time of research: `20c1d1d fix(tests): wrap over-long line in mcp_server config test`. This was a
read-only survey and no repo code changed. Researched 2026-10-13.

Data sources:
- Mimir at `http://server.ros2.lan:9009/prometheus`.
- cadvisor cgroup series per `ros2-<name>.service`, covering every node, including apt and C++ ones.
- `process_*` series from the Python nodes' own `/metrics`, joined on `robot_node_info{node}`. Only client nodes
  export these.

Goal: rewrite stereo_depth and the other heaviest nodes in C++. Where a cheaper fix gets the same saving, this brief
says so.

## 1. Measurements

**Window.** The query asked for 7 days, but Mimir only holds about 29 h: from 2026-10-12 00:59 UTC to 2026-10-13
05:49 UTC. The averages and p95 values below cover that span, using 5-minute rate samples.
- avg and p95 are almost equal for every node, which means the load is steady and not bursty.
- The last ~1 h is partial. A deploy (commit 515a353, whose colcon build stops all ros2 units) dropped the servo,
  IMU and swerve loop rates during that hour.

**The client is CPU-bound, not memory-bound.**
- CPU: the root cgroup averages **3.53 of 4 cores**, and `node_cpu` non-idle is 99%.
- Memory: of 8.3 GB, at least 5.1 GB is always available.
- Server (Pi 4): 0.79 of 4 cores, and at least 2.5 of 4 GB available.

So the case for rewrites is client CPU. RSS savings (about 60 MB per Python node) are a side benefit.

**stereo_depth has no series yet.** It is enabled in `client.yml`, but it never ran inside the window. The README
estimates 0.4 core at 5 fps. `stereo-brief.md` expected 0.7-0.8 core. The node sits in the background tier, so on
today's saturated client it will be starved.

**Column definitions.**
- CPU is in cores: the avg and p95 of `rate(...[5m])`. cadvisor and `process_cpu_seconds_total` agree to within
  0.001 where both exist.
- RSS is `process_resident_memory_bytes` (avg and max) where the node exports it. Otherwise it is the cgroup working
  set avg and the cgroup RSS max, marked `cg`.
- Peak is `container_memory_max_usage_bytes`, the cgroup peak since the last unit start. It is exported.
- user/sys is the cadvisor user and system CPU split.

| # | Node | Host | Lang | CPU avg / p95 | user / sys | RSS avg / max (MiB) | cg peak | Expected C++ saving (cores) | Rewrite risk |
|---|---|---|---|---|---|---|---|---|---|
| 1 | web_ui | client | Py (uv), FastAPI + rclpy MT executor | 0.557 / 0.600 | 0.51 / 0.04 | 188 / 201 | 141 | Bridge rewrite about 0.3; cheap fix 0.3-0.45 | High: 5.5k lines, 533 tests, WS/React contract |
| 2 | mcp_server | client | Py (uv), FastMCP + rclpy MT(4) | 0.542 / 0.573 | 0.52 / 0.02 | 280 / 293 | 305 | Full rewrite not viable; a C++ state relay saves 0.2-0.35 | Very high: 35k lines, 1859 test functions (more once parametrized) |
| 3 | nav2_bringup | client | C++ (apt + MPPI source overlay) | 0.483 / 0.515 | 0.42 / 0.06 | cg 223 / 299 | 262 | - (already C++) | - |
| 4 | lerobot_leader | server | Py (uv), feetech_servos | 0.381 / 0.385 | 0.31 / 0.08 | cg 63 / 39 | 87 | about 0.25, but the server has 3.2 cores idle | High: servo bus |
| 5 | bno055_imu | client | Py (uv), adafruit on i2c-gpio | 0.377 / 0.395 | 0.11 / **0.27** | 91 / 92 | 69 | at most 0.08: 70% is kernel bit-bang | Medium: critical tier, EKF input |
| 6 | map_guard | client | Py (uv), rclpy MT executor + TF listener | 0.332 / 0.357 | 0.31 / 0.02 | 101 / 110 | 62 | **0.25-0.30** | Medium: can reset the SLAM map; 107 tests |
| 7 | swerve_controller | client | Py (uv), numpy | 0.241 / 0.255 | 0.23 / 0.01 | 94 / 94 | 67 | **about 0.20** | **High**: drives the base, tilt guard, critical tier; 154 tests |
| 8 | gripper_uvc_camera | client | Py (uv), OpenCV V4L2 + JPEG | 0.196 / 0.252 | 0.11 / 0.09 | 107 / 107 | 83 | about 0.07; cheap fix (MJPEG pass-through) about 0.1 | Low |
| 9 | lerobot_follower | client | Py (uv), feetech_servos 95 Hz | 0.194 / 0.200 | 0.17 / 0.03 | 81 / 82 | 54 | about 0.13 | **Highest**: servo safety, torque and startup paths, 234 tests |
| 10 | filter_node | client | Py (uv), 100 Hz spin_once loop | 0.135 / 0.141 | 0.13 / 0.01 | 80 / 80 | 68 | about 0.11 | **High**: last stage before the arm servos (arbitration, Kalman); 50 tests |
| 11 | overview_camera | client | C++ (camera_ros + libcamera source build) | 0.090 / 0.096 | 0.08 / 0.01 | cg 89 / 83 | 129 | - | - |
| 12 | laser_filter | client | Py (uv) | 0.070 / 0.073 | 0.07 / 0.00 | cg 53 / 50 | 74 | about 0.05 | Low |
| 13 | robot_localization_ekf | client | C++ apt | 0.059 / 0.061 | - | cg 45 / 43 | 61 | - | - |
| 14 | gps_rtk_base | server | Py (uv) | 0.041 / 0.044 | - | cg 47 / - | 49 | negligible | - |
| 15 | slam_toolbox | client | C++ apt | 0.026 / 0.028 | - | cg 83 / 90 | 93 | - | - |
| 16 | rplidar_a1 | client | C++ apt driver + Py supervisor | 0.021 / 0.022 | - | 80 / 85 (supervisor) | 89 | - | - |
| 17 | rf2o_laser_odometry | client | C++ (source build) | 0.019 / 0.020 | - | cg 36 / 36 | 38 | - | - |
| 18 | claude_agent | client | Py + CLI child | 0.008 / 0.025 | - | cg 170 / 451 (working set max) | 128 | - | - |
| 19 | poi_store | client | Py (uv) | 0.004 / 0.004 | - | 114 / 115 | 97 | memory only (about 90 MB), and memory is not constrained | Low |
| - | stereo_depth | client | Py (uv), OpenCV SGBM | no data (not running) | - | est. 90-110 (cv2 + numpy) | - | Python glue 0.05-0.15 of the 0.4-0.8 core; SGBM is already C++ | Low: background tier, nothing safety-critical reads it |

Every node below 0.02 core is Python and not worth rewriting: gps_rtk_rover, rf2o_odom_relay, test_joint_api,
leader_bridge, costmap_zones, static_tf_publisher and ros2-master. A C++ rclcpp node typically runs at 15-30 MB RSS,
against 80-115 MB for a Python node with rclpy and numpy.

## 2. Why each top Python node is expensive

The common cost is **rclpy per-message overhead**: deserializing into Python objects plus executor dispatch. It is
worst with `MultiThreadedExecutor`, which web_ui, mcp_server and map_guard use. All three also run a Python
`tf2_ros.TransformListener`, which deserializes every `/tf` message, even though they only need a lookup a few times
per second. They also each subscribe to the high-rate topics:
- `/follower/joint_states` and `/swerve_drive/joint_states` at about 95 Hz
- `/imu/data` at 50 Hz
- `/odom`

The CPU split confirms this: all of them are around 95% user time. The exception is bno055_imu.

**Note on these figures.** Every core figure in the per-node notes below is an estimate from reading the code,
unless it comes straight from a metric named in the note. In particular, the shares of each node's cost
(per-message rclpy overhead, serialization, numpy, JPEG work) and the "a C++ rewrite saves" numbers were not
profiled. Section 4 has an open question about getting a real workload profile before choosing.

- **web_ui (0.56).**
  - Code path: `bridge.py:418-445`. Every message on every subscribed topic is serialized as it arrives
    (`msg_serializer.py`), whether or not a browser is connected:
    - CompressedImage frames (gripper camera at 10 fps, overview camera at 5 fps) are decoded and then re-encoded as
      a full JPEG **plus** a downscaled second JPEG, then base64-encoded (`msg_serializer.py:184-219`).
    - Costmaps and maps are PNG-encoded.
    - Joint states become dicts.
  - What the metrics show: only the broadcast step is cheap (`webui_broadcast_duration_seconds` sums to 0.09 core).
    The remaining 0.4+ core sits in the subscription callbacks and the MT executor.
  - Cheap fix (Python):
    - Store the raw latest message per topic and serialize only in `flush_dirty()`, and only when a client is
      connected.
    - Pass CompressedImage JPEG bytes through unchanged.
    - Expected saving: 0.3-0.45 core with no browser open, about 0.2 with one open.
  - A C++ rewrite of only the ROS bridge would still need a FastAPI/WS layer. It is a large rewrite for a similar
    saving.
- **mcp_server (0.54).**
  - Load: about 15 always-on subscriptions on a 4-thread MT executor, plus a TF listener (`ros_iface.py:318-361`).
    The high-rate ones are joint_states (95 Hz), imu (50 Hz), odom, swerve odom, rf2o twist, cmd_vel, scan, two
    CompressedImage cameras and collision_monitor state.
  - Timers: 0.25 s monitor, 0.2 s orphan check, keepalive, and 0.05 s nav poll.
  - The tools themselves are idle most of the time. The CPU is the subscription firehose.
  - Options short of a rewrite:
    - (a) A small C++ "state relay" component that merges the high-rate topics and the base_link pose from TF into
      one low-rate `/mcp/state` message at 10-20 Hz. mcp_server then subscribes to that instead. Saves 0.2-0.35.
    - (b) Try `rclpy.experimental.EventsExecutor`. Check that Jazzy's rclpy ships it before planning on it.
  - Watch out: an executor swap reopens the 2026-10-08 InvalidHandle race (`spin.py`).
- **bno055_imu (0.38).** Already a single 26-byte burst read per sample (`burst.py`).
  - 0.27 of the 0.38 core is **kernel** time: the i2c-gpio bit-banged bus (`client.yml:201` notes 6.8 ms of CPU per
    sample). C++ cannot remove that.
  - Real fixes: hardware I2C on the Pi 5 (RP1), if the BNO055 clock stretching works there; or a lower `publish_hz`.
    The EKF only fuses gyro yaw rate.
- **map_guard (0.33).**
  - Load: an MT executor (`node.py:310`) dispatches IMU (50 Hz), wheel odom, rf2o odom, cmd_vel and scan callbacks,
    plus the TF listener.
  - The scan is scored only at 2 Hz (`agreement_hz`), but every scan is still deserialized.
  - `prepare_map` re-dilates the grid in numpy on every `/map` update.
  - Logic size: small (1.8k lines) and pure: detectors, guard state machine, agreement, backup naming. Its only side
    effects are slam_toolbox services and an HTTP POST to claude_agent.
  - Verdict: the best rewrite candidate after stereo_depth.
- **swerve_controller (0.24).**
  - Load: a 50 Hz timer, plus `/swerve_drive/joint_states` at about 95 Hz and IMU at 50 Hz.
  - Per-step `numpy` least squares on 8x3 arrays (`kinematics.py`). Small-array numpy overhead dominates this.
  - Each step also publishes odom, a TF broadcast and cmd, plus tilt-guard JSON.
  - The measured loop is 1.8 ms per step (`swerve_loop_duration_seconds`), about 0.09 core. The rest is rclpy.
  - A C++ rewrite (Eigen) gets to about 0.04 core.
- **lerobot_follower (0.19) and lerobot_leader (server, 0.38).**
  - Load: a 95 Hz sync-read/sync-write loop, 6.5 ms per cycle (`servo_cycle_duration_seconds`). Mostly serial wait
    plus Python byte packing and decoding.
  - A C++ rewrite needs a C++ Feetech SCS library and a re-validation of torque, startup and watchdog behaviour.
  - Saving: about 0.13 core on the client. The server has spare CPU.
- **filter_node (0.135).**
  - Load: a 100 Hz `spin_once` loop with a per-joint Kalman and source arbitration (937 lines).
  - Saving: about 0.11 core.
  - It is in the arm command path, so a regression moves the arm.
- **gripper_uvc_camera (0.20).**
  - Load: V4L2 capture (0.09 sys) and a JPEG encode of 4.7 ms per frame at 10 fps.
  - Cheap fix: request MJPEG from the device and publish its JPEG unchanged. Saves about 0.1 and needs no rewrite.
- **stereo_depth (not yet running).**
  - SGBM, `remap` and `cvtColor` are already OpenCV C++ code. The Python share is:
    - rclpy deserialization of two 480x360 bgr8 images (518 KB each) per pair, at the camera rate, before the rate
      limit drops them
    - `np.frombuffer`/reshape
    - the depth conversion
    - `tobytes()` of the 16UC1 output (345 KB)
  - Estimate: 10-25 ms per pair, which is 0.05-0.15 core at 5 fps, plus about 80 MB RSS.
  - As an rclcpp **component** in `overview_container` (camera_ros already runs with `use_intra_process_comms`), the
    raw frames would also skip DDS altogether. web_ui and mcp_server only read the `/compressed` topics.
  - Trade-off: a component in that container runs in overview_camera's normal cgroup tier, not the background tier.

**Comparison.**

| Option | Saving (cores) |
|---|---|
| Recommended rewrites: stereo_depth glue, map_guard, swerve_controller, filter_node | about 0.6-0.7 |
| Cheap fixes: web_ui lazy serialization + JPEG pass-through, mcp_server state relay, uvc MJPEG pass-through | about 0.6-0.9 |

Doing both brings the client from about 3.5 to about 2.0-2.3 cores, which leaves room for stereo_depth.

## 3. Repo conventions and proposal for C++ nodes

**What exists today.**
- No in-repo C++ package. There is no `CMakeLists.txt`, `package.xml`, `.cpp`, gtest, ament_lint, clang-format or
  clang-tidy configuration anywhere.
- All C++ is upstream source plus patches, built by Ansible:
  - rf2o (`nodes/rf2o_laser_odometry/patches/`)
  - libcamera and camera_ros (`nodes/overview_camera/patches/`)
  - the MPPI overlay (`nodes/nav2_bringup/patches/`)
- Build path: `roles/ros2_node_deploy/tasks/main.yml:130` leads to `colcon_source_build.yml` and then
  `colcon_source_package.yml`:
  - It does a git checkout at a pinned commit, then `ansible.builtin.patch`.
  - Then it runs `systemd-run --scope -p CPUQuota=400% -p CPUWeight=20 -p IOWeight=10 nice -n 19 ionice -c 3 colcon
    build --merge-install --parallel-workers 1 --packages-select <pkg> --cmake-args -DCMAKE_BUILD_TYPE=Release`, with
    `MAKEFLAGS=-j<jobs>` (`ros2_build_jobs`, default 4; `group_vars/all.yml:45-50`).
  - Rebuild trigger: a stamp key (sha1 of commit, patches, cmake args, apt versions and the previous chain key), kept
    as `<ws>/.built-<pkg>-<key>`.
  - `stop_for_build.yml` stops **every** running ros2 unit before a build.
- Launch:
  - `templates/ros2-node-launcher.j2` writes `/usr/local/bin/ros2-<name>`. It sources `/opt/ros/jazzy`, then
    `<ws>/install/setup.bash`, then execs `node_launch_command`.
  - `ros2-node-native.service.j2` carries the cgroup tier directives (`Restart=always`, `RestartPreventExitStatus=78`).
  - A C++ node fits the existing `ros2_node_type_defaults` schema: `node_src_dir: ""`, `src_paths`, `colcon_source`,
    `apt_packages`, `node_launch_command: ros2 run ...`. No new deploy mode is needed.
- Python conventions to keep:
  - pydantic `extra="forbid"` config at `/etc/ros2/<name>/config.yaml`, path taken from an env var; exit 78 on a bad
    config.
  - Metrics through `shared/ros2_metrics`: `robot_node_info{node}` and `robot_node_start_time_seconds`, served on
    `127.0.0.1:<191xx>`. The port contract is in `tests/test_node_metrics_wiring.py`.
  - CI (`.github/workflows/ci.yml`) has a per-node uv matrix, which `tests/test_ci_workflow.py` keeps in sync.
  - Precedent for a container test run: `scripts/probe-mppi-overlay-docker.sh`, which runs colcon build and test in
    an arm64 `ros:jazzy-ros-base` container.

**Proposed conventions.**
1. **Layout.** Each C++ node is an ament_cmake package: `nodes/<name>_cpp/` with `package.xml`, `CMakeLists.txt`,
   `include/<name>_cpp/`, `src/`, `test/`, `config/`, `README.md`.
   - Pure logic (state machines, kinematics, filters, scoring) goes in a library target that does not link rclcpp,
     the same split as the Python `bridge_cycle`/`guard` modules.
   - The node is registered as an `rclcpp_components` component **and** has a standalone executable, so it can be
     composed or run alone.
   - A new node type `<name>_cpp` sits beside the Python one, with the same topics, QoS, metrics port and metric
     names. Cut-over is enabling one and disabling the other in `group_vars`. Rollback is the reverse.
2. **Build through Ansible.**
   - Extend `colcon_source_package.yml` with a **local source** mode (`colcon_src.local: nodes/<name>_cpp`):
     - Sync `src_paths` into `/opt/ros2-ws/src/<pkg>` (or a separate `/opt/ros2-ws-local` overlay chained on
       `/opt/ros2-ws`).
     - The stamp key is a hash of the synced tree plus the apt versions.
   - Keep the low-priority scope: `CPUWeight=20`, `nice 19`, `ionice -c 3`, `--parallel-workers 1`,
     `-DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF`, and `MAKEFLAGS=-j2` so the running robot keeps its cores.
   - Do **not** stop all ros2 units for an in-repo package (small build, low OOM risk). Restart only the unit itself.
     This needs a user decision (question 6).
3. **Config.**
   - yaml-cpp (`libyaml-cpp-dev`) loads into a `struct Config`.
   - An explicit `validate(const YAML::Node&) -> std::vector<std::string>` rejects unknown keys, wrong types and range
     violations. It mirrors the pydantic model field by field: `extra="forbid"`, `ge`/`gt`/`le`, cross-field
     validators.
   - Any error is logged and the process exits **78**.
   - It reads the same env var and path (`<NAME>_CONFIG`, default `/etc/ros2/<name>/config.yaml`), so the
     `group_vars` config block is unchanged.
   - Shared helpers live in a new `shared/ros2_common_cpp` ament package: `require<T>`, `range` and `forbid_unknown`.
4. **Metrics.**
   - Use prometheus-cpp (`libprometheus-cpp-dev`, core + pull) if Ubuntu 24.04 arm64 ships it. Verify with
     `apt-cache policy libprometheus-cpp-dev` on the client.
   - Fallback: a ~100-line text-format exposition over `cpp-httplib` (`libcpp-httplib-dev`).
   - Bind 127.0.0.1 on the node's existing 191xx port.
   - Keep the metric names and labels the same as the Python node, including `robot_node_info{node}` and
     `robot_node_start_time_seconds`, so Grafana and alerts carry over. A shared helper in `ros2_common_cpp` provides
     both.
5. **Tests.**
   - Use gtest via `ament_cmake_gtest`, with one test file per Python test file.
   - **Golden fixtures:** for each pure function, a one-off script runs the existing Python implementation on
     recorded or synthetic inputs and writes `test/fixtures/*.yaml`. The gtest asserts the same outputs within a
     tolerance. That carries the Python test intent over without hand-porting every assertion.
   - The Python test suite stays as the spec until cut-over. Its README rows get a C++ counterpart column.
   - Root `tests/` gains the node type in `test_node_metrics_wiring.py`, the cgroup tier tests and a config-schema
     parity test (C++ `validate` against the pydantic model on the `group_vars` block).
6. **Lint.**
   - `ament_lint_auto` with `ament_cmake_clang_format` (root `.clang-format`, ament style, 120 columns to match
     ruff), `ament_cmake_cppcheck` and `ament_cmake_xmllint`. `ament_cmake_copyright` stays off.
   - clang-tidy is optional (`readability-*`, `bugprone-*`) and runs only in CI.
7. **CI and verification.**
   - Generalize `probe-mppi-overlay-docker.sh` into `scripts/cpp-node-test.sh <pkg>`. It runs `ros:jazzy-ros-base`
     arm64: `rosdep install`, then `colcon build --packages-select <pkg> --cmake-args -DBUILD_TESTING=ON`, then
     `colcon test`, then `colcon test-result --verbose`.
   - A new CI job runs the same script for each C++ package. A matrix kept in sync by `tests/test_ci_workflow.py`
     must skip `nodes/*_cpp` directories, which have no pyproject.toml.
   - Add `poe test-cpp` and `poe lint-cpp` tasks.

## 4. Recommended rewrite set and ordering

1. **T0 C++ scaffolding** (prerequisite): `ros2_common_cpp`, the Ansible local-source build, `cpp-node-test.sh`, CI
   job, `.clang-format`, and the root test contracts.
2. **stereo_depth_cpp**: the explicit ask, lowest risk. Start as a standalone background-tier process. Composing it
   into `overview_container` is a follow-up switch (question 3).
3. **map_guard_cpp**: the largest clean saving (0.25-0.3), medium risk, no motion.
4. **Cheap fixes, if approved (question 1)**: web_ui lazy serialization + JPEG pass-through, the mcp_server C++
   state relay, and uvc MJPEG pass-through. Together they save more than all the rewrites combined.
5. **swerve_controller_cpp**: high risk. Gated on question 2; the bench test is wheels lifted, wheel-yaw first.
6. **filter_node_cpp**: high risk. Gated on question 2; gripper-only motion first.

Not recommended:
- lerobot_follower and lerobot_leader: serial-bound, safety-critical, saving about 0.13 core.
- bno055_imu: kernel-bound; question 5 instead.
- A full web_ui or mcp_server rewrite.
- poi_store and laser_filter: too small to matter.

## Plan inputs

Each task's verify command ends in the repo's mandatory Test, Commit, Push, Deploy sequence. Deploy output goes to
`.logs/`.

**Gate for every deploy that touches motion (T4, T5, T6).** Before anything moves, the user confirms the bench setup:
wheels off the floor for the base, and gripper-only, minimal-amplitude motion for the arm. This follows the
CLAUDE.md servo safety rule.

**T0 - C++ node scaffolding**
- Files:
  - `shared/ros2_common_cpp/{package.xml,CMakeLists.txt,include/ros2_common_cpp/{config.hpp,metrics.hpp},src/*.cpp,test/*.cpp}`
  - `.clang-format`
  - `scripts/cpp-node-test.sh`
  - `ansible/roles/ros2_node_deploy/tasks/colcon_source_package.yml` (local mode, no stop-all for local packages)
  - `.github/workflows/ci.yml`, `pyproject.toml` (poe `test-cpp`, `lint-cpp`)
  - `tests/test_ci_workflow.py`, `tests/README.md`, `ansible/README.md`, `shared/README.md`
- Acceptance:
  - `ros2_common_cpp` builds in `ros:jazzy-ros-base` arm64.
  - The config helper exits 78 on an unknown key.
  - The metrics helper serves `robot_node_info{node="x"} 1` on 127.0.0.1.
  - Ansible renders a local-source build with the low-priority scope and a tree-hash stamp.
- Test:
  - gtest for config validation (unknown key, wrong type, range) and the metrics text.
  - A root pytest asserting that the local-source stamp changes when a source file changes and that local builds do
    not include `stop_for_build`.
- Verify: `./scripts/cpp-node-test.sh ros2_common_cpp && uv run pytest tests/ -q && uv run poe test-ansible`

**T1 - stereo_depth_cpp**
- Files:
  - `nodes/stereo_depth_cpp/` containing:
    - `src/depth.cpp`: matcher factory, disparity to depth mm
    - `src/calibration.cpp`: load `stereo.yaml` and the left/right YAMLs, rectification maps, signature re-check
    - `src/stereo_depth_node.cpp`: component plus main, message_filters ApproximateTime, `sync_slop_s`, rate cap
      with `RATE_JITTER_FRACTION`, drop reasons
    - `config/`, `test/`
  - `ansible/group_vars/client.yml`: a `stereo_depth_cpp` node type in the background tier, port 19113,
    `STEREO_DEPTH_CONFIG`
  - `tests/test_stereo_depth_config.py` (the background tier holds only the active stereo node)
  - `nodes/stereo_depth_cpp/README.md`, `nodes/README.md`
  - The Python `calibrate`/`bench` tools stay in `nodes/stereo_depth`.
- Acceptance:
  - On fixtures from `nodes/stereo_depth/tests` (shifted synthetic pair), depth in mm matches the Python
    `disparity_to_depth_mm` within 1 mm on valid pixels.
  - The invalid mask is identical.
  - The same metric names (`stereo_depth_frames_published_total`, `..._dropped_total{reason}`, matcher seconds, pair
    skew, calibration loaded).
  - Nothing is published without a valid calibration (no placeholder data).
  - On the client: depth at 5 Hz or more when the CPU is not contended, and Mimir CPU at or below the Python node's
    minus 0.05 core.
- Test:
  - gtests for config (every `DepthConfig` rejection), depth conversion, calibration load and invalid cases, rate cap,
    skew drop, and size mismatch.
  - A fixture generator script runs the Python functions.
- Verify:
  - `./scripts/cpp-node-test.sh stereo_depth_cpp && uv run pytest tests/test_stereo_depth_config.py -q`
  - then `./scripts/deploy-nodes.sh client stereo_depth_cpp 2>&1 | tee .logs/deploy-client-stereo_depth_cpp.log`
  - then Mimir `rate(stereo_depth_frames_published_total[5m])` and
    `rate(container_cpu_usage_seconds_total{id=~".*stereo_depth.*"}[5m])`.

**T2 - map_guard_cpp**
- Files:
  - `nodes/map_guard_cpp/` containing:
    - `src/agreement.cpp`: dilate, prepare_map, scan endpoints, transform, agreement ratio
    - `src/detectors.cpp`, `src/guard.cpp`: the state machine OK/DISTURBED/SETTLING/EVALUATING/RESET with cooldown
    - `src/timing.cpp`
    - `src/backup.cpp`: names, prune
    - `src/agent.cpp`: `/robot_events` POST via cpp-httplib, sent only while claude_agent is busy
    - `src/map_guard_node.cpp`: single-threaded executor, `tf2_ros::Buffer` lookup at `agreement_hz`, the
      slam_toolbox pause-toggle and serialize/reset service calls
  - `ansible/group_vars/client.yml`, root metrics/cgroup tests, READMEs
- Acceptance:
  - Fixture parity of agreement ratios and guard transitions with `nodes/map_guard/tests`, including the
    pause-toggle-reads-parameter rule and the dry-run path.
  - Same metrics.
  - Mimir CPU at or below 0.08 core under the same topic load.
- Test: gtests mirroring `test_agreement`, `test_detectors`, `test_guard`, `test_timing`, `test_backup`, `test_agent`
  and `test_config`.
- Verify:
  - `./scripts/cpp-node-test.sh map_guard_cpp && uv run pytest tests/ -q`
  - deploy with `dry_run: true` first
  - `./scripts/deploy-nodes.sh client map_guard_cpp 2>&1 | tee .logs/deploy-client-map_guard_cpp.log`
  - then compare `map_guard_agreement_ratio` and `map_guard_state` with the Python history.

**T3 - cheap fixes (only if question 1 = yes)**
- Files:
  - `nodes/web_ui/web_ui/bridge.py` and `msg_serializer.py`: store the raw latest message, serialize in
    `flush_dirty` only when clients are connected, JPEG pass-through for CompressedImage with a downscale only when
    the head tab needs `color_small`.
  - `nodes/bridges/uvc_camera/bridge.py`: MJPEG fourcc pass-through.
  - New `nodes/state_relay_cpp/` and the matching `mcp_server/ros_iface.py` subscription change.
- Acceptance:
  - web_ui CPU at or below 0.15 core with no browser connected, with an unchanged WS envelope contract (existing 533
    tests green).
  - gripper camera CPU at or below 0.1.
  - mcp_server CPU drops by 0.2 or more, with the existing mcp tests green.
- Test:
  - pytest: no serialization without clients, pass-through byte equality, and one serialization per flush for N
    messages.
  - gtest for the relay merge.
- Verify:
  - `cd nodes/web_ui && uv run pytest tests/ -q && uv run poe lint`, the same for uvc_camera and mcp_server
  - `./scripts/deploy-nodes.sh client web_ui gripper_uvc_camera mcp_server 2>&1 | tee .logs/deploy-client-cheap.log`
  - then Mimir CPU by unit.

**T4 - swerve_controller_cpp (gated, question 2)**
- Files: `nodes/swerve_controller_cpp/`:
  - `src/kinematics.cpp`: Eigen inverse kinematics and the least-squares odometry
  - `src/control.cpp`: the control step, steer flip and speed scale
  - `src/tilt_guard.cpp`
  - the node: 50 Hz wall timer, TF broadcaster, `/tilt_guard/state` JSON
  - plus `group_vars` (critical tier, port 19107) and READMEs
- Acceptance:
  - Fixture parity of wheel commands and odometry against `test_kinematics` and `test_control_step` within 1e-9.
  - Identical tilt-guard level transitions and torque-off behaviour.
  - Loop at 50 Hz with p99 under 2 ms.
  - CPU at or below 0.06.
- Test: gtests mirroring `test_kinematics`, `test_control_step`, `test_tilt_guard`, `test_node_tilt_guard` and
  `test_config`.
- Verify:
  - `./scripts/cpp-node-test.sh swerve_controller_cpp`
  - deploy with wheels lifted
  - `./scripts/swerve_diag.sh` plus topic_scraper `/swerve_drive/joint_commands` vs `/cmd_vel` at small wheel-yaw
    amplitude.

**T5 - filter_node_cpp (gated, question 2)**
- Files: `nodes/filter_node_cpp/`:
  - `src/kalman.cpp`
  - `src/arbitration.cpp`: sources, leases, autonomy release, frame_id source tags
  - `src/command.cpp`
  - the node: 100 Hz loop with metrics `filter_*`
  - plus `group_vars` (critical tier, port 19102) and READMEs
- Acceptance:
  - Fixture parity with `test_algorithms`, `test_arbitration` and `test_command`.
  - The output is a byte-equal JointState for the same input sequence.
  - `/follower/joint_commands` at 100 Hz with overruns no higher than Python.
- Test: the gtest equivalents.
- Verify:
  - `./scripts/cpp-node-test.sh filter_node_cpp`
  - deploy, then `./scripts/teleop_diag.sh`, then topic_scraper leader vs follower oscillation rules, gripper only.

**T6 - lerobot_follower / bno055 (not recommended; only if the user insists, question 2 / 5)**
- Files:
  - `nodes/feetech_servos_cpp/`: needs a C++ Feetech SCS library (for example SCServo_Linux) or a protocol port of
    `sync_read`, `sync_write` and `registers`.
  - For bno055, a `/dev/i2c-N` ioctl driver.
- Acceptance:
  - follower at 95 Hz or more, p99 under 12 ms.
  - Identical startup torque, watchdog, battery and register-dump behaviour.
  - All 234 test intents covered.
- Verify:
  - `./scripts/cpp-node-test.sh feetech_servos_cpp`
  - teleop diag with gripper-only motion.

## Questions only the user can answer

1. **Scope.** Are the cheap non-C++ fixes in scope (web_ui lazy serialization and JPEG pass-through, uvc MJPEG
   pass-through, an mcp_server C++ state relay)? Together they save about as much as all the rewrites, at lower risk.
   Or is the scope strictly C++ rewrites?
2. **High-risk rewrites.** Which of these do you want, and is wheels-lifted (base) and gripper-only (arm) bench
   testing acceptable for them?
   - swerve_controller
   - filter_node
   - lerobot_follower/leader
3. **stereo_depth placement.** Should it be a standalone process in the background tier (keeps "the robot wins
   CPU")? Or a component in `overview_container` (zero-copy raw frames, but the normal tier)?
4. **Cut-over.** Run side by side as `<name>_cpp` node types and keep the Python node for rollback (for how long)?
   Or replace in place and delete the Python package?
5. **bno055.** Try hardware I2C on the Pi 5 (a boot config change and reboot, possibly rewiring), or lower
   `publish_hz`? Either one removes the 0.27 core of kernel bit-bang time. A rewrite does not.
6. **Builds on the Pi.** Is a build of an in-repo C++ package allowed to skip `stop_for_build`, so the robot keeps
   running while it compiles at nice 19? Or should C++ be built off-device (CI or Docker on the Mac) and shipped as a
   .deb?
7. **CI cost.** Add an arm64 `ros:jazzy` job (QEMU on GitHub's amd64 runners is slow) or build amd64 in CI?
8. **Measurement.** Mimir holds only about 29 h. Is that baseline enough? Or should we first record a defined
   workload (teleop plus driving plus one browser open) to compare before and after each rewrite?
