# Head RGB-D fusion of camera and ToF implementation plan

## Agreed rung

Rung 1 of plan.md, migrated from the epic format: the steps are the epic's tasks.

## Approach

Migrated from the epic format: one step per task file, in dependency order, each with its status, files, verify, verified head and every body section.

## Chunk 1: migrated tasks (model: opus)

### S1: Dependency merge command named branch 'worktree-head-rgb-d-fusion-of-camera--rgbd-registration-zbuffer' which does not exist; the actual branch is 'head-rgb-d-fusion-of-camera--rgbd-registration-zbuffer'

- Status: done
- Files: `docs/head-rgbd-fusion-methodology.md`
- Verify: none
- Verified head: 700bb86f58a2353b69eb8eb80abf10a36a7e2bd2

#### Requirement

Dependency merge command named branch 'worktree-head-rgb-d-fusion-of-camera--rgbd-registration-zbuffer' which does not exist; the actual branch is 'head-rgb-d-fusion-of-camera--rgbd-registration-zbuffer'

Where: plan-execute workflow dependency merge instructions

Why: Other dependent tasks will hit the same 'not something we can merge' failure

#### Acceptance conditions

- AC1: Dependency merge command named branch 'worktree-head-rgb-d-fusion-of-camera--rgbd-registration-zbuffer' which does not exist; the actual branch is 'head-rgb-d-fusion-of-camera--rgbd-registration-zbuffer' at plan-execute workflow dependency merge instructions is resolved [evidence: judgement]

#### Files

- docs/head-rgbd-fusion-methodology.md


#### Interfaces

Keep the interfaces around plan-execute workflow dependency merge instructions unchanged unless resolving it requires otherwise: Dependency merge command named branch 'worktree-head-rgb-d-fusion-of-camera--rgbd-registration-zbuffer' which does not exist; the actual branch is 'head-rgb-d-fusion-of-camera--rgbd-registration-zbuffer'.

#### Expected outcome

Dependency merge command named branch 'worktree-head-rgb-d-fusion-of-camera--rgbd-registration-zbuffer' which does not exist; the actual branch is 'head-rgb-d-fusion-of-camera--rgbd-registration-zbuffer' at plan-execute workflow dependency merge instructions is resolved, and Other dependent tasks will hit the same 'not something we can merge' failure no longer applies.

#### Steps

1. Write a failing test that shows: Dependency merge command named branch 'worktree-head-rgb-d-fusion-of-camera--rgbd-registration-zbuffer' which does not exist; the actual branch is 'head-rgb-d-fusion-of-camera--rgbd-registration-zbuffer'.
2. Fix it at plan-execute workflow dependency merge instructions.
3. Run the task verify command and confirm it passes.

#### Notes

Discovered in task `rgbd-motion-compensation` during run `head-rgb-d-fusion-of-camera`.


#### Ruling

Folded into rgbd-final-suite-and-runbook (run 3 planText note) or a workflow process note; not a separate task.

- 2026-10-22 closing (user answer A): workflow process note, no code change; verified_head is the ruling commit 700bb86f58a2353b69eb8eb80abf10a36a7e2bd2.

### S2: Design D10 drop-reason label list (no_tf, motion, no_calibration, no_camera_info, size_mismatch) omits pair_dt, which this task's requirement adds as a metric label

- Status: done
- Files: `docs/head-rgbd-fusion-methodology.md`
- Verify: none
- Verified head: c152d490bdb67a0c5265659d53469f49cd09bca7

#### Requirement

Design D10 drop-reason label list (no_tf, motion, no_calibration, no_camera_info, size_mismatch) omits pair_dt, which this task's requirement adds as a metric label

Where: docs/plans/in-progress/head-rgb-d-fusion-of-camera/design.md D10

Why: head-rgbd-node-wiring should register the pair_dt label value for head_rgbd_pairs_dropped_total

#### Acceptance conditions

- AC1: Design D10 drop-reason label list (no_tf, motion, no_calibration, no_camera_info, size_mismatch) omits pair_dt, which this task's requirement adds as a metric label at docs/plans/in-progress/head-rgb-d-fusion-of-camera/design.md D10 is resolved [evidence: judgement]

#### Files

- docs/head-rgbd-fusion-methodology.md


#### Interfaces

Keep the interfaces around docs/plans/in-progress/head-rgb-d-fusion-of-camera/design.md D10 unchanged unless resolving it requires otherwise: Design D10 drop-reason label list (no_tf, motion, no_calibration, no_camera_info, size_mismatch) omits pair_dt, which this task's requirement adds as a metric label.

#### Expected outcome

Design D10 drop-reason label list (no_tf, motion, no_calibration, no_camera_info, size_mismatch) omits pair_dt, which this task's requirement adds as a metric label at docs/plans/in-progress/head-rgb-d-fusion-of-camera/design.md D10 is resolved, and head-rgbd-node-wiring should register the pair_dt label value for head_rgbd_pairs_dropped_total no longer applies.

#### Steps

1. Write a failing test that shows: Design D10 drop-reason label list (no_tf, motion, no_calibration, no_camera_info, size_mismatch) omits pair_dt, which this task's requirement adds as a metric label.
2. Fix it at docs/plans/in-progress/head-rgb-d-fusion-of-camera/design.md D10.
3. Run the task verify command and confirm it passes.

#### Notes

Discovered in task `rgbd-motion-compensation` during run `head-rgb-d-fusion-of-camera`.


#### Ruling

Folded into rgbd-final-suite-and-runbook (run 3 planText note) or a workflow process note; not a separate task.

- 2026-10-22 closing (user answer A): no code change; resolved by c152d490bdb67a0c5265659d53469f49cd09bca7 (pair_dt documented in docs/head-rgbd-fusion-methodology.md and nodes/head_rgbd_cpp/README.md), recorded as verified_head.

### S3: Head camera mapping config

- Status: done
- Files: `nodes/head_camera_cpp/include/head_camera_cpp/config.hpp`, `nodes/head_camera_cpp/src/core/config.cpp`, `nodes/head_camera_cpp/test/core/test_config.cpp`, `nodes/head_camera_cpp/include/head_camera_cpp/mapping_stream.hpp`, `nodes/head_camera_cpp/src/core/mapping_stream.cpp`, `nodes/head_camera_cpp/test/core/test_mapping_stream.cpp`
- Verify: `scripts/cpp-node-test.sh head_camera_cpp`
- Verified head: 5cffec99d8e3260459f6413d8f4a00a860117fc3

#### Requirement

- `Config` gains (D2; design D4 of the RTAB-Map epic): `lens_position` (dioptres, float >= 0, default 1.0; the focus-lock position and the position used when `af_mode: manual`) and a nested `MappingConfig mapping` with `enabled` (default false), `lock_focus` (bool, default true; runtime-settable node parameter `mapping.lock_focus`: when true the node locks focus at `lens_position` while the mapping image topic has subscribers, decided by the user on 2026-10-18; continuous autofocus otherwise), `rate_hz` (default 2.0, 0 < r <= 10), `width` 576 / `height` 324 (even, > 0, <= the stream size, same aspect as the stream within 1 px), `jpeg_quality` 80 (1..100), and topic accessors `mapping_image_topic()` = `<topic_namespace>/mapping/image_raw/compressed`, `mapping_camera_info_topic()` = `<topic_namespace>/mapping/camera_info`. Unknown keys under `mapping` are errors; every error names its key.
- New core unit `mapping_stream.hpp/.cpp` (no ROS, no libcamera): `class MappingDecimator` (`bool due(int64_t stamp_ns)`: true for the first frame and then whenever at least `1e9 / rate_hz` ns have passed since the last accepted frame; stamps going backwards reset it), `cv::Mat resize_for_mapping(const cv::Mat &bgr, int width, int height)` (INTER_AREA), and `CameraIntrinsics scale_intrinsics(const CameraIntrinsics &calibrated, int width, int height)` (K and P scaled by width/calibrated_width and height/calibrated_height, distortion unchanged; throws when the aspect differs by more than 1 px). Reuse the calibration/intrinsics type the front-camera epic introduced for `camera_info_file` (D3 of that epic) rather than a second one.
- Tests first, seen failing: `test/core/test_config.cpp` (defaults; `mapping.lock_focus` and lens_position round-trip, af_mode stays continuous by default; mapping width odd / larger than the stream / wrong aspect, rate 0, rate 11, quality 0 rejected with the key named) and `test/core/test_mapping_stream.cpp` (decimator at 30 fps input and 2 Hz output accepts 2 frames per second +- 1 over 10 s of stamps; reset on a backwards stamp; resize output size; intrinsics scaled 1152x648 -> 576x324 halve fx, fy, cx, cy; aspect mismatch throws).

#### Acceptance conditions

- a1: lens_position, mapping.lock_focus (the focus-lock control, default true, af_mode default continuous) and the mapping section load and validate [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a2: the decimator emits the configured rate from a 30 fps stamp sequence [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a3: intrinsics are scaled to the mapping size and an aspect change is refused [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a4: the package builds and its tests and linters pass in docker [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]

#### Files

- `nodes/head_camera_cpp/include/head_camera_cpp/config.hpp`
- `nodes/head_camera_cpp/src/core/config.cpp`
- `nodes/head_camera_cpp/test/core/test_config.cpp`
- `nodes/head_camera_cpp/include/head_camera_cpp/mapping_stream.hpp`
- `nodes/head_camera_cpp/src/core/mapping_stream.cpp`
- `nodes/head_camera_cpp/test/core/test_mapping_stream.cpp`

#### Interfaces

- `head_camera_cpp::MappingConfig`, `Config::lens_position`, `Config::mapping_image_topic()`, `Config::mapping_camera_info_topic()` (`include/head_camera_cpp/config.hpp`).
- `MappingDecimator`, `resize_for_mapping`, `scale_intrinsics` (`include/head_camera_cpp/mapping_stream.hpp`): the `head-camera-mapping-core` part.

#### Expected outcome

`scripts/cpp-node-test.sh head_camera_cpp` ends `cpp-node-test: head_camera_cpp PASS` with the new config keys and the mapping core tested.

#### Steps

- Extend test_config.cpp and add test_mapping_stream.cpp (core sources and tests are globbed by CMakeLists, D7 of the front-camera epic); run `scripts/cpp-node-test.sh head_camera_cpp`; see `FAIL`.
- Implement the config keys and mapping_stream.cpp; rerun until `PASS`.
- Commit `feat(head_camera_cpp): focus lock and mapping stream core`.

#### Notes

- Builds on the end state of epic front-camera-30-fps-stream-w (head_camera_cpp exists with `af_mode` continuous|auto|manual, `camera_info_file`, the globbed core library).
- A locked lens while mapping is the precondition for valid intrinsics (outside mapping the camera keeps continuous AF) (prior-art fix-as-ac: autofocus changes intrinsics).
- No camera_info without a calibration file stays the rule (front-camera D3): scale_intrinsics only scales a loaded calibration.
- Moved unchanged in substance from epic rtab-map-rgb-d-3d-mapping-on (which now depends on this epic, U3); the RGB-D fusion node is its first consumer and engages the focus lock by subscribing (U4).

#### Ruling

Accepted by the orchestrator: verified and merged into main (root 1507 passed); head-rgbd-package-config verified by hand: scripts/cpp-node-test.sh head_rgbd_cpp PASS after freeing the Docker VM disk (the apt signature failure was a full Docker disk).

### S4: Head RGBD package config

- Status: done
- Files: `nodes/head_rgbd_cpp/CMakeLists.txt`, `nodes/head_rgbd_cpp/package.xml`, `nodes/head_rgbd_cpp/include/head_rgbd_cpp/config.hpp`, `nodes/head_rgbd_cpp/src/core/config.cpp`, `nodes/head_rgbd_cpp/test/core/test_config.cpp`, `nodes/head_rgbd_cpp/config/head_rgbd.yaml`, `nodes/head_rgbd_cpp/test/data/tof_calibration.yaml`, `scripts/cpp-node-test.Dockerfile`
- Verify: `scripts/cpp-node-test.sh head_rgbd_cpp`
- Verified head: c2fc2131ab84ff74d1225af60ff5f90970036a80

#### Requirement

- New ament_cmake package `head_rgbd_cpp` (C++17, the layout of `nodes/head_camera_cpp` / `nodes/map_guard_cpp`): a core static library globbing `src/core/*.cpp` (no ROS node code in it), gtest targets globbing `test/core/test_*.cpp`, ament linters as in the other C++ packages (root `.clang-format`), dependencies in package.xml: rclcpp, sensor_msgs, message_filters, tf2, tf2_ros, tf2_eigen, cv_bridge, image_geometry, camera_calibration_parsers, depth_image_proc, ros2_common_cpp, yaml-cpp, eigen, OpenCV with the `ximgproc` module (`libopencv-contrib-dev`; if rosdep has no key for it, `scripts/cpp-node-test.Dockerfile` installs it explicitly, with a comment), prometheus-cpp.
- `config.hpp` / `src/core/config.cpp`: `struct Config` with nested structs, `Errors validate(const YAML::Node &)`, `Config from_yaml(const YAML::Node &)`, `Config load_config_or_exit()` reading `HEAD_RGBD_CONFIG` (default `/etc/ros2/head_rgbd_cpp/config.yaml`) through `ros2_common_cpp::load_config_or_exit` (exit 78 on error, unknown keys rejected at every level, every error names its key). Keys and defaults (design D1-D10): `tof_depth_topic` `/tof/depth/image_raw`; `rgb_image_topic` `/head_camera/mapping/image_raw/compressed`; `rgb_info_topic` `/head_camera/mapping/camera_info`; `output_namespace` `/head_rgbd`; `tof_calibration_file` (string, default `/etc/ros2/head_rgbd_cpp/tof_calibration.yaml`); `output.divisor` 2 (1 or 2); `sync.max_pair_dt_s` 0.06 (0 < v <= 0.2), `sync.queue_size` 10 (>= 2); `motion.compensate` true, `motion.fixed_frame` `odom`, `motion.max_angular_rate_deg_s` 10.0 (> 0), `motion.max_linear_rate_m_s` 0.3 (> 0), `motion.rate_window_s` 0.05 (> 0), `motion.tf_timeout_s` 0.05 (>= 0); `filter.jump_rel` 0.05 (0 < v < 1), `filter.jump_abs_mm` 30 (>= 0), `filter.min_valid_neighbours` 3 (0..8); `depth.min_m` 0.1, `depth.max_m` 4.0 (min < max); `upsampling.enabled` false, `upsampling.radius` 4 (1..16), `upsampling.eps` 0.001 (> 0), `upsampling.min_support` 0.5 (0 < v <= 1); `cloud.enabled` true, `cloud.rate_hz` 1.0 (0 < v <= 10); `lazy.idle_unsubscribe_s` 5.0 (>= 0); `metrics_port` (int 1..65535 or null; default chosen free per tests/test_node_metrics_wiring.py). Topic accessors derive output topics: `depth/image_raw`, `depth/camera_info`, `rgb/image_raw/compressed`, `rgb/camera_info`, `points`, `depth_upsampled/image_raw`, `depth_upsampled/camera_info`.
- `config/head_rgbd.yaml`: the defaults, every key commented with its meaning and unit.
- `test/data/tof_calibration.yaml`: a camera_calibration YAML fixture for a 240x180 ToF (fx=fy=191, cx=120, cy=90, zero distortion) used by later tests.
- Tests first in `test/core/test_config.cpp`, seen failing: defaults; the shipped yaml loads; every range above rejected with the key named; unknown nested key rejected; divisor 3 rejected; min_m >= max_m rejected; derived topics.

#### Acceptance conditions

- a1: the config loads with the documented defaults and rejects every invalid value naming the key [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]
- a2: the package builds with OpenCV ximgproc and its tests and linters pass in docker [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]
- a3: the shipped config yaml documents every key [evidence: judgement]

#### Files

- `nodes/head_rgbd_cpp/CMakeLists.txt`
- `nodes/head_rgbd_cpp/package.xml`
- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/config.hpp`
- `nodes/head_rgbd_cpp/src/core/config.cpp`
- `nodes/head_rgbd_cpp/test/core/test_config.cpp`
- `nodes/head_rgbd_cpp/config/head_rgbd.yaml`
- `nodes/head_rgbd_cpp/test/data/tof_calibration.yaml`
- `scripts/cpp-node-test.Dockerfile`

#### Interfaces

- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/config.hpp`: `head_rgbd_cpp::Config` and nested `OutputConfig`, `SyncConfig`, `MotionConfig`, `FilterConfig`, `DepthConfig`, `UpsamplingConfig`, `CloudConfig`, `LazyConfig`, `validate`, `from_yaml`, `load_config_or_exit` (part `head-rgbd-config`).

#### Expected outcome

`scripts/cpp-node-test.sh head_rgbd_cpp` ends `cpp-node-test: head_rgbd_cpp PASS`.

#### Steps

- Write CMakeLists/package.xml and test_config.cpp; run `scripts/cpp-node-test.sh head_rgbd_cpp`; see `FAIL`.
- Implement config.cpp and the yaml; rerun until `PASS` (if ximgproc is missing in the image, add `libopencv-contrib-dev` to the Dockerfile and rerun).
- Commit `feat(head_rgbd_cpp): package skeleton and config`.

#### Notes

- shared/ros2_common_cpp provides `load_config_or_exit`, `Metrics` and the process CPU collector (see nodes/map_guard_cpp for usage).
- ROS Jazzy on noble uses system OpenCV 4.6; ximgproc comes from libopencv-contrib-dev of the same version.

#### Ruling

Accepted by the orchestrator: verified and merged into main (root 1507 passed); head-rgbd-package-config verified by hand: scripts/cpp-node-test.sh head_rgbd_cpp PASS after freeing the Docker VM disk (the apt signature failure was a full Docker disk).

### S5: RGBD calibration frames

- Status: done
- Files: `scripts/head_rgbd_calibration/__init__.py`, `scripts/head_rgbd_calibration/config.py`, `scripts/head_rgbd_calibration/frames.py`, `scripts/head_rgbd_calibrate.yaml`, `tests/test_head_rgbd_calib_frames.py`, `pyproject.toml`, `uv.lock`
- Verify: `uv run pytest tests/test_head_rgbd_calib_frames.py -q && uv run poe lint && uv run ruff check scripts/head_rgbd_calibration && uv run ruff format --check scripts/head_rgbd_calibration`
- Verified head: 0225d6fd39cd675dbb342f55fa0071d6f58c423d

#### Requirement

- Root dev dependency `rosbags` (pinned major, e.g. `rosbags>=0.10,<1`) in `pyproject.toml`, `uv.lock` regenerated with `uv lock`.
- `scripts/head_rgbd_calibration/config.py`: pydantic models (unknown keys forbidden, every error names its key) loaded by `load_calibration_config(path: Path) -> CalibrationConfig` from `scripts/head_rgbd_calibrate.yaml` (config file over CLI, CLAUDE.md):
  - `capture`: `host` (default `client.ros2.lan`), `ssh_user` (default from the ansible inventory user, configurable), `remote_dir` (`/tmp/head_rgbd_capture`), `local_dir` (`.logs/head_rgbd`), `duration_s` (120, 10..600), `ros_setup` (`/opt/ros/jazzy/setup.bash`), `topics` (default `/tof/amplitude/image_raw`, `/tof/depth/image_raw`, `/tof/camera_info`, `/head_camera/mapping/image_raw/compressed`, `/head_camera/mapping/camera_info`, `/tf`, `/tf_static`; must include the mapping image topic so the focus lock engages, D3). `validation_topics` (the calibration topics plus `/head_rgbd/depth/image_raw`, `/head_rgbd/depth/camera_info`, `/head_rgbd/rgb/image_raw/compressed`, `/head_rgbd/rgb/camera_info`; used for validation sessions).
  - `board`: `squares_x` 6, `squares_y` 5, `square_mm` 40.0, `marker_mm` 30.0 (< square), `dictionary` `DICT_5X5_100`.
  - `selection`: `still_window_s` 0.5, `still_rgb_mad` 2.0 and `still_tof_mad` 0.01 (mean absolute difference between consecutive frames, RGB in 8-bit grey levels, ToF as a fraction of the amplitude range), `max_pair_dt_s` 0.05, `skip_start_s` 2.0 (focus lock settling), `amplitude_percentiles` [1, 99].
  - `limits`: `rgb_max_rms_px` 0.5, `tof_max_rms_px` 0.3, `stereo_max_rms_px` 1.0, `tof_zero_distortion_rms_px` 0.3, `min_views` 12.
  - `outputs`: `rgb_calibration` (`nodes/head_camera_cpp/config/head_camera_calibration.yaml`), `tof_calibration` (`nodes/head_rgbd_cpp/config/tof_calibration.yaml`), `extrinsics` (`nodes/head_rgbd_cpp/config/head_rgbd_extrinsics.yaml`), `urdf` (`nodes/web_ui/urdf/head.urdf`), `report_dir` (`.logs/head_rgbd`).
  - `validation`: `distances_m` [0.5, 1.0, 2.0, 3.0], `roi_px` 20, `frames_per_distance` 50 (used by rgbd-validation-metrics).
- `scripts/head_rgbd_calibration/frames.py`:
  - `capture_commands(cfg: CaptureConfig, name: str, topics: list[str] | None = None) -> list[list[str]]` (topics default `cfg.topics`): the ssh command that runs `source <ros_setup> && ros2 bag record --max-bag-duration ... -d <duration_s> -o <remote_dir>/<name> <topics>` (exact flags for Jazzy `ros2 bag record`, including `--use-sim-time` absent and storage chosen explicitly) and the `scp -r` back to `<local_dir>/<name>`, then an ssh `rm -rf` of the remote bag; `capture_bag(cfg, name, topics=None, run=subprocess.run) -> Path` runs them in order and raises naming the failed step.
  - `read_frames(bag: Path, topics: TopicNames) -> FrameSet` with the `rosbags` reader and its typestore: decoded RGB frames (`cv2.imdecode` of the CompressedImage), ToF amplitude (uint16) and depth (uint16 mm) arrays, the last camera_infos, each with its header stamp in ns.
  - `normalise_amplitude(amp: np.ndarray, percentiles: tuple[float, float]) -> np.ndarray`: uint8 percentile stretch; an all-zero frame returns None (never a fake image).
  - `still_pairs(frames: FrameSet, sel: SelectionConfig) -> list[FramePair]`: RGB frames after `skip_start_s` whose RGB and ToF image difference stayed below the thresholds for `still_window_s`, each paired with the nearest ToF frame within `max_pair_dt_s`; at most one pair per still window (the middle one).
- Tests first in `tests/test_head_rgbd_calib_frames.py`, seen failing: config defaults load from the shipped yaml, unknown key and a topic list without the mapping image are rejected; `capture_commands` contents (ssh target, duration, topics, scp destination) and `capture_bag` with a fake runner (order, error on a failed step names it); a synthetic bag written with the `rosbags` writer (two still windows separated by motion, ToF at 10 Hz, RGB at 2 Hz) yields exactly two pairs with dt <= 50 ms; frames inside `skip_start_s` are ignored; board lockstep: the shipped yaml `board` (`squares_x`, `squares_y`, `square_mm`, `marker_mm`, `dictionary`) equals the `charuco` spec of `scripts/markers_generate.yaml` loaded with `scripts.markers.config.load_generator_config` (epic marker-and-calibration-board, a blocker of this epic), so the printed board is the one this tool detects; percentile stretch maps the 1st/99th percentiles to 0/255 and returns None for an empty frame.

#### Acceptance conditions

- a1: the calibration config loads from the shipped yaml and rejects unknown keys and a capture without the mapping topic [evidence: test: tests/test_head_rgbd_calib_frames.py]
- a2: capture builds and runs the ssh record, scp and cleanup steps, naming a failed step [evidence: test: tests/test_head_rgbd_calib_frames.py]
- a3: a synthetic bag yields one ToF/RGB pair per still window, never more than 50 ms apart [evidence: test: tests/test_head_rgbd_calib_frames.py]
- a4: 16-bit amplitude is percentile-normalised to 8 bit and an empty frame gives none [evidence: test: tests/test_head_rgbd_calib_frames.py]
- a6: the calibration board equals the generator's ChArUco spec [evidence: test: tests/test_head_rgbd_calib_frames.py]
- a5: rosbags is a pinned root dev dependency [evidence: command: grep -q 'rosbags' pyproject.toml && grep -q 'name = "rosbags"' uv.lock]

#### Files

- `scripts/head_rgbd_calibration/__init__.py`
- `scripts/head_rgbd_calibration/config.py`
- `scripts/head_rgbd_calibration/frames.py`
- `scripts/head_rgbd_calibrate.yaml`
- `tests/test_head_rgbd_calib_frames.py`
- `pyproject.toml`
- `uv.lock`

#### Interfaces

- `scripts/head_rgbd_calibration/config.py:CalibrationConfig`, `load_calibration_config` (part `head-rgbd-calib-config`).
- `scripts/head_rgbd_calibration/frames.py:capture_commands`, `capture_bag`, `read_frames`, `FrameSet`, `FramePair`, `normalise_amplitude`, `still_pairs` (part `head-rgbd-calib-frames`).

#### Expected outcome

`uv run pytest tests/test_head_rgbd_calib_frames.py -q` passes; a recorded bag becomes a list of still RGB/ToF pairs.

#### Steps

- `uv add --group dev 'rosbags>=0.10,<1'`.
- Write the tests (synthetic bag via `rosbags.rosbag2.Writer`); run `uv run pytest tests/test_head_rgbd_calib_frames.py -q`; see them fail.
- Implement config.py, frames.py and the yaml; rerun until green; `uv run poe lint && uv run ruff check scripts/head_rgbd_calibration && uv run ruff format --check scripts/head_rgbd_calibration`.
- Commit `feat(scripts): RGB-D calibration capture and frame selection`.

#### Notes

- Root tests import scripts as `scripts.<module>` (see tests/test_calibrate_retry_threshold.py); `scripts/head_rgbd_calibration/` is a regular package inside that namespace.
- Never SSH by hand: the capture step is the scripted way (pattern: scripts/rtk_calibrate.sh).
- Do not touch the robot in this task; the runbook runs the capture.

#### Ruling

Accepted by the orchestrator: verified and merged into main (root 1507 passed); head-rgbd-package-config verified by hand: scripts/cpp-node-test.sh head_rgbd_cpp PASS after freeing the Docker VM disk (the apt signature failure was a full Docker disk).

### S6: ToF stamp latency fix

- Status: done
- Files: `nodes/bridges/arducam_tof/arducam_tof/bridge.py`, `nodes/bridges/arducam_tof/arducam_tof/config.py`, `nodes/bridges/arducam_tof/tests/test_bridge.py`, `nodes/bridges/arducam_tof/tests/test_config.py`, `nodes/bridges/arducam_tof/README.md`
- Verify: `cd nodes/bridges/arducam_tof && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- Verified head: de67f4810d26ebe0258f4d593afe44a03fd3e723

#### Requirement

- `TofConfig` gains `stamp_latency_ms: float` (default 0.0, 0 <= v <= 200; documented as the measured capture-to-return latency of the SDK frame, i.e. the ToF-RGB offset found by the design's pan-sweep procedure, D5).
- `TofBridge.publish_frame` reads the node clock immediately after `requestFrame` returns a frame (before `frame_arrays`, `depth_to_mm`, `amplitude_to_mono16` and `releaseFrame`) and stamps the shared header with that reading minus `stamp_latency_ms` (as an rclpy `Time` / `Duration`, never a negative stamp: a result below zero is clamped to the reading itself and logged once). Today the stamp is taken after the conversions (`header.stamp = self.node.get_clock().now()` at the end of `publish_frame`), which adds conversion time on top of the SDK latency.
- Dropped frames (no frame, wrong type, malformed) keep publishing nothing; the startup log line includes `stamp_latency_ms`.
- Tests first, seen failing: `tests/test_config.py` (default 0.0, a value round-trips, -1 and 201 rejected naming the key) and `tests/test_bridge.py` (with a fake clock that advances during the conversion step: the published depth, amplitude and camera_info share one header whose stamp equals the clock reading taken right after `requestFrame` minus the configured latency; latency 0 gives that reading exactly; a latency larger than the reading clamps).
- `nodes/bridges/arducam_tof/README.md`: a `## Timestamps` section: what the stamp means (frame return minus `stamp_latency_ms`), why (the RGB camera stamps at start of exposure), and how the value is measured (pan sweep, docs/head-rgbd-fusion-methodology.md) and set (client.yml `arducam_tof` config).

#### Acceptance conditions

- a1: the header stamp is the clock reading right after requestFrame minus stamp_latency_ms, shared by all three messages [evidence: command: cd nodes/bridges/arducam_tof && uv run --frozen pytest tests/test_bridge.py -q]
- a2: stamp_latency_ms validates (0..200, default 0) [evidence: command: cd nodes/bridges/arducam_tof && uv run --frozen pytest tests/test_config.py -q]
- a3: the README documents the stamp meaning and the measurement [evidence: doc-section: nodes/bridges/arducam_tof/README.md#Timestamps]
- a4: arducam_tof tests and lint pass [evidence: command: cd nodes/bridges/arducam_tof && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/bridges/arducam_tof/arducam_tof/bridge.py`
- `nodes/bridges/arducam_tof/arducam_tof/config.py`
- `nodes/bridges/arducam_tof/tests/test_bridge.py`
- `nodes/bridges/arducam_tof/tests/test_config.py`
- `nodes/bridges/arducam_tof/README.md`

#### Interfaces

- `nodes/bridges/arducam_tof/arducam_tof/config.py:TofConfig.stamp_latency_ms`; `nodes/bridges/arducam_tof/arducam_tof/bridge.py:TofBridge.publish_frame` (stamp semantics consumed by head_rgbd_cpp pairing).

#### Expected outcome

ToF headers are stamped at frame return minus a configurable latency; the deployed value is set by the runbook.

#### Steps

- Add the tests; run `cd nodes/bridges/arducam_tof && uv run --frozen pytest tests -q && uv run --frozen poe lint`; see them fail on the missing field and the late stamp.
- Implement the config field and move the clock read; rerun until green.
- Update the README; commit `fix(arducam_tof): stamp ToF frames at frame return minus measured latency`.

#### Notes

- Do not touch topics, encodings or the confidence handling: only the stamp changes (epic non-goal: no driver changes beyond synchronisation).
- The SDK frame timestamp is not used: its clock domain is undocumented (design D5).
- numpy guard already satisfied (numpy is a dependency).

#### Ruling

Accepted by the orchestrator: verified and merged into main (root 1507 passed); head-rgbd-package-config verified by hand: scripts/cpp-node-test.sh head_rgbd_cpp PASS after freeing the Docker VM disk (the apt signature failure was a full Docker disk).

### S7: Head camera mapping publish

- Status: done
- Files: `nodes/head_camera_cpp/include/head_camera_cpp/head_camera_node.hpp`, `nodes/head_camera_cpp/src/head_camera_node.cpp`, `nodes/head_camera_cpp/test/test_node.cpp`, `nodes/head_camera_cpp/src/core/zoom_controller.cpp`, `nodes/head_camera_cpp/test/core/test_zoom_controller.cpp`, `nodes/head_camera_cpp/src/libcamera_controls.cpp`, `nodes/head_camera_cpp/test/test_libcamera_controls.cpp`, `nodes/head_camera_cpp/README.md`
- Verify: `scripts/cpp-node-test.sh head_camera_cpp`
- Verified head: 39c9bcd379da707c0c2a23c6a0d6bea85ffd982a

#### Requirement

- Node wiring (D2; design D4 of the RTAB-Map epic): when `mapping.enabled`, the node creates `mapping_image_topic()` (sensor_msgs/CompressedImage, JPEG at `mapping.jpeg_quality`) and `mapping_camera_info_topic()` (sensor_msgs/CameraInfo) publishers, best_effort depth 1 (head_rgbd_cpp subscribes best_effort). For each captured stream frame, if `MappingDecimator::due(stamp)` and the image publisher has subscribers, it resizes with `resize_for_mapping`, encodes, and publishes the image and, only when a calibration file is loaded, the camera_info from `scale_intrinsics` with the same header (stamp = capture stamp, frame_id = `frame_id`). Without a calibration the image is still published and camera_info is not (no placeholder intrinsics; head_rgbd_cpp then publishes nothing, which the runbook checks). No mapping publish while a zoom holds the camera in still mode.
- Focus lock (decided by the user on 2026-10-18): the camera keeps the configured `af_mode` (default continuous) normally. While `mapping.enabled` and `mapping.lock_focus` (runtime-settable node parameter) are true and the topic `/head_camera/mapping/image_raw/compressed` has at least one subscriber, the node applies `AfMode` Manual and `LensPosition` = `lens_position` (libcamera_controls); when the last subscriber leaves (or `mapping.lock_focus` is set false) it restores the configured af_mode. A zoom still autofocuses on its point for its single still; the zoom controller's RESTORE phase then returns to the mapping lock (Manual at `lens_position`) when the lock is active, else to the configured AF mode, so a zoom never leaves the lens elsewhere. The intrinsics used for mapping are those calibrated at the locked focus.
- The mapping encode runs only on due frames with subscribers: at 30 fps input and 2 Hz output at most 2 encodes per second (asserted with the fake backend).
- Tests first, seen failing: `test/test_node.cpp` with the fake backend (publishers exist only when enabled; 2 Hz output from 30 fps input with a subscriber, none without; camera_info only with the test calibration `test/data/calibration.yaml`, scaled K; same stamp on image and info; nothing published during a zoom), `test/core/test_zoom_controller.cpp` (restore after ok, after af_failed and after timeout returns to the mapping lock at the configured lens position when the lock is active; continuous stays continuous when it is not), and `test_node.cpp` also covers the lock engaging on the first mapping subscriber and releasing on the last, and `mapping.lock_focus=false` disabling it, `test/test_libcamera_controls.cpp` (manual control list carries AfMode Manual and LensPosition).
- README: `## Mapping stream` section (topics, rate, size, the calibration requirement, the focus-lock rule: continuous AF normally, locked at `lens_position` only while the mapping topic has subscribers, `mapping.lock_focus` to disable it) and the configuration table rows for `lens_position` and `mapping.*`.

#### Acceptance conditions

- a1: the mapping stream is published at the configured rate only with a subscriber and only when enabled [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a2: mapping camera_info is published only from a loaded calibration, scaled, with the image's stamp [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a3: every zoom outcome returns to the mapping focus lock at the configured lens position while mapping is active, and to the configured af_mode otherwise [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a4: the lock controls carry AfMode Manual and LensPosition [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a5: the README documents the mapping stream [evidence: doc-section: nodes/head_camera_cpp/README.md#Mapping stream]
- a6: focus is locked at lens_position only while the mapping image topic has subscribers and mapping.lock_focus is true, continuous autofocus otherwise, with a zoom autofocusing then returning to the lock [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a7: the package builds and its tests and linters pass in docker [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]

#### Files

- `nodes/head_camera_cpp/include/head_camera_cpp/head_camera_node.hpp`
- `nodes/head_camera_cpp/src/head_camera_node.cpp`
- `nodes/head_camera_cpp/test/test_node.cpp`
- `nodes/head_camera_cpp/src/core/zoom_controller.cpp`
- `nodes/head_camera_cpp/test/core/test_zoom_controller.cpp`
- `nodes/head_camera_cpp/src/libcamera_controls.cpp`
- `nodes/head_camera_cpp/test/test_libcamera_controls.cpp`
- `nodes/head_camera_cpp/README.md`

#### Interfaces

- Topics `/head_camera/mapping/image_raw/compressed` (CompressedImage, JPEG) and `/head_camera/mapping/camera_info` (CameraInfo, frame `head_camera_optical_frame`): the `head-camera-mapping-topics` part, consumed by head_rgbd_cpp (head-rgbd-node-wiring), which passes the pair on to RTAB-Map.

#### Expected outcome

With `mapping.enabled: true` and a calibration file, a subscriber receives 576x324 JPEG + matching CameraInfo at 2 Hz; focus is locked while subscribed (continuous otherwise) and returns to the lock after zooms.

#### Steps

- Write the node, zoom controller and controls tests; run `scripts/cpp-node-test.sh head_camera_cpp`; see `FAIL`.
- Implement the wiring, restore change and controls; rerun until `cpp-node-test: head_camera_cpp PASS`.
- Update the README; commit `feat(head_camera_cpp): 2 Hz mapping stream and focus lock`.

#### Notes

- File names are the front-camera epic's end state (`src/head_camera_node.cpp`, `src/core/zoom_controller.cpp`, `src/libcamera_controls.cpp`).
- The Ansible config (`mapping.enabled`, `mapping.lock_focus`, `lens_position`, `camera_info_file`; `af_mode` stays continuous) is set by head-rgbd-node-type and rgbd-final-suite-and-runbook, not here.
- CPU: the mapping encode at 576x324 / 2 Hz is a few ms per second; the runbook measures it.
- Moved unchanged in substance from epic rtab-map-rgb-d-3d-mapping-on (which now depends on this epic, U3); the RGB-D fusion node is its first consumer and engages the focus lock by subscribing (U4).

#### Ruling

Accepted by the orchestrator: verified and merged into main (root 1507 passed); head-rgbd-package-config verified by hand: scripts/cpp-node-test.sh head_rgbd_cpp PASS after freeing the Docker VM disk (the apt signature failure was a full Docker disk).

### S8: RGBD cloud builder

- Status: done
- Files: `nodes/head_rgbd_cpp/include/head_rgbd_cpp/cloud.hpp`, `nodes/head_rgbd_cpp/src/core/cloud.cpp`, `nodes/head_rgbd_cpp/test/core/test_cloud.cpp`
- Verify: `scripts/cpp-node-test.sh head_rgbd_cpp`
- Verified head: 2ed2f93dbdf78cb383dfe4aa5f44c4e082428495

#### Requirement

- `cloud.hpp/.cpp` (D9):
  - `class RateDecimator` (`bool due(int64_t stamp_ns)`: first stamp, then whenever at least `1e9 / rate_hz` ns passed since the last accepted; a backwards stamp resets).
  - `cv::Mat decode_rgb_for_depth(const std::vector<uint8_t> &jpeg, int width, int height)`: `cv::imdecode` with `IMREAD_REDUCED_COLOR_2` when the target is half the JPEG size, else full decode plus `INTER_AREA` resize; empty Mat on a corrupt JPEG.
  - `sensor_msgs::msg::PointCloud2::SharedPtr build_cloud(const sensor_msgs::msg::Image &depth, const sensor_msgs::msg::CameraInfo &depth_info, const cv::Mat &rgb)`: XYZRGB cloud in the depth frame with depth_image_proc's `convertDepth<uint16_t>` and `convertRgb` (`depth_image_proc/conversions.hpp`) and `image_geometry::PinholeCameraModel`; unorganised points with depth 0 dropped (`is_dense` true); returns nullptr when rgb is empty or sizes differ (no cloud with made-up colour).
- Tests first in `test/core/test_cloud.cpp`, seen failing: a 288x162 depth of a plane at 1 m with a known camera_info produces points with z = 1.0 m and x/y matching the pinhole back-projection within 1 mm, colours taken from the matching RGB pixel; zero-depth pixels produce no points; a corrupt JPEG gives an empty Mat and no cloud; the reduced decode of a 576x324 JPEG returns 288x162; the decimator at 1 Hz accepts 1 of every 2 pairs at 2 Hz over 10 s (+-1).

#### Acceptance conditions

- a1: the cloud reproduces pinhole geometry and RGB colours from a synthetic pair [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]
- a2: no points for invalid depth and no cloud without a real RGB image [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]
- a3: the cloud rate decimator emits the configured rate [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]

#### Files

- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/cloud.hpp`
- `nodes/head_rgbd_cpp/src/core/cloud.cpp`
- `nodes/head_rgbd_cpp/test/core/test_cloud.cpp`

#### Interfaces

- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/cloud.hpp`: `RateDecimator`, `decode_rgb_for_depth`, `build_cloud` (part `head-rgbd-cloud`).

#### Expected outcome

Tested coloured-cloud construction reusing depth_image_proc's conversions.

#### Steps

- Write test_cloud.cpp; run `scripts/cpp-node-test.sh head_rgbd_cpp`; see `FAIL`.
- Implement cloud.cpp; rerun until `PASS`.
- Commit `feat(head_rgbd_cpp): decimated coloured cloud`.

#### Notes

- If `depth_image_proc/conversions.hpp` is not installed or exported by ros-jazzy-depth-image-proc, write the back-projection with image_geometry and say so in the commit message (design D9).

- 2026-10-18 run head-rgb-d-fusion-of-camera: dependency-failed on head-rgbd-package-config

#### Ruling

Accepted by the orchestrator: verified and merged into main (root 1571 passed after integration fixes; head_rgbd_cpp C++ PASS); rgbd-methodology-doc parking flags were false positives describing runtime measurement by the robot runbook.

### S9: RGBD intrinsics calibration

- Status: done
- Files: `scripts/head_rgbd_calibration/charuco.py`, `scripts/head_rgbd_calibration/intrinsics.py`, `tests/rgbd_synthetic.py`, `tests/test_head_rgbd_calib_intrinsics.py`
- Verify: `uv run pytest tests/test_head_rgbd_calib_intrinsics.py -q && uv run poe lint && uv run ruff check scripts/head_rgbd_calibration && uv run ruff format --check scripts/head_rgbd_calibration`
- Verified head: f851d784c1f6a4c0976fbe0f27ce7f8d4053700f

#### Requirement

- `scripts/head_rgbd_calibration/charuco.py`: `make_board(cfg: BoardConfig) -> cv2.aruco.CharucoBoard` and `detect(image: np.ndarray, board) -> Detection | None` (`cv2.aruco.CharucoDetector`, corner ids and sub-pixel image points; None when fewer than 6 corners), used for both the RGB (grey) and the normalised ToF amplitude.
- `scripts/head_rgbd_calibration/intrinsics.py`: `calibrate(detections: list[Detection], board, image_size) -> IntrinsicsResult` (`cv2.calibrateCamera`, plumb_bob; K, D, RMS, per-view errors, view count) refusing fewer than `limits.min_views` views (raises naming the count); `calibrate_tof(detections, board, image_size, limits) -> IntrinsicsResult` that runs with distortion fixed at zero (`CALIB_ZERO_TANGENT_DIST | CALIB_FIX_K1 | CALIB_FIX_K2 | CALIB_FIX_K3`) and free, and returns the zero-D result when its RMS <= `limits.tof_zero_distortion_rms_px`, else the free one, with `zero_distortion_rms` recorded either way (fix-as-ac: zero-distortion assumption); `compare_sdk(result, sdk_camera_info) -> dict` (fx/fy/cx/cy differences in % against the SDK `/tof/camera_info`); `write_camera_calibration_yaml(path, result, camera_name, image_size)` writing the ROS camera_calibration format (`image_width`, `image_height`, `camera_name`, `camera_matrix`, `distortion_model`, `distortion_coefficients`, `rectification_matrix` identity, `projection_matrix`) that camera_calibration_parsers reads; writing refuses (raises, writes nothing) when RMS exceeds the camera's limit (`rgb_max_rms_px` 0.5 / `tof_max_rms_px` 0.3).
- `tests/rgbd_synthetic.py` (shared synthetic scenes for every calibration and validation test): render the configured ChArUco board through a known pinhole (K, optional plumb_bob D) at a given board pose with `cv2.aruco.CharucoBoard.generateImage` + homography warp, for an RGB camera (576x324, fx=fy=432, cx=288, cy=162) and a ToF camera (240x180, fx=fy=191, cx=120, cy=90) related by a known extrinsic (default ToF 0.05 m to the left of RGB, 1 deg yaw); a pose generator spanning 0.5-1.5 m and +-40 deg tilt; ToF amplitude rendered as uint16 with noise and depth as uint16 mm of the board plane.
- Tests first in `tests/test_head_rgbd_calib_intrinsics.py`, seen failing: 20 synthetic RGB views recover fx, fy within 1 % and cx, cy within 3 px; 20 synthetic ToF amplitude views at 240x180 (after `normalise_amplitude`) recover fx, fy within 2 % and cx, cy within 2 px; with zero true distortion the ToF result is the zero-D model; with true k1 = -0.1 the free model is chosen and k1 is recovered within 0.03; partial board views (half the board outside the image) still detect >= 6 corners; fewer than min_views raises; the YAML round-trips through `yaml.safe_load` with the right keys and shapes; RMS over the limit writes nothing; the SDK comparison reports the % difference.

#### Acceptance conditions

- a1: synthetic RGB and ToF views recover the known intrinsics within tolerance [evidence: test: tests/test_head_rgbd_calib_intrinsics.py]
- a2: the ToF zero-distortion model is used only when its RMS is within 0.3 px, else the measured distortion [evidence: test: tests/test_head_rgbd_calib_intrinsics.py]
- a3: too few views or RMS over the limit write nothing [evidence: test: tests/test_head_rgbd_calib_intrinsics.py]
- a4: calibrations are written as ROS camera_calibration YAML [evidence: test: tests/test_head_rgbd_calib_intrinsics.py]

#### Files

- `scripts/head_rgbd_calibration/charuco.py`
- `scripts/head_rgbd_calibration/intrinsics.py`
- `tests/rgbd_synthetic.py`
- `tests/test_head_rgbd_calib_intrinsics.py`

#### Interfaces

- `scripts/head_rgbd_calibration/charuco.py:make_board`, `detect`, `Detection` (part `head-rgbd-charuco`).
- `scripts/head_rgbd_calibration/intrinsics.py:calibrate`, `calibrate_tof`, `compare_sdk`, `write_camera_calibration_yaml`, `IntrinsicsResult`.
- `tests/rgbd_synthetic.py` (part `rgbd-synthetic-scenes`): `render_rgb_view`, `render_tof_view`, `board_poses`, `RGB_K`, `TOF_K`, `TRUE_EXTRINSIC`.

#### Expected outcome

Synthetic ChArUco sets recover both cameras' intrinsics and write YAML files head_camera_cpp and head_rgbd_cpp load.

#### Steps

- Write tests/rgbd_synthetic.py and the tests; run `uv run pytest tests/test_head_rgbd_calib_intrinsics.py -q`; see them fail.
- Implement charuco.py and intrinsics.py; rerun until green; `uv run poe lint && uv run ruff check scripts/head_rgbd_calibration && uv run ruff format --check scripts/head_rgbd_calibration`.
- Commit `feat(scripts): ChArUco intrinsics for head RGB and ToF`.

#### Notes

- Folds in the RTAB-Map epic's planned RGB-only `camera-calibration-script` (design D3): one calibration path, ChArUco instead of a checkerboard because the 240x180 amplitude image often shows partial boards.
- opencv-python-headless >= 4.9 ships `cv2.aruco.CharucoDetector` (objdetect); no contrib wheel needed.
- RGB is calibrated at the 576x324 mapping size; head_camera_cpp scales a calibration to each published size (front-camera D3).

#### Ruling

Accepted by the orchestrator: verified and merged into main (root 1507 passed); head-rgbd-package-config verified by hand: scripts/cpp-node-test.sh head_rgbd_cpp PASS after freeing the Docker VM disk (the apt signature failure was a full Docker disk).

### S10: RGBD registration zbuffer

- Status: done
- Files: `nodes/head_rgbd_cpp/include/head_rgbd_cpp/registration.hpp`, `nodes/head_rgbd_cpp/src/core/registration.cpp`, `nodes/head_rgbd_cpp/test/core/test_registration.cpp`, `nodes/head_rgbd_cpp/test/core/synthetic_scene.hpp`
- Verify: `scripts/cpp-node-test.sh head_rgbd_cpp`
- Verified head: 5f589f2d5d4cd25bd6a68204938c8a616797bcfa

#### Requirement

- `registration.hpp/.cpp` (D7), no ROS types:
  - `struct PinholeModel { cv::Matx33d K; cv::Vec<double, 5> D; int width; int height; }` and `PinholeModel scale_model(const PinholeModel &m, int width, int height)` (K scaled, D unchanged; throws on an aspect change > 1 px).
  - `class RayTable` built once from the ToF model (`cv::undistortPoints` of every pixel centre and of the pixel corners) giving the normalised ray per pixel and per corner.
  - `cv::Mat register_depth(const cv::Mat &tof_depth_mm, const RayTable &rays, const Eigen::Isometry3d &T_rgb_tof, const PinholeModel &rgb_out, const DepthConfig &range, RegistrationStats *stats = nullptr)`: for each valid ToF pixel with z in range: 3D point = ray * z; transform with `T_rgb_tof`; skip points with z <= 0; project the pixel's four corner points (same depth) into `rgb_out` with plumb_bob distortion (`cv::projectPoints` or the equivalent closed form, tested equal); fill the clipped bounding rectangle of the projected corners with the transformed point's z in mm, keeping the minimum per output pixel (z-buffer); output CV_16UC1 of `rgb_out` size, 0 where no point landed.
  - `cv::Rect clip_footprint(double u_min, double v_min, double u_max, double v_max, int width, int height)`: the integer pixel rectangle covered, clipped to the image; empty when fully outside (fix-as-ac: depth_image_proc's `fill_upsampling_holes` bounds bug).
- `test/core/synthetic_scene.hpp` (shared by motion and upsampling tests): planes and boxes rendered into a ToF depth image through a `PinholeModel` at a given ToF pose, ground-truth depth rendered into an RGB model at a given RGB pose, plus the RGB=ToF identity helpers.
- Tests first in `test/core/test_registration.cpp`, seen failing: identical models and identity transform reproduce the input exactly; a fronto-parallel plane at 1 m with the RGB 0.05 m right of the ToF lands shifted by fx_out * 0.05 / 1.0 px (+-0.5 px) and its depth is 1000 mm; a slanted plane matches the rendered RGB ground truth within 5 mm on >= 95 % of covered pixels; occlusion: a box at 0.8 m in front of a wall at 2.5 m, seen from the offset RGB: every output pixel covered by the box holds box depth (nearest wins) and the occlusion shadow (wall visible to RGB, hidden from the ToF) stays 0, never wall-interpolated; a footprint crossing each of the four borders (including u_max < 0 and u_min >= width) returns the clipped or empty rectangle, and a plane partly outside the RGB view produces correct border columns without any write outside the Mat (AddressSanitizer is not available, so the test also checks a guard band around a sub-Mat view stays untouched); the default 288x162 output from the default 240x180 ToF on a 1.5 m wall covers >= 90 % of the overlap area (splat closes the 1:1.15 gaps); distortion: a non-zero RGB k1 moves projected points to the distorted positions given by `cv::projectPoints` within 0.05 px.

#### Acceptance conditions

- a1: a known baseline produces the expected disparity and depth on a synthetic plane [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]
- a2: occlusion keeps the nearest depth and leaves occlusion shadows empty [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]
- a3: footprints crossing any image border are clipped with no out-of-image write [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]
- a4: the default 288x162 output fills the ToF overlap of a flat wall [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]
- a5: RGB distortion is applied consistently with OpenCV [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]

#### Files

- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/registration.hpp`
- `nodes/head_rgbd_cpp/src/core/registration.cpp`
- `nodes/head_rgbd_cpp/test/core/test_registration.cpp`
- `nodes/head_rgbd_cpp/test/core/synthetic_scene.hpp`

#### Interfaces

- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/registration.hpp`: `PinholeModel`, `scale_model`, `RayTable`, `register_depth`, `clip_footprint`, `RegistrationStats` (part `head-rgbd-registration`).
- `nodes/head_rgbd_cpp/test/core/synthetic_scene.hpp` (part `head-rgbd-synthetic-scene`).

#### Expected outcome

Tested z-buffer registration from the ToF into any scaled RGB model, ready for the motion-compensated transform.

#### Steps

- Write synthetic_scene.hpp and test_registration.cpp; run `scripts/cpp-node-test.sh head_rgbd_cpp`; see `FAIL`.
- Implement registration.cpp; rerun until `PASS`.
- Commit `feat(head_rgbd_cpp): z-buffer ToF-to-RGB registration`.

#### Notes

- Opus: projection direction, distortion order (undistort ToF rays, distort RGB projection) and the z-buffer are where correctness lives.
- Do not use depth_image_proc::RegisterNode (design D7); its footprint idea is kept with correct clipping.
- Output depth is z in the RGB optical frame (not ray length), in mm, rounded.

- 2026-10-18 run head-rgb-d-fusion-of-camera: dependency-failed on head-rgbd-package-config

#### Ruling

Accepted by the orchestrator: verified and merged into main (root 1571 passed after integration fixes; head_rgbd_cpp C++ PASS); rgbd-methodology-doc parking flags were false positives describing runtime measurement by the robot runbook.

### S11: ToF jump edge filter

- Status: done
- Files: `nodes/head_rgbd_cpp/include/head_rgbd_cpp/tof_filter.hpp`, `nodes/head_rgbd_cpp/src/core/tof_filter.cpp`, `nodes/head_rgbd_cpp/test/core/test_tof_filter.cpp`
- Verify: `scripts/cpp-node-test.sh head_rgbd_cpp`
- Verified head: 2fe36022b91592564edb8e2e3816ae64f8940ed3

#### Requirement

- `tof_filter.hpp/.cpp` (D7, fix-as-ac: flying pixels): `cv::Mat filter_jump_edges(const cv::Mat &depth_mm, const FilterConfig &cfg, FilterStats *stats = nullptr)` on CV_16UC1 (0 = invalid): a valid pixel becomes 0 when any valid 8-neighbour differs by more than `max(cfg.jump_abs_mm, cfg.jump_rel * z)` or it has fewer than `cfg.min_valid_neighbours` valid neighbours; image borders count missing neighbours as invalid; the input is not modified; `FilterStats` counts removed jump and isolated pixels.
- Tests first in `test/core/test_tof_filter.cpp` on synthetic scenes, seen failing: a 1.0 m foreground / 3.0 m background step with one column of mixed pixels at 2.0 m: the mixed column is removed and no pixel deeper than 1 px from the step on either side is touched; flat planes at 0.5, 2 and 4 m and planes slanted 30, 60 and 70 deg (depth computed through a 240x180 pinhole with fx=191) keep >= 99 % of their pixels; an isolated valid pixel in an invalid region is removed; an all-invalid image stays all-invalid; noise of 1 % of depth on a plane removes < 1 % of pixels; the input Mat is unchanged.

#### Acceptance conditions

- a1: mixed pixels on a synthetic depth step are removed while the surfaces on both sides stay [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]
- a2: flat and slanted planes up to 70 deg survive the filter [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]
- a3: isolated speckle is removed and invalid pixels stay invalid [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]

#### Files

- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/tof_filter.hpp`
- `nodes/head_rgbd_cpp/src/core/tof_filter.cpp`
- `nodes/head_rgbd_cpp/test/core/test_tof_filter.cpp`

#### Interfaces

- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/tof_filter.hpp`: `filter_jump_edges`, `FilterStats` (part `head-rgbd-tof-filter`).

#### Expected outcome

A tested flying-pixel filter for the raw ToF depth, called by the node before registration.

#### Steps

- Write test_tof_filter.cpp; run `scripts/cpp-node-test.sh head_rgbd_cpp`; see `FAIL`.
- Implement tof_filter.cpp; rerun until `PASS`.
- Commit `feat(head_rgbd_cpp): jump-edge flying-pixel filter`.

#### Notes

- The bridge's confidence threshold does not remove flying pixels (SCCG 2008, doi:10.1145/1921264.1921293); this filter runs on every frame before reprojection.

- 2026-10-18 run head-rgb-d-fusion-of-camera: dependency-failed on head-rgbd-package-config

#### Ruling

Accepted by the orchestrator: verified and merged into main (root 1571 passed after integration fixes; head_rgbd_cpp C++ PASS); rgbd-methodology-doc parking flags were false positives describing runtime measurement by the robot runbook.

### S12: RGBD extrinsic calibration

- Status: done
- Files: `scripts/head_rgbd_calibration/extrinsics.py`, `tests/test_head_rgbd_calib_extrinsics.py`
- Verify: `uv run pytest tests/test_head_rgbd_calib_extrinsics.py -q && uv run poe lint && uv run ruff check scripts/head_rgbd_calibration && uv run ruff format --check scripts/head_rgbd_calibration`
- Verified head: 3d569641ba77c81a636eaa988318db3b43ef3534

#### Requirement

- `scripts/head_rgbd_calibration/extrinsics.py`:
  - `stereo_calibrate(pairs: list[tuple[Detection, Detection]], board, rgb: IntrinsicsResult, tof: IntrinsicsResult) -> ExtrinsicResult`: object/image points restricted to corner ids seen in both images of a pair, `cv2.stereoCalibrate(..., flags=cv2.CALIB_FIX_INTRINSIC)`; returns R, t of `tof_optical_frame` expressed in `head_camera_optical_frame` (the transform that maps ToF points into the RGB frame), stereo RMS, pair count; refuses fewer than `limits.min_views` pairs.
  - `plane_depth_bias(pairs_with_depth, board, rgb, extrinsic) -> PlaneCheck`: per pair, the board plane from RGB `cv2.solvePnP`, transformed into the ToF frame and intersected with the ToF rays inside the board polygon, against the measured ToF depth there; median and per-pair bias in mm (methodology: plane depth bias <= 15 mm).
  - `write_extrinsics_yaml(path, result, plane_check, meta)` and `load_extrinsics_yaml(path) -> ExtrinsicResult`: keys `parent_frame: head_camera_optical_frame`, `child_frame: tof_optical_frame`, `translation_m: [x, y, z]`, `rotation_xyzw: [...]`, `stereo_rms_px`, `pairs`, `plane_bias_mm_median`, `lens_position`, `board`, `date`, `rgb_rms_px`, `tof_rms_px`, `tof_sdk_diff_pct`; writing refuses (raises, writes nothing) when stereo RMS > `limits.stereo_max_rms_px`.
- Tests first in `tests/test_head_rgbd_calib_extrinsics.py`, seen failing: 20 synthetic pairs from `tests/rgbd_synthetic.py` with the true extrinsic recover the translation within 2 mm and the rotation within 0.2 deg (direction checked: a ToF point mapped with the result lands on the RGB projection of the same board corner within 0.5 px); a pair with no common corner ids is skipped; fewer than min_views pairs raises; with synthetic ToF depth biased by +20 mm the plane check reports 20 +- 3 mm, unbiased 0 +- 3 mm; the YAML round-trips; RMS over the limit writes nothing.

#### Acceptance conditions

- a1: synthetic pairs recover the known ToF-to-RGB extrinsic within 2 mm and 0.2 deg, in the right direction [evidence: test: tests/test_head_rgbd_calib_extrinsics.py]
- a2: the plane depth check reports a known synthetic depth bias [evidence: test: tests/test_head_rgbd_calib_extrinsics.py]
- a3: the extrinsics file round-trips and is refused above the stereo RMS limit [evidence: test: tests/test_head_rgbd_calib_extrinsics.py]

#### Files

- `scripts/head_rgbd_calibration/extrinsics.py`
- `tests/test_head_rgbd_calib_extrinsics.py`

#### Interfaces

- `scripts/head_rgbd_calibration/extrinsics.py:stereo_calibrate`, `plane_depth_bias`, `write_extrinsics_yaml`, `load_extrinsics_yaml`, `ExtrinsicResult` and the extrinsics YAML schema (part `head-rgbd-extrinsics-file`).

#### Expected outcome

A set of still pairs produces `head_rgbd_extrinsics.yaml` with the measured transform and its quality figures.

#### Steps

- Write the tests; run `uv run pytest tests/test_head_rgbd_calib_extrinsics.py -q`; see them fail.
- Implement extrinsics.py; rerun until green; `uv run poe lint && uv run ruff check scripts/head_rgbd_calibration && uv run ruff format --check scripts/head_rgbd_calibration`.
- Commit `feat(scripts): ToF-to-RGB extrinsic calibration`.

#### Notes

- OpenCV's stereoCalibrate returns R, T mapping camera-1 points into camera-2; pass the ToF as camera 1 and the RGB as camera 2 so the result is ToF -> RGB, and test the direction explicitly.
- The node never reads this file (design D4); the URDF hand-off does.

#### Ruling

Accepted by the orchestrator: verified and merged into main (root 1507 passed); head-rgbd-package-config verified by hand: scripts/cpp-node-test.sh head_rgbd_cpp PASS after freeing the Docker VM disk (the apt signature failure was a full Docker disk).

### S13: RGBD guided upsampling

- Status: done
- Files: `nodes/head_rgbd_cpp/include/head_rgbd_cpp/upsample.hpp`, `nodes/head_rgbd_cpp/src/core/upsample.cpp`, `nodes/head_rgbd_cpp/test/core/test_upsample.cpp`
- Verify: `scripts/cpp-node-test.sh head_rgbd_cpp`
- Verified head: be37e73d08f8587e16ece152f12a6a63030b68cf

#### Requirement

- `upsample.hpp/.cpp` (D8): `cv::Mat guided_upsample(const cv::Mat &depth_mm, const cv::Mat &guide_bgr, const UpsamplingConfig &cfg, UpsampleStats *stats = nullptr)`: guide size = 2x depth size (throws otherwise); D0 = nearest-upsampled depth, M = its validity mask (float 0/1); I = grey guide in [0, 1]; `num = cv::ximgproc::guidedFilter(I, D0 * M, radius, eps)`, `den = guidedFilter(I, M, radius, eps)`; output = num / den where `den >= cfg.min_support` and the pixel lies within `cfg.radius` of a valid D0 pixel (distance transform of the mask), else 0 (fix-as-ac: no placeholder depth outside the mask); CV_16UC1 mm.
- Tests first in `test/core/test_upsample.cpp` with `synthetic_scene.hpp`, seen failing: a depth step (1 m / 2 m) whose RGB edge is sharp: the upsampled step lies within 1 px of the RGB edge, while plain nearest upsampling is off by up to 2 px (both measured); a hole wider than 2 * radius stays 0 in its interior; pixels next to a hole are not pulled toward 0 (within 2 % of the true depth); a slanted plane's upsampled RMSE against the rendered 576x324 ground truth is lower than nearest-neighbour upsampling; a guide of the wrong size throws; all-invalid input gives all-zero output.

#### Acceptance conditions

- a1: upsampled depth edges follow the RGB edges more closely than nearest upsampling [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]
- a2: holes larger than the radius stay empty and zeros never bleed into valid depth [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]
- a3: upsampling lowers the error against ground truth on a slanted plane [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]

#### Files

- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/upsample.hpp`
- `nodes/head_rgbd_cpp/src/core/upsample.cpp`
- `nodes/head_rgbd_cpp/test/core/test_upsample.cpp`

#### Interfaces

- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/upsample.hpp`: `guided_upsample`, `UpsampleStats` (part `head-rgbd-upsampling`).

#### Expected outcome

A tested optional upsampler with a validity mask; disabled by default.

#### Steps

- Write test_upsample.cpp; run `scripts/cpp-node-test.sh head_rgbd_cpp`; see `FAIL`.
- Implement upsample.cpp with cv::ximgproc::guidedFilter; rerun until `PASS`.
- Commit `feat(head_rgbd_cpp): masked guided-filter upsampling`.

#### Notes

- Library first: cv::ximgproc::guidedFilter (O(N), radius-independent); do not hand-write box-filter guided filtering.

- 2026-10-18 run head-rgb-d-fusion-of-camera: dependency-failed on rgbd-registration-zbuffer

#### Ruling

Accepted by the orchestrator: verified and merged into main (root 1571 passed after integration fixes; head_rgbd_cpp C++ PASS); rgbd-methodology-doc parking flags were false positives describing runtime measurement by the robot runbook.

### S14: RGBD motion compensation

- Status: done
- Files: `nodes/head_rgbd_cpp/include/head_rgbd_cpp/motion.hpp`, `nodes/head_rgbd_cpp/src/core/motion.cpp`, `nodes/head_rgbd_cpp/test/core/test_motion.cpp`
- Verify: `scripts/cpp-node-test.sh head_rgbd_cpp`
- Verified head: d0a154ab51b2d045b4a812ba0f3e08ab100b5d8e

#### Requirement

- `motion.hpp/.cpp` (D6), Eigen only, no tf2 types:
  - `Eigen::Isometry3d compensated_transform(const Eigen::Isometry3d &fixed_T_rgb_at_trgb, const Eigen::Isometry3d &fixed_T_tof_at_ttof)` = `inverse(fixed_T_rgb) * fixed_T_tof`: the transform that maps ToF points captured at t_tof into the RGB frame at t_rgb (the same result as tf2's time-travel `lookupTransform(rgb, t_rgb, tof, t_tof, fixed)`).
  - `MotionRate frame_rate(const Eigen::Isometry3d &fixed_T_a, const Eigen::Isometry3d &fixed_T_b, double dt_s)`: angular rate (deg/s, from the relative rotation angle) and linear rate (m/s) of a frame between two poses.
  - `PairDecision evaluate_pair(int64_t t_rgb_ns, int64_t t_tof_ns, const MotionRate &tof_rate, const SyncConfig &, const MotionConfig &)`: `accept`, or a drop reason `pair_dt` (|t_rgb - t_tof| > max_pair_dt_s) or `motion` (angular or linear rate above the limits); `MotionConfig::compensate = false` keeps the gate and makes the node use the static transform.
- Tests first in `test/core/test_motion.cpp` using `synthetic_scene.hpp` and `register_depth`, seen failing: the head pans at 30 deg/s around the pan axis (the URDF pan/tilt geometry: pan axis 0.04 m below the cameras), t_rgb - t_tof = 33 ms: registering with the compensated transform matches the ground-truth RGB depth with median edge error <= 0.5 px at 288x162, while the static transform is off by >= 3 px (both measured, proving compensation matters); base rotation of 0.5 rad/s with a static head is compensated the same way; `frame_rate` of a 10 deg rotation over 0.5 s is 20 deg/s and of a 0.1 m translation over 0.5 s is 0.2 m/s; the gate drops at 15 deg/s with the 10 deg/s limit and accepts at 5 deg/s; pair_dt 0.07 s is dropped with the 0.06 s limit; reason strings are the metric labels `pair_dt` and `motion`.

#### Acceptance conditions

- a1: the compensated transform registers a panning head within 0.5 px where the static transform misses by 3 px or more [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]
- a2: frame angular and linear rates are computed from two poses [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]
- a3: pairs too far apart in time or captured during fast motion are dropped with their reason [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]

#### Files

- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/motion.hpp`
- `nodes/head_rgbd_cpp/src/core/motion.cpp`
- `nodes/head_rgbd_cpp/test/core/test_motion.cpp`

#### Interfaces

- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/motion.hpp`: `compensated_transform`, `frame_rate`, `MotionRate`, `evaluate_pair`, `PairDecision` (part `head-rgbd-motion`).

#### Expected outcome

Tested motion maths: the node only performs the two TF lookups and calls these functions.

#### Steps

- Write test_motion.cpp; run `scripts/cpp-node-test.sh head_rgbd_cpp`; see `FAIL`.
- Implement motion.cpp; rerun until `PASS`.
- Commit `feat(head_rgbd_cpp): motion compensation and gate`.

#### Notes

- Opus: frame conventions (which side is inverted) are the classic bug here; test both directions.
- A ToF frame integrates four phase captures, so blur during fast motion is gated, not corrected (design D6).

- 2026-10-18 run head-rgb-d-fusion-of-camera: dependency-failed on rgbd-registration-zbuffer

#### Ruling

Accepted by the orchestrator: verified and merged into main (root 1571 passed after integration fixes; head_rgbd_cpp C++ PASS); rgbd-methodology-doc parking flags were false positives describing runtime measurement by the robot runbook.

### S15: Head RGBD node wiring

- Status: done
- Files: `nodes/head_rgbd_cpp/include/head_rgbd_cpp/head_rgbd_node.hpp`, `nodes/head_rgbd_cpp/src/head_rgbd_node.cpp`, `nodes/head_rgbd_cpp/src/main.cpp`, `nodes/head_rgbd_cpp/test/test_node.cpp`, `nodes/head_rgbd_cpp/CMakeLists.txt`, `nodes/head_rgbd_cpp/README.md`
- Verify: `scripts/cpp-node-test.sh head_rgbd_cpp`
- Verified head: ef80f89c8aea0fbbabf23397811bb4642e84b9e9

#### Requirement

- `HeadRgbdNode` (rclcpp, executable `head_rgbd_node` from `src/main.cpp`, node name `head_rgbd`; D1, D2, D5, D6):
  - Startup: loads the config; loads `tof_calibration_file` with `camera_calibration_parsers::readCalibration`; a missing or invalid file is logged once and the node runs but publishes nothing (counted `no_calibration`).
  - Publishers (best_effort, depth 1, under `output_namespace`): `depth/image_raw` (16UC1), `depth/camera_info`, `rgb/image_raw/compressed`, `rgb/camera_info`, `points` (only when `cloud.enabled`), `depth_upsampled/image_raw` and `depth_upsampled/camera_info` (only when `upsampling.enabled`).
  - Lazy inputs (U4): a 1 s timer checks the publishers' subscription counts; with at least one subscriber it creates `message_filters::Subscriber`s for the ToF depth (`sensor_msgs/Image`), the RGB image (`sensor_msgs/CompressedImage`) and the RGB camera_info (best_effort) synchronised by `ApproximateTime` (queue `sync.queue_size`, `setMaxIntervalDuration(sync.max_pair_dt_s)`); with no subscriber for `lazy.idle_unsubscribe_s` it destroys them, so head_camera_cpp sees no mapping subscriber and releases the focus lock.
  - Per synchronised set: drop (counted by reason, nothing published) when: the ToF image size differs from the calibration (`size_mismatch`); the RGB camera_info has an all-zero K (`no_camera_info`); `evaluate_pair` rejects (`pair_dt`, `motion`); any TF lookup fails within `motion.tf_timeout_s` (`no_tf`). Otherwise: `filter_jump_edges` -> `T_rgb_tof` from `tf2_ros::Buffer::lookupTransform(rgb_frame, t_rgb, tof_frame, t_tof, fixed_frame)` (or the static lookup at t_rgb when `motion.compensate` is false) and the ToF frame rate from two lookups `motion.rate_window_s` apart -> `register_depth` into the RGB camera_info scaled by `output.divisor` -> publish depth, depth camera_info (scaled K, P, same D, frame of the RGB header), the RGB CompressedImage unchanged and the RGB camera_info unchanged, all with the RGB header (stamp and frame_id); when upsampling is enabled, decode the RGB, `guided_upsample`, publish at the RGB camera_info size; when the cloud decimator is due and `points` has subscribers, `decode_rgb_for_depth` + `build_cloud`.
  - Metrics via `ros2_common_cpp::Metrics` on `metrics_port`: `head_rgbd_pairs_published_total`, `head_rgbd_pairs_dropped_total{reason}`, `head_rgbd_pair_dt_seconds` (histogram), `head_rgbd_process_seconds` (histogram), `head_rgbd_valid_ratio` (gauge), plus the process series. Drop logs are throttled (5 s).
- CMakeLists: the node library and `head_rgbd_node` executable, `test/test_node.cpp` gtest with rclcpp, install rules for the executable and `config/`.
- Tests first in `test/test_node.cpp` (in-process publishers, a `tf2_ros::Buffer` fed with static and timed transforms, the test calibration `test/data/tof_calibration.yaml`), seen failing: with no output subscriber the node does not subscribe its inputs (the RGB topic shows zero subscriptions) and does after a subscriber appears, and releases them after the idle time; a synchronised ToF + RGB + camera_info set with valid TF yields one depth, depth camera_info, RGB and RGB camera_info, all with the RGB stamp and frame, depth 288x162 for a 576x324 camera_info, depth values matching `register_depth` on the same input, and the RGB payload byte-identical; nothing is published (and the matching drop counter rises) without TF, without the calibration file, with a zero-K camera_info, with a wrong ToF size, or during a 20 deg/s motion; the cloud is published at most at `cloud.rate_hz`; the upsampled topics exist only when enabled.
- `nodes/head_rgbd_cpp/README.md` (node contract): sections `## Topics` (subscriptions, publications, QoS, the exact-stamp rule), `## Configuration` (table of every key), `## Metrics`, `## Lazy inputs and focus lock`, `## No placeholder data` (each drop reason).

#### Acceptance conditions

- a1: inputs are subscribed only while an output has a subscriber and are released afterwards [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]
- a2: a synchronised set publishes registered depth, its camera_info and the unchanged RGB pair, all with the RGB stamp [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]
- a3: nothing is published without TF, calibration, camera_info, the right ToF size, or during fast motion, each drop counted [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]
- a4: the cloud respects its rate and upsampled outputs exist only when enabled [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]
- a5: the README documents topics, configuration, metrics, lazy inputs and the placeholder rules [evidence: doc-section: nodes/head_rgbd_cpp/README.md#No placeholder data]
- a6: the package builds and its tests and linters pass in docker [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]

#### Files

- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/head_rgbd_node.hpp`
- `nodes/head_rgbd_cpp/src/head_rgbd_node.cpp`
- `nodes/head_rgbd_cpp/src/main.cpp`
- `nodes/head_rgbd_cpp/test/test_node.cpp`
- `nodes/head_rgbd_cpp/CMakeLists.txt`
- `nodes/head_rgbd_cpp/README.md`

#### Interfaces

- Topics `/head_rgbd/depth/image_raw`, `/head_rgbd/depth/camera_info`, `/head_rgbd/rgb/image_raw/compressed`, `/head_rgbd/rgb/camera_info`, `/head_rgbd/points`, `/head_rgbd/depth_upsampled/*` (part `head-rgbd-topics`), consumed by the RTAB-Map epic's uplink and by future web UI / agent views.
- Executable `head_rgbd_cpp head_rgbd_node`, env `HEAD_RGBD_CONFIG`.

#### Expected outcome

`ros2 run head_rgbd_cpp head_rgbd_node` publishes exact-stamp RGB-D pairs from live inputs and nothing otherwise.

#### Steps

- Write test_node.cpp; run `scripts/cpp-node-test.sh head_rgbd_cpp`; see `FAIL`.
- Implement the node, main and CMake targets; rerun until `PASS`.
- Write the README; commit `feat(head_rgbd_cpp): fusion node wiring`.

#### Notes

- The node never reads the extrinsics file: geometry comes from TF (head URDF, design D4).
- The mapping stream (head-camera-mapping-publish) publishes best_effort depth 1; match it.
- Raw RGB is never subscribed (UDPv4 fragment drops, design Constraints).

- 2026-10-18 run head-rgb-d-fusion-of-camera: dependency-failed on tof-jump-edge-filter, rgbd-motion-compensation, rgbd-guided-upsampling, rgbd-cloud-builder

#### Ruling

Accepted by the orchestrator: verified and merged into main (root 1571 passed after integration fixes; head_rgbd_cpp C++ PASS); rgbd-methodology-doc parking flags were false positives describing runtime measurement by the robot runbook.

### S16: RGBD validation metrics

- Status: done
- Files: `scripts/head_rgbd_calibration/validate.py`, `tests/test_head_rgbd_validate.py`
- Verify: `uv run pytest tests/test_head_rgbd_validate.py -q && uv run poe lint && uv run ruff check scripts/head_rgbd_calibration && uv run ruff format --check scripts/head_rgbd_calibration`
- Verified head: 94794dcf1228527f6ccdaf14d1d1a5275cd86a4c

#### Requirement

- `scripts/head_rgbd_calibration/validate.py` (D11, methodology "Validation metrics and targets"):
  - `depth_accuracy(depth_frames: list[np.ndarray], reference_m: float, roi_px: int) -> DepthStats`: central ROI over the frames, invalid (0) pixels excluded, bias and RMSE in mm, valid fraction; `registration_bias(raw_frames, registered_frames, roi) -> float` (median registered minus median raw over the matching ROI, mm).
  - `edge_alignment(registered_depth: np.ndarray, rgb: np.ndarray, board, rgb_camera, depth_scale: int) -> AlignmentStats`: the board outline from RGB ChArUco detection + `solvePnP` + the board's outer rectangle projected into the output image, against depth-step pixels of the registered depth (a valid pixel with a neighbour farther by > 0.3 m) near that outline; median and p90 distance in output pixels.
  - `valid_ratio(registered_depth, fov_mask) -> float`.
  - `time_offset(rgb_tracks, tof_tracks, extrinsic, rgb_camera, tof_camera) -> OffsetEstimate`: from a pan sweep, ToF ChArUco corners (with their ToF depth) projected into the RGB image with the extrinsic, compared with the RGB corner track of the same id: offset = (u_projected - u_rgb) / (du_rgb/dt) per matched pair, the median in ms and its spread; pairs with |du/dt| below a minimum speed are ignored.
  - `check_targets(results) -> list[TargetResult]`: each methodology target (values from the design table) with measured value and pass/fail.
- Tests first in `tests/test_head_rgbd_validate.py`, seen failing: a synthetic flat target at 1.0 m with +12 mm bias and 8 mm noise gives bias 12 +- 2 mm and RMSE within 2 mm of the truth; zeros are excluded; a synthetic registered scene (board in front of a wall) with the depth shifted 2 px right gives median alignment 2 +- 0.3 px and 0 +- 0.3 px unshifted; synthetic sweep tracks with a true 25 ms offset give 25 +- 3 ms, with 0 ms give 0 +- 3 ms; slow-motion pairs are ignored; `check_targets` flags a value over its limit.

#### Acceptance conditions

- a1: depth bias, RMSE and registration bias are measured in a central ROI excluding invalid pixels [evidence: test: tests/test_head_rgbd_validate.py]
- a2: edge alignment recovers a known synthetic pixel shift [evidence: test: tests/test_head_rgbd_validate.py]
- a3: the ToF-RGB time offset is recovered from synthetic pan-sweep corner tracks [evidence: test: tests/test_head_rgbd_validate.py]
- a4: results are checked against the methodology targets [evidence: test: tests/test_head_rgbd_validate.py]

#### Files

- `scripts/head_rgbd_calibration/validate.py`
- `tests/test_head_rgbd_validate.py`

#### Interfaces

- `scripts/head_rgbd_calibration/validate.py:depth_accuracy`, `registration_bias`, `edge_alignment`, `valid_ratio`, `time_offset`, `check_targets` (part `head-rgbd-validate`).

#### Expected outcome

Recorded bags turn into the methodology's numbers with pass/fail against the targets.

#### Steps

- Write the tests with synthetic scenes from tests/rgbd_synthetic.py; run `uv run pytest tests/test_head_rgbd_validate.py -q`; see them fail.
- Implement validate.py; rerun until green; `uv run poe lint && uv run ruff check scripts/head_rgbd_calibration && uv run ruff format --check scripts/head_rgbd_calibration`.
- Commit `feat(scripts): RGB-D validation metrics`.

#### Notes

- Validation uses OpenCV projections of a few corner points; it does not reimplement the node's z-buffer.
- Pan sweep for the time offset: head pan only, +-20 deg at about 20 deg/s, inside the pan/tilt epic's verified range (servo safety: least dramatic joint).

#### Ruling

Accepted by the orchestrator: verified and merged into main (root 1507 passed); head-rgbd-package-config verified by hand: scripts/cpp-node-test.sh head_rgbd_cpp PASS after freeing the Docker VM disk (the apt signature failure was a full Docker disk).

### S17: URDF extrinsics handoff

- Status: done
- Files: `scripts/head_rgbd_calibration/urdf_handoff.py`, `tests/test_head_rgbd_urdf_handoff.py`, `nodes/web_ui/tests/test_head_urdf.py`
- Verify: `uv run pytest tests/test_head_rgbd_urdf_handoff.py -q && uv run poe lint && uv run ruff check scripts/head_rgbd_calibration && uv run ruff format --check scripts/head_rgbd_calibration && cd nodes/web_ui && uv run --frozen pytest tests/test_head_urdf.py -q && uv run --frozen poe lint`
- Verified head: 9bd313f28afb6b932b6ff1d47915769cbc9dcff4

#### Requirement

- `scripts/head_rgbd_calibration/urdf_handoff.py` (D4):
  - `read_chain(urdf_text: str) -> HeadChain`: the fixed joints `head_camera_joint`, `head_camera_optical_joint`, `tof_joint`, `tof_optical_joint` (origin xyz and rpy) and their parents/children, as parsed by `xml.etree.ElementTree`; raises naming a missing joint.
  - `tof_joint_origin(chain: HeadChain, extrinsic: ExtrinsicResult) -> Origin`: the `tof_joint` origin (relative to `head_link`) such that `T(head_camera_optical_frame <- tof_optical_frame)` computed through the URDF equals the calibrated transform: `T_head_tof = T_head_cam * T_cam_camopt * T_camopt_tofopt * inv(T_tof_tofopt)` (scipy `Rotation` for rpy conversion, URDF fixed-axis rpy convention).
  - `rewrite_tof_joint(urdf_text: str, origin: Origin, note: str) -> str`: replaces only the `xyz`/`rpy` attributes of the `<origin>` inside `<joint name="tof_joint">` (text edit, every other byte unchanged, comments kept) and puts `note` (calibration date and stereo RMS) in a comment directly above that joint, replacing a previous calibration comment rather than stacking them.
  - `check_drift(chain, extrinsic) -> Drift`: translation (mm) and rotation (deg) between the URDF's current transform and the calibration.
- `nodes/web_ui/tests/test_head_urdf.py` (pan/tilt epic test): the exact ToF origin assertion becomes: ToF y > RGB y, camera-to-ToF spacing within 0.05 +- 0.015 m, `tof_joint` origin |x| <= 0.05 m and |rpy| <= 0.1 rad; every structural check (chain, axes, limits, optical rpy, post) stays exact.
- Tests first in `tests/test_head_rgbd_urdf_handoff.py` on a copy of a URDF fixture with the pan/tilt chain (read from `nodes/web_ui/urdf/head.urdf` when it exists, else an inline fixture with the same joints), seen failing: rewriting with a known extrinsic and re-reading the chain reproduces the extrinsic within 1e-6 m / 1e-6 rad; the diff between input and output touches only the tof_joint origin line and the calibration comment; a second rewrite replaces the comment; a missing joint raises; drift is zero after a rewrite and equals the injected offset before.

#### Acceptance conditions

- a1: the rewritten URDF chain reproduces the calibrated ToF-to-RGB transform [evidence: test: tests/test_head_rgbd_urdf_handoff.py]
- a2: only the tof_joint origin and its calibration comment change [evidence: test: tests/test_head_rgbd_urdf_handoff.py]
- a3: drift between URDF and calibration is reported without writing [evidence: test: tests/test_head_rgbd_urdf_handoff.py]
- a4: the head URDF test accepts calibrated values within tolerance and keeps the structural checks [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_head_urdf.py -q]

#### Files

- `scripts/head_rgbd_calibration/urdf_handoff.py`
- `tests/test_head_rgbd_urdf_handoff.py`
- `nodes/web_ui/tests/test_head_urdf.py`

#### Interfaces

- `scripts/head_rgbd_calibration/urdf_handoff.py:read_chain`, `tof_joint_origin`, `rewrite_tof_joint`, `check_drift` (part `head-rgbd-urdf-handoff`).

#### Expected outcome

Given an extrinsics file, the head URDF's tof_joint is rewritten so TF reproduces the calibration exactly.

#### Steps

- Write the tests and relax the web_ui URDF test; run `uv run pytest tests/test_head_rgbd_urdf_handoff.py -q`; see it fail.
- Implement urdf_handoff.py; rerun until green; run the full verify command.
- Commit `feat(scripts): write calibrated ToF extrinsic into the head URDF`.

#### Notes

- The head URDF is created by the pan/tilt epic (blocker); its chain: head_link -> head_camera_joint / tof_joint -> optical joints with rpy (-pi/2, 0, -pi/2).
- After the runbook rewrites the URDF, head_state_publisher (robot_state_publisher) must be redeployed to publish the new TF.

#### Ruling

Accepted by the orchestrator: verified and merged into main (root 1507 passed); head-rgbd-package-config verified by hand: scripts/cpp-node-test.sh head_rgbd_cpp PASS after freeing the Docker VM disk (the apt signature failure was a full Docker disk).

### S18: Head RGBD node type

- Status: done
- Files: `ansible/group_vars/client.yml`, `ansible/playbooks/deploy_nodes_client.yml`, `tests/test_head_rgbd_deploy.py`, `tests/test_unit_cgroups.py`, `tests/test_node_metrics_wiring.py`, `ansible/README.md`, `nodes/README.md`, `.claude/skills/ansible-deploy/SKILL.md`
- Verify: `uv run pytest tests/test_head_rgbd_deploy.py tests/test_unit_cgroups.py tests/test_node_metrics_wiring.py tests/test_cpp_node_deploy.py -q && uv run poe lint && uv run poe lint-ansible && uv run poe test-ansible`
- Verified head: 5883535fca2762a1bc77485aa44f232153db2958

#### Requirement

- client.yml node type `head_rgbd_cpp` (D10): native, `node_launch_command: "ros2 run head_rgbd_cpp head_rgbd_node"`, `node_src_dir: ""`, `src_paths: []`, `colcon_source` with `shared/ros2_common_cpp` then `nodes/head_rgbd_cpp` (Release, BUILD_TESTING OFF, `-j2`, base_paths `.`, `stamp_apt_packages` listing the ROS/OpenCV packages), apt: build-essential, cmake, rsync, python3-colcon-common-extensions, ros-jazzy-ament-cmake, ros-jazzy-rclcpp, ros-jazzy-sensor-msgs, ros-jazzy-message-filters, ros-jazzy-tf2-ros, ros-jazzy-tf2-eigen, ros-jazzy-cv-bridge, ros-jazzy-image-geometry, ros-jazzy-camera-calibration-parsers, ros-jazzy-depth-image-proc, libopencv-dev, libopencv-contrib-dev, libeigen3-dev, libyaml-cpp-dev, prometheus-cpp-dev; background tier `cpu_weight: 100`, `io_weight: 100`, `oom_score_adjust: 0`, `memory_low: "48M"`, `memory_max: "256M"` (until measured by the runbook); `config_path: /etc/ros2/head_rgbd_cpp`; env `HEAD_RGBD_CONFIG=/etc/ros2/head_rgbd_cpp/config.yaml`; the ToF calibration file `nodes/head_rgbd_cpp/config/tof_calibration.yaml` deployed to `/etc/ros2/head_rgbd_cpp/tof_calibration.yaml` when it exists in the repo (same mechanism head_camera_cpp uses for its `camera_info_file`); a free metrics port in the node config. No `ROS_STATIC_PEERS`.
- ros2_nodes: `head_rgbd_cpp` present/enabled with the config block (defaults from `nodes/head_rgbd_cpp/config/head_rgbd.yaml`, commented); the `head_camera_cpp` config block gets `lens_position: 1.0` and `mapping: {enabled: true, lock_focus: true, rate_hz: 2.0, width: 576, height: 324, jpeg_quality: 80}` with `af_mode` left continuous (U4); the `arducam_tof` config block gets `stamp_latency_ms` (0.0 with a comment that the runbook writes the measured value).
- `deploy_nodes_client.yml`: step `Deploy head_rgbd_cpp` (resolve_and_deploy.yml, tags `[head_rgbd_cpp, apt, build, config]`) after head_camera_cpp and arducam_tof.
- Tests first, seen failing: new `tests/test_head_rgbd_deploy.py` (node type keys, apt list contains libopencv-contrib-dev and depth-image-proc, background tier values, env, colcon sources order, the ros2_nodes entry, the head_camera_cpp mapping/lock config with af_mode continuous, arducam_tof stamp_latency_ms present, playbook step order); `tests/test_unit_cgroups.py` covers the new type in the background tier; `tests/test_node_metrics_wiring.py` knows its metrics port (unique); `tests/test_cpp_node_deploy.py` passes for the new C++ type.
- Docs: `ansible/README.md` node list entry and the head camera mapping/focus-lock config; `nodes/README.md` entry for head_rgbd_cpp; `.claude/skills/ansible-deploy/SKILL.md` lists head_rgbd_cpp among client nodes with its deploy command.

#### Acceptance conditions

- a1: the head_rgbd_cpp node type, tier, apt packages, calibration file deploy and ros2_nodes entry are in client.yml [evidence: test: tests/test_head_rgbd_deploy.py]
- a2: the head camera mapping stream with focus lock and the ToF stamp latency are configured [evidence: test: tests/test_head_rgbd_deploy.py]
- a3: cgroup tier and metrics port are covered [evidence: test: tests/test_unit_cgroups.py]
- a4: ansible lint and syntax checks pass [evidence: command: uv run poe lint-ansible && uv run poe test-ansible]
- a5: ansible README, nodes README and the deploy skill list the node [evidence: command: grep -q head_rgbd_cpp ansible/README.md && grep -q head_rgbd_cpp nodes/README.md && grep -q head_rgbd_cpp .claude/skills/ansible-deploy/SKILL.md]

#### Files

- `ansible/group_vars/client.yml`
- `ansible/playbooks/deploy_nodes_client.yml`
- `tests/test_head_rgbd_deploy.py`
- `tests/test_unit_cgroups.py`
- `tests/test_node_metrics_wiring.py`
- `ansible/README.md`
- `nodes/README.md`
- `.claude/skills/ansible-deploy/SKILL.md`

#### Interfaces

- Deployed unit `ros2-head_rgbd_cpp`; ansible node type `head_rgbd_cpp`; client.yml keys `head_camera_cpp.config.mapping.*`, `arducam_tof.config.stamp_latency_ms`.

#### Expected outcome

`./scripts/deploy-nodes.sh client head_rgbd_cpp` builds and starts the fusion node with its config.

#### Steps

- Write tests/test_head_rgbd_deploy.py and the cgroup/metrics test changes; run `uv run pytest tests/test_head_rgbd_deploy.py -q`; see it fail.
- Edit client.yml and the playbook; run the verify command until green.
- Update the READMEs and the deploy skill; commit `feat(ansible): head_rgbd_cpp node type`.

#### Notes

- Pattern: the map_guard_cpp and head_camera_cpp node types in client.yml (colcon_source, stamp_apt_packages).
- MEMORY: cgroups guarantee headroom for robot nodes; the fusion node is background work, so it gets the background tier, not protection.
- Do not deploy in this task; the runbook does.

- 2026-10-18 run head-rgb-d-fusion-of-camera: dependency-failed on head-rgbd-node-wiring

#### Ruling

Accepted by the orchestrator: verified and merged into main (root 1571 passed after integration fixes; head_rgbd_cpp C++ PASS); rgbd-methodology-doc parking flags were false positives describing runtime measurement by the robot runbook.

### S19: RGBD calibrate CLI

- Status: done
- Files: `scripts/head_rgbd_calibrate.py`, `scripts/head_rgbd_calibrate.yaml`, `tests/test_head_rgbd_calibrate_cli.py`, `scripts/README.md`
- Verify: `uv run pytest tests/test_head_rgbd_calibrate_cli.py -q && uv run poe lint && uv run ruff check scripts/head_rgbd_calibration && uv run ruff format --check scripts/head_rgbd_calibration && uv run ruff check scripts/head_rgbd_calibrate.py && uv run ruff format --check scripts/head_rgbd_calibrate.py`
- Verified head: 340a9515a3f093352355425a7e2a81e074103ab5

#### Requirement

- `scripts/head_rgbd_calibrate.py` (`main(argv: list[str] | None = None) -> int`): the only CLI arguments are an optional config path, the mode (`capture`, `capture-validation`, `calibrate`, `urdf`, `validate`) and, for `capture-validation`, the session suffix (`0.5m`, `1.0m`, `2.0m`, `3.0m`, `sweep`, `wall`); everything else comes from `scripts/head_rgbd_calibrate.yaml` (gains a `mode`-independent `session` key naming the bag to use, default the newest under `capture.local_dir`).
  - `capture`: `capture_bag`, prints the local bag path.
  - `capture-validation <suffix>`: `capture_bag` with `validation_topics`, bag named `<session>-<suffix>`.
  - `calibrate`: `read_frames` -> `still_pairs` -> detections -> RGB intrinsics, ToF intrinsics (with the SDK comparison from the bag's `/tof/camera_info`), stereo extrinsic + plane check -> writes the three output files and a markdown report in `report_dir` (RMS values, view counts, SDK differences, plane bias, lens position read from the head camera config given in the yaml); exits 1 without writing anything when any limit fails, naming it.
  - `urdf`: `check_drift`, then `rewrite_tof_joint` on `outputs.urdf`; `urdf --check` only prints the drift.
  - `validate`: computes the metrics for a validation bag (depth accuracy per configured distance from bags named `<session>-<distance>m`, edge alignment, valid ratio, time offset from a `<session>-sweep` bag) and prints and writes the target table (`check_targets`); exit 1 when a target fails.
- Tests first in `tests/test_head_rgbd_calibrate_cli.py`, seen failing: an end-to-end run on a synthetic bag written with the rosbags writer from `tests/rgbd_synthetic.py` renders (20 still poses, both cameras, camera_infos) writes three files into a tmp dir whose values match the synthetic truth (intrinsics within the intrinsics task's tolerances, extrinsic within 2 mm / 0.2 deg); a bag with too few still poses exits 1 and writes nothing; `urdf` on a tmp URDF copy rewrites tof_joint and `urdf --check` leaves it byte-identical; `validate` on synthetic distance bags prints a passing table and exits 0, and exits 1 when a synthetic bias exceeds the target; capture with a fake runner prints the bag path.
- `scripts/README.md`: a `head_rgbd_calibrate.py` entry: purpose, the board to print (from the yaml; printed with `uv run python -m scripts.markers_generate` of marker-and-calibration-board), the capture protocol (still poses, distances, warm-up), the four modes, outputs and where they are consumed (head_camera_cpp, head_rgbd_cpp, head URDF), and that capture engages the head camera focus lock by subscribing the mapping topic.

#### Acceptance conditions

- a1: calibrate on a synthetic bag writes RGB, ToF and extrinsics files matching the truth [evidence: test: tests/test_head_rgbd_calibrate_cli.py]
- a2: a failing limit exits 1 and writes nothing [evidence: test: tests/test_head_rgbd_calibrate_cli.py]
- a3: urdf rewrites the tof_joint and --check is read-only [evidence: test: tests/test_head_rgbd_calibrate_cli.py]
- a4: validate prints the target table and fails on a missed target [evidence: test: tests/test_head_rgbd_calibrate_cli.py]
- a5: scripts/README documents the tool [evidence: command: grep -q 'head_rgbd_calibrate.py' scripts/README.md]

#### Files

- `scripts/head_rgbd_calibrate.py`
- `scripts/head_rgbd_calibrate.yaml`
- `tests/test_head_rgbd_calibrate_cli.py`
- `scripts/README.md`

#### Interfaces

- `scripts/head_rgbd_calibrate.py:main` and modes `capture`, `calibrate`, `urdf`, `validate`; outputs as configured in `scripts/head_rgbd_calibrate.yaml`.

#### Expected outcome

`python scripts/head_rgbd_calibrate.py calibrate` turns a capture bag into the three calibration files; the runbook drives every robot step through it.

#### Steps

- Write the end-to-end tests; run `uv run pytest tests/test_head_rgbd_calibrate_cli.py -q`; see them fail.
- Implement the entry point; rerun until green; run the verify command.
- Update scripts/README.md; commit `feat(scripts): head RGB-D calibration CLI`.

#### Notes

- Replaces the RTAB-Map epic's planned `scripts/head_camera_calibrate.py` (design D3); do not create that file.
- Do not touch the robot here.

#### Ruling

Accepted by the orchestrator: verified and merged into main (root 1507 passed); head-rgbd-package-config verified by hand: scripts/cpp-node-test.sh head_rgbd_cpp PASS after freeing the Docker VM disk (the apt signature failure was a full Docker disk).

### S20: RGBD methodology doc

- Status: done
- Files: `docs/head-rgbd-fusion-methodology.md`, `nodes/head_rgbd_cpp/README.md`, `tests/README.md`, `MEMORY.md`
- Verify: `uv run poe lint && grep -q 'test_head_rgbd_deploy.py' tests/README.md && grep -q 'head-rgbd-fusion-methodology.md' nodes/head_rgbd_cpp/README.md`
- Verified head: 115547aa1a9f7ae4ca824879f6058f65504e2ecb

#### Requirement

- `docs/head-rgbd-fusion-methodology.md` (G1, U2), written for a mid-level ROS 2 developer, from the design's `## Methodology` section and decisions, with the citations kept: `## Hardware and geometry` (both cameras, resolutions, fields of view, the 1:1.15 mapping at 288x162, the URDF frames); `## Calibration` (focus lock and why, ChArUco board and printing, capture protocol with still poses and distances, RGB and ToF intrinsics with the zero-distortion decision rule and the SDK comparison, stereo extrinsic with fixed intrinsics, plane depth check, depth bias at known distances after warm-up and the 1 cm correction rule, the URDF hand-off and redeploying head_state_publisher); `## Synchronisation` (stamps of both sensors, the stamp_latency_ms fix, ApproximateTime pairing and its residual, exact-stamp outputs, latency budget, rolling-shutter error model); `## Motion` (time-travel compensation, the gate and its thresholds, why blur is gated); `## Registration` (ray table, transform, distortion, footprint splat, z-buffer, occlusion shadows, why not depth_image_proc's RegisterNode); `## Filtering` (jump-edge filter parameters, what confidence thresholding does and does not do, multipath as a known error); `## Upsampling` (masked guided filter, validity mask, cost, when to enable); `## Validation` (every metric of the design's target table, how it is measured with `scripts/head_rgbd_calibrate.py validate`, and a `### Results` table with one row per metric and empty measured/pass columns that the runbook fills); `## CPU budget` (0.2 core target and how it is read in Grafana).
- `nodes/head_rgbd_cpp/README.md`: a `## Methodology` section linking the document and summarising the defaults.
- `tests/README.md`: entries for test_head_rgbd_calib_frames.py, test_head_rgbd_calib_intrinsics.py, test_head_rgbd_calib_extrinsics.py, test_head_rgbd_urdf_handoff.py, test_head_rgbd_validate.py, test_head_rgbd_calibrate_cli.py, rgbd_synthetic.py, test_head_rgbd_deploy.py and the changes to test_unit_cgroups.py and test_node_metrics_wiring.py, each with what it covers.
- `MEMORY.md`: one decision entry: head ToF depth is registered on the client by head_rgbd_cpp into the head camera mapping image with exact-stamp pairs; RTAB-Map consumes the pair; the head URDF is the only extrinsic source; focus is locked only while the mapping stream has subscribers.

#### Acceptance conditions

- a1: the methodology document covers calibration, synchronisation, motion, registration, filtering, upsampling, validation and CPU [evidence: doc-section: docs/head-rgbd-fusion-methodology.md#Validation]
- a2: the calibration section states the focus lock, the board, the capture protocol and the URDF hand-off [evidence: doc-section: docs/head-rgbd-fusion-methodology.md#Calibration]
- a3: the synchronisation section states the stamp fix, pairing residual and latency budget [evidence: doc-section: docs/head-rgbd-fusion-methodology.md#Synchronisation]
- a4: the node README links the methodology [evidence: doc-section: nodes/head_rgbd_cpp/README.md#Methodology]
- a5: tests/README lists every new test file of the epic [evidence: command: grep -q 'test_head_rgbd_calib_frames.py' tests/README.md && grep -q 'test_head_rgbd_calib_intrinsics.py' tests/README.md && grep -q 'test_head_rgbd_calib_extrinsics.py' tests/README.md && grep -q 'test_head_rgbd_urdf_handoff.py' tests/README.md && grep -q 'test_head_rgbd_validate.py' tests/README.md && grep -q 'test_head_rgbd_calibrate_cli.py' tests/README.md && grep -q 'test_head_rgbd_deploy.py' tests/README.md]
- a6: MEMORY.md records the registration decision [evidence: command: grep -q 'head_rgbd_cpp' MEMORY.md]

#### Files

- `docs/head-rgbd-fusion-methodology.md`
- `nodes/head_rgbd_cpp/README.md`
- `tests/README.md`
- `MEMORY.md`

#### Interfaces

- `docs/head-rgbd-fusion-methodology.md` with a `### Results` table filled by rgbd-final-suite-and-runbook.

#### Expected outcome

The methodology is written down; the runbook only has to fill in measured numbers.

#### Steps

- Write the document and README sections from design.md (Methodology, D3-D8, targets).
- Run the verify command; expect pass.
- Commit `docs: head RGB-D fusion methodology`.

#### Notes

- Mostly prose: `review: skip`, judged at the epic audit (start-development: prose stays out of the review loop).

- 2026-10-18 run head-rgb-d-fusion-of-camera: dependency-failed on head-rgbd-node-type

#### Ruling

Accepted by the orchestrator: verified and merged into main (root 1571 passed after integration fixes; head_rgbd_cpp C++ PASS); rgbd-methodology-doc parking flags were false positives describing runtime measurement by the robot runbook.

## Files to delete

none recorded in the epic format

## Final verify

none recorded in the epic format

## After merge

none
