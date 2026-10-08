# Tests

Unit tests for the project. Run with `poetry run pytest` or `mise exec -- poetry run pytest` from the repo root. Config: [pyproject.toml](../pyproject.toml) (`[tool.pytest.ini_options]`).

## Test files and what they cover

### `test_master2master_config.py`

Unit tests for master2master topic proxy config parsing (`nodes/master2master/master2master/config.py`). Imports from `nodes/master2master` via path setup.

| Test | Description |
|------|-------------|
| `test_load_config_from_dict_empty` | Empty or missing `topics` / `topic_proxy` yields an empty list of rules. |
| `test_load_config_from_dict_string_entries` | String entries (e.g. `["/foo", "/bar"]`) become `TopicRule` with `source=dest`, `direction="in"`. |
| `test_load_config_from_dict_dict_entries` | Dict entries support `source`/`from`, `dest`/`to`, `direction`; default direction is `"in"`. |
| `test_load_config_from_dict_skips_missing_source` | Entries without a source (or from) are skipped; result list does not contain them. |
| `test_load_config_from_dict_msg_type` | Dict entries support `type` (e.g. `"JointState"`); default `msg_type` is `"string"`. |
| `test_load_config_from_dict_invalid_direction_raises` | Invalid `direction` raises `ConfigError`. |
| `test_load_config_from_dict_invalid_type_raises` | Invalid `type` raises `ConfigError`. |
| `test_load_config_from_dict_topics_not_list_raises` | `topics` / `topic_proxy` must be a list; otherwise `ConfigError`. |
| `test_load_config_from_dict_normalizes_topic_slash` | Topic names get a leading slash if missing. |
| `test_normalize_topic_empty_or_whitespace` | `normalize_topic` returns empty string for empty or whitespace input. |
| `test_normalize_topic_adds_leading_slash` | `normalize_topic` adds leading slash when missing. |
| `test_normalize_topic_strips_trailing_slash` | `normalize_topic` strips trailing slash; single slash stays `/`. |
| `test_load_config_from_file` | `load_config` reads and parses YAML from file path (tmp_path). |
| `test_load_config_nonexistent_returns_empty` | `load_config` returns empty list for nonexistent path. |
| `test_parse_rule_entry_invalid_type_raises` | `parse_rule_entry` raises `ConfigError` for non-dict non-str entry. |
| `test_validate_relay_rules_allows_acyclic_rules` | `validate_relay_rules` accepts rules where no rule's `dest` is another's `source`. |
| `test_validate_relay_rules_raises_when_dest_is_source_of_another` | `validate_relay_rules` raises `ValueError` when a rule's `dest` equals another rule's `source` (relay loop). |

### `test_shared_utils.py`

Unit tests for the shared library `shared/ros2_common/_utils.py`. Imports from repo root path.

| Test | Description |
|------|-------------|
| `test_clamp_within` | `clamp(value, low, high)` returns `value` when it lies within `[low, high]`. |
| `test_clamp_low` | `clamp` returns `low` when `value` is below the range. |
| `test_clamp_high` | `clamp` returns `high` when `value` is above the range. |
| `test_clamp_equal_bounds` | `clamp` with low == high returns that bound. |
| `test_clamp_at_bounds` | `clamp` returns value when value equals low or high. |

### `test_shared_camera_geometry.py`

Tests of `shared/ros2_common/camera_geometry.py` and `scripts/solve_camera_mount.py` with synthetic cameras.

| Test | Description |
|------|-------------|
| `test_from_hfov_fx` | `from_hfov` gives fx = (w/2)/tan(hfov/2), square pixels, centred principal point. |
| `test_mount_to_matrix_identity_and_translation` | Zero RPY gives identity rotation and translation is copied. |
| `test_mount_yaw_rotates_x_to_y` | Fixed-axis RPY: yaw pi/2 maps x to y. |
| `test_optical_from_mount_convention` | Optical z/x/y equal body x/-y/-z (REP-103). |
| `test_camera_model_calibrated` | `calibrated` needs intrinsics and mount. |
| `test_center_pixel_ray_is_optical_z` | Principal point ray is +z. |
| `test_ray_is_unit_and_right_down` | Rays are unit; right/below pixels have positive x/y. |
| `test_undistort_without_distortion` | Normalized coordinates without distortion. |
| `test_undistort_inverts_plumb_bob` | Undistortion inverts a plumb_bob distortion. |
| `test_intersect_ground_basic` | 45 degree ray from 1 m hits 1 m ahead. |
| `test_intersect_ground_parallel_and_behind` | Parallel or upward rays return None. |
| `test_camera_ray_in_frame_origin` | Ray origin is the camera translation. |
| `test_pixel_ground_roundtrip` | pixel -> ground -> pixel within 1e-6 px, with and without distortion. |
| `test_pixel_above_horizon_gives_none` | Sky pixel has no ground point. |
| `test_project_behind_camera_and_outside_image` | Behind-camera or off-image points project to None. |
| `test_from_calibration_yaml` | Loads a camera_calibration yaml. |
| `test_solver_recovers_known_mount` | Noisy samples recover the mount (< 5 mm, < 0.5 deg, rms < 0.5 px). |
| `test_solver_needs_three_samples` | Fewer than 3 samples raises ValueError. |
| `test_solve_camera_mount_script` | The CLI prints the solved mount as YAML plus rms. |

### `test_shared_battery.py`

Pure-logic tests of `shared/ros2_common/battery.py` (`BatteryConfig`, `BatteryGuard`), run by the root pytest.

| Test | Description |
|------|-------------|
| `test_battery_config_defaults` | Defaults: `/battery_state`, 3 cells, 2.8 / 2.9 V per cell, `stale_s` 5.0. |
| `test_battery_config_rejects_resume_below_cutoff` | `resume_cell_v < cutoff_cell_v` fails validation. |
| `test_battery_config_allows_equal_thresholds` | Equal cut-off and resume thresholds are valid. |
| `test_battery_config_rejects_bad_cells` | `cells` 0 and -1 fail validation. |
| `test_guard_unknown_without_reading` | No reading: not in cut-off, voltage `None`. |
| `test_guard_enters_cutoff_below_threshold` | Cut-off starts strictly below 8.4 V (3 cells). |
| `test_guard_hysteresis_leaves_only_above_resume` | Cut-off is released only strictly above 8.7 V and stays released between thresholds. |
| `test_guard_stale_reading_is_not_blocked` | A reading older than `stale_s` counts as unknown and is not blocked. |
| `test_guard_ignores_invalid_voltage` | NaN, inf, 0 and negative readings are ignored. |
| `test_guard_state_and_rejection_message` | `state()` snapshot and the `...; motion refused` error text. |
| `test_guard_is_thread_safe` | Concurrent updates and reads leave a consistent state. |
| `test_guard_from_config` | `from_config` applies the config thresholds. |

### `test_shared_package.py`

Packaging of `shared/` as the installable Poetry package `ros2-common`.

| Test | Description |
|------|-------------|
| `test_shared_pyproject_defines_ros2_common` | `shared/pyproject.toml` is named `ros2-common` and ships the `ros2_common` module. |
| `test_nodes_depend_on_shared_by_relative_develop_path` | Consumer nodes (mcp_server) declare a `develop = true` path dependency whose relative path resolves to `shared/`; the same layout holds on the Pi (whole repo in `ros2_repo_dest`, `poetry install` run in `nodes/<node>`). |

### `test_topic_scraper_collect.py`

Unit tests for `scripts/topic_scraper_collect.py` parser/format helpers.

| Test | Description |
|------|-------------|
| `test_parse_source` | Parses `name=url` source argument into normalized source object. |
| `test_parse_selector` | Parses `/topic:jq-filter` selector into topic and jq expression. |
| `test_topic_endpoint` | Verifies endpoint mapping (`/topic` -> `/topics/topic`). |
| `test_build_record` | Verifies merged NDJSON record fields for source/topic/timing/value. |

### Per-node tests (feetech_servos)

The **feetech_servos** node has its own test suite under `nodes/bridges/feetech_servos/tests/`. Run from that directory: `poetry run pytest tests/ -v` (or `poetry run poe test`). A `conftest.py` mocks the `st3215` module so script tests (calibrate_servos, set_servo_id) run without the hardware library installed. Covers: config loading and validation (`test_config.py`: namespace, joint_names as list of `{ name, id }` per joint—missing namespace/joint_names, device/baudrate, log_joint_updates, torque startup (`enable_torque_on_start` and `disable_torque_on_start`), and loop frequency option `control_loop_hz`; optional per-joint range mapping `source_min_steps`/`source_max_steps`/`command_min_steps`/`command_max_steps` (valid parse, defaults None, invalid/out-of-range or min > max ignored); `joint_entry_by_name`; rejects namespace with slash; rejects joint without id, without name, plain string list; rejects id out of range 0–253, duplicate servo id; accepts non-sequential IDs; `joint_names` property and `servo_id_for_joint_name`; `load_config_from_env` with default path and with env path), command range mapping (`test_command_mapping.py`: `map_position_to_steps` in `command_mapping.py`—identity 0/4095, midpoint, narrow command range, leader max→follower max, degenerate source range, clamping below/above source), startup torque write reliability (`test_bridge_startup_torque.py`: verify/retry logic and failed-servo reporting), register map and read helpers (`test_registers.py`: REGISTER_MAP entries, WRITABLE_REGISTER_NAMES, get_register_entry_by_name, read_all_registers and read_register with mock servo; EPROM vs RAM: `test_eprom_registers_marked_for_runtime_rejection`, `test_ram_registers_accepted_at_runtime` — bridge rejects EPROM writes from ROS set_register), set_servo_id script argparse and exactly-one-servo logic (`test_set_servo_id.py`), calibrate_servos subcommands and parser (`test_calibrate_servos.py`: parser default `--id` 1 for read/write; `list-registers` exits 0 and prints register map JSON; `read` with unknown register / `write` with read-only register / `limits-set` with min > max or out-of-range exit 1; calibrate JSON shape and missing-joint exit; `center` defines the middle and writes symmetric limits, refuses to write limits when the servo does not read ~2048, rejects out-of-range spans). Battery: `test_battery.py` covers `raw_to_volts` (0.1 V units, 0/None rejected), `BatteryMonitor` (one servo per interval round-robin, disabled at interval 0, median of fresh readings, failed/zero reads ignored, stale readings dropped) and `battery_fields` (BatteryState field values with NaN/UNKNOWN); `test_config.py` also covers the `battery_*` fields and defaults. Swerve additions: `test_config.py` also covers per-joint `mode` / `inverted` / `max_velocity_rad_s`, invalid mode rejection, `extra_groups` (parsing, duplicate IDs across groups, duplicate namespaces, `servo_id_for_joint_name` across groups) and `velocity_command_timeout_s`; `test_wheel_mode.py` covers sign-magnitude velocity encode/decode (clamp, inversion, roundtrip), inverted position conversion, sync read with per-servo fallback (`sync_read.py`), the velocity watchdog and record-only-on-successful-write (`velocity_watchdog.py`, so a failed stop is retried), and the incremental one-servo-per-cycle register dump (`register_dump.py`).

Bridge cycle (`test_bridge_cycle.py`, rclpy-free `bridge_cycle.py`): regression test that wheel `goal_speed` is never written to 0 while drive commands keep arriving faster than the loop (the 2026-10 stall: one ROS callback per loop iteration starved drive commands until the 0.3 s watchdog fired); watchdog stops wheels when drive commands cease while steering continues, works per wheel, stays fed by received commands even if a write fails, retries a failed stop; callback draining processes everything pending, is bounded and stops when nothing is ready; remaining-sleep computation; combined JointState with NaN placeholders drives steering and wheels; non-finite velocity/position entries ignored (and do not feed the watchdog); arm position-only messages and separate steer/drive messages still work. Direct command sources: web UI / autonomy gripper targets reach the servo as follower radians (0.0 rad -> step 2048, clamped to the command range), leader and untagged commands keep the exact `source_min/max_steps` mapping (step-for-step identical to a cycle without direct sources over a leader sweep), no direct sources = legacy mapping for every message, joints without a source range unaffected; `test_config.py` covers `direct_command_sources` parsing (default empty, blanks dropped, non-list rejected).

### Per-node tests (uvc_camera)

The **uvc_camera** node has tests under `nodes/bridges/uvc_camera/tests/`. Run from `nodes/bridges/uvc_camera`: `poetry run pytest tests/ -v` (or `poetry run poe test`). `test_config.py` covers env-based config (`get_config`): defaults, env overrides, device as path or index, stripping whitespace and fallback for empty topic/frame_id; `UVC_ROTATE_DEG` and `UVC_MAX_FPS` (`get_max_fps`: unset is no cap, positive number, invalid values raise). `test_frame.py` covers `rotate_frame` and `frame_due` (publish-rate throttle: no cap, first frame, period). Config lives in `config.py` (no ROS/OpenCV deps) for testability.

### Per-node tests (lerobot_teleop)

The **lerobot_teleop** node has tests under `nodes/lerobot_teleop/tests/`. Run from `nodes/lerobot_teleop`: `poetry run pytest tests/ -v` (or `poetry run poe test`). `test_config.py` covers env-based config (`get_config`): defaults, env overrides, empty env fallback. Config lives in `config.py` (no ROS deps) for testability.

### Per-node tests (swerve_drive_controller)

The **swerve_drive_controller** node has tests under `nodes/swerve_drive_controller/tests/`. Run from `nodes/swerve_drive_controller`: `poetry run pytest tests/ -v`. Covers: kinematics (`test_kinematics.py`: wheel_positions, inverse_kinematics straight/zero/sideways, forward_kinematics roundtrip, `forward_kinematics_with_residual` (residual ~0 for consistent wheels, grows with a slipping wheel), `robust_forward_kinematics` (keeps the full solution below the threshold, drops one slipping wheel and matches the other three for each of the four wheels, keeps the full solution when two wheels slip), `odometry_twist_variances` (0.01 + r^2 and 0.01 + (r / hypot(lx, ly))^2), steer_angle_difference, should_zero_drive, normalize_angle; with the real platform geometry: `fold_to_steer_range` (inside limit, backward flip, +-90 deg boundary picks side closer to current), `desaturate_wheel_speeds`, `compute_wheel_commands` forward/backward/strafe/rotate-in-place/stopped-holds-steer/desaturation/IK-FK roundtrip, `wheel_states` requires all joints, `integrate_odometry` straight/rotated/arc, steering-limit hysteresis keeps the current side just past +-90 deg and switches side when far past); config (`test_config.py`: load_config missing/minimal/defaults, defaults match Platform dimensions, motion limits and timeouts parsed, `slip_residual_threshold_mps` default 0.05 and override); control step (`test_control_step.py`, including the odometry residual: a slipping wheel is dropped and the residual is ~0 for consistent wheels).

Control step (`test_control_step.py`, rclpy-free `control.py`): combined command layout (8 joints, steer positions + NaN, drive velocities + NaN); no command for stale, missing or incomplete joint states; forward command; cmd_vel timeout gives zero twist; deadband; steer targets held when stopped; no-propulsion safeguard; odometry integration and reported twist. Coordinated steering side (`test_kinematics.py`, `test_control_step.py`): near +-90 deg all wheels pick the same side (no single-wheel 180 deg swings in a sweep), group-level hysteresis prevents chattering, pure rotation falls back to per-wheel fold within limits, FK roundtrip holds for every output, side choice carried in the controller state. Idle recentering: steering held for under `idle_recenter_s` (default 3 s) after stopping, then targets return to 0 rad; motion resets the timer; 0 disables it (`test_config.py` covers parsing and clamping). `test_config.py` also covers `publish_tf` (default true, explicit false).

### Per-node tests (rf2o_odom_relay)

The **rf2o_odom_relay** node has tests under `nodes/rf2o_odom_relay/tests/`. Run from `nodes/rf2o_odom_relay`: `poetry run pytest tests/ -v`. Covers: pose-difference twist (`test_twist.py`: straight ahead, world motion rotated into the body frame, sideways motion reported as `vy`, yaw-rate wrap across pi, mid-heading rotation during a turn, no twist for zero/negative/too large time steps, diagonal-only covariance); config loading (`test_config.py`: missing file, defaults, overrides).

### Per-node tests (static_tf_publisher)

The **static_tf_publisher** node has tests under `nodes/static_tf_publisher/tests/`. Run from `nodes/static_tf_publisher`: `poetry run pytest tests/ -v`. Covers: config loading (`test_config.py`: missing file, frames list with parent/child and offsets).

### Per-node tests (filter_node)

The **filter_node** node has tests under `nodes/filter_node/tests/`. Run from `nodes/filter_node`: `poetry run pytest tests/ -v` (or `poetry run poe test`). Covers: config loading (`test_config.py`: input/output topic, algorithm, params, joint_names); algorithm registry and Kalman (`test_algorithms.py`: get_algorithm, Kalman create_state/update/predict); output command filling (`test_command.py`: `header.frame_id` carries the command source leader / web_ui / autonomy, positions copied, velocity/effort cleared).

### Per-node tests (test_joint_api)

The **test_joint_api** node has tests under `nodes/test_joint_api/tests/`. Run from `nodes/test_joint_api`: `poetry run pytest tests/ -v` (or `poetry run poe test`). Covers: config (`test_config.py`); GET/POST `/joint-updates` (`test_app.py`: empty GET, single/multiple POST, validation errors, gripper-only POST). Async tests require **pytest-asyncio** (included in the node's Poetry dev deps; if running with system pytest, install it: `pip install pytest-asyncio`). Endpoint use in tests is limited to gripper joints (joint_5, joint_6) for safety. The utility script `scripts/joint_api_client.py` can GET or POST joint updates (see script docstring for examples).

### Per-node tests (topic_scraper_api)

The **topic_scraper_api** node has tests under `nodes/topic_scraper_api/tests/`. Run from `nodes/topic_scraper_api`: `poetry run pytest tests/ -v` (or `poetry run poe test`). Covers:

- config parsing (`test_config.py`: defaults, overrides, type allow-list, observation_rules parsing)
- endpoint mapping (`test_paths.py`: normalize topic, topic->endpoint, endpoint->topic; stream/preview path helpers)
- message serialization (`test_serializer.py`: recursive conversion for ROS-like message objects, time conversion)
- image encoding (`test_image_encoding.py`: is_image_type, CompressedImage jpeg passthrough, raw Image bgr8/mono8 to JPEG, unsupported encoding returns None)
- dynamic subscription bookkeeping (`test_scraper.py`: allow-list handling, add/remove topic subscriptions; image topic metadata and JPEG cache)
- HTTP API behavior (`test_app.py`: `/topics`, `/topics/<topic-path>`, `/rules`, `/rules/<name>`; `/streams`, `/previews` lists; stream/preview HTML pages; `/previews/<topic>/image.jpg` JPEG snapshot; 404 when no image sample for stream/preview image)
- observation rules (`test_observer.py`: RulesObserver empty rules, compare rule produces position delta, missing payload yields None comparison, rules summary)

### Per-node tests (bno055_imu)

The **bno055_imu** node has tests under `nodes/bridges/bno055_imu/tests/`. Run from `nodes/bridges/bno055_imu`: `poetry run pytest tests/ -v` (or `poetry run poe test`). Covers:

- config loading (`test_config.py`: missing/empty file, defaults, explicit topic/frame_id/publish_hz/i2c_bus/i2c_address/covariances, publish_hz clamping, load_config_from_env)
- quaternion and IMU message mapping (`test_imu_msg.py`: `quaternion_wxyz_to_xyzw`, `build_imu_message` units and covariance arrays; build_imu_message tests are skipped when sensor_msgs is not available, e.g. without a ROS environment)
- rolling-window covariance estimator (`test_covariance.py`: `CovarianceEstimator` — returns None below min_samples, correct at min_samples, zero covariance for constant signal, uncorrelated axes produce diagonal matrix, diagonal variance matches known input, rolling window evicts old samples, min_samples < 2 clamped to 2, symmetry of returned 3×3 matrix)
- node validation helpers and startup logic (`test_node.py`: `coerce` — None/float/int/negative; `all_zero` — zeros/non-zero/empty/tolerance; `has_valid_tuple` — None/too-short/None-elements/valid/allow_zeros; `valid_quat` — None/all-zeros/identity/unit-norm/too-large/too-small; `warmup_check` — OSError/None-values/valid-gyro/fallback-acceleration; `_spin_once_safe` — exception swallowing/timeout; `_warmup` — ready-immediately/timeout/rclpy-not-ok/delayed-valid-data; `_create_bno055` — mode correct first verify/retries/stuck/all-addresses-fail)

### Per-node tests (haptic_controller)

The **haptic_controller** node has tests under `nodes/haptic_controller/tests/`. Run from `nodes/haptic_controller`: `poetry run pytest tests/ -v` (or `poetry run poe test`). Covers: config loading (`test_config.py`: missing/empty file, defaults, mode off/resistance/zero_g, invalid mode fallback, resistance_gains including load_release_ratio and activation_debounce_cycles, delay_safety_max_skew_s, load_config_from_env); resistance control law (`test_node.py`: `compute_resistance_target` — no load/zero velocity returns leader_pos, opposes closing when load above deadband, respects max_step_per_cycle; `should_apply_resistance`; `should_apply_resistance_hysteresis` — activation above deadband, stay-active until release threshold). No ROS2/rclpy dependency in tests.

**Gripper-only validation (manual):** After deploying with haptic controller in resistance mode, bench-check: (1) free motion of leader gripper remains easy (no lock); (2) resistance appears only on contact (follower load above deadband, leader closing); (3) no periodic ~1 s pulsing; (4) no oscillation growth when moving slowly. Use `ros2 topic hz` on key topics and verify service health for leader/follower/haptic. Leader gripper tuning: apply `nodes/bridges/feetech_servos/leader_gripper_haptic_profile.json` via `calibrate_servos.py load-config` on the server (see feetech_servos README).

### Per-node tests (master2master)

The **master2master** node has its own test suite under `nodes/master2master/tests/`. Run from `nodes/master2master`: `poetry run pytest tests/ -v` (or `poetry run poe test`). A `conftest.py` provides path setup. Covers:

- config loading and validation (`test_config.py`: `normalize_topic` — adds leading slash, strips trailing slash, idempotent, empty string, root slash; `TopicRule` construction — defaults, topic normalisation, direction in/out, msg_type jointstate and all new types (imu, navsatfix, laserscan, occupancygrid, odometry, posestamped, image, compressedimage, twist), case-insensitive direction/msg_type, invalid direction/msg_type raises `ValidationError`, non-string source raises; `parse_rule_entry` — string entry, full dict, `from`/`to` aliases, dest defaults to source, missing source returns None, empty string returns None, invalid type raises `ConfigError`, invalid direction raises; `load_config_from_dict` — empty dict, valid topics list, `topic_proxy` key alias, empty topics list, skips None entries, non-list topics raises `ConfigError`, realistic multi-rule config, duplicate topic names allowed; `load_config` — missing file returns empty, empty file returns empty, valid file; `validate_relay_rules` — no loops passes, detects dest→source loop, empty list passes)
- relay proxy (`test_proxy.py`: `get_message_class` — string, jointstate, case-insensitive, unknown raises `KeyError`; `get_supported_message_types` — contains all expected types; parametrized new msg_type→mock mapping; `run_all_relays` — creates one pub/sub per rule, multiple rules get separate pubs/subs, relay callback publishes to correct publisher, calls `rclpy.init`/`shutdown`, shutdown called even on exception, rejects relay loop with `ValueError`, rejects unknown msg_type, `shutdown_callback` stops spin loop, node destroyed after run, two-rule callbacks publish to their own publishers)

Mocks rclpy, sensor_msgs, std_msgs, nav_msgs, and geometry_msgs at module level so proxy tests run without a ROS2 environment.

### Per-node tests (steamdeck_ui — frontend)

The **steamdeck_ui** Electron frontend has TypeScript tests under `nodes/steamdeck_ui/tests/`. Run from `nodes/steamdeck_ui` with the project's JS test runner (e.g. `npx jest` or `npm test`). Covers:

- config loading (`test_config.test.ts`: `loadConfig` — loads valid config with bridge host/port and camera tab, throws on nonexistent file, throws when `bridge.port` is missing, parses sensor_graph tab with topics/fields, missing `overlays` defaults to empty array)
- field extraction and formatting (`test_field_extract.test.ts`: `parsePath` — simple key, dot notation, array index, deep path, array-only index; `extractField` — simple nested field, array index, deep nested, missing field returns null, empty object returns null, empty path returns null, out-of-bounds returns null, non-numeric field returns null; `formatValue` — `.2f`/`.6f`/`.0f` fixed-point format, no format returns string, unit appended, unit without format)

### Per-node tests (steamdeck_ui — bridge)

The **steamdeck_ui** Python bridge has tests under `nodes/steamdeck_ui/bridge/tests/`. Run from `nodes/steamdeck_ui/bridge`: `poetry run pytest tests/ -v` (or `poetry run poe test`). Covers:

- config loading and validation (`test_config.py`: `BridgeConfig` defaults (host, port, ros_static_peers), custom port, `ros_domain_id` int coerced to str; `TabConfig` — valid camera tab, invalid type raises; `AppConfig` — empty config valid, `all_subscribed_topics` for camera and sensor_graph tabs, `publish_topics` includes nav goal_topic, no duplicate topics; `load_config` — valid file, missing file raises `FileNotFoundError`, empty YAML returns defaults)
- message serialization (`test_msg_serializer.py`: `_bytes_per_pixel` — rgb8/mono8/rgba8/mono16/rgb16; `_serialize_value` — `array.array` to list, bytes to list, primitives passthrough, nested list; `extractField_from_dict` — simple key, nested dot notation, array index, missing key returns None, non-numeric returns None)

### Per-node tests (web_ui)

The **web_ui** node has tests under `nodes/web_ui/tests/`. Run from `nodes/web_ui`: `poetry run pytest tests/ -v` (or `poetry run poe test`). A `conftest.py` installs ROS2 module stubs (rclpy, sensor_msgs, nav_msgs, geometry_msgs) and provides `config_yaml` and `urdf_dir` fixtures. Covers:

- config loading (`test_config.py`: minimal config defaults, http_port override, missing file raises `FileNotFoundError`, `all_subscribed_topics` for camera/nav/overlay tabs, `load_config` from env var `WEB_UI_CONFIG`, `publish_topics` includes goal_topic, `rgbd_camera` tab type valid with color/depth/camera_info topics, RGBD topics included in `all_subscribed_topics`, shipped default config has an `Overview` `camera` tab on `/overview_camera/image_raw/compressed` and no RGBD tab)
- bridge dirty-flag store (`test_bridge.py`: the Overview camera topic resolves to a `CompressedImage` subscription; `flush_dirty` returns dirty topics, clears after flush, only returns dirty entries; `publish_dict` rejects non-allowlisted topics with warning)
- message serialization (`test_msg_serializer.py`: `msg_to_dict` — Imu message conversion; `extract_field_from_dict` — simple key, array index, missing returns None, out-of-bounds returns None; depth image serialization — 16UC1 returns expected keys, downscaled raw bytes length, depth values preserved, zero-depth preserved; `CameraInfo` returns fx/fy/cx/cy/width/height; color image on RGBD topic includes `color_small_b64`, non-RGBD topic omits it; stereo topics: `/stereo/left/image_rect` rgb8 and raw bgr8 at 320x240 give JPEG + `color_small_b64`, 16UC1 depth at 320x240 gives preview and 160x120 grid)
- HTTP server routes (`test_server.py`: `/api/config` returns config JSON, `/api/urdf/status` lists URDF files, `/api/urdf/<file>` serves URDF, path traversal blocked, security headers present, static fallback serves index.html)
- Battery (`test_battery.py`): `battery` config (defaults, absent = off, `resume_cell_v >= cutoff_cell_v`, `cells >= 1`, in `/api/config`, topic subscribed with role `battery`), `serialize_battery`, `BatteryGuard` (unknown without reading, cut-off below 8.4 V, hysteresis release above 8.7 V, stale reading not blocked, invalid voltage ignored, thread safety, rejection message), bridge battery callback (payload with guard state, NaN dropped), server rejection of WS `publish` (error frame) and of map save/reset and arm home/set home with 503, `nav/stop` still allowed, no blocking without battery config, with a good or unknown battery.
- Battery frontend (`frontend/src/battery/batteryStatus.test.ts`, vitest): chip level/colour/label (unknown, green, amber, red incl. backend flag), stale after `stale_s`, cut-off banner text, WebSocket frame parsing (envelope, error frame, malformed), 503 message surfaced by `parseActionResult`.
- URDF directory scanner (`test_urdf_scanner.py`: `scan_urdf_directory` finds robot.urdf, parses links/joints, detects missing mesh files, empty directory returns empty list)

- Map tab backend (`test_map_nav.py`): `map_nav` tab type, defaults (`/map`, `/plan`, `/optimal_trajectory`, `/goal_pose`, `map`, `base_link`, save path) and overrides, empty frame rejected, subscribed topics and publish allowlist, topic roles; OccupancyGrid -> PNG grey levels, thresholds, row orientation (y up), metadata, size mismatch; Path serialization and downsampling to 500 points keeping the ends, paths in another frame transformed to `map` (dropped without TF); goal PoseStamped serialization; quaternion/transform helpers; latched (reliable + transient_local) map subscription; robot pose from TF published when available and omitted otherwise; typed `_dict_to_ros_msg` (ints vs floats, errors raised); goal publishing fills stamp and frame; `POST /api/map/save` success, slam failure, service unavailable, timeout, unknown tab, no bridge; cached snapshot sent on WS connect; default config has the map tab. Reset/Stop: `map_reset_service` and `navigate_action` defaults; `POST /api/map/reset` (slam_toolbox `/slam_toolbox/reset`) success, non-success result, unavailable, timeout, unknown tab, no bridge, cached map cleared only on success (cleared-map event); `POST /api/nav/stop` cancels all `NavigateToPose` goals (zero goal id and stamp), unavailable, timeout, cached goal cleared on success (cleared-goal event). Robot footprint: `footprint_topic` default and `footprint` role, PolygonStamped serialized to points and re-expressed in the map frame.
- Map tab frontend (`frontend/src/map/mapMath.test.ts` and `mapActions.test.ts`, vitest, `npm test`): Reset/Stop action helpers, front edge of the footprint polygon (`frontEdgeIndex`) and world <-> screen (y up), map image placement with origin and yaw, pan, zoom about the cursor with clamping, pinch zoom/pan, fit and centre view, yaw/quaternion helpers and angle normalization, goal heading from drag or plain click (towards goal, robot heading fallback), scale-bar length choice.
- POI backend (`test_poi.py`): `poi_list_topic` / `poi_command_topic` / `poi_result_topic` defaults and roles (`poi_list`, `poi_result`), `/poi/list` subscribed latched (reliable + transient_local) and parsed by `serialize_poi_list` (malformed payloads rejected), `BridgeNode.poi_request_async` publishes the command with a fresh `request_id` and resolves on the matching `/poi/result` (None when no poi_store subscribes, cancelled requests forgotten), `POST /api/poi` (200 with the store result, 400 on store rejection, 503 without poi_store or bridge, 504 on timeout, 422 for bad bodies, 404 unknown tab) and that it is NOT blocked by the battery cut-off.
- POI frontend (`frontend/src/poi/*.test.ts`, vitest): geometry and hit testing (`geometry.test.ts`: area, centroid, point-in-polygon, hit priority), editor state (`editor.test.ts`: click detection, area drafting, new-POI fields, drag previews and update commands, optimistic override), styling and list helpers (`style.test.ts`: colour by status, creator marker, `/poi/list` validation, sort, relative time), and the `POST /api/poi` client (`api.test.ts`). `map3d/layers.test.ts` covers the `pois` layer toggle.

### Per-node tests (claude_agent)

The **claude_agent** node has tests under `nodes/claude_agent/tests/` (no network, no real Claude calls; rclpy is stubbed in
`conftest.py`, the Agent SDK dataclasses are built directly). Run from `nodes/claude_agent`: `poetry run pytest tests -q`.
Covers:

- config (`test_config.py`): defaults (opus, hard maxima rw 150 / turns 200, no `max_ro_cap` / `max_phase_ro_cap`, SDK `max_turns` = turn cap + margin, MCP URL/token file, port 18300, history 500, thumbnail 480, `/robot_events` topic, 50 events, 2 s debounce), the removed `max_turns` / `effector_call_cap` keys rejected,
  tool lists disjoint (stop can never be an effector), invalid numbers and unknown keys rejected, YAML loading,
  `CLAUDE_AGENT_CONFIG` lookup, MCP token read (env-file line or bare token, missing/empty refused, token never in the error)
- system prompt (`test_prompt.py`): the phase-plan workflow (split into phases first, the four planning tools, the tomato example, per-phase rw/turn guidance ranges, no ro budget and unlimited sensor calls, generous rw/turn caps (double the estimate, raise early), the grasping guidance (several viewpoints by changing the roll incl. directly above, `pixel_to_ground`, average within 1 cm, centre not edge, correct by the observed offset, NOTES.md), retry budgeting (room for retries, raise the budget before it runs out or add a retry phase), fixed vs moving jaw and `object_width_m`, the agent chooses the grasp `wrist_roll`, camera views by roll, the rolling protocol (half open, arm lifted), `surface_height_m` and below-floor reach, fast arm by default, phase and instruction maxima from the config, explicit honest `complete_phase`, once-per-instruction revision and once-per-phase raise, no waiting for approval, no old fixed caps), lists every tool by kind, body awareness / spatial perception / memory-POI-calibration sections, safety rules, persona,
  `system_prompt_extra` appended
- plan (`test_budget.py`): `PlanTracker` validation (complexity enum, 1 to 12 phases, integer caps, rw 0 allowed, names and goals, a stray `ro_cap` ignored), clamping to the
  per-phase maxima with notes, rejection when the phase caps summed exceed an instruction maximum, the phase lifecycle (first phase active,
  `complete_phase` outcomes and next activation, plan end), sensor calls never refused (no plan, plan finished, turn cap used up) but counted, rw/turn counting and exact exhausted reasons, the once-per-phase raise
  (rationale, no lowering, limited to the remaining instruction room), the single `revise_plan` (closed phases kept, active one closed as failed,
  budget counts what closed phases used), the phase turn note, reset, callbacks, and the four SDK tools (schemas, texts, errors as error results)
- robot events (`test_robot_events.py`): `/robot_events` JSON parsing and normalization, idle events emitted but never interrupting, bounded
  history, non-critical events not interrupting, a critical event interrupting the running step and continuing the same instruction
  with the follow-up message (no robot stop, plan and turns carried over), the 2 s debounce, thread-safe hand-over from the rclpy thread,
  garbage and unbound-loop events dropped, a user stop winning over a pending follow-up
- tools (`test_tools.py`): classification (sensor / effector / uncapped / plan, unknown robot tool counted as rw, non-robot unclassified),
  built-in tool deny list, effector tools denied until a plan is set while sensors stay allowed (notes tools, the planning tools, `stop` and the
  control tools too), sensors counted but never denied and effectors denied at the rw cap with the exact messages, the phase turn cap, denial after the last phase, reset,
  everything outside `mcp__robot__*` denied, count and denial callbacks
- events (`test_events.py`): ring buffer and sequence numbers, subscribers, normalization of assistant text / tool calls /
  tool results from SDK objects, image thumbnails (size, no upscaling, RGBA, undecodable), 4000-char truncation flag,
  turn_end statuses (done, interrupted, max_turns, error) and auth-error detection
- session log (`test_session_log.py`): JSONL append, load on start (seq continues, last `history_size` events in RAM), corrupt
  lines skipped, paging with `before_seq` / `has_more` (RAM and disk), reset deletes file and restarts seq, rotation drops
  the oldest half, unwritable path tolerated; the API paging, WS history and reset live in `test_api.py`
- file tool sandbox (`test_tools.py`): Read/Write/Edit/Glob/Grep allowed only inside the workdir (relative and absolute,
  default path), denied for absolute / `../` / `~` / symlink escapes and escaping glob patterns, kind `notes`, never counted,
  every other built-in still denied; prompt (`test_prompt.py`) working method, notes, hardware facts from config, reach vs URDF
- env (`test_env.py`): `ANTHROPIC_API_KEY` / `ANTHROPIC_AUTH_TOKEN` removed from the child env, `DISABLE_AUTOUPDATER=1`, input untouched
- runner (`test_runner.py`): SDK options (robot HTTP MCP server with bearer header, `tools=[]`, no allow rules, `can_use_tool`), event
  flow with a fake client, token never in events, busy refusal, interrupt, plan and counters reset per instruction, `plan` / `phase_started` /
  `phase_completed` / `plan_revised` and `state` events (ro/rw/turns/plan), the instruction turn maximum (interrupt, robot stop, `turn_cap` status), the phase turn cap note (follow-up, no stop), denial events,
  auth error, session failure and missing token become error events, reset starts a new session
- API (`test_api.py`): `/api/state` (plan, active phase, usage, hard and per-phase maxima), `/api/history`, `/api/message` (202 / 409 busy / 400 empty or invalid), `/api/stop`,
  `/api/reset` (409 while busy), `/ws/events` history on connect then live events, shutdown hook
- entry point (`test_main.py`): loopback bind and port from config, the `/robot_events` subscription (volatile QoS) and its callback, ROS2 logger, API key removed from the process env, invalid config exits 1

### Per-node tests (mcp_server)

The **mcp_server** node has tests under `nodes/mcp_server/tests/` (no ROS needed: rclpy is imported only by
`ros_iface.py` and `__main__.py`). Run from `nodes/mcp_server`: `poetry run pytest tests -q` (or `poetry run poe test`).
`fakes.py` provides a simulated follower arm backend with a fake clock. Covers:

- config (`test_config.py`): defaults (0.0.0.0:18200 `/mcp`, topics, conservative limits, timeouts), repo-relative URDF
  path resolution, YAML overrides, empty file, unknown keys rejected, hard caps (0.25 m/s, speed scale 0.5, 1024 px),
  `MCP_SERVER_CONFIG` lookup, `MCP_SERVER_TOKEN` refused when missing/blank/short and stripped otherwise, settle
  tolerance default 0.08 and validated between the converge tolerance and the tracking abort (defaults 0.25 rad + 0.25 s x velocity); arm velocity default 1.0
  rad/s with the 1.5 cap, roll guard defaults (0.1 / 0.8) and validation, `arm.jaw_open_axis` default and normalisation
  (zero and wrong-length vectors rejected), `arm.joint_limit_overrides_rad` default empty with name and lower < upper checks
- trajectory (`test_trajectory.py`): quintic blend endpoints/monotonicity, limit clamping with margin, duration from the
  quintic peak velocity, sampled trajectory ends exactly at the goal without exceeding the velocity cap, tracking error
- staleness (`test_staleness.py`), home store (`test_home_store.py`: missing file, atomic round trip, corrupt file,
  non-finite values), geometry (`test_geometry.py`: yaw/quaternion, relative goals, twist clamping)
- IK (`test_ik.py`): URDF joint limits, 5-DOF chain, FK at zero pose and pitch sign, IK forward/inverse round trip with
  pitch and position only, wrist_roll kept from the seed, solutions inside limits minus margin, unreachable targets;
  per-call `extra_offset` (forward shifts by the rotated vector, the shared tool offset is not mutated, `grasp_offset`),
  inverse with a width puts the tool point half a width from the centre along the opening direction at several rolls,
  limit overrides replace the URDF limits (IK bounds, `within_limits`), a target below the floor (x 0.2, z -0.25) is
  unreachable with the URDF limits and reachable with shoulder_lift upper 2.6, invalid overrides rejected
- perception (`test_perception.py`): 8 scan sectors in base_link (lidar mounted backwards), invalid returns ignored, map
  stats, map PNG crop size, image fitting, JPEG encoding, `sensor_msgs/Image` encodings, JPEG pass-through/downscale
- top-down view (`test_topdown.py`): base_link -> pixel transform (robot-up: forward is image up, left is image left),
  map -> base_link rotation by the robot yaw, pixel-level checks that the footprint outline (front/rear/side edges and
  corners, nothing outside) and the heading arrow (up from the centre, none to the rear) land where the 470 x 386 mm
  frame is, lidar points placed in base_link, the map crop turning with the robot yaw, grid sampling with a rotated and
  offset grid frame (local costmap in `odom`), costmap tint only on non-free cells, plan/object placement, POI points
  and areas drawn, missing layers listed with a reason and never drawn, map-frame layers missing without a pose,
  PNG round trip, scan points into base_link, OccupancyGrid field conversion, point transform
- object memory (`test_object_memory.py`): new object fields, same-label merge within 0.25 m (case-insensitive,
  nearest candidate, weighted average, times_seen, max confidence, note kept unless given), different label or too far
  = new object, atomic persistence (no temp file, failed `os.replace` keeps the old file), corrupt file moved aside,
  forget, input validation, distance/bearing from the robot heading, label / near-point filters, no pose = no distance
- look_around (`test_look_around.py`): equal-step plan covering 360 deg and its limits, clearance = footprint
  circumscribed radius + margin, refusal on a close obstacle or a missing lidar (robot not moved), the rotate-capture
  loop (3 rotations between 4 stops plus the closing step, camera and lidar at every stop, expected vs achieved
  rotation and heading error), interrupted / failed steps end the sequence with `interrupted_by` and no return
  rotation, `stop` between steps, an obstacle appearing mid-turn, a camera failure recorded (not fabricated), montage
- POI client (`test_poi_client.py`): request published with op, poi and a fresh `request_id`; the result is matched by
  id (other ids ignored, answer from another thread), timeout, store not running (nothing published), store rejection,
  malformed results dropped, `/poi/list` parsing
- perception tools (`test_perception_tools.py`): the nine new tools registered and classified (`look_around` in
  `MOTION_TOOLS`, the rest always allowed), `get_topdown_view` PNG + metadata (pose, scale, layers present/missing,
  data ages), layer selection and validation, POI/object layers, object tools round trip and merge, POI tools
  (point defaults to the robot position, area polygon, `created_by` agent, argument errors, store not running),
  `list_pois` distance/bearing/status/near, `look_around` (montage + top-down images, summary, config default
  captures, argument validation, refusal does not move)
- arm controller (`test_arm.py`): lease acquire/release, streamed motion with implicit acquire and velocity cap, speed
  scale, clamping, argument validation, abort + hold on stale feedback / tracking error / stop (tracking limit grows
  with the motion's velocity: `tracking_limit`, a 0.4 rad lag passes at full speed and aborts at 0.2 rad/s), no hold after filter_node
  switches source, convergence timeout, Cartesian moves (unreachable reported without motion), gripper open fraction
  and close-until-effort, home/set_home, keepalive and lease loss, state with stale data omitted; gripper closed default
  is a follower joint position; sag ratchet (fake backend `sag`): a small steady-state error settles as `converged`
  with `residual_error` before the timeout and keeps the target commanded (also for home), unnamed joints keep the
  last commanded target over repeated motions, trajectories start at the commanded pose, Cartesian moves keep the
  commanded gripper / wrist_roll, a new lease falls back to the measured pose, larger or still-moving errors time out
  and hold the measured pose, tracking aborts still hold the measured pose; bounded residual hold: target held within
  `arm_settle_hold_s`, relaxed to the measured pose after it (only joints still off target), the next motion starts
  from the intended target, intent and pending relax cleared on release / drop_lease / lease loss; grasp from stall:
  a jaw stalling before closed (close_until_effort and open_fraction=0) reports `grasped` and holds stall + squeeze
  (also after the hold window), the squeeze never passes closed, a partial open_fraction stall is not a grasp; wrist roll
  guard (refused with a wide open gripper measured or targeted in the same call, nothing published and no lease taken,
  allowed at half open or for a change under the minimum, thresholds from config); `move_cartesian` with `wrist_roll`
  (replaces the current roll, clamped and reported in `clamped`, guard applies) and `object_width_m` (centre reached with the
  shift, `grasp_shift` reported, widths 0 / negative / 0.09 / NaN rejected); limit overrides widen the clamp of `move_joints`
  and make a below-floor Cartesian target reachable
- drive (`test_base_motion.py`): rate, clamping, duration cap, abort always ends with a zero twist
- tools (`test_tools.py`): all 32 tools registered with real descriptions (arm motion tools explain `residual_error`
  and the commanded hold), `speed_scale` descriptions derived from `arm_max_joint_velocity_rps` (no hardcoded 0.5 rad/s),
  `move_arm_cartesian` accepting `wrist_roll` / `object_width_m` (bounds, `grasp_shift` in the result, description phrases),
  the roll guard as a tool error, structured outputs, camera JPEG + stamp,
  argument validation, robot errors as tool errors, map PNG, arm tool round trip, bearer-token auth on the Streamable
  HTTP app (401 without/with a wrong token, 200 with the right one) and the configured path
- battery gate (`test_battery_gate.py`): `MOTION_TOOLS` and `ALWAYS_ALLOWED_TOOLS` partition all tools; each motion tool
  is refused in cut-off with the exact message, a logged warning and no robot call; `stop`, sensor, state and
  acquire/release tools still work in cut-off; motion works with a good or unknown battery, without a guard and after
  recovery; optional `battery` config section (absent = off, validated)
- monitor (`test_monitor.py`): servo overheat warning/critical with hysteresis, debounce and `_cleared` info events, status
  error bits decoded (critical), battery low/cut-off from the shared guard, IMU bump (baseline removed, debounce,
  warning/critical) and tilt, wheel slip from the swerve twist covariance (parked = no residual), base stall (needs a
  running base motion, commanded speed and fresh ~0 odometry for 1 s), collision STOP only during a base motion (also
  latched), human takeover only while the lease is held, CPU temperature, digest cursor/cap/vitals line, `MotionWatch`
  (only relevant critical events after creation, live battery cut-off), body state nulls and notes, last 10 events,
  failing sink tolerated, threshold validation
- camera config (`test_camera_config.py`): cameras default to not calibrated with parent frames `gripper_link` / `base_link`,
  intrinsics source rules (calibration file XOR hfov with width and height), mount parent frame adopted and checked, front
  camera restricted to `base_link`, unknown camera keys rejected, `arm.base_in_base_link` and reach ordering
- camera scene (`test_camera_scene.py`): intrinsics loading (hfov approximate, calibration yaml), not-calibrated error text,
  synthetic front camera with a known mount (analytic centre-pixel floor point, project/ground round trip, sky pixels),
  gripper camera pose from measured joints, frame conversions with and without the arm offset, map position from the
  robot yaw, `pixel_to_ground` report fields and pixel errors; `surface_height_m` (a pixel hits the plane at floor + height
  at the expected point, below the floor too, the report states the height used)
- camera overlays (`test_camera_overlay.py`): vectorised projection equals `project_raw`, grid polylines at the step
  and on the floor, behind-camera samples dropped, spaced metric labels, reach circle, lidar dots counted, candidate grid
  (whole image, region, cap, region validation) and drawing
- camera calibration (`test_camera_calib.py`): sample store JSON round trip, counts per camera, clear, parent frame change
  refused, non-finite / unwritable / corrupt files, the solver recovers a known mount from synthetic samples, minimum sample
  count, YAML snippet shape
- camera tools (`test_camera_tools.py`): the seven tools and descriptions, documented not-calibrated errors,
  `pixel_to_ground` (front/gripper, arm offset, missing map pose, stale joints, sky), annotated images (all overlays,
  notes for overlays that cannot be drawn, validation), candidate points (table vs projection, sky skipped, region, last
  10 sets, stored values with moved flags), `surface_height_m` for `pixel_to_ground` and `mark_candidate_points` (raised
  plane hit, validation, stored and resolved with the set), capture/solve/clear round trip
- IK link frames (`test_ik.py`): `link_frame` for base, `gripper_link`, `gripper_frame_link` and rejected off-chain links
- digest + body state (`test_digest.py`): every tool result carries `robot_events_since_last_call` and `vitals` (text,
  structured content, `_meta` for image tools, appended to tool errors), events reported once, events raised during the
  call included, a tool registered by a later module gets it for free, `get_body_state` content/nulls, never refused in
  cut-off
- early return (`test_early_return.py`): arm motions end `interrupted` with `interrupted_by` on overheat / servo error /
  battery cut-off mid-motion / human takeover (lease dropped, nothing published against the human) and hold the
  measured pose, warnings do not interrupt, tracking abort is a `stall` event, gripper and `arm_home` interrupts,
  expected/achieved/duration and Cartesian tool poses; drive interrupt (zero twist, status, duration), twist
  integration and relative pose; navigation via a fake Nav port (success, interrupt cancels goal and zeroes the base,
  stop, timeout, rejected, unavailable, real monitor collision stop)
- rclpy isolation (`test_rclpy_isolation.py`): only `ros_iface.py` / `__main__.py` import ROS packages

### Per-node tests (gps_rtk)

The **gps_rtk** node has tests under `nodes/bridges/gps_rtk/tests/`. Run from `nodes/bridges/gps_rtk`: `poetry run pytest tests/ -v` (or `poetry run poe test`). Covers: config loading and validation (`test_config.py`: minimal base/rover, rover with rtcm_server_host, invalid mode rejected, load_config from file/missing/empty); NMEA GGA parsing (`test_nmea_parser.py`: lat/lon N/S/E/W, altitude, fix quality, full sentence, RTK fixed quality 4, quality-to-NavSatStatus mapping); serial stream handling (`test_serial_handler.py`: NMEA checksum and append_checksum_if_missing, RTCM3 length parsing, CRC24Q, valid RTCM3 frame build/validation, parser emits NMEA with valid checksum, ignores invalid NMEA, discards unknown bytes).

---

**Maintenance:** Keep this README up to date when adding, removing, or changing tests. Document each new test file and each test (or test group) briefly so the test suite remains easy to navigate.

### test_rplidar_scan_watch.py

Decision logic of the RPLidar scan watchdog (`nodes/bridges/rplidar_a1/scan_watch.py`): no restart during the startup grace, restart when no scan arrives after it, no restart while scans keep arriving, restart when scans stop for the silence timeout, and the silence timeout applies once the first scan has arrived.

### test_client_cpu_load.py

Client CPU load trims (load about 13 on 4 cores made the EKF miss its rate and the map pose go stale).

| Test | Description |
|------|-------------|
| `test_master2master_is_disabled` | `master2master` stays `present: true` but `enabled: false` (its `/controller/*` relays, the gripper camera included, served only the legacy Steam Deck UI). |
| `test_gripper_camera_publish_rate_is_capped` | `gripper_uvc_camera` env has `UVC_MAX_FPS=10`. |

### test_dds_discovery_config.py

Static checks of the DDS discovery setup in Ansible: the shared `ros2_dds_env` is `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` and the discovery-server variables are gone; no node env overrides discovery (`ROS_DISCOVERY_SERVER`, `ROS_SUPER_CLIENT`, `ROS_LOCALHOST_ONLY`, `ROS_AUTOMATIC_DISCOVERY_RANGE`) and the only `ROS_STATIC_PEERS` is the client `master2master` pointing at the server (server nodes have none); the former `fastdds_discovery_server` node is `present: false` / `enabled: false` on both hosts; the unit template renders the shared env before node env with no discovery-server ordering; Steam Deck uses the client as static peer; `/etc/profile.d/ros2_dds.sh` sets localhost discovery for shells; `--all` deploys include `tasks/ros_packages_sync.yml` after the repo sync (upgrade all ros-jazzy packages together, restart running nodes if anything was upgraded); the client and server deploy playbooks never stop the stack up front (`stop_ros_nodes.yml` is gone) and finally include `tasks/start_ros_nodes.yml` (queued nodes restarted and stopped ones started one by one, `ros2_node_start_interval_s` = 2), and web_ui is deployed first in the client `--all` playbook; secondary ethernet ports are `optional: true` in the rendered netplan, and `--all` deploys install the wait-online drop-in (`--any --timeout=30`); every `env:` key is a list (an empty `env:` parses as null and breaks deploys); logind keeps the node user's shared memory (`RemoveIPC=no`) and running `ros2-*` services restart once when that is first applied.

### test_nav_stack_config.py

Static checks of the mapping/navigation stack from the repo files (YAML via `yaml.safe_load`, launch files via `ast`; no ROS needed).

| Test (group) | Description |
|------|-------------|
| `test_rplidar_default_frame_matches_static_tf_child` | RPLidar launch default `frame_id` (`DEFAULT_FRAME_ID`, env `RPLIDAR_FRAME_ID`) is `laser_frame`, a static TF child of `base_link`. |
| `test_lidar_static_tf_mounted_backwards` | Static TF base_link -> laser_frame is at (0.15, 0.04, 0.20) with yaw = pi: the RPLidar is mounted rotated 180 deg (verified on the robot by driving forward). |
| `test_laser_filter_box_covers_footprint` | laser_filter box filter (base_link, not inverted) covers the outer footprint with at most 5 cm margin. |
| `test_laser_filter_ansible_wiring` | laser_filter node type (apt `ros-jazzy-laser-filters`, `scan_to_scan_filter_chain` with the repo params, `/scan` -> `/scan_filtered`), enabled node entry, per-node playbook and inclusion in `deploy_nodes_client.yml`. |
| `test_node_apt_install_refreshes_stale_cache` | The ros2_node_deploy apt install refreshes the package index (`update_cache: true`, `cache_valid_time: 3600`) so new packages do not 404 on a stale cache. |
| `test_rplidar_runs_under_scan_supervisor` | rplidar_a1 is launched through `scan_supervisor.py` (runs the launch file, exits 1 for a systemd restart when `/scan` never starts or goes silent). |
| `test_web_ui_frontend_build_runs_at_lowest_priority` | The web_ui `npm ci` / `npm run build` deploy steps run in a transient systemd scope capped to one core (`CPUQuota=100%`, `IOWeight=10`) under `nice -n 19 ionice -c 3` (the build overheated and froze the Pi next to the running ROS stack). |
| `test_restart_handler_skips_disabled_nodes` | The ros2_node_deploy "Restart ROS2 node" handler is conditioned on the node being enabled (a disabled node was stopped and then restarted by the change handler). |
| `test_ekf_repo_config_matches_ansible_block` | `nodes/robot_localization_ekf/config/ekf.yaml` equals the Ansible `config: \|` block. |
| `test_ekf_frames_inputs_and_tf` | Both EKF configs: 2D, `publish_tf`, odom/base_link frames, `/odom` fuses vx/vy/vyaw, `/imu/data` fuses yaw rate (absolute yaw only with `imu0_relative: true`). |
| `test_ekf_launch_uses_repo_launch_and_deployed_config` | Ansible launches the repo EKF launch file, which reads `ROBOT_LOCALIZATION_EKF_CONFIG` (default the deployed config path). |
| `test_swerve_controller_does_not_publish_odom_tf` | swerve_controller config sets `publish_tf: false` (EKF owns `odom -> base_link`). |
| `test_slam_params_frames_and_topics` | slam_toolbox params: base_link/odom/map frames, `/scan_filtered`, mapping mode, 0.05 m resolution, no hard-coded `map_file_name`. |
| `test_slam_launch_starts_async_lifecycle_node` | Launch starts `async_slam_toolbox_node` as a lifecycle node with configure + activate transitions. |
| `test_slam_launch_resumes_only_when_posegraph_exists` | `map_resume_parameters` (compiled out of the launch file) returns `map_file_name` + `map_start_at_dock` only when `<base>.posegraph` exists; otherwise it returns `map_file_name: ""` so a `map_file_name` from a params file never makes slam_toolbox load a missing file. |
| `test_slam_launch_appends_override_params_only_when_present` | `params_files` returns the repo `slam_params.yaml` first and appends the deployed config only when it exists and is non-empty (ROS params files: later wins). |
| `test_slam_launch_override_defaults_and_env` | Launch reads the override path from `SLAM_TOOLBOX_CONFIG`, defaulting to `/etc/ros2/slam_toolbox/config.yaml`. |
| `test_slam_launch_map_base_from_configuration` | `configured_map_base` takes `map_file_name` from the last params file that sets it (falls back to `/var/lib/ros2/maps/slam_map`); an explicit `SLAM_TOOLBOX_MAP_BASE` value wins over the files. |
| `test_slam_toolbox_ansible_wiring` | Node type defaults (native, apt package, repo launch, budget, `config_path: /etc/ros2/slam_toolbox`, env `SLAM_TOOLBOX_CONFIG`) and an enabled `ros2_nodes` entry. |
| `test_slam_toolbox_ansible_config_overrides` | The `ros2_nodes` slam_toolbox `config: \|` block parses as YAML with `map_file_name` (`/var/lib/ros2/maps/slam_map`), `min_laser_range` 0.15 and `max_laser_range` 12.0, only known slam_toolbox params, and the same map path as the web_ui Map tab `map_save_path`. |
| `test_slam_playbooks_deploy_node_and_create_maps_dir` | `deploy_nodes_client.yml` deploys slam_toolbox and create `/var/lib/ros2/maps` (owner `ansible_user`, 0755). |
| `test_nav2_launch_passes_repo_params_without_localization` | Nav2 launch passes the repo `params_file`, keeps `use_localization:=False`. |
| `test_nav2_has_every_server_section` | `nav2_params.yaml` has a section for every server started by Jazzy `navigation_launch.py`. |
| `test_nav2_frames` | bt_navigator, costmaps, collision_monitor, behavior_server, docking_server and route_server frames/topics. |
| `test_nav2_costmap_layers_and_footprint` | Both costmaps: 370 x 286 mm planner footprint (outer frame minus the 5 cm wheel margin), obstacle layer on `/scan`, inflation; global static layer on `/map` (transient local). |
| `test_nav2_mppi_omni_with_swerve_limits` | MPPI Omni, vx +-0.25, vy 0.25, wz 0.5, `visualize: true` (local plan on `/optimal_trajectory`). |
| `test_nav2_planner_navfn_allows_unknown` | NavFn with `allow_unknown: true`. |
| `test_nav2_velocity_smoother_matches_swerve` | Smoother limits `[0.25, 0.25, 0.5]`, odom `/odometry/filtered`. |
| `test_nav2_collision_monitor_uses_scan` | Valid polygon(s) and a `/scan` observation source. |
| `test_nav2_collision_monitor_stopbox_enabled_on_filtered_scan` | collision_monitor stays in the `cmd_vel_smoothed -> cmd_vel` chain with its core keys; `StopBox` (stop polygon, min 4 points) is enabled on the footprint-filtered scan. |
| `test_nav2_goal_tolerance_tight_enough_for_swerve` | Goal checker tolerances 0.01 m / 0.035 rad (1 cm / 2 deg), stateful: the latch only holds because the BT no longer replans periodically. |
| `test_nav2_replans_only_when_path_invalid` | `bt_navigator` uses the "replanning only if path becomes invalid" BT: periodic replans reset the goal checker latch and the rotation shim position check. |
| `test_nav2_progress_checker_counts_rotation` | `progress_checker` is `PoseProgressChecker` (counts rotation), movement radius <= 0.05 m and a required angle set. |
| `test_costmaps_add_no_margin_beyond_given_dimensions` | Both costmaps: `footprint_padding` 0 and inflation only to the planner footprint's circumscribed radius (0.24 m, `cost_scaling_factor` >= 10). |
| `test_safety_boxes_hug_the_given_dimensions` | laser_filter self box is footprint + 1 cm and the collision-monitor StopBox footprint + 2 cm, strictly outside the filter box so obstacles stay visible to it. |
| `test_costmaps_wait_for_odom_through_a_gradual_restart` | Both costmaps: `initial_transform_timeout` >= 300 s, so a deploy that restarts nav2 before the gradual ramp brings up the servos and EKF (odom TF) does not abort the bringup. |
| `test_ekf_rate_fits_the_cpu_budget` | EKF `frequency` 30 Hz: at 50 Hz it missed its rate on the loaded Pi and its odom TF lagged the scans. |
| `test_slam_waits_for_a_lagging_odom_transform` | slam_toolbox `transform_timeout` >= 0.5 s so scans are not dropped when the odom TF lags. |
| `test_nav2_rotates_toward_path_first_and_keeps_front_leading` | `FollowPath` is the RotationShimController around MPPI (turn to face the path first, threshold 0.3-1.0 rad, rotation speed within `wz_max`, final rotation to the goal heading, MPPI GoalAngleCritic disabled so only the shim turns to the goal heading, GoalCritic weight >= 8, `max_angular_accel` <= 0.5, `rotate_to_heading_once`) and MPPI's PathAngleCritic prefers forward driving (mode 0): the lidar is partly covered at the back and right. |
| `test_nav2_readme_documents_collision_monitor_validation` | Nav2 README documents StopBox `enabled: true` on `/scan_filtered` and the live `collision_monitor_state` validation procedure. |
| `test_nav2_docking_server_configures_without_docks` | Non-empty `dock_plugins`, no docks. |
| `test_nav2_readme_documents_plan_topics` | Nav2 README names `/plan` and `/optimal_trajectory`. |
| `test_web_ui_has_map_nav_tab` | web_ui config has the `map` tab of type `map_nav` with exactly the expected fields, including the POI topics (`/poi/list`, `/poi/command`, `/poi/result`) and the merged 3D view contract fields (`local_costmap_topic`, `base_urdf`, `base_joint_states_topic`, `arm_urdf`, `arm_joint_states_topic`, arm command topic `/filter/web_ui_joint_commands`) and the arm Trigger services `/arm/home` / `/arm/set_home`. |
| `test_web_ui_installs_slam_toolbox_for_its_service_imports` | web_ui node type installs `ros-jazzy-slam-toolbox` (it imports `slam_toolbox.srv`), so deploying web_ui alone does not fail with ImportError. |
| `test_slam_scan_matcher_is_resilient_to_slip_and_transients` | slam_toolbox defaults: `correlation_search_space_dimension` 0.8 (a whole number of 0.01 cells), `check_min_dist_and_heading_precisely` true, `ceres_loss_function` HuberLoss, `occupancy_threshold` 0.25, `min_pass_through` 3; travel thresholds and loop closing unchanged. |
| `test_slam_ansible_block_does_not_override_slip_resilience_keys` | The Ansible slam_toolbox override block does not set those keys, so the repo defaults apply. |
| `test_imu_angular_velocity_covariance_is_fixed_and_small` | `bno055_imu` uses `compute_covariance: false` and a fixed `angular_velocity_covariance` of 0.0004 (no yaw-rate inflation while turning). |
| `test_swerve_controller_slip_residual_threshold_configured` | The swerve controller config sets `slip_residual_threshold_mps` to 0.05. |
| `test_ekf_fuses_rf2o_and_rejects_outlying_wheel_odometry` | Repo `ekf.yaml` and the Ansible copy fuse `/odom_rf2o_twist` (vx, vy only, not differential), reject the wheel odometry twist above Mahalanobis threshold 1.5, have no rejection on rf2o, and keep the IMU gyro yaw rate as the only IMU input. |
| `test_rf2o_node_type_builds_a_pinned_source_workspace` | `rf2o_laser_odometry` node type: native, launched with `ros2 run` and the repo params file, source pinned to a full commit SHA in `/opt/ros2-ws`, apt build dependencies listed. |
| `test_rf2o_params_match_the_stack` | rf2o params: scan `/scan_filtered`, odom `/odom_rf2o`, `publish_tf` false, frames `base_link` / `odom`, empty `init_pose_from_topic`, `freq` 7-10 Hz. |
| `test_rf2o_nodes_are_deployed_before_the_ekf` | `rf2o_laser_odometry` and `rf2o_odom_relay` are present, enabled, listed before the EKF, have per-node playbooks and are deployed before the EKF in `deploy_nodes_client.yml`. |
| `test_rf2o_relay_wiring_matches_ekf_input` | `rf2o_odom_relay` node type/config: reads `/odom_rf2o`, publishes the topic the EKF fuses, positive variances. |
| `test_colcon_source_build_runs_at_lowest_priority_and_only_on_a_new_commit` | `colcon_source_build.yml` clones the pinned commit and builds with `systemd-run` (CPUQuota 100%, IOWeight 10), nice 19, ionice idle, `--merge-install`, one worker, `MAKEFLAGS=-j2`, a per-commit-and-patch-hash stamp (`creates:`) and a restart notify; included by the role. |
| `test_colcon_source_build_applies_patches_and_keys_the_stamp_on_their_content` | `colcon_source_build.yml` applies the `colcon_source.patches` with `ansible.builtin.patch` between the (forced) clone and the build, hashes them into the stamp key, and drops the stamp when `install/setup.bash` is missing. |
| `test_rf2o_retry_laser_tf_patch_exists_and_is_wired` | `patches/0001-retry-laser-tf.patch` exists, skips scans until the laser TF lookup succeeds, and is listed in the rf2o node type's `colcon_source.patches`. |
| `test_rf2o_odom_relay_declares_numpy` | `rf2o_odom_relay/pyproject.toml` lists `numpy` explicitly (numpy guard). |
| `test_launcher_sources_the_colcon_workspace_after_ros` | The launcher template sources the colcon `install/setup.bash` after `/opt/ros/jazzy/setup.bash`, and `resolve_and_deploy.yml` passes `colcon_source`. |
| `test_every_test_is_documented_in_tests_readme` | Every `test_*` function in `test_nav_stack_config.py` is listed in this section. |

### test_poi_store_config.py

Static wiring invariants of the poi_store node (Ansible, playbooks, lint script, docs).

| Test | What it covers |
|---|---|
| `test_poi_store_node_type_defaults` | `poi_store` node type: native, `nodes/poi_store`, `python3 -m poi_store`, `config_path` `/etc/ros2/poi_store`, env `POI_STORE_CONFIG`. |
| `test_poi_store_entry_after_mcp_server_with_config` | Present + enabled `ros2_nodes` entry directly after `mcp_server`; config has `store_path` `/var/lib/ros2/poi/poi.json` and the `/poi/list`, `/poi/command`, `/poi/result` topics. |
| `test_poi_directory_task_owned_by_node_user` | `poi_store_dir.yml` creates `/var/lib/ros2/poi` owned by `ansible_user`. |
| `test_poi_playbook_deploys_node_and_creates_directory` | `deploy_nodes_client.yml` creates the directory and deploy `poi_store` (after `mcp_server`). |
| `test_lint_script_and_node_files` | `scripts/lint-all-nodes.sh` lists the node; pyproject, poetry.lock, README and tests exist. |
| `test_docs_mention_poi_store` | nodes/README.md, ansible/README.md and the ansible-deploy skill mention `poi_store`. |

The node's own tests (store, models, config) live in `nodes/poi_store/tests/` and need no ROS: `cd nodes/poi_store && poetry run pytest tests -q`.

### test_mcp_camera_config.py

Static checks of the mcp_server camera tools wiring (YAML and README only; no ROS needed).

| Test (group) | Description |
|------|-------------|
| `test_setup_tasks_create_the_calibration_directory_owned_by_the_node_user` | `tasks/mcp_server_setup.yml` creates `/var/lib/ros2/camera_calibration` owned by `ansible_user`. |
| `test_cameras_config_front_not_calibrated_gripper_calibrated` | The mcp_server `cameras` block has `calibration_dir`; the front camera is uncalibrated (`intrinsics`/`mount` null), the gripper camera has intrinsics and a mount on `gripper_link`. |
| `test_gripper_parent_frame_is_a_link_of_the_arm_urdf` | `cameras.gripper.parent_frame` names a link in the arm URDF. |
| `test_arm_reach_keys_are_ordered` | `arm.reach_inner_m` is positive and below `arm.reach_outer_m`. |
| `test_readme_documents_the_camera_tools_frames_and_calibration` | `nodes/mcp_server/README.md` documents the seven camera tools, frames, the not-calibrated default, `base_in_base_link`, the calibration directory and the procedure. |

### test_mcp_server_config.py

Static checks of the robot MCP server wiring (YAML, the unit template rendered with jinja2, the token script run with a
fake `ssh`; no ROS needed).

| Test (group) | Description |
|------|-------------|
| `test_mcp_server_node_type_defaults` | `mcp_server` node type: native, `nodes/mcp_server`, `python3 -m mcp_server`, 25% / 256M, `config_path` `/etc/ros2/mcp_server`, env `MCP_SERVER_CONFIG`, token only via `environment_file: /etc/ros2/mcp_server/token`. |
| `test_mcp_server_node_entry_and_config` | Present + enabled `ros2_nodes` entry; config block keys, server 0.0.0.0:18200 `/mcp`, repo-relative arm URDF that exists, home file `/var/lib/ros2/arm/home.yaml`. |
| `test_mcp_server_topics_match_filter_node_lease` | mcp_server autonomy command/release/active source and follower feedback topics equal filter_node's. |
| `test_filter_node_autonomy_params` | filter_node config sets `autonomy_input_topic`, `autonomy_release_topic`, `active_source_topic`. |
| `test_gripper_camera_enabled` | `gripper_uvc_camera` is present and enabled again. |
| `test_shoulder_lift_upper_limit_allows_reaching_below_the_floor` | mcp_server `arm.joint_limit_overrides_rad` widens shoulder_lift to [-1.745, 1.9] rad (tested on the robot: 1.87 rad reached without collision, the stretched arm cannot be lifted beyond about 1.85). |
| `test_gripper_camera_rotated_180_at_source` | `gripper_uvc_camera` env sets `UVC_ROTATE_DEG=180` exactly once (the wrist image is upside down at wrist roll 0; rotation happens in the camera node, not downstream). |
| `test_mcp_server_nav_tolerances_match_nav2_goal_checker` | mcp_server `nav.goal_xy_tolerance_m` / `goal_yaw_tolerance_deg` in client.yml equal the nav2_params.yaml goal checker (0.01 m, 0.035 rad ~ 2 deg). |
| `test_claude_agent_nav_tolerances_match_mcp_server` | claude_agent `nav_goal_xy_tolerance_cm` / `nav_goal_yaw_tolerance_deg` in client.yml equal the mcp_server `nav` values. |
| `test_mcp_server_arm_base_height_is_16_5_cm` | mcp_server config `arm.arm_base_height_m` is 0.165 (measured mount height above the floor). |
| `test_mcp_server_monitor_block_has_ordered_thresholds` | mcp_server `monitor` block: servo 60/70 C, CPU 75/82 C, bump warning below critical, stall 1.0 s, tilt 10 deg, battery warning margin 0.2 V/cell. |
| `test_mcp_server_monitor_topics_match_their_producers` | mcp_server monitor topics: `/follower/servo_registers`, `/imu/data`, `/robot_events`, `swerve_odom` equals the swerve controller `odom_topic`, `rf2o_twist` equals the rf2o relay `output_topic`. |
| `test_mcp_server_readme_documents_monitor_and_events_contract` | `nodes/mcp_server/README.md` documents get_body_state, the per-call digest, the `/robot_events` contract, early-return (`interrupted_by`, expected/achieved), event types and the `monitor` thresholds. |
| `test_web_ui_tab_set` | web_ui tabs are exactly map (map_nav, first), agent (agent_chat), camera (`/camera_0/image_raw/compressed`), overview_camera (type `camera`, label `Overview`, `/overview_camera/image_raw/compressed`), imu_graphs; arm_servos, local_nav, gps_nav, scene3d and robot_status are gone. |
| `test_mcp_server_front_camera_is_the_compressed_overview_camera` | mcp_server `topics.front_camera` is `/overview_camera/image_raw/compressed` and no `realsense_camera` key remains. |
| `test_web_ui_map_tab_uses_contract_fields` | The map tab carries no legacy keys (`urdf_file`, `topic`, `arm_urdf_file`, `arm_joint_topic`, `scan_topic`, `costmap_topic`), uses the frontend contract names, points `arm_home_service` / `arm_set_home_service` at the services mcp_server serves, and keeps the tile cache at `/var/cache/web_ui/tiles`. |
| `test_web_ui_tile_cache_dir_task_owned_by_node_user` | `playbooks/tasks/web_ui_tile_cache_dir.yml` creates `/var/cache/web_ui` and `/var/cache/web_ui/tiles` as directories owned by `ansible_user`. |
| `test_playbooks_create_tile_cache_before_deploying_web_ui` | `deploy_nodes_client.yml` includes the tile cache task before deploying web_ui. |
| `test_mcp_server_perception_topics_match_their_producers` | mcp_server `topics` `local_costmap` `/local_costmap/costmap`, `plan` `/plan` and `poi_list` / `poi_command` / `poi_result` equal the poi_store entry's topics. |
| `test_mcp_server_perception_config_sections` | mcp_server config carries `objects` (`/var/lib/ros2/objects/objects.json`, merge radius 0.25 m), `footprint` 0.47 x 0.386 m, `look_around` defaults (4 captures, positive clearance margin), `topdown` defaults and `poi.request_timeout_s` 3 s. |
| `test_mcp_server_readme_documents_perception_tools` | `nodes/mcp_server/README.md` documents get_topdown_view, the object memory tools, look_around, the POI tools, the objects file, the robot-up convention, `layers_missing` and the `MOTION_TOOLS` classification. |
| `test_mcp_server_setup_tasks_create_objects_dir_owned_by_node_user` | `tasks/mcp_server_setup.yml` creates `/var/lib/ros2/objects` owned by `ansible_user`. |
| `test_mcp_server_setup_tasks_create_token_and_arm_dir` | `tasks/mcp_server_setup.yml` creates `/etc/ros2/mcp_server`, the token (`MCP_SERVER_TOKEN=` + 48-char password lookup, `force: false`, 0640, owner `ansible_user`, group `mcp-token`, `no_log`) and `/var/lib/ros2/arm` owned by the node user. |
| `test_playbooks_run_setup_before_deploying_mcp_server` | `deploy_nodes_client.yml` deploys mcp_server and includes the setup tasks before it. |
| `test_unit_template_renders_environment_file_only_when_set` | The native unit template renders `EnvironmentFile=` (before `ExecStart=`) only when `node_environment_file` is non-empty. |
| `test_resolve_and_deploy_passes_environment_file` | `resolve_and_deploy.yml` passes the node type's `environment_file` (default empty) to the role. |
| `test_token_file_never_in_repo` | No tracked `token` file and no literal token value in tracked Ansible/scripts/node/test files; `.mcp.json` uses `${ROBOT_MCP_TOKEN}`. |
| `test_mcp_json_registers_robot_server` | `.mcp.json` registers server `robot` (type http, `http://client.ros2.lan:18200/mcp`, `Authorization: Bearer ${ROBOT_MCP_TOKEN}`) matching the node's port and path. |
| `test_robot_mcp_token_script` | `scripts/robot_mcp_token.sh` is executable bash (`set -euo pipefail`, passes `bash -n`), reads the token file over ssh and prints an export line. |
| `test_robot_mcp_token_script_prints_export_line` | With a fake `ssh` returning the EnvironmentFile line, the script prints `export ROBOT_MCP_TOKEN='<token>'`. |
| `test_mcp_server_node_package_layout` | `nodes/mcp_server` has a Poetry project with the `mcp` dependency and `mcp_server` package, a lock file, `__main__.py`, and a README with the `claude mcp add` setup. |
| `test_mcp_server_listed_in_nodes_readme_index` | `nodes/README.md` Layout index has an `mcp_server/` entry describing the MCP server. |
| `test_mcp_gripper_closed_target_is_inside_the_follower_gripper_command_range` | The mcp_server closed gripper target (-0.165 rad = 1940 steps, follower gripper not inverted) is at or above the follower gripper `command_min_steps` (1900), `autonomy` is a direct command source, and the mcp_server README documents the steps and `command_min_steps`. |
| `test_every_test_is_documented_in_tests_readme` | Every `test_*` function in `test_mcp_server_config.py` is listed in this section. |

### test_battery_config.py

Static invariants of the battery voltage chain in Ansible `group_vars` (follower publisher, web_ui command guard).

| Test | Description |
|------|-------------|
| `test_follower_publishes_battery_state` | `lerobot_follower` config sets `battery_topic: /battery_state`, `battery_interval_s: 1.0` and `battery_cells: 3`. |
| `test_web_ui_battery_matches_follower` | web_ui `battery.topic` and `battery.cells` equal the follower's `battery_topic` and `battery_cells`. |
| `test_web_ui_cutoff_is_8v4_for_three_cells` | web_ui `battery` block: cut-off 2.8 V/cell x 3 cells = 8.4 V, resume 2.9 V/cell (>= cut-off), `stale_s` 5.0. |
| `test_mcp_server_battery_matches_web_ui` | mcp_server `battery` block is identical to web_ui's (topic `/battery_state`, 3 cells, 2.8/2.9 V per cell, `stale_s` 5.0; 8.4 V cut-off). |
| `test_leader_does_not_publish_battery` | Server `lerobot_leader` has `battery_interval_s: 0`, so only the robot's follower bus publishes `/battery_state`. |
| `test_every_test_is_documented_in_tests_readme` | Every `test_*` function in `test_battery_config.py` is listed in this section. |

### test_claude_agent_config.py

Static checks of the claude_agent wiring (YAML, the unit template rendered with jinja2 when installed, the node package
layout; no ROS needed).

| Test (group) | Description |
|---|---|
| `test_claude_agent_node_type_defaults` | `claude_agent` node type: native, `nodes/claude_agent`, `python3 -m claude_agent`, 50% / 1G, nice, user `claude_agent`, group `mcp-token`, `environment_file` `/etc/ros2/claude_agent/env`, `DISABLE_AUTOUPDATER=1`, no secret in `env`. |
| `test_claude_agent_entry_after_mcp_server_and_enabled` | Present + enabled `ros2_nodes` entry directly after `poi_store` (which follows `mcp_server`). |
| `test_claude_agent_config_valid_and_consistent_with_mcp_server` | The entry's config validates against the node's pydantic model, binds 127.0.0.1:18300, points at mcp_server's URL and token file, and every classified tool exists in `mcp_server/tools.py`. |
| `test_claude_agent_config_has_budget_maxima_and_robot_events_topic` | The client.yml config sets the hard maxima (rw 150 / turns 200), the per-phase maxima (40 / 40), no ro maxima and `robot_events_topic: /robot_events`, and no longer has `effector_call_cap` / `max_turns`. |
| `test_effector_tools_match_mcp_server_motion_tools` | `effector_tools` in the claude_agent config equals mcp_server `MOTION_TOOLS` (parsed from tools.py with `ast`), so the effector cap and the battery gate cover the same tools. Strict: `look_around` must be in `MOTION_TOOLS`. |
| `test_round2_tools_classified_in_group_vars_and_defaults` | `look_around` is an effector and `get_body_state` plus the round-2 sensor tools (pixel_to_ground, annotated image, candidates, calibration, topdown view, objects, POIs) are sensors, in both the client.yml config and the code defaults. |
| `test_service_template_user_groups_and_nice_are_optional` | Unit template: `User=` defaults to `ansible_user`; `node_user`, `SupplementaryGroups=` and `Nice=` only when set. |
| `test_resolve_and_deploy_passes_user_groups_and_nice` | `resolve_and_deploy.yml` hands `user`, `supplementary_groups` and `nice` of the node type to the role. |
| `test_setup_tasks_read_token_from_controller_env_and_fail_clearly` | `claude_agent_setup.yml` uses `lookup('env', 'CLAUDE_CODE_OAUTH_TOKEN')`; a fail task (before the write, without `no_log`, without touching the token) tells the user to `export CLAUDE_CODE_OAUTH_TOKEN=...` and run `deploy-nodes.sh client claude_agent`. |
| `test_setup_tasks_write_env_file_0600_no_log` | The env file task: `CLAUDE_CODE_OAUTH_TOKEN=` content, mode 0600, owner `claude_agent`, `no_log`, only when the variable is non-empty (no notify: a changed token queues the claude_agent restart with `set_fact`). |
| `test_playbook_level_task_files_notify_no_role_handlers` | No task file under `ansible/playbooks/tasks/` uses `notify`: they run at playbook level, where the `ros2_node_deploy` role's `Restart ROS2 node` handler is not visible and the deploy fails. |
| `test_setup_tasks_no_log_on_every_task_touching_the_token` | Every task using the lookup or writing the token has `no_log: true`. |
| `test_setup_tasks_create_user_and_dirs_and_keep_existing_file` | System user `claude_agent` (nologin, home `/var/lib/claude_agent`), its directories, and a stat of the existing env file so an unset variable keeps it. |
| `test_mcp_token_readable_by_group_not_world` | `mcp_server_setup.yml` creates group `mcp-token` before the token, which is `0640` with that group. |
| `test_existing_mcp_token_gets_group_read_permissions` | After the `force: false` create, a file task enforces group `mcp-token` and mode 0640 on an already existing token, so `claude_agent` can read a token created before the group existed. |
| `test_playbooks_run_setup_before_deploying_claude_agent_after_mcp_server` | `deploy_nodes_client.yml`: mcp setup, then agent setup, then the deploy (after mcp_server in the full playbook). |
| `test_oauth_token_never_in_repo` | No tracked file contains a literal `CLAUDE_CODE_OAUTH_TOKEN=<token>`. |
| `test_claude_agent_package_layout_and_pinned_sdk` | Poetry project with the SDK pinned to an exact version, FastAPI/uvicorn/Pillow/pydantic, lock file, README, `__main__.py`, tests. |
| `test_claude_agent_readme_documents_api_and_token_deploy` | The node README lists every API route, the four planning tools, the hard and per-phase maxima, the `/robot_events` topic and the `export CLAUDE_CODE_OAUTH_TOKEN` deploy command. |
| `test_docs_and_lint_scripts_list_claude_agent` | `nodes/README.md`, `ansible/README.md`, the ansible-deploy skill, `scripts/lint-all-nodes.sh` and root `lint-nodes` mention the node. |
| `test_claude_agent_hardening_in_group_vars_without_filesystem_protection` | The `claude_agent` node type sets `protect_proc: invisible`, `proc_subset: pid`, `no_new_privileges`, `private_tmp` and no ProtectSystem/ProtectHome (the bearer token is in the CLI child's argv). |
| `test_only_claude_agent_sets_hardening_in_group_vars` | No other node type in client.yml / server.yml sets any of the hardening keys, so their units are unchanged. |
| `test_resolve_and_deploy_passes_hardening_with_empty_defaults` | `resolve_and_deploy.yml` hands the four hardening keys to the role, each with a `default(...)`. |
| `test_role_defaults_keep_hardening_off` | `ros2_node_deploy` defaults leave all four hardening variables empty/false. |
| `test_service_template_hardening_is_optional` | Unit template (jinja2 when installed): no hardening lines by default or with empty values (identical output); the four lines and no ProtectSystem/ProtectHome when set. |
| `test_claude_agent_config_has_watchdog_and_sdk_initialize_timeout` | The entry's config sets `instruction_timeout_s: 900` and the node env raises the SDK initialize timeout (`CLAUDE_CODE_STREAM_CLOSE_TIMEOUT=180000`). |
| `test_setup_tasks_create_persistent_workdir_owned_0750` | `claude_agent_setup.yml` creates `/var/lib/claude_agent/workspace` (the agent's persistent notes volume) as a directory owned by `claude_agent`, mode 0750, after its parent HOME directory. |
| `test_ansible_never_removes_the_workdir_or_state_dir` | No Ansible task (file `state: absent`, `rm` in command/shell) deletes anything under `/var/lib/claude_agent`, so notes and the session log survive deploys. |
| `test_claude_agent_config_workdir_state_dir_and_hardware_facts` | The entry's config sets `workdir`, `state_dir`, `arm_base_height_m` 0.165 and `arm_reach_cm`; the service `HOME` stays `/var/lib/claude_agent`, separate from the workdir. |
| `test_every_test_is_documented_in_tests_readme` | Every `test_*` function in `test_claude_agent_config.py` is listed in this section. |

### test_web_ui_agent_tab.py

Static checks of the web_ui Agent tab in `ansible/group_vars/client.yml` (no ROS needed).

| Test (group) | Description |
|---|---|
| `test_agent_tab_exists_with_type_and_label` | web_ui has a tab `agent` of type `agent_chat` labelled "Agent". |
| `test_agent_tab_directly_after_map_tab` | The Agent tab is listed immediately after the `map` tab. |
| `test_agent_tab_url_matches_claude_agent_port` | `agent_url` is 127.0.0.1 with the same port as the claude_agent `http_port` (18300). |
| `test_every_test_is_documented_in_tests_readme` | Every `test_*` function in `test_web_ui_agent_tab.py` is listed in this section. |

### `test_overview_camera_config.py`

Static checks of the overview_camera node from the repo files (YAML via `yaml.safe_load`, launch file via `ast`; no ROS needed): boot overlay and reboot, the multi-source colcon build (rf2o unchanged, libcamera meson args, camera_ros pin, patches, no apt camera stack), node type / entry / resources, retired stereo wiring, RealSense removal, `camera_link` removal, params defaults and launch structure.

| Test | Description |
|---|---|
| `test_boot_tasks_set_the_imx708_overlay_in_firmware_config` | `overview_camera_boot_config.yml` writes `camera_auto_detect=0` and `dtoverlay=imx708,cam0` (and nothing else) to `/boot/firmware/config.txt`, each with a regexp that matches its own line (no duplicates on rerun). |
| `test_boot_tasks_remove_stale_imx219_overlays_idempotently` | A first `state: absent` task deletes every `dtoverlay=imx219...` line (cam0, cam1, bare) and keeps `imx708`, `camera_auto_detect` and commented lines. |
| `test_boot_tasks_register_results_and_reboot_only_when_changed` | Each overlay task registers its result and the single reboot task (last) runs only when one of them changed. |
| `test_boot_overlays_use_the_same_path_as_the_uart_task` | The camera overlay edits the same config.txt path as the existing UART task in `deploy_nodes_client.yml`. |
| `test_client_playbook_applies_the_camera_boot_overlay_in_pre_tasks_tagged_boot_and_node` | The client playbook includes the boot task file in `pre_tasks`, tagged only `boot` and `overview_camera`. |
| `test_boot_overlays_are_only_written_on_aarch64_hosts` | Overlay tasks are guarded by `ansible_machine == "aarch64"`. |
| `test_full_client_playbook_deploys_overview_camera_and_not_realsense_before_it` | `deploy_nodes_client.yml` deploys `overview_camera`, after the `realsense_d435i` uninstall. |
| `test_retired_stereo_nodes_are_gone_from_the_client_wiring` | No `stereo_camera` / `stereo_depth` in the client playbook or group vars, their node directories, per-node playbooks and the `stereo_depth` lint-all-nodes entry are gone. |
| `test_rf2o_build_inputs_are_unchanged` | rf2o keeps its single-dict `colcon_source` with the exact repo, commit, package, workspace and patch. |
| `test_colcon_build_accepts_one_dict_or_a_list_of_sources` | `colcon_source_build.yml` wraps a single dict into a list and builds each source in list order through `colcon_source_package.yml`. |
| `test_colcon_package_build_keeps_the_limits_and_stamp_logic` | The per-source build keeps systemd-run/nice/ionice, `--merge-install`, one worker, `MAKEFLAGS=-j2`, the git `force`, the `creates` stamp, the restart handler and the Release default; supports `--meson-args` and `--cmake-args`. |
| `test_colcon_stamp_key_is_unchanged_for_the_first_source_and_chained_after` | The stamp key still starts from commit + patch checksums (rf2o unchanged) and later sources chain the earlier keys so a libcamera rebuild rebuilds camera_ros. |
| `test_overview_camera_node_type_builds_libcamera_then_camera_ros` | `overview_camera` builds libcamera (tag `v0.7.2+rpt20260817`, all required meson args) before camera_ros (pinned full SHA) in `/opt/ros2-ws`. |
| `test_builds_use_all_cores_at_lowest_priority_and_nodes_start_quickly` | `all.yml`: `ros2_build_jobs` 4 and `ros2_build_cpu_quota` 400% for source/frontend builds (still nice 19, idle IO), `ros2_node_start_interval_s` 2. |
| `test_gripper_uvc_camera_uses_a_stable_device_path` | The gripper UVC camera is configured by its `/dev/v4l/by-id` path, since the overview CSI camera's driver takes `/dev/video0..9`. |
| `test_gripper_camera_is_calibrated_with_a_repo_intrinsics_file` | mcp_server's gripper camera has a calibrated mount on `gripper_link` and intrinsics from `nodes/mcp_server/calibration/gripper_camera.yaml` (640x480, 3x3 matrix, 5 distortion terms). |
| `test_camera_image_wait_covers_discovery_on_a_loaded_pi` | mcp_server `timeouts.image_timeout_s` >= 5 s: each camera call subscribes afresh and discovery exceeds 2 s with the full stack running. |
| `test_only_the_libcamera_patch_is_listed_and_it_exists_in_the_repo` | Only the libcamera `package.xml` patch is listed and present; the camera_ros SyncMode patch is gone. |
| `test_libcamera_patch_adds_a_meson_package_xml` | The libcamera patch adds a `package.xml` with `build_type` meson. |
| `test_apt_packages_carry_the_build_deps_and_never_the_apt_camera_stack` | The apt list has the build dependencies and the image_transport compressed plugins, no stereo/image_proc/calibration packages, and no node type installs `ros-jazzy-camera-ros` or `ros-jazzy-libcamera`. |
| `test_launcher_template_sources_the_first_source_workspace_for_a_list` | The launcher template sources the workspace of a dict source or of the first list item. |
| `test_overview_camera_node_type_resources_and_launch_command` | Node type: native, `CPUQuota` 100%, `Nice` 5, `MemoryMax` 256M, config path/env and the `ros2 launch` command. |
| `test_overview_camera_node_entry_is_present_and_enabled` | The `overview_camera` ros2_nodes entry is present and enabled and its config only holds `camera_id`. |
| `test_realsense_is_uninstalled` | `realsense_d435i` is `present: false`. |
| `test_guessed_camera_link_frame_is_removed_from_the_static_tf_config` | `static_tf_publisher` no longer publishes `camera_link` (imu_link and laser_frame stay). |
| `test_node_directory_layout_without_calibration_or_stereo_leftovers` | Node files exist, there is no `calibration/` directory and the README names `cam -l`, `AfMode`, IMX708, the overlay and the compressed topic. |
| `test_camera_defaults_match_the_requirements` | `params.yaml`: 640x480, 15 fps limits, continuous autofocus, optical frame id, JPEG quality 80, only `launch` and `camera` sections. |
| `test_launch_settings_default_to_no_camera_id` | Empty `camera_id` by default. |
| `test_launch_file_is_valid_python_with_a_launch_description` | The launch file parses and defines `generate_launch_description`. |
| `test_launch_file_builds_the_single_camera_pipeline` | (parametrized) The launch file names the container, intra-process option, `camera::CameraNode`, the `/overview_camera` namespace and the compressed / camera_info topics. |
| `test_launch_file_has_no_stereo_or_calibration_stages` | No rectify / disparity / point cloud / calibration gating / SyncMode in the launch file. |
| `test_every_new_root_test_file_is_documented` | Every `test_overview_camera*.py` file and test function is listed in this README. |

### `test_overview_camera_launch_config.py`

Unit tests of the pure-Python launch helper `nodes/overview_camera/launch/overview_camera_config.py` (imported via path setup): settings merge, camera selection and the camera_ros parameter dict.

| Test | Description |
|---|---|
| `test_deep_merge_overrides_nested_keys_without_touching_the_base` | `deep_merge` merges nested dicts into a new dict. |
| `test_load_settings_returns_defaults_when_the_override_is_missing_or_empty` | Missing or empty deployed config leaves the defaults (640 wide). |
| `test_load_settings_applies_the_deployed_override` | A nested override replaces only the keys it sets. |
| `test_load_settings_accepts_a_top_level_camera_id_in_the_override` | The flat `camera_id` lands in the `launch` section. |
| `test_camera_selector_prefers_the_libcamera_id` | A configured ID is used as is, without warning. |
| `test_camera_selector_falls_back_to_index_zero_with_a_warning` | No ID selects index 0 and returns a warning that mentions `cam -l`. |
| `test_camera_selector_treats_whitespace_as_unset` | A whitespace-only ID counts as unset. |
| `test_camera_parameters_of_the_overhead_camera` | Params: ID, 640x480, 15 fps limits, `AfMode` 2 (continuous), frame id and the image_transport JPEG quality 80. |
| `test_camera_parameters_have_no_calibration_url_and_no_sync_mode` | No `camera_info_url`, no `SyncMode`, and the flat `jpeg_quality` setting is translated. |
| `test_af_mode_names_map_to_the_libcamera_enum` | (parametrized) manual / auto / continuous (any case) map to 0 / 1 / 2. |
| `test_camera_parameters_reject_an_unknown_af_mode` | An unknown AfMode name raises `ValueError`. |

### test_ansible_deploy_speed.py

Deploy speed work: stamps instead of always-run builds, queued restarts, batched apt, tags on every task and one tag-filtered playbook run per deploy. The probe scripts of the role are run for real against temporary git repos.

| Test | What it checks |
|---|---|
| `test_ansible_cfg_enables_timing_pipelining_and_connection_reuse` | `ansible.cfg` enables `profile_tasks` + `timer`, pipelining, `ControlMaster=auto` / `ControlPersist`; `requirements.yml` lists `ansible.posix`. |
| `test_ansible_cfg_gathers_minimal_facts_and_caches_them` | `gather_subset = min`, smart gathering, jsonfile fact cache (cache dir gitignored). |
| `test_documented_tag_set_lists_every_phase_tag` | The "Deploy tags" table in `ansible/README.md` lists exactly sync, apt, python, build, config, boot, setup, restart, verify, always. |
| `test_every_task_in_role_and_task_files_carries_a_documented_tag` | (parametrized per file) every task (block tags inherited) in the role, the verify role and `playbooks/tasks/*.yml` has a documented tag. |
| `test_every_task_in_the_deploy_playbooks_carries_a_documented_or_node_tag` | (client, server) every pre/main/post task of the deploy playbooks has a documented or node-name tag. |
| `test_every_node_has_a_tagged_deploy_step` | (client, server) each `ros2_nodes` entry has exactly one deploy include tagged with its name plus apt/python/build/config, applying the node tag to the included tasks. |
| `test_node_specific_setup_files_carry_their_node_tag` | The mcp_server token, claude_agent token, poi_store, slam maps, web_ui tile cache and overview_camera boot task files carry their node tag. |
| `test_shared_steps_run_under_any_node_filter` | (client, server) select_run, repo sync, ROS package sync, batched apt and the gradual restart are `always`-tagged; the verify role too; the apt steps are conditioned on `ros2_run_apt`. |
| `test_deploy_script_runs_one_playbook_with_the_joined_node_tags` | `deploy-nodes.sh client web_ui mcp_server` calls ansible-playbook once with `--tags web_ui,mcp_server` (fake ansible-playbook on PATH). |
| `test_deploy_script_all_passes_tags_and_other_options_through` | `--all` adds no tag filter by itself and passes `--tags` / `--skip-tags` through. |
| `test_deploy_script_rejects_unknown_nodes_and_node_list_with_tags` | Unknown node names are rejected with the valid list; a node list plus `--tags` is rejected; `playbooks/nodes/` no longer exists. |
| `test_role_has_no_always_changed_tasks_and_does_not_restart_or_start_nodes` | No `changed_when: true` in the role; no inline start/restart; the "Restart ROS2 node" handler only queues into `ros2_nodes_pending_restart`. |
| `test_poetry_install_runs_only_for_a_new_dependency_hash_and_stamps_after_success` | Poetry runs only when the dependency hash is stale and writes `.poetry-deps` after success; the probe covers pyproject, lock, shared metadata and tree hashes; the source stamp notifies the restart. |
| `test_web_ui_npm_steps_run_only_when_their_inputs_changed` | `npm ci` / `npm run build` are conditioned on the frontend probe, copy and stamps follow only a build, stamp written last. |
| `test_dependency_probe_detects_source_dependency_and_shared_changes` | Runs the probe in a temp repo: new venv is stale; matching stamp is not; a source commit changes only `src_key`; `shared/` source restarts dependents without reinstall; shared metadata or `poetry.lock` change the dependency hash. |
| `test_web_ui_probe_builds_only_when_frontend_inputs_change` | Runs the web_ui probe: first run needs ci + build; with stamps and outputs neither; a source commit builds without ci; a lock change runs ci; missing static output rebuilds. |
| `test_heavy_steps_stop_the_nodes_only_when_they_will_run` | The stop-before-build include is conditional (Poetry stale, web_ui build, colcon stamp missing), runs once per play, and no playbook stops nodes up front. |
| `test_colcon_clone_patch_and_build_are_skipped_when_the_stamp_exists` | Clone, patch and build only when the stamp is missing; the stamp check comes before the clone. |
| `test_start_script_restarts_the_queue_starts_stopped_nodes_in_scope_and_clears_the_queue` | Runs the rendered start script against a fake `systemctl`: queued nodes restart, stopped nodes in scope start, running unchanged ones are skipped, the queue file is emptied. |
| `test_start_script_with_a_node_filter_only_starts_selected_nodes_but_restarts_every_queued_one` | Scope limited to one node: only it is started, a queued node outside the scope is still restarted, others untouched. |
| `test_start_script_fails_on_a_failed_restart_and_keeps_the_queue` | A failing `systemctl restart` fails the script and the queue file keeps its entries for the next run. |
| `test_start_script_without_a_queue_file_acts_only_on_stopped_nodes` | No queue file: only stopped nodes start. |
| `test_start_ros_nodes_keeps_deploy_order_and_sleeps_only_after_acting` | `set -euo pipefail`, `ros2_nodes` order, skip before restart/start, sleep only after acting, restarted nodes remembered for verify. |
| `test_restart_queue_is_a_host_file_written_by_the_handler_and_claude_agent_setup` | The queue is `/var/lib/ros2-deploy/pending-restart`; the handler and the claude_agent token change append to it; nothing uses the old in-memory queue; the playbooks create the state dirs. |
| `test_deploy_tasks_run_in_a_block_whose_rescue_starts_nodes_and_still_fails_the_run` | (client, server) the node tasks are one block; its rescue runs the start step with ignored errors, releases the deploy lock, then fails the run naming the original failed task with rc and stderr; the success path still starts nodes. |
| `test_scope_facts_limit_start_and_verify_to_the_selected_nodes` | `ros2_scope_nodes` is set by `select_run.yml` and used by verify together with the restarted nodes. |
| `test_every_present_node_resolves_to_existing_source_paths` | (client, server) every present node resolves to a non-empty list of source paths that exist in the repo (`node_src_dir` or `src_paths`, extras, `shared`). |
| `test_mcp_server_restarts_when_the_web_ui_urdf_it_loads_changes` | mcp_server's source paths include `nodes/web_ui/urdf`. |
| `test_every_pyproject_depending_on_shared_is_declared_src_shared` | Every `nodes/**/pyproject.toml` with `../../shared` belongs to a node type with `src_shared: true`. |
| `test_docs_say_build_and_config_filters_skip_apt` | `ansible/README.md` states that `--tags build` / `--tags config` do not install apt packages. |
| `test_apt_packages_are_installed_in_one_batched_task` | (client, server) the playbook sets `ros2_apt_batched`, includes `apt_nodes.yml` (one apt call, hourly cache) and the role's per-node apt is skipped then. |
| `test_verify_role_checks_all_units_in_one_command_per_round` | The verify role runs one `systemctl is-active` over all units per round instead of looping per node. |

| `test_verify_runs_after_the_end_of_play_restarts` | (client, server) the `start_ros_nodes.yml` include comes before the verify role, followed only by the lock release. |
| `test_every_restart_causing_task_is_followed_directly_by_a_queue_task` | Every role/colcon task that notifies the restart handler registers its result and is followed at once by a lineinfile task appending the node to the persistent queue (only when changed and the node is enabled). |
| `test_source_stamp_is_written_after_config_launcher_and_unit` | The source stamp task comes after the config, launcher and unit tasks and before the handler flush. |
| `test_stop_for_build_records_the_stopped_units_before_stopping_them` | Runs the stop script against a fake `systemctl`: the running units land in `stopped-for-build` before `systemctl stop`. |
| `test_start_script_starts_every_unit_stopped_for_a_build_whatever_the_scope` | Units in `stopped-for-build` are started although outside the run's scope; the file is cleared. |
| `test_start_script_keeps_the_stopped_list_when_a_start_fails_and_reloads_systemd_first` | A failed start keeps the list; `systemctl daemon-reload` runs before the loop. |
| `test_recover_script_starts_the_stopped_units_only_when_the_list_is_old_and_no_deploy_is_running` | Runs `ros2-deploy-recover.sh` with file/lock mtimes: nothing for no list, a young list or a fresh lock; unique sorted starts and a cleared list when old; a stale lock is ignored. |
| `test_recover_timer_is_installed_by_the_deploy_playbooks_with_lock_taken_and_released` | `deploy_guard.yml` installs the script, service and a 2-minute enabled timer; both playbooks install it first, take the lock as the last pre-task and release it last. |
| `test_ros2_master_and_fastdds_watch_no_repo_path` | (client, server) the ros2_master and fastdds types have `src_paths: []` (constant key). |

