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

Bridge cycle (`test_bridge_cycle.py`, rclpy-free `bridge_cycle.py`): regression test that wheel `goal_speed` is never written to 0 while drive commands keep arriving faster than the loop (the 2026-10 stall: one ROS callback per loop iteration starved drive commands until the 0.3 s watchdog fired); watchdog stops wheels when drive commands cease while steering continues, works per wheel, stays fed by received commands even if a write fails, retries a failed stop; callback draining processes everything pending, is bounded and stops when nothing is ready; remaining-sleep computation; combined JointState with NaN placeholders drives steering and wheels; non-finite velocity/position entries ignored (and do not feed the watchdog); arm position-only messages and separate steer/drive messages still work.

### Per-node tests (uvc_camera)

The **uvc_camera** node has tests under `nodes/bridges/uvc_camera/tests/`. Run from `nodes/bridges/uvc_camera`: `poetry run pytest tests/ -v` (or `poetry run poe test`). `test_config.py` covers env-based config (`get_config`): defaults, env overrides, device as path or index, stripping whitespace and fallback for empty topic/frame_id. Config lives in `config.py` (no ROS/OpenCV deps) for testability.

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

The **filter_node** node has tests under `nodes/filter_node/tests/`. Run from `nodes/filter_node`: `poetry run pytest tests/ -v` (or `poetry run poe test`). Covers: config loading (`test_config.py`: input/output topic, algorithm, params, joint_names); algorithm registry and Kalman (`test_algorithms.py`: get_algorithm, Kalman create_state/update/predict).

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

- config loading (`test_config.py`: minimal config defaults, http_port override, missing file raises `FileNotFoundError`, `all_subscribed_topics` for camera/nav/overlay tabs, `load_config` from env var `WEB_UI_CONFIG`, `publish_topics` includes goal_topic, `rgbd_camera` tab type valid with color/depth/camera_info topics, RGBD topics included in `all_subscribed_topics`)
- bridge dirty-flag store (`test_bridge.py`: `flush_dirty` returns dirty topics, clears after flush, only returns dirty entries; `publish_dict` rejects non-allowlisted topics with warning)
- message serialization (`test_msg_serializer.py`: `msg_to_dict` — Imu message conversion; `extract_field_from_dict` — simple key, array index, missing returns None, out-of-bounds returns None; depth image serialization — 16UC1 returns expected keys, downscaled raw bytes length, depth values preserved, zero-depth preserved; `CameraInfo` returns fx/fy/cx/cy/width/height; color image on RGBD topic includes `color_small_b64`, non-RGBD topic omits it)
- HTTP server routes (`test_server.py`: `/api/config` returns config JSON, `/api/urdf/status` lists URDF files, `/api/urdf/<file>` serves URDF, path traversal blocked, security headers present, static fallback serves index.html)
- Battery (`test_battery.py`): `battery` config (defaults, absent = off, `resume_cell_v >= cutoff_cell_v`, `cells >= 1`, in `/api/config`, topic subscribed with role `battery`), `serialize_battery`, `BatteryGuard` (unknown without reading, cut-off below 8.4 V, hysteresis release above 8.7 V, stale reading not blocked, invalid voltage ignored, thread safety, rejection message), bridge battery callback (payload with guard state, NaN dropped), server rejection of WS `publish` (error frame) and of map save/reset and arm home/set home with 503, `nav/stop` still allowed, no blocking without battery config, with a good or unknown battery.
- Battery frontend (`frontend/src/battery/batteryStatus.test.ts`, vitest): chip level/colour/label (unknown, green, amber, red incl. backend flag), stale after `stale_s`, cut-off banner text, WebSocket frame parsing (envelope, error frame, malformed), 503 message surfaced by `parseActionResult`.
- URDF directory scanner (`test_urdf_scanner.py`: `scan_urdf_directory` finds robot.urdf, parses links/joints, detects missing mesh files, empty directory returns empty list)

- Map tab backend (`test_map_nav.py`): `map_nav` tab type, defaults (`/map`, `/plan`, `/optimal_trajectory`, `/goal_pose`, `map`, `base_link`, save path) and overrides, empty frame rejected, subscribed topics and publish allowlist, topic roles; OccupancyGrid -> PNG grey levels, thresholds, row orientation (y up), metadata, size mismatch; Path serialization and downsampling to 500 points keeping the ends, paths in another frame transformed to `map` (dropped without TF); goal PoseStamped serialization; quaternion/transform helpers; latched (reliable + transient_local) map subscription; robot pose from TF published when available and omitted otherwise; typed `_dict_to_ros_msg` (ints vs floats, errors raised); goal publishing fills stamp and frame; `POST /api/map/save` success, slam failure, service unavailable, timeout, unknown tab, no bridge; cached snapshot sent on WS connect; default config has the map tab. Reset/Stop: `map_reset_service` and `navigate_action` defaults; `POST /api/map/reset` (slam_toolbox `/slam_toolbox/reset`) success, non-success result, unavailable, timeout, unknown tab, no bridge, cached map cleared only on success (cleared-map event); `POST /api/nav/stop` cancels all `NavigateToPose` goals (zero goal id and stamp), unavailable, timeout, cached goal cleared on success (cleared-goal event). Robot footprint: `footprint_topic` default and `footprint` role, PolygonStamped serialized to points and re-expressed in the map frame.
- Map tab frontend (`frontend/src/map/mapMath.test.ts` and `mapActions.test.ts`, vitest, `npm test`): Reset/Stop action helpers, front edge of the footprint polygon (`frontEdgeIndex`) and world <-> screen (y up), map image placement with origin and yaw, pan, zoom about the cursor with clamping, pinch zoom/pan, fit and centre view, yaw/quaternion helpers and angle normalization, goal heading from drag or plain click (towards goal, robot heading fallback), scale-bar length choice.

### Per-node tests (claude_agent)

The **claude_agent** node has tests under `nodes/claude_agent/tests/` (no network, no real Claude calls; rclpy is stubbed in
`conftest.py`, the Agent SDK dataclasses are built directly). Run from `nodes/claude_agent`: `poetry run pytest tests -q`.
Covers:

- config (`test_config.py`): defaults (opus, 50 turns, cap 30, MCP URL/token file, port 18300, history 500, thumbnail 480),
  tool lists disjoint (stop can never be an effector), invalid numbers and unknown keys rejected, YAML loading,
  `CLAUDE_AGENT_CONFIG` lookup, MCP token read (env-file line or bare token, missing/empty refused, token never in the error)
- system prompt (`test_prompt.py`): built from the config caps and turn limit, lists every tool by kind, safety rules, persona,
  `system_prompt_extra` appended
- tools (`test_tools.py`): classification (sensor / effector / uncapped, unknown robot tool capped, non-robot unclassified),
  built-in tool deny list, sensor and `stop` calls never counted, effector calls counted then denied with the exact cap
  message, counter reset, everything outside `mcp__robot__*` denied, count and denial callbacks
- events (`test_events.py`): ring buffer and sequence numbers, subscribers, normalization of assistant text / tool calls /
  tool results from SDK objects, image thumbnails (size, no upscaling, RGBA, undecodable), 4000-char truncation flag,
  turn_end statuses (done, interrupted, max_turns, error) and auth-error detection
- env (`test_env.py`): `ANTHROPIC_API_KEY` / `ANTHROPIC_AUTH_TOKEN` removed from the child env, `DISABLE_AUTOUPDATER=1`, input untouched
- runner (`test_runner.py`): SDK options (robot HTTP MCP server with bearer header, `tools=[]`, no allow rules, `can_use_tool`), event
  flow with a fake client, token never in events, busy refusal, interrupt, effector counter reset per instruction, denial events,
  auth error, session failure and missing token become error events, reset starts a new session
- API (`test_api.py`): `/api/state`, `/api/history`, `/api/message` (202 / 409 busy / 400 empty or invalid), `/api/stop`,
  `/api/reset` (409 while busy), `/ws/events` history on connect then live events, shutdown hook
- entry point (`test_main.py`): loopback bind and port from config, ROS2 logger, API key removed from the process env, invalid config exits 1

### Per-node tests (mcp_server)

The **mcp_server** node has tests under `nodes/mcp_server/tests/` (no ROS needed: rclpy is imported only by
`ros_iface.py` and `__main__.py`). Run from `nodes/mcp_server`: `poetry run pytest tests -q` (or `poetry run poe test`).
`fakes.py` provides a simulated follower arm backend with a fake clock. Covers:

- config (`test_config.py`): defaults (0.0.0.0:18200 `/mcp`, topics, conservative limits, timeouts), repo-relative URDF
  path resolution, YAML overrides, empty file, unknown keys rejected, hard caps (0.25 m/s, speed scale 0.5, 1024 px),
  `MCP_SERVER_CONFIG` lookup, `MCP_SERVER_TOKEN` refused when missing/blank/short and stripped otherwise
- trajectory (`test_trajectory.py`): quintic blend endpoints/monotonicity, limit clamping with margin, duration from the
  quintic peak velocity, sampled trajectory ends exactly at the goal without exceeding the velocity cap, tracking error
- staleness (`test_staleness.py`), home store (`test_home_store.py`: missing file, atomic round trip, corrupt file,
  non-finite values), geometry (`test_geometry.py`: yaw/quaternion, relative goals, twist clamping)
- IK (`test_ik.py`): URDF joint limits, 5-DOF chain, FK at zero pose and pitch sign, IK forward/inverse round trip with
  pitch and position only, wrist_roll kept from the seed, solutions inside limits minus margin, unreachable targets
- perception (`test_perception.py`): 8 scan sectors in base_link (lidar mounted backwards), invalid returns ignored, map
  stats, map PNG crop size, image fitting, JPEG encoding, `sensor_msgs/Image` encodings, JPEG pass-through/downscale
- arm controller (`test_arm.py`): lease acquire/release, streamed motion with implicit acquire and velocity cap, speed
  scale, clamping, argument validation, abort + hold on stale feedback / tracking error / stop, no hold after filter_node
  switches source, convergence timeout, Cartesian moves (unreachable reported without motion), gripper open fraction
  and close-until-effort, home/set_home, keepalive and lease loss, state with stale data omitted
- drive (`test_base_motion.py`): rate, clamping, duration cap, abort always ends with a zero twist
- tools (`test_tools.py`): all 15 tools registered with real descriptions, structured outputs, camera JPEG + stamp,
  argument validation, robot errors as tool errors, map PNG, arm tool round trip, bearer-token auth on the Streamable
  HTTP app (401 without/with a wrong token, 200 with the right one) and the configured path
- battery gate (`test_battery_gate.py`): `MOTION_TOOLS` and `ALWAYS_ALLOWED_TOOLS` partition all tools; each motion tool
  is refused in cut-off with the exact message, a logged warning and no robot call; `stop`, sensor, state and
  acquire/release tools still work in cut-off; motion works with a good or unknown battery, without a guard and after
  recovery; optional `battery` config section (absent = off, validated)
- rclpy isolation (`test_rclpy_isolation.py`): only `ros_iface.py` / `__main__.py` import ROS packages

### Per-node tests (gps_rtk)

The **gps_rtk** node has tests under `nodes/bridges/gps_rtk/tests/`. Run from `nodes/bridges/gps_rtk`: `poetry run pytest tests/ -v` (or `poetry run poe test`). Covers: config loading and validation (`test_config.py`: minimal base/rover, rover with rtcm_server_host, invalid mode rejected, load_config from file/missing/empty); NMEA GGA parsing (`test_nmea_parser.py`: lat/lon N/S/E/W, altitude, fix quality, full sentence, RTK fixed quality 4, quality-to-NavSatStatus mapping); serial stream handling (`test_serial_handler.py`: NMEA checksum and append_checksum_if_missing, RTCM3 length parsing, CRC24Q, valid RTCM3 frame build/validation, parser emits NMEA with valid checksum, ignores invalid NMEA, discards unknown bytes).

---

**Maintenance:** Keep this README up to date when adding, removing, or changing tests. Document each new test file and each test (or test group) briefly so the test suite remains easy to navigate.

### test_rplidar_scan_watch.py

Decision logic of the RPLidar scan watchdog (`nodes/bridges/rplidar_a1/scan_watch.py`): no restart during the startup grace, restart when no scan arrives after it, no restart while scans keep arriving, restart when scans stop for the silence timeout, and the silence timeout applies once the first scan has arrived.

### test_dds_discovery_config.py

Static checks of the DDS discovery setup in Ansible: the shared `ros2_dds_env` is `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` and the discovery-server variables are gone; no node env overrides discovery (`ROS_DISCOVERY_SERVER`, `ROS_SUPER_CLIENT`, `ROS_LOCALHOST_ONLY`, `ROS_AUTOMATIC_DISCOVERY_RANGE`) and the only `ROS_STATIC_PEERS` is the client `master2master` pointing at the server (server nodes have none); the former `fastdds_discovery_server` node is `present: false` / `enabled: false` on both hosts; the unit template renders the shared env before node env with no discovery-server ordering; Steam Deck uses the client as static peer; `/etc/profile.d/ros2_dds.sh` sets localhost discovery for shells; `--all` deploys include `tasks/ros_packages_sync.yml` after the repo sync (upgrade all ros-jazzy packages together, restart running nodes if anything was upgraded); every deploy playbook (`--all` and per-node) first includes `tasks/stop_ros_nodes.yml` and finally `tasks/start_ros_nodes.yml` (present and enabled nodes started one by one, `ros2_node_start_interval_s` = 5), and web_ui is deployed first in the client `--all` playbook; secondary ethernet ports are `optional: true` in the rendered netplan, and `--all` deploys install the wait-online drop-in (`--any --timeout=30`); every `env:` key is a list (an empty `env:` parses as null and breaks deploys); logind keeps the node user's shared memory (`RemoveIPC=no`) and running `ros2-*` services restart once when that is first applied.

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
| `test_slam_playbooks_deploy_node_and_create_maps_dir` | `deploy_nodes_client.yml` and `nodes/client/slam_toolbox.yml` deploy slam_toolbox and create `/var/lib/ros2/maps` (owner `ansible_user`, 0755). |
| `test_nav2_launch_passes_repo_params_without_localization` | Nav2 launch passes the repo `params_file`, keeps `use_localization:=False`. |
| `test_nav2_has_every_server_section` | `nav2_params.yaml` has a section for every server started by Jazzy `navigation_launch.py`. |
| `test_nav2_frames` | bt_navigator, costmaps, collision_monitor, behavior_server, docking_server and route_server frames/topics. |
| `test_nav2_costmap_layers_and_footprint` | Both costmaps: 370 x 286 mm planner footprint (outer frame minus the 5 cm wheel margin), obstacle layer on `/scan`, inflation; global static layer on `/map` (transient local). |
| `test_nav2_mppi_omni_with_swerve_limits` | MPPI Omni, vx +-0.25, vy 0.25, wz 0.5, `visualize: true` (local plan on `/optimal_trajectory`). |
| `test_nav2_planner_navfn_allows_unknown` | NavFn with `allow_unknown: true`. |
| `test_nav2_velocity_smoother_matches_swerve` | Smoother limits `[0.25, 0.25, 0.5]`, odom `/odometry/filtered`. |
| `test_nav2_collision_monitor_uses_scan` | Valid polygon(s) and a `/scan` observation source. |
| `test_nav2_collision_monitor_stopbox_enabled_on_filtered_scan` | collision_monitor stays in the `cmd_vel_smoothed -> cmd_vel` chain with its core keys; `StopBox` (stop polygon, min 4 points) is enabled on the footprint-filtered scan. |
| `test_nav2_goal_tolerance_tight_enough_for_swerve` | Goal checker tolerances 0.10 m / 0.15 rad: tighter than the old 15 cm, with margin for the rotation shim's unlatched in-tolerance check during the final in-place turn. |
| `test_costmaps_add_no_margin_beyond_given_dimensions` | Both costmaps: `footprint_padding` 0 and inflation only to the planner footprint's circumscribed radius (0.24 m, `cost_scaling_factor` >= 10). |
| `test_safety_boxes_hug_the_given_dimensions` | laser_filter self box is footprint + 1 cm and the collision-monitor StopBox footprint + 2 cm, strictly outside the filter box so obstacles stay visible to it. |
| `test_costmaps_wait_for_odom_through_a_gradual_restart` | Both costmaps: `initial_transform_timeout` >= 300 s, so a deploy that restarts nav2 before the gradual ramp brings up the servos and EKF (odom TF) does not abort the bringup. |
| `test_nav2_rotates_toward_path_first_and_keeps_front_leading` | `FollowPath` is the RotationShimController around MPPI (turn to face the path first, threshold 0.3-1.0 rad, rotation speed within `wz_max`, final rotation to the goal heading, MPPI GoalAngleCritic disabled so only the shim turns to the goal heading, GoalCritic weight >= 8) and MPPI's PathAngleCritic prefers forward driving (mode 0): the lidar is partly covered at the back and right. |
| `test_nav2_readme_documents_collision_monitor_validation` | Nav2 README documents StopBox `enabled: true` on `/scan_filtered` and the live `collision_monitor_state` validation procedure. |
| `test_nav2_docking_server_configures_without_docks` | Non-empty `dock_plugins`, no docks. |
| `test_nav2_readme_documents_plan_topics` | Nav2 README names `/plan` and `/optimal_trajectory`. |
| `test_web_ui_has_map_nav_tab` | web_ui config has the `map` tab of type `map_nav` with exactly the expected fields, including the merged 3D view contract fields (`local_costmap_topic`, `base_urdf`, `base_joint_states_topic`, `arm_urdf`, `arm_joint_states_topic`, arm command topic `/filter/web_ui_joint_commands`) and the arm Trigger services `/arm/home` / `/arm/set_home`. |
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
| `test_web_ui_tab_set` | web_ui tabs are exactly map (map_nav, first), camera (`/camera_0/image_raw/compressed`), rgbd_camera, imu_graphs; arm_servos, local_nav, gps_nav, scene3d and robot_status are gone. |
| `test_web_ui_map_tab_uses_contract_fields` | The map tab carries no legacy keys (`urdf_file`, `topic`, `arm_urdf_file`, `arm_joint_topic`, `scan_topic`, `costmap_topic`), uses the frontend contract names, points `arm_home_service` / `arm_set_home_service` at the services mcp_server serves, and keeps the tile cache at `/var/cache/web_ui/tiles`. |
| `test_web_ui_tile_cache_dir_task_owned_by_node_user` | `playbooks/tasks/web_ui_tile_cache_dir.yml` creates `/var/cache/web_ui` and `/var/cache/web_ui/tiles` as directories owned by `ansible_user`. |
| `test_playbooks_create_tile_cache_before_deploying_web_ui` | `deploy_nodes_client.yml` and `nodes/client/web_ui.yml` include the tile cache task before deploying web_ui. |
| `test_mcp_server_setup_tasks_create_token_and_arm_dir` | `tasks/mcp_server_setup.yml` creates `/etc/ros2/mcp_server`, the token (`MCP_SERVER_TOKEN=` + 48-char password lookup, `force: false`, 0640, owner `ansible_user`, group `mcp-token`, `no_log`) and `/var/lib/ros2/arm` owned by the node user. |
| `test_playbooks_run_setup_before_deploying_mcp_server` | `deploy_nodes_client.yml` and `nodes/client/mcp_server.yml` deploy mcp_server and include the setup tasks before it. |
| `test_node_playbook_stops_first_and_starts_last` | The per-node playbook targets the client, starts with `stop_ros_nodes.yml`, syncs the repo and ends with `start_ros_nodes.yml`. |
| `test_unit_template_renders_environment_file_only_when_set` | The native unit template renders `EnvironmentFile=` (before `ExecStart=`) only when `node_environment_file` is non-empty. |
| `test_resolve_and_deploy_passes_environment_file` | `resolve_and_deploy.yml` passes the node type's `environment_file` (default empty) to the role. |
| `test_token_file_never_in_repo` | No tracked `token` file and no literal token value in tracked Ansible/scripts/node/test files; `.mcp.json` uses `${ROBOT_MCP_TOKEN}`. |
| `test_mcp_json_registers_robot_server` | `.mcp.json` registers server `robot` (type http, `http://client.ros2.lan:18200/mcp`, `Authorization: Bearer ${ROBOT_MCP_TOKEN}`) matching the node's port and path. |
| `test_robot_mcp_token_script` | `scripts/robot_mcp_token.sh` is executable bash (`set -euo pipefail`, passes `bash -n`), reads the token file over ssh and prints an export line. |
| `test_robot_mcp_token_script_prints_export_line` | With a fake `ssh` returning the EnvironmentFile line, the script prints `export ROBOT_MCP_TOKEN='<token>'`. |
| `test_mcp_server_node_package_layout` | `nodes/mcp_server` has a Poetry project with the `mcp` dependency and `mcp_server` package, a lock file, `__main__.py`, and a README with the `claude mcp add` setup. |
| `test_mcp_server_listed_in_nodes_readme_index` | `nodes/README.md` Layout index has an `mcp_server/` entry describing the MCP server. |
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
| `test_claude_agent_entry_after_mcp_server_and_enabled` | Present + enabled `ros2_nodes` entry directly after `mcp_server`. |
| `test_claude_agent_config_valid_and_consistent_with_mcp_server` | The entry's config validates against the node's pydantic model, binds 127.0.0.1:18300, points at mcp_server's URL and token file, and every classified tool exists in `mcp_server/tools.py`. |
| `test_effector_tools_match_mcp_server_motion_tools_when_defined` | When mcp_server defines a `MOTION_TOOLS` literal, `effector_tools` equals it (skipped until then). |
| `test_service_template_user_groups_and_nice_are_optional` | Unit template: `User=` defaults to `ansible_user`; `node_user`, `SupplementaryGroups=` and `Nice=` only when set. |
| `test_resolve_and_deploy_passes_user_groups_and_nice` | `resolve_and_deploy.yml` hands `user`, `supplementary_groups` and `nice` of the node type to the role. |
| `test_setup_tasks_read_token_from_controller_env_and_fail_clearly` | `claude_agent_setup.yml` uses `lookup('env', 'CLAUDE_CODE_OAUTH_TOKEN')`; a fail task (before the write, without `no_log`, without touching the token) tells the user to `export CLAUDE_CODE_OAUTH_TOKEN=...` and run `deploy-nodes.sh client claude_agent`. |
| `test_setup_tasks_write_env_file_0600_no_log_and_restart` | The env file task: `CLAUDE_CODE_OAUTH_TOKEN=` content, mode 0600, owner `claude_agent`, `no_log`, notifies the restart handler, only when the variable is non-empty. |
| `test_setup_tasks_no_log_on_every_task_touching_the_token` | Every task using the lookup or writing the token has `no_log: true`. |
| `test_setup_tasks_create_user_and_dirs_and_keep_existing_file` | System user `claude_agent` (nologin, home `/var/lib/claude_agent`), its directories, and a stat of the existing env file so an unset variable keeps it. |
| `test_mcp_token_readable_by_group_not_world` | `mcp_server_setup.yml` creates group `mcp-token` before the token, which is `0640` with that group. |
| `test_playbooks_run_setup_before_deploying_claude_agent_after_mcp_server` | `deploy_nodes_client.yml` and `nodes/client/claude_agent.yml`: mcp setup, then agent setup, then the deploy (after mcp_server in the full playbook). |
| `test_node_playbook_stops_first_and_starts_last` | The per-node playbook stops nodes first, starts them last, targets `client`. |
| `test_oauth_token_never_in_repo` | No tracked file contains a literal `CLAUDE_CODE_OAUTH_TOKEN=<token>`. |
| `test_claude_agent_package_layout_and_pinned_sdk` | Poetry project with the SDK pinned to an exact version, FastAPI/uvicorn/Pillow/pydantic, lock file, README, `__main__.py`, tests. |
| `test_claude_agent_readme_documents_api_and_token_deploy` | The node README lists every API route, the cap and the `export CLAUDE_CODE_OAUTH_TOKEN` deploy command. |
| `test_docs_and_lint_scripts_list_claude_agent` | `nodes/README.md`, `ansible/README.md`, the ansible-deploy skill, `scripts/lint-all-nodes.sh` and root `lint-nodes` mention the node. |
| `test_deploy_script_discovers_node_playbook` | `playbooks/nodes/client/claude_agent.yml` exists, which is how `deploy-nodes.sh` finds the node. |
| `test_every_test_is_documented_in_tests_readme` | Every `test_*` function in `test_claude_agent_config.py` is listed in this section. |
