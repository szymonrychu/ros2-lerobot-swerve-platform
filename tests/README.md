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

### `test_topic_scraper_collect.py`

Unit tests for `scripts/topic_scraper_collect.py` parser/format helpers.

| Test | Description |
|------|-------------|
| `test_parse_source` | Parses `name=url` source argument into normalized source object. |
| `test_parse_selector` | Parses `/topic:jq-filter` selector into topic and jq expression. |
| `test_topic_endpoint` | Verifies endpoint mapping (`/topic` -> `/topics/topic`). |
| `test_build_record` | Verifies merged NDJSON record fields for source/topic/timing/value. |

### Per-node tests (feetech_servos)

The **feetech_servos** node has its own test suite under `nodes/bridges/feetech_servos/tests/`. Run from that directory: `poetry run pytest tests/ -v` (or `poetry run poe test`). A `conftest.py` mocks the `st3215` module so script tests (calibrate_servos, set_servo_id) run without the hardware library installed. Covers: config loading and validation (`test_config.py`: namespace, joint_names as list of `{ name, id }` per joint—missing namespace/joint_names, device/baudrate, log_joint_updates, torque startup (`enable_torque_on_start` and `disable_torque_on_start`), and loop frequency option `control_loop_hz`; optional per-joint range mapping `source_min_steps`/`source_max_steps`/`command_min_steps`/`command_max_steps` (valid parse, defaults None, invalid/out-of-range or min > max ignored); `joint_entry_by_name`; rejects namespace with slash; rejects joint without id, without name, plain string list; rejects id out of range 0–253, duplicate servo id; accepts non-sequential IDs; `joint_names` property and `servo_id_for_joint_name`; `load_config_from_env` with default path and with env path), command range mapping (`test_command_mapping.py`: `map_position_to_steps` in `command_mapping.py`—identity 0/4095, midpoint, narrow command range, leader max→follower max, degenerate source range, clamping below/above source), startup torque write reliability (`test_bridge_startup_torque.py`: verify/retry logic and failed-servo reporting), register map and read helpers (`test_registers.py`: REGISTER_MAP entries, WRITABLE_REGISTER_NAMES, get_register_entry_by_name, read_all_registers and read_register with mock servo; EPROM vs RAM: `test_eprom_registers_marked_for_runtime_rejection`, `test_ram_registers_accepted_at_runtime` — bridge rejects EPROM writes from ROS set_register), set_servo_id script argparse and exactly-one-servo logic (`test_set_servo_id.py`), calibrate_servos subcommands and parser (`test_calibrate_servos.py`: parser default `--id` 1 for read/write; `list-registers` exits 0 and prints register map JSON; `read` with unknown register / `write` with read-only register / `limits-set` with min > max or out-of-range exit 1; calibrate JSON shape and missing-joint exit).

### Per-node tests (uvc_camera)

The **uvc_camera** node has tests under `nodes/bridges/uvc_camera/tests/`. Run from `nodes/bridges/uvc_camera`: `poetry run pytest tests/ -v` (or `poetry run poe test`). `test_config.py` covers env-based config (`get_config`): defaults, env overrides, device as path or index, stripping whitespace and fallback for empty topic/frame_id. Config lives in `config.py` (no ROS/OpenCV deps) for testability.

### Per-node tests (lerobot_teleop)

The **lerobot_teleop** node has tests under `nodes/lerobot_teleop/tests/`. Run from `nodes/lerobot_teleop`: `poetry run pytest tests/ -v` (or `poetry run poe test`). `test_config.py` covers env-based config (`get_config`): defaults, env overrides, empty env fallback. Config lives in `config.py` (no ROS deps) for testability.

### Per-node tests (swerve_drive_controller)

The **swerve_drive_controller** node has tests under `nodes/swerve_drive_controller/tests/`. Run from `nodes/swerve_drive_controller`: `poetry run pytest tests/ -v`. Covers: kinematics (`test_kinematics.py`: wheel_positions, inverse_kinematics straight/zero/sideways, forward_kinematics roundtrip, steer_angle_difference, should_zero_drive); config (`test_config.py`: load_config missing/minimal/defaults).

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
- URDF directory scanner (`test_urdf_scanner.py`: `scan_urdf_directory` finds robot.urdf, parses links/joints, detects missing mesh files, empty directory returns empty list)

### Per-node tests (gps_rtk)

The **gps_rtk** node has tests under `nodes/bridges/gps_rtk/tests/`. Run from `nodes/bridges/gps_rtk`: `poetry run pytest tests/ -v` (or `poetry run poe test`). Covers: config loading and validation (`test_config.py`: minimal base/rover, rover with rtcm_server_host, invalid mode rejected, load_config from file/missing/empty); NMEA GGA parsing (`test_nmea_parser.py`: lat/lon N/S/E/W, altitude, fix quality, full sentence, RTK fixed quality 4, quality-to-NavSatStatus mapping); serial stream handling (`test_serial_handler.py`: NMEA checksum and append_checksum_if_missing, RTCM3 length parsing, CRC24Q, valid RTCM3 frame build/validation, parser emits NMEA with valid checksum, ignores invalid NMEA, discards unknown bytes).

---

**Maintenance:** Keep this README up to date when adding, removing, or changing tests. Document each new test file and each test (or test group) briefly so the test suite remains easy to navigate.
