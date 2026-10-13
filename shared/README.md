# Shared libraries

Python code shared by multiple nodes. `shared/` is one installable uv package, `ros2-common` (import name
`ros2_common`, see [pyproject.toml](pyproject.toml)). Node code is in [nodes/](../nodes/).

See [CLAUDE.md](../CLAUDE.md): use Python 3 type hints and extend unit tests as the project grows.

A second, separate package lives in [ros2_metrics/](ros2_metrics/README.md): `ros2-metrics`, the Prometheus
exporter helper of our nodes (only `prometheus-client`).

The C++ nodes share [ros2_common_cpp/](ros2_common_cpp/), an ament_cmake package with config validation and metrics;
see [ros2_common_cpp (C++)](#ros2_common_cpp-c) below.

## Using it from a node

Add ros2-common as an editable path dependency, relative to the node directory:

```toml
[project]
dependencies = ["ros2-common"]

[tool.uv.sources]
ros2-common = { path = "../../shared", editable = true }
```

Then run `uv lock` once. On the Pi the whole repo is cloned to `ros2_repo_dest`, and the `ros2_node_deploy` role
runs `uv sync --frozen --no-dev` inside `<ros2_repo_dest>/nodes/<node>` into `/opt/ros2-nodes/<name>/venv`, so the
same `../../shared` path resolves there; nothing extra is needed in Ansible. A node one level deeper (e.g.
`nodes/bridges/<node>`) uses `../../../shared`.

## Modules

| Module | Contents |
|---|---|
| `ros2_common._utils` | `clamp(value, low, high)` |
| `ros2_common.battery` | `BatteryConfig` (pydantic: `topic` `/battery_state`, `cells` 3, `cutoff_cell_v` 2.8, `resume_cell_v` 2.9, `stale_s` 5.0; `resume_cell_v >= cutoff_cell_v`, `cells >= 1`) and `BatteryGuard` (thread-safe cut-off hysteresis: enter below `cells * cutoff_cell_v`, leave only above `cells * resume_cell_v`; no reading or older than `stale_s` = unknown = not blocked) |
| `ros2_common.camera_geometry` | Pixel <-> ground-plane maths and mount calibration (see below) |
| `ros2_common.lookups` | Camera lookup contract of mcp_server (writer) and web_ui (reader): `LookupRecord` (one `index.jsonl` line), `LookupQuery` + `NearFilter` (filters ANDed; `since` / `until` take unix seconds or ISO-8601, naive = UTC, via `parse_time`), `parse_line`, `record_summary` (record + ISO `time`) and `IndexCache` (parses the index only when its inode / mtime / size changed; corrupt lines warn once per distinct line; `appended()` lets the writer skip a re-parse) |
| `ros2_common.photos` | Photo POI contract (see below): `PhotoView`, `camera_view`, `effective_view`, `photo_as_poi`, `PhotoAdd`, `PhotoSelector`, `write_atomic`; also owns `PlanarPose` / `CameraPose` (re-exported by `lookups`) |

Used by: `nodes/mcp_server` (motion tools refused in cut-off; lookup store) and `nodes/web_ui` (lookup routes; commands rejected in cut-off; uv path source `ros2-common = { path = "../../shared", editable = true }`).

## Photo contract (`ros2_common.photos`)

A photo is a POI: one `LookupRecord` (an `index.jsonl` line) plus `<id>.jpg`. `LookupRecord` gained defaulted fields `kind` (`"photo"`), `name` (<=60), `status` (`open`/`done`/`cancelled`), `created_by` (`agent`/`user`), `updated_at` (0 = never edited) and `view`, so legacy lines parse unchanged.

- `PhotoView`: camera pose in the map (`x`, `y`, `z`, `yaw`, `pitch` positive = looking down, `hfov`, `source` `camera_mount` or `base_heading`), all finite.
- `camera_view(base_pose_map, camera_pose_base, hfov)`: base pose composed with the mount; yaw = base yaw + heading of the mount body x axis (fixed-axis xyz rpy). Without a mount: `base_heading` (z, pitch, hfov None). Without a base pose: None.
- `effective_view(rec)`: stored view, else derived from the stored base pose and mount.
- `photo_as_poi(rec)`: POI dict; `x` / `y` are None (never 0) when there is no pose.
- `PhotoAdd`: add payload (32-hex `id` required, `file == "<id>.jpg"`, no `bytes`, unknown keys rejected).
- `PhotoSelector`: exactly one of `ids` (1..500), `older_than_s` (>0), `keep_newest` (>=0); optional `created_by`, `camera`, `dry_run`.
- `write_atomic(path, data)`: fsynced temp file (0644) then rename; `mcp_server.lookup_store` re-exports it.

## Camera geometry (`ros2_common.camera_geometry`)

Conventions (ROS REP-103):

- `MountPose` is the camera BODY frame (x forward, y left, z up) in `parent_frame`, in meters and radians.
- Orientation is fixed-axis RPY like URDF `rpy` (rotate about fixed x, then y, then z).
- The OPTICAL frame (z forward, x right, y down) is the body frame times the standard camera_link -> optical
  rotation (roll -pi/2, yaw -pi/2). `optical_from_mount(T_parent_mount)` applies it, so
  `T_frame_optical = optical_from_mount(T_frame_parent @ T_parent_mount)`.
- Pixels are (u right, v down). `T_a_b` maps points of frame b into frame a.

API: models `CameraIntrinsics` (`from_calibration_yaml(path)`, `from_hfov(w, h, hfov_deg)` - an approximation: square
pixels, centred principal point, no distortion), `MountPose` (`to_matrix()`), `CameraModel` (`calibrated`);
functions `undistort_pixel`, `pixel_to_ray_camera`, `camera_ray_in_frame`, `intersect_ground` (None if parallel or
behind), `pixel_to_ground(intr, T_frame_optical, u, v, ground_z)` (None if the ray misses the floor),
`project_point_to_pixel` (None if behind the camera or off-image), `optical_from_mount`, and
`solve_mount_pose(samples, intr, initial)` returning `(MountPose, rms_px)` (RMS Euclidean reprojection error per
sample; at least 3 samples, each `(T_frame_parent, (u, v), (x, y, z))`).

Calibration procedure:

1. Put a marker at measured floor points in front of the robot (known x, y, z in the ground frame).
2. For the gripper camera, capture the marker pixel plus the parent pose (`T_frame_parent`, e.g. from FK) at several
   arm poses; for the fixed overview camera, capture several marker positions (parent pose is identity).
3. Write a samples JSON (format in the script docstring) and run
   `python scripts/solve_camera_mount.py samples.json --camera gripper`.
4. Paste the printed YAML into the Ansible node config; the RMS should be around 1 px or less.

## ros2_common_cpp (C++)

An ament_cmake package with two plain library targets, `ros2_common_cpp::config` and `ros2_common_cpp::metrics`. Neither
links rclcpp, so a node's pure-logic library can use them. Headers are in
[include/ros2_common_cpp/](ros2_common_cpp/include/ros2_common_cpp/); C++17; formatted with the root `.clang-format`
(ament style, 120 columns).

### config (`config.hpp`, yaml-cpp)

The C++ version of a pydantic model with `extra="forbid"`. A node writes one `Errors validate(const YAML::Node &)` that
returns every error (`Errors` = `std::vector<std::string>`, one `"<key>: <problem>"` per error):

| Helper | Behaviour |
|---|---|
| `require<T>(node, key, errors, mode)` | `std::optional<T>`. Missing key gives `"<key>: field required"`. A value that does not convert (including null) gives `"<key>: expected <type>"`. Types: `int`, `double`, `bool`, `std::string`, vectors of those, anything `YAML::convert` knows. |
| `get_or<T>(node, key, fallback, errors, mode)` | Optional field. Returns the fallback when the key is missing. A wrong type adds an error and returns the fallback. |
| `Coercion::LAX` (the optional `mode` of `convert`, `require`, `get_or`) | Reads a scalar as pydantic lax mode does over `yaml.safe_load`: float from bool, int, float, numeric string, sexagesimal, `.inf`/`.nan`; int from bool, int (bin, oct, hex), integral float, integer string; bool from `yes/no/on/off/true/false` words and 0/1; string only when PyYAML reads a string. Default `Coercion::STRICT` is yaml-cpp's own conversion. Helpers `lax_float`, `lax_int`, `lax_bool`, `lax_string`, `is_yaml_null` (PyYAML null words). map_guard_cpp uses LAX so it accepts exactly what its pydantic model accepts. |
| `forbid_unknown(node, allowed, errors, where = "")` | `"<where.>key: extra field not permitted"` for each key not in `allowed`. A non-mapping gives `"<where>: expected mapping"`. |
| `ge` / `gt` / `le(key, value, bound, errors)` | Range checks: `"<key>: must be >= <bound> (got <value>)"`. |
| `config_path_from_env(env_var, default_path)` | The env var when it is set and not empty, else the default (`<NAME>_CONFIG`, `/etc/ros2/<name>/config.yaml`). |
| `load_config(path, validate)` | `LoadResult{path, node, errors}`. Never exits. A missing or unreadable file, invalid YAML and a non-mapping root are errors too. An empty file counts as an empty mapping. |
| `load_config_or_exit(env_var, default_path, validate)` | Node entry point. Returns the validated root. On any error it logs every error to stderr (`config error: <path>: <error>`) and exits `EXIT_CONFIG` (78), which the unit's `RestartPreventExitStatus=78` honours. |

### metrics (`metrics.hpp`, prometheus-cpp)

`Metrics metrics("<node>")` registers `robot_node_info{node="<node>"} 1` and
`robot_node_start_time_seconds{node="<node>"}` (process start, epoch seconds). These are the same series as
`ros2_metrics.register_node_info`, so Grafana dashboards and alerts carry over.

- `counter(name, help, labels)`, `gauge(...)` and `histogram(name, help, buckets, labels)` return the prometheus-cpp
  child for those labels. The same name and labels return the same child.
- Every `Metrics` also exports the `process_*` series of prometheus_client's ProcessCollector
  (`process_cpu_seconds_total`, `process_virtual_memory_bytes`, `process_resident_memory_bytes`,
  `process_start_time_seconds`, `process_open_fds`, `process_max_fds`; unlabelled). `ProcessCollector`
  (`process_collector.hpp`) reads them from `/proc/self` on every collect, render and scrape, and returns no series
  where `/proc` cannot be read. So a C++ node and its Python counterpart have the same CPU and memory series.
- `collect()` returns every family (the registry's plus the `process_*` ones); `render()` returns them as the text
  exposition.
- `serve(port, host = "127.0.0.1")` serves `GET /metrics` in a background thread and returns the bound port. Port 0
  picks a free port.
- `resolve_metrics_port(config_port)` returns the config value, else `METRICS_PORT`, else `nullopt` (metrics off).
  A value outside 1..65535 throws `std::invalid_argument`.
- Keep each node's 191xx port from `tests/test_node_metrics_wiring.py`.

**Choice: prometheus-cpp, not a hand-written exporter over cpp-httplib.** Ubuntu 24.04 ships both as apt packages for
arm64: `prometheus-cpp-dev` 1.0.2 (core and pull, with civetweb) and `libcpp-httplib-dev` 0.14.3. This was checked
with `apt-cache policy` in the arm64 `ros:jazzy-ros-base` image. Note that the package is `prometheus-cpp-dev`, not
`libprometheus-cpp-dev`. We use prometheus-cpp because it is the established library: label escaping, histogram
buckets, the text format and the HTTP exposer are already implemented and tested upstream, so we write none of them
ourselves (CLAUDE.md, "Use the right libraries").

prometheus-cpp has no upstream rosdep rule. `package.xml` declares `<depend>prometheus-cpp</depend>`, and
`scripts/cpp-node-test.Dockerfile` maps that key to `prometheus-cpp-dev`. On the Pi, a C++ node type must list
`libyaml-cpp-dev` and `prometheus-cpp-dev` in its apt packages.

### Adding a C++ node

Layout: `nodes/<name>_cpp/`. The directory name must equal the `package.xml` `<name>`; the test script, the poe tasks
and CI address packages by it. The package contains `package.xml`, `CMakeLists.txt`, `include/<name>_cpp/`, `src/`,
`test/`, `config/` and `README.md`.

1. **Library target without rclcpp.** Put the pure logic (filters, state machines, scoring) and the config `validate`
   in it. It links `ros2_common_cpp::config` and, if it needs them, `ros2_common_cpp::metrics`. Its gtests need no ROS
   graph.
2. **Component plus standalone executable.** The rclcpp node class links the library. Register it with
   `rclcpp_components_register_node(<name>_component PLUGIN "<ns>::<Class>" EXECUTABLE <name>)`, so the node can be
   composed or run alone (`ros2 run <name>_cpp <name>`). Load the config in the constructor with
   `load_config_or_exit("<NAME>_CONFIG", "/etc/ros2/<name>/config.yaml", validate)`. Use the same env var and path as
   the Python node, so the `group_vars` config block stays the same.
3. **CMake.** In `CMakeLists.txt`:
   - `find_package(ros2_common_cpp REQUIRED)`, then link `ros2_common_cpp::config` / `ros2_common_cpp::metrics`.
   - Under `if(BUILD_TESTING)`: `set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")`,
     `find_package(ament_lint_auto REQUIRED)`, `ament_lint_auto_find_test_dependencies()`, and one `ament_add_gtest`
     per test file.
   - In `package.xml`, `test_depend` on `ament_cmake_gtest`, `ament_lint_auto`, `ament_cmake_clang_format`,
     `ament_cmake_cppcheck` and `ament_cmake_xmllint`. Do not add `ament_cmake_copyright`.
   - Copy [ros2_common_cpp/CMakeLists.txt](ros2_common_cpp/CMakeLists.txt) as the starting point.
4. **CI and tests.**
   - Add the package name to the `cpp` matrix in `.github/workflows/ci.yml`. `tests/test_ci_workflow.py` and
     `tests/test_node_lint_coverage.py` fail until you do.
   - The poe tasks `test-cpp` and `lint-cpp` find the package through their `nodes/*_cpp` glob.
   - Verify with `./scripts/cpp-node-test.sh <name>_cpp`.

### Tests (gtest, run by `./scripts/cpp-node-test.sh ros2_common_cpp`)

| File | Test | Covers |
|---|---|---|
| `test/test_config.cpp` | `ValidConfigHasNoErrors` | A valid config and a config that uses defaults give no errors. |
| | `UnknownKeyIsRejected`, `UnknownNestedKeyCarriesItsPath`, `NonMappingSectionIsRejected` | `forbid_unknown`, top level and nested (`camera.hight`). |
| | `WrongTypeIsRejected` | String for a float, float for an int, list for a string, null, bad bool. |
| | `OutOfRangeIsRejected`, `RangeBoundsAreInclusiveWhereStated` | `gt` / `ge` / `le` messages and bound inclusivity. |
| | `MissingRequiredIsRejected`, `EveryErrorIsReported` | `require` and collection of every error, not just the first. |
| | `PathFromEnvWithDefault` | Env var used when set and not empty, else the default path. |
| | `LoadConfigCollectsFileErrors`, `LoadConfigEmptyFileIsAnEmptyMapping` | Missing file, invalid YAML, non-mapping root, empty file. |
| | `LoadConfigOrExitReturnsTheValidRoot` | The success path returns the parsed root. |
| | `ConfigDeathTest.InvalidConfigLogsEveryErrorAndExits78`, `MissingFileFromDefaultPathExits78` | Exit code 78, every error logged (death tests). |
| `test/test_metrics.cpp` | `NodeInfoExactText`, `StartTimeIsNowInEpochSeconds` | The exact line `robot_node_info{node="x"} 1`, gauge types, start time close to now. |
| | `LabelValuesAreEscaped` | `"`, `\` and newline in label values are escaped. |
| | `CountersAndGaugesWithLabels` | Labelled children, the same child returned for the same labels. |
| | `HistogramBucketsAreCumulative` | `_bucket{le=...}` cumulative counts including `+Inf`, `_count` and `_sum`. |
| | `HttpGetOnLocalhostReturnsTheExposition` | `serve(0)` binds 127.0.0.1, a raw HTTP GET returns 200 with the series, and a second `serve` throws. |
| | `ResolvePortConfigThenEnvThenDisabled` | Config port, then `METRICS_PORT`, then disabled; invalid values throw. |

The ament linters (clang-format, cppcheck, xmllint) run as ctest tests of the same package.

## Tests

Run from the repo root with the root pytest (`uv run pytest tests -v`): `tests/test_shared_utils.py`,
`tests/test_shared_battery.py` (config and guard logic), `tests/test_shared_camera_geometry.py` and `tests/test_shared_package.py` (packaging and the node
path dependency). They import via `shared.ros2_common`. Documented in [tests/README.md](../tests/README.md).
