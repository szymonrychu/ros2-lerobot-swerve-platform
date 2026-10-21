# Tests

Unit tests for the project. Run with `uv run pytest` or `mise exec -- uv run pytest` from the repo root. Config: [pyproject.toml](../pyproject.toml) (`[tool.pytest.ini_options]`).

## Test files and what they cover

### `test_node_lint_coverage.py`

Root lint tasks cover every Python node and every C++ package. Node projects are discovered dynamically (`nodes/**/pyproject.toml` with a `uv.lock` beside it, ignoring `.venv`, `worktrees`, `node_modules`; a `pyproject.toml` without a lock, such as `nodes/mcp_server/mcp_core_cpp`, is a sub-package its node builds), so a new node without a lint entry fails here; C++ packages (`shared/ros2_common_cpp` plus `nodes/*_cpp`) come from `discover_cpp_package_dirs` in `test_cpp_scaffolding.py`.

| Test | Description |
|------|-------------|
| `test_node_discovery_finds_the_known_projects` | Guard against vacuous passes: discovery finds at least 18 projects, including `mcp_server` and `steamdeck_ui/bridge`. |
| `test_node_discovery_skips_sub_packages_without_a_lock` | `nodes/mcp_server/mcp_core_cpp` (no `uv.lock`) is not a node project, and every discovered node has its `uv.lock`. |
| `test_lint_all_nodes_script_covers_every_node_project` | `scripts/lint-all-nodes.sh` names every discovered node directory. |
| `test_lint_all_nodes_script_uses_uv_and_fails_loudly` | The script runs `uv run --frozen poe lint` per node, uses `set -e`, has no Poetry and no pipe/`|| true` masking. |
| `test_root_lint_nodes_task_covers_every_node_project` | The root `lint-nodes` poe task stops on the first failure and lists (or delegates to the script for) every discovered node. |
| `test_every_cpp_node_has_a_ci_row_and_is_linted` | Every C++ package has a row in the `cpp` CI matrix and is reached by the `lint-cpp` poe task (by path or the `nodes/*_cpp` glob), so a new `nodes/<x>_cpp` without a CI row fails. |
| `test_cpp_packages_are_not_python_node_projects` | C++ package discovery is non-empty and disjoint from the Python node discovery (C++ packages carry no `pyproject.toml`). |

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

### `test_ci_workflow.py`

Guards `.github/workflows/ci.yml`: uv-based, pinned, and covering every Python node project (a `pyproject.toml` with its own `uv.lock`) and every C++ package.

| Test | Description |
|---|---|
| `test_workflow_has_no_poetry` | The workflow never mentions Poetry. |
| `test_uv_pinned_in_every_job` | Every job installs uv through `astral-sh/setup-uv` pinned to version 0.11.29, except the `cpp` job, which runs everything inside its docker image. |
| `test_root_tests_job_present` | A job runs the root test suite (`uv run --frozen pytest tests`). |
| `test_node_matrix_equals_filesystem` | The node matrix equals the set of node `pyproject.toml` directories found under `nodes/`, and each entry runs `uv lock --check`, `uv run --frozen poe lint` and `uv run --frozen pytest -q`, with no duplicate entries. The count is derived from the filesystem, not hardcoded, so adding a node only touches the matrix; the marker epic added `marker_detector` to the matrix (own `pyproject.toml` and `uv.lock`) and no test edit was needed. |
| `test_uv_node_matrix_skips_cpp_packages` | The uv `nodes` matrix has no `*_cpp` entry and no `nodes/*_cpp` carries a `pyproject.toml`. |
| `test_cpp_matrix_equals_filesystem` | The `cpp` job runs on `ubuntu-latest` (amd64) with `fail-fast: false`; its `package` matrix equals the discovered C++ packages (no duplicates) and it runs `./scripts/cpp-node-test.sh ${{ matrix.package }}`. |
| `test_ansible_job_installs_collections_before_syntax_check` | The `lint-ansible` job installs `ansible/requirements.yml` collections (`ansible.posix`, used by the mount tasks) before `poe test-ansible`. |

### `test_cpp_scaffolding.py`

Contract of the C++ scaffolding: root `.clang-format`, `scripts/cpp-node-test.sh` and `scripts/cpp-node-test.Dockerfile`, the `test-cpp` / `lint-cpp` poe tasks and the lint setup of `shared/ros2_common_cpp`. Also exports `discover_cpp_package_dirs` / `discover_cpp_packages` (shared/ros2_common_cpp plus nodes/*_cpp with a package.xml) and `poe_task_body`, used by `test_ci_workflow.py` and `test_node_lint_coverage.py`. The gtests of the package itself (`shared/ros2_common_cpp/test/test_config.cpp`, `test_metrics.cpp`) run through `./scripts/cpp-node-test.sh ros2_common_cpp` and are listed in [shared/README.md](../shared/README.md).

| Test | Description |
|---|---|
| `test_cpp_package_discovery_finds_ros2_common_cpp` | Guard against vacuous passes: discovery finds `ros2_common_cpp`. |
| `test_cpp_package_dir_name_equals_package_name` | Every C++ package directory is named like its `package.xml` `<name>` (the script, poe tasks and CI address packages by it). |
| `test_clang_format_is_ament_style_at_120_columns` | `.clang-format` is the ament_clang_format style (Google base, `AccessModifierOffset -2`, `PointerAlignment Middle`, custom braces) with `ColumnLimit: 120`. |
| `test_script_usage_and_result_line_contract` | Static: executable, `set -euo pipefail`, the usage line, the `cpp-node-test: ${PKG} PASS` / `FAIL` lines, the rosdep / colcon build (`--packages-up-to`, `BUILD_TESTING=ON`, Release) / colcon test / test-result steps, a read-only repo mount and no `--platform` (native image). |
| `test_script_without_arguments_prints_usage_and_exits_2` | No argument: usage on stderr, exit 2 (docker is never touched). |
| `test_script_unknown_package_exits_2_with_known_list` | Unknown package: exit 2, every known package listed on stderr, last stdout line `cpp-node-test: <pkg> FAIL`. |
| `test_dockerfile_base_image_and_rosdep_preinstall` | The Dockerfile has exactly `FROM ros:jazzy-ros-base` and installs colcon and the packages' rosdep keys. |
| `test_poe_tasks_run_the_script_for_every_cpp_package` | `test-cpp` and `lint-cpp` (the latter with `--lint-only`) run `scripts/cpp-node-test.sh` under `set -e` over `shared/ros2_common_cpp` and `nodes/*_cpp`. |
| `test_ros2_common_cpp_declares_the_lint_packages` | `package.xml` test_depends on `ament_lint_auto`, `ament_cmake_clang_format`, `ament_cmake_cppcheck`, `ament_cmake_xmllint` and `ament_cmake_gtest`, not `ament_cmake_copyright`; build type `ament_cmake`; CMake calls `ament_lint_auto_find_test_dependencies()` and points clang-format at the root `.clang-format`. |
| `test_ros2_common_cpp_libraries_do_not_link_rclcpp` | `CMakeLists.txt` and `package.xml` never mention rclcpp; the `config` and `metrics` library targets exist. |

### `test_cpp_node_deploy.py`

In-repo C++ node deploy: the local-source colcon build of `ansible/roles/ros2_node_deploy/tasks/colcon_source_package.yml` and the `map_guard_cpp` node type. Task fields are rendered with jinja2 plus Ansible's core filters (`hash`, `regex_replace`) and the `group_vars/all.yml` build limits; the sync and hash scripts run with bash and rsync on a temporary repo and workspace.

| Test | Description |
|---|---|
| `test_missing_rsync_fails_instead_of_skipping` | The `synced` fixture fails (never skips) when rsync is missing, so the stamp-key tests cannot go quietly green on a runner without it. |
| `test_stamp_key_is_stable_for_an_unchanged_tree` | Sync again and hash again: the rendered stamp key is the same 40-char sha1, so an unchanged deploy builds nothing. |
| `test_stamp_key_changes_when_a_source_file_changes` | A same-size edit of a source file (rsync `--checksum`) changes the key. |
| `test_stamp_key_covers_paths_deletions_and_apt_versions` | A rename (same contents), a removed file (`--delete`), an apt version and the chain key each change the key; renaming back restores it. |
| `test_python_caches_do_not_reach_the_synced_tree` | A `__pycache__` in the package neither syncs nor changes the key. |
| `test_pinned_sources_keep_their_key_inputs` | A pinned source (skipped tree hash) keys on `sha1(commit + patch checksums)` as before, so existing rf2o/libcamera/MPPI stamps stay valid. |
| `test_local_only_tasks_run_only_for_local_sources` | Sync and hash run only for a `local` source, before the key and the stamp check; clone and patch only for a pinned one. |
| `test_local_builds_never_include_stop_for_build` | The only `stop_for_build.yml` include is skipped for a local source and still runs for a pinned source with a missing stamp; the file never calls systemctl. |
| `test_a_local_build_restarts_only_its_own_node` | The build notifies the restart handler and queues only `node_name`. |
| `test_local_build_runs_in_the_low_priority_scope` | Every configured local source renders `systemd-run --scope`, CPUWeight 20, IOWeight 10, nice 19, ionice idle, one worker, Release with `BUILD_TESTING=OFF`, `--base-paths` of its own tree, `MAKEFLAGS=-j2`, and a chained source sources the workspace. |
| `test_pinned_build_command_is_unchanged` | A pinned source still renders Release only, no `--base-paths`, `MAKEFLAGS=-j4`. |
| `test_cpp_node_type_is_a_local_colcon_build` | (per `_cpp` type with a Python counterpart; `head_camera_cpp` has its own identity test) native, `node_src_dir ""`, `src_paths []`, the `ros2 run` launch command, `colcon_source` = local ros2_common_cpp then the node package in `/opt/ros2-ws` with stamped apt packages that are installed, the C++ build apt packages, config path and env var, and the Python node's tier and memory keys. |
| `test_head_camera_cpp_identity_is_pinned_libcamera_then_the_two_local_trees` | `head_camera_cpp` (no Python counterpart) builds the pinned libcamera, then `ros2_common_cpp` and itself as local sources; launch command, env var and config path. |
| `test_package_xml_apt_names_maps_ros_system_and_local_keys` | `package_xml_apt_names` maps ROS keys to `ros-jazzy-<dashed>`, system rosdep keys via `ROSDEP_APT`, keeps `lib*-dev` names, drops in-tree local packages and `test_depend`. |
| `test_cpp_node_apt_packages_cover_every_in_tree_package_xml` | (per `_cpp` type, `head_camera_cpp` included; only `local` sources' package.xml are read, the pinned libcamera counts as a local package) every build/run dependency of each local source's `package.xml` that is in the tree (`ros2_common_cpp` and the `head_camera_cpp` node package) is in the type's `apt_packages`. |
| `test_ros2_common_cpp_is_a_discovered_cpp_package` | The chained `shared/ros2_common_cpp` is a C++ package found by `discover_cpp_package_dirs`. |
| `test_cpp_node_entry_is_live_with_the_python_config` | (per `_cpp` type) the `ros2_nodes` entry follows the Python node's, `present: true`, `enabled: true` (since the cut-over), the same `config: |` block (metrics port 19110). |
| `test_client_playbook_deploys_the_cpp_node_after_its_python_counterpart` | The `Deploy <name>_cpp` include follows the Python node's, with the node, build and config tags and the node tag applied. |
| `test_server_cpp_node_builds_two_local_sources_and_covers_their_package_xml` | (per server `_cpp` type, `map3d_manager_cpp`) `group_vars/server.yml` builds `ros2_common_cpp` then the node as local sources, and `apt_packages` covers every in-tree `package.xml` dependency. |
| `test_docs_describe_the_local_source_build_and_the_cpp_nodes` | `ansible/README.md`, `nodes/README.md` and the ansible-deploy skill describe the local build and both node types. |

### `test_precommit_config.py`

Guards `.pre-commit-config.yaml` against the legacy toolchain.

| Test | Description |
|---|---|
| `test_no_legacy_python_hooks` | No black, isort, autoflake or flake8 hooks are configured. |
| `test_ruff_hooks_pinned` | `ruff-check` and `ruff-format` hooks exist and are pinned to v0.15.20. |
| `test_ansible_lint_hook_matches_ansible_yaml` | The `ansible-lint` hook `files` regex matches `.yml`/`.yaml` files under `ansible/` and nothing outside it. |
| `test_ansible_lint_hook_uses_pinned_script` | The `ansible-lint` hook runs `scripts/lint-ansible.sh` (pinned version) rather than whatever `ansible-lint` is on PATH. |

### `test_shared_utils.py`

Unit tests for the shared library `shared/ros2_common/_utils.py`. Imports from repo root path.

| Test | Description |
|------|-------------|
| `test_clamp_within` | `clamp(value, low, high)` returns `value` when it lies within `[low, high]`. |
| `test_clamp_low` | `clamp` returns `low` when `value` is below the range. |
| `test_clamp_high` | `clamp` returns `high` when `value` is above the range. |
| `test_clamp_equal_bounds` | `clamp` with low == high returns that bound. |
| `test_clamp_at_bounds` | `clamp` returns value when value equals low or high. |

### `test_shared_metrics.py`

Unit tests for the metrics helper `shared/ros2_metrics` (`ros2_metrics`): `resolve_metrics_port` (config port wins over `METRICS_PORT`, env used without config, unset or empty = disabled, non-numeric / 0 / >65535 / negative raise `ValueError`); `register_node_info` exports `robot_node_info{node} 1` and `robot_node_start_time_seconds`; `start_metrics_server` returns `False` without a port and serves `/metrics` on a free localhost port with the node info.

### `test_shared_lookups.py`

Tests of `shared/ros2_common/lookups.py`, the camera lookup contract used by both `nodes/mcp_server` and `nodes/web_ui` (their own tests import it too: `test_lookup_store.py`, `test_lookup_tools.py`, `test_lookups.py`).

| Test | Description |
|------|-------------|
| `test_record_contract_is_the_shared_one` (web_ui) | web_ui uses the shared `LookupRecord` / `LookupQuery`, not a copy. |
| `test_record_contract_round_trips_every_field` | A full record survives JSON parse and dump unchanged. |
| `test_record_rejects_a_malformed_pose` | A camera pose needs three xyz and three rpy values. |
| `test_parse_line_skips_blank_and_corrupt_lines` | Blank, non-JSON and field-less lines give None. |
| `test_since_accepts_iso_string` | `since` / `until` take unix numbers, numeric strings and ISO-8601 (Z, offset, naive = UTC); garbage is rejected. |
| `test_time_filters_are_inclusive` | Records exactly at `since` / `until` match. |
| `test_filter_matrix` | camera, session, POI, note text and near filters. |
| `test_near_never_matches_a_record_without_pose` | A record without pose never matches `near`. |
| `test_record_summary_adds_iso_time` | Metadata carries the UTC ISO `time`. |
| `test_index_cache_reparses_only_when_the_index_changes` | A second read parses nothing; a rewritten index is re-parsed. |
| `test_index_cache_missing_file_is_empty` | No index file reads as empty. |
| `test_index_cache_warns_once_per_corrupt_line` | One warning per distinct corrupt line, however often it is read. |
| `test_index_cache_entries_keep_corrupt_lines` | `entries()` keeps raw corrupt lines for rewrites. |

mcp_server store tests added alongside: `test_add_after_partial_trailing_line_keeps_new_record`, `test_prune_rewrites_only_when_over_limit_by_slack`, `test_prune_removes_jpegs_without_index_line`, `test_write_atomic_fsyncs_before_replace`, `test_write_atomic_and_appended_index_are_both_world_readable`, `test_records_cached_until_index_changes`, `test_corrupt_line_warns_once_across_reads`, `test_annotate_without_change_does_not_rewrite`; tool tests `test_annotate_lookup_rejects_dangling_poi_ids` (and the poi_store-down variants), `test_since_accepts_iso_string`, `test_list_lookups_description_names_every_returned_field`, `test_text_filter_without_notes_explains_itself`. web_ui: `test_read_records_cached_until_index_changes`, `test_routes_do_not_block_the_event_loop`, `test_thumbnail_is_downscaled_server_side`, `test_since_and_until_accept_iso_strings`; frontend `api.test.ts` / `LookupList.test.tsx` cover the fetch error display and the thumbnail URL.

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

### `test_shared_scan_agreement.py`

Pure scan-to-map agreement functions of `ros2_common.scan_agreement` on a synthetic square room (ported from map_guard): endpoint filtering, TF projection, dilation, scored scans (aligned, shifted, rotated, one/three cells off), empty or missing map, too few points, points outside the map or in unknown space, rotated map origin, known cell count.

### `test_shared_poi_categories.py`

POI category tags in the note (`ros2_common.poi_categories`): constants, round trip `parse_tags(apply_tags(note, category_tags(...)))` for every category and every problem type, `category_tags` validation, `parse_tags` edge cases (first line only, unknown words ignored), `apply_tags` replacing the category while keeping survey tags and free text, no duplicate tags on repeat, tag line prepending, the length error, `has_tag` / `strip_tags`, and `check_category` accepting each valid kind / zone combination and rejecting each invalid one with the rule named.

### `test_shared_maps.py`

Named maps contract (`ros2_common.maps`): topic constants, `validate_map_name` accept/reject, `Pose2D` finiteness, `MapCommand` per-op field rules (name required for save/load/delete and empty for relocalise, `pose` load only, `note` save only, `since` / `discard_on_fail` relocalise only, unknown op / extra keys rejected), `MapResult`, `Localisation` (states, ratio 0..1), `clear_live` (no name or other field; `test_command_clear_live_takes_no_fields`), `MapSummary` (`photo_count` defaults to 0 and is >= 0; `test_summary_photo_count_defaults_and_bounds`) and `MapStatus` (`load_seq >= 0`), and JSON round trips.

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

Packaging of `shared/` as the installable uv/hatchling package `ros2-common`.

| Test | Description |
|------|-------------|
| `test_shared_pyproject_defines_ros2_common` | `shared/pyproject.toml` is named `ros2-common` and ships the `ros2_common` module. |
| `test_nodes_depend_on_shared_by_relative_develop_path` | Consumer nodes (mcp_server) declare a editable `[tool.uv.sources]` path dependency whose relative path resolves to `shared/`; the same layout holds on the Pi (whole repo in `ros2_repo_dest`, `uv sync --frozen --no-dev` run in `nodes/<node>`). |

### `test_unit_cgroups.py`

Cgroup model of the ROS2 units: robot headroom guaranteed, background constrained (no `CPUQuota` / `MemoryHigh` for robot
units, weight tiers, `MemoryLow`, `MemoryMax` safety net, `Restart=always`, host slice drop-ins). The C++ ports sit in their Python node's tier: `map_guard_cpp` normal; `head_rgbd_cpp` (head RGB-D fusion), `head_rgbd_calibration` (operator calibration tool) and `map3d_uplink` (map3d stream relay) join `head_camera_cpp` in the background tier (weights 100/100, no OOM protection); the retired types carry no tier. Every robot type has a `MemoryLow` and a `MemoryMax` of at least twice it (no pinned measurements). `test_config_error_exit_is_not_restarted` checks `RestartPreventExitStatus=78`; the `MemoryLow` sum test only bounds it by `MAX_MEMORY_LOW_SUM_MIB` (no pinned total).

### `test_config_exit_codes.py`

The mains of mcp_server and claude_agent return `os.EX_CONFIG` (78) for config/token errors, never 1 (parsed with `ast`; the unit template does not restart exit 78).

| Test | Description |
|------|-------------|
| `test_template_omits_optional_directives_when_unset` | Unset variables render no `CPUQuota`, weights, `MemoryHigh`, `MemoryLow` or `OOMScoreAdjust`; `MemoryMax=256M` stays. |
| `test_template_omits_directives_for_empty_strings` | Empty strings (unset type keys) render nothing. |
| `test_template_renders_directives_when_set` | Each directive renders with its value, `OOMScoreAdjust=0` included. |
| `test_directives_sit_in_the_service_section` | Directives land between `[Service]` and `[Install]`. |
| `test_unit_restarts_after_any_exit` | `Restart=always`, `RestartSec=15`, no `on-failure` (nav2_bringup exited 0 after a segfault and stayed down). |
| `test_stopped_and_disabled_nodes_stay_stopped` | The deploy stops nodes only through systemd (`systemctl stop` before builds, `state: stopped` for uninstall and `enabled: false`), never `systemctl kill`, so `Restart=always` does not bring them back. |
| `test_resolve_maps_cgroup_keys` | `resolve_and_deploy.yml` maps `cpu_quota`, `cpu_weight`, `io_weight`, `memory_high`, `memory_low`, `oom_score_adjust` straight through, empty when unset. |
| `test_no_memory_high_is_derived` | No `MemoryHigh` derivation on either host; an explicit `memory_high` still passes through. |
| `test_unset_cpu_quota_resolves_empty` | A type without `cpu_quota` gets no `CPUQuota` line; `memory_max` defaults to 256M. |
| `test_robot_types_carry_no_cpu_quota_or_memory_high` | No client node type sets `cpu_quota` or `memory_high`. |
| `test_tiers_have_expected_weights` | Critical 1000/1000/-900, normal robot 500/500/-500, agent-facing 300/300/-200, background 100/100/0 (the joint-state throttles) (cpu_weight/io_weight/oom_score_adjust) per node type. |
| `test_background_tier_is_below_every_robot_and_agent_tier` | The background tier has the default weight 100 and `oom_score_adjust` 0; every robot and agent tier has a higher weight and a lower `oom_score_adjust`. |
| `test_tiers_dominate_default_system_work` | Every tier is above the default weight 100 with a negative `OOMScoreAdjust`, ordered critical > normal > agent. |
| `test_tiers_do_not_overlap` | A node type is in one tier only (background included). |
| `test_every_client_type_is_tiered` | Every client type but the uninstalled `fastdds_discovery_server` is in a robot, agent or background tier. |
| `test_every_robot_type_has_memory_low_below_memory_max` | Every tiered type (background included) has `memory_low` >= 32 MiB and `memory_max` at least 2x it; mcp_server at least 1G. |
| `test_memory_low_sum_leaves_room_for_the_os` | The enabled nodes protect at most `MAX_MEMORY_LOW_SUM_MIB`, under a third of the 8 GB Pi. |
| `test_server_keeps_its_limits_without_tiers` | Server types keep `CPUQuota` / `MemoryMax`, carry no tier keys, render no `MemoryHigh` / `MemoryLow`. |
| `test_system_slice_allocates_the_robot_memory_low` | The `system.slice` drop-in template sums `memory_low` over present and enabled nodes (M and G, missing keys 0) (the client's value equals the computed sum). |
| `test_user_slice_loses_to_robot_units` | The `user.slice` drop-in renders `CPUWeight=50`, `IOWeight=50` from `group_vars/client.yml`. |
| `test_headroom_tasks_are_reversible_and_wired_into_the_client_deploy` | `cgroup_headroom.yml` installs both drop-ins, removes them when `ros2_cgroup_headroom_enabled` is false, reloads systemd; the client deploy includes it with tags `config`, `always`. |
| `test_monitoring_slice_stays_low_priority` | `monitoring.slice` keeps CPU/IO weight 20, its memory caps and a positive `OOMScoreAdjust`. |
| `test_builds_run_at_background_cpu_weight` | All three build `systemd-run` scopes pass `-p CPUWeight={{ ros2_build_cpu_weight }}` (20). |
| `test_node_readmes_state_no_cpu_quota` | No node README states a unit CPU quota (`CPUQuota=<n>`, `cpu_quota`, "CPU quota", also wrapped across two lines; reported with its line number); build scopes are described through `ros2_build_cpu_quota` / `ros2_build_cpu_weight`. |

### `test_propose_unit_limits.py`

Unit tests for `scripts/propose_unit_limits.py` (robot-headroom model: MemoryLow / MemoryMax / weights) with fixture
Prometheus JSON (no network).

| Test | Description |
|------|-------------|
| `test_settings_defaults`, `test_load_settings_from_yaml`, `test_settings_reject_bad_values` | pydantic config defaults (Mimir's Prometheus API `http://server.ros2.lan:9009/prometheus`, `host: client`, p99 CPU, p50 typical memory, `memory_max_factor` 2, minimums, default weights 500), file loading and validation. |
| `test_shipped_yaml_queries_the_robot_series_in_mimir` | `scripts/propose_unit_limits.yaml` points at the server's Mimir with `host: client`, factor 2 and the p50 typical working set. |
| `test_round_up` | Rounding to a step. |
| `test_memory_low_is_the_typical_working_set_rounded_up_to_8`, `test_memory_low_minimum` | MemoryLow = typical working set rounded up to 8 MiB, at least the minimum. |
| `test_memory_max_is_twice_the_peak_rounded_up_to_16`, `test_memory_max_keeps_a_larger_current_cap_and_the_minimum` | MemoryMax = peak x factor rounded up to 16 MiB; a larger current cap and the minimum win. |
| `test_weights_keep_the_configured_tier_or_use_the_default` | CPUWeight / IOWeight keep the type's tier, else the default. |
| `test_parse_quota_and_memory` | `50%`, `128M`, `1G` parsing. |
| `test_parse_vector_keys_by_unit_and_filters_regex`, `test_parse_vector_ignores_error_response` | Prometheus vector parsing and unit filtering. |
| `test_load_node_limits_maps_node_to_type` | Node name -> node_type -> MemoryLow / MemoryMax / weights / optional quota from a fixture group_vars, `None` when unset, MemoryMax 256M default, `present: false` skipped. |
| `test_build_queries_use_window_and_percentiles` | PromQL uses the window, the CPU percentile with the 1m rate and the typical-memory percentile. |
| `test_build_queries_select_the_configured_host` | (client, server) every query selects `host="<host>"`. |
| `test_build_rows_proposals_and_flags` | Proposals and the `oom risk`, `tight max`, `unprotected` and (quota only) `throttle risk` flags. |
| `test_build_rows_reports_no_data_and_unknown_units` | Units without a full set of series are `no data` with no proposal. |
| `test_memory_low_sum_counts_rows_with_a_proposal` | The MemoryLow sum skips rows without data. |
| `test_render_table_shows_columns_and_the_memory_low_sum`, `test_render_yaml_roundtrips` | Table columns (no CPUQuota / MemoryHigh) with the closing sum line, and YAML output (`memory_low`, `memory_max`, `cpu_weight`, `io_weight`). |

### `test_servo_neutral_config.py`

Servo torque-off and neutral keys in `ansible/group_vars/client.yml` and `server.yml`: the client bridges, command sources, mcp_server and web_ui name the same bridge groups.

| Test | Description |
|------|-------------|
| `test_web_ui_groups_are_bridge_groups_and_neutral_matches` | Every web_ui `calibration.servo_groups` namespace is a group of a client feetech bridge (arm = follower on lerobot_follower, base = swerve_drive and head on base_servos) and its `neutral` equals that group's `neutral_enabled`. |
| `test_base_neutral_joints_are_the_four_steering_servos` | The base neutral joints are exactly fl/fr/rl/rr_steer; no velocity (wheel) joint. |
| `test_follower_neutral_excludes_only_the_gripper` | lerobot_follower has `neutral_enabled: true` and `neutral_exclude_joints: [gripper]`. |
| `test_stale_marker_path_is_identical_everywhere` | The stale marker path is the same in lerobot_follower, mcp_server `arm.calibration_stale_marker` and web_ui, under `/var/lib/ros2/arm`. |
| `test_torque_state_topics_match_bridge_groups` | filter_node, swerve_controller and mcp_server torque state topics equal `/{namespace}/torque_state` of a configured bridge group; `servo_torque_stale_s` is set; both bridges set `torque_state_interval_s: 1.0`. |
| `test_leader_bridge_has_no_neutral_keys` | The server.yml lerobot_leader bridge has no `neutral_*` key (the leader keeps its zero). |
| `test_web_ui_command_topics_match_configured_topics` | web_ui group `command_topics` equal the arm tab `arm_command_topic`, the head tab `head_command_topic` and the map `goal_topic`. |
| `test_arm_state_directory_created_on_client` | `playbooks/tasks/mcp_server_setup.yml` creates `/var/lib/ros2/arm` (home pose and stale marker). |
| `test_tests_readme_lists_this_file` | This README has a section for the test file. |

### `test_calibrate_retry_threshold.py`

Unit tests for `scripts/calibrate_retry_threshold.py` (Brier score, reliability bins, Wilson-bound threshold proposal) with fixture retry records (no network).

| Test | Description |
|------|-------------|
| `test_settings_defaults_and_shipped_yaml` | Shipped `scripts/calibrate_retry_threshold.yaml` loads: 10 bins, target 0.5, confidence 0.95, min_samples 30, min_bin_samples 5, the web_ui proxy URL and no file source. |
| `test_settings_validation_errors` | Config rejected: neither or both of `source_url` / `source_file`, `bins` < 2, target or confidence outside (0, 1), non-positive sample minimums, unknown keys. |
| `test_load_settings_from_file` | YAML file loading into `Settings`. |
| `test_parse_records_skips_unknown_versions_and_invalid` | `schema_version` != 1 and invalid records are skipped and counted separately. |
| `test_fetch_records_from_file_skips_blank_and_corrupt_lines` | JSONL source: blank and corrupt lines ignored. |
| `test_fetch_records_from_url_uses_timeout` | HTTP source (httpx MockTransport): reads `records` from the URL with a 10 s timeout. |
| `test_filter_by_model_and_since_ts` | `since_ts` and `model` filters, alone and combined. |
| `test_bin_index_puts_one_in_last_bin` | Bin edges; p = 1.0 falls into the last bin. |
| `test_count_bins_total_observed_unobserved` | Per-bin total / observed / unobserved counts. |
| `test_brier_matches_hand_computed_and_excludes_unobserved` | Brier against a hand-computed value (skipped counts as failure), base-rate reference Brier, unobserved excluded. |
| `test_brier_empty_is_none` | No observed records gives no score. |
| `test_wilson_interval_matches_scipy` | `wilson_interval` equals scipy's Wilson `proportion_ci`. |
| `test_reliability_bins_and_ece` | Reliability bins (n, mean p, success rate, Wilson bounds against scipy, empty bins) and ECE on a calibrated fixture. |
| `test_reliability_bin_flags_low_n` | Bins below `min_bin_samples` are flagged. |
| `test_ece_nonzero_for_miscalibration` | ECE equals the gap for an over-confident fixture. |
| `test_eai_breakdown_success_rates` | Per-`eai_error_type` n and success rate, unobserved excluded. |
| `test_proposal_on_calibrated_set` | Smallest grid threshold on a synthetic well-calibrated set; a higher target moves the proposal up. |
| `test_proposal_never_below_min_observed_p` | A proposal is never below the minimum observed p. |
| `test_proposal_insufficient_data` | Below `min_samples`, an all-failure set and an empty set give no proposal. |
| `test_proposal_requires_min_samples_in_subset` | A subset smaller than `min_samples` never qualifies. |
| `test_analyze_result_shape_and_current_threshold` | Result counts, skip counts, Brier, bins, JSON-serializable; current threshold = most recent record's. |
| `test_analyze_insufficient_keeps_current` | No proposal keeps the current threshold. |
| `test_build_snippet_format` | Paste-ready `retry_success_threshold: <t>  # calibrate_retry_threshold.py <date> n=<n> Brier <b>` line. |
| `test_build_snippet_insufficient` | `insufficient data, keep <current>` message. |
| `test_render_report_mentions_sections_and_censoring` | Report shows Brier, ECE, error types, skip count, snippet and the censoring note. |
| `test_main_prints_and_writes_only_json` | `main` prints the snippet and writes only the `--json` file. |
| `test_main_insufficient_data_message` | `main` prints `insufficient data, keep 0.5` for a thin source. |
| `test_main_empty_source` | An empty source reports `keep unknown` instead of crashing. |

### `test_eval_testset.py`

Unit tests for `scripts/eval/testset.py` (fixed 10x3 evaluation test set).

| Test | Description |
|------|-------------|
| `test_valid_testset_loads` | A 10-object, 3-placement set validates; `TOTAL_CASES` is 30. |
| `test_wrong_object_count_rejected` | 9 or 11 objects are rejected ("exactly 10 objects"). |
| `test_wrong_placement_count_rejected` | 2 or 4 placements are rejected ("exactly 3 placements"). |
| `test_duplicate_object_ids_rejected`, `test_duplicate_placement_ids_rejected` | Duplicate ids are rejected. |
| `test_no_arm_only_placement_rejected` | A set without an `arm_only` placement is rejected. |
| `test_unknown_key_rejected` | Unknown top-level keys are rejected. |
| `test_template_with_unknown_placeholder_rejected` | `prompt_template` placeholders other than object_name / placement_description / place_description are rejected. |
| `test_full_mode_expands_to_30_object_major` | `full` gives 30 unique cases in object-major order. |
| `test_arm_only_expansion_gives_20_arm_only_cases` | `arm_only` gives the 20 arm-only cases; case key `<object>/<placement>`. |
| `test_unknown_mode_rejected` | An unknown mode raises. |
| `test_render_prompt_full_and_arm_only` | Prompt rendering; arm-only appends the suffix. |
| `test_render_prompt_unknown_case_rejected` | An unknown object id raises `KeyError`. |
| `test_shipped_v1_testset` | `scripts/eval/testset_v1.yaml` validates: 10 objects, 2 arm-only and 1 navigation placement, 20 arm-only cases, no TODO-like text, non-empty descriptions, rendered prompt. |
| `test_load_testset_from_file` | YAML loading. |

### `test_eval_stages.py`

Unit tests for `scripts/eval/stages.py` with event fixtures shaped like real claude_agent events (tool_call / tool_result with GraspResult JSON text and image blocks, phase_started / phase_completed with attempt and failure, turn_end with instruction_id).

| Test | Description |
|------|-------------|
| `test_happy_path_claims_and_grasp_attempts` | Grasp missed then grasped: grasp attempts 2, locate / grasp / verify / place ok, navigate not reached, instruction id, turn_end and lookup ids. |
| `test_per_phase_stage_attempts_and_failures` | Phase-to-stage mapping, attempts from phase_completed, failure records of failed closes. |
| `test_phase_names_are_ignored` | A phase named "navigate" that only grasps maps to grasp. |
| `test_no_nav_calls_is_navigate_not_reached` | No nav calls: navigate not reached; no grasp: grasp / verify / place not reached. |
| `test_navigate_last_result_decides`, `test_navigate_call_without_result_fails` | The last nav result decides; a nav call without a result fails. |
| `test_locate_fails_without_grasp_or_planned_plan`, `test_locate_ok_with_planned_plan_and_no_grasp`, `test_locate_not_reached_when_grasp_without_perception` | Locate rule: an infeasible plan fails, a planned plan_grasp is ok, no perception before the grasp is not reached. |
| `test_grasp_unverified_is_ok_and_missed_last_fails`, `test_grasp_error_result_fails` | Grasp rule on the last outcome. |
| `test_verify_needs_check_after_last_grasp_and_phase_close`, `test_verify_front_camera_is_not_a_check`, `test_verify_phase_closed_failed_is_fail` | Verify needs a gripper image or get_arm_state after the last grasp and a phase close; front images do not count; a failed close fails. |
| `test_verify_release_before_place_stage_fails`, `test_verify_and_place_in_one_phase_is_ok` | A release in a verify-mapped phase fails verify; check plus release in one phase maps to place (tie rule) and both are ok. |
| `test_release_then_done_gives_place_ok`, `test_release_with_phase_failed_or_open_fails`, `test_release_error_fails_place`, `test_retry_release_before_last_grasp_is_not_place` | Place rule: released and phase done; a failed or never-closed phase or an error fails; a retry release before the last grasp is not place. |
| `test_stage_mapping_tie_goes_to_later_stage` | Most stage tools wins, ties go to the later stage, a phase without stage tools has no stage. |
| `test_loop_reentry_keeps_max_attempt_per_phase_id` | Re-entries of one phase id: highest attempt, last outcome, every failed close listed (untyped as None). |
| `test_runner_direct_calls_are_ignored` | Runner calls (events with `source`) do not count. |
| `test_events_are_sorted_by_seq_and_turn_end_matches_instruction` | Events in any order are sorted by seq. |
| `test_no_turn_end_and_no_phases` | Works without turn_end or phases. |
| `test_extract_helpers` | `extract_lookup_ids` (32-hex only) and `result_outcome` over image plus JSON text blocks. |

### `test_eval_report.py`

Unit tests for `scripts/eval/report.py` (TrialRecord and the evaluation report).

| Test | Description |
|------|-------------|
| `test_trial_record_validation` | JSON round trip; unknown stage, unknown status, unknown first failed stage and trial 0 rejected. |
| `test_success_at_1_and_at_k_with_stop_on_success` | success@1 and success@k: a case stopped after a success counts, a case with fewer than k trials and no success is incomplete and left out. |
| `test_success_at_k_ignores_trials_beyond_k` | Trials after k do not count. |
| `test_stage_conditional_success_and_first_failure_share` | Per stage attempted, success conditional on reaching it and share of first failures. |
| `test_first_failure_share_none_without_failures` | No failed trial: share is None. |
| `test_agreement_and_false_ok_rate` | Claim vs label accuracy and false "ok" rate per stage; n/a labels not compared. |
| `test_mean_attempts_retries_and_failure_histograms` | Mean attempts per phase overall and per stage (unmapped bucket), mean automatic retries per trial, failure_type / eai_error_type histograms and untyped failed closes. |
| `test_cost_and_time_per_trial` | Cost total and mean, duration mean and max, mean turns. |
| `test_coverage_header_states_cases_of_30` | Coverage counts and the "Coverage: 20 of 30 cases run" header line. |
| `test_empty_report_and_json` | An empty run gives None rates and is JSON-serializable. |
| `test_build_rejects_bad_k` | k < 1 raises. |
| `test_render_text_sections` | The text report has every section. |
| `test_report_counts_harness_timeouts_apart_from_interrupts` | `timed_out` trials are counted in `timeouts` (and the text report) separately from operator or agent interrupts. |
| `test_trial_record_from_derivation` | A TrialRecord built from `stages.derive` carries instruction id, turn_end fields, claims, phases and lookup ids. |

### `test_eval_workflow_checks.py`

Unit tests for `scripts/eval/workflow_checks.py` (workflow adherence of a trial) and the workflow section of the report.

| Test | Description |
|------|-------------|
| `test_derives_workflow_skipped_retrospective_survey_writes_and_skills` | Workflow name (dry runs ignored), skipped mandatory ids from the turn_end job_end, retrospective author, survey POI writes (flagged writes plus mark_survey_spot) and loaded skills. |
| `test_free_job_has_no_workflow_and_runner_retrospective_is_kept` | A free job gives no workflow and no skipped list; a runner retrospective is reported as such. |
| `test_runner_sourced_calls_are_ignored` | Tool calls with a `source` do not count. |
| `test_pick_and_place_trace_derives_empty` | A pick-and-place trace has no workflow, skills or survey writes. |
| `test_old_trial_lines_without_the_field_load` | A trials.jsonl line without `workflow` loads with None. |
| `test_report_aggregates_workflow_section` | Trials per workflow, share without skipped mandatory steps, share with an agent retrospective, mean survey POI writes, text section. |
| `test_report_workflow_section_empty_without_derivations` | No derivations give zero trials and None rates. |

### `test_eval_workflow_testset.py`

Unit tests for the workflow test set (`scripts/eval/testset_workflows_v1.yaml`, `testset.py` kind dispatch) and the workflow trials of `run.py`, using the fake claude_agent of `test_eval_run.py`.

| Test | Description |
|------|-------------|
| `test_shipped_workflows_yaml_loads` | The shipped YAML loads as a WorkflowTestSet with explore_room, survey_here (discovery), drive_to_poi, drive_to_object (drive_to_target) and their prompts. |
| `test_kind_dispatch_and_v1_unchanged` | `load_any_testset` returns the v1 pick-and-place set with or without a `kind` key and rejects an unknown kind. |
| `test_workflow_testset_rejects_extra_and_duplicate_ids` | Extra fields and duplicate case ids are refused. |
| `test_run_trial_records_derivation_and_expected_match` | A workflow trial sends the case prompt, prints the derived workflow against the expected one, records the human label and saves events. |
| `test_wrong_or_missing_workflow_is_a_mismatch` | Another workflow or none gives `workflow_match` false. |
| `test_arm_only_is_refused_for_workflow_testsets` | `run` and `run_trial` refuse mode arm_only with a message naming `mode: full`, before any request. |
| `test_run_records_trials_and_report_for_workflow_cases` | `run` records WorkflowTrialRecord lines, honours stop_on_success and writes the per-case report. |
| `test_run_yaml_documents_the_workflow_testset` | `run.yaml` comments how to select the workflow test set. |

### `test_eval_run.py`

Unit tests for the evaluation driver (`scripts/eval/client.py`, `labels.py`, `run.py`). A fake claude_agent (state, reset, message, history, stop) runs behind `httpx.MockTransport`, the operator is an injected input function and time is a fake clock advanced by sleep, so no network, robot or terminal is involved. Agent events reuse the `test_eval_stages.py` trace.

| Test | Description |
|------|-------------|
| `test_ask_collects_success_stage_labels_and_note` | Labelling: object in target first, one key per stage, note; first failed stage is the first `n`. |
| `test_ask_reasks_on_invalid_key` | Invalid keys (and an empty answer in full mode) are asked again. |
| `test_ask_navigate_defaults_to_na_only_in_arm_only` | Enter accepts `x` for navigate in arm-only mode only. |
| `test_client_reset_retries_once_after_delay_on_409` | A 409 reset is retried once after the 5 s delay. |
| `test_client_reset_gives_up_after_second_409` | A second 409 raises AgentBusyError. |
| `test_client_message_deny_tools_and_errors` | `deny_tools` sent only when given; 503 raises BatteryBlockError with the agent message, 409 AgentBusyError, other statuses AgentError. |
| `test_client_history_since_pages_back_to_the_seq` | History polling pages backwards to the seq (1200 events) and returns nothing when up to date. |
| `test_client_state_stop_and_bounded_timeouts` | State and stop calls; connect and read timeouts are set. |
| `test_successful_trial_is_recorded_with_events_report_and_deny_tools` | A full trial: deny_tools in arm-only, reset, raw events file, TrialRecord line, claimed stages and lookup ids printed, report.json, stop_on_success. |
| `test_full_mode_sends_no_deny_tools` | Full mode sends no `deny_tools`. |
| `test_failed_trials_run_k_times_without_stop` | Failed trials run all k times with first failed stage and note recorded. |
| `test_stop_on_success_false_keeps_running` | Without stop_on_success every trial runs. |
| `test_skip_and_quit_at_setup` | `s` skips a trial, `q` quits; the report is still written and nothing was sent. |
| `test_quit_after_a_trial_writes_report_with_it` | The report on `q` includes recorded trials and planned cases. |
| `test_busy_agent_waits_then_runs` | A busy agent is waited for, then the trial runs. |
| `test_busy_agent_past_trial_timeout_aborts_without_reset` | Busy past trial_timeout_s aborts without reset or message. |
| `test_battery_block_aborts_with_message_and_keeps_recorded_trials` | A 503 aborts with the message and exit code 1; recorded trials stay. |
| `test_reset_busy_twice_aborts` | A reset still busy after the retry aborts. |
| `test_no_reset_when_disabled` | reset_between_trials false never resets. |
| `test_trial_timeout_posts_stop_and_waits_60s_more` | On timeout POST stop; the interrupted turn_end is recorded and the trial has `timed_out` true (a normal trial has false). |
| `test_trial_without_turn_end_after_stop_is_still_recorded` | No turn_end 60 s after stop: the trial is recorded with status None and `timed_out` true. |
| `test_resume_continues_with_remaining_trials` | `--resume` skips recorded trials and continues with the rest. |
| `test_resume_unknown_run_id_fails` | Resuming a missing run raises. |
| `test_run_config_defaults_and_validation` | RunConfig defaults (arm_only, `.logs/eval`, deny tools, k, timeout) and validation. |
| `test_shipped_run_yaml_loads` | `scripts/eval/run.yaml` loads. |
| `test_unknown_case_in_subset_is_rejected` | A subset case the mode does not run is rejected. |
| `test_main_rejects_missing_config` | `main` rejects an invalid config file. |

### `test_topic_scraper_config.py`

Client topic_scraper_api config (`ansible/group_vars/client.yml`).

| Test | Covers |
|---|---|
| `test_scraper_does_not_subscribe_raw_images` | Raw `sensor_msgs/msg/Image` is not in `allowed_types` (about 20 MB/s of raw frames cost about 0.4 cores to deserialize); `CompressedImage` stays. |
| `test_scraper_disabled_state_is_documented` | CLAUDE.md and scripts/README.md say the scraper is disabled by default and how to enable it (`enabled: true`), and the ansible README server table marks it `(disabled)`. |
| `test_scraper_is_disabled_on_both_hosts_and_not_scraped` | topic_scraper_api is installed (`present: true`) but `enabled: false` on the robot and the server, and Alloy has no scrape target for it. |

### `test_topic_scraper_collect.py`

Unit tests for `scripts/topic_scraper_collect.py` parser/format helpers.

| Test | Description |
|------|-------------|
| `test_parse_source` | Parses `name=url` source argument into normalized source object. |
| `test_parse_selector` | Parses `/topic:jq-filter` selector into topic and jq expression. |
| `test_topic_endpoint` | Verifies endpoint mapping (`/topic` -> `/topics/topic`). |
| `test_build_record` | Verifies merged NDJSON record fields for source/topic/timing/value. |
| `test_connection_refused_prints_the_enable_hint` | `http_get_json` against a closed port prints the "disabled by default / enabled: true" hint on stderr. |
| `test_parse_json_maps_null_and_legacy_non_finite_constants_to_none` | `parse_json` keeps null as None and turns legacy NaN/Infinity into None. |
| `test_record_with_null_value_is_strict_json` | `format_record` output with null values is strict JSON (no NaN/Infinity). |

### `test_head_state_publisher_config.py`

The `head_state_publisher` node (robot_state_publisher on `nodes/web_ui/urdf/head.urdf`) in the client Ansible vars and playbook. The launch file is parsed with `ast`, no ROS import.

| Test | What it covers |
|---|---|
| `test_node_type_fields_and_tier` | Native launch command, empty `node_src_dir`, `src_paths`, apt package and the mid tier (32M/128M, weights 500, OOM -500). |
| `test_entry_present_enabled_after_base_servos` | The `ros2_nodes` entry is present, enabled and listed after `base_servos`. |
| `test_launch_file_runs_robot_state_publisher_with_remaps` | The launch file starts robot_state_publisher as `head_state_publisher` and remaps `joint_states` and `robot_description` to `/head/*`. |
| `test_launch_file_reads_head_urdf_relative_to_itself` | The URDF path comes from `__file__` and names `web_ui/urdf/head.urdf`. |
| `test_playbook_step_exists_with_tags` | `Deploy head_state_publisher` exists with its node tag and `apt`. |
| `test_readme_documents_frames_and_check` | The node README names `/tf_static` and the `tf2_echo` check. |

### `test_head_tof_joint_override.py`

The applied `tof_joint` override in the head_state_publisher launch (`head_description.py`, stdlib only).

| Test | What it covers |
|---|---|
| `test_override_rewrites_only_tof_joint` | The tof_joint origin carries the JSON values; every other line is unchanged. |
| `test_missing_override_keeps_repo_urdf` | No override file returns the repo URDF. |
| `test_invalid_override_ignored_with_warning` | Invalid JSON and wrong shapes return the repo URDF and log a warning naming the file. |
| `test_head_description_is_stdlib_only` | `head_description.py` imports only standard library modules. |
| `test_launch_file_references_override_pieces` | The launch file names `head_description`, the applied path and `urdf_text.py`. |

### `test_head_config_consistency.py`

Cross-component pin of the pan/tilt head: the same values are copied into the `base_servos` head group, `nodes/web_ui/urdf/head.urdf`, the mcp_server `head:` block and the web_ui head tab, and must never disagree. The mcp_server and web_ui code defaults are read from their config modules.

| Test | What it covers |
|---|---|
| `test_home_pose_is_equal_everywhere_and_inside_the_limits` | Home (pan 0, tilt 30 deg down) equal within 1e-3 rad in the bridge `startup_position_rad`, mcp_server `head.home_*` (client.yml and default) and web_ui `head_home_deg` (client.yml, default.yaml, code default), and inside the URDF limits. |
| `test_ranges_agree_across_urdf_bridge_web_ui_and_mcp_server` | URDF limits equal the bridge `command_min/max_steps` (4096 steps/turn around 2048, honouring `inverted`), the web_ui `head_*_range_deg` and the mcp_server limits read from its configured URDF path. |
| `test_joint_names_agree_across_urdf_bridge_and_mcp_server` | `head_pan`, `head_tilt` in the URDF, the bridge group and mcp_server `head.joint_names`. |
| `test_topics_agree_across_bridge_mcp_server_web_ui_and_launch` | `/head/joint_states` and `/head/joint_commands` in the bridge namespace, mcp_server, web_ui (head and map_nav tabs) and the head_state_publisher launch remap. |
| `test_rgbd_topics_agree_between_client_yml_and_default_yaml` | The web_ui head tab `rgbd_rgb_topic` / `rgbd_depth_topic` and the map tab `rgbd_cloud_topic` are `/head_rgbd/rgb/image_raw/compressed`, `/head_rgbd/depth/image_raw` and `/head_rgbd/points` in client.yml and default.yaml (an unknown key would be dropped silently by the tab model). |

### `test_head_servo_config.py`

The pan/tilt head servo group in `ansible/group_vars/client.yml`: an `extra_groups` entry `head` on `base_servos` (base bus, never the arm bridge). Step ranges must equal the limits of `nodes/web_ui/urdf/head.urdf` (4096 steps per turn around 2048, +-1 step).

| Test | What it covers |
|---|---|
| `test_head_group_exists_only_on_base_servos` | `head_pan` and `head_tilt` are in the `head` group of base_servos and nowhere on lerobot_follower. |
| `test_head_joint_ids_are_the_defaults` | IDs 20 (pan) and 21 (tilt). |
| `test_joint_names_and_ids_are_unique_on_the_base_bus` | No duplicate joint name or servo ID across the swerve and head groups. |
| `test_head_steps_equal_the_urdf_limits` | `command_min_steps`/`command_max_steps` equal the URDF joint limits converted to steps (swapped if `inverted`). |
| `test_head_caps_are_present_and_positive` | `max_speed_rad_s` and `max_acceleration_rad_s2` set and > 0. |
| `test_head_startup_positions_are_inside_the_limits` | Home pose is inside the URDF limits and the command steps. |
| `test_head_pan_and_tilt_servos_are_mounted_reversed` | Both `head_pan` (found 2026-10-21: +30 deg turned right) and `head_tilt` (found 2026-10-20) are `inverted: true`; tilt steps 1707..2389 and the 0.5236 rad home maps to step 1707. |
| `test_head_pan_steps_are_symmetric_around_centre` | Pan min + max steps = 2 * 2048, so inverting only flips the sign. |
| `test_autonomy_and_web_ui_may_command_the_head_directly` | `autonomy` and `web_ui` are in `direct_command_sources`. |

### Per-node tests (feetech_servos)

The **feetech_servos** node has its own test suite under `nodes/bridges/feetech_servos/tests/`. Run from that directory: `uv run pytest tests/ -v` (or `uv run poe test`). A `conftest.py` mocks the `st3215` package entry (`ST3215` opens a serial port) so script tests (calibrate_servos, set_servo_id) run without hardware; the pure protocol modules (`protocol_packet_handler`, `GroupSyncRead`, `GroupSyncWrite`, `values`) are imported first and stay the real library code. Covers: config loading and validation (`test_config.py`: namespace, joint_names as list of `{ name, id }` per joint—missing namespace/joint_names, device/baudrate, log_joint_updates, torque startup (`enable_torque_on_start` and `disable_torque_on_start`), and loop frequency option `control_loop_hz`; optional per-joint range mapping `source_min_steps`/`source_max_steps`/`command_min_steps`/`command_max_steps` (valid parse, defaults None, invalid/out-of-range or min > max ignored); `joint_entry_by_name`; rejects namespace with slash; rejects joint without id, without name, plain string list; rejects id out of range 0–253, duplicate servo id; accepts non-sequential IDs; `joint_names` property and `servo_id_for_joint_name`; `load_config_from_env` with default path and with env path), command range mapping (`test_command_mapping.py`: `map_position_to_steps` in `command_mapping.py`—identity 0/4095, midpoint, narrow command range, leader max→follower max, degenerate source range, clamping below/above source), startup torque write reliability (`test_bridge_startup_torque.py`: verify/retry logic and failed-servo reporting), register map and read helpers (`test_registers.py`: REGISTER_MAP entries, WRITABLE_REGISTER_NAMES, get_register_entry_by_name, read_all_registers (block reads, a failed block omits its registers) and read_register with mock servo; `register_blocks` covers every map entry in two blocks (3-49, 55-70) and splits at gaps wider than `REGISTER_BLOCK_MAX_GAP`; EPROM vs RAM: `test_eprom_registers_marked_for_runtime_rejection`, `test_ram_registers_accepted_at_runtime` — bridge rejects EPROM writes from ROS set_register; `torque_limit` (STS3215 Torque_Limit, address 48, 2 bytes) is a writable RAM register, `runtime_writable_entry` accepts it and rejects EPROM / read-only / lock / unknown names, and a runtime write is 2 bytes at address 48 without an EPROM unlock), set_servo_id script argparse and exactly-one-servo logic (`test_set_servo_id.py`), calibrate_servos subcommands and parser (`test_calibrate_servos.py`: parser default `--id` 1 for read/write; `list-registers` exits 0 and prints register map JSON; `read` with unknown register / `write` with read-only register / `limits-set` with min > max or out-of-range exit 1; calibrate JSON shape and missing-joint exit; `center` defines the middle and writes symmetric limits, refuses to write limits when the servo does not read ~2048, rejects out-of-range spans). Battery: `test_battery.py` covers `raw_to_volts` (0.1 V units, 0/None rejected), `BatteryMonitor` (one servo per interval round-robin, disabled at interval 0, median of fresh readings, failed/zero reads ignored, stale readings dropped) and `battery_fields` (BatteryState field values with NaN/UNKNOWN); `test_config.py` also covers the `battery_*` fields and defaults. Swerve additions: `test_config.py` also covers per-joint `mode` / `inverted` / `max_velocity_rad_s`, invalid mode rejection, `extra_groups` (parsing, duplicate IDs across groups, duplicate namespaces, `servo_id_for_joint_name` across groups) and `velocity_command_timeout_s`; `test_load_decode.py` covers `decode_present_load` (Present_Load sign-magnitude: magnitude bits 0-9, direction bit 10, e.g. raw 1056 = -32; sign flipped for inverted joints); `test_wheel_mode.py` covers sign-magnitude velocity encode/decode (clamp, inversion, roundtrip), inverted position conversion, sync read with per-servo fallback, partial replies kept on a failed transaction (`sync_read.py`), the velocity watchdog and record-only-on-successful-write (`velocity_watchdog.py`, so a failed stop is retried), and the incremental one-servo-per-cycle register dump (`register_dump.py`). Metrics: `test_metrics.py` covers the `metrics_port` config field (default None, parsed), `servo_bus_read_failures_total` / `servo_bus_up` per read cycle (`record_read_cycle`), the per-joint load / temperature / voltage / torque gauges from a register dump (inverted load sign, unread registers stay unset), `servo_cycle_duration_seconds` via `end_cycle`, `servo_commands_total` in `BridgeCycle.write_pending_commands`, and `servo_set_register_failures_total` for every reason through `apply_set_register` (`set_register.py`). Serial wait: `test_serial_wait.py` runs the real st3215 packet handler against a pty pair (fake serial bus at 115200 baud, macOS ptys reject 1 Mbaud): `use_blocking_reads` sets a positive pyserial timeout (`SERIAL_READ_TIMEOUT_S`, the library default is 0); a read from a missing servo still returns `COMM_RX_TIMEOUT` within the library's packet timeout (+ one read timeout) with at most 200 `readPort` calls and CPU time below half the wall time (the busy-wait made ~58000 calls); a present servo's reply is decoded and returned promptly; no-op without an open port; `run_bridge` applies it to the bus it opens.

Bus transaction shape (`tests/sts_bus.py` emulates an STS bus behind the real st3215 packet handler: parses every packet, checks the checksum, applies READ / WRITE / SYNC_READ / SYNC_WRITE to in-memory register tables, counts transactions, bytes and reply timeouts and estimates wire time at 1 Mbaud plus 1 ms per transaction and 50 ms per timeout). `test_bus_io.py`: the register dump reads one servo in at most 3 transactions (2 block reads; before ~43 single reads), decodes a recorded gripper register table and 20 random tables to exactly the values and JSON (key order included) of the single-read decode, a missing servo gives `{}` within 3 transactions, a servo error byte omits the registers as before, and one dump step on top of a normal cycle stays within 2x the normal cycle's bus time; goal positions of a group go out as one SYNC_WRITE with no reply bytes, the decoded packet carries exactly the steps the single `write2ByteTxRx` path would write (servo tables identical), arm and steering targets share the packet while wheel `goal_speed` keeps one acknowledged write each, unchanged goals put nothing on the bus, a sync write the library failed to send is not cached (the same goal is sent again next time), an unchanged goal is rewritten once its last sync write is `goal_refresh_interval_s` old (not before; both refreshed goals share one packet), a sync write a servo missed (absent from the bus for that packet) is corrected by the refresh while the same goal keeps being commanded, an emulated servo reset (goal register back to 2048) is corrected within refresh interval / 10 ms + 1 cycles, interval 0 disables the refresh, and a worst-case follower cycle (every arm and steering goal refreshed plus the state sync read) takes under half the 10 ms period of bus time, `register_bytes` is little-endian like `write2ByteTxRx`, and an empty `sync_write_register` sends nothing; `read_register` returns None (a failed read the caller can retry) when st3215 raises IndexError on a truncated status packet, as happened while a servo re-read its EEPROM after `center`. `test_state_read.py` (`sync_read.py`): one cycle is one sync write plus one sync read (before: 6 goal writes + 1 sync read + 2 present_load reads), the read is 6 bytes only when effort joints are configured, the folded effort equals the single present_load read decode (inverted joints flipped), position and speed are unchanged by the longer read, a missing servo falls back to one 6-byte block read and is omitted, `read_servo_state` decodes position/speed/load, and no effort joints publish no effort; joints outside `publish_effort_joints` and an effort joint without a load reading report NaN (a measured 0 stays 0.0); a dead last servo (the sync transaction itself fails) still yields the other replies with one fallback read of the dead servo only (before: all six re-read); "one servo dead: others from the sync reply, cycle time bounded" simulates 10 s of follower cycles on the 14-servo client bus with servo 6 dead and the emulated packet timeout (`PACKET_TIMEOUT_S` = 50 ms per incomplete reply): the first `read_failure_limit` cycles cost at most a sync and one fallback timeout, after that only the 5 s probes are slow and the loop rate stays at or above 95% of the measured 95.8 Hz; a backed-off servo is left out of the sync read (one transaction, no timeout), probed after `read_retry_interval_s`, and rejoins the sync read once it answers; `ReadBackoff` needs consecutive failures; readings carry their request time (`read_at_ns`, the sync read's send time or the later fallback read's) and `group_stamp_ns` is the oldest of a group's readings. `test_wheel_mode.py` also checks that a sync read whose transaction failed keeps the replies that arrived and re-reads only the missing servo. `test_bridge_cycle.py` covers `split_stamp_ns` (header sec/nanosec). `test_config.py` covers `goal_refresh_interval_s` / `read_failure_limit` / `read_retry_interval_s` (defaults 1.0 / 3 / 5.0, overrides, negative refresh clamps to 0, invalid limit and retry fall back to defaults). `tests/sts_bus.py` counts timeouts (`FakeStsPort.timeouts`, each 50 ms in `wire_time_s`).

Bridge cycle (`test_bridge_cycle.py`, rclpy-free `bridge_cycle.py`): regression test that wheel `goal_speed` is never written to 0 while drive commands keep arriving faster than the loop (the 2026-10 stall: one ROS callback per loop iteration starved drive commands until the 0.3 s watchdog fired); watchdog stops wheels when drive commands cease while steering continues, works per wheel, stays fed by received commands even if a write fails, retries a failed stop; callback draining processes everything pending, is bounded and stops when nothing is ready; remaining-sleep computation; combined JointState with NaN placeholders drives steering and wheels; non-finite velocity/position entries ignored (and do not feed the watchdog); arm position-only messages and separate steer/drive messages still work. Direct command sources: web UI / autonomy gripper targets reach the servo as follower radians (0.0 rad -> step 2048, clamped to the command range), leader and untagged commands keep the exact `source_min/max_steps` mapping (step-for-step identical to a cycle without direct sources over a leader sweep), no direct sources = legacy mapping for every message, joints without a source range unaffected; `test_config.py` covers `direct_command_sources` parsing (default empty, blanks dropped, non-list rejected).

Servo torque and neutral (`test_torque_control.py`, `test_torque_services.py`, `test_torque_loop.py`, `test_neutral.py`, `test_set_register.py`; bus emulated by `sts_bus.py`):
- Torque release and enable (`test_torque_control.py`): release writes torque 0 even when the cache already holds 0 and verifies it, gates the group before any torque write, drops pending targets, names a joint whose torque-off is not verified, is repeatable on a released group, fails for an unknown group or with no bus; a released group gets no goal across cycles even when a refresh is due; enable writes goal = present before torque 1, retries a `None` present read, leaves a joint released (and names it) when its present read keeps failing or torque-on is not verified, and lifts the gate; the base release stops the wheels, keeps their torque and drops their commands, and enable accepts drive commands again; other groups on the same bus keep receiving commands; a register dump updates joint torque.
- Torque services and state topic (`test_torque_services.py`): `release_torque` / `enable_torque` services exist per namespace (default and extra groups); the torque state topic is reliable, transient-local, depth 1; release then enable move the published state released -> held; velocity joints are never listed or released; no bus means refusal and nothing published; a joint without a reading is omitted from the payload, any gated position joint makes it released, no reading means nothing published; publishes on change and every interval (interval 0 = on change only); a register dump showing torque 0 changes the state.
- Torque in the bridge loop (`test_torque_loop.py`): startup torque seeds the first published state, nothing is published before the startup torque sequence, release through the loop publishes released, a register dump shows a servo that dropped torque, and a release mid dump round is not overwritten by older dump readings.
- Neutral record and save (`test_neutral.py`): record is refused unless the group is released with every neutral joint verified at torque 0 and without `neutral` enabled, captures present positions without writing, retries a failed present read and fails when it never reads; save is refused without a capture, when not released, when a joint moved since record (nothing written) or when the wheel stop is not acknowledged; save unlocks, defines the middle and locks in order, enables torque holding the present position and clears the capture; excluded joints are untouched, wheels stop before the first EEPROM write, verify retries a `None` read and gives up after the center read attempts; limits are rewritten only when define-middle changed them (a transient `None` or mismatching limit read after the centre is settled and re-read first, the restore runs inside unlock/lock, a failed readback names the first limits read) and command limits refresh for EEPROM-limited joints only; the lock is written when define-middle fails or raises; a failed verify or limit restore aborts the rest and leaves torque off; a partial failure keeps the group released and names the joints; the calibration marker is written atomically (parents created, merged with an existing marker, no marker without `neutral_stale_marker`, a failure keeps torque enabled); status and payload carry the capture and last save; neutral services exist only for neutral-enabled groups; record and save publish torque state.
- `set_register` guard (`test_set_register.py`): `torque_enable` other than 0/1 and out-of-range values are rejected (range edges accepted); a released joint refuses motion and torque enable but accepts torque off and limit writes; an unreleased joint accepts a goal position.

### Per-node tests (uvc_camera)

The **uvc_camera** node has tests under `nodes/bridges/uvc_camera/tests/`. Run from `nodes/bridges/uvc_camera`: `uv run pytest tests/ -v` (or `uv run poe test`). `test_config.py` covers env-based config (`get_config`): defaults, env overrides, device as path or index, stripping whitespace and fallback for empty topic/frame_id; `UVC_ROTATE_DEG` and `UVC_MAX_FPS` (`get_max_fps`: unset is no cap, positive number, invalid values raise). `test_frame.py` covers `rotate_frame` and `frame_due` (publish-rate throttle: no cap, first frame, period). Config lives in `config.py` (no ROS/OpenCV deps) for testability. Metrics (`test_metrics.py`: `camera_frames_published_total`, `camera_frames_dropped_total{reason}` read_fail/fps_cap, `camera_encode_seconds` histogram buckets, `camera_reopens_total`); `reopen_due` threshold (`test_frame.py`); `METRICS_PORT` resolved through `resolve_metrics_port(None)` (`test_config.py`).

### Per-node tests (arducam_tof)

The **arducam_tof** node has tests under `nodes/bridges/arducam_tof/tests/`. Run from `nodes/bridges/arducam_tof`: `uv run pytest tests -q` (the `ArducamDepthCamera` SDK and rclpy are mocked, so they run on macOS). `test_config.py` covers `TofConfig` / `load_config`: an empty file gives the defaults, every key is read, unknown keys and out-of-range values are rejected, `frame_id` is stripped, a missing file, invalid YAML or a non-mapping is a config error, and the path comes from `ARDUCAM_TOF_CONFIG`. `test_bridge.py` covers the topic and encoding contract (`/tof/depth/image_raw` 16UC1, `/tof/amplitude/image_raw` mono16, `/tof/camera_info`); depth below `confidence_threshold`, non-finite or out of range is 0 and the rest rounded millimetres; amplitude rounded and clipped to mono16, falling back to confidence when the SDK has no amplitude; the publish timer runs at `publish_rate_hz`; a good frame publishes depth, amplitude and camera_info with one header, `frame_id` and near range from config, the range in metres retried when the SDK rejects millimetres; a missing frame, a frame request error, a frame of another type or a malformed frame publishes nothing, is logged and counted (`tof_frames_dropped_total{reason}`) and the frame is released; a camera that fails to open, start or set up after start is stopped and closed as needed, logged and retried without publishing; a constructor error is an open failure; camera_info is skipped without intrinsics and the principal point defaults to the image centre; a camera without frames for the reopen delay is closed and reopened; `close` stops and closes an open camera; `run` shuts down cleanly; `main` returns 1 on a config error and runs the bridge with the loaded config.

### Per-node tests (lerobot_teleop)

The **lerobot_teleop** node has tests under `nodes/lerobot_teleop/tests/`. Run from `nodes/lerobot_teleop`: `uv run pytest tests/ -v` (or `uv run poe test`). `test_config.py` covers env-based config (`get_config`): defaults, env overrides, empty env fallback. Config lives in `config.py` (no ROS deps) for testability.

### Per-node tests (swerve_drive_controller)

The **swerve_drive_controller** node has tests under `nodes/swerve_drive_controller/tests/`. Run from `nodes/swerve_drive_controller`: `uv run pytest tests/ -v`. Covers: kinematics (`test_kinematics.py`: wheel_positions, inverse_kinematics straight/zero/sideways, forward_kinematics roundtrip, `forward_kinematics_with_residual` (residual ~0 for consistent wheels, grows with a slipping wheel), `robust_forward_kinematics` (keeps the full solution below the threshold, drops one slipping wheel and matches the other three for each of the four wheels, keeps the full solution when two wheels slip), `odometry_twist_variances` (0.01 + r^2 and 0.01 + (r / hypot(lx, ly))^2; parked yaw variance 1e-6 and the EKF weight math: parked yaw rate < 1% of the IMU gyro bias), steer_angle_difference, should_zero_drive, normalize_angle; with the real platform geometry: `fold_to_steer_range` (inside limit, backward flip, +-90 deg boundary picks side closer to current), `desaturate_wheel_speeds`, `wheel_speed_scale` (min(1, max/peak), 1.0 for zeros), the full MPPI envelope grid (vx, vy in +-0.25 step 0.05, wz in +-0.5 step 0.25, limit 4.71 rad/s: no wheel above the limit, FK equals k * command with one k in (0, 1], except where an IK heading sits in the steering hysteresis band; the corner (0.25, 0.25, 0) gives k = 0.2826 / 0.3536), `compute_wheel_commands` forward/backward/strafe/rotate-in-place/stopped-holds-steer/desaturation/IK-FK roundtrip, `wheel_states` requires all joints, `integrate_odometry` straight/rotated/arc, steering-limit hysteresis keeps the current side just past +-90 deg and switches side when far past); config (`test_config.py`: load_config missing/minimal/defaults, defaults match Platform dimensions, motion limits and timeouts parsed, `slip_residual_threshold_mps` default 0.05 and override); control step (`test_control_step.py`, including the `swerve_cmd_speed_scale` gauge (about 0.799 for the (0.25, 0.25, 0) command, 1.0 for (0.1, 0, 0), 1.0 when stopped or inhibited) and the odometry residual: a slipping wheel is dropped and the residual is ~0 for consistent wheels, and `parked` only when measured wheels are still and no command is active: an active command is not parked, an expired or deadband command is). Metrics (`test_metrics.py`): `metrics_port` config field, and `run_cycle` updating `swerve_cmd_vel_age_seconds` (unset before the first cmd_vel), `swerve_slip_residual`, `swerve_loop_duration_seconds` (every cycle), `swerve_odom_published_total` (only when published) and `swerve_joint_states_stale_total`. Tilt guard (`test_tilt_guard.py`): quaternion -> roll/pitch (identity, roundtrip independent of yaw, unnormalised input, NaN/inf/zero quaternion rejected and not counted as a sample), level reference subtracted; rise only after the 0.5 s persistence (a 0.2 s spike and repeated short spikes do not trip it), straight ok -> error with the causing axis; asymmetric limits (roll 13 warning, pitch 13 and 18 warning, roll 18 error, negative pitch by magnitude, roll 7 / pitch 11 ok) and pitch defaults larger than roll; recovery only below limit - 2 deg hysteresis after the 2 s hold (error -> warning -> ok, error straight to ok, hold restarts when the tilt returns, time held below the error recovery threshold does not count as the hold below the warning one: 18 -> 12 deg for 1.9 s -> 5 deg goes to warning, still inhibited, and to ok only after 2 s at 5 deg); base drive inhibition latched from error until back at ok (still inhibited at 12 deg roll in warning, only below 8 - 2 = 6 deg after the hold), a warning from ok never inhibits, error again from warning re-latches; default level reference roll 0.7 / pitch 1.3 deg and 1 Hz heartbeat; stale IMU (error and inhibition held, a data gap restarts persistence and the recovery hold, no sample = stale and no state); `enabled: false` never inhibits; pydantic validation (warning above error, hysteresis wider than a warning limit, unknown error_action or key rejected); `/tilt_guard/state` JSON keys/order and validation against a mirror of the shared contract model (`roll_warn_deg`/`pitch_warn_deg` limits, `since` epoch s of the level start or of the first sample, `imu_ok`), values, messages and `imu_ok: false` when stale; `tilt_guard_level`/`tilt_roll_deg`/`tilt_pitch_deg` gauges; set_register torque payloads. Control step (`test_control_step.py`): while inhibited a fresh command is ignored and every drive is 0 rad/s, steering is held at its targets also past `idle_recenter_s`, not inhibited passes the command through, and `gate_cmd_vel` drops and counts commands in `tilt_guard_inhibited_commands_total`. Config (`test_config.py`): `tilt_guard` defaults (incl. level reference 0.7 / 1.3 deg and 1 Hz heartbeat) and block parsing, an invalid block rejects the whole config, and the shipped `ansible/group_vars/client.yml` swerve_controller block validates. Node wiring (`test_node_tilt_guard.py`, rclpy stubbed): `/imu/data` subscription, no state before the first IMU sample, drive passes when level and is zeroed (and counted) after a roll error with the error logged and the state published at once, motion again after recovery, drive still zeroed in a warning that follows an error, brake writes no torque registers while `torque_off` writes `torque_enable` 0 for the four drive joints, keeps torque off in the warning after the error and writes 1 only back at ok, heartbeat at 1 Hz, heartbeat shows `imu_ok: false` and logs the stale IMU; restart behaviour: no motion until the first IMU sample, a first sample beyond the error limit latches error at once (`test_tilt_guard.py` too: fresh guard inhibited, first OK sample releases, first warning does not inhibit, disabled guard never inhibited), missing IMU logged, disabled guard does not wait; torque: `torque_off` writes repeated each heartbeat while inhibited, the restore repeated twice then quiet, torque-on written once at startup (never in brake mode).

Control step (`test_control_step.py`, rclpy-free `control.py`): combined command layout (8 joints, steer positions + NaN, drive velocities + NaN); no command for stale, missing or incomplete joint states; forward command; cmd_vel timeout gives zero twist; deadband; steer targets held when stopped; no-propulsion safeguard; odometry integration and reported twist. Coordinated steering side (`test_kinematics.py`, `test_control_step.py`): near +-90 deg all wheels pick the same side (no single-wheel 180 deg swings in a sweep), group-level hysteresis prevents chattering, pure rotation falls back to per-wheel fold within limits, FK roundtrip holds for every output, side choice carried in the controller state. Idle recentering: steering held for under `idle_recenter_s` (default 3 s) after stopping, then targets return to 0 rad; motion resets the timer; 0 disables it (`test_config.py` covers parsing and clamping). `test_config.py` also covers `publish_tf` (default true, explicit false).

Torque state (`test_torque_state.py`): `parse_released`; the node subscribes with latched QoS, an empty topic disables the subscription, an unparsable payload is logged and ignored, and a release clears the latest command and ignores new ones.

### Per-node tests (rf2o_odom_relay)

The **rf2o_odom_relay** node has tests under `nodes/rf2o_odom_relay/tests/`. Run from `nodes/rf2o_odom_relay`: `uv run pytest tests/ -v`. Covers: pose-difference twist (`test_twist.py`: straight ahead, world motion rotated into the body frame, sideways motion reported as `vy`, yaw-rate wrap across pi, mid-heading rotation during a turn, no twist for zero/negative/too large time steps, diagonal-only covariance); config loading (`test_config.py`: missing file, defaults, overrides). Metrics (`test_metrics.py`): `metrics_port` config field, `relay_messages_total` per published twist and `relay_dt_rejected_total` per rejected time step (`relay_twist`).

### Per-node tests (map_guard)

The **map_guard** node has tests under `nodes/map_guard/tests/`. Run from `nodes/map_guard`: `uv run pytest tests/ -v`.

| File | Covers |
|---|---|
| `test_agreement.py` | Scan endpoint filtering (inf/nan, range_min, min(range_max, max_range_m)); TF projection (yaw, upside-down mount); square dilation; agreement on a synthetic square room ray-cast from a known pose: perfect transform (> 0.95), shifted 0.4 m and rotated 0.35 rad (< 0.3), one cell off accepted / three cells off rejected, empty map, missing map, fewer than min_points, points outside the map or in unknown space not evaluated, rotated map origin, known cell count. MismatchMonitor: fresh map never triggers, small map never triggers, mismatch after a healthy period fires only after mismatch_s, isolated low samples never fire, reset disarms, rolling median. |
| `test_detectors.py` | roll/pitch, wrapped angle difference, world-frame rotation, odometry speed (larger of twist and pose difference). LIFTED: sustained 20 deg tilt fires after lift_s, a 0.2 s spike never fires, no baseline no trigger, tilt below threshold, baseline follows an inclined floor; vertical acceleration then tilt or lidar motion fires, acceleration alone, a 0.2 s acceleration spike, gravity-included fallback samples ignored. CARRIED: sustained fires after carried_s (unexplained speed reported), 0.2 s spike never, commanded motion suppresses, command older than cmd_timeout_s does not, zero command does not, moving wheels suppress, stale lidar odometry never fires. ROTATED: sustained fires after rotate_s, 0.2 s spike never, commanded rotation and wheel yaw rate suppress. Quiet robot reports nothing. |
| `test_guard.py` | State machine with a fake slam_toolbox (toggle + parameter): lifted -> disturbed (pause, trigger counter, event) -> settling -> evaluating -> resumed -> ok; a new trigger while settling or evaluating returns to disturbed without a second toggle; low agreement backs up (timestamped name in backup_dir) then resets, unpauses, publishes the reset topic, counts `resets_total{dry_run="false"}`, event with the backup path; pruning keeps the newest keep_backups; failed backup skips the reset; failed reset publishes nothing; no samples resumes; mismatch alone pauses and evaluates without settling; cooldown blocks a mismatch retrigger until it ends but not physical triggers; disabled reasons ignored. Toggle tracking: already paused not toggled, failed toggle keeps the state, a toggle that did not take effect is detected, unreadable parameter falls back to tracking. Dry run never calls a service, never publishes or writes a backup, still counts and logs; dry_run, tilt and unexplained speed gauges. |
| `test_agent.py` | Busy fetch (true/false, malformed, unreachable, timeout = not busy), poll at most every agent_poll_s; events published only when busy (idle and unreachable get none); severity per type and reason (critical for reset and lifted/carried); dry run: `[dry run] ` prefix, lifted/carried disturbed stay critical, every other event (rotated, mismatch, resumed, reset) warning; event fields match the `/robot_events` contract of claude_agent's `parse_robot_event` (keys, types, source, message length, data size); increasing seq. |
| `test_backup.py` | Backup stem `map_guard_backup_<YYYYMMDDTHHMMSSZ>` in UTC; pruning keeps the newest N stems (both files), never touches `slam_map.*` or non-timestamped files, missing directory. |
| `test_config.py` | Defaults; rejected values (non-positive durations and rates, ratios outside 0..1, healthy_ratio <= mismatch_ratio, unknown reason, keep_backups 0, negative radius, threshold > 100, unknown key); YAML loading, missing/non-mapping file, env path. |
| `test_timing.py` | HoldTimer (full duration, reset on a false sample, short spike, reset) and RateLimiter. |
| `test_node.py` | ROS wiring with stubbed ROS packages: OccupancyGrid conversion, slam_toolbox clients (parameter read, unknown parameter type, unavailable service, timeout, Pause status, SerializePoseGraph / Reset result codes and request fields). |

### Per-node tests (static_tf_publisher)

The **static_tf_publisher** node has tests under `nodes/static_tf_publisher/tests/`. Run from `nodes/static_tf_publisher`: `uv run pytest tests/ -v`. Covers: config loading (`test_config.py`: missing file, frames list with parent/child and offsets).

### Per-node tests (filter_node)

The **filter_node** node has tests under `nodes/filter_node/tests/`. Run from `nodes/filter_node`: `uv run pytest tests/ -v` (or `uv run poe test`). Covers: config loading (`test_config.py`: input/output topic, algorithm, params, joint_names); algorithm registry and Kalman (`test_algorithms.py`: get_algorithm, Kalman create_state/update/predict); output command filling (`test_command.py`: `header.frame_id` carries the command source leader / web_ui / autonomy, positions copied, velocity/effort cleared). Metrics (`test_metrics.py`): `metrics_port` config field, `filter_input_age_seconds` per source (absent until first message), one-hot `filter_active_source`, `filter_source_switches_total` only on change, `filter_loop_overruns_total` for late loop iterations (`FilterMetrics`).

Released state (`test_arbitration.py`): `release` is ignored when false and a no-op without a lease; a release ends the lease so nobody publishes filtered, web_ui works immediately afterwards and the leader only after proximity (also after a web_ui timeout and without follower feedback); autonomy can retake the lease after a release; released refuses every source (leader, web_ui, autonomy), ends an autonomy lease, works from web_ui and leader states, re-enable requires proximity for the leader but not for web_ui, transitions are reported once, `parse_released` reads the JSON flag and rejects unparsable payloads, and an autonomy command refused while released does not take the lease.

### Per-node tests (test_joint_api)

The **test_joint_api** node has tests under `nodes/test_joint_api/tests/`. Run from `nodes/test_joint_api`: `uv run pytest tests/ -v` (or `uv run poe test`). Covers: config (`test_config.py`); GET/POST `/joint-updates` (`test_app.py`: empty GET, single/multiple POST, validation errors, gripper-only POST). Async tests require **pytest-asyncio** (included in the node's uv dev dependency group; if running with system pytest, install it: `pip install pytest-asyncio`). Endpoint use in tests is limited to gripper joints (joint_5, joint_6) for safety. The utility script `scripts/joint_api_client.py` can GET or POST joint updates (see script docstring for examples).

- Metrics (`test_metrics.py`): `GET /metrics` serves the `ros2-metrics` registry with `robot_node_info{node="test_joint_api"}`; the middleware counts `jointapi_requests_total{status}` for 200, 400 and 404 responses.

### Per-node tests (topic_scraper_api)

The **topic_scraper_api** node has tests under `nodes/topic_scraper_api/tests/`. Run from `nodes/topic_scraper_api`: `uv run pytest tests/ -v` (or `uv run poe test`). Covers:

- config parsing (`test_config.py`: defaults, overrides, type allow-list, observation_rules parsing)
- endpoint mapping (`test_paths.py`: normalize topic, topic->endpoint, endpoint->topic; stream/preview path helpers)
- message serialization (`test_serializer.py`: recursive conversion for ROS-like message objects, time conversion; `message_to_payload` omits `data` for Image/CompressedImage/PointCloud2 with a byte length and keeps other fields, 640x480 bgr8 yields no list, tracemalloc peak stays below a quarter frame)
- image encoding (`test_image_encoding.py`: is_image_type, CompressedImage jpeg passthrough, raw Image bgr8/mono8 to JPEG, unsupported encoding returns None)
- dynamic subscription bookkeeping (`test_scraper.py`: allow-list handling, add/remove topic subscriptions; image topic metadata and lazy JPEG (no encode without a request, one encode per frame cached across requests, a new frame is encoded on the next request, `has_sample` without encoding); sensor QoS selection by type; image callback payload has no data list and keeps the JPEG)
- HTTP API behavior (`test_app.py`: `/topics`, `/topics/<topic-path>`, `/rules`, `/rules/<name>`; `/streams`, `/previews` lists; stream/preview HTML pages; `/previews/<topic>/image.jpg` JPEG snapshot; 404 when no image sample for stream/preview image)
- observation rules (`test_observer.py`: RulesObserver empty rules, compare rule produces position delta, missing payload yields None comparison, rules summary; non-finite, None (serialized NaN) and non-numeric positions are skipped, never crash the rules loop)

- Metrics (`test_metrics.py`): `scraper_subscriptions` follows `sync_topics` (add and remove), each callback bumps `scraper_messages_total` and `scraper_callback_seconds`, no `scraper_*` sample carries a topic label, `GET /metrics` returns the registry with `robot_node_info{node="topic_scraper_api"}`.

### Per-node tests (bno055_imu)

The **bno055_imu** node has tests under `nodes/bridges/bno055_imu/tests/`. Run from `nodes/bridges/bno055_imu`: `uv run pytest tests/ -v` (or `uv run poe test`). Covers:

- metrics (`test_metrics.py`: `imu_soft_restores_total`, `imu_reinits_total{reason}` by recovery code, calibration levels per subsystem and the incomplete-status case, published counter and `imu_seconds_since_publish`, read errors, init attempts, mode); `Decision.code` stable escalation codes (`test_recovery.py`); `metrics_port` config field (`test_config.py`). GPS status (`test_nmea_parser.py`: `parse_gga_status` reads quality, satellites and HDOP from no-fix GGA with empty lat/lon, empty fields are `None`, no position keys, same fields as `parse_gga`, `parse_gga` still rejects no-fix GGA; `test_status.py`: rover status from no-fix GGA, receiver-silent status, `build_rover_status` timing (nothing before the timeout, silent since start, silent after NMEA stops, fresh no-fix GGA, NMEA without fresh GGA skipped), base status without position, survey state, `ref_station_position`, `gga_seq`; `test_config.py`: `nmea_timeout_s` default 3.0 and > 0; `test_rover_node.py`: no-fix GGA publishes status and real satellite metrics but no NavSatFix, fix GGA publishes both, silent statuses, non-GGA NMEA keeps the receiver not silent; `test_base_node.py`: one status object for the topic and `GET /status`, from every GGA, survey state from the serial handler, caster wired with `status_provider`; `test_ntrip_caster.py`: `GET /status` 200 JSON with Content-Length and close, 503 `no status yet`, no NTRIP auth needed, 404 without a provider, `GET /` source-table request and `/rtk` mountpoint (incl. 401) unchanged; `test_serial_handler.py`: `survey_state` for stored ECEF, no response, zero ECEF, unparseable answer and failed configure).
- Tile API key (`test_tile_api_key.py`): `tile_api_key_env` default, `{api_key}` substitution in `build_tile_url`, `tile_cache_fingerprint` (8 hex, hides the key, differs per key/template/no key), key read from the env var and cached under the fingerprint directory, a changed key refetches, missing or empty key (or no env name) gives 503 with no fetch and one `tile_api_key_missing` warning, keyless templates unchanged, the key absent from `/api/config` and from logs, `tile_max_zoom` (default 18, range 1..22; tiles above it answer 404 without contacting upstream).
- Tile sources (`test_tile_sources.py`): `tile_sources` resolution (legacy `tile_url` fields become the `default` source, unknown default / duplicate ids rejected), per-source proxy route with explicit template order (Esri `{z}/{y}/{x}`), separate cache subdirectories per fingerprint, zoom above a source's `max_zoom` is 404 without upstream, unknown source 404, old route serves the default source, a missing key is 503 for that source only, `/api/config` exposes the public source list (no URL, key or env name) and `tile_display_zoom`.
- config loading (`test_config.py`: missing/empty file, defaults, explicit topic/frame_id/publish_hz/i2c_bus/i2c_address/covariances, publish_hz clamping, load_config_from_env)
- burst read and decode (`test_burst.py`: 26-byte block from 0x14, decoded values identical to the `adafruit_bno055` properties for recorded and random raw bytes, all-zero or short block is invalid, exactly one I2C transaction per read, bus errors propagate)
- fixed-rate pacing (`test_pacing.py`: `next_tick` period without drift, overrun skips missed slots, next slot always in the future; `spin_until` spins for the remaining time and still serves callbacks after an overrun)
- metrics `imu_cycle_seconds` buckets and `imu_cycle_overruns_total` (`test_metrics.py`)
- quaternion and IMU message mapping (`test_imu_msg.py`: `quaternion_wxyz_to_xyzw`, `build_imu_message` units and covariance arrays; build_imu_message tests are skipped when sensor_msgs is not available, e.g. without a ROS environment)
- rolling-window covariance estimator (`test_covariance.py`: `CovarianceEstimator` — returns None below min_samples, correct at min_samples, zero covariance for constant signal, uncorrelated axes produce diagonal matrix, diagonal variance matches known input, rolling window evicts old samples, min_samples < 2 clamped to 2, symmetry of returned 3×3 matrix)
- node validation helpers and startup logic (`test_node.py`: `coerce` — None/float/int/negative; `all_zero` — zeros/non-zero/empty/tolerance; `has_valid_tuple` — None/too-short/None-elements/valid/allow_zeros; `valid_quat` — None/all-zeros/identity/unit-norm/too-large/too-small; `warmup_check` — OSError/None-values/valid-gyro/fallback-acceleration; `_spin_once_safe` — exception swallowing/timeout; `_warmup` — ready-immediately/timeout/rclpy-not-ok/delayed-valid-data; `_create_bno055` — mode correct first verify/retries/stuck/all-addresses-fail)
- gyro bias estimation (`test_gyro_bias.py`: converges to a constant bias when stationary; no learning before the stillness window, while moving, without or with stale odometry, above the gyro sanity limit; clamp; time constant; `test_gyro_bias_node.py`: the node publishes raw before learning and raw - bias after, raw without odometry, subscribes to `odom_topic`; running mean for the first `mean_s` (exact mean of noisy samples, EMA after, loaded bias is a one-sample prior, `settled`), `set_bias` clamps, rejected-sample counter only while stationary, bias file roundtrip/atomic/missing/corrupt/non-finite; node: loaded bias applied from the first sample, corrupt file ignored with a warning, settled bias saved, not saved before the mean window, rejected samples counted in `imu_gyro_bias_rejected_samples_total` and logged throttled, the `/imu/gyro_bias` diagnostic carries the real raw gyro; plus config defaults/disable in `test_config.py` and the `imu_gyro_bias_*` gauges and rejected counter in `test_metrics.py`)

### Per-node tests (haptic_controller)

The **haptic_controller** node has tests under `nodes/haptic_controller/tests/`. Run from `nodes/haptic_controller`: `uv run pytest tests/ -v` (or `uv run poe test`). Covers: config loading (`test_config.py`: missing/empty file, defaults, mode off/resistance/zero_g, invalid mode fallback, resistance_gains including load_release_ratio and activation_debounce_cycles, delay_safety_max_skew_s, load_config_from_env); resistance control law (`test_node.py`: `compute_resistance_target` — no load/zero velocity returns leader_pos, opposes closing when load above deadband, respects max_step_per_cycle; `should_apply_resistance`; `should_apply_resistance_hysteresis` — activation above deadband, stay-active until release threshold). No ROS2/rclpy dependency in tests.

**Gripper-only validation (manual):** After deploying with haptic controller in resistance mode, bench-check: (1) free motion of leader gripper remains easy (no lock); (2) resistance appears only on contact (follower load above deadband, leader closing); (3) no periodic ~1 s pulsing; (4) no oscillation growth when moving slowly. Use `ros2 topic hz` on key topics and verify service health for leader/follower/haptic. Leader gripper tuning: apply `nodes/bridges/feetech_servos/leader_gripper_haptic_profile.json` via `calibrate_servos.py load-config` on the server (see feetech_servos README).

### Per-node tests (master2master)

The **master2master** node has its own test suite under `nodes/master2master/tests/`. Run from `nodes/master2master`: `uv run pytest tests/ -v` (or `uv run poe test`). A `conftest.py` provides path setup. Covers:

- config loading and validation (`test_config.py`: `normalize_topic` — adds leading slash, strips trailing slash, idempotent, empty string, root slash; `TopicRule` construction — defaults, topic normalisation, direction in/out, msg_type jointstate and all new types (imu, navsatfix, laserscan, occupancygrid, odometry, posestamped, image, compressedimage, twist), case-insensitive direction/msg_type, invalid direction/msg_type raises `ValidationError`, non-string source raises; `parse_rule_entry` — string entry, full dict, `from`/`to` aliases, dest defaults to source, missing source returns None, empty string returns None, invalid type raises `ConfigError`, invalid direction raises; `load_config_from_dict` — empty dict, valid topics list, `topic_proxy` key alias, empty topics list, skips None entries, non-list topics raises `ConfigError`, realistic multi-rule config, duplicate topic names allowed; `load_config` — missing file returns empty, empty file returns empty, valid file; `validate_relay_rules` — no loops passes, detects dest→source loop, empty list passes)
- relay proxy (`test_proxy.py`: `get_message_class` — string, jointstate, case-insensitive, unknown raises `KeyError`; `get_supported_message_types` — contains all expected types; parametrized new msg_type→mock mapping; `run_all_relays` — creates one pub/sub per rule, multiple rules get separate pubs/subs, relay callback publishes to correct publisher, calls `rclpy.init`/`shutdown`, shutdown called even on exception, rejects relay loop with `ValueError`, rejects unknown msg_type, `shutdown_callback` stops spin loop, node destroyed after run, two-rule callbacks publish to their own publishers)

Mocks rclpy, sensor_msgs, std_msgs, nav_msgs, and geometry_msgs at module level so proxy tests run without a ROS2 environment.

### Per-node tests (steamdeck_ui — frontend)

The **steamdeck_ui** Electron frontend has TypeScript tests under `nodes/steamdeck_ui/tests/`. Run from `nodes/steamdeck_ui` with the project's JS test runner (e.g. `npx jest` or `npm test`). Covers:

- config loading (`test_config.test.ts`: `loadConfig` — loads valid config with bridge host/port and camera tab, throws on nonexistent file, throws when `bridge.port` is missing, parses sensor_graph tab with topics/fields, missing `overlays` defaults to empty array)
- field extraction and formatting (`test_field_extract.test.ts`: `parsePath` — simple key, dot notation, array index, deep path, array-only index; `extractField` — simple nested field, array index, deep nested, missing field returns null, empty object returns null, empty path returns null, out-of-bounds returns null, non-numeric field returns null; `formatValue` — `.2f`/`.6f`/`.0f` fixed-point format, no format returns string, unit appended, unit without format)

### Per-node tests (steamdeck_ui — bridge)

The **steamdeck_ui** Python bridge has tests under `nodes/steamdeck_ui/bridge/tests/`. Run from `nodes/steamdeck_ui/bridge`: `uv run pytest tests/ -v` (or `uv run poe test`). Covers:

- config loading and validation (`test_config.py`: `BridgeConfig` defaults (host, port, ros_static_peers), custom port, `ros_domain_id` int coerced to str; `TabConfig` — valid camera tab, invalid type raises; `AppConfig` — empty config valid, `all_subscribed_topics` for camera and sensor_graph tabs, `publish_topics` includes nav goal_topic, no duplicate topics; `load_config` — valid file, missing file raises `FileNotFoundError`, empty YAML returns defaults)
- message serialization (`test_msg_serializer.py`: `_bytes_per_pixel` — rgb8/mono8/rgba8/mono16/rgb16; `_serialize_value` — `array.array` to list, bytes to list, primitives passthrough, nested list; `extractField_from_dict` — simple key, nested dot notation, array index, missing key returns None, non-numeric returns None)

### Per-node tests (web_ui)

The **web_ui** node has tests under `nodes/web_ui/tests/`. Run from `nodes/web_ui`: `uv run pytest tests/ -v` (or `uv run poe test`). A `conftest.py` installs ROS2 module stubs (rclpy, sensor_msgs, nav_msgs, geometry_msgs) and provides `config_yaml` and `urdf_dir` fixtures. Covers:

- config loading (`test_config.py`: minimal config defaults, http_port override, missing file raises `FileNotFoundError`, `all_subscribed_topics` for camera/nav/overlay tabs, `load_config` from env var `WEB_UI_CONFIG`, `publish_topics` includes goal_topic, `rgbd_camera` tab type valid with color/depth/camera_info topics, RGBD topics included in `all_subscribed_topics`, `arm` is a valid tab type with defaults (topics, mount, arm offset, `active_source_topic`, Trigger services), unset-field overrides, an empty topic rejected, other tab types without arm defaults, arm topics subscribed and arm services in `trigger_services()`, the shipped default config has the `arm` tab in the old Gripper Cam slot (no `camera` id), no `overview_camera` tab or `/overview_camera/*` subscription (`test_default_config_has_no_overview_tab`) and no RGBD tab)
- bridge dirty-flag store (`test_bridge.py`: `/head_camera/image_raw/compressed` resolves to a `CompressedImage` subscription and the retired front/back camera names to none (`test_head_camera_topic_is_compressed_image_subscription_and_pi_cameras_names_are_gone`); `flush_dirty` returns dirty topics, clears after flush, only returns dirty entries; `publish_dict` rejects non-allowlisted topics with warning)
- message serialization (`test_msg_serializer.py`: `msg_to_dict` — Imu message conversion; `extract_field_from_dict` — simple key, array index, missing returns None, out-of-bounds returns None; depth image serialization — 16UC1 returns expected keys, downscaled raw bytes length, depth values preserved, zero-depth preserved; `CameraInfo` returns fx/fy/cx/cy/width/height; no image topic gets a downscaled colour field; 16UC1 depth at 320x240 gives preview and 160x120 grid)
- non-finite floats (`test_msg_serializer_nonfinite.py`: NaN/inf in a JointState effort array serialize as JSON null, so the websocket frame stays valid JSON; feetech publishes NaN effort for unmeasured joints)
- HTTP server routes (`test_server.py`: `/api/config` returns config JSON, `/api/urdf/status` lists URDF files, `/api/urdf/<file>` serves URDF, path traversal blocked, security headers present, static fallback serves index.html)
- Battery (`test_battery.py`): `battery` config (defaults, absent = off, `resume_cell_v >= cutoff_cell_v`, `cells >= 1`, in `/api/config`, topic subscribed with role `battery`), `serialize_battery`, `BatteryGuard` (unknown without reading, cut-off below 8.4 V, hysteresis release above 8.7 V, stale reading not blocked, invalid voltage ignored, thread safety, rejection message), bridge battery callback (payload with guard state, NaN dropped), server rejection of WS `publish` (error frame) and of map save/reset and arm home/set home with 503, `nav/stop` still allowed, no blocking without battery config, with a good or unknown battery.
- Battery frontend (`frontend/src/battery/batteryStatus.test.ts`, vitest): chip level/colour/label (unknown, green, amber, red incl. backend flag), stale after `stale_s`, cut-off banner text, WebSocket frame parsing (envelope, error frame, malformed), 503 message surfaced by `parseActionResult`.
- No master2master relay topics (`test_map3d_backend.py`: `test_default_yaml_reads_no_master2master_relay_topics`, `test_topic_type_hints_have_no_master2master_relay_topics`): the shipped `config/default.yaml` and `TOPIC_TYPE_HINTS` use the robot's own topics (`/client/gps/fix`, `/odom`, `/swerve_drive/joint_states`, `/scan`, `/local_costmap/costmap`, `/camera_0/image_raw`, `/goal_pose`), never `/controller/*`, which only the master2master proxy to external ROS2 masters publishes.
- On-demand ROS subscriptions (`test_bridge_on_demand.py`, rclpy faked by a `FakeRos` node double; design in the web_ui README "On-demand ROS subscriptions"): zero clients means zero on-demand subscriptions; a client wanting a topic creates its subscription once and a second client adds no duplicate; the last client leaving destroys it only after the 5 s linger (`DEMAND_LINGER_S`), a topic wanted again within the linger keeps its subscription and an unwant after that restarts the linger, and a resubscribe after destroy creates a new subscription; `wanted=None` subscribes every configured topic; the always-on set (tilt guard state, battery when its guard is on, map_guard reset, POI/grasp results and the other topics REST routes or safety checks read) is never destroyed; a latched topic is resubscribed transient_local so it delivers its last value again; a topic with no subscription has no sample and an inactive on-demand topic is not reported stale, and a late callback after destroy does not resume health tracking; the robot-pose timer runs only while the pose is wanted; the compass IMU follows GPS anchor demand, TF stays on while an anchor fit needs it and otherwise follows demand; the broadcaster computes demand from connected clients (ignoring one still inside its subscribe grace) and updates it even with no client; REST grasp stop and POI routes resolve and tilt state and battery are fed with zero clients.
- URDF directory scanner (`test_urdf_scanner.py`: `scan_urdf_directory` finds robot.urdf, parses links/joints, detects missing mesh files, empty directory returns empty list)

- Map tab backend (`test_map_nav.py`): `map_nav` tab type, defaults (`/map`, `/plan`, `/optimal_trajectory`, `/goal_pose`, `map`, `base_link`, save path) and overrides, empty frame rejected, subscribed topics and publish allowlist, topic roles; OccupancyGrid -> PNG grey levels, thresholds, row orientation (y up), metadata, size mismatch; Path serialization and downsampling to 500 points keeping the ends, paths in another frame transformed to `map` (dropped without TF); goal PoseStamped serialization; quaternion/transform helpers; latched (reliable + transient_local) map subscription; robot pose from TF published when available and omitted otherwise; typed `_dict_to_ros_msg` (ints vs floats, errors raised); goal publishing fills stamp and frame; `POST /api/map/save` success, slam failure, service unavailable, timeout, unknown tab, no bridge; cached snapshot sent on WS connect; default config has the map tab. Reset/Stop: `map_reset_service` and `navigate_action` defaults; `POST /api/map/reset` (slam_toolbox `/slam_toolbox/reset`) success, non-success result, unavailable, timeout, unknown tab, no bridge, cached map cleared only on success (cleared-map event); `POST /api/nav/stop` cancels all `NavigateToPose` goals (zero goal id and stamp), unavailable, timeout, cached goal cleared on success (cleared-goal event). Robot footprint: `footprint_topic` default and `footprint` role, PolygonStamped serialized to points and re-expressed in the map frame.
- Map tab frontend (`frontend/src/map/mapMath.test.ts` and `mapActions.test.ts`, vitest, `npm test`): Reset/Stop action helpers, front edge of the footprint polygon (`frontEdgeIndex`) and world <-> screen (y up), map image placement with origin and yaw, pan, zoom about the cursor with clamping, pinch zoom/pan, fit and centre view, yaw/quaternion helpers and angle normalization, goal heading from drag or plain click (towards goal, robot heading fallback), scale-bar length choice.
- Head command backend (`nodes/web_ui/tests/test_head_control.py`): head tab defaults and validation (ranges ordered, home inside), map_nav `head_urdf` / `head_joint_states_topic`, the `/head/joint_states` JointState hint, publish allowlist and `web_ui` frame_id default, WS publish of in-range commands, refusal of out-of-range / non-finite / unknown-joint commands, battery cut-off refusing first, no lease check (ruling R1).
- Swerve base model (`nodes/web_ui/tests/test_robot_urdf.py`): `urdf/robot.urdf` against `ansible/group_vars/client.yml`: joint names equal the `swerve_controller` `joint_names`; steering joints revolute about z at (+-half_length, +-half_width) with the steering limit; axle height equals the wheel radius (wheels on the floor); drive joints continuous about +y under the steer link; wheel links draw a cylinder of the wheel radius turned onto the axle and no meshes are needed; the body is only an outline: four thin bars on the floor whose outer edges span the mcp_server `footprint` (no solid placeholder box).
- Status bar robot icon and overlay fields (vitest): `components/robotIconGeometry.test.ts` (footprint to scale, rollers at the real module positions, size, straight/inactive before data, rotation by steering angle matched by joint name, missing or NaN joints), `components/RobotIcon.test.tsx` (SVG with four rollers, rotation applied), `utils/fieldExtract.test.ts` (dot paths, indices, `position[name=...]` selection and its null cases). Backend: `/odom` resolves to an Odometry subscription (`test_bridge.py`); `test_server.py` checks `index.html` and the SPA fallback are `no-cache`, `/assets/*` are not, URDF responses are `no-cache`; `/api/*` traversal never serves files.
- Base joint helper (`nodes/web_ui/frontend/src/map3d/baseJoints.test.ts`, vitest): steering angles passed through, drive angle integrated from `velocity` over dt (positive = forward, encoder position ignored), angle wrap, dt cap and negative dt, NaN/null velocities and steering skipped, drive angles carried across messages, empty/malformed messages.
- GPS tiles frontend (`frontend/src/map3d/tileSources.test.ts`, `tileStatus.test.ts` and the `tilesAround`/`tileUrl` cases of `geo.test.ts`, vitest): overzoom parent tile and sub-rectangle (z19 from z18 is 2x2 quadrants, z20 from z18 is 4x4), crop to plane UVs, source choice (stored, default, first) and its storage round trip incl. throwing storage, failure caption (HTTP status, unavailable, partial counts, ignores other URLs), per-source tile URLs and overzoomed placement.
- GPS status chips (`test_gps_status.py`, `test_config.py`; vitest `frontend/src/gps/gpsStatus.test.ts`, `basePlacement.test.ts`, `frontend/src/components/GpsChip.test.tsx`): base poll of `GET /status` (200 status with `has_status` and `sample_seq` from `gga_seq`, 503 `no status yet`, other HTTP codes and bad bodies as reachable without status, timeout and connection error as unreachable, stale when `gga_seq` stalls), `default.yaml` `gps_status` points at `http://server.ros2.lan:5016/status` with the model defaults; chips keep `no data`, `receiver silent` and `No fix - N sats` distinct, `Base: no status yet`, survey-in state in the tooltip, icons per state, no base marker without a status.
- Camera lookups backend (`nodes/web_ui/tests/test_lookups.py`): `lookups_dir` default, `GET /api/lookups` newest first with metadata and ISO time, `limit` bounds (422 outside 1-20), the filter matrix (`since`, `until`, `poi_id`, `text`, `camera`, `mcp_session`, `near_*`, combined), `near` needing all three parts, corrupt/blank index lines skipped, missing store lists nothing, `GET /api/lookups/{id}.jpg` serves `image/jpeg` read-only, unknown id and missing file are 404, path traversal (by URL id and by an indexed `../` file name) is rejected.
- Camera lookups and phase attempt/iteration frontend (`nodes/web_ui/frontend/src/lookups/api.test.ts`, `LookupList.test.tsx`, `src/agent/PhaseProgress.test.tsx`, `agentModel.test.ts`): lookups URL with the POI filter, fetch parsing and never-throw, the list renders thumbnails/camera/time/note and the selected-POI title and empty text; `attempt n/m`, `iteration k` and `p` chips from the new phase fields, `tree` ignored, old plans without the fields render with no chips. Time budget (`agentModel.test.ts` `timeBudget`, `src/agent/TimeBudgetChip.test.tsx`): null before a plan or without a declared budget, the time left counted down from the plan event deadline, a raise carried by a later state event, the state snapshot, 0 (never negative) once expired, the `budget N min, m:ss left` label; the chip renders nothing without a budget, ticks every second while the job runs and stops when it does not.
- POI backend (`test_poi.py`; object POIs with their sighting fields pass through `serialize_poi_list` unchanged): `poi_list_topic` / `poi_command_topic` / `poi_result_topic` defaults and roles (`poi_list`, `poi_result`), `/poi/list` subscribed latched (reliable + transient_local) and parsed by `serialize_poi_list` (malformed payloads rejected), `BridgeNode.poi_request_async` publishes the command with a fresh `request_id` and resolves on the matching `/poi/result` (None when no poi_store subscribes, cancelled requests forgotten), `POST /api/poi` (200 with the store result, 400 on store rejection, 503 without poi_store or bridge, 504 on timeout, 422 for bad bodies, 404 unknown tab) and that it is NOT blocked by the battery cut-off.
- Grasp backend (`nodes/web_ui/tests/test_grasp.py`): `grasp_command_topic` / `grasp_result_topic` / `grasp_timeout_s` / `grasp_accept_wait_s` defaults and validation, the `grasp_result` role (command topic not subscribed), `BridgeNode.grasp_request_async` publishes with a fresh `request_id` (a client id is replaced) and resolves on the matching `/grasp/result` (None without mcp_server, cancelled requests forgotten), execute/release publish `running` then stream the answer on `/web_ui/grasp_result` (plan and stop answers are not streamed and never overwrite the running state), malformed answers ignored; `POST /api/grasp` plan result / mcp error (400) / timeout (504), `surfaces` (call and object level) forwarded unchanged, execute and release return 202 accepted without cancelling the pending request or the immediate rejection, execute/release refused in battery cut-off (503, nothing published) while plan and `POST /api/grasp/stop` are not, 503 without mcp_server or bridge, 422 bad bodies, 404 unknown tab; `grip_profile` (preset name or inline object) is passed through unchanged with execute, a malformed one (number, list, empty string, bool) is 422 with nothing published, the 422 message lists every `GRIP_PRESETS` entry (gentle, soft, normal, firm) and `soft` passes through.
- Grasp frontend grip strength (`frontend/src/grasp/grasp.test.ts`, `objectSettings.test.ts`, `GraspPanel.test.tsx`, vitest): Gentle / Soft / Normal / Firm options (Soft hint: plush, soft toys; `soft` sent as `grip_profile`) with Normal as the default, `grip_profile` sent with execute only (not plan or release) and left out of the plan key so changing it keeps a feasible plan executable, the `unverified` outcome parsed and described as a warning (jaw closed almost fully, check the gripper camera), the grip report of a result parsed (profile name, torque limit, holding load, slipping, crush risk; junk ignored) and described ("Grip gentle, holding load 131" plus slipping / crush hints), the choice remembered in `localStorage` (`grasp.gripProfile`; unknown values and throwing storage fall back to Normal), the panel's grip strength selector marks the current profile and the result view shows the profile and holding load.
- POI frontend (`frontend/src/poi/*.test.ts`, vitest): geometry and hit testing (`geometry.test.ts`: area, centroid, point-in-polygon, hit priority, objects hit only within the pointer tolerance), editor state (`editor.test.ts`: click detection, area drafting, new-POI fields, drag previews and update commands incl. objects moving like points, optimistic override), styling and list helpers (`style.test.ts`: colour by status, creator marker, `/poi/list` validation, sort, relative time, object POIs: own colour, parsing with defaulted sighting fields, tooltip details), and the `POST /api/poi` client (`api.test.ts`). `map3d/layers.test.ts` covers the `pois` layer toggle.

- Tilt guard (`nodes/web_ui/tests/test_tilt_guard.py`): `/tilt_guard/state` topic name; `parse_tilt_state` accepts valid JSON and rejects malformed, non-object, unknown-level and non-numeric-angle payloads; `nav_rejection_message` is None for none/ok and the explicit warning (roll/pitch, automatic navigation disabled, drive back manually) and error (base drive disabled until level) texts; the bridge stores and broadcasts the parsed state, drops invalid JSON, subscribes reliable + transient_local and registers the topic for stale checks; a WS publish to the goal topic is refused with a `tilt_guard` error frame at warning and error, allowed at ok or without state, and non-goal publishes stay allowed during error; `nav_rejection_message` at warning with `base_drive_inhibited` (still-inhibited text), with `imu_ok: false` and with a stale state; the bridge marks the state stale (and broadcasts it once) after `tilt_guard_stale_s` and a fresh message clears it; `tilt_guard_stale_s` config default/validation; a goal publish refused for a stale state or `imu_ok` false at ok. `test_ws_snapshot.py`: the connect snapshot serializes each envelope once across reconnects (`test_connect_snapshot_reuses_serialized_frames`), `serialize_envelope` memoizes by envelope identity, reconnects from one remote are spaced while the first is not delayed, `broadcast_envelopes` yields between envelopes and drops the frame cache of a cleared topic, and a cancelled (timed-out) service future is removed from the rclpy client while a completed one is not. `test_nav_stop_coalesce.py` also covers a Stop after the coalesce window issuing a new cancel_all (`test_stop_after_new_goal_issues_new_cancel`). Frontend vitest: `frontend/src/tilt/tiltStatus.test.ts` (status by level, blocked flags, warning + `base_drive_inhibited` -> driveBlocked with the still-inhibited text, stale state and `imu_ok` false blocking goals at ok, `?` for missing angles, toast on entry/worsening only), `frontend/src/components/TiltBanner.test.tsx` (no banner at ok, amber warning, red error text) and `frontend/src/map/goalControl.test.ts` (Set goal disabled with the tilt tooltip at warning/error, top-view hint otherwise).
- Stop/reset coalescing (`nodes/web_ui/tests/test_nav_stop_coalesce.py`): `NAV_STOP_TIMEOUT_S` is 10 s; two concurrent `POST /api/nav/stop` produce one `cancel_all_goals_async` call and both get the same result (cached goal cleared once); a stop after completion calls again; a concurrent timeout gives both 504 with the 'may still complete' note and one call; a cancelled waiter does not cancel the shared call; two concurrent `POST /api/map/reset` produce one service call.
- Backpressure (`nodes/web_ui/tests/test_backpressure.py`): the sansio protocol subclass (paused `send()` waits for `resume_writing`, serves a real websocket under uvicorn); `ClientConnection` mailbox (latest wins per topic, ordered messages keep order, stalled client holds at most one frame per topic and is dropped after the send timeout, ordered queue overflow drops, send errors are not counted as slow, pending bytes); a stalled `/ws` client does not delay a fast one; `webui_ws_pending_*_max` gauges. `test_throttle.py` checks `ws=` is that subclass.
- Calibrate config (`nodes/web_ui/tests/test_config.py`): `test_calibration_config_needs_no_depthd_url` (`calibration: {}` is valid with the IMU defaults, `depthd_url` is not a model field), `test_calibration_rejections` (non-positive `imu_stale_s` / `imu_service_timeout_s` rejected).
- Model downloads (`nodes/web_ui/tests/test_models.py`): `GET /api/models` lists only the PDF and 3MF files of `models_dir`; `GET /models/{name}` serves each with `application/pdf` / `model/3mf` as an attachment; STL, text, missing and path-escaping names answer 404; without `models_dir` both routes answer 404.
- Calibrate backend (`nodes/web_ui/tests/test_imu_calibration.py`, built with `CalibrationConfig()`): `GET /api/calibration/config` returns exactly `{"imu": true, "arm": "deferred"}` with a `calibration` block and 404 `calibration page not configured` without; `GET /api/calibration/stereo/preview` is served by no API route (the stereo proxy and `test_calibration_proxy.py` are gone); IMU status (bridge cache, 503 without a sample or when stale), save and reset (Trigger success, 409 refusal, 503 unavailable, 504 timeout).
- Metrics (`nodes/web_ui/tests/test_metrics.py`): `GET /metrics` is not shadowed by the SPA static catch-all; `webui_http_requests_total` uses the matched route template (`/api/urdf/{path:path}`, `/api/tiles/{z}/{x}/{y}.png`, `static`) and status; `webui_ws_clients` / `webui_ws_disconnects_total` around a `/ws` session; `webui_broadcast_duration_seconds` and `webui_broadcaster_slow_total` with a slow bridge flush; `webui_topic_stale_total{topic}` once per outage and the `webui_topics_stale` gauge; `webui_map_updates_total` (map role only) and `webui_map_age_seconds`; `webui_robot_pose_ok` for fresh and missing TF; `webui_battery_cutoff_active` for low and recovered voltage.
- Camera tab (`camera` with `topics`, no longer shipped in the default config): `test_config.py` (topic labels load and are subscribed, label optional, `test_default_yaml_has_no_pi_cameras_tab`: `default.yaml` has no `pi_cameras` tab and no `camera` tab), `test_bridge.py` (both preview topics resolve to `CompressedImage`), frontend `CameraTab.test.tsx` (single stream unchanged, two labelled streams, per-stream waiting line, stacked on a phone and side by side on a desktop).
- Arm tab: `test_config.py::test_default_arm_tab_uses_the_head_camera` (the shipped default arm tab's `head_camera_topic` is `/head_camera/image_raw/compressed` and no topic of the retired front camera is subscribed).
- Head tab (`head`): `test_config.py` (`head` is a valid tab type; fields load and are subscribed; topics default to None and `tof_range_m` to 4.0 m; a non-positive range is rejected; `topic_roles` gives `head_rgb` / `head_depth` / `head_amplitude` and `tof_ranges` maps the depth topic to its range; `test_default_head_tab_subscribes_the_tof_topics`: the shipped default head tab subscribes exactly `/tof/depth/image_raw` (`head_depth`) and `/tof/amplitude/image_raw` (`head_amplitude`) plus the RGB `/head_camera/image_raw/compressed`; `test_head_tab_has_no_fused_depth_topic`: `fused_depth_topic` is not a `TabConfig` field and a YAML key of that name is neither subscribed nor given a role), `test_bridge.py` (head roles pick the message type for any topic name; the bridge subscribes the head topics; previews are rate limited per topic; a preview with a decode error is dropped), `test_msg_serializer.py` (greyscale depth preview over the ToF range: near bright, far and invalid black, beyond range black; depth without a range keeps the red-green preview; mono8 and mono16 amplitude to a grey JPEG, mono16 scaled to the frame; unknown or float encodings are never decoded as colour), `test_map3d_backend.py` (`head` is a kept tab type; the pinned default tab list is map_nav, arm, head, camera, poi; `test_client_yml_web_ui_tabs_validate_and_head_matches_default` validates the deployed `web_ui` tabs, the head tab right after arm with the same depth, amplitude and RGB topics as the default: `/tof/depth/image_raw`, `/tof/amplitude/image_raw`, `/head_camera/image_raw/compressed`). Root `tests/test_mcp_server_config.py`: the pinned deployed tab ids are map, agent, arm, head, poi and the head tab's type, label, topics (RGB `/head_camera/image_raw/compressed`, depth `/tof/depth/image_raw`, amplitude `/tof/amplitude/image_raw`, no `camera_info_topic`) and `tof_range_m` 4.0 are checked. Frontend vitest: `frontend/src/tabs/HeadTab.test.tsx` (RGB stream and depth preview render; amplitude preview only after the toggle, off by default; no amplitude switch when the tab has no amplitude topic; `Waiting for <topic>...` and no image for streams without a message; the pan/tilt block has every control disabled and publishes nothing; phone stacks the previews vertically), and the pinned lists `tabSelection.test.ts` (`SUPPORTED_TAB_TYPES` includes `head`) and `tabs/tabsDesign.test.ts` (HeadTab has no design violations).
- Head RGB-D view and Map tab "Head RGB-D" layer (`nodes/web_ui`): `test_head_rgbd.py` (the `rgbd_rgb_topic` / `rgbd_depth_topic` / `rgbd_cloud_topic` keys are `TabConfig` fields and survive loading; roles `head_rgbd_rgb`, `head_rgbd_depth`, `rgbd_cloud`, stamp-based rate gate (same forwarded stamps under arrival jitter), big-endian depth, `tof_ranges` and `all_subscribed_topics`; unset keys subscribe nothing; default.yaml and the client.yml `web_ui` block name the `/head_rgbd/*` topics; specs are best-effort depth 1 with the right message classes, the depth role shares the head rate limit and `rgbd_cloud` is a pose-frame role; `serialize_registered_depth` returns the full 288x162 raw depth with stamp, drops row padding, rejects other encodings and truncated data; ToF depth and compressed RGB payloads carry the header stamp; the cloud keeps colours, adds `frame`, and is capped at 20000 points from a 0.02 m voxel). Frontend vitest: `HeadTab.test.tsx` "RGB-D view" (switch only with both topics, topics requested only while selected, pairing by equal stamp, waiting message otherwise, no stale pair after a clear, hover readout in metres / `no depth` from the raw depth, opacity slider 0-100 default 50, legend range), `head/rgbd.test.ts` (payload parsing, depth decode, pairing history, turbo colormap with transparent invalid depth, readout, letterbox pixel mapping), `hooks/subscription.test.ts` (head RGB-D topics only for the open head tab in RGB-D view, head cloud topic only for the open map tab with the layer on), `map3d/layers.test.ts` (`headRgbd` key, label, default off, topic field), `map3d/cloudDecode.test.ts` (`anyFrame` and `rosAxes` options), `map3d/obstaclePoints.test.ts` (`rgbdCloudToMap` places points by `frame_pose`, nothing without a finite pose), `map/LayersPanel.test.tsx` (Head RGB-D row, disabled without a topic).

Also `test_map_guard_reset.py`: the bridge subscribes to `/map_guard/map_reset` (`std_msgs/Empty`) and each message drops the GPS anchor (estimator reset, `data: null`) and the cached map topics, with or without an anchor.

- Servo calibration backend (`nodes/web_ui/tests/test_servo_calibration.py`): servo group ids are unique, id and timeouts validated, derived topic and service names; `GET` returns fresh, stale and missing state plus the marker (absent marker, missing sample, no bridge); `POST` calls the derived service, a refusal is 409, 404 and 503 paths; save blocked under the battery cut-off while release is not; `/api/config` carries the `servos` flag; the bridge subscribes and caches servo state.
- Servo quiet (`nodes/web_ui/tests/test_servo_quiet.py`): error cases of the released command check; websocket publish is rejected while released and allowed when enabled, unknown or stale; arm routes answer 409 while released and are allowed when enabled; release and stop stay allowed while released.
- Servo calibrate frontend (`frontend/src/calibrate/servoApi.test.ts`, `ServoNeutral.test.tsx`, vitest): `fetchServoGroups` returns groups, 404 gives status 404, a network error status 0, a malformed body errors, a refused action (409) surfaces the backend message, `useServoGroups` treats 404 as not configured without an error and surfaces a server error; the ServoNeutral card shows chip states (null or stale = Unknown) and per-joint torque, Torque off posts release only after confirm, Record neutral is enabled only while released (absent without neutral) and shows each joint shift in degrees, save needs a capture and confirms (arm confirm carries stale and leader warnings), Re-enable torque posts without a dialog, the arm card warns when `calibration_stale` is non-empty, backend messages and refusals show in an Alert, buttons disable while a call is in flight, a poll network error is surfaced and a 404 poll shows the not-configured notice.
- Released banner in tabs (vitest): `tabs/ArmTab.test.tsx` ("ArmTab released group"), `tabs/HeadTab.test.tsx` ("HeadTab released group": banner, sliders and Home disabled) and `tabs/MapNavTab.toolbar.test.tsx` ("released groups": base released shows the banner, disables Set goal and keeps STOP enabled; arm released disables Arm home, Set home and Grasp).

### Per-node tests (claude_agent)

The **claude_agent** node has tests under `nodes/claude_agent/tests/` (no network, no real Claude calls; rclpy is stubbed in
`conftest.py`, the Agent SDK dataclasses are built directly). Run from `nodes/claude_agent`: `uv run pytest tests -q`.
Covers:

- config (`test_config.py`): defaults (opus, hard maxima rw 400 / turns 500, per phase 100 / 100, `max_instruction_time_min` 120, no `max_ro_cap` / `max_phase_ro_cap`, SDK `max_turns` = turn cap + margin, MCP URL/token file, port 18300, history 500, thumbnail 480, `/robot_events` topic, 50 events, 2 s debounce), the removed `max_turns` / `effector_call_cap` keys rejected,
  tool lists disjoint (stop can never be an effector), `get_arm_reach` a sensor tool (free, not effector-capped) in both the code
  defaults and the client.yml config, invalid numbers and unknown keys rejected, YAML loading, the plan tree and retry defaults
  (`max_loop_iterations` 5, `retry_success_threshold` 0.5, `max_auto_retries` 6) and their bounds, the same three keys set
  in the client.yml `claude_agent` config (Ansible kept in sync with the code defaults), `job_end_max_followups: 2` set in the client.yml `claude_agent` config,
  `CLAUDE_AGENT_CONFIG` lookup, `mcp_check_timeout_s` default 30 and > 0, MCP token read (env-file line or bare token, missing/empty refused, token never in the error)
- guard refusals (`test_guard_refusals.py`): the runner's `GUARD_REFUSAL_PATTERN` equals the mcp_server `guard_rules.py` constant (parsed with `ast`, no import); `agent_guard_refusals_total{rule}` counted once per tool call and rule (repeated text, several rules in one text, error results and `wait_for_event` results), dry_run log lines and non-matching errors or non-error results of other tools not counted; the prompt names `guard rule '<name>' refused`, `check_guard_rules`, `watch_constraints`, `resume_motions`, `gripper_effort`, `tilt_deg`, `distance_travelled_m`
- system prompt (`test_prompt.py`): the approach line (`suggest_approach_pose` before driving to grasp, then `approach_for_grasp` with the base lease, then `plan_grasp` / `grasp_object`; default lists classify `approach_for_grasp` as effector and `suggest_approach_pose` as sensor); the "Tilt guard" section (tilt_warning / tilt_error / tilt_clear, no Nav2 goals, manual short base moves, arm still and compact, never work around the guard); tool refusals come from the budget gate or errors, never from the person, and the generic "user doesn't want to proceed" text means re-run, not stop;
  grip profiles (`grip_profile` on set_gripper / grasp_object, gentle for fragile or light objects, soft for plush / soft toys, foam and fabric, normal for ordinary ones, firm for heavy, slippery objects and tools, check `holding_load` / `slipping` / `crush_risk` and retry firmer or gentler, soft plush gets squashed whatever the load; plush and soft toys go to `grip_profile soft` (not gentle or normal) and a relaxed hold reports `retightened`; the `unverified` grasp outcome: look at the attached gripper camera image, treat the object as held when it is in the jaws, else `release_object` and retry), pixel sizes (always pass `image_width` / `image_height` with pixels from a 384x288 `get_camera_image` picture, or use `get_annotated_camera_image`), the centred grasp (`grasp_shift`, `tool_point` beside the jaw centre by `shift_m` = half the object width plus the fixed jaw clearance is not a drift, `residual_error` / `warnings` / `held_pose`, `approach_speed_scale`), the phase-plan workflow (split into phases first, the four planning tools, the tomato example, per-phase rw/turn guidance ranges, no ro budget and unlimited sensor calls, generous rw/turn caps (double the estimate, raise early), the grasping guidance (several viewpoints by changing the roll incl. directly above, `pixel_to_ground`, average within 1 cm, centre not edge, correct by the observed offset, NOTES.md), retry budgeting (room for retries, raise the budget before it runs out or add a retry phase), fixed vs moving jaw and `object_width_m`, the agent chooses the grasp `wrist_roll`, camera views by roll, the rolling protocol (half open, arm lifted), `surface_height_m` and below-floor reach, stairs / tables / holes described as `surfaces` (step edge point and direction or polygon, height relative to the robot floor) instead of a single `surface_z_m` with `support_z` that must match, fast arm by default, phase and instruction maxima from the config, explicit honest `complete_phase`, once-per-instruction revision and once-per-phase raise, no waiting for approval, no old fixed caps), lists every tool by kind, body awareness / spatial perception / memory-POI-calibration sections (objects are POIs on the person's map, own POIs and objects removed on a new session, the person's POIs stay), safety rules, persona,
  `system_prompt_extra` appended
- epic prompt (`test_prompt.py`): control leases (take/release/get_control_state, head under either lease, arm_set_home denied, no acquire_control), the end-of-job rule (parked arm, released leases, finished plan, JOB END REFUSED, force-park after 2), dynamic footprint and carrying (carried_object_radius_m 0 to 0.3, parking refused while carrying), plan tree (seq/if/while/phase, pickloop, ask vs outcome_of, answer_condition), retry fields (p_retry_success, retry_change, Automatic retry n/m), more POIs, pick verification, lookups (list_lookups, get_lookup_image, annotate_lookup, photo POIs with the camera view, delete_photos / delete_poi only when asked, photos survive sessions) and the bump / lift interrupts; the force-park count follows `job_end_max_followups` ("after 3 refusals", "after 1 refusal", "at once" for 0) and the retry threshold value is stated (L17)
- motion queue prompt (`test_prompt.py`): the queue pattern (`enqueue_motions` with preconditions and `on_fail`,
  blending, queue at least 2 steps deep, `wait_for_event` instead of polling, `get_motion_status`, `cancel_motions` /
  `replace=true` on surprises, checkpoints only), the safety rules restated for queued motions (roll guard, negative
  elbow_flex stretches the arm, slow zone, stop), `look_around` `return_to_start`
- instruction time budget (`test_tools.py`, `test_runner.py`, `test_config.py`, `test_prompt.py`): `min_effector_time_left_s` default 60 and not
  negative; effector calls denied below the minimum with the explicit reason (time left, minimum, report and end the instruction) and not
  counted as rw; allowed at the minimum; stop, cancel_motions, release_object, acquire/release_control and an opening `set_gripper`
  allowed with no time left, a closing one denied; sensors and planning tools unaffected; a PostToolUse hook adds `instruction time left: N s`
  (never negative); the timeout `error` event names the cut-off call (`cut_off_call` name and elapsed) and stays plain with no call in flight;
  the prompt states the rule; the prompt asks set_task_plan for `time_budget_min` for the whole task (at most
  `max_instruction_time_min`), names the pre-plan watchdog (`instruction_timeout_s`), and says raise_phase_budget may be called more
  than once per phase and extends the time budget never past the ceiling (`test_prompt_teaches_the_time_budget_and_repeatable_raises`); it names time_budget_min as required, asks for time_budget_min and the caps at about twice the honest estimate, to prefer overshooting and delivering to undershooting and stopping, and to raise budgets early with raise_phase_budget rather than end unfinished (`test_prompt_plans_time_and_caps_at_twice_the_honest_estimate`)
- shutdown stop (`test_robot_stop.py`, `test_runner.py`): a refused connection is reported as `unreachable` with a plain
  reason (no raw `ExceptionGroup`), tool errors of a reachable server are not; at shutdown an unreachable mcp_server is
  logged at INFO (an ERROR outside shutdown) and a hanging stop is bounded by `shutdown_stop_timeout_s`
- direct tool calls (`test_robot_stop.py`): `call_robot_tool` against a real Streamable HTTP MCP server passes the arguments
  and returns the structured result as `data`; a missing token file is an error with no data; a hanging server is bounded
  and the timeout text names the tool
- job end (`test_job_end.py`, fake client plus fake tool caller `FakeToolCaller` in `test_runner.py`): the arm not at home and
  the arm lease held refused with a `JOB END REFUSED` follow-up naming both and `release_control(subsystem="all")` (no
  `complete_phase` clause without a plan), and the model fixing it (instruction ends `done`, no `job_end` field, no extra
  events); never fixed: 2 follow-ups (`job_end_max_followups` default), then the runner's own `cancel_motions` and
  `release_control(subsystem="all")` shown as `tool_call` / `tool_result` with `source: "job_end"` and `turn_end.job_end =
  {forced_park: true, result}`; a running motion queue cancelled before the release (`QueueRunningCaller` refuses the release
  until `cancel_motions`); a park leaving the arm out emits "arm not parked" before `turn_end`; a park refused while carrying
  falls back to `release_control(subsystem="all", keep_position=true)` (reported as `keep_position_release`) and still emits
  "arm not parked"; each park call bounded by `job_end_park_timeout_s`; `job_end_max_followups: 0` parks at once; a failing
  check (mcp_server unreachable, unreadable state) emits "job-end check failed" and ends `done` with `job_end.check_error`,
  without follow-ups; user stop, timeout, turn cap, `error` and `max_turns` skip the check, and a user Stop during the
  check's `get_control_state` (`StopDuringCheckCaller`) sends no follow-up and never parks (C1); carrying from
  `arm.carrying` (its reasons named in the follow-up; `carrying: false` wins over a stale footprint radius) or, without the
  field, from `footprint.carried_object_radius_m` puts "put the object down first (release_object)" before the park text,
  while grasp or refusal tool-result text alone no longer counts; with a finished plan the model can still call
  `release_object` through the gate and the job ends without a forced park (H9); an arm out of home the agent never held
  passes, one it took control of in this instruction is refused, a refused `take_control` does not count (M22); an
  unfinished plan names its active phase 1-based with the `complete_phase` / `revise_plan` remedy and ends with a forced park,
  a finished one passes; `plan_problems` / `plan_remedy` give one text per plan state (none, one active phase, several left,
  pending ask, finished; M25); the follow-up count resets per instruction. `test_config.py`: `job_end_max_followups` default
  2, 0 allowed, negative rejected; `job_end_park_timeout_s` default 60 and positive, `warning_interrupt_debounce_s` default 10
  and not negative, both set in client.yml. Existing runner tests close their plan (`completer()`) so the check passes
- plan (`test_budget.py`): `PlanTracker` validation (complexity enum, 1 to 12 phases, integer caps, rw 0 allowed, names and goals, a stray `ro_cap` ignored), clamping to the
  per-phase maxima with notes, rejection when the phase caps summed exceed an instruction maximum, the phase lifecycle (first phase active,
  `complete_phase` outcomes and next activation, plan end), sensor calls never refused (no plan, plan finished, turn cap used up) but counted, rw/turn counting and exact exhausted reasons, the once-per-phase raise
  (rationale, no lowering, limited to the remaining instruction room), the single `revise_plan` (closed phases kept, active one closed as failed,
  budget counts what closed phases used), the phase turn note, reset, callbacks, and the five SDK tools (schemas, texts, errors as error results);
  a failed close carries `p_retry_success` and `retry_change`, and the payload gains the tree fields (`id`, `depth`, `attempt`, ...)
- plan tree and retry policy (`test_plan_tree.py`): table-driven activation traces (plan, sequence of complete / answer calls,
  expected phase and pending-condition order) for seq, `while` with `outcome_of` (repeat until done, exit on `max_iterations`),
  `if` with `ask` (then, else, no else) and `outcome_of` (taken, not taken, `not_run`), nested `while` + `if`, an ask loop ending at
  `max_iterations` without asking again, and retries; nested loops and the innermost-loop `iteration`; a pending ask blocking
  effectors (and `complete_phase` / `raise_phase_budget`) but not sensors; `answer_condition` validation and the
  `condition_pending` / `condition_answered` callbacks; a plan starting with an ask; loop exit with reason `budget` (no rw room),
  `max_iterations` and `condition`; fresh, room-clamped caps and a new raise per re-entry; a phase entered for the first time after a
  loop used up the room clamped to what is left (rw use never passes `max_rw_cap`); effectors refused at the instruction
  rw maximum even if a phase cap is higher; a sensing-only loop running without rw
  room; the flat preorder payload with `id` / `depth` / `attempt` / `iteration` / `p_retry_success` and the `tree` field; flat
  phases converted to a seq with generated ids; exactly one of `phases` / `plan`; validation errors (depth, duplicate ids, 24
  nodes, 12 phases, `max_iterations` and `max_attempts` bounds and the config limit, unknown type, missing id / then / body,
  `outcome_of` naming an unknown or non-phase node or bad values, bad conditions, seq child count, caps); the plan-time sum
  counting each phase once with both if branches; `revise_plan` replacing a tree mid-loop (kept history, `outcome_of` naming an
  old phase, ids of kept phases rejected, a pending ask cleared); the retry policy: a failed close without `p_retry_success` /
  `retry_change` rejected, p 0.49 vs 0.5 at the threshold, re-entry with attempt n/m, fresh caps and the `Automatic retry n/m -
  apply: ...` text, attempts exhausted, the cap of 6 automatic retries across two phases, the configured threshold and cap, no
  retry with rw room 0 or turn room 0, a retry inside a `while` before the loop re-tests, a revision close needing no retry fields;
  the SDK tool schemas and handler texts (tree accepted, automatic retry, pending question, `answer_condition`, tree revise);
  the room counting turns and rw calls spent outside phases, also for a revision (M23); a loop re-entry text naming `Loop '<id>'
  iteration k/max` and `is now active` while an automatic retry says `is active again` (`retried` flag, M24); a raise note
  saying the cap stays when no room is left and no reservation for a branch not taken (L14); the budget no-retry note without
  "report to the user" and a phase entered with `rw_cap` 0 despite planned effectors saying so (L15); the retry threshold in
  the `complete_phase` description (L17)
- robot events (`test_robot_events.py`): `/robot_events` JSON parsing and normalization, idle events emitted but never interrupting, bounded
  history, non-critical events not interrupting, a critical event interrupting the running step and continuing the same instruction
  with the follow-up message (no robot stop, plan and turns carried over), the 2 s debounce, thread-safe hand-over from the rclpy thread,
  garbage and unbound-loop events dropped, a user stop winning over a pending follow-up, a critical event interrupting at once
  even while a phase-note interrupt waits for running tool calls; an event during the job-end check never interrupting (its
  text goes into the JOB END follow-up) while a critical event in the following JOB END stream still does (H10); warning
  interrupts debounced per type by `warning_interrupt_debounce_s` (M26); a follow-up taken before its scheduled interrupt
  landed cancels that interrupt so it never hits the next query (L18); tilt guard events (`tilt_warning`, `tilt_error`) interrupting with the event text in the follow-up, `tilt_clear` (info) not interrupting; warning events of a type in `interrupt_warning_types` (default `bump`) waiting for the tool call in flight and then interrupting with a `ROBOT EVENT (warning)` follow-up (also immediate with no call in flight), a warning of an unlisted type and an info event of a listed type not interrupting, the type list read from config, and a critical `map_guard_disturbed` (lifted) interrupting at once despite a call in flight; `test_config.py` covers the `interrupt_warning_types` default and the client.yml value
- control leases and denied tools (`test_tools.py`, `test_config.py`): `take_control` / `release_control` uncapped (never counted, never refused with no time left), `get_control_state` a sensor, `arm_set_home` classified `denied` and refused with the reason "arm_set_home is reserved for humans: the job-end check compares against the stored home" with or without a plan and never counted; `denied_tools` may not overlap the other tool lists.
- tools (`test_tools.py`): classification (sensor / effector / uncapped / plan, unknown robot tool counted as rw, non-robot unclassified),
  built-in tool deny list, effector tools denied until a plan is set while sensors stay allowed (notes tools, the planning tools, `stop` and the
  control tools too), sensors counted but never denied and effectors denied at the rw cap with the exact messages, the phase turn cap, denial after the last phase, reset,
  everything outside `mcp__robot__*` denied, count and denial callbacks, concurrent permission requests (a parallel effector +
  sensor call both allowed; past the rw cap the extra effectors denied with the budget reason, the sensor allowed), an exception
  inside the gate or its denial callback turned into an explicit, logged `internal error in the permission gate` denial;
  `answer_condition` allowed as a planning tool, a pending ask denying effectors (sensors allowed) until answered, the condition
  callbacks forwarded to the tracker; `release_object` and an opening `set_gripper` allowed and counted with a finished plan, a
  pending ask, an exhausted cap or no plan, a closing `set_gripper` still refused (H9)
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
  `phase_completed` / `plan_revised` and `state` events (ro/rw/turns/plan), `condition_pending` / `condition_answered` events of a plan
  tree and the tree fields of `phase_started` / `phase_completed` (`id`, `attempt`, `iteration`, `depth`, `p_retry_success`,
  `retry_change`, an automatic retry emitting a second `phase_started`) and the plan's `tree`, the instruction turn maximum (interrupt, robot stop, `turn_cap` status), the phase turn cap note (follow-up, no stop), the phase note interrupt deferred until a parallel tool call in flight finished (the 2026-10-11 cancelled-camera-call sequence), Claude Code's generic "user doesn't want to proceed" result corrected in a follow-up with a `tool_cancelled` event (not after a user stop), denial events,
  auth error, session failure and missing token become error events, reset starts a new session and deletes no POIs (an HTTP spy sees no clear call; an `agent_poi_reset` key is rejected)
- robot tools check (`test_runner.py`): a connected robot MCP server runs the instruction with no probe or reconnect; a failed
  server that answers is reconnected in place (`robot_tools_reconnected` event, session kept); a failed reconnect restarts the
  session and says so; an unreachable mcp_server fails the instruction at once (`robot server unreachable`, no query); `pending`
  is waited out; still failing after a restart is `robot tools unavailable`; a failing status query is logged and the
  instruction runs
- mcp_server probe (`test_mcp_health.py`, uvicorn on an ephemeral port): `/metrics` URL derivation, 200 = reachable, an error
  status, a hung server (timeout) and a closed port become reasons, never exceptions
- API (`test_api.py`): `/api/state` (plan, active phase, usage, hard and per-phase maxima), `/api/history`, `/api/message` (202 / 409 busy / 400 empty or invalid), `/api/stop`,
  `/api/reset` (409 while busy), `/ws/events` history on connect then live events, shutdown hook
- entry point (`test_main.py`): loopback bind and port from config, the `/robot_events` subscription (volatile QoS) and its callback, no POI publisher is created (the clear goes over HTTP), ROS2 logger, API key removed from the process env, invalid config exits 1, uvicorn `timeout_graceful_shutdown` is the 3 s constant
- clean stop (`test_runner.py`): `close()` finishes within a bound with a hung model interrupt, a hung disconnect and a background task that never ends (task cancelled, client dropped); the CLI subprocess is terminated, killed when it ignores terminate and left alone when already exited; an instruction in progress is recorded as an `error` "interrupted by service shutdown" plus one `interrupted` `turn_end`, an idle close records no `turn_end`
- retrospective after every job and feed (`test_retro_followup.py`, per-query scripted fake client; the runner's `retro_sink` is captured from `build_plan_server` and called through the real `record_retrospective` tool): a normal end gets one `RETROSPECTIVE` prompt after `JOB END REFUSED`, `WORKFLOW CHECK` and `POI REVIEW` (full form names the `robot-skills:retrospective` skill) and stores the agent record with the facts (instruction, workflow, phases, skipped_mandatory, cost and turns refreshed to the `turn_end` values, effector calls, duration, trivial); no RETROSPECTIVE prompt when the agent already recorded one; state reset per instruction; trivial jobs (no plan or complexity trivial, zero effector calls) get the short form, a trivial plan with an effector call the full form; runner records when the agent skips the call, with the flag off, after a skipped mandatory step (partial), after a user stop, timeout, error result, session exception, `max_turns` and the turn cap (no model turn after any of them); a stop during the RETROSPECTIVE turn keeps the agent record; an unwritable store never breaks `turn_end`; `retrospective_recorded` {id, author, outcome, workflow, instruction_id} before `turn_end`; a second `record_retrospective` replaces the first (same id, two events); `RetroStore.replace`; the RECENT RETROSPECTIVES feed on the first instruction of a session only (again after a reset), sized by `retro_feed_count` / `retro_feed_max_chars`, off at 0, absent for an empty store, before the digest under `fresh_with_digest`, the `user_message` text unchanged; the store under `<state_dir>`. `make_runner` keeps the RETROSPECTIVE prompt and the feed off unless `retro_followup` / `retro_feed` ask for them

- Metrics (`test_metrics.py`): `agent_busy` during and after an instruction; `agent_instructions_total{status}` and `agent_instruction_duration_seconds` for done, interrupted, error and timeout; `agent_watchdog_fires_total`; `agent_turns_total` and `agent_tool_uses_total{tool}` from a scripted stream; `agent_session_resets_total` (refused reset not counted); `agent_tokens_total{kind}` and `agent_cost_usd_total` as session-total deltas (absent usage adds nothing); `agent_seconds_since_activity` computed at scrape time; `GET /metrics` route.

### Per-node tests (mcp_server)

The **mcp_server** node has tests under `nodes/mcp_server/tests/` (no ROS needed: rclpy is imported only by
`ros_iface.py` and `__main__.py`). Run from `nodes/mcp_server`: `uv run pytest tests -q` (or `uv run poe test`).
`fakes.py` provides a simulated follower arm backend with a fake clock. Covers:

- config (`test_config.py`): defaults (0.0.0.0:18200 `/mcp`, topics, conservative limits, timeouts), repo-relative URDF
  path resolution, YAML overrides, empty file, unknown keys rejected, hard caps (0.25 m/s, speed scale 0.5, 1024 px),
  `MCP_SERVER_CONFIG` lookup, `MCP_SERVER_TOKEN` refused when missing/blank/short and stripped otherwise, settle
  tolerance default 0.08 and validated between the converge tolerance and the tracking abort (defaults 0.25 rad + 0 s x velocity: the delay window carries the lag); the `imu_level_reference` code default equals the swerve tilt guard's `DEFAULT_LEVEL_REFERENCE_*_DEG` (parity test reading its source); arm velocity default 1.0
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
- object memory (`test_object_memory.py`, against a fake poi_store): objects are added as `kind: object`, `created_by:
  agent` POIs with sighting fields, same-label merge within 0.25 m through an `update` (case-insensitive, nearest
  candidate, weighted average, times_seen, max confidence, note kept unless given), different label or too far = new
  object, other POI kinds are never merged into nor forgotten, hand-made object POIs get times_seen 1, forget deletes,
  input validation, poi_store down = RobotError, distance/bearing from the robot heading, label / near-point filters, no
  pose = no distance; one-time import of the legacy `objects.json` (fields kept, file renamed `.migrated`, no second
  import, retried while poi_store is down, empty file renamed, corrupt file moved aside)
- navigation footprint (`test_footprint.py`): static / dynamic / conservative footprints, arm shadow and carried circle,
  republish tick and consumers; a keep_position release without fresh joint states publishes the conservative
  footprint with the carried circle at once (kind `conservative`, no "footprint unchanged" outcome) and the tick keeps it
- look_around (`test_look_around.py`): equal-step plan covering 360 deg and its limits, clearance = footprint
  circumscribed radius + margin, refusal on a close obstacle or a missing lidar (robot not moved), the rotate-capture
  loop (3 rotations between 4 stops plus the closing step, camera and lidar at every stop, expected vs achieved
  rotation and heading error), interrupted / failed steps end the sequence with `interrupted_by` and no return
  rotation, `stop` between steps, an obstacle appearing mid-turn, a camera failure recorded (not fabricated), montage
- POI client (`test_poi_client.py`): request published with op, poi and a fresh `request_id`; the result is matched by
  id (other ids ignored, answer from another thread), timeout, store not running (nothing published), store rejection,
  malformed results dropped, `/poi/list` parsing, `clear(created_by)` (publishes `op: clear` + `created_by`, returns the
  removed count; `PoiStoreDown` / `PoiTimeout` for a missing or silent store)
- admin route (`test_tools.py`): `POST /admin/clear_agent_pois` returns 401 without, with a wrong or with a non-Bearer
  token (nothing cleared), 200 `{"ok": true, "removed": n}` clearing only agent POIs, 503 when poi_store is down, 504 on
  timeout, 502 on rejection, and no MCP tool exposes it
- perception tools (`test_perception_tools.py`): the nine new tools registered and classified (`look_around` in
  `MOTION_TOOLS`, the rest always allowed), `get_topdown_view` PNG + metadata (pose, scale, layers present/missing,
  data ages), the reach circle centred on the shoulder_pan axis (`arm.base_in_base_link` plus the 38.8 mm URDF pan
  origin, rotated by the mount yaw; base_link origin without a mount), layer selection and validation, POI/object layers (each POI drawn once: `pois` without objects, `objects` only object POIs; both missing with the reason when poi_store is down), object tools round trip and merge as object POIs (`list_objects` only objects, `list_pois` states `kind`), legacy `objects.json` import through the server, POI tools
  (point defaults to the robot position, area polygon, `created_by` agent, argument errors, store not running),
  `list_pois` distance/bearing/status/near, `look_around` (montage + top-down images, summary, config default
  captures, argument validation, refusal does not move)
- executor hardening and camera frame cache (`test_spin.py`): `spin_forever` survives a stale-handle error and
  re-raises others; `run_or_exit` exits the process when the executor thread dies or stops while ROS runs (systemd
  restarts the node) and stays quiet on shutdown; `FrameCache.wait_newer` returns the first frame newer than the
  request or None on timeout.
- arm controller (`test_arm.py`): lease acquire/release, streamed motion with implicit acquire and velocity cap, speed
  scale, clamping, argument validation, abort + hold on stale feedback / tracking error / stop (tracking limit grows
  with the motion's velocity: `tracking_limit`; a follower 0.35 s behind passes at full speed through the delay window, a 0.4 rad offset aborts at 0.2 rad/s), no hold after filter_node
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
  (also after the hold window), the squeeze never passes closed, a partial open_fraction stall is not a grasp; the
  2026-10-10 cases: a 39 mm jar contact after 0.147 rad travel is a grasp (`gripper_grasp_min_travel_rad` 0.10), the
  empty firm close stalling 0.055 rad before closed at load 148 and a load contact there are `closed_no_contact`
  ("empty"), never `grasped`, with the default torque restored, the threshold from `gripper_empty_stall_rad` (0.08);
  wrist roll
  guard (refused with a wide open gripper measured or targeted in the same call, nothing published and no lease taken,
  allowed at half open or for a change under the minimum, thresholds from config); `move_cartesian` with `wrist_roll`
  (replaces the current roll, clamped and reported in `clamped`, guard applies) and `object_width_m` (centre reached with the
  shift, the fixed jaw 7.5 mm clear of a 40 mm object's side face (half the width plus `jaw_clearance`, as in a planned
  centred grasp), `grasp_shift` reported with `shift_m` and `fixed_jaw_clearance_m`, widths 0 / negative / 0.09 / NaN rejected); limit overrides widen the clamp of `move_joints`
  and make a below-floor Cartesian target reachable
- drive (`test_base_motion.py`): rate, clamping, duration cap, abort always ends with a zero twist
- run_dock (`test_base_dock.py`, `FakeDockPort`): server down and no answer raise `RobotError` before/without moving, rejection, success with last feedback state, every failure code (901..906, 999) mapped to its name with retries and message, unknown code name, succeeded-with-error is failed, server-cancelled result, interrupt = cancel + zero + `interrupted` (interrupt wins over stop), stop = cancel + `canceled`, timeout = cancel + `timeout`, cancel on every early exit and only zero twists ever published, `mcp_dock_goals_total` counted
- ROS adapters of the approach tools (`nodes/mcp_server/tests/test_ros_dock.py`, ros stubs as in `test_ros_iface_footprint.py`, message classes replaced by attribute bags, fake action clients, no motion): DockRobot goal field mapping (`use_dock_id` false, map pose and yaw quaternion, type, staging flag, max staging time), accepted / rejected / no-answer goals, feedback state names, result mapping and unknown result, own goal registry, cancel; ComputePathToPose result mapping (ok with path length, aborted / empty = `no_path`, server down / rejected / silent / no goal answer = `unavailable`, an unwanted plan is cancelled); the lazy global costmap cache (no subscription until first use, row-major layout and origin, origin yaw, age, stale grid returned with its age, wait for the first message, wrong-size message ignored); `DetectionPublisher` (frame map, capture stamp, nothing published at construction, other frame refused); `RosRobot` wiring (`cancel_all_goals` covers both actions with the zero goal id even when the nav cancel fails, a docking server that is not running costs no wait, an unanswered dock cancel is reported, `stop` reaches the dock cancel, `reset_base` cancels own dock goals, `dock()` runs `run_dock` under the base motion lock and refuses when busy, `plan_path` / `global_costmap` / `publish_detection` delegate)
- camera target tracker (`nodes/mcp_server/tests/test_target_tracker.py`, a seeded random floor texture laid out in the map and rendered through the calibrated synthetic front camera of `camera_helpers.py`, no ROS): `visible` true at the image centre, false behind the camera, off the image, inside the edge margin and when the scene cannot reach base_link; `dock_pose_from_target` puts the target at `t_rel` in the dock frame; seeding from the projected target and recovering it within 1 cm at a second, closer distance (capture stamp kept, dock pose from the detected target); an agent-pixel seed recovers the true target within 1 cm when the plan is 5 cm off; a different floor scores below `min_score` and returns None; a 13 cm jump is rejected and counted (accepted with a looser bound); the template scale equals the predicted camera distance ratio; an out-of-view target does not seed; `DetectionLoop` publishes a match once per frame (the seed frame and a repeated frame publish nothing) and stops pulling frames after `stop()`, never publishes without a match, and survives missing frames and source errors
- tools (`test_tools.py`): all 41 tools (`TOOL_NAMES`) registered with real descriptions (arm motion tools explain `residual_error`
  and the commanded hold), `speed_scale` descriptions derived from `arm_max_joint_velocity_rps` (no hardcoded 0.5 rad/s),
  `move_arm_cartesian` accepting `wrist_roll` / `object_width_m` (bounds, `grasp_shift` in the result, description phrases),
  the roll guard as a tool error, structured outputs, camera JPEG + stamp,
  argument validation, robot errors as tool errors, map PNG, arm tool round trip (take_control, moves, release_control
  keep_position), take/release_control descriptions require an explicit release, `arm_home` refused without the arm
  lease (nothing published) and keeping a held lease; the `take(server, ...)` helper takes leases for the motion tests;
  bearer-token auth on the Streamable
  HTTP app (401 without/with a wrong token, 200 with the right one) and the configured path
- battery gate (`test_battery_gate.py`): `MOTION_TOOLS` and `ALWAYS_ALLOWED_TOOLS` partition all tools; each motion tool
  is refused in cut-off with the exact message, a logged warning and no robot call (both leases held, so the battery
  gate is what refuses); `stop`, sensor, state and take/release/get_control_state still work in cut-off; motion works with a good or unknown battery, without a guard and after
  recovery; optional `battery` config section (absent = off, validated)
- monitor (`test_monitor.py`): servo overheat warning/critical with hysteresis, debounce and `_cleared` info events, status
  error bits decoded (critical), battery low/cut-off from the shared guard, IMU bump (baseline removed, debounce,
  warning/critical) and tilt, wheel slip from the swerve twist covariance (parked = no residual), base stall (needs a
  running base motion, commanded speed and fresh ~0 odometry for 1 s), collision STOP only during a base motion (a STOP
  latched before the motion fires after the 0.5 s grace, a fresh state clears it), human takeover only while the lease is held, CPU temperature, digest cursor/cap/vitals line, `MotionWatch`
  (only relevant critical events after creation, live battery cut-off), body state nulls and notes, last 10 events,
  failing sink tolerated, threshold validation
- tilt guard (`test_tilt_guard.py`, consumer of `/tilt_guard/state`): entering warning emits a critical `tilt_warning`
  (roll/pitch vs limits, "automatic navigation is disabled", manual drive-back instruction) and cancels every Nav2 goal
  once (repeated 1 Hz states at the same level emit nothing); error entry (from warning or ok) emits a critical
  `tilt_error` saying the base drive is inhibited and cancels; error back to warning re-emits `tilt_warning` (base drive
  enabled again, or "base drive still inhibited" when `base_drive_inhibited` stays true); the inhibit released while
  still at warning is an info `tilt_drive_enabled`; clear is an info `tilt_clear` without cancel; malformed states dropped; `tilt_warning` / `tilt_error`
  interrupt running base motions (`BASE_INTERRUPTS`); a failing canceller is tolerated and, after every retry failed,
  raises a critical `tilt_nav_cancel_failed` (the helper runs the cancel inline with a no-op sleep); an unanswered
  cancel is retried until answered (3 calls); refusal reasons by level (warning:
  nav only, error: nav and manual, "base inhibited by the tilt guard"); manual drive, the allowed flags and the vitals
  line follow `base_drive_inhibited` at warning, not only the level; a stale state keeps its level enforced and is
  flagged; at level ok a stale state or `imu_ok: false` refuses navigation (manual drive allowed,
  `navigation_allowed` false) until a fresh state with the IMU arrives; the IMU fallback `tilt` warning is silent
  while the guard reports and returns when it goes stale; body state
  `tilt_guard` fields (level, roll/pitch, limits, allowed flags, age) and the vitals line; tools: classification
  (`TILT_NAV_TOOLS`, `TILT_MANUAL_BASE_TOOLS`), `navigate_to_pose` / `move_relative` / `look_around` refused with the
  reason in warning and error without touching the robot, `drive` allowed in warning, refused in warning while the base stays inhibited and refused in error, arm tools
  unaffected in error, `enqueue_motions` with a base step refused (arm/wait-only steps allowed), digest vitals carry the
  guard, a queued base step fails at dispatch (status `tilt_guard`); the tool tests hold both leases so the tilt guard
  is what refuses
- control leases (`test_control.py`, `control.py`): `steering_neutral` per steer joint (missing joint or no data = None);
  the steering waiter succeeds once the wheels recenter and times out naming the worst joint (or no fresh joint
  states); `take_control` is idempotent (held_since kept, the hold not re-published) for arm and base; default
  `release(arm)` runs the home motion and then publishes the release (release last, once), `keep_position` only
  releases, no home pose = `failed: no home pose stored` and a home that does not converge = `failed: ...` both still
  release, battery cut-off = `skipped: battery_cutoff` with only the release published; `release(base)` zeroes and
  reports `neutral`, a steering timeout reports `failed: ... not neutral` after `base_reset_timeout_s` and still
  releases, `keep_position` only clears the flag; `release(all)` resets the base and skips an arm not held; release
  refused while the queue is busy (lease kept); `at_home` tolerance edges (0.99 / 1.01 x the shoulder_lift settle
  tolerance), gripper excluded, None without home or fresh joint states; `TOOL_SUBSYSTEM` table (arm / base tools,
  lease-free tools absent); arm and base tools refused without their lease with the exact text before any publish;
  tools work once the lease is taken; a lease released by `/arm/home` names it in the refusal; `/arm/home` still
  releases; `arm_home` requires and keeps the lease; an enqueue mixing a base and a gripper step with only base held
  is refused naming arm only (nothing queued); lease-free steps need no lease; the lease refusal comes before the
  tilt refusal (navigate_to_pose and enqueue at tilt warning), the tilt refusal after taking the lease;
  `take_control` allowed in battery cut-off and `release_control` then skips the arm reset;
  `get_control_state` / `release_control(all)` payloads; the vitals carry `ctl arm=-@home base=-` /
  `ctl arm=H@away base=H`; the `any` need is satisfied by either lease; a queued gripper step fails at dispatch when
  the arm lease is not held
- control leases and the carrying signal (`test_control_carrying.py`, `control.py` / `arm.py` / `carry.py`): lease
  generation (H1): a `grasp_object` whose planning is interrupted by the web UI Home (`/arm/home`) ends `aborted` with
  `arm control not held (released by the /arm/home service ...)` and the arm is not re-taken (the next arm tool is
  refused), a queued grasp step whose lease was released and implicitly re-taken during planning fails
  `arm control not held`, a motion without an expected lease still acquires implicitly (GraspService, /arm/home), an
  expected generation that no longer matches refuses the motion; agent lease (H2): a web UI grasp's implicit hold does
  not let `move_arm_joints` pass, `get_control_state` shows `held: false`, `held_by: mcp_server`, and `take_control`
  adopts it (`held_by: agent`); a stop during a foreign motion holds the arm but grants no lease (release says not
  held); a lease ended and re-taken by someone else is no longer the agent's; grip torque (H3): a keep_position
  release after a soft grasp writes no torque_limit, the parked release after an open ends at the default torque; a
  keep release without carrying restores the default; carrying (H4, M4, M6, live L2): after an `unverified` grasp the
  carrying state names the unverified hold and `release_control(arm)` is refused, an open clears it, `release(all)`
  while carrying releases and resets the base and reports `reset.arm = refused: carrying (...)` with the arm lease
  kept, a measured jaw wider than `gripper_grasp_max_open_rad` clears grasp_held and the radius, radius 0 does not
  block parking, `take_control` / `get_control_state` show `arm.carrying` with the radius after a keep release and the
  refusal names it, an empty carrying state; persistence (H7): the carry record survives a restarted ArmController
  (park still refused), the orphan release notes `mcp_server restarted`; release refused while an arm motion or another blocking tool runs
  (release_control itself does not count) (M1); base tools refused during the base reset and a `take_control('base')`
  during the steering wait stays held (M3); a stop during the park keeps the lease (`stopped: ...`, no release) (L3);
  `filter_node reports no active source (restarted?)` after an active source `none` (L2); the home pose is read once
  until the file changes (L1); `NavGoalRegistry` cancels only registered goals, a discarded goal is skipped, nothing to
  cancel is a success and an unanswered cancel reports failure; the registry is thread-safe (M2)
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
- camera lookup store (`test_lookup_store.py`, tmp_path store with a fake clock): one JPEG + one `index.jsonl` line
  per add with the full `LookupRecord`, optional context fields null, prune oldest first by count / bytes / age (the
  record just written is never pruned), corrupt and blank index lines skipped, a missing store reads empty,
  `annotate` rewrites only its own line (corrupt lines kept, omitted fields kept, unknown id raises), queries newest
  first with default limit 10 and limit capped at 20, the filter matrix (camera, mcp_session, poi_id, case-insensitive
  text, near with pose-less records never matching, combined filters) and the inclusive since / until window. The
  package `conftest.py` points the default store directory at each test's tmp dir so no test writes to /var/lib
- camera lookup tools (`test_lookup_tools.py`): the three tools are registered, always allowed and not motion tools;
  retention defaults; `get_camera_image` writes exactly one record whose JPEG is byte-identical to the returned image
  (pose, arm joints, session) and returns its `lookup_id`; a failed camera call writes nothing; the MCP session id
  reaches the record and the `mcp_session` filter; an unwritable store does not fail the camera tool (`lookup_id`
  null, `mcp_lookup_write_failures_total` + 1); a failing pose read still records; `get_annotated_camera_image`
  records the annotated JPEG with `camera_pose_base`; `list_lookups` newest first, metadata only, limit and its cap,
  near / camera / poi_id / text / time filters; `get_lookup_image` returns the image and metadata, downscales with
  `max_px`, unknown id errors; `annotate_lookup` rewrites the line and errors on an unknown id
- camera tools (`test_camera_tools.py`): the seven tools and descriptions, documented not-calibrated errors,
  `pixel_to_ground` (front/gripper, arm offset, missing map pose, stale joints, sky), annotated images (all overlays,
  notes for overlays that cannot be drawn, validation), candidate points (table vs projection, sky skipped, region, last
  10 sets, stored values with moved flags), `surface_height_m` for `pixel_to_ground` and `mark_candidate_points` (raised
  plane hit, validation, stored and resolved with the set), capture/solve/clear round trip; pixels from a smaller
  image (2026-10-10: 384x288 `get_camera_image` vs the 640x480 calibration): `pixel_to_ground` with `image_width` /
  `image_height` scales to the calibrated size (same floor point, `pixel_input` reported, `image` always reported,
  unchanged without a size), both sizes required and the aspect kept, `mark_candidate_points` scales its region and
  reports `image`, `resolve_candidate` states the image size, `capture_calibration_sample` stores the scaled pixel,
  `get_camera_image` reports `calibrated_image`, `scale_to_calibrated` and a `pixel_note`, and the descriptions
  explain `image_width` / `image_height` (384x288)
- IK link frames (`test_ik.py`): `link_frame` for base, `gripper_link`, `gripper_frame_link` and rejected off-chain links
- digest + body state (`test_digest.py`): every tool result carries `robot_events_since_last_call` and `vitals` (text,
  structured content, `_meta` for image tools, appended to tool errors; the vitals line ends with the `ctl` lease
  summary), events reported once, events raised during the
  call included, a tool registered by a later module gets it for free, `get_body_state` content/nulls, never refused in
  cut-off
- early return (`test_early_return.py`): arm motions end `interrupted` with `interrupted_by` on overheat / servo error /
  battery cut-off mid-motion / human takeover (lease dropped, nothing published against the human) and hold the
  measured pose, warnings do not interrupt, tracking abort is a `stall` event, gripper and `arm_home` interrupts,
  expected/achieved/duration and Cartesian tool poses; drive interrupt (zero twist, status, duration), twist
  integration and relative pose; navigation via a fake Nav port (success, interrupt cancels goal and zeroes the base,
  stop, timeout, rejected, unavailable, real monitor collision stop)
- on-demand subscriptions (`test_on_demand_subscriptions.py`, `conftest.py` `ros_iface` fixture): an idle `RosRobot`
  has no camera, `/scan_filtered` or `/tf` subscription (only `/tf_static`); `camera_image` subscribes the asked camera
  and returns a real JPEG frame from a fake publisher thread, or fails after `image_timeout_s` without one; the reaper
  destroys a camera subscription only after `on_demand.linger_s` unused (a call restarts the linger) and the next call
  subscribes again; `scan_points` subscribes, waits at most `timeouts.state_stale_s` for a first scan (once) and a
  reaped scan drops its sample; a TF lookup subscribes `/tf`, feeds `set_transform`, waits longer right after
  subscribing, and reaping clears the buffer and re-inserts the static transforms; every `MOTION_CRITICAL_TOPICS` topic
  and the battery topic is subscribed once at start (a dual-rate joint-state field on its throttled copy) and never
  destroyed (the list names `base_servo_registers`); a JSON dump on `/swerve_drive/servo_registers` reaches
  `RobotMonitor` (`fl_drive` known, critical `overheat`); the reaper timer period, the `linger_s` default and validation, and the `client.yml` value
- dual-rate joint states (`test_joint_state_dual_rate.py`, `conftest.py` `ros_iface` fixture, a fake clock, fake
  `FakeArmBackend` / `FakeRobot` recorders): an idle `RosRobot` subscribes only `/follower/joint_states/throttled`
  (raw, best effort); `begin_motion` subscribes the full-rate topic and returns once a full-rate sample is cached,
  bounded by `timeouts.state_stale_s` (the throttled feed keeps the cache fresh meanwhile and is ignored once full rate
  delivers); subscriptions are destroyed only by the monitor tick; full rate stays during a motion and until
  `joint_state_rate.linger_s` after the last one (overlapping motions counted), then the copy returns and the full-rate
  subscription is dropped once it delivers; the fallback to full rate (one warning) when the copy has no publisher or
  stays silent, and the recovery; a throttled sample older than `follower_stale_s` is not fresh; the README consumer
  audit names both topics and its full-rate-at-rest rows equal `FULL_RATE_AT_REST`, and a field in that list stays on
  full rate; `enabled: false`; the lazy swerve joint states read the copy while the base is idle and full rate while it
  moves (also when first read during a base motion); `drive` and `navigate` hold the base at full rate; an arm motion
  and a gripper close begin full rate before their first command and the rest baseline; the base and arm MCP tools
  begin before the command and end after it, `release_control` switches both, sensor tools none; a tool cancelled
  while `begin_motion` waits still calls `end_motion` (no leaked active count); the
  `JointStateRateSettings` defaults and validation and the `client.yml` block matching the throttle node commands
- rclpy isolation (`test_rclpy_isolation.py`): only `ros_iface.py` / `__main__.py` import ROS packages
- lazy swerve joint states (`test_ros_iface_steer.py`, the `conftest.py` `ros_iface` fixture imports `ros_iface` against
  mocked ROS packages): `RosRobot` does not subscribe `/swerve_drive/joint_states` at startup; the first
  `steer_positions` subscribes, waits `control.steer_states_wait_s` for a first message and returns None without one,
  then the steer joint positions of a fresh message (drive joints left out, subscribed once); a second call without
  data does not wait again; the reaper destroys the subscription (and drops its sample) only once it is unread for
  `control.steer_states_idle_s`, a read resets the idle time, and the next read subscribes again; the reaper timer
  runs at most every half idle period; `test_config.py` covers the 0.5 s / 30 s defaults and rejects 0
- guard rules engine (`test_guard_rules.py`, pure `guard_rules.py`): constants against the node -
  `test_guardable_tools_are_motion_tools_without_arm_set_home_plus_take_control_minus_recovery` (`GUARDABLE_TOOLS` ==
  `tools.MOTION_TOOLS - {arm_set_home} | {take_control}` minus `NEVER_GATED`, so `arm_home` stays recovery),
  `test_never_gated_is_disjoint_from_guardable_and_holds_the_recovery_tools`,
  `test_every_tool_outside_guardable_tools_is_rejected_by_validation` (every read-only / recovery tool in
  `tools.TOOL_NAMES`), `test_step_kinds_equal_the_motion_queue_discriminator_kinds`,
  `test_arm_origins_are_the_arm_tools_and_arm_step_kinds`, `test_arm_joints_copy_equals_config`,
  `test_guard_rules_does_not_import_tools_or_motion_queue_or_ros`; field registry - `test_field_registry_exact_names`,
  `test_field_registry_joint_patterns` (`arm.joint.` / `target.joint.` / `motion.abs_delta.` + every ARM_JOINTS name,
  unknown joint rejected), `test_field_registry_args_pattern`, `test_field_registry_types_and_phases` (enum choices,
  phases); load-time validation matrix - `test_minimal_rule_validates_with_defaults`, `test_condition_validation_matrix`
  (unknown field, op/type mismatch, non-number / bool / infinite threshold, unknown enum value, list vs single value,
  unknown op, extra key), `test_phase_mismatch_is_rejected` / `test_phase_fit_is_accepted` (`args.*` only with
  tools/steps, `target.`/`start.`/`motion.*` only with arm_motion_from, `tool.name` / `step.kind` in their phase),
  `test_applies_to_validation` (never-gated, non-guardable, unknown step kind or arm origin, empty scope, extra key),
  `test_rule_validation` (neither all nor any, name pattern, message length, mode, when_unknown, extra key),
  `test_rule_without_all_but_with_any_is_valid`, `test_duplicate_rule_names_are_rejected`,
  `test_settings_default_is_enabled_without_rules_and_rejects_unknown_keys`, `test_validation_errors_are_value_errors`
  (EX_CONFIG path); Kleene evaluation - `test_kleene_all` / `test_kleene_any` (full 3x3 truth tables with
  when_unknown reject and allow), `test_kleene_all_and_any_blocks_combine_with_and`,
  `test_known_false_passes_even_with_unknown_fields`, `test_condition_truth_values_on_args` (NaN / None / wrong type =
  unknown, numeric bounds, inf, eq/ne/in/not_in, bool vs number never equal),
  `test_engine_reads_tool_name_step_kind_and_arm_targets_itself`, `test_arm_joint_missing_from_target_is_unknown`,
  `test_provider_receives_phase_key_args_and_poses`; laziness - `test_only_fields_of_applicable_rules_are_read` (a
  provider for an unreferenced field fails the test), `test_unruled_tool_reads_no_field`,
  `test_a_field_shared_by_rules_is_read_once_per_evaluation`, `test_a_deciding_condition_stops_reading_the_rest_of_the_rule`,
  `test_any_block_stops_at_the_first_true`, `test_nan_from_a_provider_is_unknown`; modes, text and metrics -
  `test_enforce_hit_raises_one_rejection_with_the_documented_text` (exact text and the full-text regex built from
  `GUARD_REFUSAL_PATTERN`), `test_several_enforce_hits_are_joined_and_every_name_is_parsed_by_the_pattern`,
  `test_values_are_rendered` (unknown, true/false, text, rounded numbers), `test_infinite_value_is_rendered_inf`,
  `test_step_and_arm_texts_name_what_was_refused`, `test_check_arm_without_an_agent_origin_is_never_checked`,
  `test_dry_run_hit_logs_and_counts_but_never_raises` (WARNING `guard_rule_would_refuse <json>`, not matched by the
  refusal pattern), `test_dry_run_log_is_valid_json_with_infinite_and_unknown_values`,
  `test_all_applicable_rules_are_evaluated_dry_run_counted_alongside_an_enforce_refusal`,
  `test_unknown_resolved_to_reject_is_counted_with_basis_unknown`,
  `test_pass_counts_no_hit_and_step_and_arm_phases_are_labelled`, `test_eval_seconds_is_observed_per_ruled_check_only`,
  `test_off_rules_are_skipped_and_never_read`, `test_disabled_engine_is_a_no_op`,
  `test_explain_tool_returns_verdicts_without_counters_logs_or_raising`, `test_has_rules_per_phase_and_key`,
  `test_values_in_verdicts_are_rounded_to_three_decimals`, `test_metrics_are_registered_with_their_labels_and_buckets`
  (`mcp_guard_rule_hits_total`, `mcp_guard_eval_seconds` 1e-5..5e-2, `mcp_motion_watch_triggers_total`; counters read
  through `REGISTRY.get_sample_value` throughout); latency - `test_latency_unruled_tool_p99_under_20_us` and
  `test_latency_five_rules_over_fake_providers_p99_under_500_us` (2000 `check_tool` calls each, best p99 of 3 runs against OS preemption). Config
  (`test_config.py`): `test_guard_rules_default_is_enabled_without_rules`, `test_guard_rules_load_from_yaml`,
  `test_invalid_guard_rule_fails_config_load` (never-gated tool, unknown field)
- guard field source and watch sampler (`test_guard_state.py`, fakes only): `GuardState` per registry field - fresh
  value, stale -> None, NaN -> None, collaborator not configured -> None (`test_arm_joint_*`,
  `test_arm_tool_height_is_the_forward_kinematics_z`, `test_start_and_target_tool_height_use_the_context_poses`,
  `test_outside_footprint_*`, `test_footprint_kind_and_carried_radius`, `test_base_speed_fresh_stale_missing`,
  `test_tilt_level_keeps_last_level_when_stale`, `test_tilt_deg_fresh_stale_missing`, `test_battery_*`,
  `test_queue_busy_and_unknown_without_a_queue`), `test_human_takeover_age_from_events_and_infinity_without`
  (injected events, `+inf` without); `WatchSampler` - `test_sampler_gripper_effort_is_absolute_and_finite_only`,
  `test_sampler_joint_effort_abs_nan_and_missing_joint`, `test_sampler_gripper_and_joint_position`,
  `test_sampler_imu_metrics_fresh_and_stale`, `test_sampler_base_speed_battery_and_tool_height`,
  `test_sampler_without_battery_and_unknown_metric`, `test_watch_metrics_constant`. Accessors: `test_monitor.py`
  (`test_tilt_snapshot_*`, `test_last_event_age_s_newest_of_type_and_none_without`,
  `test_odom_speed_is_planar_speed_fresh_within_odom_fresh_s`), `test_footprint.py` (`test_arm_outside_static_m_*`: 0
  for the parked arm and inside, equals the farthest shadow vertex distance, a 5 cm protrusion measures 0.05 within 5 mm)
- deployed guard rules (`test_guard_rules_deployed.py`, reads `ansible/group_vars/client.yml`):
  `test_guard_rules_section_validates_with_the_five_rules_in_order` (GuardRuleSettings, names R1-R5),
  `test_every_deployed_rule_starts_in_dry_run`, `test_every_message_names_a_way_out`, `test_scopes_match_the_design`
  (tools/steps/arm origins, grasp excluded from the roll rule, when_unknown), `test_thresholds_match_the_design`
  (0.03 m, 1.85 / 0.0 rad), `test_motion_queue_watch_settings_validate` (MotionQueueSettings, 10 Hz / 8 per call)
- guard rule wiring (`test_guard_wiring.py`, fake robot through `RobotMCPServer.call_tool`, no base motion): tool entry -
  `test_enforce_rule_refuses_navigate_to_pose_and_the_base_receives_nothing` (rejection text, digest appended, enforce
  counter, refused before the blocking counter), `test_dry_run_rule_lets_navigate_through_and_counts` (dry_run counter
  and `guard_rule_would_refuse` log), `test_tool_without_rules_never_calls_a_provider` (patched `GuardState.get` not
  read for unruled tools, read for the ruled one), `test_take_control_arm_refused_after_human_takeover_base_allowed`
  (injected `human_takeover`; arm lease not taken, base allowed), `test_guard_origin_is_the_tool_name_inside_the_tool_body`;
  queue dispatch - `test_queued_base_relative_refused_stops_the_queue_even_with_on_fail_skip` (`step_failed` status
  `guard_rule` with `data.rules`, `queue_stopped` dropping the next step, never skipped, nothing reaches the base),
  `test_queued_arm_step_runs_with_its_kind_as_guard_origin` (arm phase refusal as `status: refused`); arm motion start -
  `test_move_arm_cartesian_with_overreaching_ik_target_is_refused` (patched IK target shoulder_lift 1.9 / elbow_flex
  -0.1, nothing streamed), `test_same_pose_from_arm_home_and_park_is_not_checked` (`arm_home` and the
  `release_control` park reach that pose with no hit), `test_motion_guard_runs_once_per_motion_before_the_first_setpoint`
  (start and full target pose), `test_motion_guard_checks_every_leg_of_a_blend_and_every_path_sample`,
  `test_motion_guard_rejection_moves_nothing`, `test_roll_guard_runs_before_the_motion_guard`
- unmeasured efforts (`test_effort_nan.py`, review L25: feetech_servos publishes NaN effort for joints without a
  reading): `arm.finite_effort` (finite value, None for NaN / inf / missing); a NaN effort of a non-effort joint is
  absent from `ArmState.efforts` and a NaN gripper effort gives `gripper_effort` None in the arm state, motion results
  and grasp results; grasp verification with a NaN or missing load on the close and now reports `gripper load
  unmeasured` instead of claiming a hold, and a finite load on either still verifies; the hold report gives
  `holding_load` / `crush_risk` None and a NaN holding load never re-tightens; the `gripper_holding` precondition is
  not holding with a NaN or missing load (`gripper load unmeasured`); the live precondition state and the
  `wait_for_event` digest carry None; the carrying signal is not a holding load; `RosRobot.joint_sample` drops
  non-finite efforts
- config additions (`test_config.py`): arm base height 0.100 (floor_z -0.100) and the mount measured 2026-10-10
  `{x 0.0592, y -0.05, z 0.100, yaw 0}` enabled by default, mount z must equal `arm_base_height_m` (null allowed), `floor_guard` and `grasp` defaults and
  validation, `floor_guard.imu_level_reference` defaults to roll/pitch 0 and rejects unknown keys while the deployed
  client.yml carries roll 0.7 / pitch 1.3 deg, `/grasp/command` and `/grasp/result` topic defaults
- robot body model (`test_body_model.py`): exact signed distances of points to a box (face, edge, corner, inside, on
  the surface) and a vertical cylinder (mantle, discs, rim, inside), capsule vs box and capsule vs cylinder against
  hand-computed values (parallel, through the centre, nearest at an end, degenerate segment), vectorised evaluation;
  the default H body (names, lidar, crossbar, side frame, wheel dimensions) and its `describe()`; config validation
  (positive box sizes, positive cylinder radius and height, unique names, hard margin below the soft margin) and the
  client.yml `body:` / `body_check:` blocks equal the defaults
- self-collision checker (`test_self_collision.py`, IK poses FK-checked): the reach envelope computes in batches of at most `ENVELOPE_BATCH_SAMPLES` poses, its traced allocations stay under 64 MB and batching does not change the result; the checked parts (upper arm, forearm, wrist
  link, gripper body hull, jaw points), a clear pose, the 2026-10-10 home pose clear, the gripper pointing into the lidar
  column penetrating `lidar`, low over the front-right wheel penetrating `wheel_fr`, every measured 2026-10-10 floor
  touch-down in front of the robot clear of the hard margin (deployed offsets; the robot reached them without touching
  the lidar) except t4, which the check refuses against the lidar (`test_touch_t4_is_refused_by_the_lidar`: 0.3 mm from the
  dome with the measured dome r 37 mm at (0.150, 0.040), under the 2 mm margin; the model is a few mm conservative there,
  accepted); a trajectory sample moves a link point up to 17.4 mm (8.7 mm per `joint_line` sample) so a path skimming a
  sharp edge between samples can come closer than the margin undetected (accepted, documented in the mcp_server README); a clear pose checks its soft clearance (slow band) and its hard clearance; path checks: into the lidar rejected with obstacle, link,
  clearance, margin and hint, lowering toward the lidar slowed (scale 0.3, clearance in the slow band) not rejected, a
  clear path untouched, escaping from a colliding start allowed while going deeper is rejected, the disabled check,
  `combine_scales` (slower wins), `joint_line` sampling and the part points in base_link; bounding-sphere pruning:
  `test_pruned_path_equals_the_full_computation` (with and without escape) compares every report field (scales,
  per-sample clearances, obstacles, links, verdict, summary, rejection) of the pruned `path` with `path(...,
  prune=False)` exactly over 24 seeded random joint lines, the lines between every pair of scenario poses and single
  poses (covering rejected, slowed and clear), `test_closest_equals_the_pose_clearance` the batched `closest` against
  `pose()`, `test_point_reach_equals_the_unpruned_pose_check` `point_reach` against the per-pitch full check, and
  `test_pruned_path_evaluates_few_point_distances` counts (not times) under a third of the point and capsule distances
  of the full check on the clear-pose-to-home line
- self-collision wiring (`test_self_collision_wiring.py`): move_joints into the lidar refused before anything moves
  (details, rejection metric), a move near the lidar slowed and reported (`self_collision`, slowdown metric, min
  clearance gauge, more setpoints than unchecked), floor and body reports together, move_cartesian refused, disabled
  check moves, set_gripper checked; home via a safe intermediate when the straight path hits the right side frame,
  direct from a clear pose, and never rejected from any valid start of a grid around the robot; the motion queue
  refusing an arm step into the body at enqueue, checking a blend from the queued pose, and re-checking at execution
  start (`self_collision` outcome, ok false, details); the grasp planner rejecting candidates with
  `body_collision:lidar` (also a top_down retreat into the lidar) and not without the check, the executor refusing a
  step of an unchecked plan; `get_arm_reach` (obstacles, mount, margins, envelope grid and cells, points: reachable and
  clear, into the lidar with the limiting obstacle, out of reach with a suggestion; more than 20 points refused) and
  the envelope computed once and cached; with a cache file it starts in the background when the server is built,
  is written to disk and loaded by a new envelope without recomputing, and a changed hard margin, a changed arm link
  model (collision boxes) or a corrupt file recomputes; the blend velocity cap and home-start grid stay clear of the
  measured lidar dome
- reach map (`test_reach_map.py`, coarse fixture: the default r / z cells every 30 deg of heading, 48 x 48 x 26
  planar samples): `ReachMapSettings` defaults (5 deg, 0.02 / 0.48 m, 0.025 / 0.45 m, 72 x 72 x 37, 2 per cell, package
  `data/reach_map.npz`) and validation; `ReachEnvelope.cache_key` unchanged by extracting `reach_inputs` (compared with
  the old formula for the default and the deployed config); the key changes with the body, mount, joint limits, tool
  point, hard margin, every grid field and `REACH_MAP_VERSION` but not with the file path; the pan axis: FK at pan
  theta equals the pan 0 FK rotated about the pan joint axis (1e-6 m), the axis is vertical and the vertical rotation
  the map uses is within 5 um, a tilted pan or mount is refused; build: shapes, dtypes, grid and meta (key, version,
  counts consistent with the mask, clearance >= the hard margin where reachable and NaN elsewhere), deterministic,
  traced allocations under 64 MiB, body checks in batches of at most `ENVELOPE_BATCH_SAMPLES`, the clearance pass gives
  exactly `clear_of`'s verdict, doubling `planar_samples` changes the reachable cell-pitch count under 3 %, and
  `point_reach` confirms at least 18 of 20 sampled reachable cells and refutes at least 18 of 20 unreachable ones;
  save / load round trip (atomic, no temporary file left), missing and unreadable files raise `ReachMapMissing`, another
  grid raises `ReachMapStale` with the expected and found keys; `cells`: margins on a synthetic mask (neighbour cells,
  the ring beyond `r_max_m`, the heading rows beyond the pan range), pitch filter (nearest bin within 7.5 deg, others
  refused), heights outside the grid give nothing; `describe`; the CLI builds and checks (exit 0), and exits 1 for a
  stale or missing file; `config_from_client_yml` reads the mcp_server node
  Key stability: `test_reach_map_key_ignores_cross_platform_float_noise` - floats in the key are rounded (`canonical_floats`, 9 decimals), so last-digit differences between the Mac (map build) and the Pi (map load) do not make the committed map stale.
- approach planner (`test_approach.py`, synthetic reach cells and 2.5 cm grids, no ROS): `ApproachSettings` defaults
  (10 deg, 30 cells, margins and pose error, weights 1.0 / 0.5 / 0.2 / 0.1, 0.30 m staging, 0.03 m threshold, 5
  corridor samples, top 5 / 3, topics and actions, tracker 5 Hz / 0.6 / 48 / 96 px / 0.10 m / 20 px) and validation
  (zero steps, negative weights, a search window not larger than the template, relative topics, unknown keys); target
  forms (exactly one of xyz / object_id / poi_id / pixel, `z_m` required for objects and POIs and refused for the
  others) and `resolve_target` for each form, unknown objects / POIs / missed pixels refused; candidate yaws cover the
  circle once in [-pi, pi), a 350 deg yaw is the -10 deg pose; inversion round trip (the target in each base pose's
  base_link is the cell's heading and r from the pan axis, and B + R(yaw) a + r(cos, sin) is T); the expected margin
  formula, below-margin cells dropped and `cells_per_yaw` kept, no cell counts `out_of_reach`; the footprint raster
  area within 5 % of the polygon (robot-like concave outline and a rectangle, every 10 deg, 1 and 2.5 cm) and it turns
  with the yaw; a missing or stale grid raises `ApproachUnavailable` (also through `suggest`); a free grid accepts;
  lethal and unknown footprint cells, an inscribed centre (an inscribed edge cell is fine) and footprints off the grid
  are rejected with their counters; staging (0.30 m back along the yaw) and corridor obstacles reject, corridor
  samples 0 skips the corridor; mean / staging / corridor costs reported; the score terms (margin cap, no travel
  without a robot pose); ranking follows the margin, heading, travel and cost weights; `verify_top_k` checks only k
  with the target in base_link (pitch in rad or None), drops failures and keeps the rest unverified; `path_check`
  plans the top k staging poses, drops `no_path`, re-scores `ok` with the path length plus the staging leg, keeps
  `unavailable`, leaves the rest `skipped`; `suggest` end to end, a target inside an obstacle (all rejected as
  collision), out of reach (note, no pose) and without hooks or robot pose (notes, unavailable paths)
- approach MCP tools (`nodes/mcp_server/tests/test_approach_tools.py`, `FakeRobot` with a stub reach map, a free 5 cm costmap, a fake `point_reach` and a fake footprint, no ROS): tool classification (`approach_for_grasp` is a base-lease motion tool and queue exclusive, `suggest_approach_pose` an always-allowed sensor with no lease); suggest ranks candidates, verifies the top 5 with `point_reach`, plans the top 3 paths (planning only, nothing sent), reports the reach map key, costmap age and docking readiness (also server down), `check_path=false` plans nothing, works without a lease and without a robot pose (paths `unavailable`); a missing or stale costmap is refused; a stale or missing reach map is logged as an error at registration and both tools refuse with the expected key and the regenerate command; target forms (object, POI area centroid, pixel through the calibrated front camera, missing `z_m`, two forms, partial point, unknown id); `approach_for_grasp` refused without the base lease (an arm lease is not enough), below battery cut-off (also as a dry run), at tilt warning and error, while another base motion runs, without a robot pose and with no valid candidate (rejected counts named); a given pose is re-validated and a pose that is not a candidate is refused; dry run (navigate mode and servoed mode) sends no dock goal, no navigate, no detection and no stop, returns the would-send goal, readiness and, in servoed mode, one tracker step whose detection pose equals the plan within 2 cm, or the reason there is none; mode selection (uncalibrated cameras named in `mode_reason`, servoed only when visible from both the staging and the dock pose with the staging pose 0.30 m farther, `prefer_mode` navigate and a refused servoed); a navigate run sends one precise map goal at the dock pose and re-checks `point_reach` at the measured pose; `arrived_out_of_reach` when the re-check fails; a failed navigation is not re-checked; a servoed run sends a `grasp_servoed` goal with the staging time and timeout, runs the detection loop only while the dock runs and stops it after, stop during the dock ends `canceled` and stops the loop, a raising dock still stops the loop, a failed dock keeps its error code and retries and suggests `prefer_mode='navigate'`, an interrupted dock passes the event through. `test_tools.py`, `test_battery_gate.py`, `test_tilt_guard.py` and `test_control.py` list both tools in the tool set, the motion / always-allowed partition, the tilt nav tools and the lease table
- arm link boxes (`test_arm_link_boxes.py`): the body check's link boxes are the six collision boxes of the grasp_sim
  `so101.xml` (upper arm, forearm, wrist; read from the MJCF independently), the upper arm boxes contact-only; the face
  points cover every face within spacing / sqrt(2) and that bound stays within the default hard margin (2 mm, `BOX_POINT_SPACING_M` 2.5 mm); `clear_of` (envelope's pruned check on `SelfCollisionChecker.geometry`) equals the hard verdict of the full clearances over random poses; over 300 random poses the checker's clearance never exceeds the
  sim box surface clearance (plus the hard margin for contact-only boxes) by more than that bound; the 2026-10-10
  pose whose sim upper arm box edge comes within the hard margin of the lidar dome is rejected, home stays clear, and a
  pose with the upper arm box 23 mm from the dome edge is not slowed by the upper arm boxes
- floor slow zone (`test_floor_guard.py`, a synthetic 15 cm mount with its own floor, independent of the configured
  one): arm <-> base_link conversions (with yaw), quaternion roll/pitch, effective
  surface (robot plane vs level plane, roll/pitch signs, shifted by `surface_z_m`), fresh IMU used / stale or missing
  IMU ignored and logged once, tilt override replaces the IMU, the IMU tilt is taken relative to
  `imu_level_reference` (the reference reading gives no lift, `tilt_deg` 0 with `imu_tilt_deg` / `imu_level_reference_deg`
  in the description, a real 3 deg slope beyond it still lifts, default 0/0 keeps the raw tilt), a tilt override is not
  shifted by the reference, flat robot slows only samples inside the margin, IMU
  pitch raises the zone in front, `surface_z_m` allows normal speed down to a stair, configured surface/margin/scale,
  disabled guard, elbow and wrist checked, jaw model (gap grows with opening, inverse, closed tip at the tool point;
  `inner_gap` equals the tip gap 3 mm in, narrows deeper into the jaws (sim jar contact 46.5 mm 2.75 cm in at 0.534 rad),
  inf once the open tips retract past the depth, `angle_for_inner_gap` inverts it and opens wider than the tip model;
  `rotate` on many vectors at once equals rotating each, as `inner_gap` uses it),
  monitor IMU record to tilt sample, per-step scales and constant-rate retiming of slow steps; with `surfaces`: the
  local region height (normal speed over a stair, slow below the robot floor before its edge, flat model unchanged),
  slow near a step face (`lowest_feature`, `surfaces` in the summary), surfaces combined with a tilt override and
  `surface_z_m`, surfaces only described when given
- surface regions (`test_surfaces.py`, pure): exactly one shape and a convex polygon (either winding), height lookup
  with the default outside, later regions win (a hole in a table), arm-frame regions converted with a yawed mount,
  vertical clearance and its feature, step faces as vertical walls (beside, above the corner, near), polygon faces
  and no face between equal heights, lift (tilt) term, capsule clearance, steps crossed along a line, vectorized
  `clearance_many` equal to the single queries, JSON description
- grasp planner with surfaces (`test_grasp_surfaces.py`, deployed client.yml, the matrix stair 10 cm down with its
  edge 8 cm before the box, boxes 4 cm right of the arm centre line so the upper arm stays off the measured lidar
  dome; the wrist-into-the-step case at arm x 0.32): the single-surface angled 50 deg plan runs the wrist into the step (45 deg until the jaw
  clearance put it out of reach there), with surfaces that candidate is rejected naming the step edge, a steeper candidate clearing the edge is chosen (rejected tries
  in `rejected_candidates`), the pre-grasp and the lift clear the upper surface, auto plans clear every surface, the
  gripper body keeps the jaw clearance from a diagonal step edge (r25 at 30 deg), `support_z` must match the region
  under the object (a gap is allowed for), object surfaces merged into the call override, forearm / wrist / gripper
  body clearances
- arm slow zone and paths (`test_arm.py`): motions into the slow zone take more 25 Hz ticks at <= 0.2 of the velocity
  cap and report `slow_zone`, motions above it are unchanged, `surface_z_m` and tilt overrides, stale IMU ignored,
  `move_cartesian` / `set_gripper` / `home` pass the override, `move_path` streams through samples (velocity cap,
  validation, roll guard, slow zone)
- arm settle policy (`test_arm_settle.py`): `trajectory_end` returns right after the streamed trajectory with `settling`
  and `tracking_error_rad` (no convergence wait), keeps the goal commanded and releases the motion lock; a following
  move starts the joints it names from the measured state while other joints keep their intent; a stalled joint is
  relaxed after the hold window; stale-feedback and tracking aborts still fire; `final` waits (timeout) and splits
  `trajectory_s` / `settle_s`; a stuck gripper no longer times an arm move out, a gripper-only move still judges the
  jaw; per-joint converge/settle tolerance defaults (loaded shoulder_lift / elbow_flex wider) and their validation
- arm home under the follower lag (`test_arm_home_tracking.py`): a fake follower (`fakes.DelayedFollower`) reaching
  each command after the measured clean servo lag (0.22 s) runs the logged full-speed home from the logged start pose
  and converges with the default `arm_tracking_delay_s` 0.35; a follower stalled 1.2 s still aborts (the check compares
  against the setpoint of 0.35 s ago); a jammed arm still aborts a full-speed home at the speed-term limit
  (base + lag x vmax); `tracking_limit` grows with the speed; `arm_tracking_lag_s` is 0 and a jammed joint at vmax
  1 rad/s aborts with the newest setpoint at most error + delay x vmax (+ two ticks) ahead (about 0.6 rad, not 0.85);
  a follower 0.4 s behind (just past the delay window) still converges
- stream timing (`test_arm_stream_timing.py`): on a fake clock with jittery 40-60 ms ticks (late wake-ups) a full-speed
  move finishes at its planned `trajectory_s` (+- one period) with every setpoint equal to the analytic quintic at its
  publish time; a tick overrunning by 2.5 periods publishes the plan at the real time in one larger step with no burst
  of setpoints after it; with the body soft margin the jittery move still ends at the stretched planned time;
  `plan_sample` interpolates the plan at a fractional index (clamped at both ends); `setpoint_band` spans the setpoints
  commanded since a time (the one current then included, before the history, empty history)
- gravity sag model (`test_sag.py`): URDF gravity torques of a synthetic two-joint chain (horizontal point masses,
  hanging link, cosine law, missing joints, unknown joint rejected) and of the SO-101 (no torque on the vertical pan
  axis, stretched arm loads the shoulder most, shoulder > elbow > wrist, plausible total mass); deflection k * tau with
  saturation; compensation against gravity inside the limit band (never pushes a target already inside the margin
  further out, per-joint margins); the compensator in URDF space with zero offsets and explicit gains; lifting /
  lowering / hold gains and the approach mode from the final approach (reversal counts); gain blending; least-squares
  gain fit (exact on noise-free data, no negative gain, joints without samples), RMS before/after, leave-one-group-out
  validation with sample counts, pooled reports; `arm_settle` journal line parsing and settled records split by mode
- arm sag compensation (`test_arm_sag.py`): disabled config (even with gains) publishes exactly what the default
  controller publishes; published targets are target - predicted deflection for the loaded joints only (pan, roll,
  gripper unchanged) with the uncompensated target in the result; `max_rad` saturation; commands stay in the limit band
  and are not pushed below the trajectory; joint offsets applied in URDF space; no step when the compensation ramps in;
  lifting vs lowering gains; gripper-only motions keep the arm compensation; a compensated move under model-matching
  sag converges on the uncompensated target with `reached target` (and the same sag without compensation settles
  short); keepalive republishes the compensated target; the relax after `arm_settle_hold_s` holds measured plus
  compensation; a hold at a measured pose uses the mid-band gain; tracking judged against the uncompensated setpoint;
  `home`, blended and Cartesian moves compensated; the slow zone checks the over-command too; `arm_settle` records for
  `final` moves, for `trajectory_end` moves once settled (probe after 1 s, logged once), none for gripper-only moves;
  config validation (unknown joints, negative gains, `max_rad` cap)
- smoothness round A (`test_tools.py`, `test_early_return.py`, `test_config.py`): `get_camera_image` defaults to 384 px
  (larger `max_px` still honoured); `settle` on the arm tools (default `trajectory_end`, `final` when the gripper joint
  moves, configurable); `precise` forwarded by `navigate_to_pose` / `move_relative`; descriptions ask for checkpoints
  instead of a check after every motion; one `mcp_server.timing` log line per call (tool, start, end, duration, ok,
  and trajectory/settle seconds, time to the first setpoint and the achieved tick rate for arm moves, also for
  failing calls); `run_nav` ends the goal early within the
  intermediate tolerance (xy and wrapped yaw, same frame only), precise goals run to the Nav2 result; nav
  intermediate tolerance defaults and validation
- path trajectory (`test_trajectory.py`): `path_trajectory` stays on the joint-space polyline, ends at its last point
  and respects the velocity cap; zero-length path
- blended trajectory (`test_trajectory.py`): `blend_trajectory` passes the vias without stopping (non-zero, continuous
  velocity at each via), comes to rest at a via where a joint turns (no overshoot), keeps every sampled velocity and
  acceleration within the caps, starts and ends at rest, reports via indices and segment durations, rejects bad
  arguments
- blended arm motion (`test_arm_blend.py`): `ArmController.move_blend` streams one motion through every target (no rest
  between them, `on_via` per target) and is faster than separate stop-and-go moves, respects the velocity cap, keeps
  unnamed joints across vias, enforces the roll guard on every leg (also against a gripper target of an earlier via;
  nothing moves), slows in the floor slow zone per streamed step, aborts on stop; `motion_running`; `solve_cartesian`
  matches `move_cartesian` and raises for unreachable targets
- motion queue (`test_motion_queue.py`, real worker thread on fakes): step models (kinds, defaults, unknown fields,
  gripper exactly-one, wait bounds, `surfaces` on arm / gripper steps carried into the floor override), blend groups split by settle 'final', gripper steps, preconditions, speed and
  gripper-joint targets; enqueue returns at once and runs every step in order (`step_started` / `step_done` /
  `queue_empty`); consecutive arm steps run as one `move_blend`, cartesian steps solved at enqueue and blended;
  infeasible or invalid steps and too many steps refuse the whole call (nothing moves); precondition failure stops the
  queue (default) or skips the step, and a skipped `precondition_failed` / `step_failed` (data `on_fail: skip`) does
  not end a `queue_empty` wait while later steps still run (gated regression tests); every precondition predicate incl.
  unknown state; a tracking abort is a
  `step_failed` that stops the queue; gripper contact event; `halt_for_stop` clears the queue at once; an external stop
  between steps halts it; `cancel` aborts the running step (robot stopped) and drops the rest, but never stops an idle
  robot; `replace` drops pending steps only; battery cut-off at dispatch; enqueue refused while a blocking tool or
  another arm motion runs; grasp steps through the runner (grasped = contact, missed and unverified = failure without a contact event,
  an unverified grasp halts the queue even with `on_fail: skip`,
  refused without a runner); waits (idle at once, `step_done`, timeout, cursor), status with progress; planning phase (status `phase`
  `planning` with `planning_s` until the first setpoint, then `executing`; `planned` event with `planning_s` between
  `step_started` and `step_done`, never waking a wait) and blended-group progress that follows published trajectory
  time, not only vias passed
- motion queue watch constraints (`test_motion_watch.py`, real worker on `GateRobot`, fake sampler, injected clock;
  the watcher thread parked by an injected sleep so the tests drive `watch_tick()`): `WatchConstraint` defaults and the
  validation matrix (joint required / arm joint only / not on joint-less metrics, finite value, ops, for_s 0..5, name
  length, unknown keys), `watch_triggered` / `paused` in `FAILURE_EVENTS` and every wait condition, more than
  `watch_max_per_call` refused; `test_trip_only_after_for_s` (debounce restarts when the value recovers),
  `test_abort_drops_pending_and_stops_once` (event data, `cancelled` drops the rest, `step_aborted`, one `robot.stop`,
  counter, monitor report), `test_pause_keeps_pending_and_resume_continues` (no dispatch while paused, `resumed`, the
  pending step runs), `test_resume_refused_while_a_watch_of_the_pending_steps_violates` (and in battery cut-off),
  `test_resume_when_not_paused_is_a_no_op`, `test_stop_and_cancel_clear_the_pause`,
  `test_pause_then_enqueue_appends_and_waits_for_resume` (replace keeps the pause),
  `test_distance_travelled_integrates_fresh_base_speed_since_activation` (stale gap not integrated),
  `test_unknown_value_handling` (ignore vs trigger), `test_unknown_ignore_keeps_the_debounce_running`,
  `test_trip_racing_the_step_end_is_dropped` (the step ends between sample and trip: no event, no stop),
  `test_watch_is_bound_to_the_job_ids_of_its_call` (status view fields, no cache read while inactive),
  `test_replace_drops_orphaned_watch_sets`, `test_finished_sets_are_dropped_and_status_reports_the_active_one`;
  thread: `test_watcher_thread_wakes_wait_for_event` (reason `watch_triggered`),
  `test_watcher_thread_sleeps_while_no_set_is_active`, `test_watcher_at_20hz_keeps_the_25hz_arm_stream_period`
  (deadline-scheduled 25 Hz loop beside the watcher at 20 Hz; p95 thread CPU time per watcher tick, the most it can
  add to a stream period, under 1 ms - load-independent, unlike a wall-clock A/B of the stream). Also
  `test_monitor.py::test_report_watch_trigger_emits_a_warning_event` (warning, not debounced, not an interrupt) and
  `test_config.py::test_motion_queue_watch_settings_defaults_and_bounds`
- motion queue tools (`test_motion_tools.py`): registration and classification (`enqueue_motions` motion,
  status/cancel/wait always allowed, `QUEUE_EXCLUSIVE_TOOLS`); `enqueue_motions` returns job ids and blend groups and
  `wait_for_event` returns events, status and the state digest (plus the robot event digest); blocking motion tools are
  refused while the queue runs and allowed again afterwards; `enqueue_motions` refused while a blocking motion tool
  runs; `stop` clears the queue and reports `motion_queue_dropped`; `cancel_motions`; infeasible steps are tool errors;
  `replace`; `until='failure'` returns `idle` at once; the process shutdown hook (`stop_queued_motion`) clears a busy
  queue and stops the robot, and leaves an idle robot alone; `watch_constraints` validation errors (bad metric / op /
  missing joint / `for_s` / action / non-finite value, too many per call) queue nothing, a watched enqueue shows in
  `get_motion_status.watches`; `resume_motions` no-op when not paused, refused while the watch still violates, resumes
  once clear; the descriptions of `enqueue_motions`, `resume_motions`, `wait_for_event` and `get_motion_status` mention
  the watch fields, pause and resume
- guard rule tool (`test_guard_tools.py`): `check_guard_rules` is always allowed; reject / pass / unknown verdicts carry
  the observed values and message; off rules are `skip`, unruled tools give `[]`; no hit counter changes and no log
  line; the description says arm-target rules are evaluated at motion start
- spin (`test_base_motion.py`): `run_spin` turns the measured angle with marks in order at the right rotation and ends
  with a zero twist, clamps the yaw rate, ends on stop, a critical event, a refused mark, a lost pose or the timeout,
  rejects a zero angle or rate
- look_around spin mode (`test_look_around.py`, `test_perception_tools.py`): spin mode captures the same tiles, labels
  and heading summaries as steps mode with one spin and no Nav2 rotation; steps mode without return skips the closing
  rotation; return_to_start spins the full circle (and corrects a final heading error with one precise rotation);
  an obstacle at a heading ends the spin; an interruption never triggers a corrective rotation; the tool defaults to
  one spin without returning
- link frames (`test_ik.py`): `link_frames` returns every chain link from one FK pass
- grasp planner (`test_grasp.py`, real URDF; floor objects clear of the measured body: top_down at arm x 0.26, angled
  at 0.36 for 45 deg and 0.32 for 60 deg): strategy registry and unknown names, scoop into a tight gap skims the
  surface (roll about 0, moving jaw up, horizontal slide, opening), scoop with room under a ledge object goes below the
  object bottom, scoop needs `gap_below_m` >= jaw thickness + margin ("no gap under object for the fixed jaw", also in
  auto's reasons), top_down (pitch 90 deg, jaws aligned with the object width for two yaws), angled (requested pitch on
  every waypoint, jaws across the width), auto order (top_down first) and fallback, tall narrow objects gripped at
  `tall_grasp_height_fraction` and lifted at `lift_speed_scale` (and not below `tall_ratio`); IMU level reference and
  minimum jaw overlap: the 25 x 30 x 20 mm leg at arm (0.234, 0.124) top_down with IMU tilt 0.72 / 1.21 deg and the
  same reference puts the tool point at least 8 mm below its top (3.7 mm before) without warnings, without a tilt it
  stays at the floor clamp z -0.089, a real slope beyond the reference still lifts the grasp surface, the overlap rule
  on the floor (30 and 19 mm full overlap, 16 and 15 mm feasible with a `shallow_overlap` warning also in the summary,
  14.5 mm infeasible with a "jaw overlap" reason) and a 12 mm object on a raised support gripped 8 mm below its top
  instead of at mid-height; objects below
  `min_object_width_m` rejected, unreachable and too-wide reasons, roll only at the lifted half-open
  pre-grasp (`roll_guard_violations`), straight-line IK samples (spacing, line, joint jumps), joint-jump and shoulder
  stall reasons, slow-zone annotations, base_link objects and a missing mount, params overrides, JSON summary;
  jaw clearance: a 40 mm cube top_down / angled 45 / angled 60 puts the fixed jaw inner face 7.5 mm off its side face
  and the moving jaw 7.5 mm clear over the cube's depth in the jaws (opening 55 mm, `center_shift_m` and `grasp_shift`
  shift 27.5 mm, the reported clearances and `jaw_depth_m`, tool_point and jaw centre consistent, tips opened wider),
  `fixed_jaw_clearance_m` above half the margin widens both sides, a zero minimum still centres the margin, scoop has
  no shift, the 39 mm x 6 cm jar opens far wider than the tip model so the tilted moving jaw clears its top edge, an
  object too deep for the gripper is a reason, scoops open for the object depth too
- grasp offset (`test_ik.py`): `grasp_offset` scales the open axis by half the width plus the fixed jaw clearance
- IK heading convergence (`test_ik.py`): a pitched IK with the tool offset converges the arm plane heading in at most
  12 ikpy runs (secant step; the fixed-point iteration needed 16 or more) and a `heading_bias` start lands within
  2e-3 rad of the cold solution
- grasp planner speed and stability (`test_grasp_speed.py`, deployed client.yml config, via `plan_matrix.py`): the 6
  feasible sim-matrix scenarios in `tests/data/grasp_golden.json` (with the measured body: 4x4x4_r25_floor_auto from
  the 2026-10-09 set, the others plan again with the measured dome (r 37 mm), plus top_down, angled 45, auto and two scoop
  cases of the current matrix: floor -0.10, ledges -0.03 / 0.05; the angled reach-edge and IK budget cases at arm x
  0.34), earlier regenerated 2026-10-10 with the grid-sheet client.yml
  calibration (joint offsets, tool point = physical fixed-jaw tip; fixed arm-frame surfaces: floor -0.15, ledges -0.08 / 0.0,
  stair -0.200 = 10 cm below the configured floor -0.100; the old -0.25 stair became unreachable; regenerated again for the
  2026-10-10 jaw clearance with `python -m tests.regenerate_grasp_golden`) (top_down, angled, scoop, auto; waypoint joints and
  straight-line samples) match the current golden plans within 1e-6 rad and the `baseline` plans (before the faster
  heading convergence) within 2e-3 rad, with the same feasibility and strategy; timing tests (skipped with
  `GRASP_SPEED_SKIP=1`): feasible angled and auto golden scenarios and auto on the reference cube under 1 s, scoop 3 s,
  infeasible angled/auto fail fast with reasons, unreachable object rejected in 0.2 s without an IK search, re-planning
  served from the IK cache; the per-candidate and per-strategy IK budgets stop the search with an "IK budget" reason
- grasp execution (`test_grasp_tools.py`): tool names and battery classification, grasp with a simulated object
  (close on load, lift, retreat, never a full squeeze), miss (a contact failing verification or a blocked close: open
  and retreat), the 2026-10-11 plush croc: a close into the empty band (`closed_no_contact`, jaw stalled 0.05 rad short
  of closed at load 140, or fully closed on nothing) never opens the gripper: step `hold` keeps the jaw at the stall
  plus the profile squeeze (never past closed) through lift and retreat with the profile torque limit (gentle 250),
  outcome `unverified` with reasons naming the empty band, soft/squashed or absent and `release_object`;
  `hold_empty_close` refuses without arm control; the outcome schema lists `unverified`; `grasp_object` attaches one
  gripper camera image (`camera_image("gripper", 640)`) to an unverified result (a camera failure becomes a warning, a
  grasped result takes no picture); the grasp_object / release_object descriptions explain `unverified` (image,
  release_object, plush with grip_profile soft, retightened) and the output schema lists it; `GraspService` returns `unverified`
  with the hold step; the queue / web UI reason asks for `get_camera_image 'gripper'` (no "attached"), the
  `grasp_object` reason points at the attached image, the outcome description mentions both; an unverified hold
  keeps the soft torque until `unverified_hold_relax_s`, then the default is written once (still held); stop between
  steps and lost lease
  abort, infeasible plans do not move, half-open before the roll at the lifted pre-grasp, release (open fraction and
  lift), every straight-line step at its waypoint speed (slow lift of a tall object), the `plan_grasp` /
  `grasp_object` / `release_object` tools (dry run, overrides, bad params, schemas teach `gap_below_m`, the scoop gap
  rule and the tall-object params), `GraspService` JSON contract (plan, execute, release, stop, errors) and battery
  cut-off refusal; `surfaces`: `floor_override` carries them, `plan_grasp` reports them and the support_z mismatch,
  `GraspService` takes call and object surfaces and rejects an invalid region, the tool schemas explain step edges
  and half-planes; the pre-grasp and roll moves run at `approach_speed_scale` (default 0.3, also the planned
  pre_grasp / open speed), per call via `params`, capped to `limits.arm_max_speed_scale` and refused above the hard
  maximum, and the plan_grasp / grasp_object schemas offer it
- web-UI arm jog (`test_jog.py`): `jog` config defaults (0.01 m, 0.05 rad, 0.1 open fraction, unknown keys
  rejected); `JogSpec` accepts only the directions of its kind and a positive step; `jog` spec required; cartesian
  `up` moves the tool point one capped step along the gripper-frame up axis keeping the pitch, `forward`/`back`/`left`
  follow the gripper frame (approach axis, horizontal left); joint steps (shoulder_pan left turns the heading left,
  capped to 0.05 rad, other joints untouched; wrist_pitch up lowers the pitch; wrist_roll left/right); gripper steps
  are open-fraction steps (capped to 0.1, refused when fully closed); refusals with nothing published: no fresh joint
  states, battery cut-off, IK unreachable, self-collision (cartesian and joint), a descending step below the floor
  clearance (a rising one passes), a joint at its limit, wrist roll with the gripper open past the roll limit, a
  running grasp action; coalescing: while a jog runs a newer one replaces the pending one (the replaced one is answered
  superseded), the last request runs next and every request gets exactly one answer; `stop` drops the pending jog;
  `handle_json` answers a plain jog at once; refused while the leader (a fresh non-autonomy active source) commands the
  arm and mcp_server holds no lease, allowed once the lease is held or the source is `none`; refused while the agent
  holds the arm with take_control (wired by `build_mcp_server`), allowed after release_control
- grasp hold regressions of the 2026-10-10 grip tuning session (`test_grasp_hold.py`, fake follower that sags by the
  gravity model with sag compensation on): the full grasp_object sequence keeps shoulder_pan commanded at the planned
  grasp pose through the close and its hold check, within the planned samples through lift and retreat and at the
  retreat target after the settle hold time (every joint within its converge tolerance, no residual_error, no
  warnings); a pan pushed 0.06 rad off target while squeezing stays commanded at its intent and is reported in the
  close step's and the result's residual_error and as close / retreat warnings; a centred grasp reports `grasp_shift`
  (shift = half the width plus `fixed_jaw_clearance_m`), each waypoint's `tool_point` that far beside the jaw centre, and `held_pose` (jaw centre from the measured
  joints, expected = retreat waypoint, error under 1 cm); the empty firm close stalling 0.055 rad short of closed at
  load 148 is `closed_no_contact` and the grasp `unverified` (no open after the close)
- grip profiles config and resolution (`test_grip.py`): gentle / normal / firm presets with the 2026-10-10 tuned values
  (gentle 0.01 / 250 / 0.2 / target_load 80 / contact 120 / crush 200, normal the pre-profile squeeze, speed and
  contact threshold plus target_load 200, firm 0.05 / 650 / 0.4 / 300 / 350 / 600; target_load < contact < crush) plus
  soft (plush, foam, fabric): squeeze at the 0.08 cap, torque_limit 600 above normal and within the 800 cap, target_load
  < contact < crush < torque_limit, validates as a preset and is rejected when the squeeze cap is lowered below it;
  `retighten_load_below` defaults to 30, null disables it, 0 is refused; default must be a preset, presets above the torque or squeeze cap rejected,
  torque cap within the register range, preset / inline resolution (`<base>+custom`), inline values clamped to the
  hard caps and listed in `capped`, invalid overrides refused, the report fields, and the client.yml block validates
  and equals the built-in defaults
- grip profiles in the arm controller (`test_grip_arm.py`): the profile torque_limit is written before the jaw closes
  and kept while holding, close speed and stall squeeze come from the profile (plain close: the default squeeze),
  target_load stops the close on the closing-direction load only (configurable sign), an explicit effort threshold
  overrides the profile's, the default torque is restored after a close without contact, an aborted or raising close,
  after an open (after the jaw moved), on release / a lost lease and at startup (no write when already default),
  inline profiles capped before the write, unknown profiles and grip_profile without close_until_effort refused before
  anything moves, holding_load / slipping / crush_risk reporting and the torque_limit read-back (verified, unverified,
  mismatch); stall-path tests lower `gripper_empty_stall_rad` so their thin-object stall (0.075 rad before closed)
  counts as an object. Stall classification by position (default config): an object stalling the jaw at 0.29 rad with
  load 150 (below the normal target_load) is `grasped` with the hold at 0.29 minus the normal squeeze and the profile
  torque kept, a stall at -0.04 rad (beyond the settle tolerance) is `grasped`, a stall at -0.115 rad with load 140
  (inside the empty-jaw band) stays `closed_no_contact`, a stall after too little travel is `blocked` holding the
  measured jaw; `jaw_stalled` accepts a command already at the close target as the lead (a jaw short of closed by less
  than `gripper_stall_lead_rad`) but not one still ramping, and `stall_contact_message` is None inside the empty band.
  Hold re-tighten (2026-10-11 12:18 plush croc head: normal held at load 204, the hold read 0 and it slipped out): a
  stall hold whose holding load drops to 0 is re-tightened exactly once to `squeeze_max_rad` past the stall (normal:
  from 0.03 to 0.08), the torque limit is never raised, `retightened` true and holding_load measured again; a
  target_load contact (holds the measured jaw) relaxing to 0 is re-tightened by the full cap; the re-tighten stops at
  closed; soft (already at the cap) is not re-tightened; a healthy holding load (150) is not re-tightened; a jaw that
  sank into the empty band during the hold is not re-tightened; `retighten_load_below: null` disables it; an
  empty-band close stays `closed_no_contact` with `retightened` null; a gentle hold is never re-tightened (gentle has
  `retighten: false`, normal / firm / soft true) and an inline override can opt out; a re-tightened hold whose new
  measurement shows crush_risk goes back to the previous hold (`retighten_reverted` true). The shared `stalling` fake reports a healthy
  hold load (100) during the hold check so the other tests keep their single hold
- gripper-only motions near the floor and the body (`test_gripper_close_speed.py`, deployed client.yml config): the
  2026-10-11 10:27 close (jaw tip 12.5 mm above the floor, the stationary upper arm 14.9 mm from the lidar) plans at
  most 1.2x the unscaled `plan_trajectory` ticks (64) and commands >= 0.05 rad of closing within 25 ticks, with no
  `slow_zone` / `self_collision`; an arm motion from the same pose is still slowed. Floor guard `gripper_only`: a jaw
  swing above `GRIPPER_ONLY_SLOW_MARGIN_M` is not slowed (the arm-motion margin would slow it), with the floor raised
  to 2 mm under the lowest jaw sample exactly the samples where the jaw descends below the margin are slowed, and
  stationary points below the surface never slow it. Body check `moving_parts`: stationary links in the soft band do
  not slow a jaw swing, the moving jaw in the soft band does (reported as the part), the hard reject still applies,
  non-point parts are refused. Timing: a result reports `time_to_first_setpoint_s` (planning time), `ticks` and
  `tick_rate_hz` = ticks / trajectory_s; queued motion events carry both timing fields
- grip profiles through the interfaces (`test_grip_tools.py`): `grasp_params` takes grip_profile, a grasp applies and
  reports the profile (default normal), a grasp aborted after the close, a miss and a release restore the default
  torque, an `unverified` grasp (empty-band close) keeps the profile torque (firm, within the cap), the `set_gripper` / `plan_grasp` / `grasp_object` tools take grip_profile (capped, unknown refused, a dry
  run writes nothing), the `GraspService` contract carries it in and reports it out, and the motion queue gripper step
  takes it (refused without close_until_effort) and reports the grip fields in `step_done`; the `set_gripper`,
  `grasp_object`, `plan_grasp` and `enqueue_motions` descriptions name the soft profile, set_gripper and grasp_object
  send plush to it and explain `retightened`, and the grip profile listing shows soft's values and use
- sim scenario planning (`test_plan_matrix.py`): `sim/grasp_sim/scripts/plan_matrix.py` writes the matrix index and
  plans with the client.yml config (tool offset, `--only` filter, `--params` overrides); supports are placed relative
  to the configured floor (-0.100): ledges 7 and 15 cm above it, the stair 10 cm below; the stair is passed as a
  step surface (half-plane at `support_edge_x` = box x - 0.08, the sim scene's edge), its angled plan (3x3x6 at 30 cm:
  the 4x4x4 one leans the upper arm onto the lidar dome) is steeper than 45 deg, ledges keep a single surface height; the index records the shoulder pan axis (`pan_axis_xy`) the sim box
  faces; `--tipover` writes the 2026-10-10 jar set (39 mm x 6 cm 30 g cylinder at base_link (0.316, 0), top_down /
  angled 50 / auto, err0 and err3 = 3 mm toward the fixed jaw of the plan; top_down and auto feasible, angled 50 rejected by the measured lidar dome)
- arm tool overrides (`test_tools.py`): `move_arm_cartesian`, `move_arm_joints`, `set_gripper` take `surface_z_m` /
  `tilt_override_deg`, report `slow_zone` and describe the slow zone; they and `arm_home` take `surfaces` (a stair
  allows normal speed below the robot floor past its edge, an invalid region is refused)

- metrics (`test_metrics.py`, default Prometheus registry, deltas per test): one test per code point. Tool calls and
  duration for an ok and a failing call (`mcp_tool_calls_total`, `mcp_tool_duration_seconds`); motion queue steps by
  kind and status (converged, aborted_tracking, blended), tracking error samples and queue depth (waiting steps, zero
  after cancel and after draining); `plan_grasp` plans by mode and outcome; the grasp result mapping and
  `mcp_grasp_attempts_total` for lifted / unverified / missed / aborted / infeasible; grip profile uses and the gripper effort gauge
  (motion result and arm state); floor guard slowdowns by closest point (nothing counted with the guard off);
  `mcp_motion_first_setpoint_seconds` and `mcp_motion_tick_period_seconds` per kind (arm, gripper) with a planning
  delay: one first-setpoint observation of the planning time and one 0.04 s period per consecutive setpoint; robot
  events by severity (debounced repeats not counted); `staleness.sample_ages` and the sample age collector (no series
  without a source); navigation goals by result with their duration (errors without a goal not counted); `GET /metrics`
  (200 Prometheus text without a token for 127.0.0.1 and ::1, 403 for other clients, the MCP path stays 401)

- Calibration marker (`nodes/mcp_server/tests/test_calibration_marker.py`): absent marker loads as none, a valid one loads, an invalid one reports everything stale; clearing one item keeps the other, clearing the last deletes the file, clearing an absent file is a no-op, an unreadable marker is deleted when everything is cleared; `arm_home` is refused while home is stale and allowed after set-home (and when only offsets are stale); state reports stale items, with and without a marker; a null path disables the marker.
- Torque gate (`nodes/mcp_server/tests/test_torque_gate.py`): every motion tool is mapped to servo groups; a tool of a released group is refused and publishes nothing while other groups keep working; unknown or enabled state refuses nothing; `enqueue_motions` checks only the groups of its steps; robot and control state list released groups; the arm lease is dropped without publishing when the arm is released; a filter_node released source is torque off, not a takeover; `take_control` arm and the web_ui take are refused while released; torque state config defaults, the ROS robot subscribes to the three torque topics, a servo is not released without a sample or when stale, a malformed payload is ignored, and publish is suppressed for a released arm and head but not for a base release.

### Per-node tests (gps_rtk)

The **gps_rtk** node has tests under `nodes/bridges/gps_rtk/tests/`. Run from `nodes/bridges/gps_rtk`: `uv run pytest tests/ -v` (or `uv run poe test`). Covers: config loading and validation (`test_config.py`: minimal base/rover, rover with rtcm_server_host, invalid mode rejected, load_config from file/missing/empty); NMEA GGA parsing (`test_nmea_parser.py`: lat/lon N/S/E/W, altitude, fix quality, full sentence, RTK fixed quality 4, quality-to-NavSatStatus mapping); serial stream handling (`test_serial_handler.py`: NMEA checksum and append_checksum_if_missing, RTCM3 length parsing, CRC24Q, valid RTCM3 frame build/validation, parser emits NMEA with valid checksum, ignores invalid NMEA, discards unknown bytes). Rover metrics (`test_metrics.py`: GGA gauges with NaN for unreported fields, published-fix counter, NTRIP client connected/rx bytes/reconnects against a fake caster, serial open and read error counters by `op`); `metrics_port` config field (`test_config.py`). GPS status (`test_nmea_parser.py`: `parse_gga_status` reads quality, satellites and HDOP from no-fix GGA with empty lat/lon, empty fields are `None`, no position keys, same fields as `parse_gga`, `parse_gga` still rejects no-fix GGA; `test_status.py`: rover status from no-fix GGA, receiver-silent status, `build_rover_status` timing (nothing before the timeout, silent since start, silent after NMEA stops, fresh no-fix GGA, NMEA without fresh GGA skipped), base status without position, survey state, `ref_station_position`, `gga_seq`; `test_config.py`: `nmea_timeout_s` default 3.0 and > 0; `test_rover_node.py`: no-fix GGA publishes status and real satellite metrics but no NavSatFix, fix GGA publishes both, silent statuses, non-GGA NMEA keeps the receiver not silent; `test_base_node.py`: one status object for the topic and `GET /status`, from every GGA, survey state from the serial handler, caster wired with `status_provider`; `test_ntrip_caster.py`: `GET /status` 200 JSON with Content-Length and close, 503 `no status yet`, no NTRIP auth needed, 404 without a provider, `GET /` source-table request and `/rtk` mountpoint (incl. 401) unchanged; `test_serial_handler.py`: `survey_state` for stored ECEF, no response, zero ECEF, unparseable answer and failed configure).

---

**Maintenance:** Keep this README up to date when adding, removing, or changing tests. Document each new test file and each test (or test group) briefly so the test suite remains easy to navigate.

### test_rplidar_scan_watch.py

Decision logic of the RPLidar scan watchdog (`nodes/bridges/rplidar_a1/scan_watch.py`): no restart during the startup grace, restart when no scan arrives after it, no restart while scans keep arriving, restart when scans stop for the silence timeout, and the silence timeout applies once the first scan has arrived. Also covers the metrics logic: restart reason (startup vs gap), scan rate over the last 10 s (unknown until two scans, 0 after silence), max gap over the last 60 s including the open gap, `lidar_*` export (scans total, rate, max gap), restart counter series by reason, and that `scan_supervisor.py` starts the metrics server from the env port.

### test_client_cpu_load.py

Client CPU load trims (load about 13 on 4 cores made the EKF miss its rate and the map pose go stale).

| Test | Description |
|------|-------------|
| `test_master2master_is_disabled` | `master2master` stays `present: true` but `enabled: false` (its `/controller/*` relays, the gripper camera included, served only the legacy Steam Deck UI). |
| `test_gripper_camera_publish_rate_is_capped` | `gripper_uvc_camera` env has `UVC_MAX_FPS=10`. |

### test_dds_discovery_config.py

Static checks of the DDS discovery setup in Ansible: the shared `ros2_dds_env` is `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` plus `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` and the discovery-server variables are gone; no node env overrides discovery (`ROS_DISCOVERY_SERVER`, `ROS_SUPER_CLIENT`, `ROS_LOCALHOST_ONLY`, `ROS_AUTOMATIC_DISCOVERY_RANGE`) and the only `ROS_STATIC_PEERS` entries are the client `master2master`, `leader_bridge` and `map3d_uplink` node types pointing at the server (server nodes have none); the former `fastdds_discovery_server` node is `present: false` / `enabled: false` on both hosts; the unit template renders the shared env before node env with no discovery-server ordering; `ros2_dds_env` also carries `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` and every node type of both hosts renders it (no other unit template renders the DDS env); Steam Deck uses the client as static peer; `/etc/profile.d/ros2_dds.sh` renders the full `ros2_dds_env` (localhost discovery and UDPv4 transport) for shells; the orphan `fastdds_no_multicast.xml` is gone; the ansible README documents raw Image over loopback UDP as unsupported and the `MALLOC_MMAP_THRESHOLD_` caveat; `--all` deploys include `tasks/ros_packages_sync.yml` after the repo sync (upgrade all ros-jazzy packages together, restart running nodes if anything was upgraded); the client and server deploy playbooks never stop the stack up front (`stop_ros_nodes.yml` is gone) and finally include `tasks/start_ros_nodes.yml` (queued nodes restarted and stopped ones started one by one, `ros2_node_start_interval_s` = 2), and web_ui is deployed first in the client `--all` playbook; secondary ethernet ports are `optional: true` in the rendered netplan, and `--all` deploys install the wait-online drop-in (`--any --timeout=30`); every `env:` key is a list (an empty `env:` parses as null and breaks deploys); logind keeps the node user's shared memory (`RemoveIPC=no`) and running `ros2-*` services restart once when that is first applied.

### test_stereo_depth_config.py

Static checks of the D7 retirement of the Pi stereo pipeline: `stereo_depth` and `stereo_depth_cpp` are `present: false` (the next deploy of those names uninstalls the units), their node types keep only what the uninstall path needs, and the 19113 target is gone.

| Test | Description |
|------|-------------|
| `test_ros2_nodes_entries_are_uninstalled_by_the_next_deploy` | Both `ros2_nodes` entries keep their node type, are `present: false` and carry no `config`. |
| `test_the_entries_name_d7_in_the_comment_above_them` | The comment block directly above each entry names D7. |
| `test_node_types_keep_only_what_the_uninstall_path_needs` | Each type has only `deploy_mode: native` and a `node_launch_command`: no source dir, `src_shared`, `colcon_source`, apt, env, config path, memory or tier keys. |
| `test_background_tier_has_no_stereo_type` | The 100/100 weight tier holds only the two joint-state throttle types. |
| `test_metrics_target_is_removed` | `monitoring_agent_node_targets` has no stereo node and no `:19113` address. |
| `test_client_playbook_still_deploys_both_names_so_the_uninstall_runs` | The client playbook still has a deploy step for each name. |
| `test_ansible_readme_documents_the_retirement` | `ansible/README.md` "Cut-over and rollback" names D7 and `present: false`, has no 19113 and no stereo row in the tier or memory tables. |
| `test_tests_readme_metrics_wiring_rows_have_no_stereo_node` | The `test_config_metrics_port` and `src_shared` rows of this README name no stereo node and no 19113. |
| `test_tests_readme_lists_every_test_of_this_file` | This README has a row for every test of this file. |
| `test_stereo_sources_are_deleted` | `nodes/stereo_depth` does not exist (parametrized per retired node directory); only the uninstall entry remains. |

### test_arducam_tof_config.py

Static checks of the `arducam_tof` deployment (Arducam ToF depth camera for the web UI head tab, Client only, deployed disabled).

| Test | Covers |
|---|---|
| `test_node_type_is_declared_on_the_client` | Native node type `python3 -m arducam_tof`, `nodes/bridges/arducam_tof`, `/etc/ros2/arducam_tof`, `ARDUCAM_TOF_CONFIG`. |
| `test_node_type_is_in_the_normal_cgroup_tier_with_memory_limits` | Normal-tier cgroup weights with `memory_low` and `memory_max`. |
| `test_node_type_has_no_dds_peer` | No `ROS_STATIC_PEERS` on the node type. |
| `test_ros2_nodes_entry_deploys_enabled` | `ros2_nodes` entry is `present: true`, `enabled: true` with a `config` string. |
| `test_deployed_config_holds_only_keys_the_driver_accepts` | Deployed YAML keys are exactly the driver config keys (unknown keys are rejected by the driver), `csi_device: 1f00128000.csi`, metrics port, `range_mode` near or far. |
| `test_deployed_config_loads_in_the_real_driver_load_config` | The deployed YAML loads in the driver's real `load_config`; runs unconditionally. |
| `test_head_tab_topics_are_documented_in_the_config_comments` | The fixed depth, amplitude and camera_info topics are listed in the config comments. |
| `test_metrics_port_is_scraped` | The metrics port is in `monitoring_agent_node_targets`. |
| `test_sdk_task_adds_the_arducam_repo_and_installs_the_sdk` | `arducam_tof_sdk.yml` adds the Arducam apt repo and installs `arducam-tof-sdk-dev`. |
| `test_sdk_task_is_idempotent_and_never_fails_the_deploy` | The SDK task tolerates an unavailable repo or package. |
| `test_client_playbook_includes_the_sdk_task_and_deploys_the_node` | `deploy_nodes_client.yml` includes the SDK task and the tagged `Deploy arducam_tof` block. |

### test_leader_bridge_config.py

Static checks of the `leader_bridge` deployment (leader arm teleop relay, Client only).

| Test | Covers |
|---|---|
| `test_node_type_is_declared_on_the_client` | Native node type with `nodes/leader_bridge`, `/etc/ros2/leader_bridge`, `LEADER_BRIDGE_CONFIG` and the server as its `ROS_STATIC_PEERS`. |
| `test_ros2_nodes_entry_follows_the_schema` | `ros2_nodes` entry with `node_type`, `present: true`, `enabled: true` and a `config` string. |
| `test_deployed_config_validates_against_the_pydantic_config` | The deployed YAML loads into `BridgeConfig`: `/leader/joint_states` to `/filter/input_joint_updates`. |
| `test_master2master_stays_disabled` | `master2master` stays present and disabled. |
| `test_server_needs_no_bridge_entry_and_still_runs_the_leader` | No server entry for the bridge; `lerobot_leader` stays enabled there. |
| `test_client_playbook_deploys_the_bridge` | `deploy_nodes_client.yml` deploys `leader_bridge` with its tags. |
| `test_ci_matrix_lists_the_node` | The CI node matrix names `leader_bridge`. |
| `test_docs_list_the_node_and_this_test` | `nodes/README.md` lists the node and this README documents the file. |
| `test_shipped_arm_tab_names_the_arm_urdf` | The deployed web_ui Arm tab sets `arm_urdf: so101_arm.urdf` and the file exists under `nodes/web_ui/urdf`. |

### test_nav_stack_config.py

Static checks of the mapping/navigation stack from the repo files (YAML via `yaml.safe_load`, launch files via `ast`; no ROS needed).

| Test (group) | Description |
|------|-------------|
| `test_rplidar_default_frame_matches_static_tf_child` | RPLidar launch default `frame_id` (`DEFAULT_FRAME_ID`, env `RPLIDAR_FRAME_ID`) is `laser_frame`, a static TF child of `base_link`. |
| `test_lidar_static_tf_mounted_backwards` | Static TF base_link -> laser_frame is at (0.158, 0.06, 0.20) with yaw = pi (Platform dimensions.md gave (0.15, 0.04); corrected by the 2026-10-05 in-place turn test): the RPLidar is mounted rotated 180 deg (verified on the robot by driving forward). |
| `test_tests_readme_lidar_tf_matches_client_yml` | The lidar TF row in tests/README.md quotes the same base_link -> laser_frame position as client.yml (stale doc guard). |
| `test_nodes_readme_names_the_smac_lattice_planner` | nodes/README.md describes nav2_bringup with the Smac lattice planner, not the replaced NavFn. |
| `test_laser_filter_polygon_matches_the_h_outline` | The laser_filter node config (`/scan` -> `/scan_filtered`, `base_link`, only the known keys) has a startup polygon equal to the generated static footprint (H body + parked arm) + 1 cm: it contains the footprint and keeps returns in the rear notch and in the band just outside the body; the front notch points (under the parked gripper) are inside. |
| `test_laser_filter_ansible_wiring` | laser_filter node type is the repo Python node (`python3 -m laser_filter`, `node_src_dir nodes/laser_filter`, no laser_filters apt package, `LASER_FILTER_CONFIG` pointing at the repo config), enabled node entry, per-node playbook and inclusion in `deploy_nodes_client.yml`. |
| `test_node_apt_install_refreshes_stale_cache` | The ros2_node_deploy apt install refreshes the package index (`update_cache: true`, `cache_valid_time: 3600`) so new packages do not 404 on a stale cache. |
| `test_rplidar_runs_under_scan_supervisor` | rplidar_a1 is launched through `scan_supervisor.py` (runs the launch file, exits 1 for a systemd restart when `/scan` never starts or goes silent). |
| `test_web_ui_frontend_build_runs_at_lowest_priority` | The web_ui `npm ci` / `npm run build` deploy steps run in a transient systemd scope (`CPUQuota={{ ros2_build_cpu_quota }}`, `CPUWeight={{ ros2_build_cpu_weight }}`, `IOWeight=10`) under `nice -n 19 ionice -c 3` (the build overheated and froze the Pi next to the running ROS stack). |
| `test_restart_handler_skips_disabled_nodes` | The ros2_node_deploy "Restart ROS2 node" handler is conditioned on the node being enabled (a disabled node was stopped and then restarted by the change handler). |
| `test_ekf_repo_config_matches_ansible_block` | `nodes/robot_localization_ekf/config/ekf.yaml` equals the Ansible `config: \|` block. |
| `test_ekf_frames_inputs_and_tf` | Both EKF configs: 2D, `publish_tf`, odom/base_link frames, `/odom` fuses vx/vy/vyaw, `/imu/data` fuses yaw rate (absolute yaw only with `imu0_relative: true`). |
| `test_ekf_launch_uses_repo_launch_and_deployed_config` | Ansible launches the repo EKF launch file, which reads `ROBOT_LOCALIZATION_EKF_CONFIG` (default the deployed config path). |
| `test_swerve_controller_does_not_publish_odom_tf` | swerve_controller config sets `publish_tf: false` (EKF owns `odom -> base_link`). |
| `test_swerve_controller_tilt_guard_block` | swerve_controller `tilt_guard` block: enabled on `/imu/data` -> `/tilt_guard/state`, warning below error per axis, pitch (longer axis) limits larger than roll, `error_action: brake`, persistence/hysteresis/recovery/level reference keys present, level reference roll 0.7 / pitch 1.3 deg (same as the mcp_server floor guard), `state_publish_hz` 1. |
| `test_slam_params_frames_and_topics` | slam_toolbox params: base_link/odom/map frames, `/scan_filtered`, mapping mode, 0.025 m resolution, no hard-coded `map_file_name`. |
| `test_slam_launch_starts_async_lifecycle_node` | Launch starts `async_slam_toolbox_node` as a lifecycle node with configure + activate transitions. |
| `test_slam_launch_resumes_only_when_posegraph_exists` | `map_resume_parameters` (compiled out of the launch file) returns `map_file_name` + `map_start_at_dock` only when `<base>.posegraph` exists; otherwise it returns `map_file_name: ""` so a `map_file_name` from a params file never makes slam_toolbox load a missing file. |
| `test_slam_launch_appends_override_params_only_when_present` | `params_files` returns the repo `slam_params.yaml` first and appends the deployed config only when it exists and is non-empty (ROS params files: later wins). |
| `test_slam_launch_override_defaults_and_env` | Launch reads the override path from `SLAM_TOOLBOX_CONFIG`, defaulting to `/etc/ros2/slam_toolbox/config.yaml`. |
| `test_slam_launch_map_base_from_configuration` | `configured_map_base` takes `map_file_name` from the last params file that sets it (falls back to `/var/lib/ros2/maps/slam_map`); an explicit `SLAM_TOOLBOX_MAP_BASE` value wins over the files. |
| `test_slam_toolbox_ansible_wiring` | Node type defaults (native, apt package, repo launch, no CPU quota, `memory_low` / `memory_max`, `config_path: /etc/ros2/slam_toolbox`, env `SLAM_TOOLBOX_CONFIG`) and an enabled `ros2_nodes` entry. |
| `test_slam_toolbox_ansible_config_overrides` | The `ros2_nodes` slam_toolbox `config: \|` block parses as YAML with `map_file_name` (`/var/lib/ros2/maps/slam_map`), `min_laser_range` 0.15 and `max_laser_range` 12.0, only known slam_toolbox params, and the same map path as the web_ui Map tab `map_save_path`. |
| `test_slam_playbooks_deploy_node_and_create_maps_dir` | `deploy_nodes_client.yml` deploys slam_toolbox and create `/var/lib/ros2/maps` (owner `ansible_user`, 0755). |
| `test_nav2_launch_passes_repo_params_without_localization` | Nav2 launch passes the repo `params_file`, keeps `use_localization:=False`. |
| `test_nav2_has_every_server_section` | `nav2_params.yaml` has a section for every server started by Jazzy `navigation_launch.py`. |
| `test_nav2_frames` | bt_navigator, costmaps, collision_monitor, behavior_server, docking_server and route_server frames/topics. |
| `test_nav2_costmap_layers_and_footprint` | Both costmaps: footprint equals the H outline from `scripts/gen_nav_footprint.py`, obstacle layer on `/scan_filtered`, inflation; global static layer on `/map` (transient local). |
| `test_nav2_mppi_omni_with_swerve_limits` | MPPI Omni, vx +-0.25, vy 0.25, wz 0.5, `visualize: true` (local plan on `/optimal_trajectory`). |
| `test_nav2_planner_is_footprint_aware_smac_lattice` | `GridBased` (only planner, name kept for the BT and web UI) is `nav2_smac_planner::SmacPlannerLattice` with `allow_unknown: true`, tolerance 0.5 m, `max_planning_time` <= 2 s, no reverse expansion, moderate `cost_penalty` (1-3), analytic expansion <= 3 m. |
| `test_nav2_lattice_primitives_are_omni_at_the_costmap_resolution` | `lattice_filepath` lies under `ros2_repo_dest` and points at the committed `nodes/nav2_bringup/config/lattice/output.json`; its metadata equals `config.json` (omni, 0.5 m turning radius), trajectory count matches, and `grid_resolution` equals both costmap resolutions. |
| `test_nav2_costmaps_match_the_slam_map_resolution` | Both costmaps use the slam_toolbox `resolution`, 0.025 m. |
| `test_nav2_obstacle_layer_keeps_obstacles_in_the_notches` | Both costmaps: obstacle layer `footprint_clearing_enabled: false` (its column fill treats the H as convex and would erase obstacles in the notches). |
| `test_slam_map_resolution_cost_is_bounded` | slam_toolbox correlation and loop search windows are whole numbers of their own cells, the correlation grid is no coarser than the map, and `map_update_interval` is 10 s (the 2.5 cm raster has 4x the cells). |
| `test_nav2_velocity_smoother_matches_swerve` | Smoother limits `[0.25, 0.25, 0.5]`, odom `/odometry/filtered`. |
| `test_nav2_collision_monitor_uses_scan` | Valid polygon(s) (static ones 4+ points, every sub-polygon of StopBox and CliffStop 4+ points, the dynamic ArmStop a `polygon_sub_topic`) and an enabled `/scan_filtered` scan observation source (U5: kept next to the ToF sources). |
| `test_nav2_collision_monitor_stopbox_enabled_on_filtered_scan` | collision_monitor stays in the `cmd_vel_smoothed -> cmd_vel` chain with its core keys; `StopBox` (holonomic stop `velocity_polygon`, min 4 points) is enabled on the footprint-filtered scan. |
| `test_stopbox_covers_every_velocity` | Every command (directions in 5 deg steps, speeds 0 to 0.25 m/s, yaw rates -0.5 to 0.5) selects a StopBox sub-polygon, as Nav2 keeps the previous one for an uncovered velocity. |
| `test_stopbox_stops_toward_an_obstacle_and_lets_the_robot_escape` | Per side (ahead of a front wheel, x 0.23; ahead of the lidar dome in the front notch, x 0.202 (dome front 0.187); behind a rear wheel; beside each side frame, y +-0.1658 with the measured 35 mm frames), a return 1.5 cm outside the H footprint (just outside the self-filter polygon) stops motion toward it (straight and diagonal) and rotation in place, but not motion away from it or along it. Simulates Nav2's sub-polygon selection and ray-cast point-in-polygon. |
| `test_stopbox_ignores_notch_obstacles_unless_moving_into_them` | A return deep in the rear notch stops no translation (only rotation in place); one 1.5 cm behind the crossbar stops only backward motion; front notch returns (under the parked gripper, inside the footprint) stop every motion. |
| `test_nav2_goal_tolerance_tight_enough_for_swerve` | Goal checker tolerances 0.01 m / 0.035 rad (1 cm / 2 deg), stateful: the latch only holds because the BT no longer replans periodically. |
| `test_nav2_replans_only_when_path_invalid` | `bt_navigator` uses the "replanning only if path becomes invalid" BT: periodic replans reset the goal checker latch and the rotation shim position check. |
| `test_nav2_progress_checker_counts_rotation` | `progress_checker` is `PoseProgressChecker` (counts rotation), movement radius <= 0.05 m and a required angle set. |
| `test_costmaps_add_no_margin_beyond_the_footprint` | Both costmaps: `footprint_padding` 0, inflation only to the H footprint's circumscribed radius rounded up to one cell (0.275 m), `cost_scaling_factor` 20-30, identical inflation on both. |
| `test_safety_polygons_hug_the_footprint` | The laser_filter polygon reaches at most 1.5 cm beyond the footprint, each directional StopBox sub-polygon at most its 2 cm sweep (2.45 cm on a diagonal sector), and `rotate` is the footprint bounding box + 2 cm. |
| `test_costmaps_wait_for_odom_through_a_gradual_restart` | Both costmaps: `initial_transform_timeout` >= 300 s, so a deploy that restarts nav2 before the gradual ramp brings up the servos and EKF (odom TF) does not abort the bringup. |
| `test_ekf_rate_fits_the_cpu_budget` | EKF `frequency` 30 Hz: at 50 Hz it missed its rate on the loaded Pi and its odom TF lagged the scans. |
| `test_slam_waits_for_a_lagging_odom_transform` | slam_toolbox `transform_timeout` >= 0.5 s so scans are not dropped when the odom TF lags. |
| `test_nav2_rotates_toward_path_first_and_keeps_front_leading` | `FollowPath` is the RotationShimController around MPPI (turn to face the path first, threshold 0.3-1.0 rad, rotation speed within `wz_max`, final rotation to the goal heading, MPPI GoalAngleCritic disabled so only the shim turns to the goal heading, GoalCritic weight >= 8, `max_angular_accel` <= 0.5, `rotate_to_heading_once`) and MPPI's PathAngleCritic prefers forward driving (mode 0): the lidar is partly covered at the back and right. |
| `test_nav2_readme_documents_collision_monitor_validation` | Nav2 README documents StopBox `enabled: true` on `/scan_filtered` and the live `collision_monitor_state` validation procedure. |
| `test_nav2_docking_server_configures_without_docks` | Non-empty `dock_plugins`, no docks. |
| `test_nav2_readme_documents_plan_topics` | Nav2 README names `/plan` and `/optimal_trajectory`. |
| `test_web_ui_reads_no_master2master_relay_topics` (`test_web_ui_overlays_config.py`) | No part of the client web_ui config (tabs, overlays, map) contains a `/controller/*` topic: those exist only through the master2master proxy. |
| `test_overlays_use_no_relay_only_topics`, `test_lat_lon_come_from_the_client_gps_fix`, `test_vel_comes_from_the_swerve_odometry`, `test_fl_steer_is_selected_by_joint_name` (`test_web_ui_overlays_config.py`) | web_ui status bar overlays read live client topics: no `/controller/*`, Lat/Lon from the gps_rtk_rover `/client/gps/fix`, Vel from the swerve_controller `odom_topic`, FL Steer from its `joint_states_topic` as `position[name=fl_steer]`. |
| `test_web_ui_enables_the_imu_calibration_section`, `test_bno055_keeps_the_default_calibration_names` (`test_web_ui_calibration_config.py`) | The client web_ui config has a `calibration` block naming `/imu/calibration` and the `/imu/calibration/save` and `/imu/calibration/reset` services (otherwise the Calibrate page answers "not configured"), and bno055_imu does not rename them. |
| `test_web_ui_serves_the_repo_models_dir`, `test_models_dir_holds_a_pdf_and_3mf_per_model` (`test_web_ui_models_config.py`) | The client web_ui `models_dir` is the repo checkout's `models/`; that dir holds only PDF and 3MF files, one of each per model, including the ChArUco board. |
| `test_web_ui_has_map_nav_tab` | web_ui config has the `map` tab of type `map_nav` with exactly the expected fields, including the POI topics (`/poi/list`, `/poi/command`, `/poi/result`), the grasp topics (`/grasp/command`, `/grasp/result`), the arm mount `arm_offset` `[0.0592, -0.05, 0.104]` (measured 2026-10-10) and the merged 3D view contract fields (`local_costmap_topic`, `base_urdf`, `base_joint_states_topic`, `arm_urdf`, `arm_joint_states_topic`, arm command topic `/filter/web_ui_joint_commands`) and the arm Trigger services `/arm/home` / `/arm/set_home`, the Head RGB-D layer topic `rgbd_cloud_topic: /head_rgbd/points`, and the keyed CARTO `tile_url` (`?key={api_key}`) with `tile_api_key_env: WEB_UI_TILE_API_KEY` and the `tile_sources` list (Esri satellite max zoom 20 in `{z}/{y}/{x}` order, keyed CARTO street max zoom 18) with `default_tile_source: satellite` and `tile_display_zoom: 19`. |
| `test_web_ui_arm_offset_matches_mcp_server_arm_mount` | The web_ui map tab `arm_offset` equals the mcp_server `arm.base_in_base_link` x, y, z, so the drawn arm and the grasp planner agree. |
| `test_web_ui_loads_the_tile_api_key_env_file_and_never_stores_the_key` | web_ui's unit loads `/etc/ros2/web_ui/env` via `environment_file`; `web_ui_tile_key.yml` reads `CARTO_API_KEY` from the controller, writes `WEB_UI_TILE_API_KEY=` with mode `0600`, `no_log: true` and queues a web_ui restart, and is included by `deploy_nodes_client.yml`. |
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
| `test_colcon_source_build_runs_at_lowest_priority_and_only_on_a_new_commit` | `colcon_source_build.yml` clones the pinned commit and builds with `systemd-run` (CPUQuota `ros2_build_cpu_quota`, CPUWeight `ros2_build_cpu_weight`, IOWeight 10), nice 19, ionice idle, `--merge-install`, one worker, `MAKEFLAGS=-j{{ colcon_src.jobs | default(ros2_build_jobs) }}` (default 4), a per-commit-and-patch-hash stamp (`creates:`) and a restart notify; included by the role. |
| `test_colcon_source_build_applies_patches_and_keys_the_stamp_on_their_content` | `colcon_source_build.yml` applies the `colcon_source.patches` with `ansible.builtin.patch` between the (forced) clone and the build, hashes them into the stamp key, and drops the stamp when `install/setup.bash` is missing. |
| `test_rf2o_retry_laser_tf_patch_exists_and_is_wired` | `patches/0001-retry-laser-tf.patch` exists, skips scans until the laser TF lookup succeeds, and is listed in the rf2o node type's `colcon_source.patches`. |
| `test_rf2o_odom_relay_declares_numpy` | `rf2o_odom_relay/pyproject.toml` lists `numpy` in `[project].dependencies` (numpy guard, parsed with `tomllib`). |
| `test_launcher_sources_the_colcon_workspace_after_ros` | The launcher template sources the colcon `install/setup.bash` after `/opt/ros/jazzy/setup.bash`, and `resolve_and_deploy.yml` passes `colcon_source`. |
| `test_nav2_keepout_filter_sits_right_before_inflation` | Global plugins are `static_layer, obstacle_layer, keepout_filter, inflation_layer`, local `obstacle_layer, tof_voxel_layer, keepout_filter, inflation_layer`; the keepout filter index is the inflation index minus 1 and it is not in `filters`. |
| `test_nav2_keepout_filter_settings` | `keepout_filter` in both costmaps: `KeepoutFilter` plugin, enabled, info topic equal to the `costmap_zones` default, `transform_tolerance` 0.2, `override_lethal_cost` true, `lethal_override_cost` 252. |
| `test_nav2_speed_filter_only_in_local_costmap` | Local `filters` is `[speed_filter]` (`SpeedFilter`, info topic equal to the `costmap_zones` default, `/speed_limit`, tolerance 0.2); global costmap has no speed filter. |
| `test_nav2_keepout_docs_describe_lethal_override_correctly` | The nav2_params comments and the README zones section say 252 is written only while the robot is inside a zone (not that zone cells are blocked at 252) and that 1.3.10 gives live mask updates. |
| `test_nav2_controller_listens_on_the_speed_limit_topic` | `controller_server.speed_limit_topic` equals the SpeedFilter topic `/speed_limit`. |
| `test_nav2_launch_uses_the_repo_wrapper` | client.yml starts nav2_bringup with `nodes/nav2_bringup/launch/nav2_launch.py` and installs `ros-jazzy-opennav-docking`. |
| `test_nav2_launch_includes_stock_bringup_without_localization` | `nav2_launch.py` (via `ast`) includes the stock `bringup_launch.py` with `use_localization` `'False'` and the repo `nav2_params.yaml`. |
| `test_nav2_launch_grasp_docking_server_is_namespaced_and_remapped` | One `PushRosNamespace('grasp')` group with `opennav_docking` `docking_server` (remaps `cmd_vel` to `/cmd_vel_nav`, `navigate_to_pose` to `/navigate_to_pose`, params `grasp_docking_params.yaml`) and `lifecycle_manager_grasp` (autostart, `node_names ['docking_server']`). |
| `test_grasp_docking_params_plugin_and_external_detection` | `/grasp/docking_server`: plugin list `grasp_servoed` = `SimpleNonChargingDock`, external detection on with timeout 1.0, all translations/rotations zero, no stall detection. |
| `test_grasp_docking_controller_uses_absolute_costmap_topics_and_clears_the_dock_threshold` | Controller costmap and footprint topics are absolute (`/local_costmap/...`), collision detection on, `simulation_time_step` set, `dock_collision_threshold` >= 0.25 + `docking_threshold`. |
| `test_grasp_docking_angular_speed_within_swerve_limit` | `v_angular_max` and `rotate_to_heading_angular_vel` are at most the swerve wz limit (velocity_smoother 0.5). |
| `test_mcp_server_approach_block_matches_grasp_docking_params` | mcp_server `approach`: `staging_offset_m` = -`staging_x_offset`, `docking_threshold_m` = `docking_threshold`, `/grasp/detected_dock_pose`, `/grasp/dock_robot`, tracker `rate_hz` > 2 / `external_detection_timeout`. |
| `test_mcp_server_approach_block_sets_every_weight_explicitly` | The deployed mcp_server `approach.weights` lists margin 1.0, cost 0.5, travel 0.2, heading 0.1 instead of relying on the defaults. |
| `test_every_test_is_documented_in_tests_readme` | Every `test_*` function in `test_nav_stack_config.py` is listed in this section. |

### test_nav_footprint.py

Unit tests for `scripts/gen_nav_footprint.py` (shapely geometry, no ROS; the parked-arm shadow runs `python -m mcp_server.footprint`
in the mcp_server uv environment, cached per client.yml content) and the check that the committed `nav2_params.yaml`,
`footprint_filter.yaml` and client.yml `nav_footprint` region equal its output.

| Test (group) | Description |
|------|-------------|
| `test_default_body_has_frames_crossbar_wheels_and_lidar` | `gen_nav_footprint.yaml` body has both side frames, the crossbar, four wheels and the lidar. |
| `test_body_is_read_from_client_vars_mcp_server_when_present` | A `body:` in the mcp_server `config: |` block of client.yml is used instead of the yaml defaults. |
| `test_body_falls_back_to_generator_yaml_without_client_body` | Without a client.yml `body:` (or without client.yml) the yaml defaults are used. |
| `test_footprint_is_one_simple_polygon` | The footprint is one valid, simple (no self-intersection), counter-clockwise polygon without a repeated closing point. |
| `test_footprint_contains_every_body_part` | Every frame, the crossbar, every wheel envelope circle and the lidar circle lie inside the footprint (0.15 mm rounding tolerance). |
| `test_footprint_keeps_the_front_and_rear_notches` | The body outline keeps both notches (inside its convex hull, outside it); the static footprint keeps the rear notch and covers the front one (under the parked gripper). |
| `test_static_footprint_covers_the_parked_arm` | The parked-arm shadow is a valid polygon reaching over 8 cm beyond the body front, and the static footprint contains it and the body outline. |
| `test_parked_arm_shadow_needs_the_mcp_server_arm_config` | Without client.yml the shadow is empty and the footprint is the body outline. |
| `test_footprint_hugs_the_body_parts` | The footprint adds under 1 % area to the union of the parts and the parked-arm shadow and reaches nowhere more than one costmap cell beyond it. |
| `test_footprint_changes_with_a_body_number` | Larger wheel radii give a larger footprint. |
| `test_radii_follow_nav2_definitions` | Circumscribed radius = farthest vertex, inscribed = nearest edge (Nav2 `calculateMinAndMaxDistances`). |
| `test_inflation_radius_is_the_circumscribed_radius_rounded_up_to_a_cell` | Inflation radius = circumscribed radius rounded up to the 2.5 cm cell. |
| `test_cost_scaling_is_steep_but_keeps_the_circumscribed_cost_positive` | `cost_scaling_factor` is `footprint_geometry.cost_scaling_factor` (at most `max_cost_scaling_factor`, at most ln(252) / (circumscribed - inscribed)), 20-30, the inflation cost at the circumscribed radius stays >= 1 (MPPI / Smac footprint-check speed-up) and is under 1/4 of the inscribed cost 6 cm out. |
| `test_laser_polygon_is_the_footprint_plus_margin_and_keeps_the_notches` | The laser polygon is simple, contains the footprint + 1 cm, reaches at most 1.5 cm beyond it and leaves the rear notch point outside. |
| `test_offset_polygon_grows_by_the_margin` | A square grows by exactly the margin (mitred corners). |
| `test_stopbox_sub_polygons_sweep_the_footprint_toward_their_motion` | Nine sub-polygons (rotate + 8 sectors), each simple with 4+ points, containing the footprint and reaching beyond the self-filter polygon; each directional one contains the footprint moved 2 cm along every direction of its sector but not the footprint moved 1.5 cm away. |
| `test_stopbox_rotate_is_the_bounding_box_plus_margin` | `rotate` is the 4-point footprint bounding box + 2 cm. |
| `test_stopbox_polygons_function_matches_generate` | `stopbox_polygons` gives the same result as `generate`. |
| `test_nav2_params_equal_the_generator_output` | Re-rendering `nav2_params.yaml` changes nothing. |
| `test_laser_filter_equals_the_generator_output` | Re-rendering `footprint_filter.yaml` changes nothing. |
| `test_client_vars_equal_the_generator_output` | Re-rendering `client.yml` (the mcp_server `nav_footprint` region) changes nothing. |
| `test_mcp_nav_footprint_numbers_come_from_the_generator_config` | The mcp_server `nav_footprint` numbers (circle segments, gap closing, laser and stopbox margins, dynamic simplification, decimals, costmap resolution, maximum cost scaling) equal `gen_nav_footprint.yaml`, plus the generated parked-arm shadow. |
| `test_footprint_geometry_is_shared_with_mcp_server` | The generator's outline, parked-arm union, offset and vertex-order functions are imported from `mcp_server.footprint_geometry` (one implementation for the static and the dynamic footprint). |
| `test_inflation_formula_lives_in_footprint_geometry` | Circumscribed / inscribed radius, inflation radius and cost scaling factor come from `mcp_server.footprint_geometry` (mcp_server sets the dynamic footprint's inflation with the same formula). |
| `test_generated_values_land_in_the_parsed_yaml` | Parsed footprints, padding, inflation, StopBox order/points/ranges, the laser polygon and the mcp_server `nav_footprint` numbers equal the generator output. |
| `test_cliff_polygon_is_the_footprint_grown_by_the_cliff_margin` | The cliff polygon equals the footprint offset by `cliff_margin_m` (0.25), contains the footprint and has >= 4 points. |
| `test_cliff_margin_must_be_positive` | `Settings` rejects `cliff_margin_m` 0. |
| `test_cliffstop_region_renders_the_direction_aware_sub_polygons` | The `cliffstop` region renders `velocity_polygons: [rotate, forward, back]` and each sub-polygon's points and ranges as generated. |
| `test_cliffstop_checks_the_cliff_margin_except_while_backing_away` | `rotate` (below the StopBox rotate speed) and `forward` are the cliff polygon, `back` the footprint; every sub-polygon has >= 4 points and yaw limits +-10. |
| `test_cliffstop_selects_by_command_direction` | Every 5 deg at 0.1 m/s, first-match selection picks `forward` within +-90 deg (sideways included) and `back` beyond. |
| `test_nav2_cliffstop_polygon_is_wired_and_listed` | `CliffStop` block in nav2_params (holonomic velocity_polygon, approach, 1.2 s, tof_cliff, >= 4 points per sub-polygon) exists and is listed in `polygons`. |
| `test_main_writes_the_files_and_is_idempotent` | With bigger wheels in a temp config (and no client.yml, whose region is then skipped), `main` rewrites copies of both files (footprint now contains the bigger wheel), a second run changes nothing, and sections outside the generated regions are untouched. |

### test_poi_store_config.py

Static wiring invariants of the poi_store node (Ansible, playbooks, lint script, docs).

| Test | What it covers |
|---|---|
| `test_poi_store_node_type_defaults` | `poi_store` node type: native, `nodes/poi_store`, `python3 -m poi_store`, `config_path` `/etc/ros2/poi_store`, env `POI_STORE_CONFIG`. |
| `test_poi_store_entry_after_mcp_server_with_config` | Present + enabled `ros2_nodes` entry directly after `mcp_server`; config has `store_path` `/var/lib/ros2/poi/poi.json` and the `/poi/list`, `/poi/command`, `/poi/result` topics. |
| `test_poi_directory_task_owned_by_node_user` | `poi_store_dir.yml` creates `/var/lib/ros2/poi` owned by `ansible_user`. |
| `test_poi_store_config_photos_block` | The poi_store config carries `photos:` (`store_dir` `/var/lib/ros2/poi/lookups`, `max_records` 3000, `max_bytes` 300000000, `max_age_days` null (no age limit), `max_image_bytes` 5000000, `prune_interval_s` 3600, `summary_topic` `/poi/photos`). |
| `test_poi_store_config_photos_block_has_a_comment_per_key` | Every key of the `photos:` block has a comment line directly above it. |
| `test_poi_directory_task_ensures_photo_directory_after_parent` | `poi_store_dir.yml` also creates `/var/lib/ros2/poi/lookups` (`ansible_user`, `0755`) after `/var/lib/ros2/poi`. |
| `test_poi_playbook_deploys_node_and_creates_directory` | `deploy_nodes_client.yml` creates the directory and deploy `poi_store` (after `mcp_server`). |
| `test_lint_script_and_node_files` | `scripts/lint-all-nodes.sh` lists the node; pyproject, uv.lock, README and tests exist. |
| `test_docs_mention_poi_store` | nodes/README.md, ansible/README.md and the ansible-deploy skill mention `poi_store`. |

The node's own tests (store, models, config and `test_metrics.py`: `poi_commands_total{op,ok}` for ok, failed and unparseable (`invalid`) commands, `poi_count` after adds, clear and load, `poi_save_failures_total` with the rollback, the `metrics_port` config field; including `kind: object` POIs with their sighting fields and defaults for old files, and the `clear` op that deletes every POI of one `created_by`, persists and keeps the revision when nothing was removed) live in `nodes/poi_store/tests/` and need no ROS: `cd nodes/poi_store && uv run pytest tests -q`.

### test_costmap_zones_config.py

Static wiring invariants of the costmap_zones node (Ansible node type, entry, playbook, metrics target, docs).

| Test | What it covers |
|---|---|
| `test_costmap_zones_node_type_defaults` | Native `python3 -m costmap_zones`, `nodes/costmap_zones`, `src_shared`, memory 96M/256M, normal weights 500/500/-500, `config_path`, `ros-jazzy-nav2-msgs`, env `COSTMAP_ZONES_CONFIG`. |
| `test_costmap_zones_entry_after_poi_store_with_config` | Present + enabled entry directly after `poi_store`; config keys are exactly the `CostmapZonesConfig` fields and validate against it. |
| `test_info_topics_match_nav2_params` | The entry's `keepout_info_topic` / `speed_info_topic` equal `filter_info_topic` of the keepout filters (both costmaps) and the speed filter in `nav2_params.yaml`. |
| `test_metrics_target_for_costmap_zones` | `monitoring_agent_node_targets` scrapes `127.0.0.1:19111/metrics`. |
| `test_playbook_deploys_costmap_zones_after_poi_store` | `deploy_nodes_client.yml` deploys it after `poi_store` with tags `costmap_zones, apt, python, build, config`. |
| `test_node_files_exist` | pyproject, uv.lock and README of the node exist. |
| `test_docs_mention_costmap_zones_and_port` | ansible/README.md (incl. port 19111 table) and the ansible-deploy skill mention it. |

`test_unit_cgroups.py` lists `costmap_zones` in the normal tier.

### test_marker_detector_config.py

Static wiring invariants of the marker_detector node (Ansible node type, entry, playbook, metrics target, docs).

| Test | What it covers |
|---|---|
| `test_marker_detector_node_type_defaults` | Native `python3 -m marker_detector`, `nodes/marker_detector`, `src_shared`, memory 128M/512M, weights 500/500/-500, `config_path`, env `MARKER_DETECTOR_CONFIG`. |
| `test_marker_detector_entry_after_poi_store_with_config` | Present + enabled entry after `poi_store`; config keys are exactly the `MarkerDetectorConfig` fields (no missing, no extra), validate against it, `metrics_port` 19118. |
| `test_metrics_target_for_marker_detector` | `monitoring_agent_node_targets` scrapes `127.0.0.1:19118/metrics`. |
| `test_playbook_deploys_marker_detector_after_poi_store` | `deploy_nodes_client.yml` deploys it after `poi_store` with tags `marker_detector, apt, python, build, config`. |
| `test_node_files_exist` | pyproject, uv.lock and README of the node exist. |
| `test_docs_mention_marker_detector_and_port` | ansible/README.md (incl. port 19118 table), the ansible-deploy skill and nodes/README.md mention it. |

### test_mcp_camera_config.py

Static checks of the mcp_server camera tools wiring (YAML and README only; no ROS needed).

| Test (group) | Description |
|------|-------------|
| `test_setup_tasks_create_the_calibration_directory_owned_by_the_node_user` | `tasks/mcp_server_setup.yml` creates `/var/lib/ros2/camera_calibration` owned by `ansible_user`. |
| `test_cameras_config_front_not_calibrated_gripper_calibrated` | The mcp_server `cameras` block has `calibration_dir`; the front (head RGB) camera sits on `head_link` with the URDF `head_link -> head_camera_link` mount and null `intrinsics` (not calibrated), the gripper camera has intrinsics and a mount on `gripper_link`. |
| `test_gripper_parent_frame_is_a_link_of_the_arm_urdf` | `cameras.gripper.parent_frame` names a link in the arm URDF. |
| `test_front_parent_frame_is_a_head_urdf_link_and_the_mount_is_the_urdf_camera_origin` | `head.urdf_path` is `nodes/web_ui/urdf/head.urdf`; `cameras.front.parent_frame` names a link of it and is the parent of `head_camera_joint`; the front mount equals that joint's origin (xyz, rpy). |
| `test_arm_reach_keys_are_ordered` | `arm.reach_inner_m` is positive and below `arm.reach_outer_m`. |
| `test_readme_documents_the_camera_tools_frames_and_calibration` | `nodes/mcp_server/README.md` documents the seven camera tools, frames, the not-calibrated default, `base_in_base_link`, the calibration directory and the procedure. |

### test_mcp_server_config.py

Static checks of the robot MCP server wiring (YAML, the unit template rendered with jinja2, the token script run with a
fake `ssh`; no ROS needed).

| Test (group) | Description |
|------|-------------|
| `test_mcp_server_node_type_defaults` | `mcp_server` node type: native, `nodes/mcp_server`, `python3 -m mcp_server`, no CPU quota, MemoryMax 1G, MemoryLow 184M, agent-facing tier, `config_path` `/etc/ros2/mcp_server`, env `MCP_SERVER_CONFIG`, token only via `environment_file: /etc/ros2/mcp_server/token`. |
| `test_mcp_server_node_entry_and_config` | Present + enabled `ros2_nodes` entry; config block keys (incl. `control`), server 0.0.0.0:18200 `/mcp`, repo-relative arm URDF that exists, home file `/var/lib/ros2/arm/home.yaml`; the `grasp_check` block is an allowed key. |
| `test_mcp_server_topics_match_filter_node_lease` | mcp_server autonomy command/release/active source and follower feedback topics equal filter_node's. |
| `test_filter_node_autonomy_params` | filter_node config sets `autonomy_input_topic`, `autonomy_release_topic`, `active_source_topic`. |
| `test_gripper_camera_enabled` | `gripper_uvc_camera` is present and enabled again. |
| `test_shoulder_lift_upper_limit_allows_reaching_below_the_floor` | mcp_server `arm.joint_limit_overrides_rad` widens shoulder_lift to [-1.745, 1.9] rad (tested on the robot: 1.87 rad reached without collision, the stretched arm cannot be lifted beyond about 1.85). |
| `test_gripper_camera_rotated_180_at_source` | `gripper_uvc_camera` env sets `UVC_ROTATE_DEG=180` exactly once (the wrist image is upside down at wrist roll 0; rotation happens in the camera node, not downstream). |
| `test_mcp_server_nav_tolerances_match_nav2_goal_checker` | mcp_server `nav.goal_xy_tolerance_m` / `goal_yaw_tolerance_deg` in client.yml equal the nav2_params.yaml goal checker (0.01 m, 0.035 rad ~ 2 deg). |
| `test_claude_agent_nav_tolerances_match_mcp_server` | claude_agent `nav_goal_xy_tolerance_cm` / `nav_goal_yaw_tolerance_deg` (and the `nav_intermediate_*` pair) in client.yml equal the mcp_server `nav` values. |
| `test_mcp_server_arm_mount_measured_and_height_agree_with_claude_agent` | mcp_server config `arm.arm_base_height_m` is 0.100 and `arm.base_in_base_link` is the mount measured 2026-10-10 `{x: 0.0592, y: -0.05, z: 0.104, yaw: 0.0}`; the claude_agent `arm_base_height_m` states the same height. |
| `test_mcp_server_floor_guard_and_grasp_blocks_are_deployed` | mcp_server config has the `floor_guard` block (enabled, 2 cm margin, 0.2 slow speed scale, surface 0, IMU max age 1 s, `imu_level_reference` roll 0.7 / pitch 1.3 deg: the BNO055 mounting bias from the 2026-10-11 investigation), the `grasp` block (5 mm interpolation, 8 cm max opening, auto order top_down, angled, scoop, scoop max pitch 40 deg and gap margin 4 mm, tall ratio 1.5 / grasp height fraction 0.3 / lift speed 0.05, min object width 1 cm, fixed jaw clearance 7.5 mm with the 15 mm opening margin) and the `/grasp/command` / `/grasp/result` topics; the claude_agent lists `grasp_object` / `release_object` as effectors and `plan_grasp` as a sensor. |
| `test_mcp_server_grip_profiles_block_is_deployed` | (each preset also carries `contact_load_delta` 20) mcp_server config has the `grip_profiles` block: presets gentle / normal / firm with increasing torque_limit and squeeze, all within the hard caps (torque_limit_max 800, squeeze_max_rad 0.08), default `normal`, gentle closing slower with a target_load below its contact threshold, close speeds within the 0.5 rad/s gripper cap and crush loads above the target loads; the presets equal the 2026-10-10 tuned values (gentle 0.01 / 250 / 0.2 / 80 / 120 / 200, normal 0.03 / 500 / 0.5 / 200 / 300 / 450, firm 0.05 / 650 / 0.4 / 300 / 350 / 600) plus soft for plush (0.08 / 600 / 0.3 / 150 / 250 / 550: squeeze at the cap, torque above normal and within the cap, crush_load below its torque limit), `retighten` off for gentle and on for normal / firm / soft, and `retighten_load_below` 30; `grasp.approach_speed_scale` is 0.3, within `arm_max_speed_scale`; the grasp block sets no fixed `close_effort_threshold`; `topics.follower_set_register` is the follower bridge's `set_register` topic and the follower publishes the gripper effort. |
| `test_mcp_server_monitor_block_has_ordered_thresholds` | mcp_server `monitor` block: servo 60/70 C, CPU 75/82 C, bump warning below critical, stall 1.0 s, tilt 10 deg, battery warning margin 0.2 V/cell. |
| `test_mcp_server_monitor_topics_match_their_producers` | mcp_server monitor topics: `/follower/servo_registers`, `base_servo_registers` equals `/swerve_drive/servo_registers`, `/imu/data`, `/robot_events`, `swerve_odom` equals the swerve controller `odom_topic`, `rf2o_twist` equals the rf2o relay `output_topic`. |
| `test_mcp_server_readme_documents_monitor_and_events_contract` | `nodes/mcp_server/README.md` documents get_body_state, the per-call digest, the `/robot_events` contract, early-return (`interrupted_by`, expected/achieved), event types and the `monitor` thresholds. |
| `test_web_ui_arm_tab_mount_equals_mcp_server_gripper_camera_mount` | The arm tab's `gripper_camera_mount` (`xyz`, `rpy`) equals mcp_server `cameras.gripper.mount` in client.yml. |
| `test_web_ui_tab_set` | web_ui tabs are exactly map (map_nav, first), agent (agent_chat), arm (type `arm`, label `Arm`, `gripper_camera_topic` `/camera_0/image_raw/compressed`, `head_camera_topic` (`/head_camera/image_raw/compressed`), no `topic`), head (type `head`, `/head_camera/image_raw/compressed`, depth `/tof/depth/image_raw`, amplitude `/tof/amplitude/image_raw`, `tof_range_m` 4.0), poi (type `poi`, label `POI`); arm_servos, local_nav, gps_nav, scene3d and robot_status are gone. |
| `test_mcp_server_head_camera_is_the_head_camera_stream` | mcp_server `topics.head_camera` is `/head_camera/image_raw/compressed`, no `front_camera` or `realsense_camera` key remains. |
| `test_mcp_server_states_no_hardcoded_front_camera_size` | mcp_server `tools.py` states no 640x480 front camera size; the mcp_server README gives 1152x648 and 384x216 for the head camera and drops "640x480 for both" and the pi_cameras 640x480 wording. |
| `test_mcp_server_get_camera_image_docs_name_the_front_camera_module_3` | No mcp_server README or source text names IMX219, 480x360 or `front_camera`; the `get_camera_image` docs row names `topics.head_camera`, Camera Module 3, 1152x648 and 384x216, with no 640x480 front size, 10 fps or overhead wording. |
| `test_web_ui_map_tab_uses_contract_fields` | The map tab carries no legacy keys (`urdf_file`, `topic`, `arm_urdf_file`, `arm_joint_topic`, `scan_topic`, `costmap_topic`), uses the frontend contract names, points `arm_home_service` / `arm_set_home_service` at the services mcp_server serves, and keeps the tile cache at `/var/cache/web_ui/tiles`. |
| `test_web_ui_tile_cache_dir_task_owned_by_node_user` | `playbooks/tasks/web_ui_tile_cache_dir.yml` creates `/var/cache/web_ui` and `/var/cache/web_ui/tiles` as directories owned by `ansible_user`. |
| `test_playbooks_create_tile_cache_before_deploying_web_ui` | `deploy_nodes_client.yml` includes the tile cache task before deploying web_ui. |
| `test_mcp_server_perception_topics_match_their_producers` | mcp_server `topics` `local_costmap` `/local_costmap/costmap`, `plan` `/plan` and `poi_list` / `poi_command` / `poi_result` equal the poi_store entry's topics. |
| `test_mcp_server_perception_config_sections` | mcp_server config carries `objects` (`/var/lib/ros2/objects/objects.json`, merge radius 0.25 m), `footprint` 0.47 x 0.386 m, `look_around` defaults (4 captures, positive clearance margin), `topdown` defaults and `poi.request_timeout_s` 3 s. |
| `test_mcp_server_readme_documents_perception_tools` | `nodes/mcp_server/README.md` documents get_topdown_view, the object memory tools, look_around, the POI tools, the objects file, the robot-up convention, `layers_missing` and the `MOTION_TOOLS` classification. |
| `test_mcp_server_readme_lookups_example_has_only_store_dir` | The README config reference `lookups:` example lists only `store_dir` (no removed `max_records`/`max_bytes`/`max_age_days`). |
| `test_client_yml_lookups_comment_points_to_an_existing_readme_section` | The README section quoted in the client.yml mcp_server `lookups:` comment exists as a `##` heading in `nodes/mcp_server/README.md`. |
| `test_mcp_server_setup_tasks_create_objects_dir_owned_by_node_user` | `tasks/mcp_server_setup.yml` creates `/var/lib/ros2/objects` owned by `ansible_user`. |
| `test_mcp_server_setup_tasks_create_lookups_dir_owned_by_node_user` | `tasks/mcp_server_setup.yml` creates `/var/lib/ros2/poi` and then `/var/lib/ros2/poi/lookups` (camera lookup store), both directories owned by `ansible_user`, mode `0755`. |
| `test_mcp_server_config_lazy_steer_states` | The client.yml mcp_server `control:` block sets the lazy `/swerve_drive/joint_states` subscription: `steer_states_wait_s` 0.5 (first-message wait) and `steer_states_idle_s` 30.0 (destroyed once unread that long). |
| `test_mcp_server_auto_poi_block_is_deployed` | The client.yml mcp_server config has an `auto_poi` block (enabled, `merge_radius_m` 0.5, event types collision_stop and nav_abort, `max_pose_age_s` 2.0, `hint_after_images` 3) that validates with `McpServerConfig`. |
| `test_mcp_server_config_renders_guard_rules_in_dry_run` | The client.yml mcp_server config renders `guard_rules:` with exactly the five rules (R1-R5 names, in order), every one `mode: dry_run`, and `motion_queue` `watch_rate_hz: 10` / `watch_max_per_call: 8` (YAML only; the node test validates the schema). |
| `test_base_guard_rules_cover_every_base_moving_tool` | Every guard rule that applies to a base tool applies to all of them: the base set is read from mcp_server `tools.py` `TOOL_SUBSYSTEM` (AST, no import), so a new base-moving tool such as `go_to_frontier` / `look_toward_frontier` cannot slip past the base rules. |
| `test_mcp_server_config_pins_auto_poi_block` | The client.yml mcp_server `auto_poi:` block pins every `AutoPoiSettings` key (enabled, merge_radius_m 0.5, max_pose_age_s 2.0, event_types collision_stop and nav_abort) and the config validates with `McpServerConfig`. |
| `test_mcp_server_config_lookups_block_is_only_the_store_dir` | The client.yml mcp_server `lookups:` block is exactly `{store_dir: /var/lib/ros2/poi/lookups}` (poi_store owns retention) and the config validates with `McpServerConfig`. |
| `test_client_yml_pins_the_agent_poi_settings` | The client.yml mcp_server `auto_poi:` block pins every `AutoPoiSettings` key (hint_after_images 3) and validates; claude_agent pins `poi_review_followup: true`, `poi_hint_radius_m: 0.5`, and has no `agent_poi_reset` key. |
| `test_mcp_server_maps_section_validates_and_matches_map_manager_topics` | The client.yml mcp_server `maps:` section validates with `McpServerConfig`, its command/result/status topics equal the `map_manager` entry's `command_topic` / `result_topic` / `status_topic` (read from its config block), and the timeouts are positive. |
| `test_mcp_server_readme_documents_the_named_map_tools` | `nodes/mcp_server/README.md` has tool-table rows for `save_map`, `list_maps`, `load_map`, `delete_map`, `clear_pois` and a `## Named maps` section. |
| `test_mcp_server_config_validates_with_frontiers_block_at_defaults` | The client.yml mcp_server config validates with `McpServerConfig`; its `frontiers:` block equals every `FrontierSettings` default and each key carries a comment. |
| `test_mcp_server_setup_tasks_create_token_and_arm_dir` | `tasks/mcp_server_setup.yml` creates `/etc/ros2/mcp_server`, the token (`MCP_SERVER_TOKEN=` + 48-char password lookup, `force: false`, 0640, owner `ansible_user`, group `mcp-token`, `no_log`) and `/var/lib/ros2/arm` owned by the node user. |
| `test_playbooks_run_setup_before_deploying_mcp_server` | `deploy_nodes_client.yml` deploys mcp_server and includes the setup tasks before it. |
| `test_unit_template_renders_environment_file_only_when_set` | The native unit template renders `EnvironmentFile=` (before `ExecStart=`) only when `node_environment_file` is non-empty. |
| `test_resolve_and_deploy_passes_environment_file` | `resolve_and_deploy.yml` passes the node type's `environment_file` (default empty) to the role. |
| `test_token_file_never_in_repo` | No tracked `token` file and no literal token value in tracked Ansible/scripts/node/test files; `.mcp.json` uses `${ROBOT_MCP_TOKEN}`. |
| `test_mcp_json_registers_robot_server` | `.mcp.json` registers server `robot` (type http, `http://client.ros2.lan:18200/mcp`, `Authorization: Bearer ${ROBOT_MCP_TOKEN}`) matching the node's port and path. |
| `test_robot_mcp_token_script` | `scripts/robot_mcp_token.sh` is executable bash (`set -euo pipefail`, passes `bash -n`), reads the token file over ssh and prints an export line. |
| `test_robot_mcp_token_script_prints_export_line` | With a fake `ssh` returning the EnvironmentFile line, the script prints `export ROBOT_MCP_TOKEN='<token>'`. |
| `test_mcp_server_node_package_layout` | `nodes/mcp_server` has a uv project (PEP 621) with the `mcp` dependency and `mcp_server` package, a lock file, `__main__.py`, and a README with the `claude mcp add` setup. |
| `test_mcp_server_listed_in_nodes_readme_index` | `nodes/README.md` Layout index has an `mcp_server/` entry describing the MCP server. |
| `test_mcp_gripper_closed_target_is_inside_the_follower_gripper_command_range` | The mcp_server closed gripper target (-0.165 rad = 1940 steps, follower gripper not inverted) is at or above the follower gripper `command_min_steps` (1900), `autonomy` is a direct command source, and the mcp_server README documents the steps and `command_min_steps`. |
| `test_mcp_server_sag_compensation_uses_the_held_out_fit` | mcp_server `arm.sag_compensation` is enabled with `k` (shoulder_lift, elbow_flex) and `k_lowering` (shoulder_lift) equal to `docs/calibration/2026-10-10/sag_fit_results.json` within 5e-4, each with a held-out reduction above 50%, `max_rad` in (0, 0.12]; `sag_fit.md` exists and the mcp_server README documents the compensation, `arm_settle` and `sag_fit.py`. |
| `test_mcp_server_unit_pins_the_allocator` | The mcp_server unit env pins glibc (`MALLOC_MMAP_THRESHOLD_=1048576`, `MALLOC_ARENA_MAX=2`) and `NUMPY_MADVISE_HUGEPAGE=0` so freed numpy temporaries return to the OS. |
| `test_mcp_server_client_config_validates_and_reads_follower_currents` | The client.yml mcp_server config loads through the node's `McpServerConfig` (unknown keys rejected) and `topics.follower_joint_currents` is `/follower/joint_currents`. |
| `test_mcp_server_grip_profiles_set_delta_contact_keys` | `grip_profiles` sets `contact_baseline_samples` 10, `contact_persist_samples` 3, a `current_contact_delta_a` key (null or positive) and `contact_load_delta` 20 (arXiv 2608.23983 initial value, retuned in rollout) on every preset. |
| `test_mcp_server_grip_presets_keep_absolute_backstops` | Delta contact is primary but every preset keeps `contact_effort_threshold` (above the delta) and `target_load` as absolute backstops. |
| `test_mcp_server_grasp_check_block` | The `grasp_check` block renders `crop_depth_m` 0.045, `crop_margin_px` 24, `crop_min_px` 64, `image_px` 640 and validates. |
| `test_follower_current_read_is_rolled_back` | The follower's `publish_current_joints` is `[]`: the longer Present_Current state read failed the 95 Hz gate on the robot (93.4 Hz, p99 18 ms), so the design's config rollback applies and the grasp band runs without the current vote. |
| `test_claude_agent_confirm_grasp_is_a_sensor_tool` | claude_agent `sensor_tools` include `confirm_grasp` and `effector_tools` do not. |
| `test_reach_map_artifact_key_matches_deployed_config` | The committed `nodes/mcp_server/mcp_server/data/reach_map.npz` exists, loads, and its key equals the key computed (in the mcp_server uv environment) from client.yml's mcp_server config; the failure names `scripts/gen_reach_map.py`. |
| `test_reach_map_artifact_grid_equals_configured_grid` | The artifact's meta grid (steps, radius and height counts, planar samples, samples per cell) equals client.yml's `reach_map:` block. |
| `test_reach_map_block_is_explicit_and_default_file` | client.yml spells out every `reach_map` grid field (no `file` override) and names `scripts/gen_reach_map.py`. |
| `test_gen_reach_map_check_passes_on_committed_artifact` | `scripts/gen_reach_map.py --check` exits 0 for the committed artifact. |
| `test_mcp_server_map3d_topics_match_the_topic_contract` | client.yml `topics.map3d_cloud` / `topics.map3d_status` equal the downlink dests of the map3d topic contract and the `map3d:` block spells out the defaults with `frame_id` equal to the contract's `map3d_frame`. |
| `test_mcp_server_readme_documents_map3d_tools` | The mcp_server README tool table has rows for `get_3d_map_status`, `query_3d_map_at` and `get_3d_free_space` and a `## 3D map` section (cloud topic, refresh period, measured vs lidar-only, unknown). |
| `test_every_test_is_documented_in_tests_readme` | Every `test_*` function in `test_mcp_server_config.py` is listed in this section. |

### test_battery_config.py

Static invariants of the battery voltage chain in Ansible `group_vars` (follower publisher, base bus off, web_ui command guard).

| Test | Description |
|------|-------------|
| `test_follower_publishes_battery_state` | `lerobot_follower` config sets `battery_topic: /battery_state`, `battery_interval_s: 1.0` and `battery_cells: 3`. |
| `test_web_ui_battery_matches_follower` | web_ui `battery.topic` and `battery.cells` equal the follower's `battery_topic` and `battery_cells`. |
| `test_web_ui_cutoff_is_8v4_for_three_cells` | web_ui `battery` block: cut-off 2.8 V/cell x 3 cells = 8.4 V, resume 2.9 V/cell (>= cut-off), `stale_s` 5.0. |
| `test_mcp_server_battery_matches_web_ui` | mcp_server `battery` block is identical to web_ui's (topic `/battery_state`, 3 cells, 2.8/2.9 V per cell, `stale_s` 5.0; 8.4 V cut-off). |
| `test_only_the_follower_bus_publishes_battery` | Of the client's present `feetech_servos` entries exactly one has `battery_interval_s > 0` (`lerobot_follower`, default 1.0 when unset), and `base_servos` sets `battery_interval_s: 0`, so `/battery_state` has one publisher. |
| `test_leader_does_not_publish_battery` | Server `lerobot_leader` has `battery_interval_s: 0`, so only the robot's follower bus publishes `/battery_state`. |
| `test_every_test_is_documented_in_tests_readme` | Every `test_*` function in `test_battery_config.py` is listed in this section. |

### `test_servo_bus_split_config.py`

Servo bus split in `ansible/group_vars/client.yml` and the client deploy playbook: the arm alone on `lerobot_follower`, the eight swerve servos on the second feetech_servos instance `base_servos` (design `docs/plans/in-progress/split-servo-buses-arm-alone/design.md` D1, D3, D4, D7, D8), and every base consumer wired to its `/swerve_drive/*` topics. Node `config: |` blocks are parsed as YAML.

| Test | Description |
|------|-------------|
| `test_follower_serves_only_the_arm` | `lerobot_follower` serves exactly IDs 1-6 and has no `extra_groups` and no `velocity_command_timeout_s`. |
| `test_follower_identity_device_and_rate` | `lerobot_follower` sets `node_name: lerobot_follower`, opens the arm adapter `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AE6081020-if00` (design A1) and runs at 100 Hz (`control_loop_hz` unset or 100). |
| `test_base_servos_entry_is_present_and_enabled` | `base_servos` is a present, enabled `feetech_servos` entry with `FEETECH_SERVOS_CONFIG=/etc/ros2/base_servos/config.yaml`. |
| `test_base_servos_carries_the_eight_swerve_joints` | `base_servos` uses namespace `swerve_drive` and serves exactly fl_drive 32 velocity inverted, fl_steer 33, fr_drive 35 velocity, fr_steer 34, rl_drive 39 velocity inverted, rl_steer 38, rr_drive 36 velocity, rr_steer 37 (in that order); no head group is configured. |
| `test_base_servos_runtime_settings` | `base_servos`: `node_name: base_servos`, the original adapter `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A059004-if00`, 50 Hz, battery off (0), metrics 19115, `velocity_command_timeout_s: 0.3`, torque on start, goal refresh 1.0 s, read failure limit 3, retry 5.0 s, register dump every 10 s. |
| `test_retired_swerve_drive_servos_name_is_gone` | No `ros2_nodes` entry and no client playbook step is named `swerve_drive_servos`; `Deploy base_servos` directly follows `Deploy lerobot_follower`. |
| `test_both_devices_are_distinct_by_id_paths` | Both bridges open a `/dev/serial/by-id/` path, and the two paths differ. |
| `test_present_feetech_instances_are_exactly_follower_and_base` | The client's present `feetech_servos` entries are exactly `lerobot_follower` and `base_servos`, with distinct `node_name` and `metrics_port` and disjoint servo IDs. |
| `test_swerve_controller_drives_the_base_joints` | `swerve_controller` `joint_names` equal the base joint names (same order) and its joint state/command topics are `/swerve_drive/joint_states` and `/swerve_drive/joint_commands`. |
| `test_tilt_guard_torque_reaches_the_base_bridge` | `swerve_controller` `tilt_guard.torque_topic` is `/swerve_drive/set_register`, so a `torque_off` reaches the drive servos. |
| `test_mcp_server_register_topics_follow_the_split` | mcp_server `topics.base_servo_registers` is `/swerve_drive/servo_registers`, `topics.servo_registers` `/follower/servo_registers`, `topics.follower_set_register` `/follower/set_register`. |
| `test_base_servos_gets_the_critical_cgroup_tier_from_type_defaults` | The `feetech_servos` type defaults are the critical tier (CPUWeight/IOWeight 1000, OOMScoreAdjust -900) and the `base_servos` entry overrides none of `cpu_weight`, `io_weight`, `oom_score_adjust`. |
| `test_bridge_entry_comments_carry_no_parking_wording` | The comments from the `lerobot_follower` entry through the `base_servos` entry contain no deferral wording (`later`, `next epic`, `follow-up`, `yet`, `todo`, `re-measure`). |
| `test_every_test_is_documented_in_tests_readme` | Every `test_*` function in `test_servo_bus_split_config.py` is listed in this section. |

### `test_serial_devices.py`

Serial adapter naming in `ansible/group_vars/client.yml` and the feetech_servos device preflight in `ansible/roles/ros2_node_deploy/tasks/main.yml` (design D2).

| Test | Description |
|------|-------------|
| `test_preflight_tasks_exist_in_the_right_place` | After the schema assert and before the first block/`include_tasks`, `[config]`-tagged tasks run only for present, enabled `feetech_servos` nodes: `ansible.builtin.stat` (following symlinks) on the config's `device`, a `changed_when: false` listing of the by-id directory, and a fail whose message names `/dev/serial/by-id`. |
| `test_every_serial_device_is_a_distinct_by_id_path` | Each client feetech `device` and `RPLIDAR_SERIAL_PORT` (the lidar's CP2102 by-id path) is a `/dev/serial/by-id/` path, and all are pairwise distinct. |
| `test_no_client_node_names_a_numbered_tty` | No client `ros2_nodes` config, env or `extra_args` names `/dev/ttyUSB*` or `/dev/ttyACM*`. |
| `test_every_test_is_documented_in_tests_readme` | Every `test_*` function in `test_serial_devices.py` is listed in this section. |

### test_gps_status_config.py

Static invariants of the GPS status wiring in Ansible `group_vars` (web_ui chips, rover and base gps_rtk nodes).

| Test | Description |
|------|-------------|
| `test_web_ui_base_url_is_base_caster_status_route` | web_ui `gps_status.base_url` is `http://{{ ros2_server_hostname }}:<gps_rtk_base rtcm_tcp_port>/status` (the base node's NTRIP caster), not the topic scraper port 18100. |
| `test_web_ui_rover_topic_is_rover_status_topic` | web_ui `gps_status.rover_topic` equals the rover's `status_topic`. |
| `test_rover_connects_to_base_caster_port` | The rover's `rtcm_server_port` equals the base's `rtcm_tcp_port`. |
| `test_rover_nmea_timeout_below_web_ui_stale_window` | Rover `nmea_timeout_s` is > 0 and, plus one status period, below web_ui `stale_after_s`, so "receiver silent" arrives before the chip turns stale. |

### test_claude_agent_config.py

Static checks of the claude_agent wiring (YAML, the unit template rendered with jinja2 when installed, the node package
layout; no ROS needed).

| Test (group) | Description |
|---|---|
| `test_claude_agent_node_type_defaults` | `claude_agent` node type: native, `nodes/claude_agent`, `python3 -m claude_agent`, no CPU quota (the CLI needs about 1.3 s of CPU per request), MemoryMax 1G, MemoryLow 96M, agent-facing tier, nice, user `claude_agent`, group `mcp-token`, `environment_file` `/etc/ros2/claude_agent/env`, `DISABLE_AUTOUPDATER=1`, no secret in `env`. |
| `test_claude_agent_entry_after_mcp_server_and_enabled` | Present + enabled `ros2_nodes` entry directly after `poi_store` (which follows `mcp_server`). |
| `test_claude_agent_config_valid_and_consistent_with_mcp_server` | The entry's config validates against the node's pydantic model, binds 127.0.0.1:18300, points at mcp_server's URL and token file, and every classified tool exists in `mcp_server/tools.py`. |
| `test_claude_agent_config_has_budget_maxima_and_robot_events_topic` | The client.yml config sets the hard maxima (rw 400 / turns 500), the per-phase maxima (100 / 100), no ro maxima and `robot_events_topic: /robot_events`, and no longer has `effector_call_cap` / `max_turns`; no `poi_command_topic` (the reset clears POIs over HTTP via mcp_server). |
| `test_claude_agent_config_pins_poi_behaviour` | The client.yml config has no `agent_poi_reset` key, sets `poi_review_followup: true` and `poi_hint_radius_m: 0.5` explicitly, and validates against `ClaudeAgentConfig`. |
| `test_claude_agent_config_pins_skills` | The client.yml config sets `skills_enabled: true` and `skills_plugin_dir: ""` and validates against `ClaudeAgentConfig`. |
| `test_mcp_tool_names_finds_async_tools` | The registered-tool scanner reads both `def` and `async def` functions decorated with `@tool` / `@server.tool`, so an async mcp_server tool cannot slip past the parity checks. |
| `test_every_registered_mcp_tool_is_classified_exactly_once` | Every tool mcp_server registers sits in exactly one of `effector_tools`, `uncapped_tools`, `sensor_tools` and `denied_tools`, and the lists name no unknown tool, in both the client.yml config and the code defaults. |
| `test_client_yml_claude_agent_config_equals_the_code_defaults` | Every key the client.yml claude_agent config sets equals the `ClaudeAgentConfig` default, so config.py stays the one source of the values (including `job_end_park_timeout_s` and `warning_interrupt_debounce_s`). |
| `test_default_effector_and_denied_tools_equal_mcp_server_motion_tools` | The code defaults `DEFAULT_EFFECTOR_TOOLS` plus `DEFAULT_DENIED_TOOLS` (no overlap) equal mcp_server `MOTION_TOOLS`, like the client.yml lists. |
| `test_effector_and_denied_tools_match_mcp_server_motion_tools` | `effector_tools` plus `denied_tools` (`[arm_set_home]`, no overlap) in the claude_agent config equal mcp_server `MOTION_TOOLS` exactly (parsed from tools.py with `ast`), so the effector cap, the gate deny list and the battery gate cover the same tools. Strict: `look_around` must be in `MOTION_TOOLS`. |
| `test_control_lease_tools_classified_in_group_vars_and_defaults` | `take_control` / `release_control` are uncapped (`acquire_control` is gone), `get_control_state` a sensor and `arm_set_home` denied, in both the client.yml config and the code defaults. |
| `test_round2_tools_classified_in_group_vars_and_defaults` | `look_around` is an effector and `get_body_state` plus the round-2 sensor tools (pixel_to_ground, annotated image, candidates, calibration, topdown view, objects, POIs) are sensors, in both the client.yml config and the code defaults. |
| `test_lookup_tools_are_sensor_tools_in_group_vars_and_defaults` | The camera lookup tools (`list_lookups`, `get_lookup_image`, `annotate_lookup`, `delete_photos`) exist in mcp_server and are sensor tools (never effector or uncapped) in both the client.yml config and the code defaults. |
| `test_motion_queue_tools_classified_in_group_vars_and_defaults` | The motion queue tools are classified in both the client.yml config and the code defaults: `enqueue_motions` is an effector, `cancel_motions` uncapped (like `stop`), `get_motion_status` and `wait_for_event` are sensors. |
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
| `test_claude_agent_package_layout_and_pinned_sdk` | uv project (PEP 621) with the SDK pinned to an exact version, FastAPI/uvicorn/Pillow/pydantic, lock file, README, `__main__.py`, tests. |
| `test_claude_agent_readme_documents_api_and_token_deploy` | The node README lists every API route, the four planning tools, the hard and per-phase maxima, the `/robot_events` topic and the `export CLAUDE_CODE_OAUTH_TOKEN` deploy command. |
| `test_docs_and_lint_scripts_list_claude_agent` | `nodes/README.md`, `ansible/README.md`, the ansible-deploy skill, `scripts/lint-all-nodes.sh` and root `lint-nodes` mention the node. |
| `test_claude_agent_hardening_in_group_vars_without_filesystem_protection` | The `claude_agent` node type sets `protect_proc: invisible`, `proc_subset: pid`, `no_new_privileges`, `private_tmp` and no ProtectSystem/ProtectHome (the bearer token is in the CLI child's argv). |
| `test_only_claude_agent_sets_hardening_in_group_vars` | No other node type in client.yml / server.yml sets any of the hardening keys, so their units are unchanged. |
| `test_resolve_and_deploy_passes_hardening_with_empty_defaults` | `resolve_and_deploy.yml` hands the four hardening keys to the role, each with a `default(...)`. |
| `test_role_defaults_keep_hardening_off` | `ros2_node_deploy` defaults leave all four hardening variables empty/false. |
| `test_service_template_hardening_is_optional` | Unit template (jinja2 when installed): no hardening lines by default or with empty values (identical output); the four lines and no ProtectSystem/ProtectHome when set. |
| `test_claude_agent_config_has_watchdog_and_sdk_initialize_timeout` | The entry's config sets `instruction_timeout_s: 900` (pre-plan watchdog), `max_instruction_time_min: 120` and the node env raises the SDK initialize timeout (`CLAUDE_CODE_STREAM_CLOSE_TIMEOUT=180000`). |
| `test_setup_tasks_create_persistent_workdir_owned_0750` | `claude_agent_setup.yml` creates `/var/lib/claude_agent/workspace` (the agent's persistent notes volume) as a directory owned by `claude_agent`, mode 0750, after its parent HOME directory. |
| `test_ansible_never_removes_the_workdir_or_state_dir` | No Ansible task (file `state: absent`, `rm` in command/shell) deletes anything under `/var/lib/claude_agent`, so notes and the session log survive deploys. |
| `test_claude_agent_config_workdir_state_dir_and_hardware_facts` | The entry's config sets `workdir`, `state_dir`, `arm_base_height_m` 0.100 (measured mount, equal to mcp_server) and `arm_reach_cm`; the service `HOME` stays `/var/lib/claude_agent`, separate from the workdir. |
| `test_frontier_tools_classified_in_group_vars_and_defaults` | `go_to_frontier` / `look_toward_frontier` are effectors (and in mcp_server MOTION_TOOLS), `list_frontiers` / `plan_frontier_visit` sensors, never uncapped, in the client.yml config and the code defaults. |
| `test_every_test_is_documented_in_tests_readme` | Every `test_*` function in `test_claude_agent_config.py` is listed in this section. |

### test_web_ui_agent_tab.py

Static checks of the web_ui Agent tab in `ansible/group_vars/client.yml` (no ROS needed).

| Test (group) | Description |
|---|---|
| `test_agent_tab_exists_with_type_and_label` | web_ui has a tab `agent` of type `agent_chat` labelled "Agent". |
| `test_agent_tab_directly_after_map_tab` | The Agent tab is listed immediately after the `map` tab. |
| `test_agent_tab_url_matches_claude_agent_port` | `agent_url` is 127.0.0.1 with the same port as the claude_agent `http_port` (18300). |
| `test_retros_tab_of_type_agent_retros_after_agent_tab` | web_ui has a tab `retros` of type `agent_retros` labelled "Retros", listed immediately after the Agent tab. |
| `test_every_test_is_documented_in_tests_readme` | Every `test_*` function in `test_web_ui_agent_tab.py` is listed in this section. |

### `test_head_camera_live_refs.py`

Repo-wide guard that the retired front camera left no live reference and that no doc or prompt states the old preview size.

| Test | What it covers |
|---|---|
| `test_no_live_front_camera_reference` | `git grep` over tracked files for the retired front camera topic, config key, topic setting and optical frame names finds nothing outside `docs/plans/**` and the retirement records (the retired pi_cameras tests, the pinned MEMORY.md decision lines, pinned by a retirement test, the absence assertions in `test_mcp_server_config.py` and the web_ui `subscription.test.ts`, the retired entry in `client.yml`, `MEMORY.md`, this test). |
| `test_no_640x480_head_or_front_preview_size` | The mcp_server tool descriptions, the claude_agent prompt and config, and the web_ui README never pair a front or head camera preview with 640x480 or 10 fps. |

### `test_head_live_refs.py`

Repo-wide guard that the pan/tilt head is live: no placeholder wording and no front camera pinned to `base_link`.

| Test | What it covers |
|---|---|
| `test_no_head_placeholder_text` | `git grep` over tracked files outside `docs/plans/**` finds none of the three placeholder phrases about the head being unconfigured, disabled or a later addition; the phrases are assembled from pieces in the test so it does not match itself. |
| `test_front_camera_not_pinned_to_base_link` | mcp_server has no `FRONT_CAMERA_PARENT = "base_link"` and no "front camera works in base_link" docstring claim. |
| `test_grep_tracked_fails_loudly_on_git_error` | The `grep_tracked` helper raises (instead of returning zero hits) when `git grep` fails, e.g. on an invalid regex, so the two guards cannot pass vacuously. |

### `test_head_camera_deploy.py`

Static invariants of the head camera deploy (`head_camera_cpp` in `ansible/group_vars/client.yml`): the libcamera source is the pinned dict of `pi_cameras` (same build stamp, nothing rebuilds), the patch is a byte-identical copy.

| Test | What it pins |
|---|---|
| `test_node_type_fields` | `deploy_mode: native`, launch command `ros2 run head_camera_cpp head_camera_node`, empty `node_src_dir` and `src_paths`. |
| `test_libcamera_source_is_the_pinned_dict_of_pi_cameras` | The first `colcon_source` equals the recorded libcamera dict (repo, commit, package, workspace, meson args, patch). |
| `test_patch_is_a_byte_identical_copy_of_the_pinned_one` | `nodes/head_camera_cpp/patches/libcamera-0001-add-package-xml.patch` has the recorded sha1 of the original patch. |
| `test_sources_build_libcamera_then_common_then_the_node` | Order libcamera, `ros2_common_cpp`, `head_camera_cpp`; the local sources carry cmake args, `base_paths`, `jobs 2` and `stamp_apt_packages`. |
| `test_no_apt_camera_stack_and_all_build_packages` | No `ros-jazzy-libcamera` / `ros-jazzy-camera-ros`; libcamera build packages and the package.xml dependencies are installed. |
| `test_background_tier` | `memory_low` 128M, `memory_max` 384M, weights 100, `oom_score_adjust` 0, `nice` 10. |
| `test_config_path_and_env` | `/etc/ros2/head_camera_cpp` and `HEAD_CAMERA_CONFIG`. |
| `test_entry_is_live_after_pi_cameras_with_valid_config` | The entry follows `pi_cameras`, is present and enabled, and its config keys are known to the node (`camera_id` of cam0, `jpeg_quality: 70`, `metrics_port: 19116`). |
| `test_monitoring_target_is_present` | `monitoring_agent_node_targets` scrapes `127.0.0.1:19116/metrics`. |
| `test_docs_describe_head_camera_cpp` | `nodes/head_camera_cpp/README.md` has `## Zoom interface` and `## Configuration`; the node README, `nodes/README.md`, `ansible/README.md`, the ansible-deploy SKILL and `MEMORY.md` name `head_camera_cpp`; `nodes/README.md` and the SKILL mark `pi_cameras` retired. |
| `test_client_playbook_deploys_head_camera_cpp_right_after_pi_cameras` | `deploy_nodes_client.yml` has a `Deploy head_camera_cpp` step directly after `Deploy pi_cameras` (resolve_and_deploy, apply tag and `_deploy_node_name` `head_camera_cpp`, tags incl. apt/python/build/config). |

### `test_ground_docs.py`

Keeps the ground estimation docs consistent with the shipped config and the repo (runbook `docs/runbooks/ground-estimation.md`, MEMORY.md, `docs/diagrams/topic_flow.puml`, this file).

| Test | What it covers |
|---|---|
| `test_runbook_has_section` | Each required runbook section exists and is not empty: Preconditions, Deploy, Topic checks, Level floor, Scenes, CPU and memory, Build and CI validation, Threshold validation, Switch-over checklist, Results. |
| `test_runbook_thresholds_equal_the_design` | 0.10 core CPU budget, 1 cm plane height and 1 deg slope limits are stated where the design puts them. |
| `test_threshold_validation_defaults_equal_the_shipped_config` | The Threshold validation table equals `ground_estimator.yaml` (fit, wheel radius, max slope, step height, colour) and the costmap_zones warning percent and mask value 40. |
| `test_runbook_scripts_exist` | Every `scripts/...` path named in the runbook is tracked by git. |
| `test_runbook_ground_topics_are_node_outputs` | Every `/ground/` topic in the runbook is a ground_estimator_cpp output under the configured namespace and is documented in the node README. |
| `test_runbook_pins_the_rates_and_the_deploy_command` | The 10 / 1 / 2 Hz rates, TF drop and head joint_states records, the exact deploy command and the mcp_server session check. |
| `test_runbook_scenes_cover_both_sides_of_the_rule` | Sills and drops of 3 and 5 cm, the 10 cm drop, box, ramps, red sheet, 0.04 m, 8 deg, CliffStop, the runtime layer toggles. |
| `test_memory_has_the_decision` | MEMORY.md has the Ground plane entry with its key terms. |
| `test_topic_flow_names_the_node_and_the_ground_topics` | topic_flow.puml names ground_estimator_cpp, its three inputs and the four /ground topics. |
| `test_tests_readme_lists_every_ground_test_file` | This file lists every `tests/test_*ground*.py` and the ground tests of the other suites. |

### Ground estimation epic: other tests

- Root tests: `test_ground_estimator_readme.py`, `test_ground_estimator_deploy.py` and `test_nav_ground_voxel_layer.py` (described in their sections); changed: `test_nav_stack_config.py` (ground layers in the local costmap only), `test_nav_tof_voxel_layer.py` and `test_nav_tof_collision.py` (the fixed bands and the collision monitor stay unchanged), `test_tof_obstacle_wiring.py` (ground topics), `test_monitoring_dashboards.py` (ground_estimator_cpp panel), `test_node_cpu_compare.py` (the `nav2_bringup_ground_layer` entry), `test_node_metrics_wiring.py`, `test_cpp_node_deploy.py`, `test_unit_cgroups.py`.
- ground_estimator_cpp gtests (`nodes/ground_estimator_cpp/test`): `test_plane_fit.cpp` (`PlaneFit.*`: level, pitched, wall, table, box, depth bias, deterministic seed), `test_tracker.cpp` (`GravityUp.*`, `ChoosePrior.*`, `Tracker.*`, `Slope.*`, `Angles.*`: IMU prior, hold, motion in odom, smoothing, slope levels), `test_segmentation.cpp` (`Segmentation.*`: bands, box, drop-off, 3 cm sill, range, overhang, cloud round trip), `test_edges.cpp` (`Edges.*`: drop-off, step plate, ramp, box, hull, clustering cap), `test_traversability.cpp` (`Traversability.*`: thresholds are warnings, sill/drop/ramp both sides, box, colour sheet and plate), `test_colour.cpp` (`Colour.*`: Lab, red sheet, floor model, labelled message layout), `test_config.cpp` (`Config.*`: defaults, derived topics, one third max step, shipped yaml, rejected keys), `test_state.cpp` (`GroundState.*`, `WarningAreas.*`: key order, shapes, rounding, fixtures), `test_node.cpp` (`NodeTest.*`: level floor with box, 10 cm drop, 3 cm sill, 6 deg ramp, missing transform, plane hold, IMU prior).
- costmap_zones: `nodes/costmap_zones/tests/test_ground_warnings.py` (warning areas to the speed and ground cost masks) and `test_node.py`.
- mcp_server: `nodes/mcp_server/tests/test_ground_summary.py` (`/ground/state` contract, region conversion, summary mapping), `test_floor_guard_ground.py` (opt-in floor_guard with the ground plane).
- web_ui: `nodes/web_ui/tests/test_ground_layer.py` and `test_obstacle_points.py` (backend topics and layer config); frontend `groundLayer.test.ts` and `groundMath.test.ts` (Ground layer cloud decode, plane patch, labels).

### `test_ground_estimator_readme.py`

Keeps `nodes/ground_estimator_cpp/README.md` in sync with the node. Parses `STATE_KEYS` from `state.hpp`, the `ground_*` metric strings from `ground_node.cpp` and `ground_node.hpp`, and the shipped `config/ground_estimator.yaml` (nested keys dotted).

| Test | What it covers |
|---|---|
| `test_readme_has_every_required_section` | Topics, State, Labels, Obstacle or warning, Configuration, Metrics, Plane tracking and No placeholder data headings exist. |
| `test_readme_names_every_topic` | The three input topics and the five `/ground/` outputs appear, with base_link and plane-relative bands stated. |
| `test_readme_names_every_config_key` | Every key of the shipped yaml appears in the README. |
| `test_readme_names_every_state_key` | Every `STATE_KEYS` entry appears in the README. |
| `test_readme_names_every_metric` | Every `ground_*` metric name of the node sources appears in the README. |
| `test_readme_documents_labels_and_the_obstacle_rule` | Label values 0..4 are tabled and the obstacle-or-warning rule names one third of the wheel diameter, `max_slope_deg`, costmap_zones and the slow-down. |

### `test_ground_estimator_deploy.py`

Static invariants of the `ground_estimator_cpp` deploy: node type keys, apt list, normal tier values, env, colcon source order, `ros2_nodes` entry (equal to the shipped yaml plus `metrics_port` 19119), metrics scrape target and playbook step order. Consistency (design D3): topics equal the laser_filter, bno055_imu and head_rgbd_cpp outputs; the IMU level reference and slope limits equal tilt_guard; `wheel_radius_m` equals swerve_controller and the max step is one third of the wheel diameter; `max_slope_deg` is at most the smaller tilt_guard warn limit.

### `test_head_rgbd_deploy.py`

Static invariants of the head RGB-D fusion deploy (`head_rgbd_cpp` in `ansible/group_vars/client.yml`, the `config_files` task of the `ros2_node_deploy` role, `deploy_nodes_client.yml`).

| Test | What it pins |
|---|---|
| `test_node_type_keys` | Native, launch command `ros2 run head_rgbd_cpp head_rgbd_node`, empty `node_src_dir` and `src_paths`, no `ROS_STATIC_PEERS`. |
| `test_apt_packages` | The build and runtime apt list, incl. `libopencv-contrib-dev` and `ros-jazzy-depth-image-proc`. |
| `test_background_tier_values` | Weights 100, `oom_score_adjust` 0, `memory_low` 48M, `memory_max` 256M. |
| `test_config_path_and_env` | `/etc/ros2/head_rgbd_cpp` and `HEAD_RGBD_CONFIG`. |
| `test_colcon_sources_order_and_build` | `ros2_common_cpp` then `head_rgbd_cpp`, Release, `BUILD_TESTING=OFF`, `-j2`, `base_paths`, `stamp_apt_packages` installed. |
| `test_tof_calibration_file_is_deployed_when_present` | `config_files` maps `tof_calibration.yaml` into the config dir; the role copies existing repo files with `remote_src`. |
| `test_ros2_nodes_entry` | Present and enabled, after `head_camera_cpp` and `arducam_tof`; the config block equals the node defaults and carries comments. |
| `test_metrics_target_is_present` | Alloy scrapes `127.0.0.1:19117/metrics`. |
| `test_head_camera_mapping_and_focus_lock` | `lens_position: 1.0`, the `mapping` block (2 Hz, 576x324, JPEG 80, `lock_focus`), `af_mode` continuous. |
| `test_arducam_tof_stamp_latency` | The `arducam_tof` block sets `stamp_latency_ms: 0.0`. |
| `test_client_playbook_step_order` | `Deploy head_rgbd_cpp` after the head_camera_cpp and arducam_tof steps, tags `[head_rgbd_cpp, apt, python, build, config]`. |

### `test_pi_camera_boot_config.py`

Static checks of `ansible/playbooks/tasks/pi_camera_boot_config.yml` (IMX708 on CSI cam0, Arducam ToF on cam1).

| Test | What it pins |
|---|---|
| `test_retired_imx219_overlays_are_removed` | `dtoverlay=imx219,cam0` and `dtoverlay=imx219,cam1` are removed with `state: absent`. |
| `test_auto_detect_is_off_and_both_overlays_are_written` | Exactly `camera_auto_detect=0`, `dtoverlay=imx708,cam0` and `dtoverlay=arducam-pivariety,cam1` are written. |
| `test_every_line_task_is_aarch64_only_and_registers_its_result` | Five `lineinfile` tasks, each aarch64-only, tagged `boot`, with its own registered result. |
| `test_client_reboots_only_when_a_line_changed` | The last task reboots (timeout 120) when any registered result changed. |
| `test_client_playbook_includes_the_boot_config_under_both_camera_node_tags` | `pre_tasks` includes the file with tags `boot`, `pi_cameras`, `arducam_tof`, `head_camera_cpp` (also on `apply`). |
| `test_ansible_readme_describes_the_boot_config` | `ansible/README.md` names the file and the three written lines. |
| `test_tests_readme_lists_every_test_of_this_file` | Every test of this file is listed here. |

### `test_pi_cameras_retired.py`

Static checks of the two Pi camera retirements (D7 the overview camera, D9 `pi_cameras`), read as YAML and text.

| Test | What it pins |
|---|---|
| `test_overview_camera_entry_is_uninstalled_by_the_next_deploy` | The `overview_camera` ros2_nodes entry is `present: false` with no config. |
| `test_overview_camera_entry_names_d7_in_its_comment` | The comment above the entry names D7. |
| `test_overview_camera_node_type_keeps_only_what_the_uninstall_path_needs` | The node type has only `deploy_mode: native` and a launch command: no source, build, tier or env keys. |
| `test_pi_cameras_entry_is_uninstalled_by_the_next_deploy` | The `pi_cameras` ros2_nodes entry is `present: false` with no config. |
| `test_pi_cameras_entry_names_d9_in_its_comment` | The comment above the entry names D9. |
| `test_pi_cameras_node_type_keeps_only_launch_command_and_camera_ros_purge` | The node type keys are exactly `deploy_mode`, `node_launch_command`, `colcon_purge`, and the purge list names `camera_ros` only (libcamera stays). |
| `test_only_head_camera_cpp_builds_libcamera_and_nobody_builds_camera_ros` | Only `head_camera_cpp` builds libcamera; no node type builds camera_ros. |
| `test_pi_cameras_sources_and_old_tests_are_deleted` | `nodes/pi_cameras`, the two old pi_cameras test files do not exist. |
| `test_client_playbook_still_deploys_pi_cameras_so_the_uninstall_runs` | The playbook still deploys the name so the uninstall runs. |
| `test_old_boot_file_and_old_tests_are_deleted` | `overview_camera_boot_config.yml`, `pi_camera_overlays_removed.yml` and the two `test_overview_camera*.py` files are gone. |
| `test_client_playbook_still_deploys_overview_camera_so_the_uninstall_runs` | The playbook still deploys the name so the uninstall runs. |
| `test_ansible_readme_describes_the_removal` | `ansible/README.md` names neither removed boot file and has no memory row for `overview_camera`. |
| `test_tests_readme_lists_every_test_of_this_file` | Every test of this file is listed here. |

### test_ansible_deploy_speed.py

Deploy speed work: stamps instead of always-run builds, queued restarts, batched apt, tags on every task and one tag-filtered playbook run per deploy. The probe scripts of the role are run for real against temporary git repos.

| Test | What it checks |
|---|---|
| `test_ansible_cfg_enables_timing_pipelining_and_connection_reuse` | `ansible.cfg` enables `profile_tasks` + `timer`, pipelining, `ControlMaster=auto` / `ControlPersist`; `requirements.yml` lists `ansible.posix`. |
| `test_ansible_cfg_gathers_minimal_facts_and_caches_them` | `gather_subset = min`, smart gathering, jsonfile fact cache (cache dir gitignored). |
| `test_documented_tag_set_lists_every_phase_tag` | The "Deploy tags" table in `ansible/README.md` lists exactly sync, apt, python, build, config, boot, setup, restart, verify, always. |
| `test_every_task_in_role_and_task_files_carries_a_documented_tag` | (parametrized per file) every task (block tags inherited) in the role, the verify role and `playbooks/tasks/*.yml` has a documented tag. |
| `test_non_node_targets_are_parsed` | `non_node_targets` reads `monitoring` and `jetson_link` for the client and `monitoring` for the server from `scripts/deploy-nodes.sh`. |
| `test_every_task_in_the_deploy_playbooks_carries_a_documented_or_node_tag` | (client, server) every pre/main/post task of the deploy playbooks has a documented, node-name or non-node-target (`monitoring`) tag. |
| `test_every_node_has_a_tagged_deploy_step` | (client, server) each `ros2_nodes` entry has exactly one deploy include tagged with its name plus apt/python/build/config, applying the node tag to the included tasks. |
| `test_node_specific_setup_files_carry_their_node_tag` | The mcp_server token, claude_agent token, poi_store, slam maps, web_ui tile cache and Pi camera boot config (`arducam_tof`) task files carry their node tag. |
| `test_shared_steps_run_under_any_node_filter` | (client, server) select_run, repo sync, ROS package sync, batched apt and the gradual restart are `always`-tagged; the verify role too; the apt steps are conditioned on `ros2_run_apt`. |
| `test_deploy_script_runs_one_playbook_with_the_joined_node_tags` | `deploy-nodes.sh client web_ui mcp_server` calls ansible-playbook once with `--tags web_ui,mcp_server` (fake ansible-playbook on PATH). |
| `test_deploy_script_all_passes_tags_and_other_options_through` | `--all` adds no tag filter by itself and passes `--tags` / `--skip-tags` through. |
| `test_deploy_script_rejects_the_jetson_target` | Target `jetson` exits non-zero with "Use: client or server" and never calls ansible-playbook. |
| `test_deploy_script_uses_no_associative_arrays` | Bash 3.2: no `declare -A`. |
| `test_provision_plays_target_server_and_client_only` | site.yml and optimize.yml Pi-role plays have hosts `server:client` (parametrized). |
| `test_no_play_with_pi_roles_has_hosts_all` | No play anywhere with common/network/hostname/system_optimize targets `all`. |
| `test_deploy_script_rejects_unknown_nodes_and_node_list_with_tags` | Unknown node names are rejected with the valid list; a node list plus `--tags` is rejected; `playbooks/nodes/` no longer exists. |
| `test_role_has_no_always_changed_tasks_and_does_not_restart_or_start_nodes` | No `changed_when: true` in the role; no inline start/restart; the "Restart ROS2 node" handler only queues into `ros2_nodes_pending_restart`. |
| `test_uv_sync_runs_only_for_a_new_dependency_hash_and_stamps_after_success` | `uv sync --frozen --no-dev` runs only when the dependency hash is stale and writes `.uv-deps` after success; the probe covers pyproject, `uv.lock`, shared metadata and tree hashes; the source stamp notifies the restart. |
| `test_ros2_base_includes_the_uv_task_file` | `ros2_base` installs uv through the shared `tasks/uv.yml` (pinned version, pipx, idempotent). |
| `test_client_deploy_waits_for_an_idle_agent_before_anything_can_stop_a_unit` | the client playbook includes `tasks/agent_idle_guard.yml` as its first `always`-tagged pre_task, before the deploy lock; the task polls `/api/state` with `uri` and honours `ros2_deploy_ignore_agent`; `group_vars/all.yml` has the defaults (5 min quiet, 15 min wait, 15 s poll); `deploy-nodes.sh` documents the override. |
| `test_deploy_playbook_ensures_the_pinned_uv_before_any_node` | (client, server) the deploy playbook's `pre_tasks` include `ros2_base` `tasks_from: uv.yml` tagged `always`: `deploy-nodes.sh` never runs `ros2_base`, and a host provisioned before uv has none. |
| `test_web_ui_npm_steps_run_only_when_their_inputs_changed` | `npm ci` / `npm run build` are conditioned on the frontend probe, copy and stamps follow only a build, stamp written last. |
| `test_dependency_probe_detects_source_dependency_and_shared_changes` | Runs the probe in a temp repo: new venv is stale; matching stamp is not; a source commit changes only `src_key`; `shared/` source restarts dependents without reinstall; shared metadata or `uv.lock` change the dependency hash; a stale `.poetry-deps` stamp never skips `uv sync`. |
| `test_web_ui_probe_builds_only_when_frontend_inputs_change` | Runs the web_ui probe: first run needs ci + build; with stamps and outputs neither; a source commit builds without ci; a lock change runs ci; missing static output rebuilds. |
| `test_heavy_steps_stop_the_nodes_only_when_they_will_run` | The stop-before-build include is conditional (uv sync stale, web_ui build, colcon stamp missing for a pinned, not local, source), runs once per play, and no playbook stops nodes up front. |
| `test_colcon_clone_patch_and_build_are_skipped_when_the_stamp_exists` | Clone, patch and build only when the stamp is missing (clone and patch only for pinned sources); the stamp check comes before the clone. |
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


## Simulation harness tests (not part of this suite)

The dev-only MuJoCo grasp replay harness has its own uv project and tests under `sim/grasp_sim/tests/` (run with `cd sim/grasp_sim && uv run pytest -q`, see [sim/README.md](../sim/README.md)). They are not collected by the root `pytest tests/` run.

| File | Covers |
|------|--------|
| `test_model.py` | Vendored SO-101 model loads with six position actuators; every body frame and the URDF gripper frame match a numpy URDF FK chain (`so101_arm.urdf`) within 2 mm at six joint configs; shoulder_lift range override. |
| `test_scene.py` | Scene generation: floor at `-base_height` (default 0.100, the measured mount), box on floor / ledge / lower stair settles on its support with the configured size, mass and friction; support kind inference; validation; `gap_below_m` rests the box on two rails (support geoms) with a clear slot; an upright cylinder object (the 39 mm x 6 cm jar) stands on the floor with its diameter, height and mass, and needs equal x / y sizes. |
| `test_tcp.py` | Jaw calibration: gripper_frame_link transform, the stock Menagerie closing point, `tool_offset_m` read from client.yml, calibrated jaws close within 2 mm of it, only the fingers and the moving jaw move, the vendored XML stays unmodified, the replay grip point follows; the moving jaw inner silhouette (`moving_jaw_inner_profile`) meets the fixed jaw at the tips and flares more than 1 cm away 4 cm in, and mcp_server's `jaw_profile.MOVING_JAW_INNER_PROFILE` equals it (0.2 mm). |
| `test_matrix.py` | Scenario matrix: executor-like quintic path timing, phase order and closed gripper from the close, slower lift for a lower `speed_scale`, scene per entry (calibrated or stock jaws, rails for a gap, the stair edge at the entry's `support_edge_x`), summary table, `grasp-sim matrix` end to end (with the descent tilt / push fields); the box faces the index's shoulder pan axis; an entry's shape, mass and `sim_offset_xy_m` reach the scene; `descent_motion` is the worst tilt / push of the approach and grasp segments. |
| `test_plan.py` | Replay plan parsing, label timeline and linear interpolation. |
| `test_ik.py` | Harness IK reaches position and approach pitch, raises when unreachable, respects limits and seeds. |
| `test_replay.py` | `simulate`: trivial hold, clearance, forces, arm/jaw floor contacts, joint clipping warning, determinism, contact classification, tilt metric. |
| `test_examples.py` | Passing floor/ledge/stair scoops lift the object; the tipping scoop fails with tipped/pushed/unintended contact; open-gripper plan never reports a grasp. |
| `test_adapter.py` | Tolerant GraspPlan to replay conversion (aliases, units, relative times, error cases). |
| `test_report.py` | SimReport schema and JSON round trip. |
| `test_cli.py` | `grasp-sim example/run`, exit codes, report file, YAML scene, frames and GIF output, mjpython hint for `--render`. |

### `test_servo_bus_live_refs.py`

Pins the arm-alone bus split: stale text and the retired group name stay out of live files, and servo IDs stay on their own node.

| Test | What it covers |
|------|----------------|
| `test_no_live_text_puts_swerve_servos_on_the_arm_bus` | `git grep` finds none of the stale arm-bus phrases outside `docs/plans/**` and `graphify-out/**`. |
| `test_retired_swerve_drive_servos_name_is_gone` | `swerve_drive_servos` appears nowhere outside `docs/plans/**`, `graphify-out/**`, `tests/**` and `MEMORY.md`. |
| `test_no_feetech_entry_but_the_base_serves_swerve_ids` | In client.yml only `base_servos` lists IDs 32-39 and only `lerobot_follower` lists IDs 1-6. |

### `test_joint_state_throttle_config.py`

The 10 Hz joint-state copies (`ros-jazzy-topic-tools` `throttle messages`) deployed on the client.

| Test | Description |
|------|-------------|
| `test_mcp_server_topic_defaults_are_the_throttled_sources` | The throttled sources are exactly the joint-state topics mcp_server's config subscribes to. |
| `test_each_throttle_is_an_apt_topic_tools_throttle_with_exact_source_rate_dest` | Per node type: apt package `ros-jazzy-topic-tools`, command `throttle messages <topic> 10.0 <topic>/throttled`, node name remap, no repo source. |
| `test_each_throttle_is_a_present_enabled_node_in_the_background_tier` | One present, enabled `ros2_nodes` entry per topic with the background tier weights and memory limits. |
| `test_each_throttle_has_a_deploy_step_in_the_client_playbook` | The client playbook has a deploy step and tags for each throttle node. |

### `test_node_metrics_wiring.py`

Node metrics wiring in `ansible/group_vars/client.yml` against the port contract, and the robot's Alloy node targets (`monitoring_agent_node_targets` in `group_vars/client.yml`). Node `config: |` blocks are rendered with jinja2 and parsed as YAML; env lists merge type and node entries as `resolve_and_deploy.yml` does.

| Test | Description |
|------|-------------|
| `test_config_metrics_port` | (lerobot_follower, filter_node, bno055_imu, gps_rtk_rover, swerve_controller, rf2o_odom_relay, poi_store, map_manager, map_guard, costmap_zones, arducam_tof, the side-by-side C++ port map_guard_cpp 19110, and the standalone C++ nodes head_camera_cpp 19116 and head_rgbd_cpp 19117) `metrics_port` 19101-19112; base_servos 19115; marker_detector 19118 (added by the marker epic, which also lists `marker_detector` among the `src_shared` node types of `test_node_types_using_ros2_metrics_declare_src_shared`). |
| `test_env_metrics_port` | gripper_uvc_camera `METRICS_PORT=19105`, rplidar_a1 `METRICS_PORT=19106`. |
| `test_http_nodes_keep_their_port` | web_ui 8080, test_joint_api 18080, topic_scraper_api 18100, mcp_server 18200, claude_agent 18300, without a `metrics_port` key. |
| `test_rplidar_imports_ros2_metrics_without_a_venv` | rplidar_a1 (no venv): `python3-prometheus-client` apt package, `shared/ros2_metrics` in `src_extra_paths`, `PYTHONPATH` starting with `<ros2_repo_dest>/shared/ros2_metrics`. |
| `test_node_types_using_ros2_metrics_declare_src_shared` | Every venv node type that imports ros2_metrics sets `src_shared: true`. |
| `test_server_group_vars_untouched` | No metrics port in `group_vars/server.yml`; the server's and the agent defaults' node target lists are empty. |
| `test_a_port_is_shared_only_by_a_python_and_cpp_pair` | Two contract nodes share a port only as `<name>`/`<name>_cpp` (at least one such pair exists). |
| `test_a_python_and_cpp_pair_is_never_both_enabled` | Of every `<name>`/`<name>_cpp` pair in `ros2_nodes` (map_guard at least) at most one is present and enabled: both would bind the same port and publish the same topics. |
| `test_every_enabled_contract_node_has_a_matching_scrape_target` | Each enabled contract node (count derived from the port contract minus disabled nodes) has a target `127.0.0.1:<configured port>` with path `/metrics`, and no other targets exist. Disabled nodes other than topic_scraper_api are only members of a Python/C++ pair. |

### `monitoring_helpers.py`

Not a test file: shared helpers of the monitoring tests. Role paths (`roles/monitoring_agent`, `roles/monitoring`, `roles/web_proxy`), per-host variables (every monitoring role's defaults with the `monitoring_*` / `web_proxy_*` keys of `group_vars/<host>.yml` on top), jinja2 rendering of role templates and task values like `ansible.builtin.template` (StrictUndefined, `trim_blocks`), task walking through blocks, systemd unit parsing, the playbooks' `include_role` tasks and a docker image check.

### `test_monitoring_role.py`

The server stack `ansible/roles/monitoring` (Mimir + Loki + Grafana on the server RPi4) and the deploy integration of all monitoring roles. Templates are rendered with the server's variables.

| Test | Description |
|------|-------------|
| `test_role_files_exist` | tasks, defaults, handlers, meta, the Mimir, Loki and Grafana templates, the drop-in and the dashboards exist. |
| `test_robot_only_pieces_left_the_stack_role` | The Prometheus, Alloy, slice, nginx templates and the throttling script are no longer in the stack role. |
| `test_every_template_and_file_the_tasks_name_exists` | Every `src:` of a template/copy task (loops and `with_fileglob` resolved) is shipped with the role. |
| `test_defaults_have_enable_flag_ports_retention_and_limits` | `monitoring_enabled: true`, ports 3000/9009/3100, 24h Mimir and Loki retention, 5s interval, per-unit caps mimir 700M, loki 400M, grafana-server 768M. |
| `test_packages_from_the_grafana_apt_repo` | The role includes `monitoring_agent`'s `grafana_repo.yml` before one apt task installing mimir, loki and grafana without recommends. |
| `test_data_directories_belong_to_the_service_users` | `/var/lib/mimir` belongs to mimir, `/var/lib/loki` to loki. |
| `test_dropin_moves_unit_into_the_slice` | (mimir, loki, grafana-server) drop-in sets Slice=monitoring.slice, Nice=10, the unit's MemoryMax, OOMScoreAdjust=500. |
| `test_dropins_are_installed_for_every_unit` | One template task loops `monitoring_units` into `/etc/systemd/system/<unit>.service.d/`. |
| `test_mimir_is_one_monolithic_single_tenant_process` | Mimir: `target: all`, `multitenancy_enabled: false`, HTTP `0.0.0.0:9009`, gRPC on 127.0.0.1, usage stats off. |
| `test_mimir_stores_blocks_on_the_local_filesystem` | Filesystem backend; data, TSDB, sync and compactor dirs under `/var/lib/mimir/`. |
| `test_mimir_rings_are_in_memory_on_localhost` | Replication factor 1; distributor, ingester, querier, compactor, store-gateway, ruler and query-scheduler rings `inmemory` on 127.0.0.1; frontend address and memberlist bound to 127.0.0.1. |
| `test_mimir_keeps_one_day_and_accepts_both_hosts` | `compactor_blocks_retention_period: 24h`; series, ingestion rate and burst limits cover ~10k series at 5 s from two hosts with headroom. |
| `test_mimir_is_sized_for_the_pi4` | Ingester `instance_limits.max_series` bounded, querier `max_concurrent` at most 4. |
| `test_mimir_service_flags` | `/etc/default/mimir`: `LOG_LEVEL="warn"`, `RESTART_ON_UPGRADE="true"`; the config goes to `/etc/mimir/config.yml`. |
| `test_loki_is_monolithic_without_auth` | Loki: `auth_enabled: false`, HTTP `0.0.0.0:3100`, gRPC on 127.0.0.1, one replica, `inmemory` ring on 127.0.0.1, analytics off. |
| `test_loki_stores_tsdb_v13_on_the_filesystem` | `path_prefix /var/lib/loki`, filesystem chunks, one tsdb/filesystem/v13 schema, WAL under `/var/lib/loki/`. |
| `test_loki_deletes_logs_after_one_day` | `retention_period: 24h`, compactor `retention_enabled`, `delete_request_store: filesystem`, working dir under `/var/lib/loki/`. |
| `test_loki_config_is_verified_before_it_is_installed` | The template task validates with `/usr/bin/loki -config.file=%s -verify-config`. |
| `test_grafana_behind_nginx_at_subpath_without_login` | grafana.ini: `127.0.0.1:3000`, `root_url` ending `/grafana/`, `serve_from_sub_path`, anonymous enabled with role Editor. |
| `test_grafana_phones_nowhere` | Reporting, update checks, plugin update checks, gravatar and news are off. |
| `test_grafana_runs_only_the_prometheus_and_loki_backend_plugins` | `disable_plugins` lists exactly the 11 other bundled datasources (neither prometheus nor loki), `preinstall_disabled = true`. |
| `test_grafana_admin_password_from_host_file` | The drop-in loads `/etc/grafana/admin-password` via LoadCredential into `GF_SECURITY_ADMIN_PASSWORD__FILE`; grafana.ini holds no password. |
| `test_grafana_admin_password_generated_on_the_host` | No Ansible password lookup; one `openssl rand` task with `creates:`; the file is 0600 root:grafana. |
| `test_grafana_admin_password_command_runs` | The password task's real command, run locally, writes a 48-hex-character file with mode 0600. |
| `test_grafana_datasources_mimir_and_loki` | `/etc/grafana/provisioning/datasources/robot.yaml`: `prometheus-robot` (prometheus type, `http://127.0.0.1:9009/prometheus`, default, `prometheusType: Mimir`, 5s) and `loki-robot` (`http://127.0.0.1:3100`). |
| `test_grafana_dashboard_provider` | Folder Robot; one copy task installs every `files/*.json` (`with_fileglob`) into the provider path under its own name. |
| `test_services_restart_when_a_config_is_newer_than_their_start` | After starting, a shell task restarts mimir / loki / grafana-server when their config, drop-in or `monitoring.slice` is newer than the start; it runs between the start and the readiness wait. |
| `test_readiness_checks` | Waits for Mimir `/ready`, Loki `/ready`, Grafana `/api/health` and `/grafana/api/health` through nginx, at least 2 min. |
| `test_disabled_stack_stops_its_services` | `monitoring_enabled: false` stops and disables mimir, loki and grafana-server. |
| `test_playbooks_include_the_host_roles_tagged_monitoring_only` | (client, server) the client includes `monitoring_agent` and `web_proxy`, the server `monitoring_agent`, `web_proxy`, `monitoring` in that order, each tagged and applied `[monitoring]` only. |
| `test_robot_no_longer_includes_the_stack_role` | The client playbook does not include `monitoring`. |
| `test_monitoring_only_run_starts_and_verifies_no_nodes` | (client, server) `select_run.yml` with the host's `ros2_nodes`: `--tags monitoring` gives an empty node scope; full, phase and node runs keep theirs. |
| `test_deploy_script_accepts_monitoring` | (client, server) `deploy-nodes.sh <host> monitoring` (and with a node name) runs the host's playbook with `--tags monitoring`. |
| `test_deploy_script_lists_monitoring_on_unknown_name` | (client, server) an unknown name fails and the error lists `monitoring` next to the nodes. |

### `test_monitoring_agent_role.py`

`ansible/roles/monitoring_agent` (Grafana Alloy, `monitoring.slice`, the RPi throttling timer and the robot cleanup), rendered for each host (client, server). The throttling script runs against a fake `vcgencmd` on PATH.

| Test | Description |
|------|-------------|
| `test_role_files_exist` | tasks (main, grafana_repo, remove_local_stack), defaults, handlers, meta, the Alloy and slice templates and the throttling script exist. |
| `test_every_template_and_file_the_tasks_name_exists` | (client, server) every `src:` of a template/copy task is shipped with the role. |
| `test_defaults_are_the_agent_only_host` | Defaults: enabled, port 12345, 5s/4s, push URLs `http://server.ros2.lan:9009/api/v1/push` and `:3100/loki/api/v1/push`, no node or stack targets, no cleanup, alloy 250M, no default `monitoring_host`. |
| `test_group_vars_set_the_host_label` | (client, server) `group_vars/<host>.yml` sets `monitoring_host` to the host's name. |
| `test_monitoring_slice_caps_per_host` | (client, server) CPUWeight=20, IOWeight=20; client CPUQuota=40%, 280M/300M; server CPUQuota=150%, 2000M/2200M. |
| `test_server_unit_caps_fit_the_server_slice` | The server's mimir, loki, grafana-server and alloy caps add up to no more than the slice's MemoryMax. |
| `test_alloy_dropin_moves_it_into_the_slice` | (client, server) the alloy drop-in: monitoring.slice, Nice=10, 250M, OOMScoreAdjust=500, Restart=on-failure, User=root. |
| `test_dropins_are_installed_for_every_agent_unit` | One template task loops `monitoring_agent_units` into `/etc/systemd/system/<unit>.service.d/`. |
| `test_agent_installs_alloy_only` | One apt install: alloy (+ libraspberrypi-bin on aarch64), no recommends, never prometheus or grafana. |
| `test_node_exporter_is_masked` | prometheus-node-exporter is masked. |
| `test_grafana_apt_repo_signed_by_keyring` | `grafana_repo.yml`: deb822 repo `https://apt.grafana.com stable main` signed by the key downloaded to `/etc/apt/keyrings/`; main.yml includes it. |
| `test_alloy_remote_writes_to_mimir_with_the_host_label` | (client, server) `prometheus.remote_write "mimir"` url: robot `http://server.ros2.lan:9009/api/v1/push`, server `http://127.0.0.1:9009/api/v1/push`; `external_labels = {host = "<host>"}`. |
| `test_alloy_pushes_logs_to_loki_with_the_host_label` | (client, server) `loki.write "server"` url: robot `http://server.ros2.lan:3100/loki/api/v1/push`, server 127.0.0.1; `external_labels` host. |
| `test_alloy_scrapes_every_5s_and_writes_to_mimir` | (client, server) every scrape 5s / 4s, every metrics `forward_to` ends in the Mimir remote write (directly or via the cadvisor relabel). |
| `test_alloy_collects_host_cgroup_and_self_metrics` | (client, server) unix (textfile dir), cadvisor (`docker_only = false`) and self exporters. |
| `test_alloy_drops_high_cardinality_cgroups_and_veth` | (client, server) the `id` keep rule keeps root, slices and services, drops scopes and sessions; veth interfaces are dropped. |
| `test_alloy_listens_on_lan_port` | `/etc/default/alloy`: `0.0.0.0:12345`, reporting off, config path. |
| `test_robot_alloy_scrapes_every_node_target` | The robot's `prometheus.scrape "nodes"` renders every `monitoring_agent_node_targets` entry as `127.0.0.1:<port>` with a `node` label, job `ros2_nodes`, `honor_labels`, 5s/4s; target nodes are unique and each is a present, enabled client `ros2_nodes` entry (no hardcoded count). |
| `test_robot_alloy_scrapes_no_stack` | The robot scrapes no Prometheus or Grafana any more. |
| `test_server_alloy_scrapes_the_stack_and_no_nodes` | The server's `prometheus.scrape "stack"` targets mimir 9009, loki 3100, grafana 3000; no node scrape. |
| `test_alloy_systemd_collector_limited_to_ros_and_stack_units` | (client, server) systemd collector with restarts and start time; `unit_include` matches ros2-*, alloy, mimir, loki, grafana-server, nginx and not prometheus, ssh, sockets. |
| `test_alloy_reads_the_journal_with_unit_and_level_labels` | (client, server) `loki.source.journal` with `max_age 12h`, `job="systemd-journal"`, relabel rules unit, level and transport. |
| `test_alloy_drops_debug_lines_before_loki` | (client, server) `loki.process` drops `{level="debug"}` and forwards to `loki.write`. |
| `test_parse_throttled` | `parse_throttled` reads `throttled=0x...` as hex and rejects other output. |
| `test_throttled_metrics_names_every_bit` | `rpi_throttled_flags` plus `rpi_throttled{bit=...}` for bits 0-3 (`_now`) and 16-19 (`_occurred`). |
| `test_script_writes_textfile_atomically` | With a fake `vcgencmd` the script writes the textfile and leaves no temp file. |
| `test_script_removes_stale_textfile_on_failure` | A failing `vcgencmd` removes the old file and exits non-zero. |
| `test_throttle_timer_runs_every_5s` | (client, server) `rpi-throttled.timer` fires every 5s; the oneshot service runs in monitoring.slice with the textfile path. |
| `test_alloy_and_timer_started_then_guarded_then_checked` | Alloy and the timer start, then a shell task restarts alloy when `/etc/default/alloy`, its config, drop-in or `monitoring.slice` is newer than its start, then `/-/ready` is awaited. |
| `test_disabled_agent_stops_alloy_and_timer` | `monitoring_agent_enabled: false` stops and disables alloy and the timer. |
| `test_cleanup_runs_on_the_robot_only` | `monitoring_agent_remove_local_stack` is true on the client, false on the server; main.yml includes `remove_local_stack.yml` under it. |
| `test_cleanup_stops_and_disables_prometheus_and_grafana_when_present` | Stop + disable of prometheus and grafana-server only while service_facts still knows them. |
| `test_cleanup_purges_the_packages_and_their_leftovers` | apt purges prometheus and grafana; drop-ins, `/etc/grafana`, `/etc/prometheus`, `/etc/default/prometheus`, `/var/lib/prometheus`, `/var/lib/grafana` are removed (systemd reloaded); no shell/command task, so a second run changes nothing. |
| `test_cleanup_runs_before_the_slice_is_installed` | The cleanup comes before the (smaller) slice is written. |
| `test_alloy_has_no_host_override_and_no_host_targets` | The Alloy template has no `target.host` or `monitoring_agent_host_targets` block, the defaults and both hosts' vars have no `monitoring_agent_host_targets`, and the client `nodes` scrape has no `host` label override. |

### `test_web_proxy_role.py`

`ansible/roles/web_proxy` (nginx on port 80 of both hosts), rendered per host.

| Test | Description |
|------|-------------|
| `test_site_per_host` | (client, server) `web_proxy_site` is robot / server and the template exists. |
| `test_robot_site_proxies_web_ui_with_websockets` | Robot: `listen 80 default_server`, `/` -> 127.0.0.1:8080 with Upgrade/Connection headers and `proxy_buffering off`. |
| `test_robot_ws_location_drops_stalled_clients` | Robot `/ws` (also `/ws/agent`): proxies to 8080 with upgrade headers, `proxy_send_timeout 30s`, `send_timeout 30s`, `proxy_read_timeout 1h`, unbuffered. |
| `test_robot_site_redirects_grafana_to_the_server` | Robot `/grafana/`: `return 302 http://server.ros2.lan$request_uri;` (path and query kept), no proxy. |
| `test_server_site_serves_grafana_and_redirects_root` | Server: `/` -> `302 /grafana/`, `/grafana/` -> 127.0.0.1:3000 with WebSocket upgrade. |
| `test_nginx_installed_default_site_removed_and_checked` | (client, server) nginx installed, the host's site written to `/etc/nginx/conf.d/<site>.conf`, `sites-enabled/default` removed, `nginx -t` before the handler flush, nginx enabled and started. |
| `test_readiness_checks_per_host` | The probe expects 302 without following: robot `/grafana/d/robot-health?orgId=1` -> `http://server.ros2.lan/grafana/d/robot-health?orgId=1`, server `/` -> `http://127.0.0.1/grafana/`. |

### `test_monitoring_dashboards.py`

The Grafana dashboards in `ansible/roles/monitoring/files/` (JSON structure, metric contract, host scoping, Loki panels). Selectors are found with a small PromQL scanner (strings, ranges, `by (...)` lists, functions and Grafana variables skipped).

| Test | Description |
|------|-------------|
| `test_selector_helper` | Guard against vacuous passes: the scanner finds selectors with their matchers and skips functions, labels, ranges, numbers and variables. |
| `test_dashboard_set_is_exactly_the_five_files` | `files/` holds robot-resources, robot-agent, robot-slam, robot-grasping and robot-health. |
| `test_dashboard_is_valid_json_with_required_panels` | (each dashboard) parses, has its title and uid, every required panel by title and unique panel ids. |
| `test_dashboard_common_settings` | (each dashboard) refresh 5s, last 1h, tag `robot`, one dashboard link by tag `robot`; datasource `prometheus-robot` on every panel and target, `loki-robot` on the two Loki panels. |
| `test_dashboard_queries_use_contract_metrics_only` | (each dashboard) every PromQL expression names only node metric contract names (histograms with `_bucket/_sum/_count`), `node_*`, `container_*`, `rpi_throttled*`, `robot_node_*` or `up`. |
| `test_host_dashboards_have_a_host_picker` | (Robot resources, Robot health) a single-value query variable `host` from `label_values(up, host)` on prometheus-robot, default `client`. |
| `test_every_selector_is_scoped_to_a_host` | (each dashboard) every vector selector carries `host="$host"` (resources, health) or `host="client"` (agent, SLAM, grasping). |
| `test_robot_only_dashboards_have_no_host_picker` | Agent, SLAM and grasping have no host variable. |
| `test_resources_dashboard_still_covers_units_and_throttling` | Robot resources still queries unit CPU, CPUQuota and throttling; the per-unit regex names mimir, loki, grafana-server and alloy, not prometheus. |
| `test_agent_dashboard_queries` | Robot agent: p50/p95 from `agent_instruction_duration_seconds_bucket` and per-tool `mcp_tool_duration_seconds_bucket`, MCP errors by `outcome="error"`, tokens, cost, claude_agent/mcp_server cgroup CPU and memory. |
| `test_slam_dashboard_queries` | Robot SLAM: cgroup CPU/memory of the six upstream SLAM units, lidar gap, relay rejections, odom rate, pose TF, nav goal duration, IMU and GPS panels, the 3D mapping row, and the Ground estimate row (`assert_ground_estimate_row`: fit rate by `result`, fit duration p50/p95, inlier ratio, slope, plane vs base, regions, warning areas, obstacle/drop points, colour mismatch, drops by `reason`, every query `host="client"`, and the `ros2-ground_estimator_cpp` unit CPU with a 0.10 core threshold line). The `ground_*` metrics and the `ground_fit_seconds` histogram are in the node metric contract. |
| `test_grasping_dashboard_queries` | Robot grasping: servo load/temperature/torque per arm joint, success ratio, bus failures, grasp tool latencies, tracking error buckets. |
| `test_health_dashboard_queries` | Robot health: `node_systemd_unit_state` for ros2-*, alloy, mimir, loki, grafana-server and nginx, restarts, `up{job="ros2_nodes"}`, node start time, every per-node error counter, throttling, disk and memory pressure. |
| `test_health_dashboard_loki_panels` | Robot health: `Error log lines per unit` = `sum by (unit) (rate({host="$host", level=~"emerg\|alert\|crit\|err"}[$__auto]))` (timeseries) and `Recent error log lines` = the same selector (logs panel); exactly two LogQL queries. |

### `test_monitoring_real_binaries.py`

The rendered configs and every dashboard query checked by the real binaries in Docker. Skipped when docker or one of the images (`grafana/mimir:3.2.2`, `grafana/loki:3.7.8`, `grafana/alloy:latest`, `nginx:stable`, `prom/prometheus:v2.45.3`, `busybox:1.36`) is not present locally; the tests never pull. Mimir and Loki run with `--network none` (loopback only) and are probed from a busybox container in their network namespace (the images are distroless).

| Test | Description |
|------|-------------|
| `test_mimir_config_starts_and_is_ready` | Mimir started from the rendered config answers `/ready`; its effective config has 1d block retention and multitenancy off. |
| `test_loki_config_verifies_starts_and_is_ready` | `loki -verify-config` accepts the rendered config and Loki started from it answers `/ready`. |
| `test_every_promql_query_parses` | `promtool check rules` parses every dashboard PromQL expression (Grafana variables substituted) as recording rules. |
| `test_every_logql_query_parses` | Loki's `/loki/api/v1/format_query` accepts both dashboard LogQL expressions and rejects a broken one. |
| `test_alloy_config_runs` | (client, server) `alloy run` on the rendered config logs "now listening" with no "failed to evaluate". |
| `test_nginx_site_passes_nginx_t` | (robot, server) `nginx -t` accepts the rendered site. |
| `test_robot_nginx_redirects_grafana_keeping_path_and_query` | The running robot site answers `/grafana/d/robot-health?orgId=1&var-host=server` with 302 to `http://server.ros2.lan/grafana/d/robot-health?orgId=1&var-host=server`. |

### test_rplidar_scan_supervisor_shutdown.py

Clean shutdown of `nodes/bridges/rplidar_a1/scan_supervisor.py` with rclpy stubbed (`tests/ros_stubs.py`: `install_ros_stubs`, and `FakeRclpy`, an rclpy double whose `shutdown()` raises the "already called" RCLError like a context the SIGTERM handler already shut down). `main()` returns 0, stops the launch child with SIGINT, destroys the node and calls `rclpy.try_shutdown()` (never `shutdown()`) when `spin_once` raises `ExternalShutdownException`, `KeyboardInterrupt` or the context-invalid `RCLError`. `test_sigterm_stop_with_child_signalled_is_a_normal_stop` (child returning -15/-2/0 after SIGTERM: exit 0, INFO log, no ERROR) and `test_unexpected_child_exit_while_running_still_errors_with_its_status` (child exit with no stop request: ERROR log, child's status).

### Per-node shutdown tests

Every Python ROS2 node with an rclpy main loop has a `tests/test_shutdown.py` (poi_store, rf2o_odom_relay, swerve_drive_controller, static_tf_publisher, filter_node, haptic_controller, map_guard, claude_agent, test_joint_api, gps_rtk, uvc_camera, bno055_imu, feetech_servos) using a copy of the same `ros_stubs.py` helper. They simulate systemd stopping the unit: the loop raises `ExternalShutdownException` / `KeyboardInterrupt` / the context-invalid `RCLError` with the context already shut down, and assert the entry function returns normally, the existing cleanup ran (node destroyed, wheels stopped, serial closed, camera released), `rclpy.shutdown()` was not called and `rclpy.try_shutdown()` was. A context-invalid `RCLError` while `rclpy.ok()` is still true must propagate. mcp_server: `test_spin.py::test_run_or_exit_ignores_an_error_raised_by_the_shutdown_itself`; `nodes/mcp_server/tests/test_uvicorn_config.py` (`build_uvicorn_config` sets `timeout_graceful_shutdown` to the 3 s constant, host and port).

## test_nav2_mppi_overlay.py

Static checks of the Nav2 MPPI overlay (patched `nav2_mppi_controller` built into `/opt/ros2-ws`; YAML via `yaml.safe_load`, patch and probe as text).

| Test | Description |
|---|---|
| `test_overlay_source_is_pinned_to_the_apt_tag_with_the_patch` | `nav2_mppi_overlay` is true and `nav2_mppi_overlay_source` is exactly the navigation2 repo, the full 1.3.13 SHA, package, `/opt/ros2-ws`, the E9 patch under `ros2_repo_dest`, Release + `BUILD_TESTING=OFF`, `stamp_apt_packages` and `jobs: 2`. |
| `test_overlay_patch_exists_and_only_touches_the_mppi_package` | The patch file exists and every file it touches is under `nav2_mppi_controller/`. |
| `test_probe_pin_equals_the_client_commit` | `PIN` in `scripts/probe-mppi-overlay-docker.sh` equals the overlay commit in `client.yml`. |
| `test_nav2_colcon_source_is_gated_on_the_rollback_flag` | The nav2_bringup `colcon_source` is `overlay_source if nav2_mppi_overlay \| bool else {}`. |
| `test_nav2_apt_packages_carry_the_overlay_build_dependencies` | nav2_bringup `apt_packages` include the toolchain, colcon, xtensor/xsimd/benchmark/openmp dev packages, the ament/nav2 deps and the apt MPPI. |
| `test_apt_version_feeds_the_stamp_key_only_for_a_non_empty_list` | `dpkg-query -W -f='${Version}'` (never changed, runs in check mode) loops over `stamp_apt_packages`; its output joins the stamp key only under a `length > 0` condition and the original key formula is intact (rf2o/libcamera/camera_ros keys unchanged). |
| `test_build_jobs_default_to_ros2_build_jobs` | The build `MAKEFLAGS` is `-j{{ colcon_src.jobs \| default(ros2_build_jobs) }}`. |
| `test_overlay_build_is_limited_to_the_mppi_subdirectory` | The overlay source sets `base_paths: [nav2_mppi_controller]` and the build passes `--base-paths` before `--packages-select` only when `base_paths` is set (sibling navigation2 packages must not shadow apt). |
| `test_launcher_still_sources_the_first_source_workspace` | The launcher sources the first source's workspace `install/setup.bash` after `/opt/ros/jazzy`, so the overlay wins on `AMENT_PREFIX_PATH`. |

## test_nav2_mppi_model_delay.py

Static derivation of MPPI `model_delay_*` from the swerve steering kinematics (`nav2_params.yaml` FollowPath `model_dt`; `max_steer_angular_velocity_rad_s` and `steer_error_threshold_rad` from the `swerve_controller` config in `client.yml`).

| Test | Description |
|---|---|
| `test_model_delay_vy_matches_the_ninety_degree_resteer` | `round((pi/2)/rate/model_dt)`, `round((pi/2 - threshold)/rate/model_dt)` and `round(model_delay_vy/model_dt)` all equal 3. |
| `test_model_delay_vx_and_wz_are_zero` | `model_delay_vx` and `model_delay_wz` are 0.0. |
| `test_inplace_resteer_rounds_to_a_noop_step` | The 48.8 deg in-place re-steer (minus the steer error threshold, when drive resumes) is 1 step (MPPI treats an offset of 1 as no delay), so `wz` needs no delay. |
| `test_keys_sit_in_the_mppi_namespace_only` | The three keys are in `controller_server.FollowPath`, not at the controller_server level. |

`test_nav_stack_config.py::test_colcon_source_build_runs_at_lowest_priority_and_only_on_a_new_commit` now expects the `MAKEFLAGS` expression above.

## test_mppi_cpu_compare.py

`scripts/mppi_cpu_compare.py` against a mocked httpx transport (no network).

| Test | Description |
|---|---|
| `test_pass_when_after_equals_before` | Equal windows PASS with the expected mean and p95. |
| `test_fail_on_relative_limit` | +20 % and +0.02 core FAILs. |
| `test_pass_on_abs_slack_when_rel_exceeded` / `test_pass_on_rel_slack_when_abs_exceeded` | The limit is the larger of the relative and the absolute allowance. |
| `test_fail_on_abs_limit` | Above both allowances FAILs. |
| `test_fail_when_only_p95_regresses` | Mean inside its limit but p95 outside FAILs. |
| `test_auto_switch_time_uses_container_start_time` / `test_auto_switch_time_without_series_is_an_error` / `test_numeric_switch_time_skips_the_start_time_query` | `auto` takes `container_start_time_seconds`; an epoch needs no query. |
| `test_window_bounds_exclude_the_settle_period` / `test_settle_period_samples_are_never_requested` | The after window starts `settle_min` after the switch and no query touches the settle period. |
| `test_empty_series_is_an_error_not_a_pass` | An empty before or after series raises `NoDataError`. |
| `test_default_config_loads_with_the_documented_values` / `test_config_validation_rejects_bad_values` | The YAML defaults, and negative limits, zero window, bad switch_time are rejected. |

## test_cpp_cutover.py

Static invariants of the cut-over to the C++ nodes (`ansible/group_vars/client.yml`, `ansible/README.md`).

| Test | Description |
|---|---|
| `test_cpp_node_is_live` | `map_guard_cpp` is `present: true`, `enabled: true`. |
| `test_python_node_stays_present_and_disabled_for_rollback` | `map_guard` stays `present: true`, `enabled: false` (rollback). |
| `test_cpp_config_block_equals_the_python_block` | The parsed `_cpp` config equals the Python node's and keeps its metrics port (19110). |
| `test_map_guard_cpp_keeps_dry_run` | The map_guard_cpp config still has `dry_run: true`. |
| `test_background_tier_holds_only_throttles_and_previews` | Among running nodes the background tier (100/100/0) holds only the two joint-state throttle nodes (the stereo nodes are retired, D7). |
| `test_only_the_running_node_of_each_pair_is_scraped` | `monitoring_agent_node_targets` lists the `_cpp` names and not the disabled Python ones. |
| `test_readme_documents_cutover_and_rollback` | `ansible/README.md` has a "Cut-over and rollback" section naming both nodes, `enabled: false` and `scripts/deploy-nodes.sh`. |

## test_jetson_retired.py

Retirement pin for the Jetson coprocessor. Extend it by adding entries to the two lists in the test module.

| Test | Description |
|---|---|
| `test_removed_path_is_gone` | Parametrized over `REMOVED_PATHS`: every listed repo-relative path is absent. |
| `test_removed_token_is_absent` | Parametrized over `ABSENT_TOKENS`: no `(path, token)` pair matches (the file does not contain the case-sensitive token). |
| `test_jetson_bridge_entry_is_uninstalled_by_the_next_deploy` | `jetson_bridge_cpp` `ros2_nodes` entry is `present: false` with no `enabled` and no `config`, and the comment above it names the Jetson. |
| `test_jetson_bridge_type_keeps_only_the_uninstall_and_purge_keys` | The node type has exactly `deploy_mode: native`, the launch command and `colcon_purge` of `jetson_bridge_cpp` in `/opt/ros2-ws`. |
| `test_client_playbook_still_deploys_jetson_bridge_so_the_uninstall_runs` | `deploy_nodes_client.yml` still deploys the name. |
| `test_jetson_bridge_metrics_target_is_removed` | No `monitoring_agent_node_targets` row for the bridge or port 19114. |
| `test_allowed_references_use_only_the_known_categories` | Every `ALLOWED_REFERENCES` entry is `archive`, `retired-entry`, `uninstall-artifact`, `retirement-test`, `retirement-doc` or `pinned-build`, and only the two archive directories and `nodes/head_camera_cpp/patches/**` (pinned-build) use a directory-wide glob. |
| `test_no_live_jetson_stereo_or_overview_reference` | `git grep -n -i -E` over tracked files for `jetson`, `depthd`, `stereo`, `overview_camera`, `depth_wire`, `fused_depth` and `10.77.0.` finds no hit outside the `(path glob, line regex, category)` allowlist; any other hit is fixed by editing the line. The head RGB-D calibration (shared core, `head_rgbd_calibration` node, web_ui RGB-D section) names its ToF-to-RGB stereo extrinsic calibration (`stereo_calibrate`, stereo RMS, `stereo_max_rms_px`); each such file has its own `rgbd-calibration` entry with the stereo-calibration regex. |

## test_node_cpu_compare.py

`scripts/node_cpu_compare.py` against a mocked httpx transport (no network).

| Test | Description |
|---|---|
| `test_reports_each_entry_with_before_unit_before_and_after_unit_after` | The before unit is read over the window before the switch, the after unit over the window after the settle period. |
| `test_total_row_sums_before_and_after` | The `total` row sums the entries' means and p95s. |
| `test_render_table_lists_every_row_and_the_total` | The table names every entry and the total. |
| `test_reuses_the_query_and_statistics_functions_of_mppi_cpu_compare` | `query_window`, `summarize` and `window_bounds` are the objects of `mppi_cpu_compare`, not copies. |
| `test_empty_series_raises_no_data` | An empty series raises `NoDataError`. |
| `test_main_exits_2_on_an_empty_series` / `test_main_prints_and_exits_0` | Exit 2 on no data, exit 0 and a table otherwise. |
| `test_queries_use_the_configured_host_and_both_windows` | Two queries per entry, at the before and after window starts, with the configured host. |
| `test_entry_requires_all_fields_and_a_positive_switch_time` | pydantic rejects a missing field and a non-positive switch_time. |
| `test_shipped_config_lists_the_nodes` | `node_cpu_compare.yaml` lists map_guard, web_ui, gripper_uvc_camera, mcp_server, nav2_bringup, laser_filter, arducam_tof, nav2_bringup_ground_layer with the right units (same unit before and after for the ToF and ground-layer entries; ground_estimator_cpp is new, so measured after only by the runbook query). |

### test_shared_photos.py / test_shared_lookups.py (photo contract)

| Test | Covers |
|---|---|
| `test_camera_view_without_base_pose_is_none` / `test_camera_view_without_mount_uses_base_heading` | No base pose gives None; no mount gives a `base_heading` view with z, pitch, hfov None. |
| `test_camera_view_forward_mount_composes_position_and_yaw` / `test_camera_view_pitch_down_is_positive` / `test_camera_view_mount_yaw_adds_to_base_yaw` / `test_camera_view_rolled_mount_keeps_heading` | Position composition, yaw and pitch sign from the mount rpy. |
| `test_camera_view_lateral_mount_offset_rotates_with_base` | Nonzero mount y offset on a rotated base (yaw pi/2 and pi) gives literal map x/y, pinning the lateral sign. |
| `test_photo_view_rejects_non_finite` | NaN / inf rejected. |
| `test_effective_view_prefers_stored_view` / `test_effective_view_derives_for_legacy_record` / `test_effective_view_none_without_pose` | Stored view wins; legacy lines derive one; no pose gives None. |
| `test_photo_as_poi_located` / `test_photo_as_poi_unlocated_has_null_xy_never_zero` | POI form fields; x/y None without a pose. |
| `test_photo_add_valid` / `test_photo_add_rejects_bad_payload` / `test_photo_add_requires_id` | Id format, `file == <id>.jpg`, client `bytes` refused. |
| `test_poi_as_record_round_trip_plain` / `_with_poses_and_view` / `_legacy_pose_without_view` | `poi_as_record(photo_as_poi(rec)) == rec` with and without poses and view. |
| `test_poi_as_record_rejects_non_photo_kind` / `_missing_photo_object` / `_bad_id` / `_file_not_matching_id` | Non-photo kind, missing `photo`, id not 32 lowercase hex and `file != <id>.jpg` raise `ValueError`. |
| `test_selector_accepts_exactly_one_criterion` / `test_selector_rejects_other_shapes` / `test_selector_accepts_500_ids_and_defaults_dry_run_false` | Exactly-one rule and bounds. |
| `test_write_atomic_writes_world_readable_file` / `test_write_atomic_fsyncs_before_replace` / `test_write_atomic_cleans_temp_on_failure` / `test_module_exposes_write_atomic` | Atomic write, mode 0644, fsync order, temp cleanup. |
| `test_legacy_index_line_parses_with_photo_defaults` / `test_photo_fields_round_trip` / `test_photo_fields_reject_bad_values` | `LookupRecord` photo fields: legacy line parses, round trip, bad values rejected. |

## test_chrony_role.py

The client `chrony` role: template rendered with jinja2 and the task file read as data; nothing runs ansible.

| Test | Covers |
|---|---|
| `test_chrony_conf_is_plain_pools` | One `pool <pool> iburst` line per default pool, the base directives (driftfile, ntsdumpdir, logdir, rtcsync, `makestep 1.0 3`), and no allow, local stratum, server line or 10.77 address. |
| `test_tasks_guard_timesyncd_and_restart_on_change` | Service facts gathered; timesyncd stop only when the unit exists; restart only when the registered template task changed. |
| `test_client_provisioning_applies_chrony` | `playbooks/client.yml` lists `chrony`. |

## test_jetson_link_removal.py

The removal of the retired Pi-Jetson link from the client (`playbooks/tasks/jetson_link_remove.yml` under the `jetson_link` tag): the task file and `deploy_nodes_client.yml` read as YAML; nothing runs ansible or touches a host.

| Test | Description |
|---|---|
| `test_removal_tasks_are_state_based_and_guarded` | `state: absent` for the `jetson.ros2.lan` hosts line (the old role's regexp), the sysctl drop-in, `jetson-nat.nft` and the nftables.conf include line (no `create`); the `nft delete table ip jetson_link` runs only when the `nft list tables` listing (never failing, never changed) shows it; `/etc/nftables.d` goes only when a `find` matched nothing. |
| `test_hosts_regexp_matches_only_the_jetson_line` | The hosts regexp matches `10.77.0.2 jetson.ros2.lan` and not `client.ros2.lan`, `myjetson.ros2.lan` or a longer name. |
| `test_ip_forward_reset_is_guarded` | `sysctl -w net.ipv4.ip_forward=0` (live, not persisted) only when the drop-in removal changed and `docker.service` is not running, after the service facts. |
| `test_every_task_is_registered_and_changes_only_on_removal` | Every task registers a distinct result; commands are `changed_when: false` or change only behind a `when`; file and lineinfile tasks are `state: absent`. |
| `test_nftables_package_and_service_are_left_alone` | No apt, package, service or systemd task, and no ruleset flush. |
| `test_client_block_runs_network_chrony_and_removal` | The "Remove the retired Jetson link" block is tagged `jetson_link`, has no `when` (an untagged run applies it), and runs the `network` role, the `chrony` role and `tasks/jetson_link_remove.yml` in that order, each applying the tag. |
| `test_no_other_client_task_carries_the_jetson_link_tag` | Only the block and its three applies carry the tag; the old block name and `jetson_link_side` are gone. |
| `test_role_and_link_vars_are_gone` | No `roles/jetson_link`, no `test_jetson_link_role.py`, no `jetson_link_*` var or D8 comment in `group_vars/all.yml`, no `network_link_ethernets`. |

## test_network_netplan.py

The network role's netplan rendering after the Jetson link support was dropped, rendered with jinja2; nothing runs ansible.

| Test | Description |
|---|---|
| `test_client_render_has_no_static_ethernet_and_no_link_address` | The WiFi-primary client render has no `ethernets` block, no `10.77` address and no `ignore-carrier`. |
| `test_wifi_stanza_is_unchanged` | The `wifis` stanza (address, default route, nameservers, access point) renders exactly as before. |
| `test_dhcp_ethernets_still_render` | Other ethernets render with `dhcp4: true` and `optional: true`. |
| `test_role_names_no_retired_variable` / `test_group_vars_define_no_link_ethernets` | Role defaults, tasks and template, and every group_vars file, name neither `network_link_ethernets` nor `network_netplan_superseded_files`. |

## test_node_uninstall_purge.py

The `colcon_purge` node type key: the uninstall path removes a retired C++ node's colcon output and stamps.

| Test | Description |
|---|---|
| `test_purge_removes_only_the_named_package_and_is_idempotent` | Runs `colcon_purge.yml` with the repo venv's `ansible-playbook -c local` on a temp workspace: only `pkg_a`'s src/build/install dirs and stamps go; `pkg_b` and `install/setup.bash` stay; a second run reports `changed=0`. |
| `test_uninstall_block_includes_colcon_purge_with_length_guard` | The uninstall block includes `colcon_purge.yml` with `become`, the length guard, after the config removal and before the systemd reload. |
| `test_purge_task_file_sets_no_become` | The task file itself sets no `become`. |
| `test_resolve_and_deploy_maps_colcon_purge_with_empty_default` | `resolve_and_deploy.yml` maps `colcon_purge` to `node_colcon_purge` with `default([])`. |

### `test_markers_pdf.py`

Exact-size marker PDF from `scripts/markers/pdf.py`. Each test renders page 0 with pypdfium2 at 600 dpi to a grey array and measures it, so the printed size is checked, not just the drawing calls.

| Test | What it covers |
|------|----------------|
| `test_page_size_pt` | `page_size_pt` returns A4, A3, landscape and custom media in points (A4 = 595.28 x 841.89). |
| `test_pdf_mediabox_equals_media` | The PDF MediaBox equals the media size (no scaling). |
| `test_symbol_edge_measured_on_raster` | The dark bounding box of the symbol on the raster equals `size_mm` within 0.1 mm for 60, 100 and 150 mm markers. |
| `test_quiet_zone_is_white` | The ring around the symbol is white for at least `quiet_zone_modules` modules. |
| `test_cut_square_is_grey_outside_quiet_zone` | The thin grey cut square lies outside the quiet zone and is lighter than the dark modules. |
| `test_ruler_measures_100_mm` | The ruler bar measures 100 mm within 0.1 mm and starts at the reported origin. |
| `test_page_text_has_payload_and_scale_note` | Extracted page text contains the payload text, label, `Print at 100 %` and `100 mm`. |
| `test_raster_decodes_to_payload` | zxing-cpp decodes the rendered raster to the payload text (60, 100, 150 mm). |
| `test_oversize_marker_raises_does_not_fit` | A 200 mm marker on A4 portrait raises the does-not-fit error naming needed and available mm. |
| `test_draw_mask_pdf_draws_cells_at_origin` | `draw_mask_pdf` draws a boolean mask as vector cells at the given origin with the right extent, gaps left white and the label as text. |

### test_markers_charuco.py

ChArUco calibration board from `scripts/markers/charuco.py`; the PDF raster and the module footprint are fed to `cv2.aruco.CharucoDetector`.

| Test | What it covers |
|------|----------------|
| `test_shipped_spec_is_6x5_and_fits_bed` | Defaults are 6x5, 40/30 mm, DICT_5X5_100 and the plate is 250 x 210 mm (within the 256 mm bed). |
| `test_yaml_carries_charuco_section` | `scripts/markers_generate.yaml` loads with the default `charuco` spec. |
| `test_invalid_specs_name_the_field` | 7 columns (too wide), even rows, `marker_mm >= square_mm`, thin base and unknown keys are refused, naming the field. |
| `test_board_mask_is_exact_pixels` | Mask is 336 x 280 px at 1.4 px/mm (whole-pixel squares, marker cells and gaps). |
| `test_pdf_detects_all_corners_at_40_mm` | The 600 dpi PDF raster yields all 20 inner corners, 40 mm pitch within 0.1 mm, and `Print at 100 %` text. |
| `test_mesh_footprint_matches_board_corners` | Top view of the module body detects 20 corners within 0.2 mm of `getChessboardCorners()`. |
| `test_stl_base_is_4_mm_thick_and_plate_sized` | Base STL is 250 x 210 x 4 mm, modules rise 0.6 mm; pdf, two STLs and 3MF are written. |
| `test_outputs_subset` | `outputs: [pdf]` writes only the PDF. |

### `test_shared_marker_payload.py`

The marker payload codec `shared/ros2_common/marker_payload.py` (`P1/K:<kind>/N:<name>/S:<mm>/<KEY>:<VAL>`), no hardware needed.

| Test | What it covers |
|------|----------------|
| `test_round_trip_known_kinds` | Encode then decode returns the same payload for every known kind code and its POI kind. |
| `test_round_trip_params_and_minor` | Extra params (including alphanumeric punctuation and empty values) and a minor version survive a round trip. |
| `test_decode_example` | The documented example string decodes to the expected fields. |
| `test_canonical_order_and_size_format` | Encoding emits fields in canonical order and formats sizes without trailing zeros. |
| `test_minor_accepted_major_rejected` | `P1.3` is accepted, an unknown major version is rejected. |
| `test_unknown_kind_lowercased` | An unknown kind code decodes and maps to a lower-case POI kind. |
| `test_malformed` | Malformed texts raise `PayloadError` with the matching reason. |
| `test_payload_error_is_value_error` | `PayloadError` subclasses `ValueError`. |
| `test_model_validation` | The pydantic model refuses invalid kind, name, size and params. |
| `test_marker_info_carries_canonical_text` / `test_marker_info_from_decoded_payload` | `marker_info` carries the canonical text, key and pose sample count, also from a decoded payload. |

### `test_markers_symbol.py`

Generator config loading (`scripts/markers/config.py`) and the QR symbol builder (`scripts/markers/symbol.py`).

| Test | What it covers |
|------|----------------|
| `test_shipped_yaml_covers_known_kinds` | `scripts/markers_generate.yaml` loads and has a marker per known kind. |
| `test_unknown_key_names_it` / `test_invalid_marker_name_names_it` / `test_defaults_are_validated` | Invalid config raises a pydantic error naming the key or marker. |
| `test_charuco_slot_is_a_validated_spec` | The `charuco` section is validated as a `CharucoSpec`. |
| `test_charger_builds_v2_q_6mm_modules` | A 150 mm CHARGER marker is version 2-Q with 6 mm modules. |
| `test_longer_payload_picks_v3` / `test_over_capacity_raises_naming_capacity` / `test_ecc_m` | Version selection, over-capacity error and error-correction level are honoured (no silent EC boost). |
| `test_raster_decodes_to_payload` | A rendered raster decodes with zxing-cpp to the payload. |
| `test_capacity_table_matches_segno` | `ALNUM_CAPACITY` equals what segno accepts. |

### `test_markers_mesh.py`

Two-body marker meshes and the Bambu 3MF from `scripts/markers/mesh.py`.

| Test | What it covers |
|------|----------------|
| `test_bodies_watertight_and_disjoint` / `test_bodies_disjoint_with_inlay` | Base and modules are watertight and do not overlap, also with a flush inlay. |
| `test_module_footprint_matches_matrix` / `test_no_module_material_in_quiet_zone` | The module top view equals the QR matrix and the quiet zone stays free. |
| `test_heights` / `test_flush_inlay_heights_and_pockets` | Base thickness, relief height and inlay pockets are as configured. |
| `test_plate_size_150mm_v2` | Plate side is size + quiet zone + margin. |
| `test_module_too_small_raises_naming_size` / `test_module_size_warning` | Modules under `min_module_mm` raise, under `warn_module_mm` log a warning. |
| `test_invalid_heights_raise` | Zero module height or an inlay deeper than the base raises. |
| `test_write_bodies_3mf_structure` | The 3MF holds two named objects with extruders 1 and 2; both STLs are written. |
| `test_render_then_decode` | The module body rendered from above decodes to the payload. |
| `test_mask_bodies_unions_adjacent_cells_and_orients_rows` | Adjacent cells merge and mask row 0 is the top. |

### `test_markers_generate_cli.py`

The `scripts/markers_generate.py` command end to end, with tmp configs and output dirs.

| Test | What it covers |
|------|----------------|
| `test_two_markers_and_charuco_write_expected_files` | Two markers plus the `charuco` section write exactly the expected file set (`<kind>-<name>` stems, `charuco-6x5-40mm`), print each file and return 0. |
| `test_unknown_key_returns_1_and_writes_nothing` | An unknown config key returns 1, the error names the key, nothing is written. |
| `test_too_small_stl_marker_fails_others_written` | A marker too small for FDM returns 1, is named in the error, writes none of its files, and the other marker's files exist. |
| `test_relative_output_dir_resolves_against_repo_root` | A relative `output_dir` lands under the repo root. |
| `test_shipped_config_runs_and_pdfs_decode` | The shipped `markers_generate.yaml` (output redirected) runs; every marker PDF, rasterised, decodes to its own payload; STL and 3MF exist. |

### test_map_manager_config.py

Static invariants of the map_manager wiring (Ansible, playbook, docs).

| Test | Covers |
|---|---|
| `test_map_manager_node_type_defaults` | Node type: native, `python3 -m map_manager`, `nodes/map_manager`, `src_shared`, config path and env, normal tier, 64M/256M memory, `ros-jazzy-slam-toolbox`. |
| `test_map_manager_entry_present_and_enabled_after_poi_store` | The `ros2_nodes` entry is present, enabled and after poi_store. |
| `test_map_manager_config_validates_and_documents_every_key` | The config validates against `MapManagerConfig`, lists every field, port 19113, maps root under `/var/lib/ros2/maps`. |
| `test_map_manager_config_matches_defaults` | The config block equals the model defaults. |
| `test_map_manager_config_has_group_comments` | The config block carries group comments. |
| `test_maps_dir_task_creates_parent_owned_by_node_user` | `slam_maps_dir.yml` creates `/var/lib/ros2/maps` owned by `ansible_user`. |
| `test_playbook_deploys_map_manager_after_poi_store_with_maps_dir` | The playbook deploys map_manager after poi_store, tagged, with the maps dir task. |
| `test_docs_mention_map_manager` | nodes/README.md, ansible/README.md and the ansible-deploy SKILL.md list the node. |

### test_agent_skills.py

Static invariants of the agent skills shipped in `nodes/claude_agent/claude_agent/agent_plugin`.

| Test | Covers |
|---|---|
| `test_skills_exist` | The `recover-from-failure` skill file exists. |
| `test_every_referenced_tool_exists` | Every backticked tool call or `agent.<tool>` in a SKILL.md is a registered mcp_server tool, a claude_agent planning tool or `Skill`. |
| `test_tool_reference_scanner_flags_bare_unknown_tool` | The scanner picks up bare backticked tool names (no call parenthesis) and `agent.<tool>` references, not other identifiers. |
| `test_tool_reference_scanner_flags_unknown_tool` | The scanner finds unknown tool references. |
| `test_optional_tools_only_in_optional_section` | Head, 3D-map tools and `zoom` appear only under `## When extra perception tools exist`, which also names `look_around`. |
| `test_split_optional_isolates_section` | The section splitter separates the optional section from the rest. |
| `test_plugin_ships_read_only_from_root_checkout` | The repo is cloned with `become`, the service user is `claude_agent`, the plugin is under `nodes/claude_agent/` and the workdir is outside the repo. |

### test_agent_retrospectives_script.py

Unit tests for `scripts/agent_retrospectives.py` (pulls the agent's retrospectives into `.logs/retrospectives`), using `httpx.MockTransport`.

| Test | What it covers |
|---|---|
| `test_shipped_config_validates` | `scripts/agent_retrospectives.yaml` loads with the documented defaults. |
| `test_config_rejects_unknown_key_and_bad_limit` | The pydantic config forbids extra keys and a limit below 1. |
| `test_render_markdown_includes_all_sections` | A sample record renders date, workflow, outcome, author, summary, lists, notes for fixers, skipped mandatory steps and phases. |
| `test_render_markdown_free_job_without_lists` | A free job renders as `free` and empty sections are omitted. |
| `test_main_writes_timestamped_files` | The request path and limit, the matching timestamped `.md`/`.jsonl` names, the raw content and the printed paths and count. |
| `test_main_exit_1_on_http_error` | A 502 gives exit 1 with the status on stderr. |
| `test_main_exit_1_on_connection_error` | A connection error gives exit 1 with the reason. |
| `test_main_exit_2_on_bad_config` | An invalid or missing config gives exit 2. |

### test_agent_skills_docs.py

Docs guard for the driving-agent skills suite.

| Test | What it covers |
|---|---|
| `test_readme_has_skills_workflows_retrospectives_sections` | The claude_agent README has non-empty Skills, Workflows and Retrospectives sections. |
| `test_every_skill_directory_is_listed_in_readme` | Every skill directory name appears in the README Skills section. |
| `test_every_workflow_template_is_listed_in_readme` | Every workflow template name appears in the README Workflows section. |
| `test_nodes_readme_mentions_the_new_features` | The `nodes/README.md` claude_agent line names skills, workflows and retrospectives; the costmap_zones line names slow-zone cost. |
| `test_memory_records_the_decision` | MEMORY.md records the skills, workflows, retrospectives, POI category and slow-zone decisions. |
| `test_tests_readme_lists_this_file` | This file is listed here. |

### test_nav_ground_voxel_layer.py

Static checks of the plane-relative `ground_voxel_layer` and the `ground_cost_filter` in the Nav2 local costmap (design D8, D13, D15), the unchanged global costmap, collision monitor and CliffStop, and the README section. Reads `nav2_params.yaml`, `client.yml`, the ground_estimator_cpp config, the costmap_zones config and the READMEs.

| Test | Covers |
|---|---|
| `test_local_plugin_order_puts_ground_cost_filter_between_keepout_and_inflation` | Local plugins are exactly `obstacle_layer, tof_voxel_layer, ground_voxel_layer, keepout_filter, ground_cost_filter, inflation_layer`. |
| `test_ground_cost_filter_is_a_keepout_filter_on_the_ground_cost_info_topic` | `ground_cost_filter` is a KeepoutFilter on `/costmap_filter_info/ground_cost`, enabled, `override_lethal_cost` false, not in `filters`. |
| `test_ground_cost_filter_info_topic_equals_the_costmap_zones_default` | The info topic equals the costmap_zones `ground_cost_info_topic` default. |
| `test_ground_warning_mask_cost_stays_below_inscribed` | The costmap_zones `ground_warning_mask_value` times 254 / 100 is below INSCRIBED 253 (passable). |
| `test_local_costmap_keeps_exactly_one_speed_filter` | One SpeedFilter on `/costmap_filter_info/speed`; the slow-down uses the shared speed mask. |
| `test_ground_layer_settings_equal_the_tof_layer_settings` | Every layer setting equals `tof_voxel_layer`. |
| `test_ground_layer_sources_topics_and_pass_through_limits` | Sources `ground_obstacle_mark, ground_drop_mark, ground_clear`; topics equal the ground_estimator_cpp `output_namespace` (client.yml and package config) plus `/obstacle_points` / `/drop_points`; limits -1.5 / 1.5 and the buffer settings. |
| `test_ground_clear_equals_the_tof_clear_frustum` | `ground_clear` equals `tof_clear` on `/tof/points_filtered`. |
| `test_global_costmap_is_untouched` | Global plugins unchanged; no STVL and no ground topic. |
| `test_collision_monitor_sources_are_unchanged_and_free_of_ground_topics` | Monitor sources `scan, tof_obstacles, tof_cliff`, CliffStop on `tof_cliff` only, no `/ground/` topic. |
| `test_tof_voxel_layer_is_pinned` | `tof_voxel_layer` sources and bands unchanged. |
| `test_ground_values_are_commented_with_their_reasons` | Layer and filter values carry comments; the header names the runtime switch, the fixed bands and D-A1. |
| `test_readme_documents_the_ground_layer` | The README section names the topics, pass-through limits, what stays on the fixed bands, the obstacle vs warning rule, the slow-down mechanism, the runtime switch and the four switch-over criteria. |
| `test_readme_ground_section_has_no_parking_wording` | The section defers nothing. |
| `test_tests_readme_documents_this_file` | This section lists every test of the file. |

### test_nav_tof_voxel_layer.py

Static checks of the head ToF spatio-temporal voxel layer (`tof_voxel_layer`, STVL) in the Nav2 local costmap and of its lidar separation (U5). Reads `nav2_params.yaml`, `client.yml`, the tof_obstacle_cloud and laser_filter params and the READMEs.

| Test | Covers |
|---|---|
| `test_local_costmap_has_the_tof_layer_after_the_lidar_layer` | Local plugins are exactly `obstacle_layer, tof_voxel_layer, ground_voxel_layer, keepout_filter, ground_cost_filter, inflation_layer`. |
| `test_tof_voxel_layer_settings` | Every layer value and type: STVL plugin, enabled, `voxel_decay` 15.0, `decay_model` 0, `voxel_size` 0.05, `track_unknown_space` false, `mark_threshold` 0, `update_footprint_enabled` false, `combination_method` 1, `publish_voxel_map` false, `transform_tolerance` 0.3, `mapping_mode` false, `observation_sources`. |
| `test_stvl_footprint_clearing_is_disabled` | `update_footprint_enabled` is false and its comment names `setConvexPolygonCost` (convex fill erases obstacles in the H notches). |
| `test_tof_sources_settings` | `tof_mark`, `tof_cliff_mark` and `tof_clear` values and types (PointCloud2 on `/tof/points_filtered`, height bands, range, buffer settings, frustum). |
| `test_mark_and_clear_sources_are_split` | The two mark sources mark and never clear, `tof_clear` clears and never marks, frustum model 0. |
| `test_obstacle_band_is_above_the_floor_and_below_the_robot` | 0 < `min_obstacle_height` < `max_obstacle_height` <= 0.6 m robot height. |
| `test_mcp_tof_obstacle_band_equals_tof_mark` | `TOF_OBSTACLE_BAND_M` in mcp_server `perception.py` (read with ast, no import) equals the `tof_mark` (`min_obstacle_height`, `max_obstacle_height`) band, so the agent fixed-band summary matches the costmap. |
| `test_cliff_band_is_strictly_below_the_floor_noise` | Cliff band ends strictly below -0.05 m. |
| `test_bands_do_not_overlap` | The cliff band ends below the obstacle band start. |
| `test_ranges_stay_inside_the_cloud_max_depth` | `max_z` and both `obstacle_range` <= tof_obstacle_cloud `max_depth`; `min_z` >= its `min_depth`. |
| `test_clearing_fov_is_inside_the_tof_fov` | Clearing FOV angles are below the ToF horizontal/vertical FOV (70 deg diagonal, 240 x 180). |
| `test_tof_fov_constants_match_the_design` | The FOV constants give 58.5 x 45.6 deg. |
| `test_tof_topic_is_the_laser_filter_output` | `/tof/points_filtered` is laser_filter's `cloud_filtered_topic`. |
| `test_lidar_obstacle_layer_keeps_the_scan_source` | Both costmaps keep the enabled ObstacleLayer with the single `/scan_filtered` LaserScan source (marking and clearing). |
| `test_no_stvl_source_uses_the_lidar` | No STVL source reads `/scan_filtered`, a scan topic or a LaserScan. |
| `test_global_costmap_has_no_tof_voxels` | Global plugins unchanged; no STVL plugin and no ToF topic there (user decision Q3). |
| `test_only_one_stvl_layer_in_the_local_costmap` | The STVL layers of the local costmap are exactly `tof_voxel_layer, ground_voxel_layer`. |
| `test_layer_values_are_commented_with_their_reason` | Key values carry a comment naming their reason (decay, notches, max, TF, floor, robot height, max_depth, 70 deg FOV, collision monitor). |
| `test_nav2_bringup_installs_stvl` | nav2_bringup `apt_packages` lists `ros-jazzy-spatio-temporal-voxel-layer` with a comment. |
| `test_readme_documents_the_tof_obstacle_layer` | The nav2_bringup README "ToF obstacle layer" section names the pipeline, layer settings, the U5 separation, local-only and the no-returns cliff limitation. |
| `test_readme_tof_section_has_no_parking_wording` | The nav2_bringup README "ToF obstacle layer" section states what the values are and how to check them, with no wording that defers the values to a later change. |
| `test_tests_readme_documents_this_file` | This section lists every test of the file. |

### test_nav_tof_collision.py

Static checks of the collision_monitor ToF sources `tof_obstacles` / `tof_cliff`, the per-polygon source sets and the direction-aware CliffStop approach polygon (design D4, D5; U1, U5). Reads `nav2_params.yaml` (values and comments), the generator output of `scripts/gen_nav_footprint.py` and the READMEs; replays the nav2 1.3.13 approach model (`VelocityPolygon::isInRange`, `Polygon::getCollisionTime`, translation only) on the CliffStop sub-polygons.

| Test | Covers |
|---|---|
| `test_observation_sources_are_scan_and_the_two_tof_sources` | `observation_sources` is exactly `scan, tof_obstacles, tof_cliff`. |
| `test_tof_source_settings` | Both ToF sources: `pointcloud` on `/tof/points_filtered`, bands 0.10-0.6 m and -1.0 to -0.10 m, `min_range` 0.0, enabled (values and types). |
| `test_tof_source_band_equals_the_stvl_mark_band` | `tof_obstacles` band equals the STVL `tof_mark` band, `tof_cliff` equals `tof_cliff_mark`, same topic. |
| `test_no_collision_monitor_source_reads_a_ground_topic` | No collision_monitor source topic starts with `/ground/` (D-A1). |
| `test_scan_source_is_unchanged` | U5: `scan` stays `type: scan` on `/scan_filtered`, enabled. |
| `test_polygons_include_cliffstop` | `polygons` is exactly `StopBox, ArmStop, CliffStop`. |
| `test_every_polygon_names_only_declared_sources` | Every listed polygon has a non-empty `sources_names` that names only declared observation sources. |
| `test_polygon_source_sets` | StopBox `scan, tof_obstacles`; ArmStop `scan`; CliffStop `tof_cliff`. |
| `test_cliffstop_is_a_direction_aware_approach_polygon_with_the_generated_points` | CliffStop is an enabled holonomic `velocity_polygon` with `approach`, `time_before_collision` > 0 and sub-polygons `rotate, forward, back`; `rotate` / `forward` equal the generator's cliff polygon, `back` the footprint. |
| `test_cliffstop_stops_motion_toward_or_along_cliff_points_inside` | Cliff points 0.42-0.52 m ahead (inside CliffStop): forward, left, right, forward-left and a zero command get collision time 0 (stop). |
| `test_cliffstop_passes_backing_away_from_cliff_points_inside` | The same points: back, back-left and back-right get no collision (pass). |
| `test_cliffstop_slows_the_approach_to_a_cliff_outside_and_passes_backing_away` | Cliff points 0.62-0.72 m ahead: forward 0.2 m/s gets a collision time inside `time_before_collision` (slowdown), backward none. |
| `test_tof_sources_keep_the_stale_source_stop` | Node `source_timeout` 1.0 and no ToF source overrides it (no `source_timeout: 0.0`), so a ToF or head-TF outage stops motion. |
| `test_comments_explain_the_outage_stop_and_the_runtime_switch` | The collision_monitor comments name "invalid source", `collision_monitor_node.cpp`, `ros2 param set /collision_monitor tof_cliff.enabled false`, the ArmStop edge noise and the arm shadow. |
| `test_cliffstop_comment_explains_the_direction_aware_shape` | The CliffStop comment names `getCollisionTime`, the `rotate` / `forward` / `back` sub-polygons and that backing away passes. |
| `test_readme_documents_the_tof_collision_sources` | The nav2_bringup README "Collision monitor" section names both ToF sources, the topic, CliffStop / ArmStop, the invalid-source stop, the runtime switch, that backing away from a seen cliff passes and the cliff view limitation. |
| `test_readme_records_the_docker_validation` | The README "ToF validation in Docker" section names the image, the three packages, `tof_optical_frame`, `cmd_vel_smoothed` and the invalid-source outcome; every case with cliff points inside CliffStop records that backward -0.1 passes. |
| `test_readme_sections_have_no_parking_wording` | Neither README section defers anything to a later change. |
| `test_tests_readme_documents_this_file` | This section lists every test of the file. |

### test_tof_obstacle_cloud_config.py

Static checks of the `tof_obstacle_cloud` deployment (config-only rtabmap_util point_cloud_xyz, Client only).

| Test | Covers |
|---|---|
| `test_launch_command_runs_point_cloud_xyz_with_params_file` | The launch command runs `ros2 run rtabmap_util point_cloud_xyz` with the repo params file. |
| `test_launch_command_remaps_topics_and_node_name` | Remaps are exactly depth/image, depth/camera_info, cloud -> `/tof/points` and `__node:=tof_obstacle_cloud`. |
| `test_node_type_is_native_config_only_with_apt_package` | Native, no src dir, `src_paths`, `ros-jazzy-rtabmap-util`, no pyproject. |
| `test_node_type_is_normal_tier_with_estimated_memory` | Normal tier weights, `memory_low` 64M, `memory_max` 192M. |
| `test_params_file_values` | The params file holds the expected values and types plus a `qos`. |
| `test_params_file_comments_every_key` | Every param has a comment line above it. |
| `test_max_depth_within_arducam_range_and_min_depth_positive` | `max_depth` within the configured `range_mode` range (far 4 m, near 2 m), `min_depth` > 0. |
| `test_arducam_frame_id_is_tof_optical_frame` | arducam_tof `frame_id` is `tof_optical_frame`. |
| `test_no_tf_publishing_argument_and_slam_toolbox_owns_map_odom` | No TF argument; no other client type publishes `map -> odom` (amcl, static transform); slam_toolbox exists. |
| `test_playbook_deploys_node_after_arducam_tof` | The client playbook has the deploy block after arducam_tof. |
| `test_ros2_nodes_entry_present_enabled_after_arducam_tof` | The `ros2_nodes` entry is present, enabled and right after arducam_tof. |
| `test_node_is_not_on_the_server` | Not in server.yml. |
| `test_readme_documents_topics_and_placeholder_rule` | README names the topic, frame, rationale and empty-cloud rule. |
### test_tof_obstacle_wiring.py

Cross-file topic and frame chain of the head ToF obstacle pipeline (arducam_tof -> tof_obstacle_cloud -> laser_filter -> Nav2 STVL, collision_monitor, web_ui, mcp_server). Reads the YAML only.

| Test | Covers |
|---|---|
| `test_tof_obstacle_cloud_output_is_the_laser_filter_input` | The `cloud` remap of the tof_obstacle_cloud launch command equals laser_filter `cloud_topic`. |
| `test_laser_filter_output_feeds_every_consumer` | laser_filter `cloud_filtered_topic` equals every STVL source topic, both collision_monitor ToF sources, the web_ui map_nav `obstacle_cloud_topic` (client.yml and default.yaml) and the mcp_server `topics.tof_points` (client.yml and config default). |
| `test_ground_estimator_input_is_the_laser_filter_output_and_the_ground_clear_topic` | laser_filter `cloud_filtered_topic` equals the ground_estimator_cpp `cloud_topic` and the `ground_clear` topic. |
| `test_stvl_observation_sources_are_exactly_the_three_tof_sources` | The STVL layer lists exactly tof_mark, tof_cliff_mark, tof_clear. |
| `test_arducam_tof_frame_id_is_the_tof_optical_frame` | arducam_tof `frame_id` is `tof_optical_frame`. |
| `test_stvl_and_collision_monitor_height_bands_agree` | tof_obstacles = tof_mark and tof_cliff = tof_cliff_mark in min and max height. |
| `test_ranges_and_collision_sources_are_listed` | The collision monitor observes scan, tof_obstacles and tof_cliff. |
| `test_slam_toolbox_publishes_map_to_odom` | slam_toolbox `transform_publish_period` > 0 with map and odom frames. |
| `test_no_other_client_node_publishes_map_to_odom` | No other client node sets `publish_tf: true` (but the EKF, odom -> base_link), the static TF publisher has no map parent or odom child, tof_obstacle_cloud has no TF argument. |
| `test_new_nodes_and_tests_are_in_the_tests_readme` | tof_obstacle_cloud and the four ToF obstacle test files have a section here. |

### `test_head_rgbd_calib_core.py`

Calibration core in `shared/ros2_common/head_rgbd_calibration`: an AST scan proves no module imports `scripts`, `rosbags` or `subprocess`; `BoardConfig`, `SelectionConfig`, `LimitsConfig` reject unknown keys and their defaults equal the `board`, `selection`, `limits` blocks of `scripts/head_rgbd_calibrate.yaml`; the core `detect` finds the synthetic board of `tests/rgbd_synthetic.py`.

### `test_head_rgbd_calib_intrinsics.py`

Tests for `shared/ros2_common/head_rgbd_calibration/charuco.py` and `intrinsics.py`, on the shared synthetic scenes of `tests/rgbd_synthetic.py` (ChArUco board rendered through a known RGB 576x324 / ToF 240x180 pinhole, with noise) and real ToF amplitude frames in `tests/fixtures/head_rgbd/`.

| `test_make_board_matches_config` | Board size and lengths follow `BoardConfig`. |
| `test_rgb_views_recover_intrinsics` | 20 RGB views recover fx, fy within 1 % and cx, cy within 3 px. |
| `test_tof_views_recover_intrinsics_with_zero_distortion_model` | 20 normalised ToF amplitude views recover fx, fy within 2 % and cx, cy within 2 px; zero true distortion gives the zero-D model. |
| `test_tof_with_true_distortion_picks_free_model` | True k1 = -0.1 selects the free model and recovers k1 within 0.03. |
| `test_partial_board_still_detects_six_corners`, `test_blank_image_has_no_detection` | Half the board outside the image still gives >= 6 corners; a blank image gives None. |
| `test_fewer_than_min_views_raises_naming_count` | Fewer than `min_views` views raise, naming the count. |
| `test_yaml_round_trips_in_camera_calibration_format` | Written YAML loads with the ROS camera_calibration keys and shapes. |
| `test_rms_over_limit_writes_nothing`, `test_rms_at_limit_is_written` | RMS over `rgb_max_rms_px` / `tof_max_rms_px` raises and writes no file. |
| `test_compare_sdk_reports_percent_difference` | fx/fy/cx/cy differences in % against the SDK camera_info. |
| `test_real_near_tof_frames_need_the_upscale` | Six real ToF amplitude frames of the printed board at about 0.3 m (`tests/fixtures/head_rgbd/`) give no detection at native size and at least five with `upscale=2`. |
| `test_upscaled_corners_are_in_source_pixels` | Corners detected with `upscale=2` match the native detection within 0.1 px (mapped back to source pixels). |
| `test_detect_tof_upscales_by_default` | `pipeline.detect_tof` uses `selection.tof_upscale` (default 2): the real near frames detect, `tof_upscale=1` does not. |
| `test_synthetic_scene_geometry_is_consistent` | Generated poses project the board inside the RGB image. |

### `test_head_rgbd_calib_extrinsics.py`

Tests for `shared/ros2_common/head_rgbd_calibration/extrinsics.py` on synthetic RGB + ToF pairs with a known extrinsic.

| Test | What it checks |
|---|---|
| `test_synthetic_scene_has_enough_pairs` | The generated scene gives at least 18 pairs seen by both cameras. |
| `test_stereo_recovers_true_extrinsic` | 20 pairs recover translation within 2 mm and rotation within 0.2 deg. |
| `test_result_maps_tof_points_onto_rgb_projection` | Direction: a ToF point mapped with the result lands within 0.5 px of the RGB corner. |
| `test_pair_without_common_ids_is_skipped` | A pair with no common corner ids is not counted. |
| `test_fewer_than_min_views_raises` | Fewer than `min_views` pairs raise. |
| `test_plane_check_reports_depth_bias` | ToF depth biased +20 mm reports 20 +- 3 mm, unbiased 0 +- 3 mm. |
| `test_plane_check_rejects_pair_with_background_in_outline` | A pair with two thirds of its board pixels 400 and 800 mm farther (background) is rejected and counted, the median stays 0 +- 3 mm; with `plane_max_spread_mm` 1000 it is kept. |
| `test_plane_check_keeps_every_clean_pair` | A uniformly +20 mm biased scene rejects no pair. |
| `test_yaml_round_trips` | Written extrinsics YAML has the documented keys and loads back. |
| `test_rms_over_limit_writes_nothing` | Stereo RMS over `stereo_max_rms_px` raises and writes no file. |

### `test_head_rgbd_calib_poses.py`

Tests for `ros2_common.head_rgbd_calibration.poses` (`PoseCounter`), on a synthetic 15 s stream (ToF 10 Hz, RGB 2 Hz
from `tests/rgbd_synthetic.py`): moving board, a still pose at 0.33 m, a still pose at 1.2 m (too far for the ToF),
another still pose at 0.33 m.

| Test | What it checks |
|---|---|
| `test_counts_one_pose_per_still_window` | The two near still poses are captured once each (numbers 1 and 2); the far one is not. |
| `test_moving_board_is_not_counted` | Statuses include `moving` and no pose is attached to them. |
| `test_status_names_missing_tof_board` | A board not found by the ToF gives the `move closer` status and no pose. |
| `test_counter_agrees_with_the_offline_still_pairs` | The live count equals the offline `still_pairs` with both detections. |
| `test_counter_counts_a_still_board_under_mains_flicker` | Real flickering RGB frames of a still board (`fixtures/head_rgbd/rgb_flicker_*.png`) with a real ToF frame count one pose. |
| `test_captured_pose_carries_judged_detections` | A captured pose has the raw bytes passed with the RGB frame, the judged detections, the paired amplitude and the nearest depth frame. |
| `test_depth_absent_beyond_max_dt` | Depth outside `max_pair_dt_s` or absent leaves `depth` None; the pose is still captured. |
| `test_start_count_continues_numbering` | `start_count=4` numbers the next poses 5 and 6. |
| `test_waiting_until_both_streams` | `status` is `waiting for frames` until frames arrived and one was judged, then the last judged status. |
| `test_depth_paired_within_max_dt` | A depth frame within `max_pair_dt_s` of the amplitude is attached to the pose. |
| `test_depth_history_is_pruned_like_the_amplitude` | Depth frames older than `TOF_HISTORY_S` are dropped. |
| `test_unknown_frame_kind_is_rejected` | An unknown kind raises `ValueError`. |

### `test_head_rgbd_calib_retired.py`

Retirement pin for the ssh-streamed live pose counter (replaced by the web UI Calibrate page).

| Test | What it checks |
|---|---|
| `test_live_counter_files_untracked` | `git ls-files` returns nothing for the three removed live-counter files (counter, relay script and their test file). |
| `test_no_live_counter_references` | `git grep` finds no relay script name, start function or dotted config key of the live counter under `scripts/` and `tests/` (this file excluded). |
| `test_capture_config_has_no_live_field` | `CaptureConfig` has no `live` attribute. |
| `test_config_still_setting_live_fails_naming_it` | A config that still sets the old `live` key under `capture` fails validation and the message names `live`. |

### `test_head_rgbd_calib_frames.py`

Tests for `scripts/head_rgbd_calibration` (config, capture commands, bag reading, still-pair selection). Synthetic bags are written with the `rosbags` writer.

| `test_defaults_load_from_shipped_yaml` | Shipped `scripts/head_rgbd_calibrate.yaml` loads with the documented defaults. |
| `test_unknown_key_rejected_naming_it` | An unknown key fails validation and the message names it. |
| `test_topics_without_mapping_image_rejected` | `topics` and `validation_topics` without the mapping image topic are rejected. |
| `test_duration_range_enforced` | `duration_s` outside 10..600 is rejected. |
| `test_board_matches_marker_generator_charuco` | `board` equals the `charuco` spec of `markers_generate.yaml`. |
| `test_capture_commands_contents`, `test_capture_commands_custom_topics` | ssh target, the exported robot ROS CLI environment before `ros2 bag record`, duration, storage, topics, scp destination and remote `rm -rf`. |
| `test_capture_bag_runs_steps_in_order`, `test_capture_bag_names_failed_step` | Fake runner: record, copy, remove order; a failed step is named in the error. |
| `test_capture_bag_rejects_an_empty_bag_and_still_removes_the_remote_copy` | A copied bag with `message_count: 0` raises "recorded no messages" after all three steps ran. |
| `test_normalise_amplitude_*` | 1st/99th percentiles map to 0/255; an empty frame returns None. |
| `test_read_frames_decodes_everything` | Decoded RGB, amplitude, depth, stamps and camera infos. |
| `test_still_pairs_two_windows`, `test_still_pairs_ignores_skip_start`, `test_still_pairs_none_when_always_moving` | Two still windows give two pairs with dt <= 50 ms; the first `skip_start_s` is ignored; constant motion gives none. |
| `test_rgb_difference_ignores_mains_flicker_of_a_still_scene`, `test_rgb_difference_sees_the_board_move_one_pixel` | On two real flickering frames of a still board the plain difference exceeds 2x `still_rgb_mad` while `rgb_difference` stays under 0.8x; moving only the board by 1 px exceeds it. |
| `test_still_pairs_finds_a_still_board_under_flicker` | Alternating real flickering frames with a still ToF give one pair. |

### `test_head_rgbd_validate.py`

Covers `scripts/head_rgbd_calibration/validate.py` on synthetic data: `depth_accuracy` (flat target at 1.0 m with +12 mm bias and 8 mm noise gives bias 12 +- 2 mm and RMSE within 2 mm of the truth; zeros excluded; no valid pixel raises), `registration_bias`, `valid_ratio`, `edge_alignment` (board in front of a wall: median 0 +- 0.3 px unshifted, 2 +- 0.3 px with the depth shifted 2 px; no board raises), `time_offset` (pan-sweep corner tracks with a true 25 ms / 0 ms offset give 25 / 0 +- 3 ms; slow pairs ignored; a static scene raises), and `check_targets` (limit flags, per-range depth limits, unmeasured targets fail, lower-bound valid ratio).

### `test_head_rgbd_calibrate_cli.py`

Tests for `scripts/head_rgbd_calibrate.py` and `scripts/head_rgbd_calibration/pipeline.py` on synthetic rosbags written with the rosbags writer from `tests/rgbd_synthetic.py` renders.

| Test | What it checks |
|---|---|
| `test_scene_has_enough_views` | The synthetic scene yields 20 board poses seen by both cameras. |
| `test_calibrate_writes_three_files_matching_truth` | `calibrate` on a 20-still-pose bag writes RGB, ToF and extrinsics files whose intrinsics are within tolerance and extrinsic within 2 mm / 0.2 deg, with the lens position and SDK differences recorded. |
| `test_calibrate_writes_report` | The markdown report lists RMS, views, SDK differences, plane bias and lens position. |
| `test_too_few_still_poses_exits_one_and_writes_nothing` | Five poses exit 1 naming the view count and write no file. |
| `test_rms_limit_failure_names_limit_and_writes_nothing` | An exceeded `limits.*` key exits 1 naming it and writes no file. |
| `test_urdf_rewrites_tof_joint_and_check_is_read_only` | `urdf --check` leaves the URDF byte-identical; `urdf` rewrites `tof_joint` and the drift afterwards is 0. |
| `test_validate_passes_on_accurate_distance_bags` | Distance bags within the targets print a passing table and exit 0. |
| `test_validate_fails_when_bias_exceeds_target` | A 40 mm bias at 1 m prints FAIL and exits 1. |
| `test_validate_missing_distance_bag_exits_one` | A missing `<session>-<d>m` bag exits 1 naming it. |
| `test_capture_runs_only_record_copy_remove` | `capture` runs exactly the ssh record, scp copy and ssh remove commands and nothing else. |
| `test_capture_live_key_rejected` | A config setting the removed `live` key under `capture` fails validation and the message names `live`. |
| `test_capture_prints_bag_path` | `capture` with a fake runner runs exactly the ssh record, scp copy and ssh remove commands, nothing else, and prints the bag path. |
| `test_capture_validation_names_bag_and_uses_validation_topics` | `capture-validation 0.5m` names the bag `<session>-0.5m`, records the validation topics and runs no live counter. |
| `test_capture_validation_rejects_unknown_suffix` | An unknown suffix is rejected. |
| `test_session_defaults_to_newest_calibration_bag` | The default session is the newest bag, validation bags excluded. |
| `test_sweep_offset_is_zero_for_synchronous_streams` | A synchronous synthetic sweep gives a time offset under 10 ms. |
| `test_wall_metrics_on_registered_board` | Registration bias, edge alignment and valid ratio of a synthetic registered board are finite and in range. |

### `test_head_rgbd_urdf_handoff.py`

Tests for `shared/ros2_common/head_rgbd_calibration/urdf_handoff.py` on a copy of `nodes/web_ui/urdf/head.urdf`.

| Test | What it checks |
|---|---|
| `test_read_chain_parses_joints` | The camera, ToF and optical joint origins are read from the URDF. |
| `test_missing_joint_raises` | A URDF without `tof_joint` raises. |
| `test_rewrite_reproduces_extrinsic` | The rewritten `tof_joint` origin, composed with the other joints, reproduces the calibrated transform. |
| `test_diff_touches_only_origin_and_comment` | A text diff shows only the `tof_joint` origin and the calibration comment changed. |
| `test_second_rewrite_replaces_comment` | A second rewrite replaces the stamped comment instead of adding one. |
| `test_drift_equals_injected_offset` | `check_drift` reports the translation and rotation offset that was injected. |

### `test_head_rgbd_calibration_deploy.py`

Static invariants of the `head_rgbd_calibration` deploy (node type and `ros2_nodes` entry in `ansible/group_vars/client.yml`, `ansible/playbooks/tasks/head_rgbd_calibration_setup.yml`, the apply units, `deploy_nodes_client.yml`).

| Test | What it pins |
|---|---|
| `test_node_type_and_tier` | The node type fields, the config env and the background tier keys (cpu and io weight 100, `oom_score_adjust` 0, own memory cap). |
| `test_ros2_nodes_entry_matches_defaults` | The `ros2_nodes` entry is present and enabled after `head_rgbd_cpp`, and its config equals the node's shipped default YAML. |
| `test_state_dirs_owned_by_node_user_mode_0755` | The state, `sessions` and `applied` directories are created owned by the node user, mode 0755. |
| `test_seed_copies_never_overwrite_and_need_existing_source` | Seeding the applied calibration files copies only when the source exists and never overwrites (`force: false`). |
| `test_no_task_clears_state_dir` | No setup task removes, forces over or rewrites state dir content. |
| `test_apply_units_content` | The apply path unit watches `applied.json` and the oneshot service try-restarts the head camera, RGB-D, state publisher and arducam_tof units; both units are reloaded, enabled and started. |
| `test_playbook_setup_before_deploy` | The setup include precedes `Deploy head_rgbd_calibration` in the client playbook, with the right tags and node name. |
| `test_arducam_tof_reads_the_applied_tof_calibration` | arducam_tof's `tof_calibration_file` is the same applied `tof_calibration.yaml` as head_rgbd_cpp's, and `depth_is_radial` is true. |
| `test_http_port_unique_in_client_vars` | The node's HTTP port appears exactly once in `client.yml`. |

### `rgbd_synthetic.py`

Shared helper module (no tests) for the head RGB-D calibration and validation tests: `SyntheticCamera` pinholes for the
576x324 RGB and 240x180 ToF, `Extrinsic`, board poses, and renderers (`render_rgb`, `render_tof_amplitude`,
`render_depth`) that draw the ChArUco board through a known pose so results compare with ground truth. Used by
`test_head_rgbd_calib_intrinsics.py`, `test_head_rgbd_calib_extrinsics.py`, `test_head_rgbd_validate.py` and
`test_head_rgbd_calibrate_cli.py`.

### `test_map3d_topics.py`

The map3d topic contract `nodes/rtabmap_mapping/config/map3d_topics.yaml` (design D2, D3), typed by `nodes/rtabmap_mapping/map3d_topics.py` (`load_map3d_topics`): the shipped yaml loads and every rule rejects a crafted bad document.

| Test | Description |
|---|---|
| `test_shipped_yaml_loads` | The shipped contract loads and has uplink and downlink streams. |
| `test_unknown_keys_forbidden` | An unknown key anywhere is rejected. |
| `test_uplink_dest_prefix`, `test_downlink_dest_prefix` | Uplink dests start with `/uplink/`; downlink dests are `/map3d/*`. |
| `test_dest_equals_source_rejected`, `test_dest_equals_other_stream_source_rejected` | A relay can never feed itself or another stream's source (no loops). |
| `test_dests_unique` | Two streams never publish the same dest. |
| `test_rate_positive`, `test_budget_bytes_positive` | Rates and byte budgets must be positive. |
| `test_null_rate_requires_expected_rate` | A pass-through stream (`rate_hz: null`) must give the `expected_rate_hz` used for budgeting. |
| `test_tf_static_must_be_transient_local` | `/tf_static` must be transient_local. |
| `test_large_stream_with_two_readers_rejected`, `test_small_stream_may_have_two_readers`, `test_small_flag_must_be_true_for_small_budget` | Every large uplink stream has exactly one server reader (D2); only streams flagged `small` (CameraInfo) may have two, and the flag must match the budget. |
| `test_every_uplink_has_a_reader` | Each uplink stream names at least one server reader. |
| `test_bandwidth_within_budget`, `test_estimated_mbit_s_arithmetic` | The summed uplink and downlink estimate stays within `budget.*_mbit_s_max` and the arithmetic is bytes x rate x 8. |
| `test_head_rgb_uplink_is_mapping_stream_passthrough`, `test_depth_uplink_is_registered_depth_not_tof`, `test_head_rgbd_entries_unthrottled` | The RGB uplink is the head camera mapping JPEG, the depth uplink is the registered `/head_rgbd/depth/image_raw` (compressedDepth), and head_rgbd pairs are never throttled so a pair is not split. |
| `test_documented_values` | The documented rates, budgets and frames (`map3d_frame: rtabmap_map`) equal the shipped yaml. |

### `test_map3d_uplink_config.py`

The client `map3d_uplink` node type (`nodes/map3d_uplink`, D2): the single static-peer process carrying the map3d streams.

| Test | Description |
|---|---|
| `test_one_node_per_contract_entry_with_unique_names` | The launch builds one composable node per contract entry, with unique names. |
| `test_every_published_topic_is_a_contract_dest` | Every published topic is a contract dest. |
| `test_plugin_rate_and_lazy_per_entry` | Entries with a rate get a `ThrottleNode`, the others a `RelayNode`; all lazy except `/tf_static`. |
| `test_depth_republisher_has_no_throttle_in_front` | The registered depth goes through an `image_transport` Republisher (raw to compressedDepth) with no throttle ahead of it. |
| `test_cloud_downlink_throttled_at_a_tenth_hz_lazy` | `/map3d/cloud` is throttled to 0.1 Hz and lazy. |
| `test_tf_static_qos_overrides_on_both_sides` | `/tf_static` carries reliable, transient_local QoS overrides on subscription and publisher. |
| `test_launch_file_defines_one_container_named_map3d_uplink` | One container named `map3d_uplink`, nothing else. |
| `test_node_type_env_apt_tier_and_paths` | `group_vars/client.yml`: `ROS_STATIC_PEERS`, apt packages, background tier with `memory_low` / `memory_max`, source paths including the contract. |
| `test_ros2_nodes_entry_present_and_enabled` | The `ros2_nodes` entry is present and enabled. |
| `test_head_camera_mapping_stream_and_rgbd_node_present` | The head camera mapping stream (2 Hz, 576x324) and `head_rgbd_cpp` it depends on are configured. |
| `test_playbook_step_after_head_camera_cpp` | The client playbook deploys `map3d_uplink` after `head_camera_cpp`. |
| `test_readme_documents_purpose_lazy_and_measuring` | The node README explains the single static-peer process, lazy behaviour and how to measure bandwidth. |

### `test_rtabmap_mapping_config.py`

The server `rtabmap_mapping` launch and parameters (design D1, D5-D9, U1, U2, U6). `build_container` is called directly; the docker probe `scripts/rtabmap-launch-probe.sh` proves the same parameters load in Jazzy on arm64.

| Test | Description |
|---|---|
| `test_params_yaml_sets_every_design_value`, `test_every_param_value_is_commented`, `test_rtabmap_component_gets_every_design_value` | Every D7/D8 parameter (memory limits, no features, odom from TF, ...) is in `config/params.yaml`, commented, and reaches the rtabmap component. |
| `test_grid_is_depth_only_and_scan_is_still_subscribed`, `test_no_own_links` | The grid is built from depth only while the scan is still subscribed, and RTAB-Map adds no own scan or proximity links (alignment with slam_toolbox, D7). |
| `test_one_component_container_with_sync_and_core`, `test_no_register_node_loaded` | One container with `rgbd_sync` and the rtabmap core; the ToF is not registered again on the server. |
| `test_rgbd_sync_exact_sync_on_head_rgbd_pair`, `test_core_wrapper_inputs`, `test_services_are_at_the_rtabmap_namespace`, `test_sync_output_feeds_core` | Exact-sync inputs, core inputs, service names and the sync-to-core wiring. |
| `test_qos_follows_the_contract`, `test_subscribed_topics_equal_contract_readers` | QoS and the set of subscribed topics equal the contract's `rtabmap_mapping` readers. |
| `test_global_tf_remaps`, `test_no_tf_publisher_configured`, `test_override_cannot_enable_tf` | `/tf` remaps are process-wide to the uplink topics and RTAB-Map never publishes TF, even from an override. |
| `test_override_cannot_change_fixed_inputs` | A config override cannot change the fixed input topics. |
| `test_launch_description_starts_one_container_and_no_other_process` | The launch description starts exactly one container. |
| `test_restore_*` (4), `test_database_path_is_live_db_and_never_deleted`, `test_override_cannot_delete_db_on_start`, `test_default_database_dir` | The saved database is restored when newer than the live one (never over a newer live one), a fresh start is allowed, the live database is never deleted and lives in `/var/lib/ros2/rtabmap`. |
| `test_override_merges_key_by_key`, `test_override_ignored_when_missing_or_empty`, `test_override_rejects_unknown_top_level_key`, `test_config_env_names_and_default` | The `RTABMAP_MAPPING_CONFIG` override file semantics. |
| `test_readme_sections`, `test_readme_documents_scan_grid_override_and_rebend_behaviour`, `test_no_parking_wording` | The node README has the required sections and no deferral wording. |

### `test_rtabmap_mapping_deploy.py`

Deploy wiring of the server `rtabmap_mapping` node type (D1, D8).

| Test | Description |
|---|---|
| `test_node_type_fields` | Native launch/config-only type with `src_paths` under `nodes/rtabmap_mapping`. |
| `test_apt_list_and_forbidden_packages` | The split `ros-jazzy-rtabmap-*` packages are installed; the `rtabmap-ros` metapackage, `rtabmap-viz` and Qt are not. |
| `test_quota_and_cap_without_tier_keys` | Server rule: `cpu_quota` 200% and `memory_max` 900M, no tier keys. |
| `test_no_static_peer` | No `ROS_STATIC_PEERS` (only the client uplink bridges the hosts). |
| `test_ros2_nodes_entry` | The `ros2_nodes` entry is present and enabled. |
| `test_db_dir_task` | `playbooks/tasks/rtabmap_db_dir.yml` creates `/var/lib/ros2/rtabmap`. |
| `test_playbook_steps_and_order` | The server playbook deploys the node after its dependencies. |

### `test_map3d_manager_deploy.py`

Deploy wiring of the server `map3d_manager_cpp` node type (G3).

| Test | Description |
|---|---|
| `test_node_type_fields` | Native `ros2 run map3d_manager_cpp map3d_manager_node`, empty `src_paths` (the build stamp restarts it). |
| `test_colcon_sources_common_then_node_release_jobs_2` | Colcon local sources `ros2_common_cpp` then the node, Release, `-j2`. |
| `test_apt_covers_every_package_xml_dependency` | `apt_packages` covers both `package.xml` files. |
| `test_quota_and_cap_without_tier_keys` | Server rule: `cpu_quota` 20% and `memory_max` 128M, no tier keys. |
| `test_ros2_nodes_entry_after_rtabmap_mapping` | The entry follows `rtabmap_mapping`. |
| `test_config_defaults_equal_the_contract_manager_topics` | The manager's config topics equal the manager side of the topic contract. |
| `test_playbook_deploys_after_rtabmap_mapping` | The playbook step follows `rtabmap_mapping`. |

### Changed existing tests (3D mapping)

- `test_dds_discovery_config.py`: `map3d_uplink` is the third and last client static-peer type (see above).
- `test_unit_cgroups.py`: `map3d_uplink` is in the background tier with `head_camera_cpp` and `head_rgbd_cpp`.
- `test_cpp_node_deploy.py`: `test_server_cpp_node_builds_two_local_sources_and_covers_their_package_xml` for `map3d_manager_cpp`.
- `test_mcp_server_config.py`: `test_mcp_server_map3d_topics_match_the_topic_contract` (see above) and `test_mcp_server_readme_documents_map3d_tools`.
- `test_monitoring_dashboards.py`: `assert_3d_mapping_row` on `robot-slam.json`: a "3D mapping" row with server unit CPU and memory (`rtabmap_mapping`, `map3d_manager`, `host="server"`), client CPU for `map3d_uplink`, `head_rgbd_cpp`, `head_camera_cpp`, server memory available, and WiFi throughput in bit/s with an 8 Mbit/s uplink budget threshold line; server-only panels are exempt from the `host="client"` selector rule.

### `test_head_remount_runbook.py`

Static pins for `docs/runbooks/head-remount-floor-check.md`, the robot runbook after the head remount (z-depth, pan sign, per-cell temporal median, 10 cm ToF band). Reads docs and YAML only.

| Test | Covers |
|---|---|
| `test_runbook_has_every_section_in_order` | The eight sections Preconditions to Results, in order. |
| `test_deploy_uses_the_script_never_ansible_playbook` | Deploys name `scripts/deploy-nodes.sh`, tee to `.logs/`, the ansible-deploy skill and all six nodes; no `ansible-playbook`. |
| `test_preconditions_name_the_setup_and_the_agent_session_check` | Not on blocks, flat floor, ChArUco board, agent session check before mcp_server. |
| `test_pan_direction_check` | +0.3 rad turns the head left (CCW), user confirms. |
| `test_rezero_and_recalibration_reference_their_runbooks` | Links to servo-torque-neutral and head-rgbd-fusion; plane bias +-10 mm and the 40 mm limit. |
| `test_floor_thresholds_and_scraper_workflow` | +-1 cm, 0.6 deg, 60 s, topic_scraper_collect.py, ground_temporal_cells. |
| `test_costmap_band_equals_nav2_tof_mark` | The 0.10 band equals nav2_params.yaml tof_mark; 30 s clear check. |
| `test_temporal_defaults_match_ground_estimator_config` | Quoted cell size, window and min frames equal config/ground_estimator.yaml. |
| `test_every_scripts_path_exists` | Every `scripts/` path (and `docs/` path) named exists. |
| `test_results_section_has_a_check_table` | The Results table rows. |
| `test_other_docs_link_and_state_z_depth` | ground-estimation links the runbook; fusion runbook and methodology state z-depth and the plane bias effect. |
| `test_memory_records_the_decision` | The MEMORY.md entry. |
| `test_tests_readme_lists_this_file` | This section exists. |
