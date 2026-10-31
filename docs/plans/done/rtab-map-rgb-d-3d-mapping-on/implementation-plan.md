# RTAB-Map RGB-D 3D mapping on the server implementation plan

## Agreed rung

Rung 1 of plan.md, migrated from the epic format: the steps are the epic's tasks.

## Approach

Migrated from the epic format: one step per task file, in dependency order, each with its status, files, verify, verified head and every body section.

## Chunk 1: migrated tasks (model: opus)

### S1: Map3d manager config

- Status: done
- Files: `nodes/map3d_manager_cpp/package.xml`, `nodes/map3d_manager_cpp/CMakeLists.txt`, `nodes/map3d_manager_cpp/include/map3d_manager_cpp/config.hpp`, `nodes/map3d_manager_cpp/src/config.cpp`, `nodes/map3d_manager_cpp/test/test_config.cpp`, `nodes/map3d_manager_cpp/include/map3d_manager_cpp/command_json.hpp`, `nodes/map3d_manager_cpp/src/command_json.cpp`, `nodes/map3d_manager_cpp/test/test_command_json.cpp`
- Verify: `scripts/cpp-node-test.sh map3d_manager_cpp`
- Verified head: 0807e2648be212ce4fc612dc79d8cb3f413b5f00

#### Requirement

- New ament_cmake package `map3d_manager_cpp` (layout, flags and linters of map_guard_cpp; C++17, -Werror). `package.xml` declares everything the finished node needs: `ros2_common_cpp`, `rclcpp`, `std_msgs`, `std_srvs`, `rtabmap_msgs`, `sensor_msgs`, `yaml-cpp`, `nlohmann-json-dev`; test deps as map_guard_cpp. CMakeLists builds a static core library (config, command_json, later status) and one gtest per test file.
- `config.hpp/.cpp` (D9): `Config` loaded from `MAP3D_MANAGER_CONFIG` (default `/etc/ros2/map3d_manager/config.yaml`) through `ros2_common_cpp::load_config_or_exit` (unknown keys are errors): `command_topic` `/uplink/map3d/command`, `map_reset_topic` `/uplink/map_guard/map_reset`, `result_topic` `/map3d/result_server`, `status_topic` `/map3d/status_server`, `rtabmap_namespace` `/rtabmap`, `database_dir` `/var/lib/ros2/rtabmap`, `status_period_s` 1.0, `service_timeout_s` 10.0, `input_stale_s` 3.0, `correction_warn_m` 0.05, `correction_warn_deg` 2.0, `rgb_info_topic` `/uplink/head_rgbd/rgb/camera_info`, `depth_info_topic` `/uplink/head_rgbd/depth/camera_info` (the head_rgbd_cpp pair, epic head-rgb-d-fusion-of-camera), `rtabmap_unit` `ros2-rtabmap_mapping.service` (for cgroup memory). Topic defaults equal the map3d topic contract (checked by tests/test_map3d_manager_deploy.py later).
- `command_json.hpp/.cpp` (the command contract, header comment is the spec): command `{"request_id": str (1..64 chars), "action": "save"|"reset"|"pause"|"resume", "name": str (optional, [A-Za-z0-9_-]{1,64}, default "slam_map")}` -> `parse_command(const std::string&) -> std::variant<Command, ParseError>` (ParseError carries the request_id when present so an `invalid` result can still be matched; no request_id -> dropped); result `{"request_id", "status": "ok"|"error"|"busy"|"invalid"|"timeout", "message", "action", "db_path"?, "db_size_mb"?}` -> `to_json(const Result&)`.
- Tests first, seen failing: `test/test_config.cpp` (defaults, every bound, unknown key) and `test/test_command_json.cpp` (each action, bad name, missing/long request_id, unknown action -> invalid with id, not JSON -> dropped; result serialisation with and without db fields).

#### Acceptance conditions

- a1: config defaults and validation errors name their keys [evidence: command: scripts/cpp-node-test.sh map3d_manager_cpp]
- a2: commands parse into save/reset/pause/resume or an invalid result that keeps the request id [evidence: command: scripts/cpp-node-test.sh map3d_manager_cpp]
- a3: the package builds and its tests and linters pass in docker [evidence: command: scripts/cpp-node-test.sh map3d_manager_cpp]

#### Files

- `nodes/map3d_manager_cpp/package.xml`
- `nodes/map3d_manager_cpp/CMakeLists.txt`
- `nodes/map3d_manager_cpp/include/map3d_manager_cpp/config.hpp`
- `nodes/map3d_manager_cpp/src/config.cpp`
- `nodes/map3d_manager_cpp/test/test_config.cpp`
- `nodes/map3d_manager_cpp/include/map3d_manager_cpp/command_json.hpp`
- `nodes/map3d_manager_cpp/src/command_json.cpp`
- `nodes/map3d_manager_cpp/test/test_command_json.cpp`

#### Interfaces

- `nodes/map3d_manager_cpp/include/map3d_manager_cpp/command_json.hpp` (`Command`, `Result`, `parse_command`, `to_json`): the `map3d-command-contract` part used by web_ui.
- `nodes/map3d_manager_cpp/include/map3d_manager_cpp/config.hpp`: the `map3d-manager-config` part.

#### Expected outcome

`scripts/cpp-node-test.sh map3d_manager_cpp` ends `cpp-node-test: map3d_manager_cpp PASS`.

#### Steps

- Write package.xml, CMakeLists and both tests with stub sources; run `scripts/cpp-node-test.sh map3d_manager_cpp`; see `FAIL`.
- Implement; rerun until `PASS`.
- Commit `feat(map3d_manager_cpp): package, config and command contract`.

#### Notes

- The docker image is rebuilt from every package.xml; `rtabmap_msgs` resolves through rosdep (apt ros-jazzy-rtabmap-msgs).
- New non-web nodes are C++ (MEMORY: Python nodes stay only for web_ui and mcp_server).

### S2: Map3d topic contract

- Status: done
- Files: `nodes/rtabmap_mapping/config/map3d_topics.yaml`, `nodes/rtabmap_mapping/map3d_topics.py`, `tests/test_map3d_topics.py`, `nodes/rtabmap_mapping/README.md`
- Verify: `uv run pytest tests/test_map3d_topics.py -q && uv run poe lint`
- Verified head: c4c20988e8f637a4a18fc838f830a86e7174f50f

#### Requirement

- New directory `nodes/rtabmap_mapping` (launch/config-only node, no Python package of its own). `nodes/rtabmap_mapping/config/map3d_topics.yaml` is the single definition (D2, D3) of the cross-host streams, with three sections:
  - `uplink` (client -> server), one entry per stream with `source`, `dest`, `type`, `rate_hz` (null = pass every message), `qos` (`reliability`, `durability`, `depth`), `transport` (`raw` or `compressedDepth`), `lazy` (bool) and `budget_bytes` (expected bytes per message, documented with its source): the head RGB-D pair from head_rgbd_cpp (epic head-rgb-d-fusion-of-camera, exact-stamp pairs at the 2 Hz mapping rate, relayed unthrottled so a pair is never split, D5): RGB `/head_rgbd/rgb/image_raw/compressed` -> `/uplink/head_rgbd/rgb/image_raw/compressed` (rate null, expected_rate_hz 2.0, budget 40000), `/head_rgbd/rgb/camera_info` -> `/uplink/head_rgbd/rgb/camera_info` (rate null, expected 2.0), registered depth `/head_rgbd/depth/image_raw` -> `/uplink/head_rgbd/depth/image_raw/compressedDepth` (rate null, expected 2.0, transport compressedDepth, budget 45000), `/head_rgbd/depth/camera_info` -> `/uplink/head_rgbd/depth/camera_info` (rate null, expected 2.0); no raw ToF stream is uplinked; `/scan_filtered` -> `/uplink/scan_filtered` (rate null, budget 12000), `/tf` -> `/uplink/tf` (rate null, budget 200, expected 100 msg/s), `/tf_static` -> `/uplink/tf_static` (transient_local, depth 100), `/map_guard/map_reset` -> `/uplink/map_guard/map_reset`, `/map3d/command` -> `/uplink/map3d/command`.
  - `downlink` (server -> client): `/rtabmap/cloud_map` -> `/map3d/cloud` (rate 0.1, lazy true, budget 1500000), `/map3d/status_server` -> `/map3d/status` (rate null, budget 600), `/map3d/result_server` -> `/map3d/result` (reliable, depth 10).
  - `frames`: `map_frame: map` (slam_toolbox, used by RTAB-Map as its odometry frame, D7), `map3d_frame: rtabmap_map` (RTAB-Map's map frame and the frame of `/map3d/cloud`, identical to `map` by D7), `base_frame: base_link`, `rgb_frame: head_camera_optical_frame`, `depth_frame: tof_optical_frame`; and `budget`: `uplink_mbit_s_max: 8.0`, `downlink_mbit_s_max: 2.0` (my call, D3).
- Every uplink entry lists `server_readers` (the server processes that subscribe it: `rtabmap_mapping` and/or `map3d_manager`); validation allows more than one only for entries flagged `small: true` (budget_bytes x rate < 1 KB/s), because each extra remote reader process is one more copy over WiFi (D2). Readers: RGB, depth, scan, tf, tf_static -> `rtabmap_mapping`; the two camera_infos -> both (small); map_reset and map3d command -> `map3d_manager`.
- `nodes/rtabmap_mapping/map3d_topics.py` (stdlib + pyyaml + pydantic, importable by tests and by both launch files with `sys.path` insertion): pydantic models `Stream`, `Frames`, `Budget`, `Map3dTopics` (unknown keys forbidden), `load_map3d_topics(path: Path | None = None) -> Map3dTopics` (default: the yaml next to it) and `estimated_mbit_s(streams: list[Stream], expected_rates: dict[str, float]) -> float` (sum of budget_bytes x rate x 8 / 1e6; a null rate takes the stream's `expected_rate_hz` field, required when `rate_hz` is null).
- Validation in the models: every uplink `dest` starts with `/uplink/`, every downlink `dest` with `/map3d/`; no `dest` equals any `source` (relay loop and duplicate publishers on the client graph, D2); dests are unique; `rate_hz` > 0 when set; `budget_bytes` > 0; `/tf_static` is transient_local.
- Tests first in `tests/test_map3d_topics.py`, seen failing before the module exists: the shipped yaml loads; each rule above rejects a crafted bad document with a message naming the field; the computed uplink estimate is <= `uplink_mbit_s_max` and the downlink estimate <= `downlink_mbit_s_max`; a large stream with two server readers is rejected; the head RGB uplink is head_rgbd_cpp's pass-through of the mapping stream, never `/head_camera/image_raw` or the 30 fps compressed topic; the depth uplink is `/head_rgbd/depth/image_raw`, never `/tof/depth/image_raw`; the four head_rgbd entries have no rate (unthrottled pairs).
- `nodes/rtabmap_mapping/README.md` (first version): purpose of the node, a `## Topics` section with the uplink and downlink tables generated by hand from the yaml (source, dest, rate, budget) and the bandwidth budget arithmetic of D3.

#### Acceptance conditions

- a1: the shipped contract loads and every uplink dest is under /uplink/ and every downlink dest under /map3d/ [evidence: test: tests/test_map3d_topics.py]
- a2: relay loops, duplicate dests, a non-latched /tf_static and a second server reader of a large stream are rejected [evidence: test: tests/test_map3d_topics.py]
- a3: the estimated uplink and downlink bandwidth stay within the budget in the file [evidence: test: tests/test_map3d_topics.py]
- a4: only the head_rgbd RGB-D pair (2 Hz mapping RGB plus registered depth, unthrottled) crosses the link, never raw or 30 fps RGB or raw ToF depth [evidence: test: tests/test_map3d_topics.py]
- a5: /scan_filtered is uplinked to /uplink/scan_filtered, decided by the user on 2026-10-18 [evidence: test: tests/test_map3d_topics.py]
- a6: the README documents the streams and the budget [evidence: doc-section: nodes/rtabmap_mapping/README.md#Topics]

#### Files

- `nodes/rtabmap_mapping/config/map3d_topics.yaml`
- `nodes/rtabmap_mapping/map3d_topics.py`
- `tests/test_map3d_topics.py`
- `nodes/rtabmap_mapping/README.md`

#### Interfaces

- `nodes/rtabmap_mapping/config/map3d_topics.yaml` and `nodes/rtabmap_mapping/map3d_topics.py:load_map3d_topics` / `Map3dTopics` / `estimated_mbit_s`: the `map3d-topics` contract read by the uplink launch, the RTAB-Map launch, web_ui, mcp_server and map3d_manager_cpp config tests.

#### Expected outcome

`uv run pytest tests/test_map3d_topics.py -q` passes; the contract states every cross-host stream with its budget.

#### Steps

- Write tests/test_map3d_topics.py; run `uv run pytest tests/test_map3d_topics.py -q`; see it fail on the missing module.
- Write the yaml and map3d_topics.py; rerun until green; `uv run poe lint`.
- Write the README; commit `feat(rtabmap_mapping): cross-host map3d topic contract`.

#### Notes

- Root ruff excludes `nodes/`; the module is still linted by the test importing it. Keep it dependency-light: launch files import it on the Pis with the system python (pyyaml and pydantic are available from apt `python3-yaml`, `python3-pydantic`; the node-type tasks add them to apt_packages).
- Budget numbers come from design D3; the runbook replaces them with measured values only through a later commit.

### S3: Mcp map3d grid model

- Status: done
- Files: `nodes/mcp_server/mcp_server/map3d.py`, `nodes/mcp_server/tests/test_map3d.py`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- Verified head: 6c392981d5762485c1c7c5e394ade60aad012e98

#### Requirement

- `map3d.py` (numpy/scipy only, no ROS imports, D11 and U6): `VoxelMap.from_points(xyz: np.ndarray, voxel_m: float, stamp_s: float, frame_id: str)` (occupied voxel set keyed by integer cells, column index for x/y lookups), `LidarGrid.from_occupancy(data, width, height, resolution, origin_xy, occupied_thresh=65)` (the /map grid), and pure queries:
  - `query_at(vmap, lidar, x, y, radius_m, floor_band_m=0.05) -> AtResult`: within the radius, measured obstacle voxels above the floor band (count, min/max z of their tops, highest point, colour summary when colours are given), measured floor voxels count, lidar-only occupied cells (occupied in /map with no measured voxel in the column) count, and `source` `depth_3d` | `lidar_2d` | `both` | `unknown`. Lidar-only obstacles report `height_m: null` and `min_height_m` = the lidar plane height passed in (from config), never a guessed height.
  - `free_space(vmap, lidar, x, y, radius_m, z_min, z_max, sectors=8) -> FreeSpaceResult`: per sector, the distance to the nearest measured obstacle voxel inside [z_min, z_max] or lidar-only occupied cell, `free_m` = radius when none, and `observed` (fraction of the sector's cells seen by depth or lidar; `free_m: null` when below 0.2, an unknown sector is never reported as free).
  - `status_summary(vmap, lidar) -> dict` (voxel count, bounds, age).
- Frames: the cloud must be in the configured map3d frame (`rtabmap_map`, identical to `map` by D7); `from_points(..., expected_frame)` refuses another frame_id with ValueError.
- `alignment_agreement(vmap, lidar, lidar_plane_height_m, band_m=0.10, tolerance_cells=1) -> float | None` (D7 check): of the measured obstacle columns that have a voxel within +-band_m of the lidar plane, the fraction lying on (within tolerance_cells of) an occupied /map cell; `None` when fewer than 50 such columns exist (not enough evidence, never a made-up score).
- Tests first in `tests/test_map3d.py`, seen failing: a synthetic table (top at 0.72 m) reports max height 0.72 +- voxel; a wall only in /map reports `lidar_2d` with null height; a column with both reports `both`; free_space sees the table inside the robot band and not a 2 m high shelf above `z_max`; unseen sectors give null; another frame is refused; empty map answers `unknown`; alignment_agreement is ~1.0 for a cloud whose walls coincide with /map walls, drops below 0.5 for the same cloud shifted 0.3 m, and is None with too few columns.

#### Acceptance conditions

- a1: query_at reports measured heights and lidar-only cells with null height [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_map3d.py -q]
- a2: free_space respects the height band and never calls unobserved space free [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_map3d.py -q]
- a3: a cloud in a frame other than the configured map3d frame is refused [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_map3d.py -q]
- a4: alignment agreement separates an aligned cloud from a shifted one and abstains on too little evidence [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_map3d.py -q]
- a5: mcp_server tests and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/mcp_server/mcp_server/map3d.py`
- `nodes/mcp_server/tests/test_map3d.py`

#### Interfaces

- `nodes/mcp_server/mcp_server/map3d.py`: `VoxelMap`, `LidarGrid`, `query_at`, `free_space`, `status_summary`, `alignment_agreement`, result dataclasses `AtResult`, `FreeSpaceResult`: the `map3d-voxel-query` part.

#### Expected outcome

Pure, fast queries over the 3D map that separate measured 3D structure from lidar-only structure.

#### Steps

- Write `tests/test_map3d.py`; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_map3d.py -q`; see it fail.
- Implement; run `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint`.
- Commit `feat(mcp_server): 3D map voxel model and queries`.

#### Notes

- Reuse `frontiers.MapGrid` for /map decoding if it fits instead of a second OccupancyGrid parser.
- Opus: the honest-unknown rules are where the value is.

### S4: Map3d manager status

- Status: done
- Files: `nodes/map3d_manager_cpp/include/map3d_manager_cpp/status.hpp`, `nodes/map3d_manager_cpp/src/status.cpp`, `nodes/map3d_manager_cpp/test/test_status.cpp`
- Verify: `scripts/cpp-node-test.sh map3d_manager_cpp`
- Verified head: 7776c58165ff33d0a282748aa12b14b63a201a0c

#### Requirement

- `status.hpp/.cpp` (core, no ROS; the header comment is the status contract): `StatusInputs` (now, last mapGraph stamp and its `map_to_odom` translation/yaw, node count from the graph, last info stamp, last rgb/depth camera_info stamps, paused flag, db file size or nullopt, cgroup memory bytes or nullopt, last save {name, stamp} or nullopt, last error string) -> `Status build_status(const StatusInputs&, const Config&)` and `std::string to_json(const Status&)`:
  `{"stamp_s", "state": "mapping"|"paused"|"waiting_for_inputs"|"error", "nodes", "db_size_mb", "memory_mb", "graph_correction_m", "graph_correction_deg", "correction_warn_m", "correction_warn_deg", "correction_ok", "missing_inputs": [..], "last_update_age_s", "last_save": {"name", "stamp_s"} | null, "message"}`.
  Rules: an input is missing when never seen or older than `input_stale_s` (names: `rgb_camera_info`, `depth_camera_info`, `rtabmap_info`); state `waiting_for_inputs` when any is missing and not paused, `paused` when paused, `error` when a last error is set within the last minute, else `mapping`; unknown values are JSON null, never 0; `correction_ok` null until a graph arrived.
- `read_cgroup_memory_bytes(const std::string& unit, const std::string& root="/sys/fs/cgroup/system.slice")` reads `memory.current` (nullopt when absent).
- Tests first in `test/test_status.cpp`, seen failing: each state; stale vs fresh inputs; null fields before data; correction over/under the thresholds (translation norm and |yaw|); cgroup reader with a temp dir; JSON round-trip of every field.

#### Acceptance conditions

- a1: status state, missing inputs and correction flags follow the rules [evidence: command: scripts/cpp-node-test.sh map3d_manager_cpp]
- a2: unknown values serialise as null, never zero [evidence: command: scripts/cpp-node-test.sh map3d_manager_cpp]
- a3: the package builds and its tests and linters pass in docker [evidence: command: scripts/cpp-node-test.sh map3d_manager_cpp]

#### Files

- `nodes/map3d_manager_cpp/include/map3d_manager_cpp/status.hpp`
- `nodes/map3d_manager_cpp/src/status.cpp`
- `nodes/map3d_manager_cpp/test/test_status.cpp`

#### Interfaces

- `nodes/map3d_manager_cpp/include/map3d_manager_cpp/status.hpp` (`StatusInputs`, `Status`, `build_status`, `to_json`, `read_cgroup_memory_bytes`): the `map3d-status-contract` part read by the web UI and mcp_server.

#### Expected outcome

A pure function turns the manager's observations into the status JSON every consumer reads.

#### Steps

- Write test/test_status.cpp; run `scripts/cpp-node-test.sh map3d_manager_cpp`; see `FAIL`.
- Implement; rerun until `PASS`.
- Commit `feat(map3d_manager_cpp): status contract`.

#### Notes

- `graph_correction_*` is RTAB-Map's `MapGraph.map_to_odom`: identity while D7 holds; anything else means RTAB-Map built its own links and the 3D map has left slam_toolbox's frame.

### S5: Map3d uplink node type

- Status: done
- Files: `nodes/map3d_uplink/launch/map3d_uplink.launch.py`, `nodes/map3d_uplink/README.md`, `ansible/group_vars/client.yml`, `ansible/playbooks/deploy_nodes_client.yml`, `tests/test_map3d_uplink_config.py`, `tests/test_dds_discovery_config.py`, `tests/test_unit_cgroups.py`
- Verify: `uv run pytest tests/test_map3d_uplink_config.py tests/test_dds_discovery_config.py tests/test_unit_cgroups.py tests/test_map3d_topics.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: 17781fb9319c6c56228caf2e9c63758602b69035

#### Requirement

- `nodes/map3d_uplink/launch/map3d_uplink.launch.py` (D2): one `component_container` process named `map3d_uplink` whose composable nodes are built by a pure function `build_uplink_nodes(topics: Map3dTopics) -> list[ComposableNodeSpec]` from the map3d topic contract (`nodes/rtabmap_mapping/map3d_topics.py`, imported via sys.path): `topic_tools::ThrottleNode` (throttle_type messages, msgs_per_sec = rate_hz, lazy true) for entries with a rate, `topic_tools::RelayNode` (lazy true except `/tf_static`) for entries without, and for the `compressedDepth` entry (head_rgbd_cpp's registered depth, unthrottled) an `image_transport::Republisher` (in_transport raw, out_transport compressedDepth, PNG level 1) publishing the contract dest base, with no throttle in front of it so RGB and depth pairs stay complete. Downlink entries are built the same way (the cloud throttle at 0.1 Hz, lazy). `/tf_static` uses QoS overrides (reliable, transient_local, depth 100) on both its subscription and publisher so late joiners get every static transform whatever the start order. The launch file defines no other process.
- client.yml: node type `map3d_uplink` (native, `ros2 launch {{ ros2_repo_dest }}/nodes/map3d_uplink/launch/map3d_uplink.launch.py`, `node_src_dir: ""`, `src_paths: [nodes/map3d_uplink, nodes/rtabmap_mapping/config, nodes/rtabmap_mapping/map3d_topics.py]`, apt `ros-jazzy-topic-tools`, `ros-jazzy-image-transport-plugins`, `ros-jazzy-rclcpp-components`, `python3-pydantic`, `python3-yaml`, background tier cpu/io_weight 100, oom_score_adjust 0, memory_low 48M, memory_max 256M until measured, env `ROS_STATIC_PEERS={{ ros2_server_hostname }}`); ros2_nodes entry `map3d_uplink` present/enabled. The head_camera_cpp mapping/focus-lock config and the head_rgbd_cpp node are deployed by the blocker epic head-rgb-d-fusion-of-camera; this task does not change them and its tests assert they are present (`mapping.enabled` and `mapping.lock_focus` true, `af_mode` continuous, `head_rgbd_cpp` enabled).
- `deploy_nodes_client.yml`: step `Deploy map3d_uplink` (resolve_and_deploy.yml, tags `[map3d_uplink, apt, python, build, config]`) after head_camera_cpp.
- `tests/test_dds_discovery_config.py`: the static-peer allowlist becomes leader_bridge, master2master and map3d_uplink (docstring updated); every other node still has no static peer.
- Tests first in `tests/test_map3d_uplink_config.py`, seen failing: `build_uplink_nodes` makes one node per contract entry with the right plugin, rates, lazy flags and remaps; every published topic is a contract dest (no client topic is republished under its own name); tf_static QoS overrides; the ROS 2 launch file parses (`tests/ros_stubs.py`); node type, env, apt list, tier and ros2_nodes entry; the head camera mapping stream with the focus lock and the head_rgbd_cpp node (from the blocker epic) are present; playbook step order; `tests/test_unit_cgroups.py` covers the new type in the background tier.
- `nodes/map3d_uplink/README.md`: purpose, why a single static-peer process (ansible/README "Cross-host"), the stream table pointing at the contract, lazy behaviour (no server subscriber -> nothing subscribed, so head_rgbd_cpp releases its inputs and head_camera_cpp encodes nothing and returns to continuous autofocus), how to measure bandwidth.

#### Acceptance conditions

- a1: the uplink container is generated from the contract with throttles, relays, lazy flags and the compressedDepth republisher [evidence: test: tests/test_map3d_uplink_config.py]
- a2: only contract dests are published, never a client topic under its own name [evidence: test: tests/test_map3d_uplink_config.py]
- a3: map3d_uplink is the only new static-peer process [evidence: test: tests/test_dds_discovery_config.py]
- a4: node type, tier, apt packages and playbook step are deployed by Ansible, and the blocker epic's head camera mapping config and head_rgbd_cpp node are present [evidence: test: tests/test_map3d_uplink_config.py]
- a5: ansible lint and syntax checks pass [evidence: command: uv run poe test-ansible]

#### Files

- `nodes/map3d_uplink/launch/map3d_uplink.launch.py`
- `nodes/map3d_uplink/README.md`
- `ansible/group_vars/client.yml`
- `ansible/playbooks/deploy_nodes_client.yml`
- `tests/test_map3d_uplink_config.py`
- `tests/test_dds_discovery_config.py`
- `tests/test_unit_cgroups.py`

#### Interfaces

- `nodes/map3d_uplink/launch/map3d_uplink.launch.py:build_uplink_nodes` and the deployed unit `ros2-map3d_uplink` ; publishes the contract's `/uplink/*` and `/map3d/*` dests.

#### Expected outcome

`./scripts/deploy-nodes.sh client map3d_uplink` installs one process that bridges only the contract streams.

#### Steps

- Write tests/test_map3d_uplink_config.py and the dds/cgroups test changes; run `uv run pytest tests/test_map3d_uplink_config.py tests/test_dds_discovery_config.py -q`; see them fail.
- Write the launch file, README, client.yml and playbook step; run the verify command until green.
- Commit `feat(map3d_uplink): single static-peer relay for 3D mapping streams`.

#### Notes

- topic_tools copies publisher QoS (reliable/transient_local only if all publishers are), hence the explicit tf_static override (research brief, topic_tools tool_base_node.cpp).
- image_transport::Republisher is a lazy component (image_common jazzy republish.cpp).
- Static-peer rule: ansible/README.md "Cross-host" (one process per route); keep the discovery test's intent.

### S6: Rtabmap server launch

- Status: done
- Files: `nodes/rtabmap_mapping/launch/rtabmap_mapping.launch.py`, `nodes/rtabmap_mapping/config/rtabmap_params.yaml`, `tests/test_rtabmap_mapping_config.py`, `scripts/rtabmap-launch-probe.sh`, `nodes/rtabmap_mapping/README.md`
- Verify: `uv run pytest tests/test_rtabmap_mapping_config.py tests/test_map3d_topics.py -q && uv run poe lint && scripts/rtabmap-launch-probe.sh`
- Verified head: d49c9fa1c2d7c6d78b7c9dd79d281a34a81f817f

#### Requirement

- `nodes/rtabmap_mapping/launch/rtabmap_mapping.launch.py` (D1, D5, D6, D7): reads the map3d topic contract and `config/rtabmap_params.yaml` (overridable by `/etc/ros2/rtabmap_mapping/config.yaml` via `RTABMAP_MAPPING_CONFIG`, same merge rule as slam_toolbox) and starts ONE `component_container` process with global ros arguments `-r /tf:=/uplink/tf -r /tf_static:=/uplink/tf_static` (process-wide, because rtabmap's TransformListener runs an internal node that node-scoped remaps do not reach), loading: `rtabmap_sync::RGBDSync` (rgb <- `/uplink/head_rgbd/rgb/image_raw` with `image_transport: compressed`, rgb/camera_info <- `/uplink/head_rgbd/rgb/camera_info`, depth <- `/uplink/head_rgbd/depth/image_raw` with `depth_image_transport: compressedDepth` (the depth registered on the client by head_rgbd_cpp, epic head-rgb-d-fusion-of-camera, D5), `approx_sync: false` because each pair carries one stamp) and `rtabmap_slam::CoreWrapper` in namespace `rtabmap` (subscribe_rgbd true, subscribe_scan true <- `/uplink/scan_filtered`, subscribe_rgb/depth false). A pure function `build_container(topics, params) -> ContainerSpec` returns the plugins, remaps, parameters and ros_arguments so tests do not need ROS.
- Before starting nodes the launch restores the last saved 3D map (D9): if `<db_dir>/saved.db` exists and is newer than `<db_dir>/live.db` or live.db is missing, it copies saved.db over live.db (both under `database_dir`, default `/var/lib/ros2/rtabmap`); `database_path` = live.db; never deletes the database on start.
- `config/rtabmap_params.yaml` (defaults, every value commented with why): frames `frame_id: base_link`, `odom_frame_id: map` (slam_toolbox's map frame is RTAB-Map's odometry, so its graph is the chain of slam_toolbox poses), `map_frame_id: rtabmap_map`, `publish_tf: false`, `wait_for_transform: 0.2`, `odom_tf_linear_variance: 0.001`, `odom_tf_angular_variance: 0.001`; no own links: `Kp/MaxFeatures: "-1"`, `RGBD/ProximityBySpace: "false"`, `RGBD/OptimizeFromGraphEnd: "true"`, `RGBD/NeighborLinkRefining: "false"`, `Reg/Force3DoF: "true"`; node creation `RGBD/LinearUpdate: "0.1"`, `RGBD/AngularUpdate: "0.1"`, `Rtabmap/DetectionRate: "1"`; grid `Grid/Sensor: "1"` (depth only, so measured 3D points are never mixed with scan points, U6/D13), `Grid/3D: "true"`, `Grid/RangeMax: "4.0"`, `Grid/CellSize: "0.05"`, `Grid/DepthDecimation: "2"`, `Grid/MaxObstacleHeight: "2.0"`, `Grid/RayTracing: "false"`; memory (D8) `Rtabmap/MemoryThr: "400"`, `Rtabmap/TimeThr: "0"`, `Mem/STMSize: "10"`, `Mem/ImagePostDecimation: "2"`, `Mem/BinDataKept: "true"`, `Mem/RawDescriptorsKept: "false"`, `Rtabmap/ImageBufferSize: "1"`; outputs `cloud_output_voxelized: true`, `map_always_update: false`, `latch: false`; QoS: best_effort for images, reliable for scan.
- The scan is subscribed and stored with every node (U1 input set) but feeds neither the grid (Grid/Sensor 1) nor links (D7); the README states that `Grid/Sensor: "2"` is a config override that merges scan points into the 3D cloud and makes them indistinguishable from depth (U6/D13 then rely on slam_toolbox's /map only).
- No process in the launch publishes `/tf` or `/tf_static` (publish_tf false; nothing else broadcasts), so slam_toolbox stays the only `map -> odom` owner (U2).
- `scripts/rtabmap-launch-probe.sh`: in a `ros:jazzy-ros-base` docker container on the host's architecture (arm64 on the M4 Mac, the Pi's), apt-installs `ros-jazzy-rtabmap-slam ros-jazzy-rtabmap-sync ros-jazzy-rtabmap-util ros-jazzy-rtabmap-msgs ros-jazzy-rtabmap-conversions ros-jazzy-image-transport-plugins python3-pydantic` (NOT the rtabmap_ros metapackage or rtabmap_viz), mounts the repo read-only, launches the file for 20 s with a temporary database dir, and fails unless: both components loaded, `/rtabmap/backup` and `/rtabmap/reset` services exist, `ros2 topic info /uplink/head_rgbd/depth/image_raw/compressedDepth` shows a subscriber, no log line reports an unknown/ignored parameter, and nothing is published on `/tf` or `/uplink/tf`. Prints one final line `rtabmap-launch-probe: PASS|FAIL` (pattern: scripts/probe-mppi-overlay-docker.sh).
- Tests first in `tests/test_rtabmap_mapping_config.py`, seen failing: every parameter above; subscribed topics equal the contract dests whose `server_readers` include `rtabmap_mapping`; global TF remaps present; no TF publisher configured; no RegisterNode is loaded and rgbd_sync uses exact sync on the head_rgbd pair; restore logic (saved newer -> copied, live newer -> kept, none -> new db) with a tmp dir; the override merge.
- `nodes/rtabmap_mapping/README.md`: sections `## Pipeline` (the client-registered head_rgbd pair, exact sync, RTAB-Map, why one process), `## Frames and alignment` (D7 and its caveat), `## Parameters` (table), `## Memory` (D8), `## Database` (live/saved, restore on start).

#### Acceptance conditions

- a1: RTAB-Map runs depth-only with odom_frame_id map, map_frame_id rtabmap_map, publish_tf false and no own loop or proximity links [evidence: test: tests/test_rtabmap_mapping_config.py]
- a2: inputs are exactly the contract's uplink dests read by rtabmap_mapping, with process-wide TF remaps [evidence: test: tests/test_rtabmap_mapping_config.py]
- a3: the last saved 3D map is restored before start and never deleted [evidence: test: tests/test_rtabmap_mapping_config.py]
- a4: the Jazzy arm64 apt packages install and the launch loads with no unknown parameter and no TF output [evidence: command: scripts/rtabmap-launch-probe.sh]
- a5: the README documents pipeline, alignment, parameters, memory and database [evidence: doc-section: nodes/rtabmap_mapping/README.md#Frames and alignment]
- a6: /scan_filtered is subscribed (subscribe_scan true) and stored with every node, and uplinked by the contract, decided by the user on 2026-10-18 [evidence: test: tests/test_rtabmap_mapping_config.py]
- a7: the README documents the accepted limitation that older 3D map parts do not re-bend after later 2D loop closures, decided by the user on 2026-10-18 [evidence: doc-section: nodes/rtabmap_mapping/README.md#Frames and alignment]

#### Files

- `nodes/rtabmap_mapping/launch/rtabmap_mapping.launch.py`
- `nodes/rtabmap_mapping/config/rtabmap_params.yaml`
- `tests/test_rtabmap_mapping_config.py`
- `scripts/rtabmap-launch-probe.sh`
- `nodes/rtabmap_mapping/README.md`

#### Interfaces

- `nodes/rtabmap_mapping/launch/rtabmap_mapping.launch.py:build_container`, `nodes/rtabmap_mapping/config/rtabmap_params.yaml` (the `rtabmap-mapping-launch` part); RTAB-Map topics `/rtabmap/cloud_map`, `/rtabmap/mapGraph`, `/rtabmap/info`, services `/rtabmap/backup`, `/rtabmap/reset`, `/rtabmap/pause`, `/rtabmap/resume`.

#### Expected outcome

`scripts/rtabmap-launch-probe.sh` prints `rtabmap-launch-probe: PASS` on the arm64 Mac, proving the apt packages and parameters before any Pi deploy.

#### Steps

- Write tests/test_rtabmap_mapping_config.py; run `uv run pytest tests/test_rtabmap_mapping_config.py -q`; see it fail.
- Write the launch file and params; rerun until green; `uv run poe lint`.
- Write scripts/rtabmap-launch-probe.sh; run it; expect `rtabmap-launch-probe: PASS` (first run downloads the image and PCL/VTK/gtsam; minutes).
- Write the README; commit `feat(rtabmap_mapping): depth-only RTAB-Map aligned to slam_toolbox`.

#### Notes

- Research (scratchpad nav3d-rtabmap-docs.md, recorded in design.md): publish_tf false stops only the broadcast; the graph would still diverge after own loop closures, hence no own links; depth must be an integer fraction of RGB and registered (head_rgbd_cpp's 288x162 is half the 576x324 RGB); Grid/Sensor 2 mixes scan and depth indistinguishably; there is no scan_map topic.
- If the probe shows a parameter name differs in 0.23.13, fix the name and say so in the commit; never drop a parameter silently.
- Opus: the parameter set is where the design's correctness lives.

### S7: Map3d manager node

- Status: done
- Files: `nodes/map3d_manager_cpp/include/map3d_manager_cpp/map3d_manager_node.hpp`, `nodes/map3d_manager_cpp/src/map3d_manager_node.cpp`, `nodes/map3d_manager_cpp/src/main.cpp`, `nodes/map3d_manager_cpp/test/test_node.cpp`, `nodes/map3d_manager_cpp/CMakeLists.txt`, `nodes/map3d_manager_cpp/README.md`
- Verify: `scripts/cpp-node-test.sh map3d_manager_cpp`
- Verified head: 2954fb8e2f43ee7c98cc3a71b4d20d9243098a76

#### Requirement

- `Map3dManagerNode` (rclcpp, executable `map3d_manager_node`, D9): subscribes `command_topic` (String, reliable depth 10), `map_reset_topic` (std_msgs/Empty), `<ns>/mapGraph` (rtabmap_msgs/MapGraph, for map_to_odom and node count), `<ns>/info` (rtabmap_msgs/Info, liveness only), the two camera_info topics (best_effort depth 1); publishes `result_topic` (String JSON, reliable depth 10) and `status_topic` (String JSON every `status_period_s`, only after the node has run one period so ages are real); clients for `<ns>/backup`, `<ns>/reset`, `<ns>/pause`, `<ns>/resume` (std_srvs/Empty).
- Actions: one at a time (a second command while one runs -> `busy`). `save`: call backup, then copy `live.db` to `saved.db` atomically (copy to temp + rename) in `database_dir`, also keep `<name>.db` (same content) so the 3D map is saved under the 2D map's name; result `ok` with db_path and size. `reset`: call reset; result ok. `pause`/`resume`: call the service, track the paused flag. A service not available within `service_timeout_s` -> `timeout`; a failed copy -> `error` with the errno text; invalid commands -> `invalid` with the request id. A `map_reset` message triggers the reset action without a result message and is logged.
- No placeholder data: status only from observed values (null for the unknown, per the status contract).
- Tests first in `test/test_node.cpp` with in-process fake Empty services and a temp database dir: save produces saved.db and <name>.db and an ok result; reset calls the service; busy while one runs; timeout when a service is missing; invalid keeps the request id; map_reset triggers reset; status JSON published with waiting_for_inputs until camera_infos arrive, then mapping; graph correction taken from a MapGraph.
- CMakeLists: the node library, executable and `test_node`. README: purpose, topics/services table, command and status contract (pointing at the headers), save/load semantics (saved.db restored on start by the launch, D9).

#### Acceptance conditions

- a1: save backs up RTAB-Map and writes saved.db and <name>.db atomically with an ok result [evidence: command: scripts/cpp-node-test.sh map3d_manager_cpp]
- a2: reset, pause, resume and map_guard resets call the RTAB-Map services; busy, timeout and invalid are reported [evidence: command: scripts/cpp-node-test.sh map3d_manager_cpp]
- a3: status is published from observed values with waiting_for_inputs until inputs arrive [evidence: command: scripts/cpp-node-test.sh map3d_manager_cpp]
- a4: the package builds and its tests and linters pass in docker [evidence: command: scripts/cpp-node-test.sh map3d_manager_cpp]

#### Files

- `nodes/map3d_manager_cpp/include/map3d_manager_cpp/map3d_manager_node.hpp`
- `nodes/map3d_manager_cpp/src/map3d_manager_node.cpp`
- `nodes/map3d_manager_cpp/src/main.cpp`
- `nodes/map3d_manager_cpp/test/test_node.cpp`
- `nodes/map3d_manager_cpp/CMakeLists.txt`
- `nodes/map3d_manager_cpp/README.md`

#### Interfaces

- Executable `map3d_manager_node`; topics `/map3d/result_server`, `/map3d/status_server` (downlinked to `/map3d/result`, `/map3d/status`); consumes `/uplink/map3d/command`, `/uplink/map_guard/map_reset`.

#### Expected outcome

A command from the web UI saves or resets the RTAB-Map database on the server and the robot sees the result and a live status.

#### Steps

- Write test/test_node.cpp; run `scripts/cpp-node-test.sh map3d_manager_cpp`; see `FAIL`.
- Implement node, main and CMake targets; rerun until `PASS`.
- Write the README; commit `feat(map3d_manager_cpp): save, reset and status node`.

#### Notes

- Pattern for request/result JSON over topics: poi_store and head_camera zoom.
- Do not subscribe any large uplink stream here (one WiFi copy per server reader, contract rule).

### S8: Mcp map3d tools

- Status: done
- Files: `nodes/mcp_server/mcp_server/map3d_tools.py`, `nodes/mcp_server/mcp_server/tools.py`, `nodes/mcp_server/mcp_server/config.py`, `nodes/mcp_server/mcp_server/ros_iface.py`, `nodes/mcp_server/tests/test_map3d_tools.py`, `nodes/mcp_server/tests/test_tools.py`, `nodes/mcp_server/tests/test_battery_gate.py`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- Verified head: 795538895185843918dca8ee2c3e305a534ef76e

#### Requirement

- `map3d_tools.py` registers three sensor tools (read-only, no lease, `SENSOR_TOOL_NAMES`):
  - `get_3d_map_status()`: the latest `/map3d/status` JSON plus the local cloud's voxel count, bounds, age and `alignment_agreement` against /map (D7 check); `null` fields with a note when nothing was received.
  - `query_3d_map_at(x, y, radius_m=0.3)` (map frame metres, radius 0.05..2.0): `query_at` result rounded (m to 3 decimals), with `source` and a note explaining lidar-only heights are unknown.
  - `get_3d_free_space(x, y, radius_m=1.0, z_min_m=0.02, z_max_m=null)` (radius 0.2..5.0; `z_max_m` defaults to `map3d.robot_height_m`): `free_space` per sector with bearings in degrees and `observed`.
- Data: lazily subscribes `topics.map3d_cloud` (PointCloud2, best_effort, keep_last 1) and `topics.map3d_status` on first use through the existing on-demand subscription machinery (`on_demand.linger_s` reaper), rebuilds the VoxelMap only when the cloud stamp changes, and reuses the node's /map subscription for the LidarGrid. A tool called before any cloud arrives raises a ToolError "no 3D map received yet" (no placeholder answer).
- `config.py`: `topics.map3d_cloud` (default `/map3d/cloud`), `topics.map3d_status` (default `/map3d/status`), `Map3dSettings` (`frame_id` `rtabmap_map`, `voxel_m` 0.05, `lidar_plane_height_m` 0.20 = base_link->laser_frame z, `robot_height_m` 0.55, `max_age_s` 120 after which results carry a stale note).
- `tools.py`: registers the module and adds its names to the sensor tool set; `test_tools.py` and `test_battery_gate.py` gain the three tools.
- Tests first in `tests/test_map3d_tools.py` with the fakes in `tests/fakes.py`: each tool's output on a synthetic cloud + grid, the no-data ToolError, a stale note after max_age_s, argument bounds, VoxelMap rebuilt once per stamp.

#### Acceptance conditions

- a1: the three 3D map tools answer from a synthetic cloud and grid [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_map3d_tools.py -q]
- a2: tools refuse with a clear error before any 3D map arrived [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_map3d_tools.py -q]
- a3: the tools are registered as sensor tools and pass the battery gate table [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_tools.py -q]
- a4: mcp_server tests and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/mcp_server/mcp_server/map3d_tools.py`
- `nodes/mcp_server/mcp_server/tools.py`
- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/tests/test_map3d_tools.py`
- `nodes/mcp_server/tests/test_tools.py`
- `nodes/mcp_server/tests/test_battery_gate.py`

#### Interfaces

- MCP tools `get_3d_map_status`, `query_3d_map_at`, `get_3d_free_space` (`nodes/mcp_server/mcp_server/map3d_tools.py:SENSOR_TOOL_NAMES`): the `mcp-map3d-tools` part; config `topics.map3d_cloud`, `topics.map3d_status`, `map3d.*`.

#### Expected outcome

The agent can query the 3D map through three read-only tools.

#### Steps

- Write `tests/test_map3d_tools.py` and the test_tools/battery gate entries; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_map3d_tools.py tests/test_tools.py tests/test_battery_gate.py -q`; see them fail.
- Implement; run `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint`.
- Commit `feat(mcp_server): 3D map query tools`.

#### Notes

- Pattern: `frontier_tools.py` (module TOOL_NAMES, register(ctx), pydantic results, tool_errors).
- Do not deploy mcp_server during an agent session (MEMORY).

### S9: Rtabmap server node type

- Status: done
- Files: `ansible/group_vars/server.yml`, `ansible/playbooks/deploy_nodes_server.yml`, `ansible/playbooks/tasks/rtabmap_db_dir.yml`, `tests/test_rtabmap_mapping_deploy.py`
- Verify: `uv run pytest tests/test_rtabmap_mapping_deploy.py tests/test_unit_cgroups.py tests/test_dds_discovery_config.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: 97a2c1420d1d952dc6e0a843e12d6f199a6272f1

#### Requirement

- server.yml node type `rtabmap_mapping` (D1, D8): native, `ros2 launch {{ ros2_repo_dest }}/nodes/rtabmap_mapping/launch/rtabmap_mapping.launch.py`, `node_src_dir: ""`, `src_paths: [nodes/rtabmap_mapping]`, apt `ros-jazzy-rtabmap-slam`, `ros-jazzy-rtabmap-sync`, `ros-jazzy-rtabmap-util`, `ros-jazzy-rtabmap-msgs`, `ros-jazzy-rtabmap-conversions`, `ros-jazzy-image-transport-plugins`, `ros-jazzy-rclcpp-components`, `python3-pydantic`, `python3-yaml` (never `ros-jazzy-rtabmap-ros` or `ros-jazzy-rtabmap-viz`), `cpu_quota: "200%"`, `memory_max: "900M"` (server rule: quota and cap, no tier keys), `config_path: /etc/ros2/rtabmap_mapping`, env `RTABMAP_MAPPING_CONFIG=/etc/ros2/rtabmap_mapping/config.yaml`, no `ROS_STATIC_PEERS`. ros2_nodes entry `rtabmap_mapping` present/enabled with an empty override config block commented with the tunable keys.
- `tasks/rtabmap_db_dir.yml`: `/var/lib/ros2/rtabmap` owned by the node user, mode 0755 (pattern: slam_maps_dir.yml), included in the server playbook before `Deploy rtabmap_mapping` with tags `[setup, rtabmap_mapping]`.
- `deploy_nodes_server.yml`: step `Deploy rtabmap_mapping` (resolve_and_deploy.yml, tags `[rtabmap_mapping, apt, python, build, config]`) after gps_rtk_base.
- Tests first in `tests/test_rtabmap_mapping_deploy.py`, seen failing: node type fields, apt list (and the forbidden metapackage/viz), quota/cap, no static peer, ros2_nodes schema, db dir task and its order, playbook step; existing `tests/test_unit_cgroups.py::test_server_keeps_its_limits_without_tiers` and the dds test still pass unchanged.

#### Acceptance conditions

- a1: the server deploys rtabmap_mapping from the split apt packages with CPUQuota 200% and MemoryMax 900M [evidence: test: tests/test_rtabmap_mapping_deploy.py]
- a2: the database directory is created before the node deploy [evidence: test: tests/test_rtabmap_mapping_deploy.py]
- a3: server limits and discovery rules still hold [evidence: test: tests/test_unit_cgroups.py::test_server_keeps_its_limits_without_tiers]
- a4: ansible lint and syntax checks pass [evidence: command: uv run poe test-ansible]

#### Files

- `ansible/group_vars/server.yml`
- `ansible/playbooks/deploy_nodes_server.yml`
- `ansible/playbooks/tasks/rtabmap_db_dir.yml`
- `tests/test_rtabmap_mapping_deploy.py`

#### Interfaces

- Unit `ros2-rtabmap_mapping` on the server, tag `rtabmap_mapping` for `scripts/deploy-nodes.sh server rtabmap_mapping`.

#### Expected outcome

`./scripts/deploy-nodes.sh server rtabmap_mapping` installs and starts RTAB-Map with its caps.

#### Steps

- Write the test; run `uv run pytest tests/test_rtabmap_mapping_deploy.py -q`; see it fail.
- Edit server.yml, the playbook and the db dir task; run the verify command.
- Commit `feat(ansible): deploy rtabmap_mapping on the server`.

#### Notes

- Server budget: monitoring.slice up to 2.2 GB / 150 % CPU; ~1.1 GB left for ROS units (server.yml comment). 900M leaves room for lerobot_leader, gps_rtk_base, ros2-master and map3d_manager; the runbook measures and proposes final caps.

### S10: Web UI map3d cloud backend

- Status: done
- Files: `nodes/web_ui/web_ui/cloud_serializer.py`, `nodes/web_ui/web_ui/config.py`, `nodes/web_ui/web_ui/bridge.py`, `nodes/web_ui/config/default.yaml`, `nodes/web_ui/tests/test_map3d_cloud.py`, `ansible/group_vars/client.yml`
- Verify: `cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_map3d_topics.py -q`
- Verified head: a46e254fc8a0195dda111d8bd793d56ae04c26ce

#### Requirement

- `cloud_serializer.py` (numpy only, D10): `serialize_map3d_cloud(msg, max_points: int, voxel_m: float) -> dict` decodes a sensor_msgs/PointCloud2 with float32 x, y, z and an optional packed `rgb`/`rgba` field (uint32 or float32 reinterpretation; any other layout raises ValueError naming the fields), drops non-finite points, voxel-downsamples (one point per `voxel_m` cell, first-seen colour) until at most `max_points` remain (increasing the voxel by x1.5 steps when needed), and returns `points_b64` (little-endian int16 centimetres x, y, z, clamped to +-327.67 m, out-of-range points dropped and counted), `colors_b64` (uint8 r, g, b per point; absent field -> key omitted), `count`, `source_count`, `voxel_m` (the one actually used), `dropped_out_of_range`, `frame_id`, `stamp`. An empty cloud returns `count: 0` (a real empty map, not placeholder) only when the message itself is empty; a malformed message is logged and not stored (no placeholder payload).
- Config (`config.py`, `default.yaml`, client.yml web_ui block): map_nav TabConfig fields `map3d_cloud_topic` (default `/map3d/cloud`), `map3d_status_topic` (default `/map3d/status`), `map3d_result_topic` (default `/map3d/result`), `map3d_command_topic` (default `/map3d/command`), `map3d_frame_id` (default `rtabmap_map`, shown in the map frame unchanged because D7 keeps RTAB-Map's map frame identical to slam_toolbox's `map`), `map3d_max_points` (default 60000, 1000..300000), `map3d_voxel_m` (default 0.05, 0.02..0.5), `lidar_wall_height_m` (default 1.0, 0.1..3.0) added to `MAP_NAV_DEFAULTS` and `MAP_NAV_TOPIC_ROLES` (`map3d_cloud`, `map3d_status`, `map3d_result`). Defaults equal the map3d topic contract's downlink dests (asserted by reading `nodes/rtabmap_mapping/config/map3d_topics.yaml` in the test).
- Bridge (`bridge.py`): role specs `map3d_cloud` (PointCloud2, best_effort keep_last 1, serializer above, on-demand: subscribed only while a client shows the layer, via `register_on_demand`, latest-wins), `map3d_status` (std_msgs/String JSON passed through as parsed JSON, reliable depth 1), `map3d_result` (String JSON, reliable depth 10). The cloud serializer runs lazily (the existing lazy-serialization path) so an unchanged cloud is not re-encoded per WS client.
- Tests first in `tests/test_map3d_cloud.py`, seen failing: xyz+rgb and xyz-only clouds; NaN dropped; cap respected with the voxel grown and reported; centimetre rounding; out-of-range counted; unsupported layout raises; config defaults and bounds; the defaults match the contract yaml; the role is on-demand and has the stated QoS.

#### Acceptance conditions

- a1: PointCloud2 clouds serialize to capped int16-centimetre points with colours and metadata [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_map3d_cloud.py -q]
- a2: malformed clouds are rejected and never stored as placeholder payloads [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_map3d_cloud.py -q]
- a3: map3d topic defaults equal the contract's downlink dests [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_map3d_cloud.py -q]
- a4: the cloud subscription is on-demand with the stated QoS [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_map3d_cloud.py -q]
- a5: web_ui tests and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/web_ui/web_ui/cloud_serializer.py`
- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/web_ui/bridge.py`
- `nodes/web_ui/config/default.yaml`
- `nodes/web_ui/tests/test_map3d_cloud.py`
- `ansible/group_vars/client.yml`

#### Interfaces

- WS roles `map3d_cloud` (payload `points_b64`, `colors_b64`, `count`, `source_count`, `voxel_m`, `dropped_out_of_range`, `frame_id`, `stamp`), `map3d_status`, `map3d_result`; TabConfig fields `map3d_*` and `lidar_wall_height_m`: the `web-ui-map3d-roles` part (`nodes/web_ui/web_ui/cloud_serializer.py:serialize_map3d_cloud`, `nodes/web_ui/web_ui/config.py`).

#### Expected outcome

With the Map tab's 3D layer on, web_ui sends at most 60k points of the downlinked map; with it off nothing is subscribed.

#### Steps

- Write `nodes/web_ui/tests/test_map3d_cloud.py`; run `cd nodes/web_ui && uv run --frozen pytest tests/test_map3d_cloud.py -q`; see it fail.
- Implement serializer, config and bridge roles; run `cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint` until green.
- Commit `feat(web_ui): map3d cloud, status and result roles`.

#### Notes

- If the ToF obstacle epic already added a PointCloud2 decoder to web_ui, reuse it inside serialize_map3d_cloud instead of a second decoder (one PointCloud2 parser per node).
- `/map3d/cloud` frame is `rtabmap_map` (design D7: identical to `map`); the serializer keeps frame_id and the frontend refuses any frame other than `map3d_frame_id` (map3d-frontend-layers).
- Placeholder rule: the payload only ever carries decoded points.

### S11: Map3d frontend layers

- Status: done
- Files: `nodes/web_ui/frontend/src/map3d/cloudDecode.ts`, `nodes/web_ui/frontend/src/map3d/cloudDecode.test.ts`, `nodes/web_ui/frontend/src/map3d/lidarWalls.ts`, `nodes/web_ui/frontend/src/map3d/lidarWalls.test.ts`, `nodes/web_ui/frontend/src/map3d/layers.ts`, `nodes/web_ui/frontend/src/map3d/layers.test.ts`, `nodes/web_ui/frontend/src/map3d/sceneLayers.tsx`, `nodes/web_ui/frontend/src/types.ts`
- Verify: `cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build`
- Verified head: b91e5ee944d223b32e8484504d6430c2ab0e5c84

#### Requirement

- `cloudDecode.ts`: `decodeMap3dCloud(payload)` -> `{positions: Float32Array (metres, three.js order after the existing map->scene conversion in coords.ts), colors?: Float32Array, count, voxel}`; rejects (returns null and logs once) a payload whose `frame_id` is not the tab's `map3d_frame_id` (drawn in the map frame, D7) or whose byte lengths do not match `count`.
- `lidarWalls.ts` (U6, D13): `buildLidarWalls(grid, measuredColumns, heightM)` builds wall boxes from occupied cells of the SLAM `/map` payload already in the browser (same occupied threshold as the SLAM map layer), merged per row run, extruded from 0 to `lidar_wall_height_m`, skipping every cell whose column contains a measured 3D point (`measuredColumnsFromCloud(positions, resolution, origin)` hashes cloud x/y into map cells). Pure functions, no three.js in the geometry builder.
- `layers.ts`: new layer keys `map3dCloud` ('3D map (measured)') and `lidarWalls` ('Lidar walls (2D only)'), both visible by default; stored state of older browsers still parses.
- `sceneLayers.tsx`: a points layer (size = voxel, vertex colours when present, a single neutral colour otherwise) and a lidar-walls layer drawn translucent with a distinct hatch-free solid colour from the palette and no vertex colours, so lidar-derived structure is never mistaken for measured points; the cloud layer's visibility drives the on-demand `map3d_cloud` subscription (subscribed only while visible), walls recompute when /map or the cloud changes, at most once per second.
- `types.ts`: payload types for `map3d_cloud`, `map3d_status`, `map3d_result`, TabConfig fields `map3d_*` (including `map3d_frame_id`), `lidar_wall_height_m`.
- Tests first, seen failing: `cloudDecode.test.ts` (round-trip of a payload built in the test, frame mismatch rejected, length mismatch rejected), `lidarWalls.test.ts` (an occupied run becomes one box of the configured height; a column with a measured point is skipped; free and unknown cells give nothing), `layers.test.ts` (new keys, defaults, legacy stored state).

#### Acceptance conditions

- a1: cloud payloads decode to scene positions and colours and wrong frames or lengths are rejected [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/map3d/cloudDecode.test.ts]
- a2: lidar walls extrude occupied /map cells to the configured height except where measured points exist [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/map3d/lidarWalls.test.ts]
- a3: the two new layers exist, default on, and legacy layer state parses [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/map3d/layers.test.ts]
- a4: measured points and lidar walls are drawn with distinguishable styles [evidence: judgement]
- a5: frontend typecheck, tests, lint and build pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build]

#### Files

- `nodes/web_ui/frontend/src/map3d/cloudDecode.ts`
- `nodes/web_ui/frontend/src/map3d/cloudDecode.test.ts`
- `nodes/web_ui/frontend/src/map3d/lidarWalls.ts`
- `nodes/web_ui/frontend/src/map3d/lidarWalls.test.ts`
- `nodes/web_ui/frontend/src/map3d/layers.ts`
- `nodes/web_ui/frontend/src/map3d/layers.test.ts`
- `nodes/web_ui/frontend/src/map3d/sceneLayers.tsx`
- `nodes/web_ui/frontend/src/types.ts`

#### Interfaces

- `nodes/web_ui/frontend/src/map3d/cloudDecode.ts:decodeMap3dCloud`, `nodes/web_ui/frontend/src/map3d/lidarWalls.ts:buildLidarWalls`, `measuredColumnsFromCloud`; layer keys `map3dCloud`, `lidarWalls`.

#### Expected outcome

The Map tab renders the room in 3D: coloured measured points, and translucent lidar walls only where the ToF never looked.

#### Steps

- Write the three test files; run `cd nodes/web_ui/frontend && npx vitest run src/map3d/cloudDecode.test.ts src/map3d/lidarWalls.test.ts src/map3d/layers.test.ts`; see them fail.
- Implement; run `cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build`.
- Commit `feat(web_ui): 3D map and lidar wall layers`.

#### Notes

- The LayersPanel lists layers from LAYER_LABELS, so no panel edit is needed; check `map/LayersPanel.test.tsx` still passes.
- Keep the point count from the backend cap; no client-side resampling beyond decoding.

### S12: Map3d manager node type

- Status: done
- Files: `ansible/group_vars/server.yml`, `ansible/playbooks/deploy_nodes_server.yml`, `tests/test_map3d_manager_deploy.py`, `tests/test_cpp_node_deploy.py`
- Verify: `uv run pytest tests/test_map3d_manager_deploy.py tests/test_cpp_node_deploy.py tests/test_unit_cgroups.py tests/test_map3d_topics.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: 6a1af434a70ac00ada34dc72db7aae78f6e6a93c

#### Requirement

- server.yml node type `map3d_manager_cpp` (colcon in-repo C++ like map_guard_cpp on the client: `colcon_source` of `shared/ros2_common_cpp` then `nodes/map3d_manager_cpp`, Release, BUILD_TESTING off, jobs 2; `ros2 run map3d_manager_cpp map3d_manager_node`; apt for every package.xml dependency incl. `ros-jazzy-rtabmap-msgs`, `nlohmann-json3-dev`, `libyaml-cpp-dev`), `cpu_quota: "20%"`, `memory_max: "128M"`, config `/etc/ros2/map3d_manager/config.yaml` via `MAP3D_MANAGER_CONFIG`; ros2_nodes entry `map3d_manager` present/enabled after `rtabmap_mapping`, config block with the defaults that differ from code (none expected; comment says so).
- `deploy_nodes_server.yml`: `Deploy map3d_manager` after `Deploy rtabmap_mapping`.
- Tests first: `tests/test_map3d_manager_deploy.py` (type, apt, caps, order, config topics equal the map3d topic contract's manager-side topics: command, map_reset, result/status server names), `tests/test_cpp_node_deploy.py` gains the package in its C++ node list.

#### Acceptance conditions

- a1: map3d_manager is built and deployed on the server after rtabmap_mapping with its caps [evidence: test: tests/test_map3d_manager_deploy.py]
- a2: its topics equal the contract's manager-side topics [evidence: test: tests/test_map3d_manager_deploy.py]
- a3: the C++ deploy invariants cover the new package [evidence: test: tests/test_cpp_node_deploy.py]
- a4: ansible lint and syntax checks pass [evidence: command: uv run poe test-ansible]

#### Files

- `ansible/group_vars/server.yml`
- `ansible/playbooks/deploy_nodes_server.yml`
- `tests/test_map3d_manager_deploy.py`
- `tests/test_cpp_node_deploy.py`

#### Interfaces

- Unit `ros2-map3d_manager` on the server, tag `map3d_manager`.

#### Expected outcome

`./scripts/deploy-nodes.sh server map3d_manager` builds and starts the manager.

#### Steps

- Write the tests; run them; see them fail.
- Edit server.yml and the playbook; run the verify command.
- Commit `feat(ansible): deploy map3d_manager on the server`.

#### Notes

- The server has not built in-repo C++ before: check `ros2_node_deploy` colcon tasks need no client-only variable (e.g. nav2 overlay vars) and say what you found in the commit.

### S13: Mcp map3d config docs

- Status: done
- Files: `ansible/group_vars/client.yml`, `tests/test_mcp_server_config.py`, `nodes/mcp_server/README.md`, `nodes/claude_agent/claude_agent/config.py`, `nodes/claude_agent/claude_agent/prompt.py`, `nodes/claude_agent/tests/test_prompt.py`, `nodes/claude_agent/tests/test_config.py`, `nodes/claude_agent/README.md`
- Verify: `cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: 222a9e47de75e9b7063c50b9c83bd5c0e56470de

#### Requirement

- client.yml mcp_server config: `topics.map3d_cloud: /map3d/cloud`, `topics.map3d_status: /map3d/status` and a `map3d:` block with the defaults; `tests/test_mcp_server_config.py` asserts they equal the map3d topic contract's downlink dests and that the README tool table lists the three tools.
- `nodes/mcp_server/README.md`: tool table rows for `get_3d_map_status`, `query_3d_map_at`, `get_3d_free_space` (arguments, units, sources, honest-unknown rules) and a `## 3D map` section (where the cloud comes from, refresh period, measured vs lidar-only).
- claude_agent: the three tools join the allowed tool list (`config.py`); the prompt gains a short 3D-map paragraph: use `query_3d_map_at` for heights and what is at a spot, `get_3d_free_space` for clearance above the floor (e.g. under a table), lidar-only results have unknown height, the 3D map refreshes about every 10 s and only where the head looked. Tests in `test_prompt.py`/`test_config.py` assert the names and the paragraph; README tool list updated.

#### Acceptance conditions

- a1: mcp_server deploy config points at the contract's downlink topics [evidence: test: tests/test_mcp_server_config.py]
- a2: the agent's tool list and prompt name the three 3D map tools [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_prompt.py -q]
- a3: the mcp_server README documents the 3D map tools [evidence: doc-section: nodes/mcp_server/README.md#3D map]
- a4: claude_agent tests, root config tests and ansible checks pass [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py -q && uv run poe test-ansible]

#### Files

- `ansible/group_vars/client.yml`
- `tests/test_mcp_server_config.py`
- `nodes/mcp_server/README.md`
- `nodes/claude_agent/claude_agent/config.py`
- `nodes/claude_agent/claude_agent/prompt.py`
- `nodes/claude_agent/tests/test_prompt.py`
- `nodes/claude_agent/tests/test_config.py`
- `nodes/claude_agent/README.md`

#### Interfaces

- client.yml `mcp_server` config keys `topics.map3d_*`, `map3d.*`; claude_agent allowed tools.

#### Expected outcome

After deploy the agent can call the 3D map tools and is told how to read their answers.

#### Steps

- Extend the tests; run them; see them fail.
- Edit config, prompt, READMEs, client.yml; run the verify command.
- Commit `feat(agent): expose the 3D map tools`.

#### Notes

- `tests/test_mcp_server_config.py` couples client.yml, READMEs and sources; keep every coupling it checks.

### S14: Web UI map3d save reset

- Status: done
- Files: `nodes/web_ui/web_ui/server.py`, `nodes/web_ui/web_ui/bridge.py`, `nodes/web_ui/tests/test_map3d_commands.py`, `nodes/web_ui/README.md`
- Verify: `cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- Verified head: fdf018ea4c3f8694641713f36315cf085b9e2d03

#### Requirement

- `POST /api/map/save`: after slam_toolbox `serialize_map` succeeds, publishes one `save` command on `map3d_command_topic` (JSON per the map3d command contract: `request_id` uuid4 hex, `action: "save"`, `name` = basename of `map_save_path`) and waits up to `map3d_command_timeout_s` (new TabConfig field, default 15 s, > 0) for the matching `/map3d/result`. The response gains `map3d: {status, message}`; a 3D failure or timeout does not undo the 2D save and is reported as `map3d.status` `error`/`timeout` with HTTP 200 and the 2D result unchanged (my call: the 2D map is the source of truth).
- `POST /api/map/reset`: after slam_toolbox reset succeeds, publishes `reset` and reports `map3d` the same way. A reset that slam_toolbox refused publishes nothing.
- Bridge: a reliable depth-10 publisher on `map3d_command_topic`; results matched by `request_id` (the poi/grasp request-result pattern); unmatched results ignored.
- Tests first in `tests/test_map3d_commands.py`, seen failing: save publishes exactly one command after a successful 2D save and reports the 3D result; 3D timeout leaves the 2D success intact; failed 2D save or reset publishes nothing; result matching by request_id; concurrent save calls coalesce like today's.
- README: the save/reset rows of the API table say they also act on the 3D map, and a `## 3D map` section (layers, status, save/reset behaviour).

#### Acceptance conditions

- a1: a successful 2D save or reset publishes one matching map3d command and reports its result [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_map3d_commands.py -q]
- a2: a failed 2D save or reset publishes nothing and a 3D timeout keeps the 2D result [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_map3d_commands.py -q]
- a3: the README documents the 3D map behaviour [evidence: doc-section: nodes/web_ui/README.md#3D map]
- a4: web_ui tests and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/web_ui/web_ui/server.py`
- `nodes/web_ui/web_ui/bridge.py`
- `nodes/web_ui/tests/test_map3d_commands.py`
- `nodes/web_ui/README.md`

#### Interfaces

- Uses the map3d command contract (`nodes/map3d_manager_cpp/include/map3d_manager_cpp/command_json.hpp`); response field `map3d` on `/api/map/save` and `/api/map/reset`.

#### Expected outcome

Save in the web UI writes the 2D posegraph and backs up the RTAB-Map database; reset clears both.

#### Steps

- Write the tests; run `cd nodes/web_ui && uv run --frozen pytest tests/test_map3d_commands.py -q`; see them fail.
- Implement; run `cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint`.
- Commit `feat(web_ui): save and reset the 3D map with the 2D map`.

#### Notes

- map_guard_cpp resets reach the 3D map through `/map_guard/map_reset` in the uplink (map3d-manager-node), not through web_ui.

### S15: Map3d frontend status

- Status: done
- Files: `nodes/web_ui/frontend/src/map/Map3dStatus.tsx`, `nodes/web_ui/frontend/src/map/Map3dStatus.test.tsx`, `nodes/web_ui/frontend/src/map/LayersPanel.tsx`, `nodes/web_ui/frontend/src/map/LayersPanel.test.tsx`, `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`, `nodes/web_ui/frontend/src/tabs/MapNavTab.toolbar.test.tsx`
- Verify: `cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build`
- Verified head: 29c499a2564a5603301bf3a9dc3051b98022388c

#### Requirement

- `Map3dStatus.tsx`: renders the latest `/map3d/status` JSON (map3d status contract): state (`mapping`, `paused`, `waiting_for_inputs`, `error`), node count, database size MB, process memory MB, last update age, RTAB-Map graph correction (`graph_correction_m`, `graph_correction_deg`) with a warning colour above the status' `correction_warn_m`/`correction_warn_deg`, and the `missing_inputs` list when waiting. No status received yet shows "no 3D map status yet" (never zeros).
- `LayersPanel.tsx`: the `map3dCloud` row shows the status widget's one-line summary next to its checkbox.
- `MapNavTab.tsx`: the save/reset toolbar actions show the response's `map3d` result (status and message) after the 2D result.
- Tests first, seen failing: `Map3dStatus.test.tsx` (each state, warning colour over the correction threshold, no-status text), `LayersPanel.test.tsx` (summary next to the 3D row), `MapNavTab.toolbar.test.tsx` (save and reset show the 3D result, timeout shown as a warning).

#### Acceptance conditions

- a1: status states, the alignment warning and the no-status case render correctly [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/map/Map3dStatus.test.tsx]
- a2: save and reset show the 3D result after the 2D result [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/tabs/MapNavTab.toolbar.test.tsx]
- a3: frontend typecheck, tests, lint and build pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build]

#### Files

- `nodes/web_ui/frontend/src/map/Map3dStatus.tsx`
- `nodes/web_ui/frontend/src/map/Map3dStatus.test.tsx`
- `nodes/web_ui/frontend/src/map/LayersPanel.tsx`
- `nodes/web_ui/frontend/src/map/LayersPanel.test.tsx`
- `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`
- `nodes/web_ui/frontend/src/tabs/MapNavTab.toolbar.test.tsx`

#### Interfaces

- Reads the map3d status contract (`nodes/map3d_manager_cpp/include/map3d_manager_cpp/status.hpp`).

#### Expected outcome

An operator sees at a glance whether the 3D map is being built and aligned with the 2D map.

#### Steps

- Write the tests; run `cd nodes/web_ui/frontend && npx vitest run src/map/Map3dStatus.test.tsx src/map/LayersPanel.test.tsx src/tabs/MapNavTab.toolbar.test.tsx`; see them fail.
- Implement; run `cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build`.
- Commit `feat(web_ui): 3D map status and save/reset results`.

#### Notes

- Status fields and thresholds are defined once in the map3d status contract; do not invent missing ones in the UI.

### S16: Mapping CPU dashboards

- Status: done
- Files: `ansible/roles/monitoring/files/robot-slam.json`, `tests/test_monitoring_dashboards.py`
- Verify: `uv run pytest tests/test_monitoring_dashboards.py tests/test_monitoring_real_binaries.py -q && uv run poe lint`
- Verified head: d5982549bba7468dad2d519fa30014137e74bf1c

#### Requirement

- `robot-slam.json` gains a "3D mapping" row with panels: "3D mapping server unit CPU" and "3D mapping server unit memory" (`container_cpu_usage_seconds_total` / `container_memory_working_set_bytes` for `ros2-rtabmap_mapping.service` and `ros2-map3d_manager.service`, `host="server"`), "Map3d uplink client CPU" (`ros2-map3d_uplink.service`, `ros2-head_rgbd_cpp.service` and `ros2-head_camera_cpp.service`, `host="client"`), "Server memory available" (`node_memory_MemAvailable_bytes{host="server"}`), and "WiFi throughput" (`rate(node_network_transmit_bytes_total{device=~"wlan.*"}[1m])*8` and receive, per host, in bit/s) with a threshold line at the contract's `uplink_mbit_s_max` (8 Mbit/s).
- `tests/test_monitoring_dashboards.py`: `test_slam_dashboard_queries` asserts the new panels and units; the robot-only host-scoping rule allows `host="server"` only in panels whose title starts with "3D mapping server" or is "Server memory available" / "WiFi throughput" (every other selector still `host="client"`); the metric contract stays (node_* host metrics are allowed).

#### Acceptance conditions

- a1: the SLAM dashboard has the 3D mapping CPU, memory and WiFi panels [evidence: test: tests/test_monitoring_dashboards.py::test_slam_dashboard_queries]
- a2: server selectors appear only in the 3D mapping server panels [evidence: test: tests/test_monitoring_dashboards.py]
- a3: PromQL parses with the real tools [evidence: test: tests/test_monitoring_real_binaries.py]

#### Files

- `ansible/roles/monitoring/files/robot-slam.json`
- `tests/test_monitoring_dashboards.py`

#### Interfaces

- Dashboard panel titles used by the runbook: "3D mapping server unit CPU", "3D mapping server unit memory", "Map3d uplink client CPU", "Server memory available", "WiFi throughput".

#### Expected outcome

One Grafana row shows server and client cost and the link bandwidth of 3D mapping.

#### Steps

- Extend the tests; run `uv run pytest tests/test_monitoring_dashboards.py -q`; see them fail.
- Edit robot-slam.json; rerun with test_monitoring_real_binaries.py.
- Commit `feat(monitoring): 3D mapping cost panels`.

#### Notes

- Unit names come from the node names in group_vars (rtabmap_mapping, map3d_manager on the server; map3d_uplink, head_rgbd_cpp, head_camera_cpp on the client).
- Deploying the dashboard is `./scripts/deploy-nodes.sh server monitoring` in the runbook.

### S17: Map3d docs and memory

- Status: done
- Files: `tests/README.md`, `nodes/README.md`, `ansible/README.md`, `MEMORY.md`, `.claude/skills/ansible-deploy/SKILL.md`, `docs/diagrams/topic_flow.puml`
- Verify: `uv run pytest tests/test_node_lint_coverage.py -q && uv run poe lint && grep -q 'test_map3d_topics.py' tests/README.md && grep -q 'map3d_uplink' ansible/README.md && grep -q 'rtabmap_mapping' nodes/README.md`
- Verified head: b2bde7000cc5b2e437edc52a949dcf0e790e1e61

#### Requirement

- `tests/README.md`: entries for every new or changed test file of the epic (test_map3d_topics.py, test_map3d_uplink_config.py, test_rtabmap_mapping_config.py, test_rtabmap_mapping_deploy.py, test_map3d_manager_deploy.py, and the changes to test_dds_discovery_config.py, test_unit_cgroups.py, test_cpp_node_deploy.py, test_mcp_server_config.py, test_monitoring_dashboards.py), each with what it covers.
- `nodes/README.md`: rows for rtabmap_mapping (server), map3d_manager_cpp (server), map3d_uplink (client) and the head camera mapping stream.
- `ansible/README.md`: "Cross-host" names map3d_uplink as the third static-peer process and why; the server node list gains rtabmap_mapping and map3d_manager with their caps; the memory budget table gains the new units.
- `MEMORY.md`: a "3D mapping (RTAB-Map on the server)" entry with the key decisions D1, D2, D7, D8, D13 (one line each).
- `.claude/skills/ansible-deploy/SKILL.md`: the new node names and their targets.
- `docs/diagrams/topic_flow.puml`: the uplink/downlink streams (regenerating the PNG with scripts/generate-diagrams.sh when plantuml is available; otherwise say so in the commit).

#### Acceptance conditions

- a1: tests/README lists every new test file of the epic [evidence: command: grep -q 'test_map3d_topics.py' tests/README.md && grep -q 'test_rtabmap_mapping_config.py' tests/README.md && grep -q 'test_map3d_uplink_config.py' tests/README.md && grep -q 'test_map3d_manager_deploy.py' tests/README.md]
- a2: ansible README names map3d_uplink as a static-peer process [evidence: command: grep -q 'map3d_uplink' ansible/README.md]
- a3: MEMORY records the 3D mapping decisions [evidence: command: grep -q 'RTAB-Map' MEMORY.md]
- a4: docs read correctly for a mid-level ROS 2 developer [evidence: judgement]

#### Files

- `tests/README.md`
- `nodes/README.md`
- `ansible/README.md`
- `MEMORY.md`
- `.claude/skills/ansible-deploy/SKILL.md`
- `docs/diagrams/topic_flow.puml`

#### Interfaces

- Docs only.

#### Expected outcome

The docs describe the 3D mapping stack end to end.

#### Steps

- Edit the docs; run the verify command.
- Commit `docs: 3D mapping stack`.

#### Notes

- Doc-only task (TDD exception: documentation).

## Files to delete

none recorded in the epic format

## Final verify

none recorded in the epic format

## After merge

none
