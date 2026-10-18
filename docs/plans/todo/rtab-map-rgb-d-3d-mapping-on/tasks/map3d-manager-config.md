---
title: Map3d manager config
status: todo
repo: ros2-lerobot-sverve-platform
value: "The 3D map's save, reset and status messages have one defined JSON contract shared by the server node, web UI and agent."
files: [nodes/map3d_manager_cpp/package.xml, nodes/map3d_manager_cpp/CMakeLists.txt, nodes/map3d_manager_cpp/include/map3d_manager_cpp/config.hpp, nodes/map3d_manager_cpp/src/config.cpp, nodes/map3d_manager_cpp/test/test_config.cpp, nodes/map3d_manager_cpp/include/map3d_manager_cpp/command_json.hpp, nodes/map3d_manager_cpp/src/command_json.cpp, nodes/map3d_manager_cpp/test/test_command_json.cpp]
verify: scripts/cpp-node-test.sh map3d_manager_cpp
depends_on: []
owns: [map3d-command-contract, map3d-manager-config]
uses: []
model: sonnet
estimate: 70
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Map3d manager config

## Requirement

- New ament_cmake package `map3d_manager_cpp` (layout, flags and linters of map_guard_cpp; C++17, -Werror). `package.xml` declares everything the finished node needs: `ros2_common_cpp`, `rclcpp`, `std_msgs`, `std_srvs`, `rtabmap_msgs`, `sensor_msgs`, `yaml-cpp`, `nlohmann-json-dev`; test deps as map_guard_cpp. CMakeLists builds a static core library (config, command_json, later status) and one gtest per test file.
- `config.hpp/.cpp` (D9): `Config` loaded from `MAP3D_MANAGER_CONFIG` (default `/etc/ros2/map3d_manager/config.yaml`) through `ros2_common_cpp::load_config_or_exit` (unknown keys are errors): `command_topic` `/uplink/map3d/command`, `map_reset_topic` `/uplink/map_guard/map_reset`, `result_topic` `/map3d/result_server`, `status_topic` `/map3d/status_server`, `rtabmap_namespace` `/rtabmap`, `database_dir` `/var/lib/ros2/rtabmap`, `status_period_s` 1.0, `service_timeout_s` 10.0, `input_stale_s` 3.0, `correction_warn_m` 0.05, `correction_warn_deg` 2.0, `rgb_info_topic` `/uplink/head_camera/mapping/camera_info`, `depth_info_topic` `/uplink/tof/camera_info`, `rtabmap_unit` `ros2-rtabmap_mapping.service` (for cgroup memory). Topic defaults equal the map3d topic contract (checked by tests/test_map3d_manager_deploy.py later).
- `command_json.hpp/.cpp` (the command contract, header comment is the spec): command `{"request_id": str (1..64 chars), "action": "save"|"reset"|"pause"|"resume", "name": str (optional, [A-Za-z0-9_-]{1,64}, default "slam_map")}` -> `parse_command(const std::string&) -> std::variant<Command, ParseError>` (ParseError carries the request_id when present so an `invalid` result can still be matched; no request_id -> dropped); result `{"request_id", "status": "ok"|"error"|"busy"|"invalid"|"timeout", "message", "action", "db_path"?, "db_size_mb"?}` -> `to_json(const Result&)`.
- Tests first, seen failing: `test/test_config.cpp` (defaults, every bound, unknown key) and `test/test_command_json.cpp` (each action, bad name, missing/long request_id, unknown action -> invalid with id, not JSON -> dropped; result serialisation with and without db fields).

## Acceptance conditions

- a1: config defaults and validation errors name their keys [evidence: test: nodes/map3d_manager_cpp/test/test_config.cpp]
- a2: commands parse into save/reset/pause/resume or an invalid result that keeps the request id [evidence: test: nodes/map3d_manager_cpp/test/test_command_json.cpp]
- a3: the package builds and its tests and linters pass in docker [evidence: command: scripts/cpp-node-test.sh map3d_manager_cpp]

## Files

- `nodes/map3d_manager_cpp/package.xml`
- `nodes/map3d_manager_cpp/CMakeLists.txt`
- `nodes/map3d_manager_cpp/include/map3d_manager_cpp/config.hpp`
- `nodes/map3d_manager_cpp/src/config.cpp`
- `nodes/map3d_manager_cpp/test/test_config.cpp`
- `nodes/map3d_manager_cpp/include/map3d_manager_cpp/command_json.hpp`
- `nodes/map3d_manager_cpp/src/command_json.cpp`
- `nodes/map3d_manager_cpp/test/test_command_json.cpp`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
