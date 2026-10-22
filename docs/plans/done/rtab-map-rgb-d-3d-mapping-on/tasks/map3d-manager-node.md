---
title: Map3d manager node
status: done
repo: ros2-lerobot-sverve-platform
value: "Save, reset, pause and map_guard resets reach RTAB-Map on the server, and the 3D map's status reaches the robot, without a new cross-host service path."
files: [nodes/map3d_manager_cpp/include/map3d_manager_cpp/map3d_manager_node.hpp, nodes/map3d_manager_cpp/src/map3d_manager_node.cpp, nodes/map3d_manager_cpp/src/main.cpp, nodes/map3d_manager_cpp/test/test_node.cpp, nodes/map3d_manager_cpp/CMakeLists.txt, nodes/map3d_manager_cpp/README.md]
verify: scripts/cpp-node-test.sh map3d_manager_cpp
depends_on: [map3d-manager-status]
owns: [map3d-manager-node]
uses: [map3d-command-contract, map3d-status-contract, map3d-manager-config]
model: sonnet
estimate: 80
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/rtab-map-rgb-d-3d-mapping-on--map3d-manager-node
branch: rtab-map-rgb-d-3d-mapping-on--map3d-manager-node
verified_head: 2954fb8e2f43ee7c98cc3a71b4d20d9243098a76
run: rtab-map-rgb-d-3d-mapping-on
provides: []
acs_for_audit: []
process_gaps: []
---

# Map3d manager node

## Requirement

- `Map3dManagerNode` (rclcpp, executable `map3d_manager_node`, D9): subscribes `command_topic` (String, reliable depth 10), `map_reset_topic` (std_msgs/Empty), `<ns>/mapGraph` (rtabmap_msgs/MapGraph, for map_to_odom and node count), `<ns>/info` (rtabmap_msgs/Info, liveness only), the two camera_info topics (best_effort depth 1); publishes `result_topic` (String JSON, reliable depth 10) and `status_topic` (String JSON every `status_period_s`, only after the node has run one period so ages are real); clients for `<ns>/backup`, `<ns>/reset`, `<ns>/pause`, `<ns>/resume` (std_srvs/Empty).
- Actions: one at a time (a second command while one runs -> `busy`). `save`: call backup, then copy `live.db` to `saved.db` atomically (copy to temp + rename) in `database_dir`, also keep `<name>.db` (same content) so the 3D map is saved under the 2D map's name; result `ok` with db_path and size. `reset`: call reset; result ok. `pause`/`resume`: call the service, track the paused flag. A service not available within `service_timeout_s` -> `timeout`; a failed copy -> `error` with the errno text; invalid commands -> `invalid` with the request id. A `map_reset` message triggers the reset action without a result message and is logged.
- No placeholder data: status only from observed values (null for the unknown, per the status contract).
- Tests first in `test/test_node.cpp` with in-process fake Empty services and a temp database dir: save produces saved.db and <name>.db and an ok result; reset calls the service; busy while one runs; timeout when a service is missing; invalid keeps the request id; map_reset triggers reset; status JSON published with waiting_for_inputs until camera_infos arrive, then mapping; graph correction taken from a MapGraph.
- CMakeLists: the node library, executable and `test_node`. README: purpose, topics/services table, command and status contract (pointing at the headers), save/load semantics (saved.db restored on start by the launch, D9).

## Acceptance conditions

- a1: save backs up RTAB-Map and writes saved.db and <name>.db atomically with an ok result [evidence: command: scripts/cpp-node-test.sh map3d_manager_cpp]
- a2: reset, pause, resume and map_guard resets call the RTAB-Map services; busy, timeout and invalid are reported [evidence: command: scripts/cpp-node-test.sh map3d_manager_cpp]
- a3: status is published from observed values with waiting_for_inputs until inputs arrive [evidence: command: scripts/cpp-node-test.sh map3d_manager_cpp]
- a4: the package builds and its tests and linters pass in docker [evidence: command: scripts/cpp-node-test.sh map3d_manager_cpp]

## Files

- `nodes/map3d_manager_cpp/include/map3d_manager_cpp/map3d_manager_node.hpp`
- `nodes/map3d_manager_cpp/src/map3d_manager_node.cpp`
- `nodes/map3d_manager_cpp/src/main.cpp`
- `nodes/map3d_manager_cpp/test/test_node.cpp`
- `nodes/map3d_manager_cpp/CMakeLists.txt`
- `nodes/map3d_manager_cpp/README.md`

## Interfaces

- Executable `map3d_manager_node`; topics `/map3d/result_server`, `/map3d/status_server` (downlinked to `/map3d/result`, `/map3d/status`); consumes `/uplink/map3d/command`, `/uplink/map_guard/map_reset`.

## Expected outcome

A command from the web UI saves or resets the RTAB-Map database on the server and the robot sees the result and a live status.

## Steps

- Write test/test_node.cpp; run `scripts/cpp-node-test.sh map3d_manager_cpp`; see `FAIL`.
- Implement node, main and CMake targets; rerun until `PASS`.
- Write the README; commit `feat(map3d_manager_cpp): save, reset and status node`.

## Notes

- Pattern for request/result JSON over topics: poi_store and head_camera zoom.
- Do not subscribe any large uplink stream here (one WiFi copy per server reader, contract rule).
