---
title: Map3d manager status
status: done
repo: ros2-lerobot-sverve-platform
value: "The 3D map reports a truthful health line: state, size, memory, RTAB-Map's graph correction and which inputs are missing."
files: [nodes/map3d_manager_cpp/include/map3d_manager_cpp/status.hpp, nodes/map3d_manager_cpp/src/status.cpp, nodes/map3d_manager_cpp/test/test_status.cpp]
verify: scripts/cpp-node-test.sh map3d_manager_cpp
depends_on: [map3d-manager-config]
owns: [map3d-status-contract]
uses: [map3d-manager-config]
model: sonnet
estimate: 55
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/rtab-map-rgb-d-3d-mapping-on--map3d-manager-status
branch: rtab-map-rgb-d-3d-mapping-on--map3d-manager-status
verified_head: 7776c58165ff33d0a282748aa12b14b63a201a0c
run: rtab-map-rgb-d-3d-mapping-on
provides: [map3d-status-contract]
acs_for_audit: []
process_gaps: []
---

# Map3d manager status

## Requirement

- `status.hpp/.cpp` (core, no ROS; the header comment is the status contract): `StatusInputs` (now, last mapGraph stamp and its `map_to_odom` translation/yaw, node count from the graph, last info stamp, last rgb/depth camera_info stamps, paused flag, db file size or nullopt, cgroup memory bytes or nullopt, last save {name, stamp} or nullopt, last error string) -> `Status build_status(const StatusInputs&, const Config&)` and `std::string to_json(const Status&)`:
  `{"stamp_s", "state": "mapping"|"paused"|"waiting_for_inputs"|"error", "nodes", "db_size_mb", "memory_mb", "graph_correction_m", "graph_correction_deg", "correction_warn_m", "correction_warn_deg", "correction_ok", "missing_inputs": [..], "last_update_age_s", "last_save": {"name", "stamp_s"} | null, "message"}`.
  Rules: an input is missing when never seen or older than `input_stale_s` (names: `rgb_camera_info`, `depth_camera_info`, `rtabmap_info`); state `waiting_for_inputs` when any is missing and not paused, `paused` when paused, `error` when a last error is set within the last minute, else `mapping`; unknown values are JSON null, never 0; `correction_ok` null until a graph arrived.
- `read_cgroup_memory_bytes(const std::string& unit, const std::string& root="/sys/fs/cgroup/system.slice")` reads `memory.current` (nullopt when absent).
- Tests first in `test/test_status.cpp`, seen failing: each state; stale vs fresh inputs; null fields before data; correction over/under the thresholds (translation norm and |yaw|); cgroup reader with a temp dir; JSON round-trip of every field.

## Acceptance conditions

- a1: status state, missing inputs and correction flags follow the rules [evidence: command: scripts/cpp-node-test.sh map3d_manager_cpp]
- a2: unknown values serialise as null, never zero [evidence: command: scripts/cpp-node-test.sh map3d_manager_cpp]
- a3: the package builds and its tests and linters pass in docker [evidence: command: scripts/cpp-node-test.sh map3d_manager_cpp]

## Files

- `nodes/map3d_manager_cpp/include/map3d_manager_cpp/status.hpp`
- `nodes/map3d_manager_cpp/src/status.cpp`
- `nodes/map3d_manager_cpp/test/test_status.cpp`

## Interfaces

- `nodes/map3d_manager_cpp/include/map3d_manager_cpp/status.hpp` (`StatusInputs`, `Status`, `build_status`, `to_json`, `read_cgroup_memory_bytes`): the `map3d-status-contract` part read by the web UI and mcp_server.

## Expected outcome

A pure function turns the manager's observations into the status JSON every consumer reads.

## Steps

- Write test/test_status.cpp; run `scripts/cpp-node-test.sh map3d_manager_cpp`; see `FAIL`.
- Implement; rerun until `PASS`.
- Commit `feat(map3d_manager_cpp): status contract`.

## Notes

- `graph_correction_*` is RTAB-Map's `MapGraph.map_to_odom`: identity while D7 holds; anything else means RTAB-Map built its own links and the 3D map has left slam_toolbox's frame.
