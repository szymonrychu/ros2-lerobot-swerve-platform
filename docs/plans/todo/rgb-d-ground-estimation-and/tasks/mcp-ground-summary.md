---
title: MCP ground summary
status: todo
repo: ros2-lerobot-sverve-platform
value: "The agent sees the floor height, slope, steps and drop-offs in get_map_summary."
files: [nodes/mcp_server/mcp_server/config.py, nodes/mcp_server/mcp_server/ros_iface.py, nodes/mcp_server/mcp_server/models.py, nodes/mcp_server/mcp_server/tools.py, nodes/mcp_server/tests/test_ground_summary.py, nodes/mcp_server/README.md, ansible/group_vars/client.yml]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint"
depends_on: [ground-node-type]
owns: [mcp-ground-state-client]
uses: [ground-state, ground-topics, ground-node-unit]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# MCP ground summary

## Requirement

- mcp_server config (design D9): `topics.ground_state` (default `/ground/state`) and `timeouts.ground_state_stale_s` (default 1.0, > 0) in `mcp_server/config.py`; the client.yml mcp_server config block sets `topics.ground_state: /ground/state` with a comment (like `tof_points`).
- `ros_iface.py`: a permanent subscription to `topics.ground_state` (std_msgs/String, reliable + transient_local depth 1, like `tilt_guard_state`), parsed with a pydantic model `GroundState` (in `models.py`, fields = the ground-state contract keys of `nodes/ground_estimator_cpp/include/ground_estimator_cpp/state.hpp` STATE_KEYS, `extra="forbid"`, regions as a list of `GroundRegion` with `name, kind, height_m, frame, edge {point, direction, side}, distance_m, points`); an invalid message is logged once per distinct error and not stored; `ground_state() -> tuple[GroundState, float] | None` returns the latest with its receive age, or None when absent or older than `ground_state_stale_s` (part `mcp-ground-state-client`).
- `models.py` `MapSummary.ground: GroundSummary | None` with field descriptions for the agent: floor height relative to base_link (m), plane source (fit/held), slope (deg) and slope_level, plane_vs_base (deg, large means head calibration error), the regions (kind, height, distance, edge), colour mismatch ratio, state age (s); null with a note `ground estimate unavailable: no fresh /ground/state` when absent, and with a note `no floor plane: <reason>` when `plane` is null.
- `tools.py` `get_map_summary`: fills `ground` and documents it in the tool docstring (what it means, that it covers only what the head ToF sees, that obstacles_3d stays the fixed-band summary).
- Tests first in `nodes/mcp_server/tests/test_ground_summary.py`, seen failing: the contract fixture `nodes/ground_estimator_cpp/test/data/state_example.json` parses into GroundState (path read from the repo); unknown or missing keys rejected; stale and absent states give the note; a plane-null state gives the reason note; summary fields mapped and rounded; config defaults and the client.yml value equal `/ground/state` and equal the ground_estimator_cpp `output_namespace` + `/state`.
- `nodes/mcp_server/README.md`: the ground field of get_map_summary, the topic and the stale timeout.

## Acceptance conditions

- a1: the ground state contract fixture parses into GroundState and malformed states are rejected [evidence: test: nodes/mcp_server/tests/test_ground_summary.py]
- a2: get_map_summary reports the ground estimate, or a note when it is stale, absent or plane-less [evidence: test: nodes/mcp_server/tests/test_ground_summary.py]
- a3: the topic config is consistent with the ground_estimator_cpp output and client.yml [evidence: test: nodes/mcp_server/tests/test_ground_summary.py]
- a4: the full mcp_server suite and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]
- a5: the mcp_server README documents the ground field [evidence: command: grep -q 'ground_state' nodes/mcp_server/README.md]

## Files

- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/mcp_server/models.py`
- `nodes/mcp_server/mcp_server/tools.py`
- `nodes/mcp_server/tests/test_ground_summary.py`
- `nodes/mcp_server/README.md`
- `ansible/group_vars/client.yml`

## Interfaces

`GroundState`, `GroundRegion`, `GroundSummary` (models.py); `RosInterface.ground_state()` (part `mcp-ground-state-client`); consumes part `ground-state`.

## Expected outcome

`cd nodes/mcp_server && uv run --frozen pytest tests/test_ground_summary.py -q` passes and the full mcp suite stays green.

## Steps

- write tests/test_ground_summary.py
- run it, see FAIL
- implement config, models, ros_iface, tools
- run `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- update the README
- commit `feat(mcp_server): ground estimate in get_map_summary`.

## Notes

pattern: `tilt_guard_state` subscription (ros_iface.py:131, 798) and `tof_points` (ros_iface.py:1859). Deploying mcp_server kills a running agent session; do not deploy here.
