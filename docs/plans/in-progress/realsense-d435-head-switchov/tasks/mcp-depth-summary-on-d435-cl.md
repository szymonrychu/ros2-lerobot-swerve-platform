---
title: MCP depth summary on D435 cloud
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The agent's 3D obstacle summary comes from the D435 cloud."
files: [nodes/mcp_server/mcp_server/config.py, nodes/mcp_server/mcp_server/ros_iface.py, nodes/mcp_server/mcp_server/tool_context.py, nodes/mcp_server/mcp_server/tools.py, nodes/mcp_server/tests/test_map_summary_tof.py, nodes/mcp_server/tests/test_map_summary_depth.py, nodes/mcp_server/tests/test_tof_obstacles.py, ansible/group_vars/client.yml]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_map_summary_depth.py tests/test_tof_obstacles.py tests/test_tools.py -q && uv run poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py -q && uv run poe test-ansible"
depends_on: [laser-filter-on-the-d435-clo, mcp-image-tools-front-only]
model: opus
review: ""
verified_head: ""
points: 3
expected_minutes: 34
expected_tokens: 2250000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# MCP depth summary on D435 cloud

## Requirement

Goal G7: "Every MCP tool that uses ... ToF or RGB-D is D435-backed". Design D8: "get_map_summary obstacles_3d reads the D435 cloud (topics.depth_points)".

- `mcp_server/config.py` Topics: `tof_points` renamed `depth_points`, default `/head/d435/points_filtered`.
- `mcp_server/ros_iface.py` and `mcp_server/tool_context.py`: `tof_points()` renamed `depth_points()`, and `TofPoints` renamed `DepthPoints` (the same lazy subscription, latest message only).
- `mcp_server/tools.py` `get_map_summary`: `obstacles_3d` from `depth_points`; the summary text says "head depth (D435)". The ground note is unchanged. `summarize_tof` is renamed `summarize_depth` wherever it lives.
- `ansible/group_vars/client.yml` mcp_server topics: `depth_points: /head/d435/points_filtered` (comment: D435 cloud).
- Tests: `git mv tests/test_map_summary_tof.py tests/test_map_summary_depth.py` and update it, together with `tests/test_tof_obstacles.py`, for the new names (the ToF names are no longer accurate; behaviour assertions kept, and the cloud frame is `d435_depth_optical_frame`).

## Acceptance conditions

- AC1: get_map_summary obstacles_3d comes from the D435 cloud topic [evidence: test:nodes/mcp_server/tests/test_map_summary_depth.py::test_obstacles_3d_from_the_d435_cloud]
- AC2: no stale cloud is summarised (latest-only and freshness kept) [evidence: test:nodes/mcp_server/tests/test_map_summary_depth.py::test_stale_depth_cloud_is_not_summarised]
- AC3: the deployed config names topics.depth_points [evidence: command:uv run pytest tests/test_mcp_server_config.py -q]
- AC4: mcp_server suites and lint pass [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests/test_map_summary_depth.py tests/test_tof_obstacles.py tests/test_tools.py -q && uv run poe lint]

## Files

- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/mcp_server/tool_context.py`
- `nodes/mcp_server/mcp_server/tools.py`
- `nodes/mcp_server/tests/test_map_summary_tof.py`
- `nodes/mcp_server/tests/test_map_summary_depth.py`
- `nodes/mcp_server/tests/test_tof_obstacles.py`
- `ansible/group_vars/client.yml`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- `git mv` and update the summary test, update test_tof_obstacles.py; run them; expect failures.
- Rename config, protocol, ros_iface and tool code; update client.yml; rerun; expect pass.
- Run the verify command; commit `refactor(mcp_server): map summary depth from the D435 cloud`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
