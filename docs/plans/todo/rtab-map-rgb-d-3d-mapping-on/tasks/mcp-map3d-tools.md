---
title: Mcp map3d tools
status: todo
repo: ros2-lerobot-sverve-platform
value: "The agent can ask what is at a spot, how tall it is and how much free space surrounds a point in 3D."
files: [nodes/mcp_server/mcp_server/map3d_tools.py, nodes/mcp_server/mcp_server/tools.py, nodes/mcp_server/mcp_server/config.py, nodes/mcp_server/mcp_server/ros_iface.py, nodes/mcp_server/tests/test_map3d_tools.py, nodes/mcp_server/tests/test_tools.py, nodes/mcp_server/tests/test_battery_gate.py]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint"
depends_on: [mcp-map3d-grid-model, map3d-manager-status]
owns: [mcp-map3d-tools]
uses: [map3d-voxel-query, map3d-status-contract]
model: sonnet
estimate: 75
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Mcp map3d tools

## Requirement

- `map3d_tools.py` registers three sensor tools (read-only, no lease, `SENSOR_TOOL_NAMES`):
  - `get_3d_map_status()`: the latest `/map3d/status` JSON plus the local cloud's voxel count, bounds, age and `alignment_agreement` against /map (D7 check); `null` fields with a note when nothing was received.
  - `query_3d_map_at(x, y, radius_m=0.3)` (map frame metres, radius 0.05..2.0): `query_at` result rounded (m to 3 decimals), with `source` and a note explaining lidar-only heights are unknown.
  - `get_3d_free_space(x, y, radius_m=1.0, z_min_m=0.02, z_max_m=null)` (radius 0.2..5.0; `z_max_m` defaults to `map3d.robot_height_m`): `free_space` per sector with bearings in degrees and `observed`.
- Data: lazily subscribes `topics.map3d_cloud` (PointCloud2, best_effort, keep_last 1) and `topics.map3d_status` on first use through the existing on-demand subscription machinery (`on_demand.linger_s` reaper), rebuilds the VoxelMap only when the cloud stamp changes, and reuses the node's /map subscription for the LidarGrid. A tool called before any cloud arrives raises a ToolError "no 3D map received yet" (no placeholder answer).
- `config.py`: `topics.map3d_cloud` (default `/map3d/cloud`), `topics.map3d_status` (default `/map3d/status`), `Map3dSettings` (`frame_id` `rtabmap_map`, `voxel_m` 0.05, `lidar_plane_height_m` 0.20 = base_link->laser_frame z, `robot_height_m` 0.55, `max_age_s` 120 after which results carry a stale note).
- `tools.py`: registers the module and adds its names to the sensor tool set; `test_tools.py` and `test_battery_gate.py` gain the three tools.
- Tests first in `tests/test_map3d_tools.py` with the fakes in `tests/fakes.py`: each tool's output on a synthetic cloud + grid, the no-data ToolError, a stale note after max_age_s, argument bounds, VoxelMap rebuilt once per stamp.

## Acceptance conditions

- a1: the three 3D map tools answer from a synthetic cloud and grid [evidence: test: nodes/mcp_server/tests/test_map3d_tools.py]
- a2: tools refuse with a clear error before any 3D map arrived [evidence: test: nodes/mcp_server/tests/test_map3d_tools.py]
- a3: the tools are registered as sensor tools and pass the battery gate table [evidence: test: nodes/mcp_server/tests/test_tools.py]
- a4: mcp_server tests and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/mcp_server/mcp_server/map3d_tools.py`
- `nodes/mcp_server/mcp_server/tools.py`
- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/tests/test_map3d_tools.py`
- `nodes/mcp_server/tests/test_tools.py`
- `nodes/mcp_server/tests/test_battery_gate.py`

## Interfaces

- MCP tools `get_3d_map_status`, `query_3d_map_at`, `get_3d_free_space` (`nodes/mcp_server/mcp_server/map3d_tools.py:SENSOR_TOOL_NAMES`): the `mcp-map3d-tools` part; config `topics.map3d_cloud`, `topics.map3d_status`, `map3d.*`.

## Expected outcome

The agent can query the 3D map through three read-only tools.

## Steps

- Write `tests/test_map3d_tools.py` and the test_tools/battery gate entries; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_map3d_tools.py tests/test_tools.py tests/test_battery_gate.py -q`; see them fail.
- Implement; run `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint`.
- Commit `feat(mcp_server): 3D map query tools`.

## Notes

- Pattern: `frontier_tools.py` (module TOOL_NAMES, register(ctx), pydantic results, tool_errors).
- Do not deploy mcp_server during an agent session (MEMORY).
