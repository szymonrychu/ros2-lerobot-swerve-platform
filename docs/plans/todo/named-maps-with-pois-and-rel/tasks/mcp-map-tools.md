---
title: MCP map tools
status: todo
repo: ros2-lerobot-sverve-platform
value: "The agent can save, list and load named maps and learns whether the robot is localised after a load."
files: [nodes/mcp_server/mcp_server/map_tools.py, nodes/mcp_server/mcp_server/ros_iface.py, nodes/mcp_server/mcp_server/config.py, nodes/mcp_server/mcp_server/tools.py, nodes/mcp_server/tests/test_map_tools.py]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_map_tools.py tests/test_config.py -q && uv run --frozen poe lint"
depends_on: [map-manager-contract]
owns: []
uses: [map-manager-contract]
model: sonnet
estimate: 80
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# MCP map tools

## Requirement

mcp_server tools for named maps (design D11).

- `config.py`: a `maps` section with the map_manager topics (defaults from `ros2_common.maps`) and timeouts
  (`save_timeout_s` 30, `load_timeout_s` 120, `delete` not exposed).
- `ros_iface.py`: latched subscription to `/map_manager/status` (cached), publisher `/map_manager/command`,
  subscription `/map_manager/result`; a `MapRequests` client (request_id matching like `poi_client.PoiRequests`,
  raising `MapManagerDown` when nothing subscribes, `MapTimeout` on timeout, `RobotError` with map_manager's message
  on `ok: false`).
- `map_tools.py` registered from `tools.py`: `save_map(name, overwrite=False, note="")`; `list_maps()` returning maps,
  active map and localisation (an error when no status has arrived, never an invented empty list);
  `load_map(name, x=None, y=None, yaw=None)` which calls the existing stop path first, sends the pose only when all of
  x, y, yaw are given (a partial pose is an error), and returns the verdict (state, ratio, pose, reason). Tool
  descriptions say: a load replaces the current map and POIs (an autosave is kept), do not navigate when the state is
  not `localised`, retry with a pose hint when the robot is elsewhere.
- `tests/test_map_tools.py` (test first): each tool's request payload and result mapping, partial pose error, stop
  called before load, map_manager down, timeout, rejected command, list_maps before any status.

## Acceptance conditions

- a1: save_map, list_maps and load_map send the right commands and map the results [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_map_tools.py -q]
- a2: load_map stops the robot before sending the load and rejects a partial pose [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_map_tools.py -q -k 'stop or partial']
- a3: list_maps errors before the first status instead of returning an empty list [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_map_tools.py -q -k before_status]
- a4: mcp_server lint passes [evidence: command: cd nodes/mcp_server && uv run --frozen poe lint]

## Files

- `nodes/mcp_server/mcp_server/map_tools.py`
- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/tools.py`
- `nodes/mcp_server/tests/test_map_tools.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
