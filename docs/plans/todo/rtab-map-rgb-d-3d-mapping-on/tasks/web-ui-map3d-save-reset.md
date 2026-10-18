---
title: Web UI map3d save reset
status: todo
repo: ros2-lerobot-sverve-platform
value: "Saving or resetting the map in the web UI saves or resets the 3D map with it, so the 3D map never outlives the 2D map frame it belongs to."
files: [nodes/web_ui/web_ui/server.py, nodes/web_ui/web_ui/bridge.py, nodes/web_ui/tests/test_map3d_commands.py, nodes/web_ui/README.md]
verify: "cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint"
depends_on: [web-ui-map3d-cloud-backend, map3d-manager-config]
owns: []
uses: [web-ui-map3d-roles, map3d-command-contract]
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

# Web UI map3d save reset

## Requirement

- `POST /api/map/save`: after slam_toolbox `serialize_map` succeeds, publishes one `save` command on `map3d_command_topic` (JSON per the map3d command contract: `request_id` uuid4 hex, `action: "save"`, `name` = basename of `map_save_path`) and waits up to `map3d_command_timeout_s` (new TabConfig field, default 15 s, > 0) for the matching `/map3d/result`. The response gains `map3d: {status, message}`; a 3D failure or timeout does not undo the 2D save and is reported as `map3d.status` `error`/`timeout` with HTTP 200 and the 2D result unchanged (my call: the 2D map is the source of truth).
- `POST /api/map/reset`: after slam_toolbox reset succeeds, publishes `reset` and reports `map3d` the same way. A reset that slam_toolbox refused publishes nothing.
- Bridge: a reliable depth-10 publisher on `map3d_command_topic`; results matched by `request_id` (the poi/grasp request-result pattern); unmatched results ignored.
- Tests first in `tests/test_map3d_commands.py`, seen failing: save publishes exactly one command after a successful 2D save and reports the 3D result; 3D timeout leaves the 2D success intact; failed 2D save or reset publishes nothing; result matching by request_id; concurrent save calls coalesce like today's.
- README: the save/reset rows of the API table say they also act on the 3D map, and a `## 3D map` section (layers, status, save/reset behaviour).

## Acceptance conditions

- a1: a successful 2D save or reset publishes one matching map3d command and reports its result [evidence: test: nodes/web_ui/tests/test_map3d_commands.py]
- a2: a failed 2D save or reset publishes nothing and a 3D timeout keeps the 2D result [evidence: test: nodes/web_ui/tests/test_map3d_commands.py]
- a3: the README documents the 3D map behaviour [evidence: doc-section: nodes/web_ui/README.md#3D map]
- a4: web_ui tests and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/web_ui/web_ui/server.py`
- `nodes/web_ui/web_ui/bridge.py`
- `nodes/web_ui/tests/test_map3d_commands.py`
- `nodes/web_ui/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
