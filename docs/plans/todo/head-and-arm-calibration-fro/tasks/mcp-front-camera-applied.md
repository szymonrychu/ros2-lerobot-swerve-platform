---
title: MCP front camera applied
status: todo
repo: ros2-lerobot-sverve-platform
value: "The agent's front camera tools become calibrated: pixel rays use the applied intrinsics and the measured camera mount."
files: [nodes/mcp_server/mcp_server/config.py, nodes/mcp_server/mcp_server/camera_scene.py, ansible/group_vars/client.yml, nodes/mcp_server/tests/test_camera_config.py, nodes/mcp_server/tests/test_camera_scene_head.py]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_camera_config.py tests/test_camera_scene_head.py tests/test_config.py -q && uv run poe lint"
depends_on: [mcp-head-kinematics-applied, arm-board-pose-set]
owns: []
uses: [head-chain-override]
model: ""
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# MCP front camera applied

## Requirement



## Acceptance conditions



## Files

- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/camera_scene.py`
- `ansible/group_vars/client.yml`
- `nodes/mcp_server/tests/test_camera_config.py`
- `nodes/mcp_server/tests/test_camera_scene_head.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
