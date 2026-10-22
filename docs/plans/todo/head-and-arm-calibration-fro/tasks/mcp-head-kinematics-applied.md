---
title: MCP head kinematics applied
status: todo
repo: ros2-lerobot-sverve-platform
value: "The agent's head aiming and head-camera geometry use the measured chain, reloaded on apply without killing an agent session."
files: [nodes/mcp_server/mcp_server/head_kinematics.py, nodes/mcp_server/mcp_server/head.py, nodes/mcp_server/mcp_server/config.py, nodes/mcp_server/mcp_server/ros_iface.py, nodes/mcp_server/tests/test_head_kinematics.py, nodes/mcp_server/tests/test_head_controller.py, nodes/mcp_server/tests/test_config.py]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_head_kinematics.py tests/test_head_controller.py tests/test_head_tools.py tests/test_camera_scene_head.py tests/test_config.py -q && uv run poe lint"
depends_on: []
owns: []
uses: [head-chain-override]
model: ""
estimate: 70
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# MCP head kinematics applied

## Requirement



## Acceptance conditions



## Files

- `nodes/mcp_server/mcp_server/head_kinematics.py`
- `nodes/mcp_server/mcp_server/head.py`
- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/tests/test_head_kinematics.py`
- `nodes/mcp_server/tests/test_head_controller.py`
- `nodes/mcp_server/tests/test_config.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
