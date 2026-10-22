---
title: Web UI applied head URDF
status: todo
repo: ros2-lerobot-sverve-platform
value: The web UI 3D view and head tab draw the head where it really is after a calibration.
files: [nodes/web_ui/web_ui/server.py, nodes/web_ui/web_ui/config.py, nodes/web_ui/config/default.yaml, nodes/web_ui/tests/test_head_urdf.py, ansible/group_vars/client.yml]
verify: "cd nodes/web_ui && uv run --frozen pytest tests/test_head_urdf.py tests/test_server.py -q && uv run poe lint"
depends_on: [mcp-front-camera-applied]
owns: []
uses: [head-chain-override]
model: ""
estimate: 45
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Web UI applied head URDF

## Requirement



## Acceptance conditions



## Files

- `nodes/web_ui/web_ui/server.py`
- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/config/default.yaml`
- `nodes/web_ui/tests/test_head_urdf.py`
- `ansible/group_vars/client.yml`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
