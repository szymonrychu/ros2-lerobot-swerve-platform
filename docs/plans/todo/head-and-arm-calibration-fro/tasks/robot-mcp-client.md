---
title: Robot MCP client
status: todo
repo: ros2-lerobot-sverve-platform
value: "The calibrator moves the head and arm only through mcp_server's guarded tools (leases, floor guard, body check, roll guard, battery gate)."
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/robot_client.py, nodes/head_rgbd_calibration/pyproject.toml, nodes/head_rgbd_calibration/uv.lock, nodes/head_rgbd_calibration/head_rgbd_calibration/config.py, nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml, ansible/group_vars/client.yml, nodes/head_rgbd_calibration/tests/test_robot_client.py, nodes/head_rgbd_calibration/tests/test_config.py]
verify: "(cd nodes/head_rgbd_calibration && uv lock --check && uv run --frozen pytest tests/test_robot_client.py tests/test_config.py -q && uv run poe lint) && uv run --frozen pytest tests/test_head_rgbd_calibration_deploy.py -q"
depends_on: [joint-state-still-capture]
owns: [calib-robot-client]
uses: []
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

# Robot MCP client

## Requirement



## Acceptance conditions



## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/robot_client.py`
- `nodes/head_rgbd_calibration/pyproject.toml`
- `nodes/head_rgbd_calibration/uv.lock`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py`
- `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`
- `ansible/group_vars/client.yml`
- `nodes/head_rgbd_calibration/tests/test_robot_client.py`
- `nodes/head_rgbd_calibration/tests/test_config.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
