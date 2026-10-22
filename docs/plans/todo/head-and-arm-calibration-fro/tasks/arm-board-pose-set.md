---
title: Arm board pose set
status: todo
repo: ros2-lerobot-sverve-platform
value: "A shipped arm pose set that is proven safe by FK, keeps the board in view and is known to give a passing fit before the arm ever moves."
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/config.py, nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml, ansible/group_vars/client.yml, nodes/head_rgbd_calibration/tests/test_arm_pose_set.py, nodes/head_rgbd_calibration/tests/test_config.py]
verify: "(cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_arm_pose_set.py tests/test_config.py -q && uv run poe lint) && uv run --frozen pytest tests/test_head_rgbd_calibration_deploy.py -q"
depends_on: [robot-mcp-client, arm-pose-safety-check, head-from-arm-board-solver, board-on-gripper-hardware-st]
owns: [arm-sequence-config]
uses: [arm-pose-check, head-aim, head-arm-fit, head-arm-synthetic-scenes]
model: opus
estimate: 75
actual: ""
review: full
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Arm board pose set

## Requirement



## Acceptance conditions



## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py`
- `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`
- `ansible/group_vars/client.yml`
- `nodes/head_rgbd_calibration/tests/test_arm_pose_set.py`
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
