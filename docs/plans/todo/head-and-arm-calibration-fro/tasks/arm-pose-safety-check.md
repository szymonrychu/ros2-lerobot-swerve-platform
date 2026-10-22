---
title: Arm pose safety check
status: todo
repo: ros2-lerobot-sverve-platform
value: "No arm pose or transit with the board on the gripper is commanded unless FK shows the tool, the board and the arm links clear of the floor, the body and the head."
files: [shared/ros2_common/head_rgbd_calibration/arm_pose_check.py, tests/test_arm_pose_check.py]
verify: "uv run --frozen pytest tests/test_arm_pose_check.py -q && uv run poe lint"
depends_on: [calib-config-and-view-types]
owns: [arm-pose-check]
uses: [head-arm-calib-config, urdf-chain-fk]
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

# Arm pose safety check

## Requirement



## Acceptance conditions



## Files

- `shared/ros2_common/head_rgbd_calibration/arm_pose_check.py`
- `tests/test_arm_pose_check.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
