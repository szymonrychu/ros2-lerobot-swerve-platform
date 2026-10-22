---
title: Session kinds joint poses
status: todo
repo: ros2-lerobot-sverve-platform
value: "Board-on-gripper views are stored with all seven measured joints in the existing session store, never with invented values."
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/store.py, nodes/head_rgbd_calibration/tests/test_store.py, nodes/head_rgbd_calibration/tests/pose_factory.py]
verify: "cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_store.py tests/test_images.py -q && uv run poe lint"
depends_on: [calib-config-and-view-types]
owns: [calib-session-kinds]
uses: [head-arm-view-types, rgbd-pose-hints]
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

# Session kinds joint poses

## Requirement



## Acceptance conditions



## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/store.py`
- `nodes/head_rgbd_calibration/tests/test_store.py`
- `nodes/head_rgbd_calibration/tests/pose_factory.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
