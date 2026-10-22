---
title: Calib config and view types
status: todo
repo: ros2-lerobot-sverve-platform
value: "One validated place for the board mount, the fit limits, the trusted arm gauge and the view type, so every later fit, check and store task reads the same values."
files: [shared/ros2_common/head_rgbd_calibration/config.py, shared/ros2_common/head_rgbd_calibration/charuco.py, shared/ros2_common/head_rgbd_calibration/views.py, tests/test_head_arm_calib_config.py]
verify: "uv run --frozen pytest tests/test_head_arm_calib_config.py tests/test_head_rgbd_calib_core.py -q && uv run poe lint"
depends_on: []
owns: [head-arm-calib-config, head-arm-view-types]
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

# Calib config and view types

## Requirement



## Acceptance conditions



## Files

- `shared/ros2_common/head_rgbd_calibration/config.py`
- `shared/ros2_common/head_rgbd_calibration/charuco.py`
- `shared/ros2_common/head_rgbd_calibration/views.py`
- `tests/test_head_arm_calib_config.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
