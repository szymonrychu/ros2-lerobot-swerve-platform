---
title: Head from arm board solver
status: todo
repo: ros2-lerobot-sverve-platform
value: "Head yaw, mount x/y and the rest of the head chain are measured against the trusted arm, which the floor fit cannot see."
files: [shared/ros2_common/head_rgbd_calibration/head_arm_fit.py, tests/head_arm_synthetic.py, tests/test_head_arm_fit.py]
verify: "uv run --frozen pytest tests/test_head_arm_fit.py tests/test_head_arm_calib_config.py -q && uv run poe lint"
depends_on: [calib-config-and-view-types]
owns: [head-arm-fit, head-aim, head-arm-synthetic-scenes]
uses: [head-arm-calib-config, head-arm-view-types, urdf-chain-fk, head-chain-override]
model: opus
estimate: 90
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Head from arm board solver

## Requirement



## Acceptance conditions



## Files

- `shared/ros2_common/head_rgbd_calibration/head_arm_fit.py`
- `tests/head_arm_synthetic.py`
- `tests/test_head_arm_fit.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
