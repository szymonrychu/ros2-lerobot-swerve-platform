---
title: Arm offsets plate solver
status: todo
repo: ros2-lerobot-sverve-platform
value: "The arm joint offsets are measured against the calibrated head, which is what grasping from head images needs."
files: [shared/ros2_common/head_rgbd_calibration/arm_offsets_fit.py, tests/head_arm_synthetic.py, tests/test_arm_offsets_fit.py]
verify: "uv run --frozen pytest tests/test_arm_offsets_fit.py -q && uv run poe lint"
depends_on: []
owns: [arm-offsets-fit]
uses: []
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

# Arm offsets plate solver

## Requirement



## Acceptance conditions



## Files

- `shared/ros2_common/head_rgbd_calibration/arm_offsets_fit.py`
- `tests/head_arm_synthetic.py`
- `tests/test_arm_offsets_fit.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
