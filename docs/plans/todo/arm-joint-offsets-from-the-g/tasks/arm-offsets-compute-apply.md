---
title: Arm offsets compute apply
status: todo
repo: ros2-lerobot-sverve-platform
value: The arm result gives a reviewable joint_offsets_rad snippet.
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py, nodes/head_rgbd_calibration/head_rgbd_calibration/apply.py, nodes/head_rgbd_calibration/head_rgbd_calibration/jobs.py, nodes/head_rgbd_calibration/tests/test_compute_arm.py, nodes/head_rgbd_calibration/tests/test_apply.py, nodes/head_rgbd_calibration/tests/test_jobs.py]
verify: "cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_compute_arm.py tests/test_apply.py tests/test_jobs.py -q && uv run poe lint"
depends_on: [arm-offsets-plate-solver]
owns: []
uses: [arm-offsets-fit]
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

# Arm offsets compute apply

## Requirement



## Acceptance conditions



## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/apply.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/jobs.py`
- `nodes/head_rgbd_calibration/tests/test_compute_arm.py`
- `nodes/head_rgbd_calibration/tests/test_apply.py`
- `nodes/head_rgbd_calibration/tests/test_jobs.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
