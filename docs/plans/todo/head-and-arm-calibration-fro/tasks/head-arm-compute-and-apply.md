---
title: Head arm compute and apply
status: todo
repo: ros2-lerobot-sverve-platform
value: "A captured session becomes a checked head chain result that is applied to every consumer through the floor fit's one apply path, or refused."
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/head_arm_results.py, nodes/head_rgbd_calibration/head_rgbd_calibration/floor_results.py, nodes/head_rgbd_calibration/tests/test_head_arm_results.py, nodes/head_rgbd_calibration/tests/test_floor_results.py]
verify: "cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_head_arm_results.py tests/test_floor_results.py tests/test_floor_api.py tests/test_apply.py -q && uv run poe lint"
depends_on: [head-from-arm-board-solver, joint-state-still-capture]
owns: [calib-head-arm-results]
uses: [head-arm-fit, head-arm-synthetic-scenes, calib-session-kinds, head-chain-override, head-chain-applied-record, head-rgbd-applied-calibration, rgbd-result-pose-errors]
model: ""
estimate: 75
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Head arm compute and apply

## Requirement



## Acceptance conditions



## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/head_arm_results.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/floor_results.py`
- `nodes/head_rgbd_calibration/tests/test_head_arm_results.py`
- `nodes/head_rgbd_calibration/tests/test_floor_results.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
