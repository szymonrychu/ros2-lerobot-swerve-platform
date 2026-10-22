---
title: Head arm pose capture sequence
status: todo
repo: ros2-lerobot-sverve-platform
value: "The node captures the board-on-gripper views by itself, moving only checked poses, asking before every big-joint motion and holding the arm on every exit."
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/sequencer.py, nodes/head_rgbd_calibration/tests/test_sequencer.py]
verify: "cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_sequencer.py tests/test_arm_pose_set.py -q && uv run poe lint"
depends_on: [arm-board-pose-set]
owns: [calib-sequencer]
uses: [calib-still-capture, calib-robot-client, arm-sequence-config, arm-pose-check, head-aim]
model: opus
estimate: 90
actual: ""
review: full
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Head arm pose capture sequence

## Requirement



## Acceptance conditions



## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/sequencer.py`
- `nodes/head_rgbd_calibration/tests/test_sequencer.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
