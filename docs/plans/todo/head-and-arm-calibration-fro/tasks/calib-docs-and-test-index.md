---
title: Calib docs and test index
status: todo
repo: ros2-lerobot-sverve-platform
value: "A mid-level ROS2 developer can understand, run and extend the head-from-arm calibration from the READMEs."
files: [shared/README.md, nodes/head_rgbd_calibration/README.md, nodes/head_state_publisher/README.md, nodes/mcp_server/README.md, nodes/web_ui/README.md, ansible/README.md, docs/markers.md, tests/README.md]
verify: "grep -q 'head_arm_fit' shared/README.md && grep -q 'head-arm' nodes/head_rgbd_calibration/README.md && grep -q 'test_head_arm_fit.py' tests/README.md && grep -q 'arm-joint-offsets-from-the-g' nodes/head_rgbd_calibration/README.md"
depends_on: [head-arm-calibrate-ui-sectio, head-tilt-range-widened]
owns: []
uses: []
model: ""
estimate: 60
actual: ""
review: skip
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Calib docs and test index

## Requirement



## Acceptance conditions



## Files

- `shared/README.md`
- `nodes/head_rgbd_calibration/README.md`
- `nodes/head_state_publisher/README.md`
- `nodes/mcp_server/README.md`
- `nodes/web_ui/README.md`
- `ansible/README.md`
- `docs/markers.md`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
