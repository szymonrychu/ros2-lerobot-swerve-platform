---
title: Joint state still capture
status: todo
repo: ros2-lerobot-sverve-platform
value: "Each view pairs a still head image of the board with the head and arm joints measured at that instant, or is not stored at all."
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/capture.py, nodes/head_rgbd_calibration/head_rgbd_calibration/config.py, nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml, ansible/group_vars/client.yml, nodes/head_rgbd_calibration/tests/test_capture.py, nodes/head_rgbd_calibration/tests/test_config.py, nodes/head_rgbd_calibration/tests/conftest.py]
verify: "(cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_capture.py tests/test_config.py tests/test_store.py -q && uv run poe lint) && uv run --frozen pytest tests/test_head_rgbd_calibration_deploy.py -q"
depends_on: [session-kinds-joint-poses]
owns: [calib-still-capture]
uses: [calib-session-kinds, head-arm-calib-config]
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

# Joint state still capture

## Requirement



## Acceptance conditions



## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/capture.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py`
- `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`
- `ansible/group_vars/client.yml`
- `nodes/head_rgbd_calibration/tests/test_capture.py`
- `nodes/head_rgbd_calibration/tests/test_config.py`
- `nodes/head_rgbd_calibration/tests/conftest.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
