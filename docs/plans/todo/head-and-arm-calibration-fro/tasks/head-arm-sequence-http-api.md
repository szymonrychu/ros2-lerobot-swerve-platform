---
title: Head arm sequence HTTP API
status: todo
repo: ros2-lerobot-sverve-platform
value: "The web UI can drive the guided head-from-arm calibration, with every confirm and refusal enforced by the node, not only by the browser."
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/head_arm_api.py, nodes/head_rgbd_calibration/head_rgbd_calibration/__main__.py, nodes/head_rgbd_calibration/tests/test_head_arm_api.py, nodes/head_rgbd_calibration/tests/test_main.py, nodes/head_rgbd_calibration/tests/contract.py, nodes/head_rgbd_calibration/tests/fixtures/api/head_arm_state.json, nodes/head_rgbd_calibration/tests/fixtures/api/head_arm_result.json, nodes/head_rgbd_calibration/tests/fixtures/api/head_arm_sessions.json]
verify: "cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint"
depends_on: [head-arm-pose-capture-sequen, head-arm-compute-and-apply]
owns: [head-arm-calib-api]
uses: [calib-sequencer, calib-robot-client, calib-head-arm-results]
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

# Head arm sequence HTTP API

## Requirement



## Acceptance conditions



## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/head_arm_api.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/__main__.py`
- `nodes/head_rgbd_calibration/tests/test_head_arm_api.py`
- `nodes/head_rgbd_calibration/tests/test_main.py`
- `nodes/head_rgbd_calibration/tests/contract.py`
- `nodes/head_rgbd_calibration/tests/fixtures/api/head_arm_state.json`
- `nodes/head_rgbd_calibration/tests/fixtures/api/head_arm_result.json`
- `nodes/head_rgbd_calibration/tests/fixtures/api/head_arm_sessions.json`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
