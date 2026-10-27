---
title: Floor sweep on D435 depth
status: todo
repo: ros2-lerobot-sverve-platform
value: The head chain floor fit runs on the D435 depth.
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/floor_sweep.py, nodes/head_rgbd_calibration/head_rgbd_calibration/floor_results.py, nodes/head_rgbd_calibration/tests/test_floor_sweep.py, nodes/head_rgbd_calibration/tests/test_floor_results.py, nodes/head_rgbd_calibration/tests/test_floor_api.py]
verify: "cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_floor_sweep.py tests/test_floor_results.py tests/test_floor_api.py -q && uv run poe lint"
depends_on: [calibration-config-on-d435, calibration-library-d435-fra, d435-obstacle-cloud-node]
model: sonnet
review: ""
verified_head: ""
points: 3
expected_minutes: 34
expected_tokens: 2250000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Floor sweep on D435 depth

## Requirement

Goal G8: "... floor fit ...". Design D10: "Head floor fit runs on /head/d435/points in d435_depth_optical_frame." Servo rule: the sweep moves the head only (unchanged).

- `head_rgbd_calibration/floor_sweep.py`: clouds from `floor.cloud_topic` (`/head/d435/points`), planes fitted in `floor.sensor_frame` (`d435_depth_optical_frame`). The ToF-specific settings (ToF frame, ToF near-floor assumptions in comments) are gone. The pan/tilt grid is re-checked for the D435 FOV: tilt values keep 0.4-2 m of floor in view; change `floor.tilt_deg` in the config defaults only with a comment giving the FOV reason.
- `head_rgbd_calibration/floor_results.py`: calls `fit_head_chain_floor(..., sensor_frame=cfg.floor.sensor_frame)`, `base_urdf` applies only the head chain overrides, results write `head_chain.json` and `head.urdf` and no `tof_joint.json`.
- Tests: `tests/test_floor_sweep.py` and `tests/test_floor_results.py` with D435 frames and topics; tof_joint outputs are no longer accurate. New `test_floor_results.py::test_result_has_no_tof_joint`.

## Acceptance conditions

- AC1: the sweep fits planes from the D435 cloud in d435_depth_optical_frame [evidence: test:nodes/head_rgbd_calibration/tests/test_floor_sweep.py::test_planes_in_the_d435_depth_frame]
- AC2: a floor result writes the head chain and no tof_joint [evidence: test:nodes/head_rgbd_calibration/tests/test_floor_results.py::test_result_has_no_tof_joint]
- AC3: floor suites and lint pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_floor_sweep.py tests/test_floor_results.py tests/test_floor_api.py -q && uv run poe lint]

## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/floor_sweep.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/floor_results.py`
- `nodes/head_rgbd_calibration/tests/test_floor_sweep.py`
- `nodes/head_rgbd_calibration/tests/test_floor_results.py`
- `nodes/head_rgbd_calibration/tests/test_floor_api.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
