---
title: Calibration library D435 frames
status: todo
repo: ros2-lerobot-sverve-platform
value: The floor and head-from-arm fits calibrate the D435 mount through the pan/tilt chain.
files: [shared/ros2_common/head_rgbd_calibration/floor_fit.py, shared/ros2_common/head_rgbd_calibration/head_arm_fit.py, tests/head_floor_synthetic.py, tests/head_arm_synthetic.py, tests/test_head_floor_fit.py, tests/test_head_arm_fit.py, tests/test_head_arm_calib_chain.py]
verify: "uv run pytest tests/test_head_floor_fit.py tests/test_head_arm_fit.py tests/test_head_arm_calib_chain.py tests/test_head_arm_real_session.py -q && uv run poe lint"
depends_on: [head-chain-overrides-on-d435]
model: opus
review: ""
verified_head: ""
points: 5
expected_minutes: 57
expected_tokens: 3750000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Calibration library D435 frames

## Requirement

Goal G8. Design D10: "Head floor fit runs on /head/d435/points in d435_depth_optical_frame. Head-from-arm runs on the D435 colour." Reuse-brief fix-as-ac: "ToF names hardcoded in shared calibration code ... AC: sensor joint/frame and topics come from config; ToF paths not called for D435".

- `floor_fit.py`: `TOF_FRAME` replaced by a `sensor_frame: str` argument (default `d435_depth_optical_frame`) on `fit_head_chain_floor` and the plane prediction. Docstrings say the depth sensor frame.
- `head_arm_fit.py`: `CAMERA_FRAME` replaced by a `camera_frame: str` argument (default `d435_color_optical_frame`) on `fit_head_from_arm`, `camera_pose`, `aim_head`, `board_pnp` callers. Docstrings name the D435 colour camera. The free head parameters include `d435_joint` instead of `head_camera_joint`.
- `tests/head_floor_synthetic.py` and `tests/head_arm_synthetic.py` build scenes on the D435 frames of head.urdf. `tests/test_head_floor_fit.py`, `tests/test_head_arm_fit.py` and `tests/test_head_arm_calib_chain.py` use them. The ToF/head_camera frame assertions are no longer accurate; the recovery tolerances are unchanged. New tests `test_head_floor_fit.py::test_recovers_the_chain_from_d435_depth_planes` and `test_head_arm_fit.py::test_recovers_the_d435_mount_from_board_views`.

## Acceptance conditions

- AC1: the floor fit recovers the head chain from planes in d435_depth_optical_frame [evidence: test:tests/test_head_floor_fit.py::test_recovers_the_chain_from_d435_depth_planes]
- AC2: the head-from-arm fit recovers the d435_joint mount from board views in the D435 colour camera [evidence: test:tests/test_head_arm_fit.py::test_recovers_the_d435_mount_from_board_views]
- AC3: no ToF or head_camera frame constant remains in the two fit modules [evidence: command:! grep -n "tof_optical_frame\|head_camera_optical_frame" shared/ros2_common/head_rgbd_calibration/floor_fit.py shared/ros2_common/head_rgbd_calibration/head_arm_fit.py]
- AC4: the calibration chain tests and lint pass [evidence: command:uv run pytest tests/test_head_arm_calib_chain.py tests/test_head_arm_real_session.py -q && uv run poe lint]

## Files

- `shared/ros2_common/head_rgbd_calibration/floor_fit.py`
- `shared/ros2_common/head_rgbd_calibration/head_arm_fit.py`
- `tests/head_floor_synthetic.py`
- `tests/head_arm_synthetic.py`
- `tests/test_head_floor_fit.py`
- `tests/test_head_arm_fit.py`
- `tests/test_head_arm_calib_chain.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
