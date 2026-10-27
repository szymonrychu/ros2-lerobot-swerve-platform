---
title: Retire ToF fits in calibration
status: todo
repo: ros2-lerobot-sverve-platform
value: The calibration library is only what the D435 needs.
files: [shared/ros2_common/head_rgbd_calibration/calibrate.py, shared/ros2_common/head_rgbd_calibration/intrinsics.py, shared/ros2_common/head_rgbd_calibration/config.py, shared/ros2_common/head_rgbd_calibration/frames.py, tests/test_head_rgbd_calib_core.py, tests/test_head_rgbd_calib_intrinsics.py, tests/test_head_rgbd_calib_retired.py, tests/README.md]
verify: "uv run pytest tests/test_head_rgbd_calib_retired.py tests/test_head_rgbd_intrinsics_check.py tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_intrinsics.py tests/test_web_ui_agent_tab.py -q && uv run poe lint && cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q"
depends_on: [d435-intrinsics-check-comput, retire-offline-rgb-d-calibra, floor-sweep-on-d435-depth, head-arm-calibration-on-d435]
model: opus
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

# Retire ToF fits in calibration

## Requirement

Goal G9: "... ToF-only code paths ... removed ...". Design D10: "ToF-only maths (... calibrate_tof, detect_tof) ... deleted".

- `calibrate.py`: `FITS`, `detect_tof`, the ToF and stereo fits of `calibrate_detections`, `fit_error`/`failed_limits`/`report_markdown` for tof/stereo are removed, and so is the import of extrinsics. What remains is the colour check (`check_intrinsics`) and the helpers it uses.
- `intrinsics.py`: `calibrate_tof` removed.
- `config.py`: `SelectionConfig` loses `still_tof_mad`, `amplitude_percentiles`, `tof_upscale`, `max_pair_dt_s`; `LimitsConfig` loses `tof_max_rms_px`, `stereo_max_rms_px`, `tof_zero_distortion_rms_px`, `rgb_max_rms_px` (replaced by the intrinsics limits).
- `frames.py`: `normalise_amplitude` removed.
- Tests: `tests/test_head_rgbd_calib_core.py` and `tests/test_head_rgbd_calib_intrinsics.py` lose the ToF/stereo cases (they test deleted code). `tests/test_head_rgbd_calib_retired.py::test_tof_fits_are_gone` asserts that no tracked .py file under shared/ or nodes/ defines `detect_tof`, `calibrate_tof` or `normalise_amplitude`.

## Acceptance conditions

- AC1: no ToF fit, detector or amplitude helper remains defined [evidence: test:tests/test_head_rgbd_calib_retired.py::test_tof_fits_are_gone]
- AC2: the colour check and remaining calibration tests pass [evidence: command:uv run pytest tests/test_head_rgbd_intrinsics_check.py tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_intrinsics.py -q]
- AC3: the calibration node suite still passes [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q]

## Files

- `shared/ros2_common/head_rgbd_calibration/calibrate.py`
- `shared/ros2_common/head_rgbd_calibration/intrinsics.py`
- `shared/ros2_common/head_rgbd_calibration/config.py`
- `shared/ros2_common/head_rgbd_calibration/frames.py`
- `tests/test_head_rgbd_calib_core.py`
- `tests/test_head_rgbd_calib_intrinsics.py`
- `tests/test_head_rgbd_calib_retired.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
