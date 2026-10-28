---
title: Retire ToF fits in calibration
status: in-progress
repo: ros2-lerobot-sverve-platform
value: The calibration library is only what the D435 needs.
files: [shared/ros2_common/head_rgbd_calibration/calibrate.py, shared/ros2_common/head_rgbd_calibration/intrinsics.py, shared/ros2_common/head_rgbd_calibration/frames.py, tests/test_head_rgbd_calib_core.py, tests/test_head_rgbd_calib_intrinsics.py, tests/test_head_rgbd_calib_extrinsics.py, tests/test_head_rgbd_calib_retired.py, tests/README.md]
verify: "uv run pytest tests/test_head_rgbd_calib_retired.py tests/test_head_rgbd_intrinsics_check.py tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_intrinsics.py -q && uv run poe lint && cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q"
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

- `calibrate.py`: `FITS`, `detect_tof`, the ToF and stereo fits of `calibrate_detections`, and the tof/stereo branches of `fit_error`/`failed_limits`/`report_markdown` are removed, along with the imports of `extrinsics` and `normalise_amplitude`. What remains is the colour check (`check_intrinsics`) and the helpers it uses.
- `intrinsics.py`: `calibrate_tof` removed.
- `frames.py`: `normalise_amplitude` removed.
- `tests/test_head_rgbd_calib_core.py` and `tests/test_head_rgbd_calib_intrinsics.py` lose the ToF/stereo cases (they test deleted code). `tests/test_head_rgbd_calib_extrinsics.py` is deleted: after this task nothing calls the stereo extrinsics module, and that module is deleted next (retire-tof-stereo-calibratio).
- `tests/test_head_rgbd_calib_retired.py::test_tof_fits_are_gone`: no tracked .py file under shared/ or nodes/ defines `detect_tof`, `calibrate_tof` or `normalise_amplitude`, and nothing outside extrinsics.py/urdf_handoff.py imports `extrinsics`. tests/README.md updated.
TDD exception: deletion pinned by a retirement test.

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

- Add `test_tof_fits_are_gone`; run it; expect a failure listing the definitions.
- Remove the ToF code and its tests; rerun the root calibration tests and the node suite; expect pass.
- Run the verify command; commit `refactor(calibration): remove the ToF and stereo fits` (deletion pinned by a retirement test).

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-27 amended: normalise_amplitude and the extrinsics test leave here; the config fields move to the stereo retirement
- 2026-10-28: shared/ros2_common/head_rgbd_calibration/poses.py imports detect_tof and is unused once capture drops PoseCounter (calibration-config-on-d435): retire poses.py and its tests here too.
