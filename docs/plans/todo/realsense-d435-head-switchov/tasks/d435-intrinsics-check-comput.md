---
title: D435 intrinsics check compute
status: todo
repo: ros2-lerobot-sverve-platform
value: The calibration screen can prove the D435 factory intrinsics on this unit.
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py, nodes/head_rgbd_calibration/head_rgbd_calibration/apply.py, nodes/head_rgbd_calibration/tests/fixtures/api/, nodes/head_rgbd_calibration/tests/test_compute.py, nodes/head_rgbd_calibration/tests/test_apply.py, nodes/head_rgbd_calibration/tests/test_app.py]
verify: "cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint"
depends_on: [calibration-config-on-d435, d435-intrinsics-check-maths]
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

# D435 intrinsics check compute

## Requirement

Goal G8: "... intrinsics check against factory; the web_ui calibration screen drives it." Design D10: "The RGB-D (RGB + ToF stereo) session becomes a 'D435 intrinsics check' ... Apply writes no intrinsics (the factory values stay in use)."

- `compute.py` for the RGB-D kind (renamed kind `intrinsics`): it loads the session's colour stills, detects ChArUco and calls `check_intrinsics` with the factory CameraInfo recorded at capture. It measures `board_plane_bias_mm` per still from the aligned depth. It writes `result.json` (`kind: "intrinsics"`, rms_px, focal_diff_pct, principal_diff_px, plane bias median/spread, per-pose errors, failed_limits, passed) and `report.md`. It no longer writes head_camera_calibration.yaml, tof_calibration.yaml, head_rgbd_extrinsics.yaml, tof_joint.json or head.urdf. Its imports of extrinsics/urdf_handoff/write_camera_calibration_yaml are dropped.
- `apply.py` for kind `intrinsics`: records the passed check in `applied.json` (kind, session, result id, date) and copies no file into applied/ (factory intrinsics stay authoritative). A failed result cannot be applied (error).
- `tests/fixtures/api/` (`result.json`, `apply.json`, `applied.json`, `state.json`, `session.json`, `capture_*.json`) carry the new shapes (contract fixtures shared with the web_ui frontend tests through contract.py).
- Tests: `tests/test_compute.py` and `tests/test_apply.py`. The RGB/ToF/stereo cases are no longer accurate and are replaced by intrinsics-check cases: pass, focal fail, missing factory CameraInfo -> error result, apply of a failed result refused, apply copies nothing.

## Acceptance conditions

- AC1: an intrinsics session computes a pass/fail result with the plane bias and writes no calibration file [evidence: test:nodes/head_rgbd_calibration/tests/test_compute.py::test_intrinsics_session_result]
- AC2: a session without a factory CameraInfo gives an error result [evidence: test:nodes/head_rgbd_calibration/tests/test_compute.py::test_missing_factory_camera_info_is_an_error]
- AC3: apply records a passed check, copies nothing and refuses a failed one [evidence: test:nodes/head_rgbd_calibration/tests/test_apply.py::test_intrinsics_apply_records_only]
- AC4: the API contract fixtures match the service [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_app.py -q]
- AC5: node suite and lint pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint]

## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/apply.py`
- `nodes/head_rgbd_calibration/tests/fixtures/api/`
- `nodes/head_rgbd_calibration/tests/test_compute.py`
- `nodes/head_rgbd_calibration/tests/test_apply.py`
- `nodes/head_rgbd_calibration/tests/test_app.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Write the new compute/apply tests and update the fixtures; run `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_compute.py tests/test_apply.py tests/test_app.py -q`; expect failures.
- Rework compute and apply; rerun the node suite; expect pass.
- Run the verify command; commit `feat(head_rgbd_calibration): D435 intrinsics check session`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
