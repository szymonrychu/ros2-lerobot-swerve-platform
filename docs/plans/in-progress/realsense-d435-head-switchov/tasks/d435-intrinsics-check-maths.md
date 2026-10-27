---
title: D435 intrinsics check maths
status: done
repo: ros2-lerobot-sverve-platform
value: The D435 factory calibration is checked against a ChArUco fit before anything relies on it.
files: [shared/ros2_common/head_rgbd_calibration/calibrate.py, shared/ros2_common/head_rgbd_calibration/config.py, tests/test_head_rgbd_intrinsics_check.py, tests/README.md]
verify: "uv run pytest tests/test_head_rgbd_intrinsics_check.py tests/test_head_rgbd_calib_core.py -q && uv run poe lint"
depends_on: []
model: sonnet
review: ""
verified_head: f3ca9899
points: 3
expected_minutes: 34
expected_tokens: 2250000
points_actual: 3
actual_minutes: 2
actual_tokens: 458704
actual_roles: "impl 2, review 1"
actual_role_tokens: "impl 363052, review 95652"
---

# D435 intrinsics check maths

## Requirement

Goal G8: "... intrinsics check against factory ...". Design D10: "fits the colour intrinsics, and compares them with the factory camera_info (intrinsics.compare_sdk). It also measures the board-plane depth bias from the aligned depth. The report passes or fails against limits". Reuse: intrinsics.calibrate/compare_sdk, charuco detect.

Additive (the ToF fits are removed later by retire-tof-fits-in-calibrati):
- `calibrate.py`: `check_intrinsics(detections: list[Detection], board, factory: CameraInfoData, limits: LimitsConfig) -> IntrinsicsCheck`. It fits the colour intrinsics with `intrinsics.calibrate` and compares them with `compare_sdk`. `IntrinsicsCheck` is frozen and holds: fitted K/D, rms_px, focal_diff_pct, principal_diff_px, per-view reprojection errors, failed_limits and passed. Also `board_plane_bias_mm(depth_mm: np.ndarray, corners_px, board_pose, intrinsics) -> tuple[float, float] | None`. It returns the median and spread of (aligned depth - PnP board plane depth) over board pixels, and None when fewer than 200 valid board pixels exist.
- `config.py` `SelectionConfig` gains `still_depth_mad` (0.01, mean absolute change between consecutive aligned depth frames as a fraction of the median depth); `LimitsConfig` gains `intrinsics_max_rms_px` (0.5), `intrinsics_max_focal_diff_pct` (1.0), `intrinsics_max_principal_diff_px` (5.0); `plane_max_bias_mm` stays (its D435 value is set by calibration-config-on-d435).
- `report_markdown`-style `intrinsics_report_markdown(check, bias, session, date) -> str`.
- New `tests/test_head_rgbd_intrinsics_check.py` on synthetic views (charuco synthetic board as in tests/test_head_rgbd_calib_core.py): factory K equal to truth passes; a factory K 3 % off in focal length fails `intrinsics_max_focal_diff_pct`; a plane offset of 15 mm in synthetic aligned depth gives bias 15 +- 1 mm; too few board pixels -> None. tests/README.md documents the file.

## Acceptance conditions

- AC1: factory intrinsics equal to the truth pass the check [evidence: test:tests/test_head_rgbd_intrinsics_check.py::test_factory_matching_truth_passes]
- AC2: a factory focal length 3 % off fails the focal limit [evidence: test:tests/test_head_rgbd_intrinsics_check.py::test_focal_offset_fails_the_limit]
- AC3: the board-plane depth bias is measured from aligned depth, None without enough pixels [evidence: test:tests/test_head_rgbd_intrinsics_check.py::test_board_plane_bias]
- AC4: root lint passes [evidence: command:uv run poe lint]

## Files

- `shared/ros2_common/head_rgbd_calibration/calibrate.py`
- `shared/ros2_common/head_rgbd_calibration/config.py`
- `tests/test_head_rgbd_intrinsics_check.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Write `tests/test_head_rgbd_intrinsics_check.py`; run it; expect failures (functions missing).
- Implement check_intrinsics, board_plane_bias_mm, the report and the limits; rerun; expect pass.
- Run the verify command; commit `feat(calibration): D435 intrinsics check against factory`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-27 amended: still_depth_mad belongs to the shared SelectionConfig

- 2026-10-27 done
