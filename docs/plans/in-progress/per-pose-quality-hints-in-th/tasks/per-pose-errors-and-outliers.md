---
title: Per-pose errors and outliers
status: in-progress
repo: ros2-lerobot-sverve-platform
value: Compute names the bad pose itself instead of failing with one opaque RMS number.
files: [shared/ros2_common/head_rgbd_calibration/calibrate.py, shared/ros2_common/head_rgbd_calibration/extrinsics.py, tests/test_head_rgbd_calib_quality.py]
verify: "uv run --frozen pytest tests/test_head_rgbd_calib_quality.py tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_extrinsics.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_poses.py tests/test_head_rgbd_calibrate_cli.py -q && uv run poe lint"
depends_on: []
owns: [pose-quality-errors]
uses: []
model: ""
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/per-pose-quality-hints-in-th--per-pose-errors-and-outliers
branch: per-pose-quality-hints-in-th--per-pose-errors-and-outliers
verified_head: ""
run: ""
provides: []
---

# Per-pose errors and outliers

## Requirement

Record per-pose reprojection errors for the RGB intrinsic fit, the ToF intrinsic fit and the stereo fit in the shared calibration core, flag outlier poses, and name them in `failed_limits` and the report (design D2, D3, D4). Hints only: no pose is ever dropped or refitted automatically (D1).

- `extrinsics.py::stereo_calibrate` uses `cv2.stereoCalibrateExtended` (CALIB_FIX_INTRINSIC, same inputs, R and T passed as None or identity/zeros) and `ExtrinsicResult` gains `pair_indices: tuple[int, ...] = ()` (index into `pairs` of every pair actually used, in order; pairs under MIN_COMMON_CORNERS stay skipped) and `per_pair_errors: tuple[float, ...] = ()` (per used pair `sqrt((e_tof**2 + e_rgb**2) / 2)` from perViewErrors). `stereo_rms`, `r`, `t` and `pair_count` are unchanged in value for the same input; `load_extrinsics_yaml` and every existing caller keep working through the defaults.
- `calibrate.py` gains, grouped at the top, `OUTLIER_MEDIAN_FACTOR = 3.0`, a frozen dataclass `PoseErrors(pose_id: str, rgb_px: float | None, tof_px: float | None, stereo_px: float | None, outlier: tuple[str, ...])` (`outlier` is the subset of `("rgb", "tof", "stereo")`, in that order, of the fits in which the pose is flagged; a `None` error means the pose was not in that fit), and `CalibrationRun.poses: tuple[PoseErrors, ...] = ()` (one entry per input view, in view order).
- `calibrate_detections(..., pose_ids: list[str] | None = None)`: `pose_ids` names the views (default `f"{i + 1:03d}"`); a length other than `len(views)` raises ValueError naming both lengths. The RGB and ToF errors come from `IntrinsicsResult.per_view_errors` mapped back to the view index, the stereo error from `pair_indices`.
- Outlier rule, per fit: a pose is flagged when its error is greater than `max(OUTLIER_MEDIAN_FACTOR * median of that fit's per-pose errors, the fit's limit)`; the limits are `limits.rgb_max_rms_px`, `limits.tof_max_rms_px`, `limits.stereo_max_rms_px`. `outlier_pose_ids(run) -> list[str]` returns the flagged pose ids in view order.
- `failed_limits`: a failed RMS limit whose fit has flagged poses gets the suffix ` - outlier pose 007 (66.600 px): delete pose 007 and Compute again` (one pose) or ` - outlier poses 007 (66.600 px), 012 (5.100 px): delete poses 007, 012 and Compute again` (several, view order). A failed limit without flagged poses and the plane-bias message are byte-identical to today.
- `report_markdown`: when `run.poses` is non-empty it appends a `## Per-pose reprojection errors` table (`| Pose | RGB px | ToF px | Stereo px | Flag |`, errors with 3 decimals, `-` for None, Flag `outlier: RGB, stereo` or empty) followed by `Flagged poses (error > max(3 x median, limit)):` and one line per flagged pose `- pose 007: RGB 66.600 px, stereo 40.100 px - delete pose 007 and Compute again`, or the line `Flagged poses: none`. With `run.poses` empty the report is byte-identical to today, so `GOLDEN_REPORT` stays unchanged.
- New `tests/test_head_rgbd_calib_quality.py` covers: a synthetic session (reuse `tests/rgbd_synthetic.py` / the existing synthetic board helpers) with one view whose RGB points are corrupted gets that view flagged in `rgb` (and `stereo` when it crosses the rule) and no other view flagged; a clean synthetic session flags nothing; the rule uses the limit when 3 x median is lower and 3 x median when higher (unit test of the flagging helper with fixed numbers); `pose_ids` default and length mismatch; `stereo_calibrate` `pair_indices` skips an under-6-common-corners pair and `per_pair_errors` reproduces `stereo_rms` as the point-weighted RMS; the `failed_limits` suffix for one and for several flagged poses; the report section with a flagged pose and with none.
- Every existing calibration test passes with no assertion changed; root `uv run poe lint` passes.

## Acceptance conditions

- a1: an RGB-corrupted synthetic view is flagged and no clean view is [evidence: test:tests/test_head_rgbd_calib_quality.py::test_corrupted_view_is_the_only_outlier]
- a2: a clean synthetic session flags no pose [evidence: test:tests/test_head_rgbd_calib_quality.py::test_clean_session_flags_nothing]
- a3: the threshold is max(3 x median, limit) [evidence: test:tests/test_head_rgbd_calib_quality.py::test_outlier_threshold_is_max_of_median_factor_and_limit]
- a4: pose_ids default to 001.. and a length mismatch raises [evidence: test:tests/test_head_rgbd_calib_quality.py::test_pose_ids_default_and_length_mismatch]
- a5: stereo pair_indices skip short pairs and per_pair_errors reproduce stereo_rms [evidence: test:tests/test_head_rgbd_calib_quality.py::test_stereo_per_pair_errors_match_rms]
- a6: failed_limits names the outlier poses with delete and Compute again [evidence: test:tests/test_head_rgbd_calib_quality.py::test_failed_limits_names_outlier_poses]
- a7: report.md lists per-pose errors and the flagged poses [evidence: test:tests/test_head_rgbd_calib_quality.py::test_report_lists_flagged_poses]
- a8: PoseErrors and outlier_pose_ids exist in the core [evidence: symbol:shared/ros2_common/head_rgbd_calibration/calibrate.py::outlier_pose_ids]
- a9: every existing calibration test passes unchanged [evidence: command:uv run --frozen pytest tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_extrinsics.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_poses.py tests/test_head_rgbd_calibrate_cli.py -q]
- a10: root lint passes [evidence: command:uv run poe lint]

## Files

- `shared/ros2_common/head_rgbd_calibration/calibrate.py`
- `shared/ros2_common/head_rgbd_calibration/extrinsics.py`
- `tests/test_head_rgbd_calib_quality.py`

## Interfaces

- `shared/ros2_common/head_rgbd_calibration/calibrate.py`: `OUTLIER_MEDIAN_FACTOR`, `PoseErrors(pose_id, rgb_px, tof_px, stereo_px, outlier)`, `CalibrationRun.poses`, `calibrate_detections(views, rgb_size, tof_size, tof_info, board_cfg, limits, pose_ids=None)`, `outlier_pose_ids(run) -> list[str]`, `failed_limits`, `report_markdown` (part `pose-quality-errors`, used by compute-result-pose-errors).
- `shared/ros2_common/head_rgbd_calibration/extrinsics.py`: `ExtrinsicResult.pair_indices`, `ExtrinsicResult.per_pair_errors` (defaults `()`).

## Expected outcome

`calibrate_detections` on a session with one bad view returns a run whose `poses` flag exactly that view, `failed_limits` says `delete pose NNN and Compute again`, and report.md lists it; the existing golden report and every existing test are unchanged.

## Steps

- Write `tests/test_head_rgbd_calib_quality.py` (the a1-a7 tests); run `uv run --frozen pytest tests/test_head_rgbd_calib_quality.py -q`: red (ImportError on `PoseErrors` / `outlier_pose_ids`).
- Switch `stereo_calibrate` to `cv2.stereoCalibrateExtended` and record `pair_indices` / `per_pair_errors`.
- Add `PoseErrors`, the flagging helper, `CalibrationRun.poses`, `pose_ids`, the `failed_limits` suffix and the report section.
- Run the verify command: all pass, lint clean.

## Notes

- `IntrinsicsResult.per_view_errors` already holds the per-view RMS of `cv2.calibrateCameraExtended`; only the mapping back to the view index is new (the RGB and ToF fits use only views whose detection is not None).
- OpenCV 4.14 signature (probed): `stereoCalibrateExtended(objectPoints, imagePoints1, imagePoints2, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageSize, R, T[, E[, F[, rvecs[, tvecs[, perViewErrors[, flags[, criteria]]]]]]]) -> retval, K1, D1, K2, D2, R, T, E, F, rvecs, tvecs, perViewErrors`; perViewErrors is (N, 2), camera 1 is the ToF.
- The real failure this targets: RGB RMS 8.28 px against 0.5 because pose 007 (6 RGB corners) had 66.6 px while every other pose was <= 0.33 px.
- `OUTLIER_MEDIAN_FACTOR` is a module constant, not a `LimitsConfig` field (design D3): `test_core_config_defaults_match_cli_yaml` pins LimitsConfig to the CLI yaml and every result.json copies the limits.
- Hints only (D1): never drop or refit a view.
