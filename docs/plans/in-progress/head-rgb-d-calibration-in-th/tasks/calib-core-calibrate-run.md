---
title: Calib core calibrate run
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The calibration runs from stored poses and their detections, so the robot never decodes a bag; the CLI gives identical results."
files: [shared/ros2_common/head_rgbd_calibration/calibrate.py, scripts/head_rgbd_calibration/pipeline.py, scripts/head_rgbd_calibration/validate.py, scripts/head_rgbd_calibrate.py, tests/rgbd_synthetic.py, tests/test_head_rgbd_calibrate_cli.py, tests/test_head_rgbd_validate.py, tests/test_head_rgbd_calib_core.py]
verify: "uv run --frozen pytest tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_live.py tests/test_head_rgbd_calib_frames.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_extrinsics.py tests/test_head_rgbd_calibrate_cli.py tests/test_head_rgbd_validate.py tests/test_head_rgbd_urdf_handoff.py -q && uv run poe lint"
depends_on: [calib-core-extrinsics-urdf]
owns: []
uses: [rgbd-calib-core]
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

# Calib core calibrate run

## Requirement

Move the calibration run into the core so it can run from stored poses with their detections, never from a bag; the CLI gives identical results (design D2, D5).

- `shared/ros2_common/head_rgbd_calibration/calibrate.py`: `detect_tof(amplitude, board, sel)` and `CalibrationRun` moved from `scripts/head_rgbd_calibration/pipeline.py`; `PoseView` (frozen dataclass: `rgb: Detection | None`, `tof: Detection | None`, `depth: np.ndarray | None`); `calibrate_detections(views, rgb_size, tof_size, tof_info, board_cfg, limits) -> CalibrationRun`, the fit half of today's `run_calibration` (RGB `calibrate` on views with an RGB detection, `calibrate_tof` on views with a ToF detection, `stereo_calibrate` on views with both, `plane_depth_bias` on those with depth, `compare_sdk`; `pair_count = len(views)`; `ValueError` when `tof_info` is None, same message as today); `failed_limits(run, limits: LimitsConfig)` and `report_markdown(run, limits: LimitsConfig, session, lens, date)` producing byte-identical text for the same inputs.
- `scripts/head_rgbd_calibration/pipeline.py`: `run_calibration(frames, cfg)` becomes `still_pairs` + `detect`/`detect_tof` per pair + core `calibrate_detections`; `failed_limits`/`report_markdown` callers (`write_calibration`, `scripts/head_rgbd_calibrate.py`) pass `cfg.limits`.
- `scripts/head_rgbd_calibration/pipeline.py`, `scripts/head_rgbd_calibration/validate.py`, `scripts/head_rgbd_calibrate.py`, `tests/rgbd_synthetic.py`, `tests/test_head_rgbd_calibrate_cli.py`, `tests/test_head_rgbd_validate.py` import moved names from the core directly, not through the transitional re-exports.
- `tests/test_head_rgbd_calib_core.py` gains: `calibrate_detections` on the views built from a synthetic FrameSet equals `run_calibration` on that FrameSet (RMS values, view counts, K, extrinsic `t` within 1e-9); `failed_limits` names each exceeded `limits.*` key; `report_markdown` of a fixed run equals a golden string.
- Every existing calibration test passes with no assertion changed; root `uv run poe lint` passes; `scripts/README.md` notes that the maths lives in `ros2_common.head_rgbd_calibration` and the CLI wraps it.

## Acceptance conditions

- a1: calibrate_detections matches run_calibration on the same frames [evidence: test:tests/test_head_rgbd_calib_core.py::test_calibrate_detections_matches_run_calibration]
- a2: failed_limits names each exceeded limits key [evidence: test:tests/test_head_rgbd_calib_core.py::test_failed_limits_names_limit_keys]
- a3: the report text is unchanged [evidence: test:tests/test_head_rgbd_calib_core.py::test_report_markdown_golden]
- a4: the CLI and validation tests pass unchanged [evidence: command:uv run --frozen pytest tests/test_head_rgbd_calibrate_cli.py tests/test_head_rgbd_validate.py tests/test_head_rgbd_calib_frames.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_extrinsics.py tests/test_head_rgbd_calib_live.py tests/test_head_rgbd_urdf_handoff.py -q]
- a5: root lint passes [evidence: command:uv run poe lint]
- a6: calibrate_detections exists in the core [evidence: symbol:shared/ros2_common/head_rgbd_calibration/calibrate.py::calibrate_detections]

## Files

- `shared/ros2_common/head_rgbd_calibration/calibrate.py`
- `scripts/head_rgbd_calibration/pipeline.py`
- `scripts/head_rgbd_calibration/validate.py`
- `scripts/head_rgbd_calibrate.py`
- `tests/rgbd_synthetic.py`
- `tests/test_head_rgbd_calibrate_cli.py`
- `tests/test_head_rgbd_validate.py`
- `tests/test_head_rgbd_calib_core.py`

## Interfaces

- `shared/ros2_common/head_rgbd_calibration/calibrate.py::calibrate_detections(views: list[PoseView], rgb_size: tuple[int, int], tof_size: tuple[int, int], tof_info: CameraInfoData | None, board_cfg: BoardConfig, limits: LimitsConfig) -> CalibrationRun`.
- `detect_tof(amplitude, board, sel) -> Detection | None`, `failed_limits(run, limits) -> list[str]`, `report_markdown(run, limits, session, lens, date) -> str`, `PoseView`, `CalibrationRun`.

## Expected outcome

The node can calibrate from stored detections; `scripts/head_rgbd_calibrate.py calibrate` writes the same files and report as before for the same bag.

## Steps

- Add the parity, limits and golden-report tests; run: red.
- Move `detect_tof`, `CalibrationRun`, `failed_limits`, `report_markdown`; extract `calibrate_detections` from `run_calibration`; switch the listed importers to the core.
- Run the verify command: all pass, lint clean.

## Notes

- `run_calibration` keeps its signature for the CLI; only the fit half moves.
