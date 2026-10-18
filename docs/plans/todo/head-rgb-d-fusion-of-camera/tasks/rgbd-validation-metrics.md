---
title: RGBD validation metrics
status: todo
repo: ros2-lerobot-sverve-platform
value: "Depth accuracy, alignment and time offset are computed from recorded data with one tool, so the methodology's targets are measured, not eyeballed."
files: [scripts/head_rgbd_calibration/validate.py, tests/test_head_rgbd_validate.py]
verify: "uv run pytest tests/test_head_rgbd_validate.py -q && uv run poe lint && uv run ruff check scripts/head_rgbd_calibration && uv run ruff format --check scripts/head_rgbd_calibration"
depends_on: [rgbd-extrinsic-calibration]
owns: [head-rgbd-validate]
uses: [head-rgbd-calib-config, rgbd-synthetic-scenes, head-rgbd-charuco, head-rgbd-extrinsics-file]
model: sonnet
estimate: 75
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# RGBD validation metrics

## Requirement

- `scripts/head_rgbd_calibration/validate.py` (D11, methodology "Validation metrics and targets"):
  - `depth_accuracy(depth_frames: list[np.ndarray], reference_m: float, roi_px: int) -> DepthStats`: central ROI over the frames, invalid (0) pixels excluded, bias and RMSE in mm, valid fraction; `registration_bias(raw_frames, registered_frames, roi) -> float` (median registered minus median raw over the matching ROI, mm).
  - `edge_alignment(registered_depth: np.ndarray, rgb: np.ndarray, board, rgb_camera, depth_scale: int) -> AlignmentStats`: the board outline from RGB ChArUco detection + `solvePnP` + the board's outer rectangle projected into the output image, against depth-step pixels of the registered depth (a valid pixel with a neighbour farther by > 0.3 m) near that outline; median and p90 distance in output pixels.
  - `valid_ratio(registered_depth, fov_mask) -> float`.
  - `time_offset(rgb_tracks, tof_tracks, extrinsic, rgb_camera, tof_camera) -> OffsetEstimate`: from a pan sweep, ToF ChArUco corners (with their ToF depth) projected into the RGB image with the extrinsic, compared with the RGB corner track of the same id: offset = (u_projected - u_rgb) / (du_rgb/dt) per matched pair, the median in ms and its spread; pairs with |du/dt| below a minimum speed are ignored.
  - `check_targets(results) -> list[TargetResult]`: each methodology target (values from the design table) with measured value and pass/fail.
- Tests first in `tests/test_head_rgbd_validate.py`, seen failing: a synthetic flat target at 1.0 m with +12 mm bias and 8 mm noise gives bias 12 +- 2 mm and RMSE within 2 mm of the truth; zeros are excluded; a synthetic registered scene (board in front of a wall) with the depth shifted 2 px right gives median alignment 2 +- 0.3 px and 0 +- 0.3 px unshifted; synthetic sweep tracks with a true 25 ms offset give 25 +- 3 ms, with 0 ms give 0 +- 3 ms; slow-motion pairs are ignored; `check_targets` flags a value over its limit.

## Acceptance conditions

- a1: depth bias, RMSE and registration bias are measured in a central ROI excluding invalid pixels [evidence: test: tests/test_head_rgbd_validate.py]
- a2: edge alignment recovers a known synthetic pixel shift [evidence: test: tests/test_head_rgbd_validate.py]
- a3: the ToF-RGB time offset is recovered from synthetic pan-sweep corner tracks [evidence: test: tests/test_head_rgbd_validate.py]
- a4: results are checked against the methodology targets [evidence: test: tests/test_head_rgbd_validate.py]

## Files

- `scripts/head_rgbd_calibration/validate.py`
- `tests/test_head_rgbd_validate.py`

## Interfaces

- `scripts/head_rgbd_calibration/validate.py:depth_accuracy`, `registration_bias`, `edge_alignment`, `valid_ratio`, `time_offset`, `check_targets` (part `head-rgbd-validate`).

## Expected outcome

Recorded bags turn into the methodology's numbers with pass/fail against the targets.

## Steps

- Write the tests with synthetic scenes from tests/rgbd_synthetic.py; run `uv run pytest tests/test_head_rgbd_validate.py -q`; see them fail.
- Implement validate.py; rerun until green; `uv run poe lint && uv run ruff check scripts/head_rgbd_calibration && uv run ruff format --check scripts/head_rgbd_calibration`.
- Commit `feat(scripts): RGB-D validation metrics`.

## Notes

- Validation uses OpenCV projections of a few corner points; it does not reimplement the node's z-buffer.
- Pan sweep for the time offset: head pan only, +-20 deg at about 20 deg/s, inside the pan/tilt epic's verified range (servo safety: least dramatic joint).
