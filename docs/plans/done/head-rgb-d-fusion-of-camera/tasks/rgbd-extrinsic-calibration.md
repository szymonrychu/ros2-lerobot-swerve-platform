---
title: RGBD extrinsic calibration
status: done
repo: ros2-lerobot-sverve-platform
value: "The exact position of the ToF relative to the RGB camera is measured, not assumed from the drawing, so depth lands on the right pixels."
files: [scripts/head_rgbd_calibration/extrinsics.py, tests/test_head_rgbd_calib_extrinsics.py]
verify: "uv run pytest tests/test_head_rgbd_calib_extrinsics.py -q && uv run poe lint && uv run ruff check scripts/head_rgbd_calibration && uv run ruff format --check scripts/head_rgbd_calibration"
depends_on: [rgbd-intrinsics-calibration]
owns: [head-rgbd-extrinsics-file]
uses: [head-rgbd-calib-config, rgbd-synthetic-scenes, head-rgbd-charuco]
model: sonnet
estimate: 60
actual: ""
review: ""
branch: head-rgb-d-fusion-of-camera--rgbd-extrinsic-calibration
verified_head: 3d569641ba77c81a636eaa988318db3b43ef3534
run: head-rgb-d-fusion-of-camera
provides: [head-rgbd-extrinsics-file]
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/head-rgb-d-fusion-of-camera--rgbd-extrinsic-calibration
acs_for_audit: [a1, a2]
process_gaps: []
---

# RGBD extrinsic calibration

## Requirement

- `scripts/head_rgbd_calibration/extrinsics.py`:
  - `stereo_calibrate(pairs: list[tuple[Detection, Detection]], board, rgb: IntrinsicsResult, tof: IntrinsicsResult) -> ExtrinsicResult`: object/image points restricted to corner ids seen in both images of a pair, `cv2.stereoCalibrate(..., flags=cv2.CALIB_FIX_INTRINSIC)`; returns R, t of `tof_optical_frame` expressed in `head_camera_optical_frame` (the transform that maps ToF points into the RGB frame), stereo RMS, pair count; refuses fewer than `limits.min_views` pairs.
  - `plane_depth_bias(pairs_with_depth, board, rgb, extrinsic) -> PlaneCheck`: per pair, the board plane from RGB `cv2.solvePnP`, transformed into the ToF frame and intersected with the ToF rays inside the board polygon, against the measured ToF depth there; median and per-pair bias in mm (methodology: plane depth bias <= 15 mm).
  - `write_extrinsics_yaml(path, result, plane_check, meta)` and `load_extrinsics_yaml(path) -> ExtrinsicResult`: keys `parent_frame: head_camera_optical_frame`, `child_frame: tof_optical_frame`, `translation_m: [x, y, z]`, `rotation_xyzw: [...]`, `stereo_rms_px`, `pairs`, `plane_bias_mm_median`, `lens_position`, `board`, `date`, `rgb_rms_px`, `tof_rms_px`, `tof_sdk_diff_pct`; writing refuses (raises, writes nothing) when stereo RMS > `limits.stereo_max_rms_px`.
- Tests first in `tests/test_head_rgbd_calib_extrinsics.py`, seen failing: 20 synthetic pairs from `tests/rgbd_synthetic.py` with the true extrinsic recover the translation within 2 mm and the rotation within 0.2 deg (direction checked: a ToF point mapped with the result lands on the RGB projection of the same board corner within 0.5 px); a pair with no common corner ids is skipped; fewer than min_views pairs raises; with synthetic ToF depth biased by +20 mm the plane check reports 20 +- 3 mm, unbiased 0 +- 3 mm; the YAML round-trips; RMS over the limit writes nothing.

## Acceptance conditions

- a1: synthetic pairs recover the known ToF-to-RGB extrinsic within 2 mm and 0.2 deg, in the right direction [evidence: test: tests/test_head_rgbd_calib_extrinsics.py]
- a2: the plane depth check reports a known synthetic depth bias [evidence: test: tests/test_head_rgbd_calib_extrinsics.py]
- a3: the extrinsics file round-trips and is refused above the stereo RMS limit [evidence: test: tests/test_head_rgbd_calib_extrinsics.py]

## Files

- `scripts/head_rgbd_calibration/extrinsics.py`
- `tests/test_head_rgbd_calib_extrinsics.py`

## Interfaces

- `scripts/head_rgbd_calibration/extrinsics.py:stereo_calibrate`, `plane_depth_bias`, `write_extrinsics_yaml`, `load_extrinsics_yaml`, `ExtrinsicResult` and the extrinsics YAML schema (part `head-rgbd-extrinsics-file`).

## Expected outcome

A set of still pairs produces `head_rgbd_extrinsics.yaml` with the measured transform and its quality figures.

## Steps

- Write the tests; run `uv run pytest tests/test_head_rgbd_calib_extrinsics.py -q`; see them fail.
- Implement extrinsics.py; rerun until green; `uv run poe lint && uv run ruff check scripts/head_rgbd_calibration && uv run ruff format --check scripts/head_rgbd_calibration`.
- Commit `feat(scripts): ToF-to-RGB extrinsic calibration`.

## Notes

- OpenCV's stereoCalibrate returns R, T mapping camera-1 points into camera-2; pass the ToF as camera 1 and the RGB as camera 2 so the result is ToF -> RGB, and test the direction explicitly.
- The node never reads this file (design D4); the URDF hand-off does.

## Ruling

Accepted by the orchestrator: verified and merged into main (root 1507 passed); head-rgbd-package-config verified by hand: scripts/cpp-node-test.sh head_rgbd_cpp PASS after freeing the Docker VM disk (the apt signature failure was a full Docker disk).
