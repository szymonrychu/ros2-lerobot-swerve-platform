---
title: RGBD extrinsic calibration
status: todo
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
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
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

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
