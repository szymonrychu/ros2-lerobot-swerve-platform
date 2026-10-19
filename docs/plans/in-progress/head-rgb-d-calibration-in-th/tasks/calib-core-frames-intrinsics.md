---
title: Calib core frames intrinsics
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "Still-pose selection, flicker-proof stillness and the intrinsic solver become importable from the robot node."
files: [shared/ros2_common/head_rgbd_calibration/frames.py, shared/ros2_common/head_rgbd_calibration/intrinsics.py, scripts/head_rgbd_calibration/frames.py, scripts/head_rgbd_calibration/intrinsics.py, tests/test_head_rgbd_calib_core.py]
verify: "uv run --frozen pytest tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_live.py tests/test_head_rgbd_calib_frames.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_extrinsics.py tests/test_head_rgbd_calibrate_cli.py tests/test_head_rgbd_validate.py tests/test_head_rgbd_urdf_handoff.py -q && uv run poe lint"
depends_on: [calib-core-config-charuco]
owns: []
uses: [rgbd-calib-core]
model: ""
estimate: 40
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/head-rgb-d-calibration-in-th--calib-core-frames-intrinsics
branch: head-rgb-d-calibration-in-th--calib-core-frames-intrinsics
verified_head: ""
run: ""
provides: []
---

# Calib core frames intrinsics

## Requirement

Move the frame selection and the intrinsic solver into the core; maths and behaviour unchanged (design D2).

- `shared/ros2_common/head_rgbd_calibration/frames.py`: `NS_PER_S`, `MAX_UINT8`, `RGB_DIFF_BLUR_SIGMA`, `StampedImage`, `CameraInfoData`, `FrameSet`, `FramePair`, `normalise_amplitude`, `mean_abs_diff`, `rgb_difference`, `tof_diff_fraction`, `nearest_index`, `still_pairs` moved unchanged from `scripts/head_rgbd_calibration/frames.py`; no rosbags import.
- `scripts/head_rgbd_calibration/frames.py` keeps `TopicNames`, `TYPESTORE`, `ROS_CLI_ENV`, `BAG_STORAGE`, `STEP_NAMES`, `BAG_METADATA`, `capture_commands`, `bag_message_count`, `capture_bag`, `stamp_ns`, `decode_mono16`, `decode_compressed`, `decode_camera_info`, `read_frames`, and imports the moved names from the core (its importers keep resolving `scripts.head_rgbd_calibration.frames.still_pairs` and the dataclasses).
- `shared/ros2_common/head_rgbd_calibration/intrinsics.py`: the whole module moved with `git mv`, importing from the core `charuco`, `config`, `frames`. `scripts/head_rgbd_calibration/intrinsics.py` is re-created as a transitional re-export of its public names, deleted by calib-core-retire-shims.
- `tests/test_head_rgbd_calib_core.py` gains: core `still_pairs` selects one pair per still window of a synthetic FrameSet; core `rgb_difference` of two frames differing only by a per-row gain (mains flicker bands) is below 1 grey level; core intrinsics `calibrate` recovers the synthetic camera matrix within the tolerance the existing intrinsics test uses.
- Every existing calibration test passes with no assertion changed; root `uv run poe lint` passes; `shared/README.md` section lists the new modules.

## Acceptance conditions

- a1: core still_pairs selects one pair per still window [evidence: test:tests/test_head_rgbd_calib_core.py::test_core_still_pairs_one_pair_per_window]
- a2: core rgb_difference ignores mains flicker bands [evidence: test:tests/test_head_rgbd_calib_core.py::test_core_rgb_difference_ignores_flicker_bands]
- a3: core intrinsics recover the synthetic camera [evidence: test:tests/test_head_rgbd_calib_core.py::test_core_intrinsics_recover_synthetic_camera]
- a4: every existing calibration test passes unchanged [evidence: command:uv run --frozen pytest tests/test_head_rgbd_calib_frames.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_extrinsics.py tests/test_head_rgbd_calibrate_cli.py tests/test_head_rgbd_calib_live.py tests/test_head_rgbd_validate.py tests/test_head_rgbd_urdf_handoff.py -q]
- a5: root lint passes [evidence: command:uv run poe lint]
- a6: still_pairs lives in the core frames module [evidence: symbol:shared/ros2_common/head_rgbd_calibration/frames.py::still_pairs]

## Files

- `shared/ros2_common/head_rgbd_calibration/frames.py`
- `shared/ros2_common/head_rgbd_calibration/intrinsics.py`
- `scripts/head_rgbd_calibration/frames.py`
- `scripts/head_rgbd_calibration/intrinsics.py`
- `tests/test_head_rgbd_calib_core.py`

## Interfaces

- `shared/ros2_common/head_rgbd_calibration/frames.py::still_pairs(frames, sel) -> list[FramePair]`, `rgb_difference`, `tof_diff_fraction`, `normalise_amplitude`, `nearest_index`, dataclasses `StampedImage`, `CameraInfoData`, `FrameSet`, `FramePair`.
- `shared/ros2_common/head_rgbd_calibration/intrinsics.py::calibrate`, `calibrate_tof`, `compare_sdk`, `write_camera_calibration_yaml`, `IntrinsicsResult`.

## Expected outcome

The pure frame functions and the intrinsic solver import from the core without rosbags; the CLI still reads bags through `scripts/head_rgbd_calibration/frames.py`.

## Steps

- Add the three new tests to `tests/test_head_rgbd_calib_core.py`; run them: red on the missing core modules.
- Move the pure part of frames.py and `git mv` intrinsics.py; re-create the transitional intrinsics re-export.
- Run the verify command: all pass, lint clean.

## Notes

- `decode_*` and `read_frames` stay in scripts (they use the rosbags typestore messages).
- Do not change any threshold or formula (design non-goal).
