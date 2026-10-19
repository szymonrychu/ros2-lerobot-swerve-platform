---
title: Calib core retire shims
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The calibration code lives in one place; no transitional module re-exports remain."
files: [scripts/head_rgbd_calibration/charuco.py, scripts/head_rgbd_calibration/intrinsics.py, scripts/head_rgbd_calibration/extrinsics.py, scripts/head_rgbd_calibration/urdf_handoff.py, tests/test_head_rgbd_calib_extrinsics.py, tests/test_head_rgbd_calib_intrinsics.py, tests/test_head_rgbd_urdf_handoff.py, tests/test_head_rgbd_calib_core.py]
verify: "uv run --frozen pytest tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_poses.py tests/test_head_rgbd_calib_retired.py tests/test_head_rgbd_calib_frames.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_extrinsics.py tests/test_head_rgbd_calibrate_cli.py tests/test_head_rgbd_validate.py tests/test_head_rgbd_urdf_handoff.py -q && uv run poe lint"
depends_on: [retire-ssh-live-counter]
owns: []
uses: [rgbd-calib-core]
model: ""
estimate: 30
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Calib core retire shims

## Requirement

Delete the transitional re-export modules left by the core move so the calibration code lives in one place (design D2). Deletion pinned by a retirement test.

- `git rm` `scripts/head_rgbd_calibration/charuco.py`, `scripts/head_rgbd_calibration/intrinsics.py`, `scripts/head_rgbd_calibration/extrinsics.py`, `scripts/head_rgbd_calibration/urdf_handoff.py`.
- `tests/test_head_rgbd_calib_extrinsics.py`, `tests/test_head_rgbd_calib_intrinsics.py`, `tests/test_head_rgbd_urdf_handoff.py` (and any other tracked importer found by `git grep`) import from `ros2_common.head_rgbd_calibration` instead; no assertion changes.
- `tests/test_head_rgbd_calib_core.py` gains `test_moved_modules_left_scripts`: `git ls-files` returns nothing for the four paths, and `git grep` finds no `scripts.head_rgbd_calibration.charuco|intrinsics|extrinsics|urdf_handoff` import in tracked files.
- The whole calibration suite and the CLI tests pass; root `uv run poe lint` passes; `scripts/README.md` module list matches what is left under `scripts/head_rgbd_calibration/` (doc file).

## Acceptance conditions

- a1: the four re-export modules are untracked and nothing imports them [evidence: test:tests/test_head_rgbd_calib_core.py::test_moved_modules_left_scripts]
- a2: the calibration suite passes [evidence: command:uv run --frozen pytest tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_poses.py tests/test_head_rgbd_calib_frames.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_extrinsics.py tests/test_head_rgbd_calibrate_cli.py tests/test_head_rgbd_validate.py tests/test_head_rgbd_urdf_handoff.py tests/test_head_rgbd_calib_retired.py -q]
- a3: root lint passes [evidence: command:uv run poe lint]

## Files

- `scripts/head_rgbd_calibration/charuco.py`
- `scripts/head_rgbd_calibration/intrinsics.py`
- `scripts/head_rgbd_calibration/extrinsics.py`
- `scripts/head_rgbd_calibration/urdf_handoff.py`
- `tests/test_head_rgbd_calib_extrinsics.py`
- `tests/test_head_rgbd_calib_intrinsics.py`
- `tests/test_head_rgbd_urdf_handoff.py`
- `tests/test_head_rgbd_calib_core.py`

## Interfaces

- No new interface; importers use `ros2_common.head_rgbd_calibration.{charuco,intrinsics,extrinsics,urdf_handoff}`.

## Expected outcome

`scripts/head_rgbd_calibration/` holds only CLI-side modules (config, frames, pipeline, validate); no transitional re-export remains.

## Steps

- Add `test_moved_modules_left_scripts`; run: red.
- `git rm` the four re-exports; rewrite the remaining importers.
- Run the verify command: all pass, lint clean.

## Notes

- TDD exception: deletion pinned by a retirement test.
