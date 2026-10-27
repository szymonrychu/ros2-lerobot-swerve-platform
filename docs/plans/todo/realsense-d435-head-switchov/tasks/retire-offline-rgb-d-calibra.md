---
title: Retire offline RGB-D calibration CLI
status: todo
repo: ros2-lerobot-sverve-platform
value: "The ToF+CM3 bag calibration CLI no longer has to be kept working."
files: [scripts/head_rgbd_calibrate.py, scripts/head_rgbd_calibrate.yaml, scripts/head_rgbd_calibration/, tests/test_head_rgbd_calibrate_cli.py, tests/test_head_rgbd_validate.py, tests/test_head_rgbd_calib_frames.py, tests/test_head_rgbd_calib_retired.py, tests/README.md]
verify: "uv run pytest tests/test_head_rgbd_calib_retired.py tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_intrinsics.py tests/test_web_ui_agent_tab.py -q && uv run poe lint"
depends_on: [calibration-tests-off-the-cl]
model: sonnet
review: ""
verified_head: ""
points: 2
expected_minutes: 23
expected_tokens: 1500000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Retire offline RGB-D calibration CLI

## Requirement

Goal G9: "... ToF-only code paths ... removed ...". Design D10: "the ToF+CM3 offline CLI (scripts/head_rgbd_calibrate.py) [is] deleted" (the web_ui calibration screen drives D435 recalibration; the CLI reads ToF amplitude and the CM3 mapping stream over bags).

- Delete `scripts/head_rgbd_calibrate.py`, `scripts/head_rgbd_calibrate.yaml` and `scripts/head_rgbd_calibration/` (config, frames, pipeline, validate, __init__).
- Delete `tests/test_head_rgbd_calibrate_cli.py` and `tests/test_head_rgbd_validate.py` (they test only the CLI). In `tests/test_head_rgbd_calib_frames.py`, delete the tests of `scripts.head_rgbd_calibration.frames`/`config`; tests of `ros2_common.head_rgbd_calibration.frames` stay. If none remain, delete the file.
- `tests/test_head_rgbd_calib_retired.py` gains `test_offline_cli_is_gone` (git ls-files of the three paths is empty; no tracked file outside docs/ names `head_rgbd_calibrate.py`).
- tests/README.md updated. TDD exception: deletion pinned by a retirement test.

## Acceptance conditions

- AC1: the offline CLI and its package are no longer tracked or referenced [evidence: test:tests/test_head_rgbd_calib_retired.py::test_offline_cli_is_gone]
- AC2: the remaining calibration tests pass [evidence: command:uv run pytest tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_extrinsics.py tests/test_head_rgbd_calib_retired.py -q]
- AC3: root lint (vulture included) passes [evidence: command:uv run poe lint]

## Files

- `scripts/head_rgbd_calibrate.py`
- `scripts/head_rgbd_calibrate.yaml`
- `scripts/head_rgbd_calibration/`
- `tests/test_head_rgbd_calibrate_cli.py`
- `tests/test_head_rgbd_validate.py`
- `tests/test_head_rgbd_calib_frames.py`
- `tests/test_head_rgbd_calib_retired.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
