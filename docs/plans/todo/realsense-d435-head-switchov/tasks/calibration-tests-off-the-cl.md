---
title: Calibration tests off the CLI config
status: todo
repo: ros2-lerobot-sverve-platform
value: The offline CLI can be deleted without breaking the shared calibration tests.
files: [tests/test_head_rgbd_calib_core.py, tests/test_head_rgbd_calib_intrinsics.py, tests/test_head_rgbd_calib_extrinsics.py, tests/test_head_rgbd_calibrate_cli.py, tests/README.md]
verify: "uv run pytest tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_extrinsics.py tests/test_head_rgbd_calibrate_cli.py tests/test_web_ui_agent_tab.py -q && uv run poe lint"
depends_on: []
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

# Calibration tests off the CLI config

## Requirement

Goal G9: "... ToF-only code paths ... removed ...". Design D10: "the ToF+CM3 offline CLI (scripts/head_rgbd_calibrate.py) [is] deleted". This task is the preparation: tests of shared library code must not import the CLI package, so the CLI can be deleted without touching them.

- `tests/test_head_rgbd_calib_core.py`: tests that run the CLI pipeline (`run_calibration`, `DEFAULT_CONFIG`, `load_calibration_config`, `test_core_config_defaults_match_cli_yaml`) move unchanged into `tests/test_head_rgbd_calibrate_cli.py` (still accurate there; they leave with the CLI). The remaining core tests import config classes from `ros2_common.head_rgbd_calibration.config`.
- `tests/test_head_rgbd_calib_intrinsics.py` and `tests/test_head_rgbd_calib_extrinsics.py` import `BoardConfig`, `LimitsConfig`, `SelectionConfig` from `ros2_common.head_rgbd_calibration.config` and `CameraInfoData`, `normalise_amplitude` from `ros2_common.head_rgbd_calibration.frames` instead of `scripts.head_rgbd_calibration.*`. Assertions are unchanged.
- tests/README.md moves the documented test names accordingly.
No behaviour change (test refactor): the moved and re-imported tests pass before and after.

## Acceptance conditions

- AC1: no shared-library test imports the CLI package [evidence: command:! grep -n "scripts.head_rgbd_calibration" tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_extrinsics.py]
- AC2: the moved CLI pipeline tests pass in the CLI test file [evidence: command:uv run pytest tests/test_head_rgbd_calibrate_cli.py -q]
- AC3: the shared calibration tests pass [evidence: command:uv run pytest tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_extrinsics.py tests/test_web_ui_agent_tab.py -q]

## Files

- `tests/test_head_rgbd_calib_core.py`
- `tests/test_head_rgbd_calib_intrinsics.py`
- `tests/test_head_rgbd_calib_extrinsics.py`
- `tests/test_head_rgbd_calibrate_cli.py`
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
