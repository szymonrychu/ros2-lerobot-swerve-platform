---
title: Retire ToF stereo calibration maths
status: todo
repo: ros2-lerobot-sverve-platform
value: No ToF-to-RGB stereo code is left to maintain.
files: [shared/ros2_common/head_rgbd_calibration/extrinsics.py, shared/ros2_common/head_rgbd_calibration/urdf_handoff.py, shared/ros2_common/head_rgbd_calibration/chain.py, shared/ros2_common/head_rgbd_calibration/hints.py, tests/test_head_rgbd_calib_extrinsics.py, tests/test_head_rgbd_urdf_handoff.py, tests/test_head_rgbd_calib_retired.py, tests/README.md]
verify: "uv run pytest tests/test_head_rgbd_calib_retired.py tests/test_head_rgbd_calib_hints.py tests/test_head_chain_override.py tests/test_head_floor_fit.py tests/test_head_arm_fit.py tests/test_web_ui_agent_tab.py -q && uv run poe lint && cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q"
depends_on: [retire-tof-fits-in-calibrati]
model: opus
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

# Retire ToF stereo calibration maths

## Requirement

Goal G9. Design D10: "ToF-only maths (extrinsics.py stereo, urdf_handoff.py ...) ... deleted". Do-not-build: "ToF-to-RGB stereo extrinsic calibration - D435 factory extrinsics".

- Delete `shared/ros2_common/head_rgbd_calibration/extrinsics.py` and `urdf_handoff.py`, with `tests/test_head_rgbd_calib_extrinsics.py` and `tests/test_head_rgbd_urdf_handoff.py` (they test only that code).
- `chain.py` defines `RPY_SEQUENCE` itself (it imported it from urdf_handoff). `hints.py` defines `MIN_COMMON_CORNERS` itself (from extrinsics). Same values.
- `tests/test_head_rgbd_calib_retired.py::test_tof_stereo_modules_are_gone` (git ls-files of both modules is empty; no tracked file imports them). tests/README.md and shared/README.md updated.
TDD exception: deletion pinned by a retirement test.

## Acceptance conditions

- AC1: extrinsics.py and urdf_handoff.py are no longer tracked or imported [evidence: test:tests/test_head_rgbd_calib_retired.py::test_tof_stereo_modules_are_gone]
- AC2: chain and hints keep their constants and tests [evidence: command:uv run pytest tests/test_head_rgbd_calib_hints.py tests/test_head_chain_override.py tests/test_head_floor_fit.py tests/test_head_arm_fit.py -q]
- AC3: calibration node suite and root lint pass [evidence: command:uv run poe lint && cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q]

## Files

- `shared/ros2_common/head_rgbd_calibration/extrinsics.py`
- `shared/ros2_common/head_rgbd_calibration/urdf_handoff.py`
- `shared/ros2_common/head_rgbd_calibration/chain.py`
- `shared/ros2_common/head_rgbd_calibration/hints.py`
- `tests/test_head_rgbd_calib_extrinsics.py`
- `tests/test_head_rgbd_urdf_handoff.py`
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
