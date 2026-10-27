---
title: Retire ToF stereo calibration maths
status: todo
repo: ros2-lerobot-sverve-platform
value: No ToF-to-RGB stereo code is left to maintain.
files: [shared/ros2_common/head_rgbd_calibration/extrinsics.py, shared/ros2_common/head_rgbd_calibration/urdf_handoff.py, shared/ros2_common/head_rgbd_calibration/chain.py, shared/ros2_common/head_rgbd_calibration/hints.py, shared/ros2_common/head_rgbd_calibration/config.py, tests/test_head_rgbd_urdf_handoff.py, tests/test_head_rgbd_calib_retired.py, tests/README.md]
verify: "uv run pytest tests/test_head_rgbd_calib_retired.py tests/test_head_rgbd_calib_hints.py tests/test_head_chain_override.py tests/test_head_floor_fit.py tests/test_head_arm_fit.py -q && uv run poe lint && cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q"
depends_on: [retire-tof-fits-in-calibrati, arm-offsets-compute-apply]
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

- Delete `shared/ros2_common/head_rgbd_calibration/extrinsics.py` and `urdf_handoff.py`, with `tests/test_head_rgbd_urdf_handoff.py` (its test of extrinsics went in retire-tof-fits-in-calibrati).
- `chain.py` defines `RPY_SEQUENCE` itself (it imported it from urdf_handoff). Any other non-ToF helper still imported from urdf_handoff (for example `read_chain`, if arm-offsets-compute-apply uses it) moves into chain.py unchanged. `hints.py` defines `MIN_COMMON_CORNERS` itself (from extrinsics). The values are unchanged.
- `config.py`: `SelectionConfig` loses `still_tof_mad`, `amplitude_percentiles`, `tof_upscale` and `max_pair_dt_s`. `LimitsConfig` loses `tof_max_rms_px`, `stereo_max_rms_px`, `tof_zero_distortion_rms_px` and `rgb_max_rms_px` (replaced by the intrinsics limits). Nothing reads them after this task.
- `tests/test_head_rgbd_calib_retired.py::test_tof_stereo_modules_are_gone` (git ls-files of both modules is empty; no tracked file imports them; the removed config fields are rejected). tests/README.md updated (shared/README.md is updated by node-readmes-and-diagrams-on).
TDD exception: deletion pinned by a retirement test.

## Acceptance conditions

- AC1: extrinsics.py and urdf_handoff.py are no longer tracked or imported, and the ToF config fields are rejected [evidence: test:tests/test_head_rgbd_calib_retired.py::test_tof_stereo_modules_are_gone]
- AC2: chain and hints keep their constants and the fits still pass [evidence: command:uv run pytest tests/test_head_rgbd_calib_hints.py tests/test_head_chain_override.py tests/test_head_floor_fit.py tests/test_head_arm_fit.py tests/test_arm_offsets_fit.py -q]
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

- Add the retirement test; run it; expect a failure.
- Inline the two constants, `git rm` the modules and their tests; rerun the listed tests and the node suite; expect pass.
- Run the verify command; commit `refactor(calibration): remove the ToF stereo extrinsics and URDF hand-off` (deletion pinned by a retirement test).

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-27 amended: takes the shared ToF config fields; the extrinsics test left in the previous task
