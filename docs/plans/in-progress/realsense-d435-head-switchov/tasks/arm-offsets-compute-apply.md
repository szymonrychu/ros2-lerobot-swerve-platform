---
title: Arm offsets compute apply
status: in-progress
repo: ros2-lerobot-sverve-platform
value: The arm result gives a reviewable joint_offsets_rad / tool_offset_m snippet.
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py, nodes/head_rgbd_calibration/head_rgbd_calibration/apply.py, nodes/head_rgbd_calibration/head_rgbd_calibration/jobs.py, nodes/head_rgbd_calibration/tests/test_compute_arm.py, nodes/head_rgbd_calibration/tests/test_apply.py, nodes/head_rgbd_calibration/tests/test_jobs.py]
verify: "cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint"
depends_on: [arm-offsets-plate-solver, d435-intrinsics-check-comput, head-arm-calibration-on-d435]
model: opus
review: ""
verified_head: ""
points: 5
expected_minutes: 57
expected_tokens: 3750000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Arm offsets compute apply

## Requirement

Goal G6. User decision 3. Design D9. Re-plan of `arm-joint-offsets-from-the-g/arm-offsets-compute-apply` for the D435 (no tof_joint; kind `arm`).

- `jobs.py`: `ComputeJob.start(sid, floor_session: str | None = None)` passes `--floor-session <sid>` to the child; `compute.py main` accepts it.
- `compute.py` for kind `arm`: the floor session is `--floor-session`, else `applied.json` `head_session`; none -> `result.json` `passed: false`, `error: "apply a head calibration first"`. Floor views come from the floor session and plate views from the arm session. Intrinsics are the factory D435 colour CameraInfo recorded with the stills. It calls `fit_head_arm(..., current_offsets=cfg.arm_sequence.joint_offsets_rad)`.
  - Result files: `head_chain.json` (all head joints incl. d435_joint, x/y/yaw measured), `head.urdf`, `joint_offsets_snippet.yaml` (`offsets_snippet_yaml`, with tool_offset_m when fitted), `report.md`, `result.json` `{id, session, kind: "arm", floor_session, created, passed, failed_limits, rms_px, holdout_rms_px, holdout_tool_mm, wrist_roll_fitted, tool_fitted, tool_offset_m, offsets: {joint: {value, sigma, current}}, head: {name: {value, sigma, urdf}}, limits}`.
- `apply.py` for kind `arm`: applies `head_chain.json` only. The snippet never goes to `applied/`; offsets are committed by hand after review. `applied.json` gets `kind: "arm"` and `head_session` = the floor session. `download_zip` adds `joint_offsets_snippet.yaml`.
- Tests: new `tests/test_compute_arm.py` (synthetic floor + plate sessions): a passing result with every file and the snippet; no head applied and no floor session -> error; `--floor-session` wins over `head_session`; failed limits -> only report and result. `tests/test_apply.py`: arm apply writes the chain and never the snippet, `head_session` = floor session, and the zip has the snippet. `tests/test_jobs.py`: the child command carries `--floor-session`.

## Acceptance conditions

- AC1: an arm session computes a result with the snippet and the full head chain [evidence: test:nodes/head_rgbd_calibration/tests/test_compute_arm.py::test_arm_result_files_and_snippet]
- AC2: without an applied head chain or floor session the result is an error [evidence: test:nodes/head_rgbd_calibration/tests/test_compute_arm.py::test_arm_compute_needs_head_calibration]
- AC3: applying an arm result applies the head chain and never the snippet [evidence: test:nodes/head_rgbd_calibration/tests/test_apply.py::test_arm_apply_writes_chain_not_snippet]
- AC4: the compute child receives the floor session [evidence: test:nodes/head_rgbd_calibration/tests/test_jobs.py::test_child_gets_floor_session]
- AC5: node suite and lint pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint]

## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/apply.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/jobs.py`
- `nodes/head_rgbd_calibration/tests/test_compute_arm.py`
- `nodes/head_rgbd_calibration/tests/test_apply.py`
- `nodes/head_rgbd_calibration/tests/test_jobs.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Write `tests/test_compute_arm.py` and the apply/jobs tests; run them; expect failures.
- Implement jobs, compute and apply for kind arm; rerun the node suite; expect pass.
- Run the verify command; commit `feat(head_rgbd_calibration): arm offsets compute and apply`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
- 2026-10-28: arm-offsets-plate-solver ships plate_mount, wrist_roll_offset_rad and tool_in_board_m as null (fit refuses to run until set). Fill wrist_roll_offset_rad from the wrist_roll servo neutral recorded on the robot (feetech_servos neutral recording) and tool_in_board_m from the user-measured board-to-jaw-tip offset (gate Q2) before computing; stop and show offsets to the user before commit/deploy (gate Q6).
- 2026-10-28 (user): the board is the same 3D-printed 6x5 ChArUco (same dimensions as before), 4 mm thick; it is in front of the robot and the D435 head is mounted. Account for the 4 mm thickness where the board surface vs mount face matters (plate_mount / tool_in_board_m, board-plane depth bias).
