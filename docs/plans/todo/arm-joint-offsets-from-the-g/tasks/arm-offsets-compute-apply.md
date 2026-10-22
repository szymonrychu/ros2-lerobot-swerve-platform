---
title: Arm offsets compute apply
status: todo
repo: ros2-lerobot-sverve-platform
value: The arm result gives a reviewable joint_offsets_rad snippet.
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py, nodes/head_rgbd_calibration/head_rgbd_calibration/apply.py, nodes/head_rgbd_calibration/head_rgbd_calibration/jobs.py, nodes/head_rgbd_calibration/tests/test_compute_arm.py, nodes/head_rgbd_calibration/tests/test_apply.py, nodes/head_rgbd_calibration/tests/test_jobs.py]
verify: "cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_compute_arm.py tests/test_apply.py tests/test_jobs.py -q && uv run poe lint"
depends_on: [arm-offsets-plate-solver]
owns: []
uses: [arm-offsets-fit]
model: ""
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Arm offsets compute apply

## Requirement

Moved verbatim on 2026-10-22 from head-and-arm-calibration-fro (user decision: the arm calibration is trusted; the head is calibrated against the arm there). Before running, re-plan this task with `ticket.py amend` against the parts that epic delivers (head_arm sessions and still capture, the sequencer and pose check, the head-from-arm fit and its synthetic scenes, the head-arm HTTP API); task slugs and design decisions named below refer to the old head-and-arm plan in its git history.

Compute and apply of an arm session: the offsets snippet for review, the head chain applied (design D5, D8).

- `jobs.py`: `ComputeJob.start(sid, floor_session: str | None = None)` passes `--floor-session <sid>` to the child; `compute.py main` accepts it.
- `compute.py` for kind `arm`: the floor session is `--floor-session`, else `applied.json` `head_session`; none: `result.json` `passed: false`, `error: "apply a head calibration first"`. Views from `load_floor_views(floor)` and `load_plate_views(sid)`; intrinsics as for head_chain; `fit_head_arm(..., current_offsets=cfg.arm_sequence.joint_offsets_rad)`.
  - Result files: `head_chain.json` (all three joints, x/y/yaw measured), `tof_joint.json` (recomputed as for head_chain), `head.urdf`, `joint_offsets_snippet.yaml` (`offsets_snippet_yaml`), `report.md` (`arm_report_markdown`), `result.json` `{id, session, kind: "arm", floor_session, created, passed, failed_limits, rms_px, holdout_rms_px, holdout_tool_mm, wrist_roll_fitted, offsets: {joint: {value, sigma, current}}, head: {name: {value, sigma, urdf}}, limits}`.
- `apply.py` for kind `arm`: applies `head_chain.json` and `tof_joint.json` only (the snippet never goes to `applied/`), `applied.json` with `kind: "arm"` and `head_session` = the floor session. `download_zip` adds `joint_offsets_snippet.yaml`.
- New `tests/test_compute_arm.py` (synthetic floor + plate sessions): passing result with every file and the snippet; no head applied and no floor session -> error; `--floor-session` wins over `head_session`; failed limits -> only report and result. `tests/test_apply.py`: arm apply writes the chain, never the snippet, `head_session` = floor session; zip has the snippet. `tests/test_jobs.py`: the child command carries `--floor-session`.

## Acceptance conditions

- a1: an arm session computes a result with the snippet and the full head chain [evidence: test:nodes/head_rgbd_calibration/tests/test_compute_arm.py::test_arm_result_files_and_snippet]
- a2: without an applied head chain or floor session the result is an error [evidence: test:nodes/head_rgbd_calibration/tests/test_compute_arm.py::test_arm_compute_needs_head_calibration]
- a3: applying an arm result applies the head chain and never the snippet [evidence: test:nodes/head_rgbd_calibration/tests/test_apply.py::test_arm_apply_writes_chain_not_snippet]
- a4: the compute child receives the floor session [evidence: test:nodes/head_rgbd_calibration/tests/test_jobs.py::test_child_gets_floor_session]
- a5: compute, apply and jobs tests pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_compute.py tests/test_compute_head_chain.py tests/test_compute_arm.py tests/test_apply.py tests/test_jobs.py -q]
- a6: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run poe lint]

## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/apply.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/jobs.py`
- `nodes/head_rgbd_calibration/tests/test_compute_arm.py`
- `nodes/head_rgbd_calibration/tests/test_apply.py`
- `nodes/head_rgbd_calibration/tests/test_jobs.py`

## Interfaces

- `ComputeJob.start(sid, floor_session=None)`, child arg `--floor-session`; result files add `joint_offsets_snippet.yaml`; `result.json` kind `arm`.

## Expected outcome

The arm result gives a reviewable joint_offsets_rad snippet and applies the measured head yaw and position.

## Steps

- Tests first (red), implement, verify.
- Run `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_compute_arm.py tests/test_compute_head_chain.py tests/test_apply.py tests/test_jobs.py -q && uv run poe lint`; expect every test passed and lint clean.

## Notes

- Design: the epic design.md (`ticket.py path head-and-arm-calibration-fro design`) (decisions D1-D12, review focus).

- 2026-10-22 moved from head-and-arm-calibration-fro; dependencies on that epic dropped (it is this epic's blocker).
