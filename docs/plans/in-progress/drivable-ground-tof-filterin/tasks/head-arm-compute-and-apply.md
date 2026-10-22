---
title: Head arm compute and apply
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "A captured session becomes a checked head chain result that is applied to every consumer through the floor fit's one apply path, or refused."
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/head_arm_results.py, nodes/head_rgbd_calibration/head_rgbd_calibration/floor_results.py, nodes/head_rgbd_calibration/tests/test_head_arm_results.py, nodes/head_rgbd_calibration/tests/test_floor_results.py]
verify: "cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_head_arm_results.py tests/test_floor_results.py tests/test_floor_api.py tests/test_apply.py -q && uv run poe lint"
depends_on: [head-from-arm-board-solver, joint-state-still-capture]
owns: [calib-head-arm-results]
uses: [head-arm-fit, head-arm-synthetic-scenes, calib-session-kinds, head-chain-override, head-chain-applied-record, head-rgbd-applied-calibration, rgbd-result-pose-errors]
model: ""
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/drivable-ground-tof-filterin--head-arm-compute-and-apply
branch: drivable-ground-tof-filterin--head-arm-compute-and-apply
verified_head: ""
run: ""
provides: []
---

# Head arm compute and apply

## Requirement

Compute and apply of a `head_arm` session, reusing the floor fit's result and apply machinery (owns
`calib-head-arm-results`; design D31, D34, D36; approvals H1, H3, H4).

- New `head_rgbd_calibration/head_arm_results.py`:
  - `compute_head_arm(cfg: NodeConfig, store: SessionStore, sid: str, now: datetime) -> dict`: views from
    `store.load_arm_board_views(sid)`; intrinsics from `<state_dir>/applied/head_camera_calibration.yaml` (missing:
    `result.json` with `passed: false`, `error: "no applied head camera calibration"`), its image size equal to the
    session's (else an error naming both); head URDF `floor_results.base_urdf(cfg)` (the applied chain), arm URDF
    `cfg.arm_urdf_file`; `floor` = the `params` (`floor_fit/<sid>/results/<rid>/result.json`, read with
    `floor_results.read_result`) of the floor result named by the `applied/applied.json` `head_chain` record: its
    `floor` entry (below), or, for a record the current `apply_floor` wrote (`{session, result, applied_at}`, no
    `floor` key; this is the robot's state after the 2026-10-22 apply of floor-20261022-064337 /
    r-20261022-065211) whose `session` matches `FLOOR_SESSION_ID_RE`, that record's own session and result; None when
    neither exists; `fit_head_from_arm(views, K, D, head_urdf, arm_urdf, cfg.fit, floor)`.
  - Result files in the store's `results/<rid>/` of the session: `result.json` `{id, session, kind: "head_arm",
    created, passed, failed_limits, error, rms_px, holdout_rms_px, holdout_board_mm, views, arm_poses, pan_span_deg,
    tilt_span_deg, max_correlation, params: {name: {value, sigma, start, floor, floor_delta}}, board_mount, per_view,
    limits, floor: {session, result} | null}`, `report.md` (`head_arm_report_markdown`) and, only when passed,
    `head_chain.json` (`head_chain_json` of the applied origins merged with the fitted `head_mount_joint` and
    `head_tilt`; note with the date, RMS and held-out board error). The camera joint is fixed, so `tof_joint.json` is
    left as applied.
  - `apply_head_arm(cfg, store, sid, rid, now, run) -> dict`: a passed result only (else NotApplicable), through the
    writer below.
- `floor_results.py`: extract the body of `apply_floor` into one writer `apply_head_chain_file(cfg, source: Path,
  record: dict, now, run) -> dict` (copy `head_chain.json` into `applied/` by temp file + `os.replace`, write the
  `head_chain` record into `applied.json`, return the unit states) used by both `apply_floor` and `apply_head_arm`. The
  `head_chain` record (contract `head-chain-applied-record`) gains `source` (`"floor"` or `"head_arm"`) and `floor`
  (`{session, result}` of the floor result the chain was checked against: a floor apply names itself, a head_arm apply
  carries the record's current `floor` forward); existing keys unchanged. `applied_head_chain` returns the new keys.
  The floor routes and their fixtures keep working.
- New `tests/test_head_arm_results.py` (a `head_arm` session built with `pose_factory.still_arm_pose` and detections
  from `tests/head_arm_synthetic.py::arm_board_scene`, loaded by path): a passing result with every file; missing
  intrinsics error; a failed limit writes only `report.md` and `result.json`; `floor` read from the applied record's
  `floor` entry and, for a legacy `{session, result, applied_at}` floor record, from that record itself;
  no floor record -> `floor_cross_check` failed; the fitted joints merged over the applied chain; apply of a passed
  result writes `applied/head_chain.json` and the record with `source: "head_arm"` and the carried `floor`; a failed
  result refused. `tests/test_floor_results.py` gains: a floor apply records `source: "floor"` and itself as `floor`.
- The existing floor, apply and API tests pass unchanged.

## Acceptance conditions

- a1: a head_arm session computes a passing result with its files [evidence: test:nodes/head_rgbd_calibration/tests/test_head_arm_results.py::test_compute_writes_result_report_and_chain]
- a2: a failed limit writes no head_chain.json and cannot be applied [evidence: test:nodes/head_rgbd_calibration/tests/test_head_arm_results.py::test_failed_result_not_applicable]
- a3: the floor cross-check reads the floor result named by applied/applied.json (its floor entry, or a legacy floor record written by the current apply_floor) [evidence: test:nodes/head_rgbd_calibration/tests/test_head_arm_results.py::test_floor_values_from_applied_record]
- a8: a legacy floor record without the floor key is used as the floor reference [evidence: test:nodes/head_rgbd_calibration/tests/test_head_arm_results.py::test_legacy_floor_record_is_the_reference]
- a4: apply goes through the shared writer and records the source [evidence: test:nodes/head_rgbd_calibration/tests/test_head_arm_results.py::test_apply_writes_applied_chain_and_record]
- a5: a floor apply records itself as source and floor [evidence: test:nodes/head_rgbd_calibration/tests/test_floor_results.py::test_floor_apply_records_source]
- a6: floor, apply and API tests pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_floor_results.py tests/test_floor_api.py tests/test_apply.py tests/test_app.py -q]
- a7: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run poe lint]

## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/head_arm_results.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/floor_results.py`
- `nodes/head_rgbd_calibration/tests/test_head_arm_results.py`
- `nodes/head_rgbd_calibration/tests/test_floor_results.py`

## Interfaces

- `head_arm_results.py::compute_head_arm`, `apply_head_arm`, `read_head_arm_result`, `result.json` keys above (part
  `calib-head-arm-results`).
- `floor_results.py::apply_head_chain_file`; the `applied.json` `head_chain` record `{session, result, applied_at,
  source, floor}`.

## Expected outcome

A captured session becomes a checked head chain result that is applied to every consumer through the floor fit's one
apply path, or refused.

## Steps

- Write `tests/test_head_arm_results.py` and the floor record test first; run them: red.
- Extract the writer, implement `head_arm_results.py`; run
  `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_head_arm_results.py tests/test_floor_results.py tests/test_floor_api.py tests/test_apply.py -q && uv run poe lint`;
  expect every test passed and lint clean.

## Notes

- Compute runs inline in the HTTP handler like `compute_floor` (my call); bound `least_squares` with `max_nfev` if a
  synthetic session of the shipped size takes over 10 s.
- Design: `ticket.py path drivable-ground-tof-filterin design` (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).
