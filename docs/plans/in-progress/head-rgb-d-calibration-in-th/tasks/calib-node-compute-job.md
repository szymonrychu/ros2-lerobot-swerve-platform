---
title: Calib node compute job
status: done
repo: ros2-lerobot-sverve-platform
value: "The calibration is computed on the robot from stored poses in a niced child process, without blocking the UI or the capture."
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py, nodes/head_rgbd_calibration/head_rgbd_calibration/jobs.py, nodes/head_rgbd_calibration/tests/test_compute.py, nodes/head_rgbd_calibration/tests/test_jobs.py, nodes/head_rgbd_calibration/README.md]
verify: "(cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_compute.py tests/test_jobs.py -q && uv run --frozen poe lint)"
depends_on: [calib-node-session-store]
owns: [rgbd-calib-result-files]
uses: [rgbd-calib-session-store, rgbd-calib-node-config, rgbd-calib-core]
model: ""
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/head-rgb-d-calibration-in-th--calib-node-compute-job
branch: head-rgb-d-calibration-in-th--calib-node-compute-job
verified_head: 9f608847e8b258c46cca89d99f6dfde543e3da85
run: head-rgb-d-calibration-in-th
provides: [rgbd-calib-result-files]
acs_for_audit: []
process_gaps: ["No red/green tails for the TDD proof (tests.red, verification.evidence). Both production files are new, so the red state is an ImportError at collection."]
---

# Calib node compute job

## Requirement

Compute the calibration on the robot from a session's stored poses in a niced child process, one job at a time (design D5).

- `head_rgbd_calibration/compute.py`:
  - `compute_result(store, sid, rid, cfg, now) -> dict`: `store.load_views`, core `calibrate_detections`, `failed_limits(run, cfg.limits)`, `report_markdown(run, cfg.limits, sid, lens, date)`; writes `results/<rid>/` under a temporary name then renames it: `head_camera_calibration.yaml` and `tof_calibration.yaml` (core `write_camera_calibration_yaml`, camera names `head_camera` and `tof` as the CLI writes), `head_rgbd_extrinsics.yaml` (core `write_extrinsics_yaml` with the same meta as `scripts/head_rgbd_calibration/pipeline.py:write_calibration`), `tof_joint.json` (`{xyz, rpy, note}` from core `read_chain` + `tof_joint_origin`, fed the URDF text read from `cfg.head_urdf_file` before `urdf_handoff.read_chain`), `head.urdf` (core `rewrite_tof_joint` of that file), `report.md`, `result.json` (`id`, `session`, `created`, `passed`, `failed_limits`, `rgb_rms_px`, `tof_rms_px`, `stereo_rms_px`, `plane_bias_mm`, `rgb_views`, `tof_views`, `stereo_pairs`, `lens_position`, `pose_ids`).
  - A run over the limits is written with `passed: false` (its report stays visible). A core `ValueError` (too few views, no ToF camera_info) writes only `result.json` with `passed: false` and `error`; there is no `report.md`, so `GET result` returns `report: null` (the frontend tolerates null).
  - `main(argv)` / `python -m head_rgbd_calibration.compute --config <path> --session <sid> --result <rid>` runs one computation and exits 0 when `result.json` was written.
- `head_rgbd_calibration/jobs.py` `ComputeJob(cfg, store, popen, clock, config_path)` (`config_path` is the node config path taken from the `HEAD_RGBD_CALIBRATION_CONFIG` env var the node reads): `start(sid) -> rid` spawns the child with `--config <config_path>` using `subprocess.Popen` at niceness `cfg.compute_nice`; `Busy` while one runs; `state() -> {running, session_id, result_id, started_at, error}`; kills the child after `compute_timeout_s` (error `timed out after N s`); a non-zero exit without `result.json` reports the last 20 stderr lines as `error`.
- Tests `tests/test_compute.py` (synthetic session written through the store: passing result and files; limits exceeded gives `passed: false`; too few poses gives `error` and no YAMLs; `tof_joint.json` and `head.urdf` agree) and `tests/test_jobs.py` (fake child commands: busy, timeout, failure tail).
- README `## Compute` section (doc file).

## Acceptance conditions

- a1: a passing session writes every result file [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_compute.py::test_passing_session_writes_all_result_files -q]
- a2: a run over the limits is kept with passed false [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_compute.py::test_limits_exceeded_marks_not_passed -q]
- a3: too few poses report an error and write no YAMLs [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_compute.py::test_too_few_poses_reports_error -q]
- a4: tof_joint.json and the rewritten head.urdf agree [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_compute.py::test_tof_joint_matches_rewritten_urdf -q]
- a5: one job at a time, killed after the timeout, failures report the stderr tail [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_jobs.py -q]
- a6: the child runs niced [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_jobs.py::test_child_is_niced -q]
- a7: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen poe lint]

## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/jobs.py`
- `nodes/head_rgbd_calibration/tests/test_compute.py`
- `nodes/head_rgbd_calibration/tests/test_jobs.py`
- `nodes/head_rgbd_calibration/README.md`

## Interfaces

- `nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py::compute_result(store, sid, rid, cfg, now) -> dict`, `main(argv) -> int`.
- `nodes/head_rgbd_calibration/head_rgbd_calibration/jobs.py::ComputeJob`, `.start(sid) -> str`, `.state() -> dict`, `.stop()`.
- Contract `rgbd-calib-result-files` (owned here): the `results/<rid>/` file set of design D3 and the `result.json` keys.

## Expected outcome

A compute on a stored session produces a result directory in one rename; the HTTP thread only starts and polls a child process.

## Steps

- Write `tests/test_compute.py` and `tests/test_jobs.py`; run: red.
- Implement compute and the job runner.
- Run the verify command: all pass, lint clean.

## Notes

- Memory: load only the session's poses (about 6 MB for 30); never a bag.
- The deployed checkout URDF path comes from `cfg.head_urdf_file`; the node never writes into the checkout.

- 2026-10-19 run head-rgb-d-calibration-in-th: dependency-failed on calib-node-session-store
