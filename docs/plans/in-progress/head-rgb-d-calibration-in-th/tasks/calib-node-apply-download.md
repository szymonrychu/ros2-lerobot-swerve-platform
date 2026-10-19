---
title: Calib node apply download
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "A computed calibration goes live on the robot with one press and survives deploys; the operator can also download it to commit."
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/apply.py, nodes/head_rgbd_calibration/tests/test_apply.py, nodes/head_rgbd_calibration/README.md]
verify: "(cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_apply.py -q && uv run --frozen poe lint)"
depends_on: [calib-node-compute-job]
owns: [rgbd-calib-applied-files]
uses: [rgbd-calib-result-files, rgbd-calib-session-store, rgbd-calib-node-config]
model: ""
estimate: 50
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/head-rgb-d-calibration-in-th--calib-node-apply-download
branch: head-rgb-d-calibration-in-th--calib-node-apply-download
verified_head: ""
run: ""
provides: []
---

# Calib node apply download

## Requirement

Apply a passing result to the robot atomically and package a result for download (design D6).

- `head_rgbd_calibration/apply.py`:
  - `APPLIED_FILES = ("head_camera_calibration.yaml", "tof_calibration.yaml", "head_rgbd_extrinsics.yaml", "tof_joint.json")` and `APPLIED_MARKER = "applied.json"` (the contract the Ansible path unit and the head nodes read).
  - `apply_result(store, sid, rid, now) -> dict`: refuses with `NotApplicable` unless `result.json` has `passed: true`; copies each of `APPLIED_FILES` into `<state_dir>/applied/` through a temporary file and `os.replace`, then writes `applied.json` (`session`, `result`, `applied_at`, `files`) last, also through `os.replace`.
  - `applied_state(store, units, run) -> dict | None`: `applied.json` plus per unit `{name, active_state, restarted}`, where `restarted` means the unit's `ActiveEnterTimestamp` (read with `systemctl show --timestamp=unix -p ActiveEnterTimestamp -p ActiveState <unit>`, unprivileged, through the injectable `run`) is not older than `applied_at`; `--timestamp=unix` values have the form `@<sec>` and the leading `@` is stripped before parsing; None when nothing is applied.
  - `download_zip(store, sid, rid) -> tuple[str, bytes]`: file name `<sid>-<rid>.zip` and a zip holding the three YAMLs, `tof_joint.json`, `head.urdf` and `report.md`; `NotApplicable` when the result has no YAMLs.
- Tests `tests/test_apply.py`: a passing result is applied with every file and `applied.json` written last; a failing result is refused and `applied/` is unchanged; an `os.replace` failure mid-apply leaves the previous `applied.json` untouched; restart flags from a fake `systemctl`; zip member names and content.
- README `## Apply and download` section (doc file).

## Acceptance conditions

- a1: a passing result is applied with applied.json written last [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_apply.py::test_apply_writes_marker_last -q]
- a2: a failing result is refused and applied/ is unchanged [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_apply.py::test_failing_result_refused -q]
- a3: a failure mid-apply keeps the previous marker [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_apply.py::test_interrupted_apply_keeps_previous_marker -q]
- a4: per-unit restart state is reported from systemctl show [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_apply.py::test_units_restarted_after_apply -q]
- a5: the download zip holds the YAMLs, tof_joint.json, head.urdf and report.md [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_apply.py::test_download_zip_members -q]
- a6: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen poe lint]

## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/apply.py`
- `nodes/head_rgbd_calibration/tests/test_apply.py`
- `nodes/head_rgbd_calibration/README.md`

## Interfaces

- `nodes/head_rgbd_calibration/head_rgbd_calibration/apply.py::APPLIED_FILES`, `APPLIED_MARKER`, `apply_result`, `applied_state`, `download_zip`, `NotApplicable`.
- Contract `rgbd-calib-applied-files` (owned here): `<state_dir>/applied/{head_camera_calibration.yaml,tof_calibration.yaml,head_rgbd_extrinsics.yaml,tof_joint.json,applied.json}`; `tof_joint.json` is `{xyz: [3], rpy: [3], note}`.

## Expected outcome

Apply on a passing result swaps the applied files atomically and touches `applied.json` last (the path unit trigger); the node can tell when each head unit restarted.

## Steps

- Write `tests/test_apply.py`; run: red.
- Implement apply, state and zip.
- Run the verify command: all pass, lint clean.

## Notes

- `systemctl show` needs no privilege; never call `systemctl restart` from the node (design D6).
