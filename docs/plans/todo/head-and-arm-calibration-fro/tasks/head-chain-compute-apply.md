---
title: Head chain compute apply
status: todo
repo: ros2-lerobot-sverve-platform
value: "A head result is computed on the robot and applied with one click, with the ToF joint kept consistent with the new camera pose."
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py, nodes/head_rgbd_calibration/head_rgbd_calibration/apply.py, nodes/head_rgbd_calibration/tests/test_compute_head_chain.py, nodes/head_rgbd_calibration/tests/test_apply.py]
verify: "cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_compute_head_chain.py tests/test_compute.py tests/test_apply.py -q && uv run poe lint"
depends_on: [session-kinds-joint-poses, head-chain-board-solver]
owns: [calib-head-result-files]
uses: [calib-session-kinds, head-chain-fit, head-chain-override, head-arm-synthetic-scenes]
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

# Head chain compute apply

## Requirement

Compute and apply of a head_chain session (owns `calib-head-result-files`; design D5, D8).

- `compute.py` dispatches on the session kind (rgbd unchanged). For `head_chain`:
  - views from `store.load_floor_views`; intrinsics from `<state_dir>/applied/head_camera_calibration.yaml` (missing: `result.json` with `passed: false`, `error: "no applied head camera calibration"`); its image size must equal the session's `rgb_size` (else an error naming both).
  - Base URDF: `cfg.head_urdf_file` with the currently applied `head_chain.json` applied (`apply_overrides`), so the components the head fit keeps fixed (mount x, y, yaw after an arm result) carry over; `fit_head_chain(views, K, D, base_urdf, cfg.fit)`.
  - Result files in `results/<rid>/`: `head_chain.json` (`head_chain_json`, note with date and RMS), `tof_joint.json` recomputed with `urdf_handoff.tof_joint_origin(read_chain(new_urdf), load_extrinsics_yaml(applied/head_rgbd_extrinsics.yaml))` (no applied extrinsics: not written, the report says the applied tof_joint stays), `head.urdf` (repo URDF with both), `report.md` (`head_report_markdown`), `result.json` `{id, session, kind: "head_chain", created, passed, failed_limits, rms_px, holdout_rms_px, views, spots, params: {name: {value, sigma, urdf}}, limits}`. A failed result keeps only `report.md` and `result.json`, as today.
- `apply.py`: a `head_chain` result (passed only, else NotApplicable) copies `head_chain.json` and, when present, `tof_joint.json` into `applied/` (temp file + `os.replace` each), then writes `applied.json` `{session, result, kind, head_session, applied_at, files}` last (`head_session` = the session). An rgbd apply keeps working and records `kind: "rgbd"` and keeps the previous `head_session`; neither kind deletes the other's applied files. `download_zip` adds `head_chain.json`.
- New `tests/test_compute_head_chain.py` (synthetic session from `pose_factory.still_pose` + `tests/head_arm_synthetic.py`): a passing result with every file; missing intrinsics error; limits exceeded -> only report and result; `tof_joint.json` equals `urdf_handoff.tof_joint_origin` on the new chain; fixed components taken from the applied chain. `tests/test_apply.py` gains: head_chain apply writes the files and `applied.json` last with `kind` and `head_session`; a failing head result refused; an rgbd apply after a head apply keeps `head_chain.json`.
- The existing compute and apply tests pass unchanged.

Steps: tests first (red), implement, verify.

## Acceptance conditions

- a1: a head_chain session computes a passing result with every file [evidence: test:nodes/head_rgbd_calibration/tests/test_compute_head_chain.py::test_head_chain_result_files]
- a2: no applied intrinsics gives an error result [evidence: test:nodes/head_rgbd_calibration/tests/test_compute_head_chain.py::test_missing_intrinsics_error]
- a3: tof_joint is recomputed on the new chain [evidence: test:nodes/head_rgbd_calibration/tests/test_compute_head_chain.py::test_tof_joint_recomputed_on_new_chain]
- a4: applying a head result writes head_chain.json and the marker last [evidence: test:nodes/head_rgbd_calibration/tests/test_apply.py::test_head_chain_apply_writes_marker_last]
- a5: an rgbd apply keeps the applied head chain [evidence: test:nodes/head_rgbd_calibration/tests/test_apply.py::test_rgbd_apply_keeps_head_chain]
- a6: existing compute and apply tests pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_compute.py tests/test_apply.py tests/test_compute_head_chain.py -q]
- a7: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run poe lint]

## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/apply.py`
- `nodes/head_rgbd_calibration/tests/test_compute_head_chain.py`
- `nodes/head_rgbd_calibration/tests/test_apply.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
