---
title: Compute result pose errors
status: todo
repo: ros2-lerobot-sverve-platform
value: result.json and report.md tell which pose to delete before computing again.
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py, nodes/head_rgbd_calibration/tests/test_compute.py, nodes/head_rgbd_calibration/tests/fixtures/api/result.json]
verify: "cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_compute.py tests/test_app.py -q && uv run poe lint"
depends_on: [per-pose-errors-and-outliers]
owns: [rgbd-result-pose-errors]
uses: [pose-quality-errors]
model: ""
estimate: 40
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Compute result pose errors

## Requirement

Carry the per-pose errors and the outlier poses of a Compute into result.json and report.md (design D4, D6).

- `nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py::compute_result` passes `pose_ids=views.pose_ids` to `calibrate_detections`, so report.md and `failed_limits` name the stored pose ids (e.g. `007`).
- result.json gains `poses: [{id, rgb_error_px, tof_error_px, stereo_error_px, outlier}]` (one entry per stored pose in pose id order; errors float or null when the pose was not in that fit; `outlier` the list of flagged fits from `PoseErrors.outlier`) and `outlier_poses: [id, ...]` (from `outlier_pose_ids`). Both are in `base`, empty lists, so an input-error result (too few views, no ToF camera_info, no poses) carries `poses: []` and `outlier_poses: []`. A result over the limits keeps its report.md, which now lists the flagged poses.
- `failed_limits` in result.json is the core text, so a failed RGB/ToF/stereo limit caused by a flagged pose reads `... - outlier pose 007 (66.600 px): delete pose 007 and Compute again`.
- The contract fixture `tests/fixtures/api/result.json` gains `poses` (one entry with float errors and `outlier: ["rgb"]`) and `outlier_poses: ["007"]`; `test_passing_session_writes_all_result_files` and `test_result_matches_contract_fixture` in `test_app.py` keep passing against it (the passing synthetic session uses no skipped stereo pair, so every error is a float).
- New `test_compute.py` tests: a stored synthetic session with one pose whose pose.json RGB points are corrupted gives `passed: false`, `outlier_poses == [that id]`, that pose's `outlier` contains `rgb`, the `limits.rgb_max_rms_px` line of `failed_limits` names the pose with `delete pose <id> and Compute again`, and report.md has the `## Per-pose reprojection errors` section naming it; a clean passing session gives `outlier_poses == []` and one `poses` entry per pose id; an input-error result carries both empty lists.
- Node `uv run poe lint` passes; every existing node test passes.

## Acceptance conditions

- a1: a corrupted pose is reported as the outlier with delete and Compute again [evidence: test:nodes/head_rgbd_calibration/tests/test_compute.py::test_outlier_pose_named_in_result_and_report]
- a2: a clean session has per-pose errors and no outlier [evidence: test:nodes/head_rgbd_calibration/tests/test_compute.py::test_clean_session_has_pose_errors_without_outliers]
- a3: an input-error result carries empty poses and outlier_poses [evidence: test:nodes/head_rgbd_calibration/tests/test_compute.py::test_error_result_has_empty_pose_lists]
- a4: the result matches the updated contract fixture [evidence: test:nodes/head_rgbd_calibration/tests/test_compute.py::test_passing_session_writes_all_result_files]
- a5: compute_result passes the stored pose ids [evidence: symbol:nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py::compute_result]
- a6: node tests and lint pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint]

## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py`
- `nodes/head_rgbd_calibration/tests/test_compute.py`
- `nodes/head_rgbd_calibration/tests/fixtures/api/result.json`

## Interfaces

- result.json keys `poses: [{id, rgb_error_px, tof_error_px, stereo_error_px, outlier}]` and `outlier_poses: [id]`, defined by `nodes/head_rgbd_calibration/tests/fixtures/api/result.json` (part `rgbd-result-pose-errors`, used by calibrate-pose-list-hints).
- Uses `calibrate.py::calibrate_detections(pose_ids=...)`, `PoseErrors`, `outlier_pose_ids` (part `pose-quality-errors`).

## Expected outcome

A Compute on a session with a bad pose writes `passed: false`, `outlier_poses: ["<id>"]`, a `failed_limits` line ending `delete pose <id> and Compute again`, and a report.md naming it; a clean session lists per-pose errors with no outlier.

## Steps

- Update the `result.json` fixture and write the three new `test_compute.py` tests; run `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_compute.py -q`: red (missing keys).
- Pass `pose_ids`, add `poses` / `outlier_poses` to `base` and to the computed result.
- Run the verify command: all pass, lint clean.

## Notes

- `store.load_views` already returns `pose_ids` matching `views`; `store.py` is not touched.
- `test_app.py::new_result` copies the result fixture, so `test_result_matches_contract_fixture` keeps passing with no edit.
- Corrupt a pose by rewriting its pose.json `rgb.points` (e.g. shift half of them by 40 px) after `add_pose`; keep at least `limits.min_views` poses (the test config may lower it).
