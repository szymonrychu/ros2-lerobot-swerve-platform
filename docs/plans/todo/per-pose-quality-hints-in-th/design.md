# Per-pose quality hints in the RGB-D calibration capture design

## Summary

Adds per-pose quality hints to the head RGB-D calibration. At capture each pose gets hints from its corner counts (too few RGB or ToF corners, too few shared corners for the stereo fit), stored in pose.json and returned by the session API. Compute records each pose's RGB, ToF and stereo reprojection error, flags outliers (error > max(3 x median, limit)), and names them in result.json, report.md and failed_limits with "delete pose NNN and Compute again". The Calibrate page pose list shows both. Hints only: nothing is pruned automatically.

## Intent and why

On 2026-10-21 Compute failed with RGB RMS 8.28 px (limit 0.5) because one pose (007, only 6 RGB corners) had a 66.6 px per-view error while every other pose was <= 0.33 px. The orchestrator found it by hand with `cv2.calibrateCameraExtended` perViewErrors; the user had no way to see which pose was bad. The user asked for such hints as a small epic.

## Goals

- `pose.json` carries `hints`, and `GET /api/sessions/{sid}` returns `poses[].hints` for every pose, also for poses stored before this change.
- `result.json` carries `poses` (per-pose `rgb_error_px`, `tof_error_px`, `stereo_error_px`, `outlier`) and `outlier_poses`; `report.md` has a per-pose table and the flagged poses; a failed RMS limit with flagged poses names them.
- The Calibrate page RGB-D pose list shows the hints, the errors of the newest result, and highlights flagged poses with "high error - delete and Compute again".
- Root, node and frontend tests cover each part; READMEs and `tests/README.md` updated; both nodes deployed to the client.

## Non-goals

- Automatic pruning, refitting or reordering of poses.
- New or changed acceptance limits; `LimitsConfig` is unchanged.
- Any change to `ansible/group_vars/client.yml` (client-cpu-budget-recovery-a is editing it).
- Hints for the still sessions (`head_chain`, `arm`) that head-and-arm-calibration-fro adds later.
- Plane-check per-pair outliers.

## Anti-goals

- A changed fit result: `stereo_rms`, `r`, `t` and the intrinsics must stay numerically identical for the same input.
- A changed golden report or `failed_limits` text when no pose is flagged.
- A UI that breaks against an older node or an older result (fields absent).
- Hint noise: a hint on most good poses.

## Constraints

- Python type hints, constants in SCREAMING_SNAKE_CASE at the top of the file, docstrings with types, no try/except around imports (CLAUDE.md).
- Use OpenCV's own per-view errors (`calibrateCameraExtended`, `stereoCalibrateExtended`), no hand-rolled reprojection.
- Test first; existing calibration tests pass unchanged.
- Deploy only through `scripts/deploy-nodes.sh`, logs under `.logs/`.
- Node and frontend share the API shapes through the JSON fixtures in `nodes/head_rgbd_calibration/tests/fixtures/api/`.

## Decisions

### D1: Hints only, no automatic pruning

The operator gets the per-pose errors and hints and decides; deleting a pose and computing again stays a manual action through the existing delete-pose button. Alternative: drop outliers and refit automatically; rejected by the user decision (2026-10-21) because a silent refit hides a bad capture and can drop a valid edge-of-frame pose. Serves A1, A3.

### D2: Per-pose errors from OpenCV's extended fits

RGB and ToF errors are `IntrinsicsResult.per_view_errors` (already from `cv2.calibrateCameraExtended`), mapped back to the view index. `stereo_calibrate` switches to `cv2.stereoCalibrateExtended` with the same flags; its (N, 2) perViewErrors become one per-pair value `sqrt((e_tof**2 + e_rgb**2) / 2)` (each pair has the same point count in both cameras, so this is the pair's RMS over both images) (my call), with `pair_indices` mapping used pairs back to views. Alternative: re-project per pose by hand with solvePnP; rejected, it duplicates OpenCV and can disagree with the fit's RMS. Serves A1.

### D3: Outlier rule max(3 x median, limit), as a module constant

A pose is flagged in a fit when its error > max(`OUTLIER_MEDIAN_FACTOR` x median of that fit's per-pose errors, the fit's RMS limit). `OUTLIER_MEDIAN_FACTOR = 3.0` is a constant at the top of `calibrate.py`, not a `LimitsConfig` field (my call): it is a hint heuristic, not an acceptance limit, and `LimitsConfig` is pinned to the CLI yaml by `test_core_config_defaults_match_cli_yaml` and copied into every result.json. On the 2026-10-21 session: median about 0.3 px, threshold max(0.9, 0.5) = 0.9 px, pose 007 at 66.6 px is flagged and no other pose is. Serves A1.

### D4: Outliers named in failed_limits and report.md

A failed RGB, ToF or stereo RMS limit whose fit has flagged poses gets the suffix ` - outlier pose 007 (66.600 px): delete pose 007 and Compute again` (several: ` - outlier poses 007 (66.600 px), 012 (5.100 px): delete poses 007, 012 and Compute again`). report.md appends `## Per-pose reprojection errors` (table and flagged list) only when per-pose data exists, so the golden report is unchanged. `calibrate_detections` takes `pose_ids` so the texts name stored ids. Serves A1.

### D5: Capture hints from corner counts, computed on read for old poses

`hints.py::capture_hints(rgb, tof, board)`: a few-corners hint per camera when its count < `FEW_CORNERS_FRACTION` (0.5) x the board's inner corners (10 of 20 on the 6x5 board) (my call, Q1), and a stereo hint when the shared corners < `MIN_COMMON_CORNERS` (6, the pair would be skipped). Stored in pose.json at `add_pose`; `SessionStore.pose_hints` computes them from the stored ids when pose.json has no `hints`, so the robot's current session (with pose 007) gets hints after the deploy without recapture (my call). Alternative: hint only at `MIN_CORNERS` (6); rejected, a stored pose always has >= 6 so it would never fire. Serves A2.

### D6: API shapes and a tolerant UI

result.json gains `poses: [{id, rgb_error_px, tof_error_px, stereo_error_px, outlier}]` and `outlier_poses`; input-error results carry empty lists. The session API gains `poses[].hints`. The frontend reads the errors of the newest result (`detail.results[0]`), refetches the session when a compute finishes, and its guards accept the new fields as optional because web_ui and head_rgbd_calibration deploy as separate units (my call). Serves A3.

### D7: Deploy both nodes with deploy-nodes.sh

`./scripts/deploy-nodes.sh client head_rgbd_calibration web_ui`, logged to `.logs/`; then a robot check on the session with pose 007 (hint visible, a fresh Compute flags it). User decision. Serves A4.

## Research

| Lens | Ran | Adopted ideas | Rejected ideas |
|---|---|---|---|
| brief (calib-hints.md) | ran | per-view errors from the extended OpenCV calls; max(3 x median, limit) rule; corner-count capture hints; "delete pose NNN and Compute again" wording | automatic pruning (brief, kept out by D1) |
| codebase (grep) | ran | `IntrinsicsResult.per_view_errors` already exists (`intrinsics.py::fit`); `store.load_views` returns `pose_ids`; node and frontend share JSON fixtures (`tests/contract.py`); web_ui proxy forwards bodies unchanged (`web_ui/rgbd_calibration.py`) | new LimitsConfig field (pinned to CLI yaml, `tests/test_head_rgbd_calib_core.py::test_core_config_defaults_match_cli_yaml`) |
| OpenCV probe | ran | `cv2.stereoCalibrateExtended` signature on 4.14.0 returns perViewErrors (N, 2) | none |

## Research inputs

| Source | Entry | Lands in |
|---|---|---|
| brief scope 1 | per-pose errors for RGB, ToF, stereo; outlier rule; report and failed_limits text | per-pose-errors-and-outliers, compute-result-pose-errors |
| brief scope 2 | capture hints from corner counts in pose.json and the session API | capture-corner-hints |
| brief scope 3 | Calibrate page pose list hints, errors, highlight | calibrate-pose-list-hints |
| brief scope 4 | tests for each part, READMEs, tests/README.md | every task (tests), hints-docs-and-deploy (docs) |
| brief deploy | deploy head_rgbd_calibration and web_ui via deploy-nodes.sh | hints-docs-and-deploy |
| intrinsics.py::fit | per_view_errors already computed | per-pose-errors-and-outliers (reuse) |
| tests/contract.py | fixtures shared by node and frontend tests | capture-corner-hints, compute-result-pose-errors, calibrate-pose-list-hints |
| brief Plan inputs | none present in the brief | rejected: the brief has no Plan inputs section |

## Decision coverage

| Decision | Tasks |
|---|---|
| D1 Hints only | per-pose-errors-and-outliers, calibrate-pose-list-hints |
| D2 Per-pose errors from OpenCV | per-pose-errors-and-outliers |
| D3 Outlier rule | per-pose-errors-and-outliers |
| D4 Outliers in failed_limits and report | per-pose-errors-and-outliers, compute-result-pose-errors |
| D5 Capture hints | capture-corner-hints |
| D6 API shapes and tolerant UI | compute-result-pose-errors, calibrate-pose-list-hints |
| D7 Deploy | hints-docs-and-deploy |

## Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|
| `pose-quality-errors` | per-pose-errors-and-outliers | compute-result-pose-errors | `shared/ros2_common/head_rgbd_calibration/calibrate.py` |
| `pose-capture-hints` | capture-corner-hints | calibrate-pose-list-hints | `nodes/head_rgbd_calibration/tests/fixtures/api/session.json` |
| `rgbd-result-pose-errors` | compute-result-pose-errors | calibrate-pose-list-hints | `nodes/head_rgbd_calibration/tests/fixtures/api/result.json` |

Cross-epic (registered with `ticket.py contract add`): `rgbd-pose-hints` and `rgbd-result-pose-errors`, consumer head-and-arm-calibration-fro (it extends pose.json, the session API, result.json and rgbdApi.ts).

## Review focus

- The view-index mapping: RGB/ToF fits skip views without a detection and the stereo fit skips pairs under 6 shared corners; an off-by-one names the wrong pose.
- `stereo_calibrate` result values identical before and after the switch to the extended call.
- Golden report and `failed_limits` texts byte-identical when nothing is flagged.
- Frontend guards tolerant of absent fields, strict on wrong types.

## Left to the builder

- Exact MUI styling of the hint and error lines, within: warning colour for hints, error colour border for flagged cards, touch targets unchanged.
- How the synthetic outlier is made (corrupted points, shifted pose), as long as only that view is flagged.
- Internal helper names besides the named interfaces.

## Assumptions and unknowns

- The robot's session with pose 007 still exists (not deleted); confirmed by the deploy task's `GET /api/sessions` probe; if it is gone, any session gives the hint check and the Compute check uses a fresh capture.
- The client Pi is reachable at deploy time; otherwise the deploy step is blocked on the user.

## Open questions

- Q1: Capture few-corners hint threshold: hint when a camera sees fewer than half the board's inner corners (10 of 20)? [recommended: yes, half the board (10 of 20)]

## Irreversible operations in scope

- Deploying head_rgbd_calibration and web_ui to the client (restarts both services); covered by the user decision and reversible by redeploying the previous commit.
- A Compute on the robot session adds a result directory; nothing is deleted (pose 007 stays until the user deletes it).
