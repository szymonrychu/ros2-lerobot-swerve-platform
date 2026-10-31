# Per-pose quality hints in the RGB-D calibration capture implementation plan

## Agreed rung

Rung 1 of plan.md, migrated from the epic format: the steps are the epic's tasks.

## Approach

Migrated from the epic format: one step per task file, in dependency order, each with its status, files, verify, verified head and every body section.

## Chunk 1: migrated tasks (model: sonnet)

### S1: Capture corner hints

- Status: done
- Files: `shared/ros2_common/head_rgbd_calibration/hints.py`, `tests/test_head_rgbd_calib_hints.py`, `nodes/head_rgbd_calibration/head_rgbd_calibration/store.py`, `nodes/head_rgbd_calibration/head_rgbd_calibration/app.py`, `nodes/head_rgbd_calibration/tests/test_store.py`, `nodes/head_rgbd_calibration/tests/test_app.py`, `nodes/head_rgbd_calibration/tests/fixtures/api/session.json`
- Verify: `uv run --frozen pytest tests/test_head_rgbd_calib_hints.py -q && uv run poe lint && cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_store.py tests/test_app.py -q && uv run poe lint`
- Verified head: 9258c6ea32655f850a1092d127eafd109d3c0429

#### Requirement

Give every captured RGB-D pose a capture-time quality hint from its corner counts, store it in pose.json and return it from the session API (design D5).

- New `shared/ros2_common/head_rgbd_calibration/hints.py` with, grouped at the top, `FEW_CORNERS_FRACTION = 0.5`, and `capture_hints(rgb: Detection, tof: Detection, board: BoardConfig) -> list[str]`. With `inner = (board.squares_x - 1) * (board.squares_y - 1)` (20 for the default 6x5 board), in this order: `few corners: {n} RGB - move the board closer / fill the frame` when the RGB corner count `n < FEW_CORNERS_FRACTION * inner`; `few corners: {n} ToF - move the board closer to the ToF camera / fill its frame` for the ToF count by the same rule; `only {n} corners seen by both cameras - not used for the stereo fit; keep the board in both views` when the shared corner ids `n < extrinsics.MIN_COMMON_CORNERS`. A good pose gives `[]`. It imports `MIN_COMMON_CORNERS` from `extrinsics.py` and does not edit that module.
- `nodes/head_rgbd_calibration/head_rgbd_calibration/store.py::SessionStore.add_pose` writes `"hints": capture_hints(...)` (with `self.cfg.board`) into pose.json next to `rgb`, `tof` and `depth_stamp_ns`.
- `SessionStore.pose_hints(sid, pid) -> list[str]`: the pose.json `hints` when the key is present, otherwise computed with `capture_hints` from the stored `rgb` / `tof` ids and points, so sessions captured before this change (the robot's current session) get hints too. Nothing is written back.
- `GET /api/sessions/{sid}` (`app.py`): every `poses[]` entry gains `hints: list[str]` from `pose_hints`; the other fields are unchanged.
- New contract fixture `nodes/head_rgbd_calibration/tests/fixtures/api/session.json` (the full GET /api/sessions/{sid} body, one pose with one hint) and a `test_app.py` test asserting the endpoint matches it with `assert_matches_fixture`.
- New root test `tests/test_head_rgbd_calib_hints.py`: each of the three hints at its threshold edge (n just below gives the hint, n at the threshold gives none), the order, a full-board pose gives `[]`, a non-default board size changes the threshold.
- Node tests: `test_store.py` checks pose.json carries `hints` after `add_pose` (a pose built with few RGB corners) and the legacy fallback for a pose.json without the key; `test_app.py` checks the session endpoint returns the hint of a few-corner pose and `[]` for a good one.
- Root `uv run poe lint` and node `uv run poe lint` pass; every existing node and calibration test passes.

#### Acceptance conditions

- a1: each capture hint fires just below its threshold and not at it [evidence: test:tests/test_head_rgbd_calib_hints.py::test_hints_at_threshold_edges]
- a2: a full-board pose has no hint [evidence: test:tests/test_head_rgbd_calib_hints.py::test_full_board_pose_has_no_hint]
- a3: the threshold follows the board size [evidence: test:tests/test_head_rgbd_calib_hints.py::test_threshold_follows_board_size]
- a4: add_pose stores hints in pose.json [evidence: test:nodes/head_rgbd_calibration/tests/test_store.py::test_add_pose_stores_hints]
- a5: poses stored without hints get them computed [evidence: test:nodes/head_rgbd_calibration/tests/test_store.py::test_pose_hints_fallback_for_legacy_pose]
- a6: the session endpoint returns per-pose hints [evidence: test:nodes/head_rgbd_calibration/tests/test_app.py::test_session_poses_carry_hints]
- a7: the session endpoint matches the session.json contract fixture [evidence: test:nodes/head_rgbd_calibration/tests/test_app.py::test_session_matches_contract_fixture]
- a8: capture_hints exists in the core [evidence: symbol:shared/ros2_common/head_rgbd_calibration/hints.py::capture_hints]
- a9: node tests and lint pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint]
- a10: root lint passes [evidence: command:uv run poe lint]

#### Files

- `shared/ros2_common/head_rgbd_calibration/hints.py`
- `tests/test_head_rgbd_calib_hints.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/store.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/app.py`
- `nodes/head_rgbd_calibration/tests/test_store.py`
- `nodes/head_rgbd_calibration/tests/test_app.py`
- `nodes/head_rgbd_calibration/tests/fixtures/api/session.json`

#### Interfaces

- `shared/ros2_common/head_rgbd_calibration/hints.py::capture_hints(rgb: Detection, tof: Detection, board: BoardConfig) -> list[str]`, `FEW_CORNERS_FRACTION`.
- `nodes/head_rgbd_calibration/head_rgbd_calibration/store.py`: pose.json key `hints: list[str]`; `SessionStore.pose_hints(sid, pid) -> list[str]`.
- `GET /api/sessions/{sid}` `poses[].hints: list[str]`; fixture `nodes/head_rgbd_calibration/tests/fixtures/api/session.json` (part `pose-capture-hints`, used by calibrate-pose-list-hints).

#### Expected outcome

A pose captured with 6 RGB corners has `"hints": ["few corners: 6 RGB - move the board closer / fill the frame"]` in pose.json and in the session API; poses stored before this change get the same hint computed on read.

#### Steps

- Write `tests/test_head_rgbd_calib_hints.py`; run `uv run --frozen pytest tests/test_head_rgbd_calib_hints.py -q`: red (no module `hints`).
- Implement `hints.py`; root tests green.
- Write the store and app tests and the `session.json` fixture; run `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_store.py tests/test_app.py -q`: red.
- Implement `add_pose` hints, `pose_hints` and the session endpoint field; run the verify command: all pass, lint clean.

#### Notes

- `charuco.MIN_CORNERS` (6) is the detection floor, so a stored pose has at least 6 corners per camera; the few-corner threshold is half the board's inner corners (10 of 20), design D5, open question Q1.
- `tests/pose_factory.py::make_pose` and `detection_at` build poses with chosen corner points.
- The `contract.py` list rule: every list element must match the first fixture element, an empty fixture list requires an empty list, so the fixture's `results` is `[]` and the test session has no result.
- The head-and-arm-calibration-fro epic later adds still sessions without ToF; this task keeps hints to `rgbd` poses (both detections present), cross-epic contract `rgbd-pose-hints`.

### S2: Per-pose errors and outliers

- Status: done
- Files: `shared/ros2_common/head_rgbd_calibration/calibrate.py`, `shared/ros2_common/head_rgbd_calibration/extrinsics.py`, `tests/test_head_rgbd_calib_quality.py`
- Verify: `uv run --frozen pytest tests/test_head_rgbd_calib_quality.py tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_extrinsics.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_poses.py tests/test_head_rgbd_calibrate_cli.py -q && uv run poe lint`
- Verified head: 331a7ac9b3d4698db148799846781a29be978b9b

#### Requirement

Record per-pose reprojection errors for the RGB intrinsic fit, the ToF intrinsic fit and the stereo fit in the shared calibration core, flag outlier poses, and name them in `failed_limits` and the report (design D2, D3, D4). Hints only: no pose is ever dropped or refitted automatically (D1).

- `extrinsics.py::stereo_calibrate` uses `cv2.stereoCalibrateExtended` (CALIB_FIX_INTRINSIC, same inputs, R and T passed as None or identity/zeros) and `ExtrinsicResult` gains `pair_indices: tuple[int, ...] = ()` (index into `pairs` of every pair actually used, in order; pairs under MIN_COMMON_CORNERS stay skipped) and `per_pair_errors: tuple[float, ...] = ()` (per used pair `sqrt((e_tof**2 + e_rgb**2) / 2)` from perViewErrors). `stereo_rms`, `r`, `t` and `pair_count` are unchanged in value for the same input; `load_extrinsics_yaml` and every existing caller keep working through the defaults.
- `calibrate.py` gains, grouped at the top, `OUTLIER_MEDIAN_FACTOR = 3.0`, a frozen dataclass `PoseErrors(pose_id: str, rgb_px: float | None, tof_px: float | None, stereo_px: float | None, outlier: tuple[str, ...])` (`outlier` is the subset of `("rgb", "tof", "stereo")`, in that order, of the fits in which the pose is flagged; a `None` error means the pose was not in that fit), and `CalibrationRun.poses: tuple[PoseErrors, ...] = ()` (one entry per input view, in view order).
- `calibrate_detections(..., pose_ids: list[str] | None = None)`: `pose_ids` names the views (default `f"{i + 1:03d}"`); a length other than `len(views)` raises ValueError naming both lengths. The RGB and ToF errors come from `IntrinsicsResult.per_view_errors` mapped back to the view index, the stereo error from `pair_indices`.
- Outlier rule, per fit: a pose is flagged when its error is greater than `max(OUTLIER_MEDIAN_FACTOR * median of that fit's per-pose errors, the fit's limit)`; the limits are `limits.rgb_max_rms_px`, `limits.tof_max_rms_px`, `limits.stereo_max_rms_px`. `outlier_pose_ids(run) -> list[str]` returns the flagged pose ids in view order.
- `failed_limits`: a failed RMS limit whose fit has flagged poses gets the suffix ` - outlier pose 007 (66.600 px): delete pose 007 and Compute again` (one pose) or ` - outlier poses 007 (66.600 px), 012 (5.100 px): delete poses 007, 012 and Compute again` (several, view order). A failed limit without flagged poses and the plane-bias message are byte-identical to today.
- `report_markdown`: when `run.poses` is non-empty it appends a `## Per-pose reprojection errors` table (`| Pose | RGB px | ToF px | Stereo px | Flag |`, errors with 3 decimals, `-` for None, Flag `outlier: RGB, stereo` or empty) followed by `Flagged poses (error > max(3 x median, limit)):` and one line per flagged pose `- pose 007: RGB 66.600 px, stereo 40.100 px - delete pose 007 and Compute again`, or the line `Flagged poses: none`. With `run.poses` empty the report is byte-identical to today, so `GOLDEN_REPORT` stays unchanged.
- New `tests/test_head_rgbd_calib_quality.py` covers: a synthetic session (reuse `tests/rgbd_synthetic.py` / the existing synthetic board helpers) with one view whose RGB points are corrupted gets that view flagged in `rgb` (and `stereo` when it crosses the rule) and no other view flagged; a clean synthetic session flags nothing; the rule uses the limit when 3 x median is lower and 3 x median when higher (unit test of the flagging helper with fixed numbers); `pose_ids` default and length mismatch; `stereo_calibrate` `pair_indices` skips an under-6-common-corners pair and `per_pair_errors` reproduces `stereo_rms` as the point-weighted RMS; the `failed_limits` suffix for one and for several flagged poses; the report section with a flagged pose and with none.
- Every existing calibration test passes with no assertion changed; root `uv run poe lint` passes.

#### Acceptance conditions

- a1: an RGB-corrupted synthetic view is flagged and no clean view is [evidence: test:tests/test_head_rgbd_calib_quality.py::test_corrupted_view_is_the_only_outlier]
- a2: a clean synthetic session flags no pose [evidence: test:tests/test_head_rgbd_calib_quality.py::test_clean_session_flags_nothing]
- a3: the threshold is max(3 x median, limit) [evidence: test:tests/test_head_rgbd_calib_quality.py::test_outlier_threshold_is_max_of_median_factor_and_limit]
- a4: pose_ids default to 001.. and a length mismatch raises [evidence: test:tests/test_head_rgbd_calib_quality.py::test_pose_ids_default_and_length_mismatch]
- a5: stereo pair_indices skip short pairs and per_pair_errors reproduce stereo_rms [evidence: test:tests/test_head_rgbd_calib_quality.py::test_stereo_per_pair_errors_match_rms]
- a6: failed_limits names the outlier poses with delete and Compute again [evidence: test:tests/test_head_rgbd_calib_quality.py::test_failed_limits_names_outlier_poses]
- a7: report.md lists per-pose errors and the flagged poses [evidence: test:tests/test_head_rgbd_calib_quality.py::test_report_lists_flagged_poses]
- a8: PoseErrors and outlier_pose_ids exist in the core [evidence: symbol:shared/ros2_common/head_rgbd_calibration/calibrate.py::outlier_pose_ids]
- a9: every existing calibration test passes unchanged [evidence: command:uv run --frozen pytest tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_extrinsics.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_poses.py tests/test_head_rgbd_calibrate_cli.py -q]
- a10: root lint passes [evidence: command:uv run poe lint]

#### Files

- `shared/ros2_common/head_rgbd_calibration/calibrate.py`
- `shared/ros2_common/head_rgbd_calibration/extrinsics.py`
- `tests/test_head_rgbd_calib_quality.py`

#### Interfaces

- `shared/ros2_common/head_rgbd_calibration/calibrate.py`: `OUTLIER_MEDIAN_FACTOR`, `PoseErrors(pose_id, rgb_px, tof_px, stereo_px, outlier)`, `CalibrationRun.poses`, `calibrate_detections(views, rgb_size, tof_size, tof_info, board_cfg, limits, pose_ids=None)`, `outlier_pose_ids(run) -> list[str]`, `failed_limits`, `report_markdown` (part `pose-quality-errors`, used by compute-result-pose-errors).
- `shared/ros2_common/head_rgbd_calibration/extrinsics.py`: `ExtrinsicResult.pair_indices`, `ExtrinsicResult.per_pair_errors` (defaults `()`).

#### Expected outcome

`calibrate_detections` on a session with one bad view returns a run whose `poses` flag exactly that view, `failed_limits` says `delete pose NNN and Compute again`, and report.md lists it; the existing golden report and every existing test are unchanged.

#### Steps

- Write `tests/test_head_rgbd_calib_quality.py` (the a1-a7 tests); run `uv run --frozen pytest tests/test_head_rgbd_calib_quality.py -q`: red (ImportError on `PoseErrors` / `outlier_pose_ids`).
- Switch `stereo_calibrate` to `cv2.stereoCalibrateExtended` and record `pair_indices` / `per_pair_errors`.
- Add `PoseErrors`, the flagging helper, `CalibrationRun.poses`, `pose_ids`, the `failed_limits` suffix and the report section.
- Run the verify command: all pass, lint clean.

#### Notes

- `IntrinsicsResult.per_view_errors` already holds the per-view RMS of `cv2.calibrateCameraExtended`; only the mapping back to the view index is new (the RGB and ToF fits use only views whose detection is not None).
- OpenCV 4.14 signature (probed): `stereoCalibrateExtended(objectPoints, imagePoints1, imagePoints2, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageSize, R, T[, E[, F[, rvecs[, tvecs[, perViewErrors[, flags[, criteria]]]]]]]) -> retval, K1, D1, K2, D2, R, T, E, F, rvecs, tvecs, perViewErrors`; perViewErrors is (N, 2), camera 1 is the ToF.
- The real failure this targets: RGB RMS 8.28 px against 0.5 because pose 007 (6 RGB corners) had 66.6 px while every other pose was <= 0.33 px.
- `OUTLIER_MEDIAN_FACTOR` is a module constant, not a `LimitsConfig` field (design D3): `test_core_config_defaults_match_cli_yaml` pins LimitsConfig to the CLI yaml and every result.json copies the limits.
- Hints only (D1): never drop or refit a view.

### S3: Compute result pose errors

- Status: done
- Files: `nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py`, `nodes/head_rgbd_calibration/tests/test_compute.py`, `nodes/head_rgbd_calibration/tests/fixtures/api/result.json`
- Verify: `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_compute.py tests/test_app.py -q && uv run poe lint`
- Verified head: e45812cd3010afd92e236a048565e771b79b46b6

#### Requirement

Carry the per-pose errors and the outlier poses of a Compute into result.json and report.md (design D4, D6).

- `nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py::compute_result` passes `pose_ids=views.pose_ids` to `calibrate_detections`, so report.md and `failed_limits` name the stored pose ids (e.g. `007`).
- result.json gains `poses: [{id, rgb_error_px, tof_error_px, stereo_error_px, outlier}]` (one entry per stored pose in pose id order; errors float or null when the pose was not in that fit; `outlier` the list of flagged fits from `PoseErrors.outlier`) and `outlier_poses: [id, ...]` (from `outlier_pose_ids`). Both are in `base`, empty lists, so an input-error result (too few views, no ToF camera_info, no poses) carries `poses: []` and `outlier_poses: []`. A result over the limits keeps its report.md, which now lists the flagged poses.
- `failed_limits` in result.json is the core text, so a failed RGB/ToF/stereo limit caused by a flagged pose reads `... - outlier pose 007 (66.600 px): delete pose 007 and Compute again`.
- The contract fixture `tests/fixtures/api/result.json` gains `poses` (one entry with float errors and `outlier: ["rgb"]`) and `outlier_poses: ["007"]`; `test_passing_session_writes_all_result_files` and `test_result_matches_contract_fixture` in `test_app.py` keep passing against it (the passing synthetic session uses no skipped stereo pair, so every error is a float).
- New `test_compute.py` tests: a stored synthetic session with one pose whose pose.json RGB points are corrupted gives `passed: false`, `outlier_poses == [that id]`, that pose's `outlier` contains `rgb`, the `limits.rgb_max_rms_px` line of `failed_limits` names the pose with `delete pose <id> and Compute again`, and report.md has the `## Per-pose reprojection errors` section naming it; a clean passing session gives `outlier_poses == []` and one `poses` entry per pose id; an input-error result carries both empty lists.
- Node `uv run poe lint` passes; every existing node test passes.

#### Acceptance conditions

- a1: a corrupted pose is reported as the outlier with delete and Compute again [evidence: test:nodes/head_rgbd_calibration/tests/test_compute.py::test_outlier_pose_named_in_result_and_report]
- a2: a clean session has per-pose errors and no outlier [evidence: test:nodes/head_rgbd_calibration/tests/test_compute.py::test_clean_session_has_pose_errors_without_outliers]
- a3: an input-error result carries empty poses and outlier_poses [evidence: test:nodes/head_rgbd_calibration/tests/test_compute.py::test_error_result_has_empty_pose_lists]
- a4: the result matches the updated contract fixture [evidence: test:nodes/head_rgbd_calibration/tests/test_compute.py::test_passing_session_writes_all_result_files]
- a5: compute_result passes the stored pose ids [evidence: symbol:nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py::compute_result]
- a6: node tests and lint pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint]

#### Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py`
- `nodes/head_rgbd_calibration/tests/test_compute.py`
- `nodes/head_rgbd_calibration/tests/fixtures/api/result.json`

#### Interfaces

- result.json keys `poses: [{id, rgb_error_px, tof_error_px, stereo_error_px, outlier}]` and `outlier_poses: [id]`, defined by `nodes/head_rgbd_calibration/tests/fixtures/api/result.json` (part `rgbd-result-pose-errors`, used by calibrate-pose-list-hints).
- Uses `calibrate.py::calibrate_detections(pose_ids=...)`, `PoseErrors`, `outlier_pose_ids` (part `pose-quality-errors`).

#### Expected outcome

A Compute on a session with a bad pose writes `passed: false`, `outlier_poses: ["<id>"]`, a `failed_limits` line ending `delete pose <id> and Compute again`, and a report.md naming it; a clean session lists per-pose errors with no outlier.

#### Steps

- Update the `result.json` fixture and write the three new `test_compute.py` tests; run `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_compute.py -q`: red (missing keys).
- Pass `pose_ids`, add `poses` / `outlier_poses` to `base` and to the computed result.
- Run the verify command: all pass, lint clean.

#### Notes

- `store.load_views` already returns `pose_ids` matching `views`; `store.py` is not touched.
- `test_app.py::new_result` copies the result fixture, so `test_result_matches_contract_fixture` keeps passing with no edit.
- Corrupt a pose by rewriting its pose.json `rgb.points` (e.g. shift half of them by 40 px) after `add_pose`; keep at least `limits.min_views` poses (the test config may lower it).

### S4: Calibrate pose list hints

- Status: done
- Files: `nodes/web_ui/frontend/src/calibrate/rgbdApi.ts`, `nodes/web_ui/frontend/src/calibrate/rgbdApi.test.ts`, `nodes/web_ui/frontend/src/calibrate/RgbdCapture.tsx`, `nodes/web_ui/frontend/src/calibrate/RgbdCapture.test.tsx`
- Verify: `cd nodes/web_ui/frontend && npx vitest run src/calibrate && npx tsc --noEmit && npm run lint`
- Verified head: 44356aba35e6cd165710cae3ac1fa7875e3c8af6

#### Requirement

Show the capture hints and the per-pose Compute errors in the Calibrate page RGB-D pose list (design D6). Delete-pose stays the only action; nothing is pruned automatically (D1).

- `nodes/web_ui/frontend/src/calibrate/rgbdApi.ts`: `PoseSummary` gains `hints?: string[]`; new `PoseError {id: string; rgb_error_px: number | null; tof_error_px: number | null; stereo_error_px: number | null; outlier: string[]}`; `ResultSummary` gains `poses?: PoseError[]` and `outlier_poses?: string[]`. The guards accept the fields when absent (a node or result from before this change) and reject them when present with the wrong type. Exported helper `poseErrors(detail: SessionDetail): Map<string, PoseError>` returns the `poses` of the newest result (`detail.results[0]`, the node lists newest first) keyed by pose id, empty when there is no result or it has no `poses`.
- `RgbdCapture.tsx`, per pose card: every capture hint as its own warning-coloured caption line under the corner counts; when the newest result has the pose, a caption `RGB 0.210 / ToF 0.120 / stereo 0.400 px` (3 decimals, `-` for null); when its `outlier` is non-empty, the card border uses the error colour and a line `high error - delete and Compute again` is shown. A pose missing from the newest result (captured after it) shows no error line. The delete button and confirm dialog are unchanged.
- The session detail is fetched again when a compute finishes (`compute.running` goes from true to false), so the errors appear without reopening the session.
- Tests: `rgbdApi.test.ts` accepts the node fixtures `nodes/head_rgbd_calibration/tests/fixtures/api/session.json` and `result.json` (with `hints`, `poses`, `outlier_poses`), accepts a session and a result without them, rejects a non-array `hints` / `poses`, and checks `poseErrors` in a `describe('poseErrors')` block; `RgbdCapture.test.tsx` renders the hint line, the error caption, the highlighted flagged card with its one-line hint, no error line for a pose absent from the result, and the refetch after a compute finishes.
- `npx vitest run src/calibrate`, `npx tsc --noEmit` and `npm run lint` pass in `nodes/web_ui/frontend`.

#### Acceptance conditions

- a1: the api guards and poseErrors accept the node fixtures and older bodies [evidence: test:nodes/web_ui/frontend/src/calibrate/rgbdApi.test.ts::poseErrors]
- a2: the pose list shows hints, errors and the flagged-pose highlight with its hint [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/RgbdCapture.test.tsx]
- a3: poseErrors is exported from the api module [evidence: symbol:nodes/web_ui/frontend/src/calibrate/rgbdApi.ts::poseErrors]
- a4: calibrate tests, typecheck and lint pass [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate && npx tsc --noEmit && npm run lint]

#### Files

- `nodes/web_ui/frontend/src/calibrate/rgbdApi.ts`
- `nodes/web_ui/frontend/src/calibrate/rgbdApi.test.ts`
- `nodes/web_ui/frontend/src/calibrate/RgbdCapture.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdCapture.test.tsx`

#### Interfaces

- `nodes/web_ui/frontend/src/calibrate/rgbdApi.ts`: `PoseSummary.hints?`, `PoseError`, `ResultSummary.poses?`, `ResultSummary.outlier_poses?`, `poseErrors(detail: SessionDetail): Map<string, PoseError>`.
- Uses the node fixtures `nodes/head_rgbd_calibration/tests/fixtures/api/session.json` (part `pose-capture-hints`) and `result.json` (part `rgbd-result-pose-errors`).

#### Expected outcome

In the Calibrate page RGB-D section each pose card shows its capture hints; after a Compute it shows `RGB / ToF / stereo` errors, and a flagged pose has an error-coloured border and `high error - delete and Compute again` next to its delete button.

#### Steps

- Write the `rgbdApi.test.ts` and `RgbdCapture.test.tsx` cases; run `cd nodes/web_ui/frontend && npx vitest run src/calibrate/rgbdApi.test.ts src/calibrate/RgbdCapture.test.tsx`: red.
- Extend the types, guards and `poseErrors`; render the hint, error and flagged lines; refetch the detail when a compute finishes.
- Run the verify command: all pass, typecheck and lint clean.

#### Notes

- The web_ui backend proxy (`nodes/web_ui/web_ui/rgbd_calibration.py`) forwards bodies unchanged; no backend change.
- Guards stay tolerant of absent fields because web_ui and head_rgbd_calibration deploy as separate units (design D6).
- Fixture import path pattern: `import x from '../../../../head_rgbd_calibration/tests/fixtures/api/<name>.json'` (as in `rgbdApi.test.ts`).
- `RgbdCapture` already receives `compute: ComputeState`; track the previous `compute.running` to detect the finish.

### S5: Hints docs and deploy

- Status: done
- Files: `nodes/head_rgbd_calibration/README.md`, `shared/README.md`, `nodes/web_ui/README.md`, `tests/README.md`
- Verify: `grep -q test_head_rgbd_calib_quality.py tests/README.md && grep -q hints.py shared/README.md && uv run poe lint`
- Verified head: 8c540d68cad318c488ed9e663a6d2383bd5adfd1

#### Requirement

Document the per-pose hints and errors, index the tests, and deploy both changed nodes to the client (design D7).

- `nodes/head_rgbd_calibration/README.md`: the Capture section documents pose.json `hints` (the three texts, the half-the-board and 6-common-corner thresholds, the fallback for older poses); a new `### Per-pose errors and outliers` heading under the Compute section documents the per-pose errors, the outlier rule (error > max(3 x median, limit)), the result.json `poses` / `outlier_poses` keys, the report.md section and the `failed_limits` suffix, and that nothing is pruned automatically (delete the pose and Compute again); the HTTP API / Response contract section names the `hints` field and the new `session.json` fixture.
- `shared/README.md` `## head_rgbd_calibration` lists `hints.py` and the per-pose error and outlier API of `calibrate.py` / `extrinsics.py`.
- `nodes/web_ui/README.md` `## Calibrate page (hidden)` describes the hint lines, the per-pose error caption and the flagged-pose highlight.
- `tests/README.md` lists `tests/test_head_rgbd_calib_quality.py` and `tests/test_head_rgbd_calib_hints.py` with every test, and the new node and frontend tests in the head_rgbd_calibration and web_ui entries.
- Deploy with `./scripts/deploy-nodes.sh client head_rgbd_calibration web_ui 2>&1 | tee .logs/deploy-client-calib-hints.log` (after the `ansible-deploy` skill check); the recap shows `failed=0` and `unreachable=0`. No change to `ansible/group_vars/client.yml`.
- On the robot (`http://client.ros2.lan:18110`), `GET /api/sessions/<sid>` for the session the brief's pose 007 is in returns `hints` on every pose and pose 007 carries the `few corners: 6 RGB` hint; then `POST /api/sessions/<sid>/compute` and the finished result lists 007 in `outlier_poses` with `delete pose 007 and Compute again` in `failed_limits`. The two JSON bodies are saved under `.logs/` and quoted in the task report. If the robot is unreachable, the deploy step reports `blocked` with whoCanClear "the user (power on / network the client Pi)".

#### Acceptance conditions

- a1: the node README documents capture hints [evidence: doc-section:nodes/head_rgbd_calibration/README.md#Capture]
- a2: the node README documents per-pose errors and outliers [evidence: doc-section:nodes/head_rgbd_calibration/README.md#Per-pose errors and outliers]
- a3: the shared README lists the hints module [evidence: command:grep -q "hints.py" shared/README.md]
- a4: the web_ui README describes the pose hints [evidence: command:grep -q "high error - delete and Compute again" nodes/web_ui/README.md]
- a5: tests/README.md indexes the new test files [evidence: command:grep -q "test_head_rgbd_calib_quality.py" tests/README.md && grep -q "test_head_rgbd_calib_hints.py" tests/README.md]
- a6: the client deploy of both nodes succeeded [evidence: command:grep -Eq "client.*failed=0" .logs/deploy-client-calib-hints.log && ! grep -Eq "unreachable=[1-9]|failed=[1-9]" .logs/deploy-client-calib-hints.log]
- a7: on the robot pose 007 carries its capture hint and the new Compute flags it [evidence: judgement:]

#### Files

- `nodes/head_rgbd_calibration/README.md`
- `shared/README.md`
- `nodes/web_ui/README.md`
- `tests/README.md`

#### Interfaces

No code interfaces; documents `hints`, `poses`, `outlier_poses`, `capture_hints`, `PoseErrors`, `outlier_pose_ids`.

#### Expected outcome

The READMEs and tests/README.md describe the hints and per-pose errors; head_rgbd_calibration and web_ui run the new code on the client; on the robot pose 007 shows its few-corners hint and a new Compute flags it.

#### Steps

- Update the four READMEs; run `grep -q test_head_rgbd_calib_quality.py tests/README.md && grep -q hints.py shared/README.md && uv run poe lint`.
- Invoke the `ansible-deploy` skill, then `mkdir -p .logs && ./scripts/deploy-nodes.sh client head_rgbd_calibration web_ui 2>&1 | tee .logs/deploy-client-calib-hints.log`; expect the play recap with `failed=0 unreachable=0`.
- `curl -s http://client.ros2.lan:18110/api/sessions` to find the session holding pose 007, then save `GET /api/sessions/<sid>` to `.logs/calib-hints-session.json` and check pose 007's `hints`.
- `curl -s -X POST http://client.ros2.lan:18110/api/sessions/<sid>/compute`, poll `GET /api/state` until compute stops, save the result to `.logs/calib-hints-result.json` and check `outlier_poses` contains `007`.

#### Ruling

- 2026-10-21 accepted: merged 90e642da, deployed 2026-10-21 23:2x; live check 2026-10-22 01:03: GET session returns per-pose hints (few corners on 9/8/6-corner poses), compute r-20261022-010256 passed with per-pose rgb/tof/stereo errors for 29 poses, no outliers (pose 007 already deleted by the user); bodies in .logs/hints-live-*.json

#### Notes

- Docs-only file changes plus a deploy: the TDD exception for documentation applies; say so in `tests.note`.
- Deploying web_ui and head_rgbd_calibration does not touch claude_agent or mcp_server, so a running agent session survives.
- client-cpu-budget-recovery-a is editing `ansible/group_vars/client.yml`; this task must not change it.
- Do not delete pose 007 on the robot: the user decides (hints only, D1).

## Files to delete

none recorded in the epic format

## Final verify

none recorded in the epic format

## After merge

none

## Rulings (migrated)

- hints-docs-and-deploy: accept true, at 2026-10-21T23:06:43Z: merged 90e642da, deployed 2026-10-21 23:2x; live check 2026-10-22 01:03: GET session returns per-pose hints (few corners on 9/8/6-corner poses), compute r-20261022-010256 passed with per-pose rgb/tof/stereo errors for 29 poses, no outliers (pose 007 already deleted by the user); bodies in .logs/hints-live-*.json
