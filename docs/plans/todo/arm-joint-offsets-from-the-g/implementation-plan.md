# Arm joint offsets from the gripper-held ChArUco board implementation plan

## Agreed rung

Not agreed yet: a draft migrated from the epic's tasks; the plan-writer revises it after the gate.

## Approach

Migrated from the epic format: one step per task file, in dependency order, each with its status, files, verify, verified head and every body section.

## Chunk 1: migrated tasks (model: opus)

### S1: Arm offsets plate solver

- Status: todo
- Files: `shared/ros2_common/head_rgbd_calibration/arm_offsets_fit.py`, `tests/head_arm_synthetic.py`, `tests/test_arm_offsets_fit.py`
- Verify: `uv run --frozen pytest tests/test_arm_offsets_fit.py -q && uv run poe lint`
- Verified head: none

#### Requirement

Moved verbatim on 2026-10-22 from head-and-arm-calibration-fro (user decision: the arm calibration is trusted; the head is calibrated against the arm there). Before running, re-plan this task with `ticket.py amend` against the parts that epic delivers (head_arm sessions and still capture, the sequencer and pose check, the head-from-arm fit and its synthetic scenes, the head-arm HTTP API); task slugs and design decisions named below refer to the old head-and-arm plan in its git history.

The joint head + arm fit from floor-board and gripper-plate views (owns `arm-offsets-fit`; design D1, D4, D8, D12, D14).

- New `shared/ros2_common/head_rgbd_calibration/arm_offsets_fit.py`:
  - `fit_head_arm(floor_views: list[FloorView], plate_views: list[PlateView], camera_matrix, dist, head_urdf: str, arm_urdf: str, cfg: HeadArmFitConfig, current_offsets: dict[str, float]) -> HeadArmFit`.
  - Plate-view model: plate corner (`board_corners_3d(cfg.plate)`) -> plate pose in `gripper_link` (by default all 6 DOF free, starting at `plate_mount.rpy_init`/`xyz_init`; when `cfg.plate_mount.rpy` is set the rotation is fixed to it and only the translation is free) -> arm chain (`UrdfChain` of so101_arm.urdf, measured joints + offsets) -> `arm_in_robot_base(cfg.base_in_base_link)` -> head chain at the view's measured head angles -> camera. Floor views use the head-chain model of `head_chain_fit`.
  - Free: every D3 head parameter plus `head_mount_joint` x, y, yaw; offsets of shoulder_lift, elbow_flex and wrist_flex; wrist_roll only when `cfg.plate_mount.rpy` is set (otherwise it is fixed at `cfg.wrist_roll_offset_rad`, the zero from the user's recorded servo neutral, and the report says `wrist_roll fixed at the recorded neutral; plate rotation fitted`); plate pose (6 by default, 3 with a CAD rotation); floor spot poses. Fixed: shoulder_pan offset = `cfg.shoulder_pan_offset_rad`, wrist_roll as above, `base_in_base_link`, link lengths, intrinsics.
  - Init: head parameters from `fit_head_chain` on the floor views; offsets from `current_offsets`; plate pose from `cfg.plate_mount.rpy_init`/`xyz_init` refined by solvePnP of the first plate views. Then one Huber `least_squares` over floor + plate corner residuals with every `arm_limits.holdout_every`-th plate view held out; sigma as in the head fit.
  - `HeadArmFit` (frozen): `origins` (all three head joints, x/y/yaw fitted), `offsets: dict[str, float]` (all five arm joints; shoulder_pan and, without a CAD rotation, wrist_roll at their fixed values), `current_offsets`, `params`, `sigma`, `plate_pose` (xyz, rpy), `rms_px`, `holdout_rms_px`, `holdout_tool_mm` (RMS over held-out plate views of the distance between the tool point (`gripper_frame_link` + `cfg.tool_offset_m`) from the fitted model and from that view's solvePnP plate pose mapped through the fitted plate mount), `wrist_roll_fitted: bool`, `failed_limits` (`arm_limits.*` keys plus the head limits on the floor part), `passed`.
  - `offsets_snippet_yaml(fit: HeadArmFit, session: str, date: str) -> str`: a `joint_offsets_rad:` block with the five joints (4 decimals, shoulder_pan unchanged, wrist_roll the fixed value unless fitted) and one comment line naming the date, session, rms and held-out tool error, indented to drop into the client.yml mcp_server `arm:` block.
  - `arm_report_markdown(fit, cfg, session, date) -> str`.
- `tests/head_arm_synthetic.py` gains `plate_scene(truth, arm_poses: list[dict[str, float]], noise_px, seed) -> list[PlateView]`: head aimed at the plate (pan/tilt from the true chain), corners projected through the true head + arm chain, views with < 6 corners dropped; the plate is the 6x5 board (`PLATE_BOARD`).
- New `tests/test_arm_offsets_fit.py`:
  - default (no CAD rotation): truth offsets differ from current by shoulder_lift +0.03, elbow_flex -0.04, wrist_flex +0.05 rad, wrist_roll truth equal to `wrist_roll_offset_rad`, plate rotated 10 deg about each axis from `rpy_init`; head yaw +1.5 deg, x +8 mm, y -6 mm; 36 plate poses spanning wrist_roll +-1.5 rad + the floor scene; 0.3 px noise: each fitted offset within 3 sigma and 0.004 rad, plate rotation within 0.2 deg, head x/y within 2 mm, yaw within 0.2 deg, `holdout_tool_mm` < 3, `passed`, `wrist_roll_fitted` false, the report says wrist_roll is fixed at the recorded neutral;
  - with `plate_mount.rpy` set to the truth rotation and a wrist_roll truth of +0.06 rad: wrist_roll is fitted within 0.004 rad and `wrist_roll_fitted` true;
  - default mode with a 2 deg wrist_roll zero error in the truth: the fit absorbs it into the plate rotation and the tool point moves by less than 1 mm (pins the accuracy consequence stated in the design);
  - shoulder_pan never changes;
  - the snippet parses as YAML (`yaml.safe_load`) to the five values;
  - fewer than `min_views` plate views: `min_views` failed.

#### Acceptance conditions

- a1: arm offsets and head x/y/yaw are recovered from noisy plate and floor views [evidence: test:tests/test_arm_offsets_fit.py::test_recovers_arm_offsets_and_head_yaw]
- a2: by default wrist_roll stays at the recorded neutral and the plate rotation is fitted [evidence: test:tests/test_arm_offsets_fit.py::test_default_fits_plate_rotation_with_wrist_roll_fixed]
- a8: with a CAD plate rotation wrist_roll is fitted [evidence: test:tests/test_arm_offsets_fit.py::test_cad_rotation_frees_wrist_roll]
- a3: shoulder_pan stays the fixed gauge [evidence: test:tests/test_arm_offsets_fit.py::test_shoulder_pan_offset_is_fixed]
- a4: the snippet is valid YAML with the fitted offsets [evidence: test:tests/test_arm_offsets_fit.py::test_offsets_snippet_round_trips]
- a5: too few plate views fail min_views [evidence: test:tests/test_arm_offsets_fit.py::test_too_few_plate_views_fail]
- a6: the head fit tests still pass [evidence: command:uv run --frozen pytest tests/test_head_chain_fit.py -q]
- a7: root lint passes [evidence: command:uv run poe lint]

#### Files

- `shared/ros2_common/head_rgbd_calibration/arm_offsets_fit.py`
- `tests/head_arm_synthetic.py`
- `tests/test_arm_offsets_fit.py`

#### Interfaces

- `arm_offsets_fit.py::fit_head_arm(...) -> HeadArmFit`, `HeadArmFit`, `offsets_snippet_yaml`, `arm_report_markdown` (part `arm-offsets-fit`); `tests/head_arm_synthetic.py::plate_scene`.

#### Expected outcome

The arm joint offsets and the head yaw and position relative to the arm are measured, which is what grasping from head images needs.

#### Steps

- extend the synthetic helper and write the tests (red), implement, verify.
- Run `uv run --frozen pytest tests/test_arm_offsets_fit.py tests/test_head_chain_fit.py -q && uv run poe lint`; expect every test passed and lint clean.

#### Notes

- Design: the epic design.md (`ticket.py path head-and-arm-calibration-fro design`) (decisions D1-D12, review focus).

- 2026-10-20 amended: gate: guided process, reuse the existing 6x5 board, wrist_roll zero from the recorded neutral with the plate rotation fitted, tiers 0-1 without confirm
- 2026-10-22 moved from head-and-arm-calibration-fro; dependencies on that epic dropped (it is this epic's blocker).

### S2: Arm offsets compute apply

- Status: todo
- Files: `nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py`, `nodes/head_rgbd_calibration/head_rgbd_calibration/apply.py`, `nodes/head_rgbd_calibration/head_rgbd_calibration/jobs.py`, `nodes/head_rgbd_calibration/tests/test_compute_arm.py`, `nodes/head_rgbd_calibration/tests/test_apply.py`, `nodes/head_rgbd_calibration/tests/test_jobs.py`
- Verify: `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_compute_arm.py tests/test_apply.py tests/test_jobs.py -q && uv run poe lint`
- Verified head: none

#### Requirement

Moved verbatim on 2026-10-22 from head-and-arm-calibration-fro (user decision: the arm calibration is trusted; the head is calibrated against the arm there). Before running, re-plan this task with `ticket.py amend` against the parts that epic delivers (head_arm sessions and still capture, the sequencer and pose check, the head-from-arm fit and its synthetic scenes, the head-arm HTTP API); task slugs and design decisions named below refer to the old head-and-arm plan in its git history.

Compute and apply of an arm session: the offsets snippet for review, the head chain applied (design D5, D8).

- `jobs.py`: `ComputeJob.start(sid, floor_session: str | None = None)` passes `--floor-session <sid>` to the child; `compute.py main` accepts it.
- `compute.py` for kind `arm`: the floor session is `--floor-session`, else `applied.json` `head_session`; none: `result.json` `passed: false`, `error: "apply a head calibration first"`. Views from `load_floor_views(floor)` and `load_plate_views(sid)`; intrinsics as for head_chain; `fit_head_arm(..., current_offsets=cfg.arm_sequence.joint_offsets_rad)`.
  - Result files: `head_chain.json` (all three joints, x/y/yaw measured), `tof_joint.json` (recomputed as for head_chain), `head.urdf`, `joint_offsets_snippet.yaml` (`offsets_snippet_yaml`), `report.md` (`arm_report_markdown`), `result.json` `{id, session, kind: "arm", floor_session, created, passed, failed_limits, rms_px, holdout_rms_px, holdout_tool_mm, wrist_roll_fitted, offsets: {joint: {value, sigma, current}}, head: {name: {value, sigma, urdf}}, limits}`.
- `apply.py` for kind `arm`: applies `head_chain.json` and `tof_joint.json` only (the snippet never goes to `applied/`), `applied.json` with `kind: "arm"` and `head_session` = the floor session. `download_zip` adds `joint_offsets_snippet.yaml`.
- New `tests/test_compute_arm.py` (synthetic floor + plate sessions): passing result with every file and the snippet; no head applied and no floor session -> error; `--floor-session` wins over `head_session`; failed limits -> only report and result. `tests/test_apply.py`: arm apply writes the chain, never the snippet, `head_session` = floor session; zip has the snippet. `tests/test_jobs.py`: the child command carries `--floor-session`.

#### Acceptance conditions

- a1: an arm session computes a result with the snippet and the full head chain [evidence: test:nodes/head_rgbd_calibration/tests/test_compute_arm.py::test_arm_result_files_and_snippet]
- a2: without an applied head chain or floor session the result is an error [evidence: test:nodes/head_rgbd_calibration/tests/test_compute_arm.py::test_arm_compute_needs_head_calibration]
- a3: applying an arm result applies the head chain and never the snippet [evidence: test:nodes/head_rgbd_calibration/tests/test_apply.py::test_arm_apply_writes_chain_not_snippet]
- a4: the compute child receives the floor session [evidence: test:nodes/head_rgbd_calibration/tests/test_jobs.py::test_child_gets_floor_session]
- a5: compute, apply and jobs tests pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_compute.py tests/test_compute_head_chain.py tests/test_compute_arm.py tests/test_apply.py tests/test_jobs.py -q]
- a6: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run poe lint]

#### Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/apply.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/jobs.py`
- `nodes/head_rgbd_calibration/tests/test_compute_arm.py`
- `nodes/head_rgbd_calibration/tests/test_apply.py`
- `nodes/head_rgbd_calibration/tests/test_jobs.py`

#### Interfaces

- `ComputeJob.start(sid, floor_session=None)`, child arg `--floor-session`; result files add `joint_offsets_snippet.yaml`; `result.json` kind `arm`.

#### Expected outcome

The arm result gives a reviewable joint_offsets_rad snippet and applies the measured head yaw and position.

#### Steps

- Tests first (red), implement, verify.
- Run `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_compute_arm.py tests/test_compute_head_chain.py tests/test_apply.py tests/test_jobs.py -q && uv run poe lint`; expect every test passed and lint clean.

#### Notes

- Design: the epic design.md (`ticket.py path head-and-arm-calibration-fro design`) (decisions D1-D12, review focus).

- 2026-10-22 moved from head-and-arm-calibration-fro; dependencies on that epic dropped (it is this epic's blocker).

### S3: Arm calibrate UI section

- Status: todo
- Files: `nodes/web_ui/frontend/src/calibrate/ArmCalibration.tsx`, `nodes/web_ui/frontend/src/calibrate/ArmCalibration.test.tsx`, `nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx`, `nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx`
- Verify: `cd nodes/web_ui/frontend && npx vitest run src/calibrate/ArmCalibration.test.tsx src/calibrate/CalibratePage.test.tsx && npx tsc --noEmit && npm run lint && npm run build`
- Verified head: none

#### Requirement

Moved verbatim on 2026-10-22 from head-and-arm-calibration-fro (user decision: the arm calibration is trusted; the head is calibrated against the arm there). Before running, re-plan this task with `ticket.py amend` against the parts that epic delivers (head_arm sessions and still capture, the sequencer and pose check, the head-from-arm fit and its synthetic scenes, the head-arm HTTP API); task slugs and design decisions named below refer to the old head-and-arm plan in its git history.

The "Arm calibrate" button and section as a guided process (design D7, D9, D15, D16; approval A3; CLAUDE.md servo safety).

- New `nodes/web_ui/frontend/src/calibrate/ArmCalibration.tsx`, a step-by-step guide:
  - "Arm calibrate" is disabled with the reason "apply a head calibration first" until the applied record's kind is `head_chain` or `arm`.
  - Step 1 (prompt card): "Take the ChArUco board off the floor and clip it to the fixed jaw; keep the area clear. The arm moves to the calibration pose, closes the gripper on the clip and runs the wrist and elbow poses slowly." A "Start" button calls `startArmSequence({allowBigJoints: false})` (tiers 0 and 1; no confirm dialog).
  - Step 2: after tiers 0 and 1, "Allow shoulder moves" opens a confirm dialog naming shoulder_lift and shoulder_pan; only that Confirm calls `startArmSequence({sessionId: <same session>, allowBigJoints: true})`. No other path sends `allowBigJoints: true`.
  - "Stop" is visible whenever a sequence runs; progress shows tier, step/total, captured, skipped, status.
  - Step 3: "Compute" -> result card: offsets table (current, new, change in deg, sigma), head x/y/yaw change, held-out tool error in mm, wrist_roll fitted or fixed at the recorded neutral, pass/fail and failed limits; the snippet in a monospace block with "Copy" (`navigator.clipboard.writeText`) and "Download" (zip); "Apply head part" only for a passed result, with a note that the offsets are committed by hand after review.
- `CalibratePage.tsx`: renders `ArmCalibration` after `HeadCalibration` when `config.rgbd`; the "Arm calibration - deferred" line is removed.
- New `ArmCalibration.test.tsx`: disabled without an applied head chain; Start sends `allowBigJoints: false` without a dialog; `allowBigJoints` true only after the shoulder confirm (never on Start); Copy writes the snippet; Apply disabled on a failed result; Stop calls the route. `CalibratePage.test.tsx`: the deferred line is gone.

#### Acceptance conditions

- a1: Start runs tiers 0 and 1 without a dialog and without shoulder moves [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/ArmCalibration.test.tsx -t "start runs wrist and elbow tiers"]
- a2: shoulder moves are sent only after the shoulder confirm [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/ArmCalibration.test.tsx -t "allow big joints only after the shoulder confirm"]
- a3: Arm calibrate is disabled until a head chain is applied [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/ArmCalibration.test.tsx -t "disabled without an applied head chain"]
- a4: Copy writes the offsets snippet [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/ArmCalibration.test.tsx -t "copy writes the snippet"]
- a5: the deferred placeholder line is gone [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/CalibratePage.test.tsx -t "has no deferred arm line"]
- a6: type check, lint and build pass [evidence: command:cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint && npm run build]

#### Files

- `nodes/web_ui/frontend/src/calibrate/ArmCalibration.tsx`
- `nodes/web_ui/frontend/src/calibrate/ArmCalibration.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx`

#### Interfaces

- `ArmCalibration.tsx::ArmCalibration` rendered by `CalibratePage` after `HeadCalibration`.

#### Expected outcome

The user calibrates the arm from a browser, confirming every arm motion tier, and gets the offsets to review.

#### Steps

- Tests first (red), implement, verify.
- Run `cd nodes/web_ui/frontend && npx vitest run src/calibrate/ArmCalibration.test.tsx src/calibrate/CalibratePage.test.tsx && npx tsc --noEmit && npm run lint && npm run build`; expect every test passed and lint clean.

#### Notes

- Design: the epic design.md (`ticket.py path head-and-arm-calibration-fro design`) (decisions D1-D12, review focus).

- 2026-10-20 amended: builder check: head tools need a lease, three arm tiers with confirms, gripper set once, mcp-token via supplementary_groups, optional drive check

- 2026-10-20 amended: gate: guided process, reuse the existing 6x5 board, wrist_roll zero from the recorded neutral with the plate rotation fitted, tiers 0-1 without confirm
- 2026-10-22 moved from head-and-arm-calibration-fro; dependencies on that epic dropped (it is this epic's blocker).

## Files to delete

none recorded in the epic format

## Final verify

none recorded in the epic format

## After merge

none
