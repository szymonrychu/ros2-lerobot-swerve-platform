---
title: Arm offsets plate solver
status: todo
repo: ros2-lerobot-sverve-platform
value: "The arm joint offsets are measured against the calibrated head, which is what grasping from head images needs."
files: [shared/ros2_common/head_rgbd_calibration/arm_offsets_fit.py, tests/head_arm_synthetic.py, tests/test_arm_offsets_fit.py]
verify: "uv run --frozen pytest tests/test_arm_offsets_fit.py -q && uv run poe lint"
depends_on: []
owns: [arm-offsets-fit]
uses: []
model: opus
estimate: 90
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Arm offsets plate solver

## Requirement

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

## Acceptance conditions

- a1: arm offsets and head x/y/yaw are recovered from noisy plate and floor views [evidence: test:tests/test_arm_offsets_fit.py::test_recovers_arm_offsets_and_head_yaw]
- a2: by default wrist_roll stays at the recorded neutral and the plate rotation is fitted [evidence: test:tests/test_arm_offsets_fit.py::test_default_fits_plate_rotation_with_wrist_roll_fixed]
- a8: with a CAD plate rotation wrist_roll is fitted [evidence: test:tests/test_arm_offsets_fit.py::test_cad_rotation_frees_wrist_roll]
- a3: shoulder_pan stays the fixed gauge [evidence: test:tests/test_arm_offsets_fit.py::test_shoulder_pan_offset_is_fixed]
- a4: the snippet is valid YAML with the fitted offsets [evidence: test:tests/test_arm_offsets_fit.py::test_offsets_snippet_round_trips]
- a5: too few plate views fail min_views [evidence: test:tests/test_arm_offsets_fit.py::test_too_few_plate_views_fail]
- a6: the head fit tests still pass [evidence: command:uv run --frozen pytest tests/test_head_chain_fit.py -q]
- a7: root lint passes [evidence: command:uv run poe lint]

## Files

- `shared/ros2_common/head_rgbd_calibration/arm_offsets_fit.py`
- `tests/head_arm_synthetic.py`
- `tests/test_arm_offsets_fit.py`

## Interfaces

- `arm_offsets_fit.py::fit_head_arm(...) -> HeadArmFit`, `HeadArmFit`, `offsets_snippet_yaml`, `arm_report_markdown` (part `arm-offsets-fit`); `tests/head_arm_synthetic.py::plate_scene`.

## Expected outcome

The arm joint offsets and the head yaw and position relative to the arm are measured, which is what grasping from head images needs.

## Steps

- extend the synthetic helper and write the tests (red), implement, verify.
- Run `uv run --frozen pytest tests/test_arm_offsets_fit.py tests/test_head_chain_fit.py -q && uv run poe lint`; expect every test passed and lint clean.

## Notes

- Design: the epic design.md (`ticket.py path head-and-arm-calibration-fro design`) (decisions D1-D12, review focus).

- 2026-10-20 amended: gate: guided process, reuse the existing 6x5 board, wrist_roll zero from the recorded neutral with the plate rotation fitted, tiers 0-1 without confirm
- 2026-10-22 moved from head-and-arm-calibration-fro; dependencies on that epic dropped (it is this epic's blocker).
