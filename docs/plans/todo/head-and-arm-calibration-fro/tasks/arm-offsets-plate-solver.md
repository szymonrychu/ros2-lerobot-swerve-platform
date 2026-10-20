---
title: Arm offsets plate solver
status: todo
repo: ros2-lerobot-sverve-platform
value: "The arm joint offsets and the head yaw and position relative to the arm are measured, which is what grasping from head images needs."
files: [shared/ros2_common/head_rgbd_calibration/arm_offsets_fit.py, tests/head_arm_synthetic.py, tests/test_arm_offsets_fit.py]
verify: "uv run --frozen pytest tests/test_arm_offsets_fit.py tests/test_head_chain_fit.py -q && uv run poe lint"
depends_on: [head-chain-board-solver]
owns: [arm-offsets-fit]
uses: [head-chain-fit, head-arm-synthetic-scenes, urdf-chain-fk, head-arm-calib-config, head-arm-view-types]
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

The joint head + arm fit from floor-board and gripper-plate views (owns `arm-offsets-fit`; design D1, D4, D8, D12).

- New `shared/ros2_common/head_rgbd_calibration/arm_offsets_fit.py`:
  - `fit_head_arm(floor_views: list[FloorView], plate_views: list[PlateView], camera_matrix, dist, head_urdf: str, arm_urdf: str, cfg: HeadArmFitConfig, current_offsets: dict[str, float]) -> HeadArmFit`.
  - Plate-view model: plate corner (`board_corners_3d(cfg.plate)`) -> plate pose in `gripper_link` (rotation = `cfg.plate_mount.rpy`, fixed, when set; translation free) -> arm chain (`UrdfChain` of so101_arm.urdf, measured joints + offsets) -> `arm_in_robot_base(cfg.base_in_base_link)` -> head chain at the view's measured head angles -> camera. Floor views use the head-chain model of `head_chain_fit`.
  - Free: every D3 head parameter plus `head_mount_joint` x, y, yaw; offsets of shoulder_lift, elbow_flex, wrist_flex and wrist_roll (wrist_roll only when `cfg.plate_mount.rpy` is set; otherwise it stays at `current_offsets["wrist_roll"]` and the report says `wrist_roll not fitted: no plate CAD rotation`); plate translation (3); floor spot poses. Fixed: shoulder_pan offset = `cfg.shoulder_pan_offset_rad`, `base_in_base_link`, link lengths, intrinsics.
  - Init: head parameters from `fit_head_chain` on the floor views; offsets from `current_offsets`; plate translation from `cfg.plate_mount.xyz_init`. Then one Huber `least_squares` over floor + plate corner residuals with every `arm_limits.holdout_every`-th plate view held out; sigma as in the head fit.
  - `HeadArmFit` (frozen): `origins` (all three head joints, x/y/yaw fitted), `offsets: dict[str, float]` (all five arm joints, shoulder_pan = the fixed value), `current_offsets`, `params`, `sigma`, `plate_xyz`, `rms_px`, `holdout_rms_px`, `holdout_tool_mm` (RMS over held-out plate views of the distance between the tool point (`gripper_frame_link` + `cfg.tool_offset_m`) from the fitted model and from that view's solvePnP plate pose mapped through the fitted plate mount), `wrist_roll_fitted: bool`, `failed_limits` (`arm_limits.*` keys plus the head limits on the floor part), `passed`.
  - `offsets_snippet_yaml(fit: HeadArmFit, session: str, date: str) -> str`: a `joint_offsets_rad:` block with the five joints (4 decimals, shoulder_pan unchanged) and one comment line naming the date, session, rms and held-out tool error, indented to drop into the client.yml mcp_server `arm:` block.
  - `arm_report_markdown(fit, cfg, session, date) -> str`.
- `tests/head_arm_synthetic.py` gains `plate_scene(truth, arm_poses: list[dict[str, float]], noise_px, seed) -> list[PlateView]`: head aimed at the plate (pan/tilt from the true chain), corners projected through the true head + arm chain, views with < 6 corners dropped.
- New `tests/test_arm_offsets_fit.py`:
  - truth offsets differ from current by shoulder_lift +0.03, elbow_flex -0.04, wrist_flex +0.05, wrist_roll +0.06 rad; head yaw +1.5 deg, x +8 mm, y -6 mm; 36 plate poses spanning wrist_roll +-1.5 rad + the floor scene; 0.3 px noise: each offset within 3 sigma and 0.004 rad, head x/y within 2 mm, yaw within 0.2 deg, `holdout_tool_mm` < 3, `passed`;
  - without `plate_mount.rpy`: wrist_roll equals current, `wrist_roll_fitted` false, the report says so;
  - shoulder_pan never changes;
  - the snippet parses as YAML (`yaml.safe_load`) to the five fitted values;
  - fewer than `min_views` plate views: `min_views` failed.

## Acceptance conditions

- a1: arm offsets and head x/y/yaw are recovered from noisy plate and floor views [evidence: test:tests/test_arm_offsets_fit.py::test_recovers_arm_offsets_and_head_yaw]
- a2: without a plate CAD rotation wrist_roll is not fitted and the report says so [evidence: test:tests/test_arm_offsets_fit.py::test_wrist_roll_not_fitted_without_plate_rotation]
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

