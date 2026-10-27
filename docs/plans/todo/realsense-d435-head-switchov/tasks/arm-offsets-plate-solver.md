---
title: Arm offsets plate solver
status: todo
repo: ros2-lerobot-sverve-platform
value: "The arm joint offsets and tool point are measured against the head D435, which is what grasping from head images needs."
files: [shared/ros2_common/head_rgbd_calibration/arm_offsets_fit.py, shared/ros2_common/head_rgbd_calibration/config.py, tests/head_arm_synthetic.py, tests/test_arm_offsets_fit.py, tests/README.md]
verify: "uv run pytest tests/test_arm_offsets_fit.py tests/test_head_arm_fit.py tests/test_head_floor_fit.py -q && uv run poe lint"
depends_on: [calibration-library-d435-fra]
model: opus
review: ""
verified_head: ""
points: 8
expected_minutes: 91
expected_tokens: 6000000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Arm offsets plate solver

## Requirement

Goal G6: "The arm joint offset / tool point solve (2026-10-10, gripper camera) is re-done from the head D435 watching a gripper-held ChArUco board." User decision 3 (absorbs todo epic arm-joint-offsets-from-the-g). Design D9, Q2 (recommended: fit tool_offset_m from the board pose and the clip CAD offset; accepted only if held-out tool error < 5 mm). Prior-art fix-as-ac: accept on held-out poses at several head pan/tilt settings (reprojection px plus FK tool mm), never on fit residual alone; rigid ChArUco board with rotation diversity.

Re-plan of `arm-joint-offsets-from-the-g/arm-offsets-plate-solver` (its requirement below, amended for the D435: camera = D435 colour `d435_color_optical_frame` with factory intrinsics, head chain with `d435_joint`, no tof_joint):
- New `shared/ros2_common/head_rgbd_calibration/arm_offsets_fit.py`:
  - `fit_head_arm(floor_views, plate_views, camera_matrix, dist, head_urdf: str, arm_urdf: str, cfg: HeadArmFitConfig, current_offsets: dict[str, float]) -> HeadArmFit`.
  - Plate-view model: plate corner (`board_corners_3d(cfg.plate)`) -> plate pose in `gripper_link` (6 DOF free from `plate_mount.rpy_init`/`xyz_init`, or rotation fixed to `cfg.plate_mount.rpy` when set) -> arm chain (`UrdfChain` of so101_arm.urdf, measured joints + offsets) -> `base_in_base_link` -> head chain at the view's measured head angles -> `d435_color_optical_frame`. Floor views use the head-chain floor model on `d435_depth_optical_frame`.
  - Free: the head chain parameters plus `head_mount_joint` x, y, yaw; offsets of shoulder_lift, elbow_flex, wrist_flex; wrist_roll only with a CAD plate rotation (else fixed at `cfg.wrist_roll_offset_rad`, report says "wrist_roll fixed at the recorded neutral; plate rotation fitted"); the plate pose; floor spot poses. Fixed: shoulder_pan offset (gauge), base_in_base_link, link lengths, intrinsics.
  - Tool point (Q2): when `cfg.plate_mount.tool_in_board_m` is set, `tool_offset_m` = fitted plate pose composed with that CAD board-to-jaw-tip offset, expressed in `gripper_frame_link`. `tool_fitted` is true only when `holdout_tool_mm` < `arm_limits.max_tool_holdout_mm` (5.0); otherwise the current tool_offset_m is reported unchanged with the reason.
  - Init from `fit_head_chain` (floor) and solvePnP of the first plate views; one Huber `least_squares`; every `arm_limits.holdout_every`-th plate view held out, and the held-out set covers at least 3 distinct head pan/tilt settings (else `holdout_coverage` fails).
  - `HeadArmFit` (frozen): origins, offsets (all five), current_offsets, params, sigma, plate_pose, tool_offset_m, tool_fitted, rms_px, holdout_rms_px, holdout_tool_mm, wrist_roll_fitted, failed_limits, passed.
  - `offsets_snippet_yaml(fit, session, date) -> str` (a `joint_offsets_rad:` block, 4 decimals, and `tool_offset_m:` when fitted, one comment line with date, session, rms and held-out tool error, indented for the client.yml mcp_server `arm:` block); `arm_report_markdown(fit, cfg, session, date) -> str`.
- `config.py` `HeadArmFitConfig`: `plate_mount.tool_in_board_m: list[float] | None`, `arm_limits.max_tool_holdout_mm: 5.0`.
- `tests/head_arm_synthetic.py` gains `plate_scene(truth, arm_poses, noise_px, seed) -> list[PlateView]` on the D435 frames (views with < 6 corners dropped; 6x5 board).
- New `tests/test_arm_offsets_fit.py`: the original cases (offsets within 3 sigma and 0.004 rad, plate rotation within 0.2 deg, head x/y within 2 mm, yaw within 0.2 deg, holdout_tool_mm < 3, wrist_roll fixed by default, fitted with a CAD rotation, 2 deg wrist_roll zero error absorbed with < 1 mm tool shift, shoulder_pan never changes, the snippet parses with yaml.safe_load, too few views fail min_views), plus: the tool point recovered within 2 mm with tool_in_board_m; tool not fitted above the held-out limit; held-out views from fewer than 3 head settings fail holdout_coverage.

## Acceptance conditions

- AC1: arm offsets and head x/y/yaw are recovered from noisy D435 plate and floor views [evidence: test:tests/test_arm_offsets_fit.py::test_recovers_arm_offsets_and_head_yaw]
- AC2: by default wrist_roll stays at the recorded neutral and the plate rotation is fitted [evidence: test:tests/test_arm_offsets_fit.py::test_default_fits_plate_rotation_with_wrist_roll_fixed]
- AC3: with a CAD plate rotation wrist_roll is fitted [evidence: test:tests/test_arm_offsets_fit.py::test_cad_rotation_frees_wrist_roll]
- AC4: the tool point is fitted from the board pose and accepted only under the held-out limit [evidence: test:tests/test_arm_offsets_fit.py::test_tool_point_fitted_and_gated]
- AC5: held-out views must span at least 3 head settings [evidence: test:tests/test_arm_offsets_fit.py::test_holdout_needs_three_head_settings]
- AC6: shoulder_pan stays the fixed gauge and the snippet round-trips [evidence: test:tests/test_arm_offsets_fit.py::test_offsets_snippet_round_trips]
- AC7: too few plate views fail min_views [evidence: test:tests/test_arm_offsets_fit.py::test_too_few_plate_views_fail]
- AC8: head fit tests and root lint pass [evidence: command:uv run pytest tests/test_head_arm_fit.py tests/test_head_floor_fit.py -q && uv run poe lint]

## Files

- `shared/ros2_common/head_rgbd_calibration/arm_offsets_fit.py`
- `shared/ros2_common/head_rgbd_calibration/config.py`
- `tests/head_arm_synthetic.py`
- `tests/test_arm_offsets_fit.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
