---
title: Arm pose safety check
status: done
repo: ros2-lerobot-sverve-platform
value: "No arm pose or transit with the board on the gripper is commanded unless FK shows the tool, the board and the arm links clear of the floor, the body and the head."
files: [shared/ros2_common/head_rgbd_calibration/arm_pose_check.py, tests/test_arm_pose_check.py]
verify: "uv run --frozen pytest tests/test_arm_pose_check.py -q && uv run poe lint"
depends_on: [calib-config-and-view-types]
owns: [arm-pose-check]
uses: [head-arm-calib-config, urdf-chain-fk]
model: opus
estimate: 75
actual: ""
review: full
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/drivable-ground-tof-filterin--arm-pose-safety-check
branch: drivable-ground-tof-filterin--arm-pose-safety-check
verified_head: 8efd29ed2722218b63b8129374d765f2266fcc7c
run: drivable-ground-tof-filterin
provides: [obstacle-set-distances, head-envelope, board-points-fk, arm-pose-check]
acs_for_audit: []
process_gaps: ["tests/README.md has no entry for tests/test_arm_pose_check.py (CLAUDE.md requires one); the implementer listed it as an epic task."]
---

# Arm pose safety check

## Requirement

An FK check of arm poses and joint paths with the board on the gripper (owns `arm-pose-check`; design D38, D39;
approval H2; CLAUDE.md servo safety; MEMORY feedback_arm_motion_safety). Pure numpy over `UrdfChain`, no ROS.

- New `shared/ros2_common/head_rgbd_calibration/arm_pose_check.py`:
  - Config models (`StrictModel`): `BoxObstacle(name, center: [x, y, z], size: [x, y, z])`, `CylinderObstacle(name, x,
    y, radius, z_min, z_max)` and `BodyConfig(boxes, cylinders)`, defaults equal to the client.yml mcp_server `body`
    block (frame_left, frame_right, crossbar boxes; wheel_fl/fr/rl/rr and lidar cylinders, the values of that block);
    `PoseCheckConfig`: `body: BodyConfig`, `margin_m` 0.03, `min_tool_height_m` 0.10, `min_board_height_m` 0.05,
    `link_radius_m` 0.025, `head_radius_m` (the builder sets it from the head URDF so the test below holds),
    `head_height_m` (above the mount z, same rule), `roll_min_tool_height_m` 0.15, `max_roll_gripper_rad` 0.8 (the
    mcp_server roll guard, gripper at most half open), `stretched_shoulder_lift_max_rad` 1.85, `path_step_rad` 0.05,
    `board_grid` 5 (points per board edge).
  - `head_envelope(head_urdf: str, cfg: PoseCheckConfig) -> CylinderObstacle`: a vertical cylinder about the pan axis
    (x, y of `head_mount_joint` in the given URDF, so the applied chain is used) from the crossbar top to the mount z
    plus `head_height_m`, radius `head_radius_m`, grown when needed so it contains the head visuals at every pan/tilt
    inside that URDF's `head_pan`/`head_tilt` joint limits (sampled at the limits and every 15 deg between), so a
    widened tilt range is covered automatically.
  - `PoseChecker(arm_urdf: str, head_urdf: str, fit: HeadArmFitConfig, cfg: PoseCheckConfig)` with
    `check_pose(joints: dict[str, float], gripper_rad: float, rolling: bool) -> list[str]` and
    `check_path(start: dict[str, float], target: dict[str, float], gripper_rad: float) -> list[str]`; an empty list
    means safe, every entry names the rule, the part and the clearance in mm.
  - Pose rules (FK with `fit.joint_offsets_rad`, `urdf = measured + offset`, mapped by `arm_in_robot_base(
    fit.base_in_base_link)`): every arm joint inside the arm URDF limits; tool point (`gripper_frame_link` origin) >=
    `min_tool_height_m` above the floor (z 0); board outline and grid points (`board_outline(fit.board,
    fit.board_mount.outline_margin_m)` plus a `board_grid` x `board_grid` grid, placed by `fit.board_mount`
    xyz_init/rpy_init in `gripper_link`) >= `min_board_height_m` above the floor and >= `margin_m` from every body
    obstacle and the head envelope; arm link capsules (upper arm: `upper_arm_link` -> `lower_arm_link` origins,
    forearm: `lower_arm_link` -> `wrist_link`, wrist: `wrist_link` -> `gripper_frame_link`, radius `link_radius_m`)
    >= `margin_m` from every body obstacle except the crossbar for the upper arm (the shoulder stands on it, as in
    mcp_server `self_collision.py`) and from the head envelope; board points >= `margin_m` from the upper arm and
    forearm capsules; `rolling` (wrist_roll changes on this move) requires `gripper_rad <= max_roll_gripper_rad` and the
    tool point >= `roll_min_tool_height_m`; `shoulder_lift > stretched_shoulder_lift_max_rad` with `elbow_flex < 0`
    (negative elbow stretches the arm) refused.
  - `check_path`: joint-linear samples from start to target with no joint step over `path_step_rad`, `check_pose` on
    each (`rolling` when wrist_roll differs between start and target); entries carry the sample fraction.
- New `tests/test_arm_pose_check.py`:
  - the `BodyConfig` defaults equal the client.yml mcp_server `body` block (read from the YAML);
  - the head envelope contains every visual of `head_post`, `head_pan_base`, `head_pan_link`, `head_link`, the camera
    and ToF links of `nodes/web_ui/urdf/head.urdf` at pan/tilt sampled across the limits read from that URDF (never
    hard-coded), also for a fixture copy with the tilt limit set to [-0.5236, 1.0472] (the widened range), and moves
    with an applied `head_mount_joint` override;
  - a pose with the tool at 0.05 m is refused naming `min_tool_height_m`; a board corner inside the lidar cylinder, a
    forearm through the head envelope, a board point on the forearm each refused with their rule;
  - a roll move with the gripper at 1.2 rad or the tool at 0.12 m refused; the same roll at 0.5 rad and 0.20 m passes;
  - shoulder_lift 1.9 with elbow_flex -0.2 refused; a joint outside the URDF limits refused;
  - `check_path` catches a mid-path floor contact that both end poses avoid; a clear path returns `[]`.

## Acceptance conditions

- a1: the body obstacles equal the mcp_server body config [evidence: test:tests/test_arm_pose_check.py::test_body_defaults_equal_mcp_server_body]
- a2: the head envelope contains the head at every pan/tilt [evidence: test:tests/test_arm_pose_check.py::test_head_envelope_contains_head_visuals]
- a3: floor, body, head and board self-collision violations are refused with their rule [evidence: test:tests/test_arm_pose_check.py::test_pose_violations_are_named]
- a4: wrist roll only half open and lifted [evidence: test:tests/test_arm_pose_check.py::test_roll_rules]
- a5: the stretched-arm rule and joint limits hold [evidence: test:tests/test_arm_pose_check.py::test_stretched_arm_and_limits]
- a6: a path check catches a mid-path contact [evidence: test:tests/test_arm_pose_check.py::test_path_check_catches_mid_path_contact]
- a7: PoseChecker exists [evidence: symbol:shared/ros2_common/head_rgbd_calibration/arm_pose_check.py::PoseChecker]
- a8: root lint passes [evidence: command:uv run poe lint]

## Files

- `shared/ros2_common/head_rgbd_calibration/arm_pose_check.py`
- `tests/test_arm_pose_check.py`

## Interfaces

- `arm_pose_check.py::PoseCheckConfig`, `BodyConfig`, `head_envelope`, `PoseChecker.check_pose`,
  `PoseChecker.check_path` (part `arm-pose-check`).

## Expected outcome

No arm pose or transit with the board on the gripper is commanded unless FK shows the tool, the board and the arm links
clear of the floor, the body and the head.

## Steps

- Write `tests/test_arm_pose_check.py` first; run it: red on the missing module.
- Implement; run `uv run --frozen pytest tests/test_arm_pose_check.py -q && uv run poe lint`; expect every test passed
  and lint clean.

## Notes

- mcp_server's checker (`nodes/mcp_server/mcp_server/self_collision.py`, C++ kernel) cannot be imported here; mirror
  its geometry choices (capsules, the crossbar exclusion) without copying the kernel.
- The pose values themselves are arm-board-pose-set's; this task only builds the check.
- Design: `ticket.py path drivable-ground-tof-filterin design` (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).
