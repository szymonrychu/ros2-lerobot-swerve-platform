---
title: Calib config and view types
status: todo
repo: ros2-lerobot-sverve-platform
value: "One validated place for the board mount, the fit limits, the trusted arm gauge and the view type, so every later fit, check and store task reads the same values."
files: [shared/ros2_common/head_rgbd_calibration/config.py, shared/ros2_common/head_rgbd_calibration/charuco.py, shared/ros2_common/head_rgbd_calibration/views.py, tests/test_head_arm_calib_config.py]
verify: "uv run --frozen pytest tests/test_head_arm_calib_config.py tests/test_head_rgbd_calib_core.py -q && uv run poe lint"
depends_on: []
owns: [head-arm-calib-config, head-arm-view-types]
uses: []
model: ""
estimate: 45
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Calib config and view types

## Requirement

Foundation config and view type for the head-from-arm fit (design D32, D34, D35, D45; approval H1). Every later fit, pose
check, store and capture task reads these.

- `shared/ros2_common/head_rgbd_calibration/config.py` (pydantic `StrictModel`, unknown keys rejected):
  - `BoardMountConfig`: `xyz_init: tuple[float, float, float] = (0.0, 0.0, 0.0)` (board frame origin in
    `gripper_link`, m; the hardware step records the real value), `rpy_init: tuple[float, float, float] = (0.0, 0.0,
    0.0)` (fixed-axis xyz, rad), `outline_margin_m: float = 0.01` (ge 0; plate margin beyond the outer squares, used by
    the pose check).
  - `HeadArmFitLimits`: `rms_px` 1.0, `holdout_rms_px` 1.5, `holdout_board_mm` 5.0, `max_sigma_mm` 2.0,
    `max_sigma_deg` 0.2, `min_views` 40, `min_arm_poses` 16, `min_pan_span_deg` 30.0, `min_tilt_span_deg` 10.0,
    `holdout_every` 5 (ge 2), `max_floor_delta_mm` 10.0, `max_floor_delta_deg` 1.5, `max_start_delta_mm` 30.0,
    `max_start_delta_deg` 5.0 (all gt 0).
  - `BaseInBaseLink`: `x` 0.0592, `y` -0.05, `z` 0.100, `yaw` 0.0 (the client.yml mcp_server `arm.base_in_base_link`).
  - `HeadArmFitConfig`: `board: BoardConfig = BoardConfig()` (the existing 6x5 board, design D45), `board_mount:
    BoardMountConfig`, `limits: HeadArmFitLimits`, `huber_f_scale_px: float = 1.0` (gt 0), `base_in_base_link:
    BaseInBaseLink`, `joint_offsets_rad: dict[str, float]` defaulting to the client.yml mcp_server
    `arm.joint_offsets_rad` (shoulder_pan 0.0182, shoulder_lift -0.0345, elbow_flex -0.1070, wrist_flex 0.0988,
    wrist_roll -0.0710) and validated to name exactly those five joints (error names the missing or extra joint),
    `floor_prior_weight: float = 0.0` (ge 0; 0 means the floor values are only start values and a cross-check, D34).
- `shared/ros2_common/head_rgbd_calibration/charuco.py`: `board_corners_3d(cfg: BoardConfig) -> np.ndarray` (N, 3)
  ChArUco inner corner positions in the board frame (m, z = 0), row i = corner id i, consistent with `make_board`; and
  `board_outline(cfg: BoardConfig, margin_m: float) -> np.ndarray` (4, 3) outer corners of the board rectangle (outer
  squares plus `margin_m`) in the same frame.
- New `shared/ros2_common/head_rgbd_calibration/views.py`: constants `HEAD_JOINTS = ("head_pan", "head_tilt")`,
  `ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")`; frozen dataclass
  `ArmBoardView(view_id: str, arm_pose: int, tier: int, joints: dict[str, float], detection: Detection)` (joints are
  measured rad of all seven joints, never filled with defaults); `require_joints(view: ArmBoardView) -> None` raises
  ValueError naming the view id and every missing joint.
- New `tests/test_head_arm_calib_config.py`: defaults equal the values above; `HeadArmFitConfig().board` equals the
  `charuco:` spec of `scripts/markers_generate.yaml` (squares, square_mm, marker_mm, dictionary); unknown keys and a
  wrong `joint_offsets_rad` joint set rejected; `joint_offsets_rad` default equals the client.yml mcp_server
  `arm.joint_offsets_rad` and `base_in_base_link` equals `arm.base_in_base_link` (read from the YAML); `board_corners_3d`
  shape `(20, 3)`, spacing 0.040 m, and its corners projected with a known pose match the corners `detect` finds on a
  rendered board image within 0.5 px; `board_outline` spans the outer squares plus the margin; `require_joints` names
  the missing joints.
- Existing core tests pass with no assertion changed; root lint passes.

## Acceptance conditions

- a1: the default board equals the existing 6x5 board spec of the marker generator [evidence: test:tests/test_head_arm_calib_config.py::test_board_is_the_existing_charuco_board]
- a2: the fit config defaults equal the design values [evidence: test:tests/test_head_arm_calib_config.py::test_head_arm_fit_config_defaults]
- a3: the arm gauge and offsets default to the client.yml mcp_server values [evidence: test:tests/test_head_arm_calib_config.py::test_arm_gauge_matches_client_yml]
- a4: board_corners_3d matches the detected corners of a rendered board [evidence: test:tests/test_head_arm_calib_config.py::test_board_corners_3d_match_detection]
- a5: require_joints names every missing joint [evidence: test:tests/test_head_arm_calib_config.py::test_require_joints_names_missing]
- a6: ArmBoardView exists [evidence: symbol:shared/ros2_common/head_rgbd_calibration/views.py::ArmBoardView]
- a7: existing core tests pass unchanged [evidence: command:uv run --frozen pytest tests/test_head_rgbd_calib_core.py tests/test_head_floor_fit.py tests/test_head_arm_calib_chain.py -q]
- a8: root lint passes [evidence: command:uv run poe lint]

## Files

- `shared/ros2_common/head_rgbd_calibration/config.py`
- `shared/ros2_common/head_rgbd_calibration/charuco.py`
- `shared/ros2_common/head_rgbd_calibration/views.py`
- `tests/test_head_arm_calib_config.py`

## Interfaces

- `config.py::BoardMountConfig`, `HeadArmFitLimits`, `BaseInBaseLink`, `HeadArmFitConfig` (part `head-arm-calib-config`).
- `charuco.py::board_corners_3d(cfg) -> np.ndarray`, `board_outline(cfg, margin_m) -> np.ndarray`.
- `views.py::ArmBoardView`, `HEAD_JOINTS`, `ARM_JOINTS`, `require_joints` (part `head-arm-view-types`).

## Expected outcome

One validated place for the board mount, the fit limits, the trusted arm gauge and the view type, so every later fit,
check and store task reads the same values.

## Steps

- Write `tests/test_head_arm_calib_config.py` first and run it: red on the missing names.
- Implement the config models, the two charuco helpers and `views.py`; run
  `uv run --frozen pytest tests/test_head_arm_calib_config.py tests/test_head_rgbd_calib_core.py -q && uv run poe lint`;
  expect every test passed and lint clean.

## Notes

- `BoardConfig` already must equal `scripts/markers_generate.yaml`; reuse it, do not add a second board.
- `floor_fit.PARAM_NAMES` (mount_z_m, mount_roll_deg, mount_pitch_deg, tilt_zero_deg, tilt_roll_deg) are the names the
  cross-check compares by; do not rename them.
- Design: `ticket.py path drivable-ground-tof-filterin design` (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).
