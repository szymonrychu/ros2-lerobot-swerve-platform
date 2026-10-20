---
title: Calib config and view types
status: todo
repo: ros2-lerobot-sverve-platform
value: "One validated place for the plate board, fit limits and view types, so the plate and floor board can never be confused."
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

Foundation config and view types for the head and arm fits (design D4, D8, D13). Every later fit, store and capture task reads these.

- `shared/ros2_common/head_rgbd_calibration/config.py`:
  - `PLATE_BOARD = BoardConfig()`: the gripper plate is the existing 6x5 ChArUco board (40 mm squares, 30 mm markers, DICT_5X5_100, ids 0-14; `models/charuco-6x5-40mm.pdf`/`.3mf`, `scripts/markers_generate.yaml` `charuco:`). Floor and gripper captures use the same physical board in separate sessions (`head_chain` and `arm`), so no id split is needed.
  - `PlateMountConfig(StrictModel)`: `rpy: tuple[float, float, float] | None = None` (optional CAD rotation of the board frame in `gripper_link`, fixed-axis xyz rad; when set the plate rotation is fixed and the wrist_roll offset is fitted, design D14), `rpy_init: tuple[float, float, float] = (0.0, 0.0, 0.0)` (initial plate rotation for the fit), `xyz_init: tuple[float, float, float] = (0.0, 0.0, 0.0)` (initial plate translation in gripper_link, m).
  - `HeadFitLimits(StrictModel)`: `rms_px` 1.0, `holdout_rms_px` 1.5, `max_sigma_mm` 3.0 (translation parameters), `max_sigma_deg` 0.3 (angle parameters), `min_views` 24, `min_spots` 3, `holdout_every` 5 (ge 2).
  - `ArmFitLimits(StrictModel)`: `rms_px` 1.5, `holdout_rms_px` 2.0, `max_sigma_offset_deg` 0.5, `holdout_tool_mm` 5.0, `min_views` 20, `holdout_every` 5.
  - `BaseInBaseLink(StrictModel)`: `x` 0.0592, `y` -0.05, `z` 0.100, `yaw` 0.0 (the client.yml `arm.base_in_base_link` gauge, design D1).
  - `HeadArmFitConfig(StrictModel)`: `board: BoardConfig = BoardConfig()`, `plate: BoardConfig = PLATE_BOARD`, `plate_mount: PlateMountConfig`, `head_limits: HeadFitLimits`, `arm_limits: ArmFitLimits`, `huber_f_scale_px: float = 1.0` (gt 0), `board_z_m: float = 0.004`, `shoulder_pan_offset_rad: float = 0.0182`, `wrist_roll_offset_rad: float = 0.0` (the fixed wrist_roll zero when no plate CAD rotation is given: 0 means the servo neutral the user recorded with the jaws aligned by eye is the URDF zero), `base_in_base_link: BaseInBaseLink`, `tool_offset_m: tuple[float, float, float] = (0.0010, -0.0056, -0.0014)`. All unknown keys rejected.
- `shared/ros2_common/head_rgbd_calibration/charuco.py`: new `board_corners_3d(cfg: BoardConfig) -> np.ndarray` (N, 3) ChArUco corner positions in the board frame (m, z = 0), row i = corner id i, consistent with `make_board`.
- New `shared/ros2_common/head_rgbd_calibration/views.py`: frozen dataclasses `FloorView(view_id: str, spot: int, joints: dict[str, float], detection: Detection)` and `PlateView(view_id: str, joints: dict[str, float], detection: Detection)`. `joints` holds measured rad by joint name (`head_pan`, `head_tilt`; plus `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll` for PlateView); never filled with defaults.
- New `tests/test_head_arm_calib_config.py`: defaults equal the values above; `PLATE_BOARD` equals the `charuco:` spec of `scripts/markers_generate.yaml` (squares, square_mm, marker_mm, dictionary); unknown keys rejected; `board_corners_3d` shape `(20, 3)`, spacing 0.040 m, and its corners projected with a known pose match the corners `detect` finds on a rendered board image within 0.5 px.
- Existing core tests pass with no assertion changed; root lint passes.

## Acceptance conditions

- a1: the plate board equals the existing 6x5 board spec of the marker generator [evidence: test:tests/test_head_arm_calib_config.py::test_plate_board_is_the_existing_charuco_board]
- a2: the fit config defaults equal the design values [evidence: test:tests/test_head_arm_calib_config.py::test_head_arm_fit_config_defaults]
- a3: board_corners_3d matches the detected corners of a rendered board [evidence: test:tests/test_head_arm_calib_config.py::test_board_corners_3d_match_detection]
- a4: FloorView and PlateView exist [evidence: symbol:shared/ros2_common/head_rgbd_calibration/views.py::PlateView]
- a5: existing core tests pass unchanged [evidence: command:uv run --frozen pytest tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_extrinsics.py tests/test_head_rgbd_calib_poses.py -q]
- a6: root lint passes [evidence: command:uv run poe lint]

## Files

- `shared/ros2_common/head_rgbd_calibration/config.py`
- `shared/ros2_common/head_rgbd_calibration/charuco.py`
- `shared/ros2_common/head_rgbd_calibration/views.py`
- `tests/test_head_arm_calib_config.py`

## Interfaces

- `config.py::PLATE_BOARD`, `PlateMountConfig`, `HeadFitLimits`, `ArmFitLimits`, `BaseInBaseLink`, `HeadArmFitConfig` (part `head-arm-calib-config`).
- `charuco.py::board_corners_3d(cfg) -> np.ndarray`; `views.py::FloorView`, `PlateView` (part `head-arm-view-types`).

## Expected outcome

One validated place for the plate board, fit limits and view types, so the plate and floor board can never be confused.

## Steps

- write the tests first and run them (red: missing names); implement; run the verify command (green).
- Run `uv run --frozen pytest tests/test_head_arm_calib_config.py tests/test_head_rgbd_calib_core.py -q && uv run poe lint`; expect every test passed and lint clean.

## Notes

- the existing `BoardConfig` docstring says it must equal `scripts/markers_generate.yaml`; the plate equivalent is pinned by the gripper-plate-marker task.
- Design: the epic design.md (`ticket.py path head-and-arm-calibration-fro design`) (decisions D1-D12, review focus).

- 2026-10-20 amended: gate: guided process, reuse the existing 6x5 board, wrist_roll zero from the recorded neutral with the plate rotation fitted, tiers 0-1 without confirm
