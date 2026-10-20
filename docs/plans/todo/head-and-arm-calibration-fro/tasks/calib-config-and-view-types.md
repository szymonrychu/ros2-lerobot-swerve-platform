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

Foundation config and view types for the head and arm fits (design D4, D8). Every later fit, store and capture task reads these.

- `shared/ros2_common/head_rgbd_calibration/config.py`:
  - `BoardConfig.first_marker_id: int = 0` (ge 0). The board's marker ids are `first_marker_id .. first_marker_id + n_markers - 1`; a model validator refuses ids past the dictionary size, naming `first_marker_id`.
  - `PLATE_BOARD = BoardConfig(squares_x=5, squares_y=3, square_mm=18.0, marker_mm=14.0, dictionary="DICT_5X5_100", first_marker_id=15)` (the 6x5 board uses ids 0-14).
  - `PlateMountConfig(StrictModel)`: `rpy: tuple[float, float, float] | None = None` (CAD rotation of the plate board frame in `gripper_link`, fixed-axis xyz rad; None means wrist_roll is not fitted, design D4), `xyz_init: tuple[float, float, float] = (0.0, 0.0, 0.0)` (initial plate translation in gripper_link, m).
  - `HeadFitLimits(StrictModel)`: `rms_px` 1.0, `holdout_rms_px` 1.5, `max_sigma_mm` 3.0 (translation parameters), `max_sigma_deg` 0.3 (angle parameters), `min_views` 24, `min_spots` 3, `holdout_every` 5 (ge 2).
  - `ArmFitLimits(StrictModel)`: `rms_px` 1.5, `holdout_rms_px` 2.0, `max_sigma_offset_deg` 0.5, `holdout_tool_mm` 5.0, `min_views` 20, `holdout_every` 5.
  - `BaseInBaseLink(StrictModel)`: `x` 0.0592, `y` -0.05, `z` 0.100, `yaw` 0.0 (the client.yml `arm.base_in_base_link` gauge, design D1).
  - `HeadArmFitConfig(StrictModel)`: `board: BoardConfig = BoardConfig()`, `plate: BoardConfig = PLATE_BOARD`, `plate_mount: PlateMountConfig`, `head_limits: HeadFitLimits`, `arm_limits: ArmFitLimits`, `huber_f_scale_px: float = 1.0` (gt 0), `board_z_m: float = 0.004`, `shoulder_pan_offset_rad: float = 0.0182`, `base_in_base_link: BaseInBaseLink`, `tool_offset_m: tuple[float, float, float] = (0.0010, -0.0056, -0.0014)`. All unknown keys rejected.
- `shared/ros2_common/head_rgbd_calibration/charuco.py`: `make_board(cfg)` passes `ids = np.arange(cfg.first_marker_id, cfg.first_marker_id + n_markers)` so the plate and the 6x5 board never decode as each other; default boards keep ids 0..n-1 (the existing RGB-D tests unchanged). New `board_corners_3d(cfg: BoardConfig) -> np.ndarray` (N, 3) ChArUco corner positions in the board frame (m, z = 0), row i = corner id i.
- New `shared/ros2_common/head_rgbd_calibration/views.py`: frozen dataclasses `FloorView(view_id: str, spot: int, joints: dict[str, float], detection: Detection)` and `PlateView(view_id: str, joints: dict[str, float], detection: Detection)`. `joints` holds measured rad by joint name (`head_pan`, `head_tilt`; plus `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll` for PlateView); never filled with defaults.
- New `tests/test_head_arm_calib_config.py`: defaults equal the values above; plate ids 15..21 and board ids 0..14 are disjoint; an image of the plate (cv2 `generateImage`) is detected by `detect` with `make_board(PLATE_BOARD)` and returns None with `make_board(BoardConfig())`, and the board image vice versa; an overflowing `first_marker_id` raises naming the field; unknown keys rejected; `board_corners_3d` shape `(8, 3)` for the plate and spacing 0.018 m.
- Existing core tests pass with no assertion changed; root lint passes.

## Acceptance conditions

- a1: plate and board marker ids are disjoint and each image decodes only with its own board [evidence: test:tests/test_head_arm_calib_config.py::test_plate_and_board_decode_only_with_their_own_ids]
- a2: a first_marker_id past the dictionary is refused naming the field [evidence: test:tests/test_head_arm_calib_config.py::test_first_marker_id_must_fit_the_dictionary]
- a3: the fit config defaults equal the design values [evidence: test:tests/test_head_arm_calib_config.py::test_head_arm_fit_config_defaults]
- a4: board_corners_3d gives the corner grid in metres [evidence: test:tests/test_head_arm_calib_config.py::test_board_corners_3d_grid]
- a5: FloorView and PlateView exist [evidence: symbol:shared/ros2_common/head_rgbd_calibration/views.py::PlateView]
- a6: existing core tests pass unchanged [evidence: command:uv run --frozen pytest tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_extrinsics.py tests/test_head_rgbd_calib_poses.py -q]
- a7: root lint passes [evidence: command:uv run poe lint]

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

