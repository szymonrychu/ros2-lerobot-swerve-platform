"""Config models of the head RGB-D calibration core (pydantic, unknown keys forbidden so errors name the key)."""

from typing import Self

import cv2
from pydantic import BaseModel, ConfigDict, Field, model_validator


class StrictModel(BaseModel):
    """Base model that rejects unknown keys."""

    model_config = ConfigDict(extra="forbid")


class BoardConfig(StrictModel):
    """ChArUco board; FULL_BOARD and ARM_BOARD must equal the boards of scripts/markers_generate.yaml.

    name is the board id; ids_offset is the first ArUco marker id (the board uses squares_x * squares_y // 2
    consecutive ids from it).
    """

    name: str
    squares_x: int = Field(ge=2)
    squares_y: int = Field(ge=2)
    square_mm: float = Field(gt=0)
    marker_mm: float = Field(gt=0)
    dictionary: str
    ids_offset: int = Field(ge=0)
    thickness_mm: float = Field(gt=0)  # plate thickness behind the printed face (markers_generate.yaml)

    @property
    def marker_ids(self) -> range:
        """The ArUco marker ids of the board."""
        return range(self.ids_offset, self.ids_offset + self.squares_x * self.squares_y // 2)

    @model_validator(mode="after")
    def check_marker(self) -> Self:
        if self.marker_mm >= self.square_mm:
            raise ValueError("board.marker_mm must be smaller than board.square_mm")
        return self

    @model_validator(mode="after")
    def check_ids(self) -> Self:
        """The marker ids exist in the dictionary."""
        available = len(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.dictionary)).bytesList)
        if self.marker_ids.stop > available:
            raise ValueError(
                f"ids_offset {self.ids_offset} + {len(self.marker_ids)} markers exceeds the {available} ids of "
                f"{self.dictionary}"
            )
        return self


FULL_BOARD = BoardConfig(
    name="charuco-6x5-40mm",
    squares_x=6,
    squares_y=5,
    square_mm=40.0,
    marker_mm=30.0,
    dictionary="DICT_5X5_100",
    ids_offset=0,
    thickness_mm=4.0,
)
ARM_BOARD = BoardConfig(
    name="charuco-arm-6x5-28mm",
    squares_x=6,
    squares_y=5,
    square_mm=28.0,
    marker_mm=21.0,
    dictionary="DICT_5X5_100",
    ids_offset=15,
    thickness_mm=4.0,
)


class SelectionConfig(StrictModel):
    """Still-frame selection. MADs are mean absolute differences between consecutive frames.

    still_depth_mad is the mean absolute change between consecutive aligned depth frames as a fraction of the median
    depth.
    """

    still_window_s: float = Field(default=0.5, gt=0)
    still_rgb_mad: float = Field(default=2.0, ge=0)
    still_depth_mad: float = Field(default=0.01, ge=0)
    skip_start_s: float = Field(default=2.0, ge=0)


class LimitsConfig(StrictModel):
    """Acceptance limits of the colour intrinsics check."""

    plane_max_bias_mm: float = Field(default=40.0, gt=0)
    plane_max_spread_mm: float = Field(default=50.0, gt=0)
    min_views: int = Field(default=12, ge=1)
    intrinsics_max_rms_px: float = Field(default=0.5, gt=0)
    intrinsics_max_focal_diff_pct: float = Field(default=1.0, gt=0)
    intrinsics_max_principal_diff_px: float = Field(default=5.0, gt=0)


class FloorPlaneConfig(StrictModel):
    """Floor plane fit of one ToF cloud: candidate gate around the predicted plane, RANSAC, SVD refit."""

    ransac_distance_m: float = Field(default=0.03, gt=0, le=0.1)
    ransac_iterations: int = Field(default=200, ge=10)
    min_inliers: int = Field(default=150, ge=10)
    min_inlier_ratio: float = Field(default=0.5, gt=0, le=1)
    max_range_m: float = Field(default=1.8, gt=0)
    candidate_band_m: float = Field(default=0.25, gt=0)
    seed: int = 0

    @model_validator(mode="after")
    def check_band(self) -> Self:
        if self.candidate_band_m <= self.ransac_distance_m:
            raise ValueError("plane.candidate_band_m must be larger than plane.ransac_distance_m")
        return self


class FloorFitLimits(StrictModel):
    """Acceptance limits of the floor-plane head chain fit; a result outside any of them is refused.

    `max_height_rms_mm` and `max_normal_rms_deg` apply to the pose-mean residuals (the mean of a pose's observations),
    not to each observation: one pose's clouds scatter about 1 deg cloud to cloud (structured ToF noise) that no chain
    explains. `max_sigma_deg` and `max_delta_deg` apply to every fitted angle (mount roll and pitch, tilt zero and
    tilt roll).
    """

    min_observations: int = Field(default=30, gt=0)
    min_poses: int = Field(default=8, gt=0)
    min_pan_span_deg: float = Field(default=60.0, gt=0)
    max_height_rms_mm: float = Field(default=8.0, gt=0)
    max_normal_rms_deg: float = Field(default=1.0, gt=0)
    max_sigma_z_mm: float = Field(default=3.0, gt=0)
    max_sigma_deg: float = Field(default=0.3, gt=0)
    max_delta_z_m: float = Field(default=0.10, gt=0)
    max_delta_deg: float = Field(default=10.0, gt=0)


class FloorFitConfig(StrictModel):
    """Floor-plane head chain fit: plane fit, limits, residual scales and the Huber loss scale."""

    plane: FloorPlaneConfig = Field(default_factory=FloorPlaneConfig)
    limits: FloorFitLimits = Field(default_factory=FloorFitLimits)
    sigma_height_mm: float = Field(default=5.0, gt=0)
    sigma_normal_deg: float = Field(default=0.5, gt=0)
    huber_scale: float = Field(default=1.0, gt=0)


ARM_JOINT_NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
DEFAULT_JOINT_OFFSETS_RAD = {
    "shoulder_pan": 0.0182,
    "shoulder_lift": -0.0345,
    "elbow_flex": -0.1070,
    "wrist_flex": 0.0988,
    "wrist_roll": -0.0710,
}
# The current tool point (client.yml mcp_server arm.tool_offset_m, fitted 2026-10-10), m in gripper_frame_link.
DEFAULT_TOOL_OFFSET_M = (0.0010, -0.0056, -0.0014)


class BoardMountConfig(StrictModel):
    """Board frame in gripper_link: start values of the mount and the plate margin.

    `xyz_init` is the board frame origin in gripper_link (m), `rpy_init` the fixed-axis xyz rotation (rad),
    `outline_margin_m` the plate margin beyond the outer squares used by the pose check. `handle_boxes_mm` is the
    material of the held board beyond its plate (the edge handle's post, gusset and tab, user m540; on the pattern
    side they reach z < 0 and lie outside the plate outline),
    one ((min x, y, z), (max x, y, z)) box per part in the board frame (mm); the pose check checks them as board
    points. Empty for a bare plate.
    """

    xyz_init: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy_init: tuple[float, float, float] = (0.0, 0.0, 0.0)
    outline_margin_m: float = Field(default=0.01, ge=0)
    handle_boxes_mm: list[tuple[tuple[float, float, float], tuple[float, float, float]]] = Field(default_factory=list)

    @model_validator(mode="after")
    def check_handle_boxes(self) -> Self:
        """Every handle box has its min below its max on each axis (anywhere in the board frame)."""
        for index, (low, high) in enumerate(self.handle_boxes_mm):
            if any(lo >= hi for lo, hi in zip(low, high, strict=True)):
                raise ValueError(f"handle_boxes_mm[{index}]: min {low} must be below max {high} on every axis")
        return self


class PlateMountConfig(StrictModel):
    """Plate (the gripper-held ChArUco board of the arm offsets fit) in gripper_link.

    `xyz_init` / `rpy_init` are the start of the plate frame origin (m) and fixed-axis xyz rotation (rad). `rpy` is
    the CAD rotation of the plate: when set the rotation is fixed to it and wrist_roll is fitted; when None the
    rotation is fitted and wrist_roll stays at `HeadArmFitConfig.wrist_roll_offset_rad` (a wrist_roll zero error turns
    the plate about the wrist_roll axis exactly, so both cannot be free). `tool_in_board_m` is the CAD board-to-jaw-tip
    offset, the tool point in the plate frame (m); the key has no default and is null until it is measured, in which
    case the tool point is not fitted (design Q2).
    """

    xyz_init: tuple[float, float, float]
    rpy_init: tuple[float, float, float]
    rpy: tuple[float, float, float] | None = None
    tool_in_board_m: tuple[float, float, float] | None


class ArmFitLimits(StrictModel):
    """Acceptance limits of the arm offsets fit (arm_offsets_fit); held-out views decide, never the fit residual alone
    (design D9).

    `rms_px` and `holdout_rms_px` are the head-from-arm evidence limits (same camera, board and arm). Every
    `holdout_every`-th plate view is held out; the held-out views must cover at least `min_holdout_head_settings`
    distinct head settings (head offset indices). `max_tool_holdout_mm` bounds the held-out tool error (RMS of the arm
    FK tool point against the view's solvePnP plate pose through the fitted chain); the tool point is fitted only
    below it (design Q2: 5 mm). `min_views` counts the fit plate views (held-out views not counted).
    """

    rms_px: float = Field(default=3.0, gt=0)
    holdout_rms_px: float = Field(default=4.0, gt=0)
    max_tool_holdout_mm: float = Field(default=5.0, gt=0)
    min_views: int = Field(default=40, gt=0)
    holdout_every: int = Field(default=5, ge=2)
    min_holdout_head_settings: int = Field(default=3, ge=1)


class HeadArmFitLimits(StrictModel):
    """Acceptance limits of the head-from-arm fit; a result outside any of them is refused.

    The residual limits come from the real session calib-20261023-074418 (29 arm poses, 113 views) refitted with the
    pan axis free over five leave-poses-out folds (tests/test_head_arm_real_session.py): the arm is about 2 mm
    accurate (gripper camera calibration), about 2.5 px at the board distance, and the folds gave fit rms 2.51-2.72 px,
    held-out 2.56-3.59 px and held-out board error 2.89-3.77 mm, formal sigma at most 0.72 mm and 0.153 deg, the
    largest correlation 0.973-0.976 (mount y and mount yaw). A wrong model of the same views stays above `rms_px`: the
    fixed pan axis 11.2-12.2 px (held-out 9.4-14.9 px, 9.4-10.5 mm), a pan scale factor instead of the axis
    3.07-3.33 px. Degenerate parameter sets reach a correlation of 0.994 or more (`max_correlation`).
    `max_pan_axis_delta_mm` bounds the pan axis shift against its start (the head is 90 mm wide; the session gave
    38 mm).
    """

    rms_px: float = Field(default=3.0, gt=0)
    holdout_rms_px: float = Field(default=4.0, gt=0)
    holdout_board_mm: float = Field(default=4.5, gt=0)
    max_sigma_mm: float = Field(default=2.0, gt=0)
    max_sigma_deg: float = Field(default=0.2, gt=0)
    max_correlation: float = Field(default=0.99, gt=0, le=1)
    min_views: int = Field(default=40, gt=0)
    min_arm_poses: int = Field(default=16, gt=0)
    min_pan_span_deg: float = Field(default=30.0, gt=0)
    min_tilt_span_deg: float = Field(default=10.0, gt=0)
    holdout_every: int = Field(default=5, ge=2)
    max_floor_delta_mm: float = Field(default=10.0, gt=0)
    max_floor_delta_deg: float = Field(default=1.5, gt=0)
    max_start_delta_mm: float = Field(default=30.0, gt=0)
    max_start_delta_deg: float = Field(default=5.0, gt=0)
    max_pan_axis_delta_mm: float = Field(default=50.0, gt=0)


class BaseInBaseLink(StrictModel):
    """Arm base pose in base_link (m, rad); the client.yml mcp_server arm.base_in_base_link."""

    x: float = 0.0592
    y: float = -0.05
    z: float = 0.100
    yaw: float = 0.0


class ArmCameraBounds(StrictModel):
    """Physical plausibility bounds of the arm to camera fit (user decision 324): a result past any of them is refused
    outright (it cannot be accepted), unlike the advisory limits, which only warn.

    `max_offset_change_deg` bounds every arm joint offset change against the applied value, `max_base_move_mm` the arm
    base translation and `max_base_turn_deg` its yaw change, `max_skew_mm` / `max_skew_deg` the board seat against the
    CAD mount, `max_sag_deg` the board turn about the grip axis at the reference torque and `max_droop_mm` the droop
    along gravity at the reference torque.
    """

    max_offset_change_deg: float = Field(default=15.0, gt=0)
    max_base_move_mm: float = Field(default=100.0, gt=0)
    max_base_turn_deg: float = Field(default=10.0, gt=0)
    max_skew_mm: float = Field(default=10.0, gt=0)
    max_skew_deg: float = Field(default=10.0, gt=0)
    max_sag_deg: float = Field(default=5.0, gt=0)
    max_droop_mm: float = Field(default=5.0, gt=0)


# ARM board mass model (markers/charuco-arm-6x5-28mm.json, pinned by tests/test_arm_camera_fit.py): mass, centre of
# mass and grip point (the handle tool point) in the board frame, grip axis (the jaw closing axis, board x).
ARM_BOARD_MASS_KG = 0.211728
ARM_BOARD_COM_M = (0.08395, 0.109253, -0.002385)
ARM_BOARD_GRIP_POINT_M = (0.0705, 0.193767, -0.021386)
ARM_BOARD_GRIP_AXIS = (1.0, 0.0, 0.0)


class ArmCameraFitConfig(StrictModel):
    """Arm to camera fit (arm_camera_fit, user decisions 315, 324-326): the head chain is fixed to the applied one;
    the arm joint offsets, the arm base x, y (z with `fit_base_z`), the 6-DOF board seat in the gripper and a gravity
    sag are fitted.

    Gauges: the arm base yaw stays at the applied value (a yaw turn is exactly a shoulder_pan offset change plus a base
    x, y shift); the wrist_roll offset stays at the applied value unless `fit_wrist_roll` (it is exactly a board turn
    about the wrist_roll axis, so only the board prior pins it). Priors (one sigma): the board seat against the CAD
    `board_mount` (`board_prior_mm`, `board_prior_deg`), the sag (`sag_prior_deg` at the reference torque) and the droop
    (`droop_prior_mm` at the reference torque, only with `fit_droop`). Corner residuals are divided by
    `pixel_sigma_px` so the priors weigh in sigma units.

    Board mass model: `board_mass_kg`, `board_com_m` (centre of mass in the board frame), `grip_point_m` (where the
    jaws hold it, board frame) and `grip_axis` (the jaw closing axis, board frame). The reference torque is the board
    weight times the centre of mass distance from the grip axis.

    Accuracy examples: the tool point at `reach_m` horizontal reach and at full reach, and the gripper at
    `camera_distance_m` in front of the head camera.
    """

    fit_base_z: bool = True
    fit_wrist_roll: bool = False
    fit_droop: bool = False
    pixel_sigma_px: float = Field(default=1.0, gt=0)
    board_prior_mm: float = Field(default=3.0, gt=0)
    board_prior_deg: float = Field(default=2.0, gt=0)
    sag_prior_deg: float = Field(default=1.0, gt=0)
    droop_prior_mm: float = Field(default=1.0, gt=0)
    board_mass_kg: float = Field(default=ARM_BOARD_MASS_KG, gt=0)
    board_com_m: tuple[float, float, float] = ARM_BOARD_COM_M
    grip_point_m: tuple[float, float, float] = ARM_BOARD_GRIP_POINT_M
    grip_axis: tuple[float, float, float] = ARM_BOARD_GRIP_AXIS
    reach_m: float = Field(default=0.30, gt=0)
    camera_distance_m: float = Field(default=0.50, gt=0)
    bounds: ArmCameraBounds = Field(default_factory=ArmCameraBounds)

    @model_validator(mode="after")
    def check_axis(self) -> Self:
        """The grip axis has a length."""
        if sum(v * v for v in self.grip_axis) <= 0.0:
            raise ValueError("arm_camera.grip_axis must not be zero")
        return self


class HeadArmFitConfig(StrictModel):
    """Head-from-arm fit: board and its mount on the gripper, limits, the trusted arm gauge and the floor prior.

    `fit_pan_axis` frees the head pan axis position (head_pan origin x, y) relative to the head, which stays where
    it is at pan 0 (head_tilt origin moved back by the same amount). `fit_camera_mount` frees the camera mount joint
    (d435_joint) x, z, roll and yaw from its CAD start; its y and pitch are not observable (head_arm_fit).

    `floor_prior_weight` 10 (user decision 2026-10-23, replacing D49's prior off): without it the arm views of
    calib-20261023-074418 put the pan axis direction 2.1-2.8 deg off the floor fit (mount roll, mount pitch and tilt
    zero fail the floor cross-check; mount pitch plus tilt zero agree within 0.04 deg); with it the fit passes at rms
    2.93 px against 2.67 px, held-out 2.88 px against 2.98 px.

    The arm offsets fit (arm_offsets_fit) reads `plate`, `plate_mount` (required by that fit, None otherwise),
    `wrist_roll_offset_rad` (the wrist_roll zero from the user's recorded servo neutral, rad; required when the plate
    rotation is fitted), `tool_offset_m` (the current tool point in gripper_frame_link, reported unchanged when the
    tool point is not fitted), `floor_fit` (its floor views) and `arm_limits`.

    The arm to camera fit (arm_camera_fit) reads `arm_camera` (ArmCameraFitConfig), `board`, `board_mount` (the CAD
    seat its prior pulls toward), `joint_offsets_rad` and `base_in_base_link` (the applied calibration it starts from),
    `tool_offset_m`, `limits.holdout_every`, `limits.min_arm_poses` and `arm_limits` (advisory warnings).
    """

    board: BoardConfig = ARM_BOARD
    fit_pan_axis: bool = True
    fit_camera_mount: bool = True
    board_mount: BoardMountConfig = Field(default_factory=BoardMountConfig)
    limits: HeadArmFitLimits = Field(default_factory=HeadArmFitLimits)
    huber_f_scale_px: float = Field(default=1.0, gt=0)
    base_in_base_link: BaseInBaseLink = Field(default_factory=BaseInBaseLink)
    joint_offsets_rad: dict[str, float] = Field(default_factory=lambda: dict(DEFAULT_JOINT_OFFSETS_RAD))
    floor_prior_weight: float = Field(default=10.0, ge=0)
    plate: BoardConfig = ARM_BOARD
    plate_mount: PlateMountConfig | None = None
    wrist_roll_offset_rad: float | None = None
    tool_offset_m: tuple[float, float, float] = DEFAULT_TOOL_OFFSET_M
    floor_fit: FloorFitConfig = Field(default_factory=FloorFitConfig)
    arm_limits: ArmFitLimits = Field(default_factory=ArmFitLimits)
    arm_camera: ArmCameraFitConfig = Field(default_factory=ArmCameraFitConfig)

    @model_validator(mode="after")
    def check_joints(self) -> Self:
        names = set(self.joint_offsets_rad)
        missing = [name for name in ARM_JOINT_NAMES if name not in names]
        extra = sorted(names - set(ARM_JOINT_NAMES))
        if missing or extra:
            raise ValueError(f"joint_offsets_rad must name exactly the arm joints: missing {missing}, extra {extra}")
        return self
