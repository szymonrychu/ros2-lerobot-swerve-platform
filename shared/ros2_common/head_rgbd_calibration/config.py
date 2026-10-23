"""Config models of the head RGB-D calibration core (pydantic, unknown keys forbidden so errors name the key)."""

from typing import Self

from pydantic import BaseModel, ConfigDict, Field, model_validator


class StrictModel(BaseModel):
    """Base model that rejects unknown keys."""

    model_config = ConfigDict(extra="forbid")


class BoardConfig(StrictModel):
    """ChArUco board; must equal the charuco spec of scripts/markers_generate.yaml."""

    squares_x: int = Field(default=6, ge=2)
    squares_y: int = Field(default=5, ge=2)
    square_mm: float = Field(default=40.0, gt=0)
    marker_mm: float = Field(default=30.0, gt=0)
    dictionary: str = "DICT_5X5_100"

    @model_validator(mode="after")
    def check_marker(self) -> Self:
        if self.marker_mm >= self.square_mm:
            raise ValueError("board.marker_mm must be smaller than board.square_mm")
        return self


class SelectionConfig(StrictModel):
    """Still-frame selection. MADs are mean absolute differences between consecutive frames."""

    still_window_s: float = Field(default=0.5, gt=0)
    still_rgb_mad: float = Field(default=2.0, ge=0)
    still_tof_mad: float = Field(default=0.01, ge=0)
    max_pair_dt_s: float = Field(default=0.05, gt=0)
    skip_start_s: float = Field(default=2.0, ge=0)
    amplitude_percentiles: tuple[float, float] = (1, 99)
    tof_upscale: int = Field(default=2, ge=1)

    @model_validator(mode="after")
    def check_percentiles(self) -> Self:
        low, high = self.amplitude_percentiles
        if not 0 <= low < high <= 100:
            raise ValueError("selection.amplitude_percentiles must satisfy 0 <= low < high <= 100")
        return self


class LimitsConfig(StrictModel):
    """Acceptance limits for the calibration fits."""

    rgb_max_rms_px: float = Field(default=0.5, gt=0)
    tof_max_rms_px: float = Field(default=0.3, gt=0)
    stereo_max_rms_px: float = Field(default=1.0, gt=0)
    tof_zero_distortion_rms_px: float = Field(default=0.3, gt=0)
    plane_max_bias_mm: float = Field(default=40.0, gt=0)
    plane_max_spread_mm: float = Field(default=50.0, gt=0)
    min_views: int = Field(default=12, ge=1)


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


class BoardMountConfig(StrictModel):
    """Board frame in gripper_link: start values of the mount and the plate margin.

    `xyz_init` is the board frame origin in gripper_link (m), `rpy_init` the fixed-axis xyz rotation (rad),
    `outline_margin_m` the plate margin beyond the outer squares used by the pose check.
    """

    xyz_init: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy_init: tuple[float, float, float] = (0.0, 0.0, 0.0)
    outline_margin_m: float = Field(default=0.01, ge=0)


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


class HeadArmFitConfig(StrictModel):
    """Head-from-arm fit: board and its mount on the gripper, limits, the trusted arm gauge and the floor prior.

    `fit_pan_axis` frees the head pan axis position (head_pan origin x, y) relative to the head, which stays where
    it is at pan 0 (head_tilt origin moved back by the same amount).
    """

    board: BoardConfig = Field(default_factory=BoardConfig)
    fit_pan_axis: bool = True
    board_mount: BoardMountConfig = Field(default_factory=BoardMountConfig)
    limits: HeadArmFitLimits = Field(default_factory=HeadArmFitLimits)
    huber_f_scale_px: float = Field(default=1.0, gt=0)
    base_in_base_link: BaseInBaseLink = Field(default_factory=BaseInBaseLink)
    joint_offsets_rad: dict[str, float] = Field(default_factory=lambda: dict(DEFAULT_JOINT_OFFSETS_RAD))
    floor_prior_weight: float = Field(default=0.0, ge=0)

    @model_validator(mode="after")
    def check_joints(self) -> Self:
        names = set(self.joint_offsets_rad)
        missing = [name for name in ARM_JOINT_NAMES if name not in names]
        extra = sorted(names - set(ARM_JOINT_NAMES))
        if missing or extra:
            raise ValueError(f"joint_offsets_rad must name exactly the arm joints: missing {missing}, extra {extra}")
        return self
