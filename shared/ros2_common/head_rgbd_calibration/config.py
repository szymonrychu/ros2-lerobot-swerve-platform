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
    max_normal_rms_deg: float = Field(default=0.6, gt=0)
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
