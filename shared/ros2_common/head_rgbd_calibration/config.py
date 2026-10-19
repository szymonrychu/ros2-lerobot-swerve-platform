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
    min_views: int = Field(default=12, ge=1)
