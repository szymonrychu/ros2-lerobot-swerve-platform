"""Config models for the marker generator (pydantic, extra keys forbidden so errors name the key)."""

from pathlib import Path
from typing import Any, Literal, Self

import yaml
from pydantic import BaseModel, ConfigDict, Field, ValidationError, model_validator

from shared.ros2_common.marker_payload import MarkerPayload

OUTPUT_FORMATS = ("pdf", "stl", "3mf")
DEFAULT_OUTPUT_DIR = ".logs/markers"


class StrictModel(BaseModel):
    """Base model rejecting unknown keys."""

    model_config = ConfigDict(extra="forbid")


class PageSpec(StrictModel):
    """Page for the PDF: a named size or an explicit width_mm and height_mm.

    size: A4/A3/LETTER (used when width_mm/height_mm are absent). orientation: portrait or landscape.
    """

    size: Literal["A4", "A3", "LETTER"] = "A4"
    width_mm: float | None = Field(default=None, gt=0)
    height_mm: float | None = Field(default=None, gt=0)
    orientation: Literal["portrait", "landscape"] = "portrait"

    @model_validator(mode="after")
    def check_custom(self) -> Self:
        """width_mm and height_mm are given together."""
        if (self.width_mm is None) != (self.height_mm is None):
            raise ValueError("page width_mm and height_mm must be given together")
        return self


class SymbolDefaults(StrictModel):
    """Symbol parameters shared by all markers.

    ecc: error correction M or Q. min_version / max_version: QR version range (2..4). quiet_zone_modules: >= 2.
    """

    ecc: Literal["m", "q"] = "q"
    min_version: int = Field(default=2, ge=2, le=4)
    max_version: int = Field(default=3, ge=2, le=4)
    quiet_zone_modules: int = Field(default=4, ge=2)
    page: PageSpec = Field(default_factory=PageSpec)

    @model_validator(mode="after")
    def check_range(self) -> Self:
        """min_version <= max_version."""
        if self.min_version > self.max_version:
            raise ValueError(f"min_version {self.min_version} exceeds max_version {self.max_version}")
        return self


class StlOptions(StrictModel):
    """3D print options in mm."""

    base_thickness_mm: float = Field(default=2.0, gt=0)
    relief_mm: float = Field(default=0.6, ge=0)
    inlay_mm: float = Field(default=0.0, ge=0)
    margin_mm: float = Field(default=3.0, ge=0)
    min_module_mm: float = Field(default=1.5, gt=0)
    warn_module_mm: float = Field(default=2.0, gt=0)


class MarkerSpec(StrictModel):
    """One marker to generate.

    kind_code / name / size_mm / params: the payload fields. outputs: subset of pdf/stl/3mf. page: overrides
    defaults.page.
    """

    kind_code: str
    name: str
    size_mm: float
    params: dict[str, str] = Field(default_factory=dict)
    outputs: list[Literal["pdf", "stl", "3mf"]] = Field(default_factory=lambda: list(OUTPUT_FORMATS), min_length=1)
    page: PageSpec | None = None

    def payload(self) -> MarkerPayload:
        """Build the payload.

        Returns:
            The validated MarkerPayload.

        Raises:
            ValueError: naming the marker when the codec rejects a field.
        """
        try:
            return MarkerPayload(
                version_major=1, kind_code=self.kind_code, name=self.name, size_mm=self.size_mm, params=self.params
            )
        except ValidationError as exc:
            problems = "; ".join(str(err["msg"]) for err in exc.errors())
            raise ValueError(f"marker {self.name!r} ({self.kind_code}): {problems}") from exc

    @model_validator(mode="after")
    def check_payload(self) -> Self:
        """Fail at config load when the codec rejects the marker."""
        self.payload()
        return self


class GeneratorConfig(StrictModel):
    """Whole generator config. charuco: free-form board options, validated by the charuco generator."""

    output_dir: str = DEFAULT_OUTPUT_DIR
    defaults: SymbolDefaults = Field(default_factory=SymbolDefaults)
    stl: StlOptions = Field(default_factory=StlOptions)
    markers: list[MarkerSpec] = Field(default_factory=list)
    charuco: dict[str, Any] | None = None


def load_generator_config(path: Path) -> GeneratorConfig:
    """Load and validate the generator YAML.

    Args:
        path: Config file.

    Returns:
        The GeneratorConfig.

    Raises:
        ValidationError: on unknown keys or invalid values (the message names them).
    """
    return GeneratorConfig.model_validate(yaml.safe_load(path.read_text()) or {})
