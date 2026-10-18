"""Config models for the marker generator (pydantic, extra keys forbidden so errors name the key)."""

from pathlib import Path

import yaml
from pydantic import Field

from scripts.markers.charuco import CharucoSpec
from scripts.markers.models import MarkerSpec, PageSpec, StlOptions, StrictModel, SymbolDefaults

__all__ = [
    "CharucoSpec",
    "GeneratorConfig",
    "MarkerSpec",
    "PageSpec",
    "StlOptions",
    "StrictModel",
    "SymbolDefaults",
    "load_generator_config",
]
DEFAULT_OUTPUT_DIR = ".logs/markers"


class GeneratorConfig(StrictModel):
    """Whole generator config. charuco: the ChArUco calibration board spec (omit to skip it)."""

    output_dir: str = DEFAULT_OUTPUT_DIR
    defaults: SymbolDefaults = Field(default_factory=SymbolDefaults)
    stl: StlOptions = Field(default_factory=StlOptions)
    markers: list[MarkerSpec] = Field(default_factory=list)
    charuco: CharucoSpec | None = None


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
