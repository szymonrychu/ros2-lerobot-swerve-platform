"""Config models for the marker generator (pydantic, extra keys forbidden so errors name the key)."""

from pathlib import Path
from typing import Self

import yaml
from pydantic import Field, model_validator

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
    """Whole generator config. boards: named ChArUco calibration boards (unique names, disjoint marker ids)."""

    output_dir: str = DEFAULT_OUTPUT_DIR
    defaults: SymbolDefaults = Field(default_factory=SymbolDefaults)
    stl: StlOptions = Field(default_factory=StlOptions)
    markers: list[MarkerSpec] = Field(default_factory=list)
    boards: list[CharucoSpec] = Field(default_factory=list)

    @model_validator(mode="after")
    def check_boards(self) -> Self:
        """Board names are unique and boards sharing a dictionary use disjoint marker ids."""
        names = [board.name for board in self.boards]
        if len(set(names)) != len(names):
            raise ValueError(f"board names must be unique, got {names}")
        for i, first in enumerate(self.boards):
            for second in self.boards[i + 1 :]:
                if first.dictionary == second.dictionary and set(first.marker_ids) & set(second.marker_ids):
                    raise ValueError(f"board ids of {first.name} and {second.name} overlap")
        return self


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
