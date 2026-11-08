"""Config models for the marker generator and the board renders (pydantic, extra keys forbidden so errors name the
key)."""

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
    "RenderConfig",
    "StlOptions",
    "StrictModel",
    "SymbolDefaults",
    "load_generator_config",
    "load_render_config",
]
DEFAULT_OUTPUT_DIR = ".logs/markers"
DEFAULT_ARM_URDF = "nodes/web_ui/urdf/so101_arm.urdf"
DEFAULT_MARKERS_DIR = "markers"
DEFAULT_RENDER_BOARD = "charuco-arm-6x5-28mm"
DEFAULT_RENDER_DIR = ".logs/arm-calib-board/renders"
DEFAULT_RENDER_SIZE = (1200, 900)


class GeneratorConfig(StrictModel):
    """Whole generator config. boards: named ChArUco calibration boards (unique names, disjoint marker ids).
    arm_urdf_file: SO-101 arm URDF whose gripper meshes seat board handles (relative paths resolve against the repo
    root)."""

    output_dir: str = DEFAULT_OUTPUT_DIR
    arm_urdf_file: str = DEFAULT_ARM_URDF
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


class RenderConfig(StrictModel):
    """Render options of scripts/markers_render.py. Relative paths resolve against the repo root.

    markers_dir: directory with the written board STLs and JSON. board: board name (file stem). arm_urdf_file: the
    SO-101 arm URDF. out_dir: PNG output directory. size: image (width, height) in pixels.
    """

    markers_dir: str = DEFAULT_MARKERS_DIR
    board: str = DEFAULT_RENDER_BOARD
    arm_urdf_file: str
    out_dir: str = DEFAULT_RENDER_DIR
    size: tuple[int, int] = DEFAULT_RENDER_SIZE


def load_render_config(path: Path) -> RenderConfig:
    """Load and validate the render YAML.

    Args:
        path (Path): YAML file.

    Returns:
        RenderConfig: The config.

    Raises:
        ValidationError: On unknown keys or invalid values.
    """
    return RenderConfig.model_validate(yaml.safe_load(path.read_text()))
