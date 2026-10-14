"""Settings of the calibration tool (config/calibrate.yaml), a pydantic model."""

import os
from pathlib import Path
from typing import Dict, Optional

import cv2
from pydantic import BaseModel, ConfigDict, Field, ValidationError, model_validator

CONFIG_ENV_VAR = "STEREO_CALIBRATE_CONFIG"
DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "calibrate.yaml"


class ConfigError(Exception):
    """The config file is missing, not a flat key: value file or invalid."""


class CalibrateConfig(BaseModel):
    """Board, depthd address, capture rules and the RMS gate.

    Attributes:
        squares_x, squares_y (int): Board squares along its long and short side (7 x 5).
        square_mm, marker_mm (float): Printed or displayed square and ArUco marker edge, millimetres.
        dictionary (str): cv2.aruco predefined dictionary name.
        depthd_url (str): depthd HTTP API base URL.
        captures_dir (Path): Kept pairs (PNG), cleared at the start of capture.
        output_path (Path): Calibration file depthd loads (on the Jetson, never on the Pi).
        capture_count (int): Pairs to keep before capture stops.
        min_shared_corners (int): Fewest ChArUco corners seen in both images for a pair to count.
        capture_poll_s, capture_timeout_s (float): Pause between requests; give up after this long.
        min_position_shift, min_scale_change, min_tilt_change, min_roll_change_deg (float): A pair is new when its
            pose differs from every kept pose by at least one of these.
        rms_limit_px (float): Largest accepted stereo RMS; the tool refuses to write above it.
        min_pairs (int): Fewest usable pairs for a solve.
    """

    model_config = ConfigDict(extra="forbid", frozen=True)

    squares_x: int = Field(default=7, ge=3)
    squares_y: int = Field(default=5, ge=3)
    square_mm: float = Field(default=30.0, gt=0.0)
    marker_mm: float = Field(default=22.0, gt=0.0)
    dictionary: str = "DICT_5X5_100"
    depthd_url: str = "http://127.0.0.1:8090"
    captures_dir: Path = Path("/var/lib/depthd/calibration/cli-captures")
    output_path: Path = Path("/var/lib/depthd/stereo.yaml")
    capture_count: int = Field(default=30, ge=1)
    min_shared_corners: int = Field(default=12, ge=6)
    capture_poll_s: float = Field(default=0.25, ge=0.0)
    capture_timeout_s: float = Field(default=900.0, gt=0.0)
    min_position_shift: float = Field(default=0.08, gt=0.0)
    min_scale_change: float = Field(default=0.15, gt=0.0)
    min_tilt_change: float = Field(default=0.08, gt=0.0)
    min_roll_change_deg: float = Field(default=12.0, gt=0.0)
    rms_limit_px: float = Field(default=0.5, gt=0.0)
    min_pairs: int = Field(default=10, ge=3)

    @model_validator(mode="after")
    def check_board(self) -> "CalibrateConfig":
        """Require a marker smaller than the square and a known aruco dictionary.

        Returns:
            CalibrateConfig: The validated settings.
        """
        if self.marker_mm >= self.square_mm:
            raise ValueError("marker_mm must be below square_mm")
        if not self.dictionary.startswith("DICT_") or not hasattr(cv2.aruco, self.dictionary):
            raise ValueError("dictionary %r is not a cv2.aruco predefined dictionary" % self.dictionary)
        return self


def default_config_path() -> Path:
    """Config path from STEREO_CALIBRATE_CONFIG, else the calibrate.yaml next to the package.

    Returns:
        Path: Config file path.
    """
    return Path(os.environ.get(CONFIG_ENV_VAR) or DEFAULT_CONFIG_PATH)


def parse_flat_yaml(text: str, source: str) -> Dict[str, str]:
    """Parse the flat `key: value` YAML of calibrate.yaml (comments and blank lines allowed), stdlib only.

    Args:
        text (str): File content.
        source (str): Name for error messages.

    Returns:
        Dict[str, str]: Key to raw scalar text.

    Raises:
        ConfigError: On a line that is not `key: value` or a repeated key.
    """
    data: Dict[str, str] = {}
    for number, raw in enumerate(text.splitlines(), start=1):
        line = raw.split(" #", 1)[0] if not raw.lstrip().startswith("#") else ""
        if not line.strip():
            continue
        key, sep, value = line.partition(":")
        if not sep or not key.strip() or line[0].isspace():
            raise ConfigError("%s:%d: expected a top-level `key: value` line" % (source, number))
        if key.strip() in data:
            raise ConfigError("%s:%d: repeated key %r" % (source, number, key.strip()))
        data[key.strip()] = value.strip().strip("\"'")
    return data


def load_config(path: Optional[Path] = None) -> CalibrateConfig:
    """Load and validate the config; an empty file gives the defaults.

    Args:
        path (Optional[Path]): YAML file; default_config_path() when None.

    Returns:
        CalibrateConfig: Validated settings.

    Raises:
        ConfigError: When the file is missing, not a flat key: value file or invalid.
    """
    path = Path(path) if path else default_config_path()
    if not path.is_file():
        raise ConfigError("config file not found: %s" % path)
    data = parse_flat_yaml(path.read_text(), str(path))
    try:
        return CalibrateConfig(**data)
    except ValidationError as e:
        raise ConfigError("%s: %s" % (path, e)) from e
