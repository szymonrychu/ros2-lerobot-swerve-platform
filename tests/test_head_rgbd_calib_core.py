"""Tests for the head RGB-D calibration core in shared/ros2_common/head_rgbd_calibration."""

import ast
from pathlib import Path

import numpy as np
import pytest
import yaml
from pydantic import ValidationError
from ros2_common.head_rgbd_calibration import charuco as core_charuco
from ros2_common.head_rgbd_calibration.config import BoardConfig, LimitsConfig, SelectionConfig

from tests.rgbd_synthetic import board_poses, render_rgb, rgb_camera

ROOT = Path(__file__).resolve().parent.parent
CORE_DIR = ROOT / "shared" / "ros2_common" / "head_rgbd_calibration"
CALIBRATE_YAML = ROOT / "scripts" / "head_rgbd_calibrate.yaml"
FORBIDDEN_MODULES = {"scripts", "rosbags", "subprocess"}


def imported_roots(path: Path) -> set[str]:
    """Top-level module names imported by a Python file."""
    roots: set[str] = set()
    for node in ast.walk(ast.parse(path.read_text())):
        if isinstance(node, ast.Import):
            roots.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            roots.add(node.module.split(".")[0])
    return roots


def test_core_modules_import_no_scripts_rosbags_or_subprocess() -> None:
    files = sorted(CORE_DIR.glob("*.py"))
    assert {f.name for f in files} >= {"__init__.py", "config.py", "charuco.py"}
    for f in files:
        assert not imported_roots(f) & FORBIDDEN_MODULES, f.name


@pytest.mark.parametrize(
    ("model", "block"), [(BoardConfig, "board"), (SelectionConfig, "selection"), (LimitsConfig, "limits")]
)
def test_core_config_rejects_unknown_keys_and_defaults_match_yaml(model: type, block: str) -> None:
    with pytest.raises(ValidationError):
        model(bogus=1)
    yaml_block = yaml.safe_load(CALIBRATE_YAML.read_text())[block]
    assert model.model_validate(yaml_block) == model()


def test_core_detect_finds_the_synthetic_board() -> None:
    cfg = BoardConfig()
    camera = rgb_camera()
    image = render_rgb(camera, board_poses(1, camera, cfg)[0], cfg)
    detection = core_charuco.detect(image, core_charuco.make_board(cfg))
    assert detection is not None
    assert len(detection.ids) >= core_charuco.MIN_CORNERS
    assert detection.points.shape == (len(detection.ids), 2)
    assert np.isfinite(detection.points).all()
