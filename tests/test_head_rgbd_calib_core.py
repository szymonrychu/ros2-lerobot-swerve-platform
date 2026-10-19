"""Tests for the head RGB-D calibration core in shared/ros2_common/head_rgbd_calibration."""

import ast
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml
from pydantic import ValidationError
from ros2_common.head_rgbd_calibration import charuco as core_charuco
from ros2_common.head_rgbd_calibration.config import BoardConfig, LimitsConfig, SelectionConfig
from ros2_common.head_rgbd_calibration.frames import FrameSet, StampedImage, rgb_difference, still_pairs
from ros2_common.head_rgbd_calibration.intrinsics import calibrate

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


def test_core_imports_no_scripts_rosbags_or_subprocess() -> None:
    files = sorted(CORE_DIR.glob("*.py"))
    assert {f.name for f in files} >= {"__init__.py", "config.py", "charuco.py", "frames.py", "intrinsics.py"}
    for f in files:
        assert not imported_roots(f) & FORBIDDEN_MODULES, f.name


@pytest.mark.parametrize(
    ("model", "block"), [(BoardConfig, "board"), (SelectionConfig, "selection"), (LimitsConfig, "limits")]
)
def test_core_config_defaults_match_cli_yaml(model: type, block: str) -> None:
    with pytest.raises(ValidationError):
        model(bogus=1)
    yaml_block = yaml.safe_load(CALIBRATE_YAML.read_text())[block]
    assert model.model_validate(yaml_block) == model()


def test_core_detect_finds_synthetic_board() -> None:
    cfg = BoardConfig()
    camera = rgb_camera()
    image = render_rgb(camera, board_poses(1, camera, cfg)[0], cfg)
    detection = core_charuco.detect(image, core_charuco.make_board(cfg))
    assert detection is not None
    assert len(detection.ids) >= core_charuco.MIN_CORNERS
    assert detection.points.shape == (len(detection.ids), 2)
    assert np.isfinite(detection.points).all()


NS = 1_000_000_000
FLICKER_AMPLITUDE = 0.05
FLICKER_PERIOD_ROWS = 40
INTRINSICS_VIEWS = 20
INTRINSICS_DISTANCE_M = (0.4, 0.9)


def textured_frame(seed: int, shape: tuple[int, int] = (120, 160)) -> np.ndarray:
    """A smooth random texture as uint8 BGR."""
    noise = np.random.default_rng(seed).uniform(60, 180, shape).astype(np.float32)
    grey = cv2.GaussianBlur(noise, (0, 0), 3)
    return cv2.cvtColor(np.rint(grey).astype(np.uint8), cv2.COLOR_GRAY2BGR)


def test_core_still_pairs_one_pair_per_window() -> None:
    still_a, still_b = textured_frame(1), textured_frame(2)
    amp_still = np.full((18, 24), 1000, dtype=np.uint16)
    amp_still[:, :12] = 3000
    # stills at 2-5 s (scene A) and 8-11 s (scene B); one different frame between them, 10 Hz ToF, 2 Hz RGB
    rgb = []
    for i in range(24):
        t = i * 0.5
        image = still_a if 2 <= t < 5 else still_b if 8 <= t < 11 else textured_frame(100 + i)
        rgb.append(StampedImage(int(t * NS) + 5_000_000, image))
    amplitude = []
    for i in range(120):
        t = i * 0.1
        moving = not (2 <= t < 5 or 8 <= t < 11)
        amp = amp_still if not moving else np.roll(amp_still, i % 12, axis=1)
        amplitude.append(StampedImage(int(t * NS), amp))
    pairs = still_pairs(FrameSet(rgb=rgb, amplitude=amplitude), SelectionConfig())
    assert len(pairs) == 2
    assert 2 * NS <= pairs[0].rgb_stamp_ns < 5 * NS
    assert 8 * NS <= pairs[1].rgb_stamp_ns < 11 * NS


def test_core_rgb_difference_ignores_flicker_bands() -> None:
    a = cv2.cvtColor(textured_frame(3), cv2.COLOR_BGR2GRAY)
    rows = np.arange(a.shape[0])
    gain = 1 + FLICKER_AMPLITUDE * np.sin(2 * np.pi * rows / FLICKER_PERIOD_ROWS)
    b = np.clip(np.rint(a * gain[:, None]), 0, 255).astype(np.uint8)
    assert np.mean(np.abs(a.astype(float) - b.astype(float))) > 3
    assert rgb_difference(a, b) < 1.0


def test_core_intrinsics_recover_synthetic_camera() -> None:
    cfg = BoardConfig()
    camera = rgb_camera()
    board = core_charuco.make_board(cfg)
    poses = board_poses(INTRINSICS_VIEWS, camera, cfg, seed=3, distance_m=INTRINSICS_DISTANCE_M)
    found = [core_charuco.detect(render_rgb(camera, pose, cfg, seed=i), board) for i, pose in enumerate(poses)]
    detections = [d for d in found if d is not None]
    assert len(detections) >= 18
    result = calibrate(detections, board, camera.size)
    assert result.k[0, 0] == pytest.approx(camera.k[0, 0], rel=0.01)
    assert result.k[1, 1] == pytest.approx(camera.k[1, 1], rel=0.01)
    assert abs(result.k[0, 2] - camera.k[0, 2]) < 3
    assert abs(result.k[1, 2] - camera.k[1, 2]) < 3
    assert result.rms < 0.5
