"""Tests for the marker_detector config models and the venv's OpenCV."""

from pathlib import Path

import cv2
import pytest
from pydantic import ValidationError

from marker_detector.config import MarkerDetectorConfig, config_path, load_config


def write(tmp_path: Path, text: str) -> Path:
    path = tmp_path / "config.yaml"
    path.write_text(text)
    return path


def test_defaults_from_empty_yaml(tmp_path: Path) -> None:
    cfg = load_config(write(tmp_path, ""))
    assert cfg.topics.image == "/head_camera/image_raw/compressed"
    assert cfg.topics.detections == "/markers/detections"
    assert cfg.process_hz == 2.0
    assert cfg.map_frame == "map"
    assert cfg.pose.ambiguity_ratio == 2.0
    assert cfg.zoom.full_size == (4608, 2592)
    assert cfg.tof.max_residual_mm == 10
    assert cfg.metrics_port is None


def test_full_yaml(tmp_path: Path) -> None:
    cfg = load_config(
        write(
            tmp_path,
            "process_hz: 5\nmetrics_port: 9100\nzoom:\n  auto_zoom: true\n  min_module_px: 7\n  target_module_px: 15\n",
        )
    )
    assert cfg.process_hz == 5
    assert cfg.metrics_port == 9100
    assert cfg.zoom.auto_zoom is True
    assert isinstance(cfg, MarkerDetectorConfig)


def test_zoom_flag_defaults(tmp_path: Path) -> None:
    cfg = load_config(write(tmp_path, ""))
    assert cfg.zoom.auto_zoom is False
    assert cfg.zoom.process_stills is True


def test_unknown_key_rejected_naming_key(tmp_path: Path) -> None:
    with pytest.raises(ValidationError, match="bogus_key"):
        load_config(write(tmp_path, "pose:\n  bogus_key: 1\n"))


def test_inconsistent_zoom_threshold_rejected(tmp_path: Path) -> None:
    with pytest.raises(ValidationError, match="target_module_px"):
        load_config(write(tmp_path, "zoom:\n  target_module_px: 5\n"))
    with pytest.raises(ValidationError, match="min_module_px"):
        load_config(write(tmp_path, "zoom:\n  min_module_px: 3\n"))


def test_process_hz_range(tmp_path: Path) -> None:
    with pytest.raises(ValidationError, match="process_hz"):
        load_config(write(tmp_path, "process_hz: 50\n"))


def test_env_override(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.delenv("MARKER_DETECTOR_CONFIG", raising=False)
    assert config_path() == Path("/etc/ros2/marker_detector/config.yaml")
    monkeypatch.setenv("MARKER_DETECTOR_CONFIG", str(tmp_path / "x.yaml"))
    assert config_path() == tmp_path / "x.yaml"


def test_venv_opencv_is_modern() -> None:
    major, minor = (int(x) for x in cv2.__version__.split(".")[:2])
    assert (major, minor) >= (4, 8)
    assert hasattr(cv2, "QRCodeDetectorAruco")
    assert hasattr(cv2.aruco, "CharucoDetector")
    import zxingcpp

    assert zxingcpp.__name__ == "zxingcpp"
