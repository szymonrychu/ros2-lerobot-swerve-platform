"""Unit tests for the stereo_depth config loading."""

from pathlib import Path

import pytest

from stereo_depth.config import load_config


def test_missing_file_is_none(tmp_path: Path) -> None:
    assert load_config(tmp_path / "nope.yaml") is None


def test_defaults(tmp_path: Path) -> None:
    path = tmp_path / "config.yaml"
    path.write_text("{}\n")
    cfg = load_config(path)
    assert cfg is not None
    assert cfg.disparity_topic == "/stereo/disparity"
    assert cfg.camera_info_topic == "/stereo/left/camera_info"
    assert cfg.depth_topic == "/stereo/depth/image_rect"
    assert cfg.depth_camera_info_topic == "/stereo/depth/camera_info"
    assert (cfg.min_depth_m, cfg.max_depth_m) == (0.2, 4.0)


def test_overrides(tmp_path: Path) -> None:
    path = tmp_path / "config.yaml"
    path.write_text("disparity_topic: /a\ndepth_topic: /b\nmin_depth_m: 0.5\nmax_depth_m: 3.0\n")
    cfg = load_config(path)
    assert cfg is not None
    assert (cfg.disparity_topic, cfg.depth_topic, cfg.min_depth_m, cfg.max_depth_m) == ("/a", "/b", 0.5, 3.0)


def test_non_mapping_is_none(tmp_path: Path) -> None:
    path = tmp_path / "config.yaml"
    path.write_text("- 1\n")
    assert load_config(path) is None


def test_invalid_values_raise(tmp_path: Path) -> None:
    path = tmp_path / "config.yaml"
    path.write_text("min_depth_m: -1\n")
    with pytest.raises(ValueError):
        load_config(path)
