"""Unit tests for the stereo_depth config: defaults, every key, rejection of unknown keys and invalid values."""

from pathlib import Path

import pytest

from stereo_depth.config import (
    CONFIG_ENV_VAR,
    DEFAULT_CONFIG_PATH,
    ConfigError,
    DepthConfig,
    config_path_from_env,
    load_config,
)


def write(tmp_path: Path, text: str) -> Path:
    """Write a YAML config file.

    Args:
        tmp_path (Path): Test directory.
        text (str): File content.

    Returns:
        Path: The written file.
    """
    path = tmp_path / "config.yaml"
    path.write_text(text)
    return path


def test_empty_file_gives_the_defaults(tmp_path: Path) -> None:
    config = load_config(write(tmp_path, ""))
    assert config == DepthConfig()
    assert config.left_image_topic == "/overview_camera/image_raw"
    assert config.right_image_topic == "/stereo/right/image_raw"
    assert config.depth_topic == "/stereo/depth/image_raw"
    assert config.camera_info_topic == "/stereo/camera_info"
    assert config.publish_disparity is False
    assert config.disparity_topic == "/stereo/disparity"
    assert config.calibration_dir == Path("/var/lib/ros2/camera_calibration")
    assert config.calibration_check_s == 5.0
    assert (config.num_disparities, config.block_size) == (48, 5)
    assert (config.p1, config.p2) == (200, 800)
    assert config.uniqueness_ratio == 10
    assert (config.speckle_window_size, config.speckle_range) == (100, 2)
    assert (config.disp12_max_diff, config.pre_filter_cap) == (1, 31)
    assert (config.min_depth_m, config.max_depth_m) == (0.3, 4.0)
    assert config.sync_slop_s == 0.002
    assert config.max_rate_hz == 5.0
    assert config.matcher_threads == 2
    assert config.frame_id == "stereo_left_optical_frame"
    assert config.metrics_port == 19113


def test_every_key_can_be_set(tmp_path: Path) -> None:
    text = (
        "left_image_topic: /l\nright_image_topic: /r\ndepth_topic: /d\ncamera_info_topic: /i\n"
        "publish_disparity: true\ndisparity_topic: /disp\ncalibration_dir: /tmp/cal\ncalibration_check_s: 2.0\n"
        "num_disparities: 64\nblock_size: 7\np1: 100\np2: 400\nuniqueness_ratio: 5\nspeckle_window_size: 50\n"
        "speckle_range: 1\ndisp12_max_diff: 2\npre_filter_cap: 63\nmin_depth_m: 0.5\nmax_depth_m: 3.0\n"
        "sync_slop_s: 0.005\nmax_rate_hz: 2.5\nmatcher_threads: 1\nframe_id: ' f '\nmetrics_port: 19999\n"
    )
    config = load_config(write(tmp_path, text))
    assert (config.left_image_topic, config.right_image_topic) == ("/l", "/r")
    assert (config.depth_topic, config.camera_info_topic, config.disparity_topic) == ("/d", "/i", "/disp")
    assert config.publish_disparity is True
    assert config.calibration_dir == Path("/tmp/cal")
    assert config.calibration_check_s == 2.0
    assert (config.num_disparities, config.block_size, config.p1, config.p2) == (64, 7, 100, 400)
    assert (config.uniqueness_ratio, config.speckle_window_size, config.speckle_range) == (5, 50, 1)
    assert (config.disp12_max_diff, config.pre_filter_cap) == (2, 63)
    assert (config.min_depth_m, config.max_depth_m) == (0.5, 3.0)
    assert (config.sync_slop_s, config.max_rate_hz, config.matcher_threads) == (0.005, 2.5, 1)
    assert config.frame_id == "f"
    assert config.metrics_port == 19999


def test_metrics_port_can_be_disabled_to_fall_back_to_env(tmp_path: Path) -> None:
    assert load_config(write(tmp_path, "metrics_port: null\n")).metrics_port is None


def test_config_is_frozen() -> None:
    config = DepthConfig()
    with pytest.raises(ValueError):
        setattr(config, "num_disparities", 64)


@pytest.mark.parametrize(
    "text",
    [
        "unknown_key: 1\n",
        "num_disparities: 40\n",
        "num_disparities: 0\n",
        "block_size: 4\n",
        "block_size: 0\n",
        "block_size: 13\n",
        "p1: 800\np2: 200\n",
        "p1: 0\n",
        "uniqueness_ratio: -1\n",
        "speckle_window_size: -1\n",
        "speckle_range: -1\n",
        "disp12_max_diff: -2\n",
        "pre_filter_cap: 0\n",
        "pre_filter_cap: 64\n",
        "min_depth_m: 2.0\nmax_depth_m: 1.0\n",
        "min_depth_m: 0\n",
        "sync_slop_s: 0\n",
        "sync_slop_s: 0.5\n",
        "max_rate_hz: 0\n",
        "max_rate_hz: 31\n",
        "matcher_threads: 0\n",
        "calibration_check_s: 0\n",
        "frame_id: '  '\n",
        "left_image_topic: ''\n",
        "metrics_port: 0\n",
        "metrics_port: 70000\n",
    ],
)
def test_invalid_values_are_rejected(tmp_path: Path, text: str) -> None:
    with pytest.raises(ConfigError):
        load_config(write(tmp_path, text))


def test_missing_file_is_an_error(tmp_path: Path) -> None:
    with pytest.raises(ConfigError, match="not found"):
        load_config(tmp_path / "nope.yaml")


def test_non_mapping_is_an_error(tmp_path: Path) -> None:
    with pytest.raises(ConfigError, match="mapping"):
        load_config(write(tmp_path, "- 1\n"))


def test_invalid_yaml_is_an_error(tmp_path: Path) -> None:
    with pytest.raises(ConfigError, match="invalid YAML"):
        load_config(write(tmp_path, "a: [1\n"))


def test_config_path_from_env(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.delenv(CONFIG_ENV_VAR, raising=False)
    assert config_path_from_env() == DEFAULT_CONFIG_PATH == Path("/etc/ros2/stereo_depth/config.yaml")
    monkeypatch.setenv(CONFIG_ENV_VAR, str(tmp_path / "c.yaml"))
    assert config_path_from_env() == tmp_path / "c.yaml"
