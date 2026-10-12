"""Unit tests for arducam_tof config: YAML loading, defaults, and rejection of unknown keys and out-of-range values."""

from pathlib import Path

import pytest

from arducam_tof.config import (
    CONFIG_ENV_VAR,
    DEFAULT_CONFIG_PATH,
    ConfigError,
    TofConfig,
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
    assert config == TofConfig()
    assert config.camera_index == 0
    assert config.range_mode == "far"
    assert config.confidence_threshold == 30
    assert config.frame_id == "tof_optical_frame"
    assert config.publish_rate_hz == 10.0
    assert config.metrics_port is None


def test_every_key_is_read(tmp_path: Path) -> None:
    config = load_config(
        write(
            tmp_path,
            "camera_index: 1\nrange_mode: near\nconfidence_threshold: 12.5\nframe_id: head_tof\n"
            "publish_rate_hz: 5\nmetrics_port: 19120\n",
        )
    )
    assert config.camera_index == 1
    assert config.range_mode == "near"
    assert config.confidence_threshold == 12.5
    assert config.frame_id == "head_tof"
    assert config.publish_rate_hz == 5.0
    assert config.metrics_port == 19120


def test_unknown_key_is_rejected(tmp_path: Path) -> None:
    with pytest.raises(ConfigError, match="camera_idx"):
        load_config(write(tmp_path, "camera_idx: 1\n"))


@pytest.mark.parametrize(
    "line",
    [
        "camera_index: -1",
        "camera_index: 64",
        "range_mode: medium",
        "confidence_threshold: -1",
        "confidence_threshold: 70000",
        "frame_id: ''",
        "frame_id: '   '",
        "publish_rate_hz: 0",
        "publish_rate_hz: -2",
        "publish_rate_hz: 31",
        "metrics_port: 0",
        "metrics_port: 70000",
    ],
)
def test_out_of_range_value_is_rejected(tmp_path: Path, line: str) -> None:
    with pytest.raises(ConfigError):
        load_config(write(tmp_path, line + "\n"))


def test_frame_id_is_stripped(tmp_path: Path) -> None:
    assert load_config(write(tmp_path, "frame_id: '  tof  '\n")).frame_id == "tof"


def test_missing_file_is_a_config_error(tmp_path: Path) -> None:
    with pytest.raises(ConfigError, match="not found"):
        load_config(tmp_path / "absent.yaml")


def test_invalid_yaml_is_a_config_error(tmp_path: Path) -> None:
    with pytest.raises(ConfigError, match="invalid YAML"):
        load_config(write(tmp_path, "camera_index: [1\n"))


def test_non_mapping_is_a_config_error(tmp_path: Path) -> None:
    with pytest.raises(ConfigError, match="mapping"):
        load_config(write(tmp_path, "- 1\n- 2\n"))


def test_config_path_comes_from_the_environment(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv(CONFIG_ENV_VAR, raising=False)
    assert CONFIG_ENV_VAR == "ARDUCAM_TOF_CONFIG"
    assert config_path_from_env() == DEFAULT_CONFIG_PATH
    monkeypatch.setenv(CONFIG_ENV_VAR, "/tmp/tof.yaml")
    assert config_path_from_env() == Path("/tmp/tof.yaml")
