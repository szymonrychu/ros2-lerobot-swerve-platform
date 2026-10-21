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
    resolve_camera_index,
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
    assert config.stamp_latency_ms == 0.0
    assert config.tof_calibration_file is None
    assert config.depth_is_radial is True


def test_every_key_is_read(tmp_path: Path) -> None:
    config = load_config(
        write(
            tmp_path,
            "camera_index: 1\nrange_mode: near\nconfidence_threshold: 12.5\nframe_id: head_tof\n"
            "publish_rate_hz: 5\nmetrics_port: 19120\nstamp_latency_ms: 42.5\n"
            "tof_calibration_file: /etc/ros2/tof.yaml\ndepth_is_radial: false\n",
        )
    )
    assert config.camera_index == 1
    assert config.range_mode == "near"
    assert config.confidence_threshold == 12.5
    assert config.frame_id == "head_tof"
    assert config.publish_rate_hz == 5.0
    assert config.metrics_port == 19120
    assert config.stamp_latency_ms == 42.5
    assert config.tof_calibration_file == Path("/etc/ros2/tof.yaml")
    assert config.depth_is_radial is False


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
        "stamp_latency_ms: -1",
        "stamp_latency_ms: 201",
    ],
)
def test_out_of_range_value_is_rejected(tmp_path: Path, line: str) -> None:
    with pytest.raises(ConfigError, match=line.split(":")[0]):
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


def fake_video_node(root: Path, index: int, name: str, controller: str) -> None:
    """Create a sysfs-like /sys/class/video4linux/video<index> entry.

    Args:
        root (Path): Fake video4linux class directory.
        index (int): Video node number.
        name (str): Content of the name file (e.g. rp1-cfe-csi2_ch0).
        controller (str): Basename the device symlink resolves to (e.g. 1f00128000.csi).
    """
    devices = root.parent / "devices" / controller
    devices.mkdir(parents=True, exist_ok=True)
    node = root / f"video{index}"
    node.mkdir(parents=True)
    (node / "name").write_text(f"{name}\n")
    (node / "device").symlink_to(devices)


def test_csi_device_defaults_to_none_and_the_fixed_index_is_used(tmp_path: Path) -> None:
    config = TofConfig(camera_index=3)
    assert config.csi_device is None
    assert resolve_camera_index(config, tmp_path / "video4linux") == 3


def test_csi_device_resolves_the_csi2_ch0_node_of_that_controller(tmp_path: Path) -> None:
    root = tmp_path / "video4linux"
    fake_video_node(root, 0, "rp1-cfe-csi2_ch0", "1f00110000.csi")
    fake_video_node(root, 9, "rp1-cfe-embedded", "1f00128000.csi")
    fake_video_node(root, 10, "rp1-cfe-csi2_ch0", "1f00128000.csi")
    config = TofConfig(camera_index=8, csi_device="1f00128000.csi")
    assert resolve_camera_index(config, root) == 10


def test_csi_device_without_a_capture_node_raises(tmp_path: Path) -> None:
    root = tmp_path / "video4linux"
    fake_video_node(root, 0, "rp1-cfe-csi2_ch0", "1f00110000.csi")
    with pytest.raises(LookupError, match="1f00128000.csi"):
        resolve_camera_index(TofConfig(csi_device="1f00128000.csi"), root)


def test_csi_device_is_loaded_from_yaml(tmp_path: Path) -> None:
    config = load_config(write(tmp_path, "csi_device: 1f00128000.csi\n"))
    assert config.csi_device == "1f00128000.csi"


def test_depth_is_radial_must_be_a_bool(tmp_path: Path) -> None:
    with pytest.raises(ConfigError):
        load_config(write(tmp_path, "depth_is_radial: [1]\n"))
