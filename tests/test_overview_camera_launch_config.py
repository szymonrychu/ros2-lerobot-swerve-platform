"""Unit tests for the overview_camera launch helper (nodes/overview_camera/launch/overview_camera_config.py).

The helper is pure Python (no ROS imports): settings merge, camera selection by libcamera ID or index and the
camera_ros parameter dictionary of the single IMX708 camera.
"""

import sys
from pathlib import Path

import pytest

LAUNCH_DIR = Path(__file__).resolve().parent.parent / "nodes" / "overview_camera" / "launch"
PARAMS_FILE = LAUNCH_DIR.parent / "config" / "params.yaml"
sys.path.insert(0, str(LAUNCH_DIR))

import overview_camera_config as cfg  # noqa: E402

CAMERA_ID = "/base/axi/pcie@1000120000/rp1/i2c@88000/imx708@1a"


def defaults() -> dict:
    """Repo defaults from params.yaml.

    Returns:
        dict: Parsed settings.
    """
    return cfg.load_settings(PARAMS_FILE, Path("/nonexistent/config.yaml"))


def test_deep_merge_overrides_nested_keys_without_touching_the_base() -> None:
    base = {"a": {"b": 1, "c": 2}, "d": 3}
    merged = cfg.deep_merge(base, {"a": {"b": 9}, "e": 4})
    assert merged == {"a": {"b": 9, "c": 2}, "d": 3, "e": 4}
    assert base == {"a": {"b": 1, "c": 2}, "d": 3}


def test_load_settings_returns_defaults_when_the_override_is_missing_or_empty(tmp_path: Path) -> None:
    empty = tmp_path / "config.yaml"
    empty.write_text("")
    assert cfg.load_settings(PARAMS_FILE, empty) == defaults()
    assert defaults()["camera"]["width"] == 640


def test_load_settings_applies_the_deployed_override(tmp_path: Path) -> None:
    override = tmp_path / "config.yaml"
    override.write_text(f"launch:\n  camera_id: {CAMERA_ID}\ncamera:\n  jpeg_quality: 60\n")
    settings = cfg.load_settings(PARAMS_FILE, override)
    assert settings["launch"]["camera_id"] == CAMERA_ID
    assert settings["camera"]["jpeg_quality"] == 60
    assert settings["camera"]["width"] == 640


def test_load_settings_accepts_a_top_level_camera_id_in_the_override(tmp_path: Path) -> None:
    """The Ansible `config: |` block is flat: camera_id."""
    override = tmp_path / "config.yaml"
    override.write_text(f"camera_id: {CAMERA_ID}\n")
    assert cfg.load_settings(PARAMS_FILE, override)["launch"]["camera_id"] == CAMERA_ID


def test_camera_selector_prefers_the_libcamera_id() -> None:
    assert cfg.camera_selector(CAMERA_ID) == (CAMERA_ID, None)


def test_camera_selector_falls_back_to_index_zero_with_a_warning() -> None:
    value, warning = cfg.camera_selector("")
    assert value == 0 and isinstance(value, int)
    assert warning is not None and "cam -l" in warning and "index 0" in warning


def test_camera_selector_treats_whitespace_as_unset() -> None:
    assert cfg.camera_selector("   ")[0] == 0


def test_camera_parameters_of_the_overhead_camera() -> None:
    params = cfg.camera_parameters(defaults(), CAMERA_ID)
    assert params["camera"] == CAMERA_ID
    assert params["width"] == 640 and params["height"] == 480
    assert params["FrameDurationLimits"] == [66666, 66666]
    assert params["AfMode"] == 2
    assert params["frame_id"] == "overview_camera_optical_frame"
    assert params["image_raw.compressed.jpeg_quality"] == 80


def test_camera_parameters_have_no_calibration_url_and_no_sync_mode() -> None:
    params = cfg.camera_parameters(defaults(), 0)
    assert params["camera"] == 0
    assert "camera_info_url" not in params
    assert "SyncMode" not in params
    assert "jpeg_quality" not in params, "the flat setting is translated to the image_transport parameter name"


@pytest.mark.parametrize(("name", "value"), [("manual", 0), ("auto", 1), ("continuous", 2), ("Continuous", 2)])
def test_af_mode_names_map_to_the_libcamera_enum(name: str, value: int) -> None:
    settings = defaults()
    settings["camera"]["AfMode"] = name
    assert cfg.camera_parameters(settings, 0)["AfMode"] == value


def test_camera_parameters_reject_an_unknown_af_mode() -> None:
    settings = defaults()
    settings["camera"]["AfMode"] = "bogus"
    with pytest.raises(ValueError, match="AfMode"):
        cfg.camera_parameters(settings, 0)
