"""Unit tests for the pi_cameras launch helper (nodes/pi_cameras/launch/pi_cameras_config.py).

The helper is pure Python (no ROS imports): settings merge, front/back selection from the CSI port, and the camera_ros
parameter dictionaries of the two Camera Module 3 (IMX708) previews.
"""

import sys
from pathlib import Path

import pytest

LAUNCH_DIR = Path(__file__).resolve().parent.parent / "nodes" / "pi_cameras" / "launch"
PARAMS_FILE = LAUNCH_DIR.parent / "config" / "params.yaml"
sys.path.insert(0, str(LAUNCH_DIR))

import pi_cameras_config as cfg  # noqa: E402

CAM0_ID = "/base/axi/pcie@120000/rp1/i2c@88000/imx708@1a"
CAM1_ID = "/base/axi/pcie@120000/rp1/i2c@80000/imx708@1a"


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


def test_defaults_put_cam0_in_front_with_the_pi5_libcamera_ids() -> None:
    launch = defaults()["launch"]
    assert launch["front_port"] == "cam0"
    assert launch["camera_ids"] == {"cam0": CAM0_ID, "cam1": CAM1_ID}


def test_defaults_are_a_cheap_640x480_10fps_preview() -> None:
    camera = defaults()["camera"]
    assert (camera["width"], camera["height"]) == (640, 480)
    assert camera["FrameDurationLimits"] == [100000, 100000]
    assert defaults()["compressed"]["jpeg_quality"] <= 80


def test_override_file_is_merged_over_the_defaults(tmp_path: Path) -> None:
    override = tmp_path / "config.yaml"
    override.write_text("front_port: cam1\ncamera:\n  width: 320\n")
    settings = cfg.load_settings(PARAMS_FILE, override)
    assert settings["launch"]["front_port"] == "cam1"
    assert settings["camera"]["width"] == 320
    assert settings["camera"]["height"] == 480


def test_empty_override_file_keeps_the_defaults(tmp_path: Path) -> None:
    override = tmp_path / "config.yaml"
    override.write_text("")
    assert cfg.load_settings(PARAMS_FILE, override) == defaults()


def test_cam0_front_maps_cam0_to_front_and_cam1_to_back() -> None:
    assert cfg.role_camera_ids(defaults()) == {"front": CAM0_ID, "back": CAM1_ID}


def test_front_port_cam1_swaps_the_roles() -> None:
    settings = cfg.deep_merge(defaults(), {"launch": {"front_port": "cam1"}})
    assert cfg.role_camera_ids(settings) == {"front": CAM1_ID, "back": CAM0_ID}


def test_unknown_front_port_is_rejected() -> None:
    settings = cfg.deep_merge(defaults(), {"launch": {"front_port": "cam2"}})
    with pytest.raises(ValueError, match="front_port"):
        cfg.role_camera_ids(settings)


def test_namespaces_are_not_the_overview_camera_namespace() -> None:
    assert cfg.role_namespace("front") == "/front_camera"
    assert cfg.role_namespace("back") == "/back_camera"


def test_camera_parameters_select_the_camera_and_compress_the_stream() -> None:
    params = cfg.camera_parameters(defaults(), "front", CAM0_ID)
    assert params["camera"] == CAM0_ID
    assert params["frame_id"] == "front_camera_optical_frame"
    assert params["width"] == 640 and params["height"] == 480
    assert params["FrameDurationLimits"] == [100000, 100000]
    assert params[cfg.JPEG_QUALITY_PARAMETER] == defaults()["compressed"]["jpeg_quality"]
    assert "SyncMode" not in params


def test_back_camera_has_its_own_frame_id() -> None:
    assert cfg.camera_parameters(defaults(), "back", CAM1_ID)["frame_id"] == "back_camera_optical_frame"


def test_unknown_role_is_rejected() -> None:
    with pytest.raises(ValueError, match="role"):
        cfg.role_namespace("left")


def test_defaults_launch_both_roles() -> None:
    assert cfg.enabled_roles(defaults()) == ("front", "back")


def test_flat_roles_override_launches_only_the_listed_cameras(tmp_path: Path) -> None:
    override = tmp_path / "config.yaml"
    override.write_text("front_port: cam0\nroles: [front]\n")
    settings = cfg.load_settings(PARAMS_FILE, override)
    assert settings["launch"]["roles"] == ["front"]
    assert cfg.enabled_roles(settings) == ("front",)


def test_unknown_or_empty_roles_are_rejected() -> None:
    for roles in (["left"], []):
        settings = cfg.deep_merge(defaults(), {"launch": {"roles": roles}})
        with pytest.raises(ValueError, match="roles"):
            cfg.enabled_roles(settings)
