"""Unit tests for the overview_camera launch helper (nodes/overview_camera/launch/overview_camera_config.py).

The helper is pure Python (no ROS imports): settings merge, camera selection by libcamera ID or index, the
calibration gate of camera_info_url and the camera_ros parameter dictionaries of the left (overview) and right IMX219.
"""

import sys
from pathlib import Path

import pytest

LAUNCH_DIR = Path(__file__).resolve().parent.parent / "nodes" / "overview_camera" / "launch"
PARAMS_FILE = LAUNCH_DIR.parent / "config" / "params.yaml"
sys.path.insert(0, str(LAUNCH_DIR))

import overview_camera_config as cfg  # noqa: E402

LEFT_ID = "/base/axi/pcie@1000120000/rp1/i2c@88000/imx219@10"
RIGHT_ID = "/base/axi/pcie@1000120000/rp1/i2c@80000/imx219@10"
VALID_P = [400.0, 0.0, 240.0, 0.0, 0.0, 400.0, 180.0, 0.0, 0.0, 0.0, 1.0, 0.0]


def defaults() -> dict:
    """Repo defaults from params.yaml.

    Returns:
        dict: Parsed settings.
    """
    return cfg.load_settings(PARAMS_FILE, Path("/nonexistent/config.yaml"))


def write_camera_info(path: Path, projection: list) -> Path:
    """Write a minimal camera_info YAML with the given projection matrix.

    Args:
        path: Target file.
        projection: projection_matrix.data.

    Returns:
        Path: The written path.
    """
    path.write_text(
        f"image_width: 480\nimage_height: 360\nprojection_matrix:\n  rows: 3\n  cols: 4\n  data: {projection}\n"
    )
    return path


def test_deep_merge_overrides_nested_keys_without_touching_the_base() -> None:
    base = {"a": {"b": 1, "c": 2}, "d": 3}
    merged = cfg.deep_merge(base, {"a": {"b": 9}, "e": 4})
    assert merged == {"a": {"b": 9, "c": 2}, "d": 3, "e": 4}
    assert base == {"a": {"b": 1, "c": 2}, "d": 3}


def test_load_settings_returns_defaults_when_the_override_is_missing_or_empty(tmp_path: Path) -> None:
    empty = tmp_path / "config.yaml"
    empty.write_text("")
    assert cfg.load_settings(PARAMS_FILE, empty) == defaults()
    assert defaults()["camera"]["width"] == 480


def test_load_settings_applies_the_deployed_override(tmp_path: Path) -> None:
    override = tmp_path / "config.yaml"
    override.write_text(f"launch:\n  left_camera_id: {LEFT_ID}\nleft:\n  jpeg_quality: 60\n")
    settings = cfg.load_settings(PARAMS_FILE, override)
    assert settings["launch"]["left_camera_id"] == LEFT_ID
    assert settings["left"]["jpeg_quality"] == 60
    assert settings["camera"]["width"] == 480


def test_load_settings_accepts_top_level_camera_ids_in_the_override(tmp_path: Path) -> None:
    """The Ansible `config: |` block is flat: left_camera_id, right_camera_id."""
    override = tmp_path / "config.yaml"
    override.write_text(f"left_camera_id: {LEFT_ID}\nright_camera_id: {RIGHT_ID}\n")
    launch = cfg.load_settings(PARAMS_FILE, override)["launch"]
    assert launch["left_camera_id"] == LEFT_ID and launch["right_camera_id"] == RIGHT_ID


def test_camera_selector_prefers_the_libcamera_id() -> None:
    assert cfg.camera_selector(LEFT_ID, 0) == (LEFT_ID, None)
    assert cfg.camera_selector(RIGHT_ID, 1) == (RIGHT_ID, None)


@pytest.mark.parametrize("index", [0, 1])
def test_camera_selector_falls_back_to_the_side_index_with_a_warning(index: int) -> None:
    value, warning = cfg.camera_selector("", index)
    assert value == index and isinstance(value, int)
    assert warning is not None and "cam -l" in warning and f"index {index}" in warning


def test_camera_selector_treats_whitespace_as_unset() -> None:
    assert cfg.camera_selector("   ", 1)[0] == 1


def test_side_indexes_keep_left_on_cam0_and_right_on_cam1() -> None:
    assert cfg.SIDE_INDEX == {"left": 0, "right": 1}


@pytest.mark.parametrize(
    "projection",
    [None, [], [0.0] * 12, VALID_P[:11], [*VALID_P[:5], 0.0, *VALID_P[6:]], ["x"] * 12, [float("nan"), *VALID_P[1:]]],
)
def test_projection_is_valid_rejects_missing_short_zero_focal_or_non_numeric_matrices(projection: object) -> None:
    assert not cfg.projection_is_valid(projection)


def test_projection_is_valid_accepts_a_real_projection() -> None:
    assert cfg.projection_is_valid(VALID_P)


def test_calibration_problem_names_a_missing_file(tmp_path: Path) -> None:
    problem = cfg.calibration_problem(tmp_path / "stereo_left.yaml")
    assert problem is not None and "stereo_left.yaml" in problem and "missing" in problem


def test_calibration_problem_names_a_zero_projection(tmp_path: Path) -> None:
    path = write_camera_info(tmp_path / "stereo_right.yaml", [0.0] * 12)
    problem = cfg.calibration_problem(path)
    assert problem is not None and "projection" in problem


def test_calibration_problem_names_invalid_yaml(tmp_path: Path) -> None:
    path = tmp_path / "stereo_left.yaml"
    path.write_text("a: [unclosed\n")
    problem = cfg.calibration_problem(path)
    assert problem is not None and "YAML" in problem


def test_calibration_problem_is_none_for_a_valid_file(tmp_path: Path) -> None:
    assert cfg.calibration_problem(write_camera_info(tmp_path / "stereo_left.yaml", VALID_P)) is None


def test_calibration_path_uses_the_stereo_depth_file_names(tmp_path: Path) -> None:
    assert cfg.calibration_path(tmp_path, "left") == tmp_path / "stereo_left.yaml"
    assert cfg.calibration_path(tmp_path, "right") == tmp_path / "stereo_right.yaml"


def test_camera_info_url_is_empty_with_a_reason_when_the_calibration_is_unusable(tmp_path: Path) -> None:
    url, reason = cfg.camera_info_url(tmp_path, "left")
    assert url == ""
    assert reason is not None and "stereo_left.yaml" in reason


def test_camera_info_url_points_at_a_valid_calibration_file(tmp_path: Path) -> None:
    write_camera_info(tmp_path / "stereo_right.yaml", VALID_P)
    url, reason = cfg.camera_info_url(tmp_path, "right")
    assert url == f"file://{tmp_path / 'stereo_right.yaml'}"
    assert reason is None


def test_left_camera_parameters_keep_the_overview_role() -> None:
    params = cfg.camera_parameters(defaults(), "left", LEFT_ID, "")
    assert params["camera"] == LEFT_ID
    assert params["sensor_mode"] == "1640:1232"
    assert params["width"] == 480 and params["height"] == 360
    assert params["role"] == "video" and params["format"] == "RGB888"
    assert params["FrameDurationLimits"] == [200000, 200000]
    assert params["AeExposureMode"] == 1
    assert params["SyncMode"] == 1
    assert params["frame_id"] == "overview_camera_optical_frame"
    assert params["image_raw.compressed.jpeg_quality"] == 80
    assert params["camera_info_url"] == ""
    assert "AfMode" not in params
    assert "jpeg_quality" not in params and "namespace" not in params and "sync_mode" not in params


def test_right_camera_parameters_are_the_sync_client() -> None:
    url = "file:///var/lib/ros2/camera_calibration/stereo_right.yaml"
    params = cfg.camera_parameters(defaults(), "right", 1, url)
    assert params["camera"] == 1
    assert params["SyncMode"] == 2
    assert params["frame_id"] == "stereo_right_optical_frame"
    assert params["camera_info_url"] == url
    assert params["width"] == 480 and params["height"] == 360
    assert params["FrameDurationLimits"] == [200000, 200000]
    assert "AfMode" not in params


def test_namespaces_keep_the_overview_topics_on_the_left() -> None:
    settings = defaults()
    assert cfg.side_namespace(settings, "left") == "/overview_camera"
    assert cfg.side_namespace(settings, "right") == "/stereo/right"


@pytest.mark.parametrize(
    ("table", "name", "value"),
    [
        (cfg.SYNC_MODES, "off", 0),
        (cfg.SYNC_MODES, "server", 1),
        (cfg.SYNC_MODES, "Client", 2),
        (cfg.AE_EXPOSURE_MODES, "normal", 0),
        (cfg.AE_EXPOSURE_MODES, "short", 1),
        (cfg.AE_EXPOSURE_MODES, "long", 2),
    ],
)
def test_enum_names_map_to_the_libcamera_values(table: dict, name: str, value: int) -> None:
    assert cfg.enum_value(table, name, "setting") == value


def test_camera_parameters_reject_an_unknown_sync_mode() -> None:
    settings = defaults()
    settings["right"]["sync_mode"] = "bogus"
    with pytest.raises(ValueError, match="sync_mode"):
        cfg.camera_parameters(settings, "right", 1, "")
