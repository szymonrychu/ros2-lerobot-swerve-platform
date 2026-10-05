"""Unit tests for the stereo_camera launch helper (nodes/stereo_camera/launch/stereo_camera_config.py).

The helper is pure Python (no ROS imports): settings merge, camera selection, calibration gating of the
rectify/disparity stages, and the camera_ros / DisparityNode parameter dictionaries.
"""

import sys
from pathlib import Path

import pytest
import yaml

LAUNCH_DIR = Path(__file__).resolve().parent.parent / "nodes" / "stereo_camera" / "launch"
PARAMS_FILE = LAUNCH_DIR.parent / "config" / "params.yaml"
sys.path.insert(0, str(LAUNCH_DIR))

import stereo_camera_config as cfg  # noqa: E402

GOOD_P = [250.0, 0.0, 160.0, 0.0, 0.0, 250.0, 120.0, 0.0, 0.0, 0.0, 1.0, 0.0]
GOOD_P_RIGHT = [250.0, 0.0, 160.0, -20.0, 0.0, 250.0, 120.0, 0.0, 0.0, 0.0, 1.0, 0.0]


def write_camera_info(path: Path, projection: list[float] | None) -> Path:
    """Write a camera_info YAML with the given projection matrix.

    Args:
        path: Target file.
        projection: 12 values of P, or None to omit the projection_matrix key.

    Returns:
        Path: The written file.
    """
    doc: dict = {"image_width": 320, "image_height": 240, "camera_name": "x", "distortion_model": "plumb_bob"}
    if projection is not None:
        doc["projection_matrix"] = {"rows": 3, "cols": 4, "data": projection}
    path.write_text(yaml.safe_dump(doc))
    return path


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
    assert defaults()["camera"]["width"] == 320


def test_load_settings_applies_the_deployed_override(tmp_path: Path) -> None:
    override = tmp_path / "config.yaml"
    override.write_text("launch:\n  left_camera_id: /base/axi/pcie@120000/rp1/i2c@88000/imx219@10\n  publish_points: true\n")
    settings = cfg.load_settings(PARAMS_FILE, override)
    assert settings["launch"]["left_camera_id"].endswith("imx219@10")
    assert settings["launch"]["publish_points"] is True
    assert settings["launch"]["right_camera_id"] == ""


def test_load_settings_accepts_top_level_launch_keys_in_the_override(tmp_path: Path) -> None:
    """The Ansible `config: |` block is flat: left_camera_id / right_camera_id / publish_points."""
    override = tmp_path / "config.yaml"
    override.write_text("left_camera_id: L\nright_camera_id: R\npublish_points: true\n")
    launch = cfg.load_settings(PARAMS_FILE, override)["launch"]
    assert (launch["left_camera_id"], launch["right_camera_id"], launch["publish_points"]) == ("L", "R", True)


def test_camera_selector_prefers_the_libcamera_id() -> None:
    assert cfg.camera_selector("/base/imx219@10", 0) == ("/base/imx219@10", None)


def test_camera_selector_falls_back_to_the_index_with_a_warning() -> None:
    value, warning = cfg.camera_selector("", 1)
    assert value == 1 and isinstance(value, int)
    assert warning is not None and "cam -l" in warning and "index 1" in warning


def test_camera_selector_treats_whitespace_as_unset() -> None:
    assert cfg.camera_selector("   ", 0)[0] == 0


def test_projection_is_valid_requires_nonzero_finite_focal_lengths() -> None:
    assert cfg.projection_is_valid(GOOD_P) is True
    assert cfg.projection_is_valid([0.0] * 12) is False
    assert cfg.projection_is_valid(GOOD_P[:11]) is False
    assert cfg.projection_is_valid([float("nan")] + GOOD_P[1:]) is False
    assert cfg.projection_is_valid([0.0] + GOOD_P[1:]) is False
    assert cfg.projection_is_valid(None) is False


def test_calibration_ready_when_both_files_have_a_projection(tmp_path: Path) -> None:
    left = write_camera_info(tmp_path / "left.yaml", GOOD_P)
    right = write_camera_info(tmp_path / "right.yaml", GOOD_P_RIGHT)
    assert cfg.calibration_ready(left, right) == (True, [])


def test_calibration_not_ready_when_a_file_is_missing(tmp_path: Path) -> None:
    left = write_camera_info(tmp_path / "left.yaml", GOOD_P)
    ready, reasons = cfg.calibration_ready(left, tmp_path / "right.yaml")
    assert ready is False
    assert len(reasons) == 1 and "right.yaml" in reasons[0] and "missing" in reasons[0]


def test_calibration_not_ready_with_a_zero_projection(tmp_path: Path) -> None:
    left = write_camera_info(tmp_path / "left.yaml", [0.0] * 12)
    right = write_camera_info(tmp_path / "right.yaml", GOOD_P_RIGHT)
    ready, reasons = cfg.calibration_ready(left, right)
    assert ready is False and any("left.yaml" in r and "projection" in r for r in reasons)


def test_calibration_not_ready_without_a_projection_key_or_with_broken_yaml(tmp_path: Path) -> None:
    left = write_camera_info(tmp_path / "left.yaml", None)
    right = tmp_path / "right.yaml"
    right.write_text("not: [valid")
    ready, reasons = cfg.calibration_ready(left, right)
    assert ready is False and len(reasons) == 2


def test_calibration_not_ready_when_both_files_are_missing(tmp_path: Path) -> None:
    ready, reasons = cfg.calibration_ready(tmp_path / "left.yaml", tmp_path / "right.yaml")
    assert ready is False and len(reasons) == 2


def test_camera_info_url_is_a_file_url() -> None:
    assert cfg.camera_info_url(Path("/srv/repo/nodes/stereo_camera/calibration"), "left") == (
        "file:///srv/repo/nodes/stereo_camera/calibration/left.yaml"
    )


def test_camera_parameters_for_the_left_server_camera() -> None:
    settings = defaults()
    params = cfg.camera_parameters(settings, "left", "/base/imx219@10", "file:///c/left.yaml")
    assert params["camera"] == "/base/imx219@10"
    assert params["sensor_mode"] == "1640:1232"
    assert params["width"] == 320 and params["height"] == 240
    assert params["role"] == "video"
    assert params["FrameDurationLimits"] == [66666, 66666]
    assert params["AeExposureMode"] == 1
    assert params["SyncMode"] == 1
    assert params["frame_id"] == "stereo_left_optical_frame"
    assert params["camera_info_url"] == "file:///c/left.yaml"


def test_camera_parameters_for_the_right_client_camera_by_index() -> None:
    params = cfg.camera_parameters(defaults(), "right", 1, "file:///c/right.yaml")
    assert params["camera"] == 1
    assert params["SyncMode"] == 2
    assert params["frame_id"] == "stereo_right_optical_frame"


def test_camera_parameters_reject_unknown_enum_names() -> None:
    settings = defaults()
    settings["camera"]["AeExposureMode"] = "bogus"
    with pytest.raises(ValueError, match="AeExposureMode"):
        cfg.camera_parameters(settings, "left", 0, "file:///x")
    settings = defaults()
    settings["left"]["sync_mode"] = "master"
    with pytest.raises(ValueError, match="sync_mode"):
        cfg.camera_parameters(settings, "left", 0, "file:///x")


def test_disparity_parameters_have_the_node_types() -> None:
    params = cfg.disparity_parameters(defaults())
    assert params["stereo_algorithm"] == 1 and params["sgbm_mode"] == 2
    assert params["approximate_sync"] is True
    assert params["approximate_sync_tolerance_seconds"] == 0.002
    # DisparityNode declares P1, P2 and uniqueness_ratio as doubles and the rest as integers.
    for key in ("P1", "P2", "uniqueness_ratio", "approximate_sync_tolerance_seconds"):
        assert isinstance(params[key], float), key
    for key in ("correlation_window_size", "disparity_range", "min_disparity", "speckle_size", "prefilter_cap"):
        assert isinstance(params[key], int), key


def test_disparity_parameters_cast_integral_floats() -> None:
    settings = defaults()
    settings["disparity"]["P1"] = 200
    settings["disparity"]["speckle_size"] = 50.0
    params = cfg.disparity_parameters(settings)
    assert isinstance(params["P1"], float) and isinstance(params["speckle_size"], int)


def test_stage_plan_publishes_only_raw_images_without_calibration() -> None:
    assert cfg.stage_plan(calibrated=False, publish_points=True) == ["camera_left", "camera_right"]


def test_stage_plan_adds_rectify_and_disparity_with_calibration() -> None:
    assert cfg.stage_plan(calibrated=True, publish_points=False) == [
        "camera_left",
        "camera_right",
        "rectify_left",
        "rectify_right",
        "disparity",
    ]


def test_stage_plan_adds_the_point_cloud_only_on_request() -> None:
    plan = cfg.stage_plan(calibrated=True, publish_points=True)
    assert plan[-1] == "points"
    assert "points" not in cfg.stage_plan(calibrated=True, publish_points=False)
