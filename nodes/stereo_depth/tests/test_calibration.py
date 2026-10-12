"""Unit tests for the stereo calibration files: load, validate, write (round trip) and rectification maps."""

import math
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from stereo_depth.calibration import (
    LEFT_FILE,
    RIGHT_FILE,
    STEREO_FILE,
    CalibrationError,
    calibration_signature,
    camera_info_yaml,
    load_camera_info_yaml,
    load_stereo_calibration,
    rectification_maps,
    rectify,
    stereo_calibration_from_opencv,
    write_stereo_calibration,
)
from tests.conftest import BASELINE_M, CX, CY, FX, HEIGHT, WIDTH, ideal_calibration, textured_image

ROS_LEFT_YAML = """\
image_width: 480
image_height: 360
camera_name: narrow_stereo/left
camera_matrix:
  rows: 3
  cols: 3
  data: [400.0, 0.0, 240.0, 0.0, 400.0, 180.0, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.0, 0.0, 0.0, 0.0, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [400.0, 0.0, 240.0, 0.0, 0.0, 400.0, 180.0, 0.0, 0.0, 0.0, 1.0, 0.0]
"""


def opencv_calibration() -> tuple:
    """Run cv2.stereoRectify (alpha 0) on a known, slightly imperfect rig, as the calibration tool does.

    Returns:
        tuple: (StereoCalibration, T vector) for the rig.
    """
    k1 = np.array([[402.0, 0.0, 238.0], [0.0, 401.0, 182.0], [0.0, 0.0, 1.0]])
    k2 = np.array([[398.0, 0.0, 243.0], [0.0, 399.0, 178.0], [0.0, 0.0, 1.0]])
    d1 = np.array([-0.05, 0.01, 0.0, 0.0, 0.0])
    d2 = np.array([-0.04, 0.008, 0.0, 0.0, 0.0])
    r, _ = cv2.Rodrigues(np.array([0.002, -0.01, 0.003]))
    t = np.array([-BASELINE_M, 0.0005, 0.001])
    r1, r2, p1, p2, q, _, _ = cv2.stereoRectify(k1, d1, k2, d2, (WIDTH, HEIGHT), r, t, alpha=0)
    calibration = stereo_calibration_from_opencv(WIDTH, HEIGHT, k1, d1, k2, d2, r, t, r1, r2, p1, p2, q, rms=0.25)
    return calibration, t


def test_round_trip_of_the_ideal_calibration(tmp_path: Path) -> None:
    calibration = ideal_calibration()
    paths = write_stereo_calibration(tmp_path, calibration)
    assert sorted(p.name for p in paths) == sorted([LEFT_FILE, RIGHT_FILE, STEREO_FILE])
    loaded = load_stereo_calibration(tmp_path)
    assert loaded.reason == ""
    assert loaded.calibration == calibration
    assert sorted(p.name for p in tmp_path.iterdir()) == sorted([LEFT_FILE, RIGHT_FILE, STEREO_FILE])


def test_derived_values() -> None:
    calibration = ideal_calibration()
    assert calibration.fx == FX
    assert calibration.baseline_m == pytest.approx(BASELINE_M)
    assert (calibration.width, calibration.height) == (WIDTH, HEIGHT)


def test_written_files_use_the_ros_camera_info_layout(tmp_path: Path) -> None:
    write_stereo_calibration(tmp_path, ideal_calibration())
    left = yaml.safe_load((tmp_path / LEFT_FILE).read_text())
    assert (left["image_width"], left["image_height"], left["camera_name"]) == (WIDTH, HEIGHT, "stereo_left")
    assert left["camera_matrix"] == {"rows": 3, "cols": 3, "data": [FX, 0.0, CX, 0.0, FX, CY, 0.0, 0.0, 1.0]}
    assert left["distortion_model"] == "plumb_bob"
    assert left["distortion_coefficients"]["rows"] == 1 and left["distortion_coefficients"]["cols"] == 5
    assert left["rectification_matrix"]["rows"] == 3
    assert left["projection_matrix"]["cols"] == 4
    right = yaml.safe_load((tmp_path / RIGHT_FILE).read_text())
    assert right["projection_matrix"]["data"][3] == pytest.approx(-FX * BASELINE_M)
    stereo = yaml.safe_load((tmp_path / STEREO_FILE).read_text())
    assert (stereo["image_width"], stereo["image_height"], stereo["rms"]) == (WIDTH, HEIGHT, 0.2)
    assert stereo["R"]["rows"] == 3 and stereo["T"] == {"rows": 3, "cols": 1, "data": [-BASELINE_M, 0.0, 0.0]}
    assert stereo["Q"]["rows"] == 4 and len(stereo["Q"]["data"]) == 16


def test_camera_info_yaml_matches_the_written_file(tmp_path: Path) -> None:
    calibration = ideal_calibration()
    write_stereo_calibration(tmp_path, calibration)
    assert yaml.safe_load((tmp_path / LEFT_FILE).read_text()) == camera_info_yaml(calibration.left)


def test_loads_a_ros_cameracalibrator_file(tmp_path: Path) -> None:
    path = tmp_path / "left.yaml"
    path.write_text(ROS_LEFT_YAML)
    camera = load_camera_info_yaml(path)
    assert (camera.width, camera.height, camera.camera_name) == (480, 360, "narrow_stereo/left")
    assert camera.p[0] == 400.0 and len(camera.d) == 5


def test_opencv_calibration_round_trips_and_keeps_the_baseline(tmp_path: Path) -> None:
    calibration, t = opencv_calibration()
    assert calibration.baseline_m == pytest.approx(float(np.linalg.norm(t)), rel=0.01)
    write_stereo_calibration(tmp_path, calibration)
    loaded = load_stereo_calibration(tmp_path)
    assert loaded.reason == "" and loaded.calibration is not None
    assert loaded.calibration.baseline_m == pytest.approx(calibration.baseline_m)
    assert loaded.calibration.extrinsics.rms == 0.25


def test_missing_directory_and_files_give_none_with_the_reason(tmp_path: Path) -> None:
    missing = load_stereo_calibration(tmp_path / "nope")
    assert missing.calibration is None and LEFT_FILE in missing.reason
    write_stereo_calibration(tmp_path, ideal_calibration())
    (tmp_path / STEREO_FILE).unlink()
    loaded = load_stereo_calibration(tmp_path)
    assert loaded.calibration is None and STEREO_FILE in loaded.reason and "missing" in loaded.reason


def mutate(tmp_path: Path, name: str, key: str, index: int | None, value: object) -> str:
    """Write the ideal calibration, change one value in one file and return the loader's reason.

    Args:
        tmp_path (Path): Calibration directory.
        name (str): File to change.
        key (str): Top-level key.
        index (int | None): Index into the key's data list, or None to replace the key's value.
        value (object): New value.

    Returns:
        str: The reason; the calibration must be None.
    """
    write_stereo_calibration(tmp_path, ideal_calibration())
    path = tmp_path / name
    data = yaml.safe_load(path.read_text())
    if index is None:
        data[key] = value
    else:
        data[key]["data"][index] = value
    path.write_text(yaml.safe_dump(data))
    loaded = load_stereo_calibration(tmp_path)
    assert loaded.calibration is None
    assert loaded.reason
    return loaded.reason


@pytest.mark.parametrize(
    ("name", "key", "index", "value", "expected"),
    [
        (LEFT_FILE, "projection_matrix", 0, 0.0, "focal"),
        (LEFT_FILE, "projection_matrix", 5, 0.0, "focal"),
        (LEFT_FILE, "camera_matrix", 0, -1.0, "focal"),
        (LEFT_FILE, "projection_matrix", 3, -5.0, "left"),
        (RIGHT_FILE, "projection_matrix", 3, 0.0, "baseline"),
        (RIGHT_FILE, "projection_matrix", 3, 10.0, "baseline"),
        (RIGHT_FILE, "projection_matrix", 0, 380.0, "rectified"),
        (RIGHT_FILE, "projection_matrix", 6, 150.0, "rectified"),
        (RIGHT_FILE, "image_width", None, 640, "size"),
        (STEREO_FILE, "image_height", None, 480, "size"),
        (STEREO_FILE, "rms", None, -0.1, "rms"),
        (STEREO_FILE, "T", 0, -0.2, "baseline"),
        (LEFT_FILE, "distortion_model", None, "equidistant", "distortion_model"),
        (LEFT_FILE, "projection_matrix", 2, float("nan"), "finite"),
        (LEFT_FILE, "image_width", None, 0, "image_width"),
    ],
)
def test_invalid_values_give_none_with_the_reason(
    tmp_path: Path, name: str, key: str, index: int | None, value: object, expected: str
) -> None:
    assert expected in mutate(tmp_path, name, key, index, value)


def test_wrong_matrix_shape_is_rejected(tmp_path: Path) -> None:
    write_stereo_calibration(tmp_path, ideal_calibration())
    path = tmp_path / LEFT_FILE
    data = yaml.safe_load(path.read_text())
    data["projection_matrix"]["data"] = data["projection_matrix"]["data"][:9]
    path.write_text(yaml.safe_dump(data))
    loaded = load_stereo_calibration(tmp_path)
    assert loaded.calibration is None and LEFT_FILE in loaded.reason


def test_invalid_yaml_and_non_mapping_are_rejected(tmp_path: Path) -> None:
    write_stereo_calibration(tmp_path, ideal_calibration())
    (tmp_path / RIGHT_FILE).write_text("a: [1\n")
    loaded = load_stereo_calibration(tmp_path)
    assert loaded.calibration is None and RIGHT_FILE in loaded.reason
    (tmp_path / RIGHT_FILE).write_text("- 1\n")
    loaded = load_stereo_calibration(tmp_path)
    assert loaded.calibration is None and "mapping" in loaded.reason
    with pytest.raises(CalibrationError):
        load_camera_info_yaml(tmp_path / RIGHT_FILE)


def test_maps_are_fixed_point_and_built_for_the_calibrated_size() -> None:
    maps = rectification_maps(ideal_calibration())
    for map1, map2 in (maps.left, maps.right):
        assert map1.dtype == np.int16 and map1.shape == (HEIGHT, WIDTH, 2)
        assert map2.dtype == np.uint16 and map2.shape == (HEIGHT, WIDTH)


def test_ideal_calibration_rectifies_to_the_same_image() -> None:
    maps = rectification_maps(ideal_calibration())
    left = textured_image(1)
    right = textured_image(2)
    rect_left, rect_right = rectify(maps, left, right)
    assert np.array_equal(rect_left, left)
    assert np.array_equal(rect_right, right)


def test_opencv_calibration_rectifies_rows_into_line() -> None:
    calibration, _ = opencv_calibration()
    maps = rectification_maps(calibration)
    # A 3-D point projects through each rectified P onto the same row after rectification.
    left_p = np.array(calibration.left.p).reshape(3, 4)
    right_p = np.array(calibration.right.p).reshape(3, 4)
    point = np.array([0.1, -0.05, 1.5, 1.0])
    ul, vl, wl = left_p @ point
    ur, vr, wr = right_p @ point
    assert math.isclose(vl / wl, vr / wr, abs_tol=1e-6)
    assert (ul / wl - ur / wr) == pytest.approx(calibration.fx * calibration.baseline_m / 1.5, rel=1e-6)
    assert maps.left[0].shape == (HEIGHT, WIDTH, 2)


def test_signature_changes_when_the_files_change(tmp_path: Path) -> None:
    empty = calibration_signature(tmp_path)
    write_stereo_calibration(tmp_path, ideal_calibration())
    written = calibration_signature(tmp_path)
    assert written != empty
    assert calibration_signature(tmp_path) == written
    write_stereo_calibration(tmp_path, ideal_calibration(rms=0.123456))
    assert calibration_signature(tmp_path) != written
