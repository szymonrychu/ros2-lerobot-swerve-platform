"""Calibration sample store and mount solver wiring."""

import json
import math

import numpy as np
import pytest
import yaml
from ros2_common.camera_geometry import CameraIntrinsics, MountPose, optical_from_mount, project_raw

from mcp_server.camera_calib import CalibrationError, SampleStore, mount_yaml, solve_samples

INTR = CameraIntrinsics.from_hfov(640, 480, 90.0)
TRUE = MountPose(parent_frame="base_link", x=0.25, y=0.02, z=0.8, roll=0.0, pitch=1.0, yaw=0.05)
GROUND = [(0.7, -0.3, 0.0), (0.9, 0.3, 0.0), (1.2, 0.0, 0.0), (0.6, 0.1, 0.0), (1.0, -0.2, 0.0), (0.8, 0.25, 0.0)]


def pixel_of(ground: tuple[float, float, float], mount: MountPose = TRUE) -> tuple[float, float]:
    t_opt = optical_from_mount(mount.to_matrix())
    u, v, _ = project_raw(INTR, t_opt, np.array(ground))
    return u, v


def test_add_creates_file_and_counts(tmp_path) -> None:
    store = SampleStore(tmp_path / "calib")
    n = store.add("front", "base_link", np.eye(4), (320.0, 240.0), (1.0, 0.0, 0.0))
    assert n == 1
    n = store.add("front", "base_link", np.eye(4), (300.0, 250.0), (1.1, 0.1, 0.0))
    assert n == 2
    data = json.loads((tmp_path / "calib" / "front.json").read_text())
    assert data["camera"] == "front" and data["parent_frame"] == "base_link"
    assert len(data["samples"]) == 2
    s = data["samples"][0]
    assert s["pixel"] == [320.0, 240.0] and s["ground"] == [1.0, 0.0, 0.0]
    assert np.allclose(np.array(s["t_frame_parent"]), np.eye(4)) and s["captured_at"]


def test_load_returns_solver_tuples_and_survives_new_store(tmp_path) -> None:
    SampleStore(tmp_path).add("gripper", "gripper_link", np.eye(4), (1.0, 2.0), (0.1, 0.2, -0.165))
    parent, samples = SampleStore(tmp_path).load("gripper")
    assert parent == "gripper_link"
    t, pix, ground = samples[0]
    assert np.allclose(t, np.eye(4)) and pix == (1.0, 2.0) and ground == (0.1, 0.2, -0.165)


def test_load_empty_and_clear(tmp_path) -> None:
    store = SampleStore(tmp_path)
    assert store.load("front") == (None, [])
    store.add("front", "base_link", np.eye(4), (1.0, 1.0), (1.0, 0.0, 0.0))
    assert store.clear("front") == 1
    assert store.clear("front") == 0
    assert store.load("front") == (None, [])


def test_cameras_are_stored_separately(tmp_path) -> None:
    store = SampleStore(tmp_path)
    store.add("front", "base_link", np.eye(4), (1.0, 1.0), (1.0, 0.0, 0.0))
    assert store.count("gripper") == 0 and store.count("front") == 1


def test_parent_frame_change_is_refused(tmp_path) -> None:
    store = SampleStore(tmp_path)
    store.add("gripper", "gripper_link", np.eye(4), (1.0, 1.0), (0.2, 0.0, -0.165))
    with pytest.raises(CalibrationError, match="clear"):
        store.add("gripper", "wrist_link", np.eye(4), (1.0, 1.0), (0.2, 0.0, -0.165))


def test_non_finite_values_are_refused(tmp_path) -> None:
    with pytest.raises(CalibrationError, match="finite"):
        SampleStore(tmp_path).add("front", "base_link", np.eye(4), (math.nan, 1.0), (1.0, 0.0, 0.0))


def test_unwritable_directory_is_a_calibration_error(tmp_path) -> None:
    blocker = tmp_path / "file"
    blocker.write_text("x")
    with pytest.raises(CalibrationError, match="cannot"):
        SampleStore(blocker / "sub").add("front", "base_link", np.eye(4), (1.0, 1.0), (1.0, 0.0, 0.0))


def test_corrupt_file_is_a_calibration_error(tmp_path) -> None:
    (tmp_path / "front.json").write_text("{not json")
    with pytest.raises(CalibrationError, match="corrupt"):
        SampleStore(tmp_path).load("front")


def test_solver_recovers_the_known_mount(tmp_path) -> None:
    store = SampleStore(tmp_path)
    for g in GROUND:
        store.add("front", "base_link", np.eye(4), pixel_of(g), g)
    parent, samples = store.load("front")
    guess = TRUE.model_copy(update={"x": 0.2, "z": 0.7, "pitch": 0.8, "yaw": 0.0})
    mount, rms = solve_samples(samples, INTR, guess)
    assert rms < 0.05
    for key in ("x", "y", "z", "roll", "pitch", "yaw"):
        assert getattr(mount, key) == pytest.approx(getattr(TRUE, key), abs=1e-3)
    assert mount.parent_frame == "base_link"


def test_solver_needs_three_samples() -> None:
    with pytest.raises(CalibrationError, match="at least 3"):
        solve_samples([(np.eye(4), (1.0, 1.0), (1.0, 0.0, 0.0))], INTR, TRUE)


def test_mount_yaml_pastes_under_cameras_with_rounded_floats() -> None:
    text = mount_yaml("front", TRUE.model_copy(update={"x": 0.123456789012}))
    data = yaml.safe_load(text)
    mount = data["cameras"]["front"]["mount"]
    assert mount["parent_frame"] == "base_link" and mount["x"] == 0.123457
    assert set(mount) == {"parent_frame", "x", "y", "z", "roll", "pitch", "yaw"}
