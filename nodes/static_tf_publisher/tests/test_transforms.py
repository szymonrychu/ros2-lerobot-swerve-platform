"""Tests for rpy_to_quaternion and roll/pitch config parsing."""

import math
from pathlib import Path

import pytest

from .ros_stubs import install_ros_stubs

install_ros_stubs("geometry_msgs", "geometry_msgs.msg", "tf2_ros")

from static_tf_publisher.config import load_config  # noqa: E402
from static_tf_publisher.node import rpy_to_quaternion  # noqa: E402

TOL = 1e-9


def assert_quat(actual: tuple[float, ...], expected: tuple[float, ...]) -> None:
    assert actual == pytest.approx(expected, abs=TOL)


def test_yaw_only_unchanged() -> None:
    assert_quat(rpy_to_quaternion(0.0, 0.0, math.pi), (0.0, 0.0, 1.0, 0.0))
    yaw = 0.7
    assert_quat(rpy_to_quaternion(0.0, 0.0, yaw), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))


def test_roll_only() -> None:
    s = math.sqrt(0.5)
    assert_quat(rpy_to_quaternion(math.pi / 2, 0.0, 0.0), (s, 0.0, 0.0, s))


def test_pitch_only() -> None:
    s = math.sqrt(0.5)
    assert_quat(rpy_to_quaternion(0.0, math.pi / 2, 0.0), (0.0, s, 0.0, s))


def test_optical_rotation() -> None:
    assert_quat(rpy_to_quaternion(-math.pi / 2, 0.0, -math.pi / 2), (-0.5, 0.5, -0.5, 0.5))


def test_unit_norm() -> None:
    q = rpy_to_quaternion(0.3, -1.1, 2.5)
    assert math.sqrt(sum(c * c for c in q)) == pytest.approx(1.0, abs=TOL)


def write_cfg(tmp_path: Path, body: str) -> Path:
    path = tmp_path / "c.yaml"
    path.write_text(body)
    return path


FULL_POSE = "    x: 0.0\n    y: 0.0\n    z: 0.0\n"


def test_config_roll_pitch_present(tmp_path: Path) -> None:
    body = "frames:\n  - child: a\n" + FULL_POSE + "    roll: 0.1\n    pitch: -0.2\n    yaw: 0.3\n"
    out = load_config(write_cfg(tmp_path, body))
    assert out is not None
    assert (out[0].roll, out[0].pitch, out[0].yaw) == (0.1, -0.2, 0.3)


def test_config_roll_pitch_absent_is_rejected(tmp_path: Path) -> None:
    out = load_config(write_cfg(tmp_path, "frames:\n  - child: a\n" + FULL_POSE + "    yaw: 0.3\n"))
    assert out is None


@pytest.mark.parametrize("key", ["roll", "pitch"])
def test_config_roll_pitch_invalid(tmp_path: Path, key: str) -> None:
    other = "pitch" if key == "roll" else "roll"
    body = "frames:\n  - child: a\n" + FULL_POSE + f"    yaw: 0.0\n    {other}: 0.0\n    {key}: abc\n"
    with pytest.raises(ValueError):
        load_config(write_cfg(tmp_path, body))
