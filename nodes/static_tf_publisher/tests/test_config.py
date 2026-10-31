"""Tests for static TF config loading."""

import logging
from pathlib import Path

import pytest

from static_tf_publisher.config import load_config

CLIENT_FRAMES = """
parent_frame: base_link
frames:
  - child: imu_link
    x: 0.0
    y: 0.0
    z: 0.05
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
  - child: laser_frame
    x: 0.158
    y: 0.06
    z: 0.20
    roll: 0.0
    pitch: 0.0
    yaw: 3.14159265
"""


def write(tmp_path: Path, text: str) -> Path:
    """Write a config file.

    Args:
        tmp_path (Path): pytest temp dir.
        text (str): YAML text.

    Returns:
        Path: the written file.
    """
    path = tmp_path / "config.yaml"
    path.write_text(text)
    return path


def test_load_config_missing() -> None:
    assert load_config(Path("/nonexistent")) is None


def test_load_config_frames_list(tmp_path: Path) -> None:
    out = load_config(
        write(
            tmp_path,
            """
parent_frame: base_link
frames:
  - child: imu_link
    x: 0.0
    y: 0.0
    z: 0.0
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
  - child: laser_frame
    x: 0.1
    y: 0.0
    z: 0.0
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
""",
        )
    )
    assert out is not None
    assert len(out) == 2
    assert out[0].child_frame == "imu_link"
    assert out[1].child_frame == "laser_frame"
    assert out[1].x == 0.1


def test_complete_frames_load_unchanged(tmp_path: Path) -> None:
    out = load_config(write(tmp_path, CLIENT_FRAMES))
    assert out is not None
    by_child = {frame.child_frame: frame for frame in out}
    assert set(by_child) == {"imu_link", "laser_frame"}
    imu = by_child["imu_link"]
    assert (imu.parent_frame, imu.x, imu.y, imu.z, imu.roll, imu.pitch, imu.yaw) == (
        "base_link",
        0.0,
        0.0,
        0.05,
        0.0,
        0.0,
        0.0,
    )
    laser = by_child["laser_frame"]
    assert (laser.x, laser.y, laser.z, laser.roll, laser.pitch, laser.yaw) == (0.158, 0.06, 0.20, 0.0, 0.0, 3.14159265)


def test_frame_missing_a_pose_key_is_rejected(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    text = (
        CLIENT_FRAMES
        + """
  - child: no_z_link
    x: 0.1
    y: 0.2
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
  - child: no_yaw_link
    x: 0.1
    y: 0.2
    z: 0.3
    roll: 0.0
    pitch: 0.0
"""
    )
    with caplog.at_level(logging.ERROR, logger="static_tf_publisher.config"):
        out = load_config(write(tmp_path, text))
    assert out is not None
    assert [frame.child_frame for frame in out] == ["imu_link", "laser_frame"]
    errors = [record.getMessage() for record in caplog.records if record.levelno == logging.ERROR]
    assert len(errors) == 2
    assert "no_z_link" in errors[0] and "z" in errors[0].split("missing", 1)[1]
    assert "no_yaw_link" in errors[1] and "yaw" in errors[1].split("missing", 1)[1]
