"""Tests for static TF config loading."""

import tempfile
from pathlib import Path

from static_tf_publisher.config import load_config


def test_load_config_missing() -> None:
    assert load_config(Path("/nonexistent")) is None


def test_load_config_frames_list() -> None:
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
        f.write(
            """
parent_frame: base_link
frames:
  - child: imu_link
    x: 0.0
    y: 0.0
    z: 0.0
    yaw: 0.0
  - child: laser_frame
    x: 0.1
    y: 0.0
    yaw: 0.0
"""
        )
        path = Path(f.name)
    try:
        out = load_config(path)
        assert out is not None
        assert len(out) == 2
        assert out[0].child_frame == "imu_link"
        assert out[1].child_frame == "laser_frame"
        assert out[1].x == 0.1
    finally:
        path.unlink(missing_ok=True)
