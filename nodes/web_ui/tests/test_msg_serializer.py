"""Tests for msg_serializer — ROS2 message → JSON dict conversion."""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from web_ui.msg_serializer import extractField_from_dict, msg_to_dict


def _make_imu() -> MagicMock:
    """Return a mock sensor_msgs/Imu message."""
    msg = MagicMock()
    msg.__class__.__name__ = "Imu"
    msg.__slots__ = ["_linear_acceleration", "_angular_velocity", "_orientation"]
    msg.linear_acceleration = MagicMock(__slots__=["_x", "_y", "_z"], x=1.0, y=2.0, z=3.0)
    msg.angular_velocity = MagicMock(__slots__=["_x", "_y", "_z"], x=0.1, y=0.2, z=0.3)
    msg.orientation = MagicMock(__slots__=["_x", "_y", "_z", "_w"], x=0.0, y=0.0, z=0.0, w=1.0)
    return msg


def test_msg_to_dict_imu() -> None:
    result = msg_to_dict(_make_imu())
    assert "linear_acceleration" in result
    assert result["linear_acceleration"]["x"] == 1.0


def test_extract_field_simple() -> None:
    data = {"linear_acceleration": {"x": 9.8, "y": 0.1, "z": 0.0}}
    assert extractField_from_dict(data, "linear_acceleration.x") == pytest.approx(9.8)


def test_extract_field_array_index() -> None:
    data = {"position": [0.1, 0.2, 0.3, 0.4]}
    assert extractField_from_dict(data, "position[2]") == pytest.approx(0.3)


def test_extract_field_missing_returns_none() -> None:
    assert extractField_from_dict({}, "does.not.exist") is None


def test_extract_field_out_of_bounds_returns_none() -> None:
    assert extractField_from_dict({"position": [1.0]}, "position[5]") is None
