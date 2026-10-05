"""Tests for msg_serializer — ROS2 message → JSON dict conversion."""

from __future__ import annotations

import base64
from unittest.mock import MagicMock

import numpy as np
import pytest

from web_ui.msg_serializer import extract_field_from_dict, msg_to_dict


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
    assert extract_field_from_dict(data, "linear_acceleration.x") == pytest.approx(9.8)


def test_extract_field_array_index() -> None:
    data = {"position": [0.1, 0.2, 0.3, 0.4]}
    assert extract_field_from_dict(data, "position[2]") == pytest.approx(0.3)


def test_extract_field_missing_returns_none() -> None:
    assert extract_field_from_dict({}, "does.not.exist") is None


def test_extract_field_out_of_bounds_returns_none() -> None:
    assert extract_field_from_dict({"position": [1.0]}, "position[5]") is None


def _make_depth_image_msg(width: int = 640, height: int = 480, fill_mm: int = 1000) -> MagicMock:
    """Return a mock 16UC1 depth Image message."""
    arr = np.full((height, width), fill_mm, dtype=np.uint16)
    msg = MagicMock()
    msg.__class__.__name__ = "Image"
    msg.encoding = "16UC1"
    msg.width = width
    msg.height = height
    msg.step = width * 2
    msg.data = arr.tobytes()
    return msg


def _make_rgb_image_msg(width: int = 640, height: int = 480) -> MagicMock:
    """Return a mock rgb8 Image message."""
    arr = np.zeros((height, width, 3), dtype=np.uint8)
    arr[:, :, 0] = 200
    msg = MagicMock()
    msg.__class__.__name__ = "Image"
    msg.encoding = "rgb8"
    msg.width = width
    msg.height = height
    msg.step = width * 3
    msg.data = arr.tobytes()
    return msg


def _make_camera_info_msg(width: int = 640, height: int = 480) -> MagicMock:
    """Return a mock CameraInfo message."""
    msg = MagicMock()
    msg.__class__.__name__ = "CameraInfo"
    msg.width = width
    msg.height = height
    msg.k = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
    return msg


def test_serialize_depth_image_returns_expected_keys() -> None:
    """16UC1 depth image returns depth_preview_b64, depth_b64, depth_width, depth_height."""
    result = msg_to_dict(_make_depth_image_msg(), topic="/camera/camera/depth/image_rect_raw")
    assert "depth_preview_b64" in result
    assert "depth_b64" in result
    assert result["depth_width"] == 160
    assert result["depth_height"] == 120
    assert result["depth_preview_b64"] is not None


def test_serialize_depth_image_raw_bytes_length() -> None:
    """Downscaled depth_b64 decodes to exactly 160*120*2 bytes (uint16 LE)."""
    result = msg_to_dict(_make_depth_image_msg(), topic="/camera/camera/depth/image_rect_raw")
    raw = base64.b64decode(result["depth_b64"])
    assert len(raw) == 160 * 120 * 2


def test_serialize_depth_image_values_preserved() -> None:
    """Downscaled depth values match the fill value from the source image."""
    fill_mm = 1500
    result = msg_to_dict(_make_depth_image_msg(fill_mm=fill_mm), topic="/camera/camera/depth/image_rect_raw")
    raw = base64.b64decode(result["depth_b64"])
    arr = np.frombuffer(raw, dtype=np.uint16)
    assert int(arr[0]) == fill_mm


def test_serialize_camera_info() -> None:
    """CameraInfo returns fx, fy, cx, cy, width, height from K matrix."""
    result = msg_to_dict(_make_camera_info_msg())
    assert result == {"fx": 600.0, "fy": 600.0, "cx": 320.0, "cy": 240.0, "width": 640, "height": 480}


def test_color_image_rgbd_topic_includes_color_small() -> None:
    """Color image on RGBD color topic includes color_small_b64."""
    result = msg_to_dict(_make_rgb_image_msg(), topic="/stereo/left/image_rect")
    assert "jpeg_b64" in result
    assert "color_small_b64" in result
    assert result["color_small_b64"] is not None


def test_color_image_non_rgbd_topic_no_color_small() -> None:
    """Color image on non-RGBD topic does not include color_small_b64."""
    result = msg_to_dict(_make_rgb_image_msg(), topic="/controller/camera_0/image_raw")
    assert "jpeg_b64" in result
    assert "color_small_b64" not in result


def test_serialize_depth_image_zero_depth_preserved_in_bytes() -> None:
    """Zero-depth pixels (invalid) are preserved as 0 in the raw depth_b64 bytes."""
    result = msg_to_dict(_make_depth_image_msg(fill_mm=0), topic="/camera/camera/depth/image_rect_raw")
    assert result["depth_b64"] is not None
    raw = base64.b64decode(result["depth_b64"])
    arr = np.frombuffer(raw, dtype=np.uint16)
    assert int(arr[0]) == 0


STEREO_COLOR = "/stereo/left/image_rect"
STEREO_DEPTH = "/stereo/depth/image_rect"
STEREO_INFO = "/stereo/depth/camera_info"


def _bgr_image_msg(width: int = 320, height: int = 240) -> MagicMock:
    """Return a mock raw bgr8 Image message."""
    arr = np.zeros((height, width, 3), dtype=np.uint8)
    arr[:, :, 2] = 200
    msg = MagicMock()
    msg.__class__.__name__ = "Image"
    msg.encoding = "bgr8"
    msg.width = width
    msg.height = height
    msg.step = width * 3
    msg.data = arr.tobytes()
    return msg


def test_stereo_color_topic_includes_color_small() -> None:
    """The stereo left colour topic feeds the RGBD mesh with a downscaled colour frame (rgb8, 320x240)."""
    result = msg_to_dict(_make_rgb_image_msg(320, 240), topic=STEREO_COLOR)
    assert result["jpeg_b64"] is not None
    assert result["color_small_b64"] is not None


def test_stereo_color_bgr8_raw_image_serializes() -> None:
    """A raw bgr8 Image at 320x240 serializes to JPEG with the colour mesh frame."""
    result = msg_to_dict(_bgr_image_msg(), topic=STEREO_COLOR)
    assert result["jpeg_b64"] is not None
    assert result["color_small_b64"] is not None
    assert "error" not in result


def test_stereo_depth_16uc1_320x240_serializes() -> None:
    """The 16UC1 stereo depth at 320x240 yields the preview and the 160x120 mesh grid."""
    result = msg_to_dict(_make_depth_image_msg(width=320, height=240), topic=STEREO_DEPTH)
    assert result["depth_preview_b64"] is not None
    assert (result["depth_width"], result["depth_height"]) == (160, 120)
    assert len(base64.b64decode(result["depth_b64"])) == 160 * 120 * 2
