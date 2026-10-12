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


def _jpeg_gray(b64: str) -> np.ndarray:
    """Decode a base64 JPEG to a BGR array."""
    import cv2

    return cv2.imdecode(np.frombuffer(base64.b64decode(b64), dtype=np.uint8), cv2.IMREAD_COLOR)


def _depth_row_msg(values_mm: list[int]) -> MagicMock:
    """Return a 16UC1 Image of one repeated row, 4 rows high."""
    arr = np.tile(np.array(values_mm, dtype=np.uint16), (4, 1))
    msg = MagicMock()
    msg.__class__.__name__ = "Image"
    msg.encoding = "16UC1"
    msg.width = arr.shape[1]
    msg.height = 4
    msg.step = arr.shape[1] * 2
    msg.data = arr.tobytes()
    return msg


def test_tof_range_depth_preview_is_greyscale_near_bright_far_dark_invalid_black() -> None:
    """With tof_range_m the depth preview is grey: near bright, far dark, 0 black; raw fields unchanged."""
    width = 64
    row = [500] * 16 + [2000] * 16 + [3900] * 16 + [0] * 16
    result = msg_to_dict(_depth_row_msg(row), topic="/tof/depth", tof_range_m=4.0)
    img = _jpeg_gray(result["depth_preview_b64"])
    assert np.abs(img[..., 0].astype(int) - img[..., 1].astype(int)).max() <= 3
    near, mid, far, invalid = (int(img[2, 8 + 16 * i, 1]) for i in range(4))
    assert near > mid > far > invalid
    assert near > 200 and invalid < 10 and width == img.shape[1]
    assert result["depth_width"] == 160 and result["depth_height"] == 120
    assert result["depth_b64"] and result["depth_preview_b64"]


def test_depth_beyond_tof_range_is_black() -> None:
    """Depth at or beyond the range clips to dark."""
    result = msg_to_dict(_depth_row_msg([6000] * 16), topic="/tof/depth", tof_range_m=4.0)
    assert int(_jpeg_gray(result["depth_preview_b64"])[2, 8, 1]) < 10


def test_depth_without_range_keeps_red_green_preview() -> None:
    """The rgbd tab path (no tof_range_m) keeps the red-green colormap."""
    result = msg_to_dict(_depth_row_msg([1000] * 16), topic="/stereo/depth/image_rect")
    img = _jpeg_gray(result["depth_preview_b64"])
    assert abs(int(img[2, 8, 2]) - int(img[2, 8, 1])) > 20


def _amp_msg(encoding: str, dtype: type, fill: int) -> MagicMock:
    arr = np.full((4, 8), fill, dtype=dtype)
    msg = MagicMock()
    msg.__class__.__name__ = "Image"
    msg.encoding = encoding
    msg.width = 8
    msg.height = 4
    msg.step = 8 * arr.itemsize
    msg.data = arr.tobytes()
    return msg


def test_mono8_amplitude_serialises_to_grey_jpeg() -> None:
    result = msg_to_dict(_amp_msg("mono8", np.uint8, 120), topic="/tof/amp")
    img = _jpeg_gray(result["jpeg_b64"])
    assert img.shape[:2] == (4, 8)
    assert abs(int(img[1, 1, 1]) - 120) <= 6


def test_mono16_amplitude_serialises_to_grey_jpeg_scaled_to_frame() -> None:
    """mono16 is scaled so the brightest value of the frame is white."""
    arr = np.zeros((4, 8), dtype=np.uint16)
    arr[:, :4] = 1000
    arr[:, 4:] = 4000
    msg = _amp_msg("mono16", np.uint16, 0)
    msg.data = arr.tobytes()
    img = _jpeg_gray(msg_to_dict(msg, topic="/tof/amp")["jpeg_b64"])
    assert img[2, 6, 1] > 240
    assert 40 < int(img[2, 1, 1]) < 90


@pytest.mark.parametrize("encoding", ["32FC1", "16UC3", "bayer_rggb8"])
def test_unknown_or_float_encoding_is_not_decoded_as_colour(encoding: str) -> None:
    """32FC1 and other unknown encodings return an error and no image."""
    msg = _amp_msg(encoding, np.float32, 1)
    result = msg_to_dict(msg, topic="/tof/amp")
    assert result["jpeg_b64"] is None
    assert "encoding" in result["error"]
