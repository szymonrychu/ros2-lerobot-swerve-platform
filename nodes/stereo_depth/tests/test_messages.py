"""Unit tests for the message conversion (no rclpy)."""

import struct

import numpy as np
import pytest

from stereo_depth.config import DepthConfig
from stereo_depth.messages import DepthConverter, disparity_to_array
from tests.conftest import CameraInfo, DisparityImage

WIDTH, HEIGHT = 3, 2


def make_disparity(values: list[float], big_endian: bool = False, step_padding: int = 0) -> DisparityImage:
    msg = DisparityImage()
    msg.header.stamp.sec, msg.header.stamp.nanosec = 12, 345
    msg.header.frame_id = "stereo_left_optical_frame"
    msg.f, msg.t = 400.0, 0.1
    msg.min_disparity = 0.0
    img = msg.image
    img.width, img.height, img.encoding = WIDTH, HEIGHT, "32FC1"
    img.is_bigendian = int(big_endian)
    img.step = WIDTH * 4 + step_padding
    fmt = (">" if big_endian else "<") + "f"
    rows = []
    for r in range(HEIGHT):
        row = b"".join(struct.pack(fmt, values[r * WIDTH + c]) for c in range(WIDTH))
        rows.append(row + b"\x00" * step_padding)
    img.data = b"".join(rows)
    return msg


def make_info() -> CameraInfo:
    info = CameraInfo()
    info.header.stamp.sec = 99
    info.header.frame_id = "other"
    info.width, info.height = WIDTH, HEIGHT
    info.p = [400.0, 0.0, 1.0, 0.0, 0.0, 400.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    return info


VALID = [40.0, 20.0, 0.0, 10.0, np.nan, 40.0]


def test_disparity_to_array_little_endian() -> None:
    arr = disparity_to_array(make_disparity(VALID))
    assert arr is not None and arr.shape == (HEIGHT, WIDTH)
    assert arr[0, 0] == 40.0 and np.isnan(arr[1, 1])


def test_disparity_to_array_big_endian_and_padded_rows() -> None:
    arr = disparity_to_array(make_disparity(VALID, big_endian=True, step_padding=4))
    assert arr is not None and arr[1, 2] == 40.0 and arr.shape == (HEIGHT, WIDTH)


def test_disparity_to_array_rejects_empty_and_wrong_encoding() -> None:
    empty = make_disparity(VALID)
    empty.image.data = b""
    assert disparity_to_array(empty) is None
    wrong = make_disparity(VALID)
    wrong.image.encoding = "16UC1"
    assert disparity_to_array(wrong) is None
    short = make_disparity(VALID)
    short.image.data = short.image.data[:-1]
    assert disparity_to_array(short) is None


def test_depth_image_layout_and_header() -> None:
    converter = DepthConverter(DepthConfig())
    result = converter.convert(make_disparity(VALID), make_info())
    assert result is not None
    depth, info = result
    assert (depth.width, depth.height) == (WIDTH, HEIGHT)
    assert depth.encoding == "16UC1" and depth.is_bigendian == 0
    assert depth.step == WIDTH * 2 and len(depth.data) == WIDTH * HEIGHT * 2
    assert (depth.header.stamp.sec, depth.header.stamp.nanosec) == (12, 345)
    assert depth.header.frame_id == "stereo_left_optical_frame"
    mm = np.frombuffer(depth.data, dtype="<u2").reshape(HEIGHT, WIDTH)
    assert mm.tolist() == [[1000, 2000, 0], [4000, 0, 1000]]


def test_camera_info_carries_depth_stamp_and_frame_without_mutating_input() -> None:
    converter = DepthConverter(DepthConfig())
    source = make_info()
    result = converter.convert(make_disparity(VALID), source)
    assert result is not None and result[1] is not None
    info = result[1]
    assert (info.header.stamp.sec, info.header.stamp.nanosec) == (12, 345)
    assert info.header.frame_id == "stereo_left_optical_frame"
    assert info.p == source.p and (info.width, info.height) == (WIDTH, HEIGHT)
    assert source.header.stamp.sec == 99 and source.header.frame_id == "other"


def test_no_camera_info_still_gives_depth_but_no_info() -> None:
    result = DepthConverter(DepthConfig()).convert(make_disparity(VALID), None)
    assert result is not None and result[1] is None


def test_nothing_before_first_valid_disparity() -> None:
    converter = DepthConverter(DepthConfig())
    invalid = make_disparity([0.0, np.nan, -1.0, np.inf, 0.0, 0.0])
    assert converter.convert(invalid, make_info()) is None
    empty = make_disparity(VALID)
    empty.image.data = b""
    assert converter.convert(empty, make_info()) is None
    assert converter.convert(make_disparity(VALID), make_info()) is not None
    # After the first valid frame a fully invalid frame is a real "no reading" frame and is published.
    after = converter.convert(invalid, make_info())
    assert after is not None
    assert not np.frombuffer(after[0].data, dtype="<u2").any()


def test_non_positive_f_or_t_is_not_converted() -> None:
    msg = make_disparity(VALID)
    msg.t = 0.0
    assert DepthConverter(DepthConfig()).convert(msg, make_info()) is None


def test_config_limits_are_applied() -> None:
    converter = DepthConverter(DepthConfig(max_depth_m=1.5))
    result = converter.convert(make_disparity(VALID), make_info())
    assert result is not None
    assert np.frombuffer(result[0].data, dtype="<u2").tolist() == [1000, 0, 0, 0, 0, 1000]
    with pytest.raises(ValueError):
        DepthConfig(min_depth_m=2.0, max_depth_m=1.0)
