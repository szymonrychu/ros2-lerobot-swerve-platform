"""Unit tests for the message conversion (no rclpy): camera images in, depth, camera_info and disparity out."""

import numpy as np
import pytest

from stereo_depth.messages import (
    DEPTH_ENCODING,
    camera_info_msg,
    depth_image_msg,
    disparity_msg,
    image_to_grey,
    stamp_ns,
)
from tests.conftest import BASELINE_M, FX, Header, Image, Stamp, ideal_calibration, image_msg

GREY = np.arange(12, dtype=np.uint8).reshape(3, 4) * 20


def header() -> Header:
    """Header with a known stamp and frame.

    Returns:
        Header: The header.
    """
    h = Header()
    h.stamp = Stamp(12, 345)
    h.frame_id = "stereo_left_optical_frame"
    return h


@pytest.mark.parametrize("encoding", ["bgr8", "rgb8", "mono8"])
def test_grey_image_round_trips_through_each_encoding(encoding: str) -> None:
    grey = image_to_grey(image_msg(GREY, encoding))
    assert grey is not None and grey.dtype == np.uint8
    assert np.array_equal(grey, GREY)


def test_colour_conversion_weights_the_channels_by_encoding() -> None:
    msg = Image()
    msg.height, msg.width, msg.encoding, msg.step = 1, 1, "bgr8", 3
    msg.data = bytes([255, 0, 0])  # pure blue in bgr8
    blue = image_to_grey(msg)
    msg.encoding = "rgb8"  # the same bytes are pure red in rgb8
    red = image_to_grey(msg)
    assert blue is not None and red is not None
    assert int(blue[0, 0]) < int(red[0, 0])


def test_padded_rows_are_cropped() -> None:
    msg = image_msg(GREY, "bgr8")
    rows = [msg.data[r * 12 : (r + 1) * 12] + b"\xff\xff\xff\xff" for r in range(3)]
    msg.data = b"".join(rows)
    msg.step = 16
    grey = image_to_grey(msg)
    assert grey is not None and np.array_equal(grey, GREY)


def test_rgba_and_bgra_are_supported() -> None:
    for encoding in ("bgra8", "rgba8"):
        msg = Image()
        msg.height, msg.width, msg.encoding, msg.step = 3, 4, encoding, 16
        msg.data = np.repeat(GREY[:, :, None], 4, axis=2).tobytes()
        grey = image_to_grey(msg)
        assert grey is not None and np.array_equal(grey, GREY)


def test_unsupported_empty_and_short_images_are_rejected() -> None:
    wrong = image_msg(GREY, "bgr8")
    wrong.encoding = "16UC1"
    assert image_to_grey(wrong) is None
    empty = image_msg(GREY, "bgr8")
    empty.width = 0
    assert image_to_grey(empty) is None
    short = image_msg(GREY, "bgr8")
    short.data = short.data[:-1]
    assert image_to_grey(short) is None
    narrow = image_msg(GREY, "bgr8")
    narrow.step = 4
    assert image_to_grey(narrow) is None


def test_depth_image_layout_and_header() -> None:
    depth_mm = np.array([[1000, 0, 65535], [1, 2, 3]], dtype=np.uint16)
    msg = depth_image_msg(header(), depth_mm)
    assert (msg.width, msg.height) == (3, 2)
    assert msg.encoding == DEPTH_ENCODING == "16UC1" and msg.is_bigendian == 0
    assert msg.step == 6 and len(msg.data) == 12
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (12, 345)
    assert msg.header.frame_id == "stereo_left_optical_frame"
    assert np.frombuffer(msg.data, dtype="<u2").reshape(2, 3).tolist() == depth_mm.tolist()


def test_camera_info_is_the_rectified_left_camera() -> None:
    calibration = ideal_calibration()
    info = camera_info_msg(header(), calibration)
    assert info.header.frame_id == "stereo_left_optical_frame" and info.header.stamp.sec == 12
    assert (info.width, info.height) == (calibration.width, calibration.height)
    assert info.p == list(calibration.left.p)
    assert info.k == [info.p[0], info.p[1], info.p[2], info.p[4], info.p[5], info.p[6], 0.0, 0.0, 1.0]
    assert info.r == [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    assert info.distortion_model == "plumb_bob" and info.d == [0.0] * 5


def test_disparity_message() -> None:
    calibration = ideal_calibration()
    disparity = np.array([[20.0, -1.0], [12.5, 0.0]], dtype=np.float32)
    msg = disparity_msg(header(), disparity, calibration, num_disparities=48)
    assert msg.header.frame_id == "stereo_left_optical_frame"
    assert msg.f == FX and msg.t == pytest.approx(BASELINE_M)
    assert (msg.min_disparity, msg.max_disparity, msg.delta_d) == (0.0, 47.0, 1.0 / 16.0)
    img = msg.image
    assert (img.width, img.height, img.encoding, img.step) == (2, 2, "32FC1", 8)
    assert img.header.frame_id == "stereo_left_optical_frame"
    assert np.frombuffer(img.data, dtype="<f4").reshape(2, 2).tolist() == disparity.tolist()
    assert (msg.valid_window.width, msg.valid_window.height) == (2, 2)


def test_stamp_ns() -> None:
    assert stamp_ns(Stamp(3, 5)) == 3_000_000_005
