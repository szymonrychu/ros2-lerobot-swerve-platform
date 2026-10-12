"""ROS message conversion for stereo_depth (no rclpy, no cv_bridge; testable with stubbed message packages)."""

from typing import Any

import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo, Image
from stereo_msgs.msg import DisparityImage

from .calibration import StereoCalibration
from .depth import DISPARITY_SCALE, MIN_DISPARITY

DEPTH_ENCODING = "16UC1"
DISPARITY_ENCODING = "32FC1"
UINT16_BYTES = 2
FLOAT32_BYTES = 4
NS_PER_S = 1_000_000_000
RECTIFIED_DISTORTION_MODEL = "plumb_bob"
RECTIFIED_DISTORTION = [0.0] * 5
IDENTITY_R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
# Encoding -> (channels, cv2 conversion to grey; None when the image is grey already).
GREY_CONVERSIONS: dict[str, tuple[int, int | None]] = {
    "mono8": (1, None),
    "bgr8": (3, cv2.COLOR_BGR2GRAY),
    "rgb8": (3, cv2.COLOR_RGB2GRAY),
    "bgra8": (4, cv2.COLOR_BGRA2GRAY),
    "rgba8": (4, cv2.COLOR_RGBA2GRAY),
}


def stamp_ns(stamp: Any) -> int:
    """Convert a builtin_interfaces/Time to integer nanoseconds.

    Args:
        stamp (Any): Message stamp with sec and nanosec.

    Returns:
        int: Nanoseconds.
    """
    return int(stamp.sec) * NS_PER_S + int(stamp.nanosec)


def image_to_grey(msg: Image) -> np.ndarray | None:
    """Decode an 8-bit camera Image into a grey array (numpy view of Image.data, no cv_bridge).

    Args:
        msg (Image): mono8, bgr8, rgb8, bgra8 or rgba8 image; rows may be padded (step > width * channels).

    Returns:
        np.ndarray | None: uint8 (height, width), or None for an empty, unsupported or truncated image.
    """
    conversion = GREY_CONVERSIONS.get(msg.encoding)
    if conversion is None or msg.width == 0 or msg.height == 0:
        return None
    channels, code = conversion
    row_bytes = msg.width * channels
    if msg.step < row_bytes or len(msg.data) < msg.height * msg.step:
        return None
    raw = np.frombuffer(msg.data, dtype=np.uint8, count=msg.height * msg.step).reshape(msg.height, msg.step)
    pixels = raw[:, :row_bytes].reshape(msg.height, msg.width, channels)
    if code is None:
        return np.ascontiguousarray(pixels[:, :, 0])
    return cv2.cvtColor(np.ascontiguousarray(pixels), code)


def depth_image_msg(header: Any, depth_mm: np.ndarray) -> Image:
    """Build the 16UC1 depth Image (millimetres, 0 = invalid, little-endian).

    Args:
        header (Any): std_msgs/Header (left stamp, config frame_id).
        depth_mm (np.ndarray): uint16 (height, width).

    Returns:
        Image: The message.
    """
    msg = Image()
    msg.header = header
    msg.height, msg.width = depth_mm.shape
    msg.encoding = DEPTH_ENCODING
    msg.is_bigendian = 0
    msg.step = msg.width * UINT16_BYTES
    msg.data = depth_mm.astype("<u2").tobytes()
    return msg


def camera_info_msg(header: Any, calibration: StereoCalibration) -> CameraInfo:
    """Build the CameraInfo of the depth image: the rectified left camera (K from P, no distortion, identity R).

    Args:
        header (Any): The depth image's header.
        calibration (StereoCalibration): The loaded pair.

    Returns:
        CameraInfo: The message.
    """
    p = list(calibration.left.p)
    msg = CameraInfo()
    msg.header = header
    msg.width = calibration.width
    msg.height = calibration.height
    msg.distortion_model = RECTIFIED_DISTORTION_MODEL
    msg.d = list(RECTIFIED_DISTORTION)
    msg.k = [p[0], p[1], p[2], p[4], p[5], p[6], 0.0, 0.0, 1.0]
    msg.r = list(IDENTITY_R)
    msg.p = p
    return msg


def disparity_msg(
    header: Any, disparity: np.ndarray, calibration: StereoCalibration, num_disparities: int
) -> DisparityImage:
    """Build the stereo_msgs/DisparityImage (32FC1 pixels; unmatched pixels stay negative, below min_disparity).

    Args:
        header (Any): The depth image's header.
        disparity (np.ndarray): float32 disparity in pixels (height, width).
        calibration (StereoCalibration): The loaded pair (f and T).
        num_disparities (int): Matcher disparity range.

    Returns:
        DisparityImage: The message.
    """
    height, width = disparity.shape
    image = Image()
    image.header = header
    image.height, image.width = height, width
    image.encoding = DISPARITY_ENCODING
    image.is_bigendian = 0
    image.step = width * FLOAT32_BYTES
    image.data = disparity.astype("<f4").tobytes()
    msg = DisparityImage()
    msg.header = header
    msg.image = image
    msg.f = calibration.fx
    msg.t = calibration.baseline_m
    msg.valid_window.width, msg.valid_window.height = width, height
    msg.min_disparity = float(MIN_DISPARITY)
    msg.max_disparity = float(MIN_DISPARITY + num_disparities - 1)
    msg.delta_d = 1.0 / DISPARITY_SCALE
    return msg
