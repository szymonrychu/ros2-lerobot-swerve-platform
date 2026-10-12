"""Doubles for rclpy, message_filters and the ROS2 message packages, so stereo_depth is tested without a ROS2 install.

They are registered in sys.modules at import time, before any test module imports stereo_depth.messages or
stereo_depth.node. A real ROS2 install is never needed (nor used) by the tests.
"""

import sys
import types
from typing import Any
from unittest.mock import MagicMock

import numpy as np

from stereo_depth.calibration import CameraCalibration, StereoCalibration, StereoExtrinsics

WIDTH = 480
HEIGHT = 360
FX = 400.0
CX = 240.0
CY = 180.0
BASELINE_M = 0.08


class Stamp:
    """builtin_interfaces/Time stand-in."""

    def __init__(self, sec: int = 0, nanosec: int = 0) -> None:
        self.sec = sec
        self.nanosec = nanosec


class Header:
    """std_msgs/Header stand-in."""

    def __init__(self) -> None:
        self.stamp = Stamp()
        self.frame_id = ""


class Image:
    """sensor_msgs/Image stand-in."""

    def __init__(self) -> None:
        self.header = Header()
        self.height = 0
        self.width = 0
        self.encoding = ""
        self.is_bigendian = 0
        self.step = 0
        self.data = b""


class RegionOfInterest:
    """sensor_msgs/RegionOfInterest stand-in."""

    def __init__(self) -> None:
        self.x_offset = 0
        self.y_offset = 0
        self.height = 0
        self.width = 0
        self.do_rectify = False


class CameraInfo:
    """sensor_msgs/CameraInfo stand-in."""

    def __init__(self) -> None:
        self.header = Header()
        self.height = 0
        self.width = 0
        self.distortion_model = ""
        self.d: list[float] = []
        self.k = [0.0] * 9
        self.r = [0.0] * 9
        self.p = [0.0] * 12
        self.binning_x = 0
        self.binning_y = 0
        self.roi = RegionOfInterest()


class DisparityImage:
    """stereo_msgs/DisparityImage stand-in."""

    def __init__(self) -> None:
        self.header = Header()
        self.image = Image()
        self.f = 0.0
        self.t = 0.0
        self.valid_window = RegionOfInterest()
        self.min_disparity = 0.0
        self.max_disparity = 0.0
        self.delta_d = 0.0


class ExternalShutdownException(Exception):
    """Stand-in rclpy.executors.ExternalShutdownException."""


class RCLError(Exception):
    """Stand-in rclpy._rclpy_pybind11.RCLError."""


class FakeSubscriber:
    """message_filters.Subscriber double that records its arguments."""

    def __init__(self, node: Any, msg_type: type, topic: str, qos_profile: Any = None) -> None:
        self.node = node
        self.msg_type = msg_type
        self.topic = topic
        self.qos_profile = qos_profile


class FakeSynchronizer:
    """message_filters.ApproximateTimeSynchronizer double; the last instance is kept for the tests."""

    last: "FakeSynchronizer | None" = None

    def __init__(self, fs: list[FakeSubscriber], queue_size: int, slop: float) -> None:
        self.fs = fs
        self.queue_size = queue_size
        self.slop = slop
        self.callbacks: list[Any] = []
        FakeSynchronizer.last = self

    def registerCallback(self, callback: Any) -> None:  # noqa: N802 - message_filters API name
        self.callbacks.append(callback)


def install_modules() -> None:
    """Register the doubles in sys.modules (replacing any real package, so tests never touch ROS2)."""
    modules: dict[str, dict[str, Any]] = {
        "rclpy": {
            "init": MagicMock(),
            "ok": MagicMock(return_value=True),
            "spin": MagicMock(),
            "try_shutdown": MagicMock(),
        },
        "rclpy.node": {"Node": MagicMock()},
        "rclpy.qos": {"qos_profile_sensor_data": "sensor-qos"},
        "rclpy.executors": {"ExternalShutdownException": ExternalShutdownException},
        "rclpy._rclpy_pybind11": {"RCLError": RCLError},
        "message_filters": {"Subscriber": FakeSubscriber, "ApproximateTimeSynchronizer": FakeSynchronizer},
        "sensor_msgs": {},
        "sensor_msgs.msg": {"Image": Image, "CameraInfo": CameraInfo, "RegionOfInterest": RegionOfInterest},
        "stereo_msgs": {},
        "stereo_msgs.msg": {"DisparityImage": DisparityImage},
        "std_msgs": {},
        "std_msgs.msg": {"Header": Header},
    }
    for name, attrs in modules.items():
        module = types.ModuleType(name)
        if "." not in name:
            setattr(module, "__path__", [])
        for attr, value in attrs.items():
            setattr(module, attr, value)
        sys.modules[name] = module
        if "." in name:
            parent, child = name.rsplit(".", 1)
            setattr(sys.modules[parent], child, module)


install_modules()


def textured_image(seed: int = 7, height: int = HEIGHT, width: int = WIDTH) -> np.ndarray:
    """Random texture with blobs of a few pixels, which block matching locks onto reliably.

    Args:
        seed (int): RNG seed.
        height (int): Rows.
        width (int): Columns.

    Returns:
        np.ndarray: uint8 grey image (height, width).
    """
    rng = np.random.default_rng(seed)
    coarse = rng.integers(0, 256, size=(height // 2 + 1, width // 2 + 1), dtype=np.uint8)
    return np.kron(coarse, np.ones((2, 2), dtype=np.uint8))[:height, :width]


def shifted_pair(disparity_px: int, seed: int = 7) -> tuple[np.ndarray, np.ndarray]:
    """A rectified pair where every scene point sits disparity_px further left in the right image.

    Args:
        disparity_px (int): Constant disparity in pixels.
        seed (int): Texture seed.

    Returns:
        tuple[np.ndarray, np.ndarray]: (left, right) uint8 grey images (HEIGHT, WIDTH).
    """
    wide = textured_image(seed, HEIGHT, WIDTH + disparity_px)
    left = wide[:, :WIDTH].copy()
    right = wide[:, disparity_px : WIDTH + disparity_px].copy()
    return left, right


def ideal_calibration(width: int = WIDTH, height: int = HEIGHT, rms: float = 0.2) -> StereoCalibration:
    """A distortion-free, already rectified pair: identity R, P = K, right P[3] = -fx * B.

    Args:
        width (int): Image width.
        height (int): Image height.
        rms (float): Reprojection error recorded in stereo.yaml.

    Returns:
        StereoCalibration: The calibration.
    """
    k = [FX, 0.0, CX, 0.0, FX, CY, 0.0, 0.0, 1.0]
    identity = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    left = CameraCalibration(
        camera_name="stereo_left",
        width=width,
        height=height,
        distortion_model="plumb_bob",
        d=[0.0] * 5,
        k=k,
        r=identity,
        p=[FX, 0.0, CX, 0.0, 0.0, FX, CY, 0.0, 0.0, 0.0, 1.0, 0.0],
    )
    right = left.model_copy(
        update={
            "camera_name": "stereo_right",
            "p": [FX, 0.0, CX, -FX * BASELINE_M, 0.0, FX, CY, 0.0, 0.0, 0.0, 1.0, 0.0],
        }
    )
    q = [1.0, 0.0, 0.0, -CX, 0.0, 1.0, 0.0, -CY, 0.0, 0.0, 0.0, FX, 0.0, 0.0, 1.0 / BASELINE_M, 0.0]
    extrinsics = StereoExtrinsics(width=width, height=height, r=identity, t=[-BASELINE_M, 0.0, 0.0], q=q, rms=rms)
    return StereoCalibration(left=left, right=right, extrinsics=extrinsics)


def image_msg(grey: np.ndarray, encoding: str = "bgr8", sec: int = 10, nanosec: int = 0) -> Image:
    """Build a camera Image from a grey array (replicated to three channels for the colour encodings).

    Args:
        grey (np.ndarray): uint8 (H, W).
        encoding (str): "bgr8", "rgb8" or "mono8".
        sec (int): Stamp seconds.
        nanosec (int): Stamp nanoseconds.

    Returns:
        Image: The message.
    """
    msg = Image()
    msg.header.stamp = Stamp(sec, nanosec)
    msg.header.frame_id = "camera"
    pixels = grey if encoding == "mono8" else np.repeat(grey[:, :, None], 3, axis=2)
    msg.height, msg.width = grey.shape
    msg.encoding = encoding
    msg.step = msg.width * (1 if encoding == "mono8" else 3)
    msg.data = np.ascontiguousarray(pixels).tobytes()
    return msg
