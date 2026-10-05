"""Stub the ROS2 message packages so the conversion code is tested without a ROS2 install."""

import sys
import types
from typing import Any


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


class CameraInfo:
    """sensor_msgs/CameraInfo stand-in."""

    def __init__(self) -> None:
        self.header = Header()
        self.height = 0
        self.width = 0
        self.p = [0.0] * 12


class DisparityImage:
    """stereo_msgs/DisparityImage stand-in."""

    def __init__(self) -> None:
        self.header = Header()
        self.image = Image()
        self.f = 0.0
        self.t = 0.0
        self.min_disparity = 0.0
        self.max_disparity = 0.0


def install_stub(name: str, **attrs: Any) -> None:
    """Register a stub module (and its parents) in sys.modules unless a real one is importable.

    Args:
        name: Dotted module name.
        **attrs: Attributes set on the stub.
    """
    parent = types.ModuleType(name.split(".")[0])
    parent.__path__ = []  # type: ignore[attr-defined]
    sys.modules.setdefault(parent.__name__, parent)
    module = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(module, key, value)
    sys.modules[name] = module
    setattr(sys.modules[parent.__name__], name.split(".")[1], module)


install_stub("sensor_msgs.msg", Image=Image, CameraInfo=CameraInfo)
install_stub("stereo_msgs.msg", DisparityImage=DisparityImage)
