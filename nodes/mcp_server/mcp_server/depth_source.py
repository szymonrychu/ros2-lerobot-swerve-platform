"""Aligned depth frames of the head D435: Image decoding and pairing with the colour frame of the same frameset.

The depth image is registered to the colour image (``aligned_depth_to_color``), so a colour pixel (u, v) reads the depth
at the same (u, v). The pixel was chosen on a colour picture; the depth frame is only valid for it when it comes from
the same frameset, i.e. its header stamp agrees with the colour frame's within half a frame period. ROS types are not
imported: messages are read by attribute, so the logic runs in tests with plain objects.
"""

import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import numpy as np

from .camera_scene import CameraInfoData
from .config import CameraArmSettings
from .models import RobotError
from .spin import FrameCache

ENCODINGS = {"16UC1": 2, "32FC1": 4}  # encoding -> bytes per pixel
POLL_S = 0.01


@dataclass(frozen=True)
class DepthFrame:
    """One aligned depth frame paired with its colour frame.

    Attributes:
        depth (np.ndarray): (h, w) depth: uint16 millimetres (16UC1) or float32 metres (32FC1), 0 = no depth.
        stamp_s (float): Header stamp of the depth frame (ROS time, s).
        color_stamp_s (float): Header stamp of the colour frame it was paired with (ROS time, s).
        age_s (float): Age of the depth frame when it was read (s).
        info (CameraInfoData): CameraInfo of the aligned depth stream.
    """

    depth: np.ndarray
    stamp_s: float
    color_stamp_s: float
    age_s: float
    info: CameraInfoData


def header_stamp_s(msg: Any) -> float:
    """Header stamp of a message in seconds.

    Args:
        msg (Any): Message with header.stamp.sec / nanosec.

    Returns:
        float: Seconds (ROS time).
    """
    return msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9


def decode_depth_image(msg: Any) -> np.ndarray:
    """Decode a sensor_msgs/Image of 16UC1 (mm) or 32FC1 (m) into a native-endian array.

    Args:
        msg (Any): Image with width, height, encoding, step, is_bigendian and data.

    Returns:
        np.ndarray: (height, width) uint16 or float32.

    Raises:
        RobotError: For another encoding, or when the data is shorter than height x step.
    """
    encoding = str(msg.encoding)
    if encoding not in ENCODINGS:
        raise RobotError(f"aligned depth image has encoding {encoding!r}; only {', '.join(ENCODINGS)} are supported")
    item = ENCODINGS[encoding]
    width, height, step = int(msg.width), int(msg.height), int(msg.step)
    data = bytes(msg.data)
    if width <= 0 or height <= 0 or step < width * item or len(data) < step * height:
        raise RobotError(
            f"aligned depth image size is inconsistent: {width}x{height}, step {step}, {len(data)} data bytes"
        )
    rows = np.frombuffer(data, dtype=np.uint8, count=step * height).reshape(height, step)[:, : width * item]
    kind = "u2" if item == 2 else "f4"
    dtype = np.dtype((">" if msg.is_bigendian else "<") + kind)
    return np.ascontiguousarray(rows).view(dtype).reshape(height, width).astype(np.uint16 if item == 2 else np.float32)


class DepthSource:
    """Newest aligned depth frame whose stamp matches the newest colour frame (the same frameset)."""

    def __init__(
        self,
        depth_cache: FrameCache,
        colour_cache: FrameCache,
        read_info: Callable[[], CameraInfoData | None],
        touch: Callable[[], None],
        ros_age: Callable[[Any], float],
        cfg: CameraArmSettings,
        first_message_s: float,
        max_age_s: float,
        depth_topic: str,
        colour_topic: str,
    ) -> None:
        """Bind the frame caches and the checks.

        Args:
            depth_cache (FrameCache): Latest Image of the aligned depth topic.
            colour_cache (FrameCache): Latest message (header only is read) of the colour topic.
            read_info (Callable[[], CameraInfoData | None]): Latest CameraInfo of the aligned depth stream.
            touch (Callable[[], None]): Subscribes the on-demand depth, colour and CameraInfo topics.
            ros_age (Callable[[Any], float]): Age of a header stamp against the node clock (s).
            cfg (CameraArmSettings): max_pair_dt_s.
            first_message_s (float): How long to wait for a fresh depth frame and for the colour frame to pair (s).
            max_age_s (float): Oldest accepted depth frame (s).
            depth_topic (str): Depth topic name, for messages.
            colour_topic (str): Colour topic name, for messages.
        """
        self.depth_cache = depth_cache
        self.colour_cache = colour_cache
        self.read_info = read_info
        self.touch = touch
        self.ros_age = ros_age
        self.cfg = cfg
        self.first_message_s = first_message_s
        self.max_age_s = max_age_s
        self.depth_topic = depth_topic
        self.colour_topic = colour_topic

    def read(self) -> DepthFrame:
        """Wait for a fresh aligned depth frame and the colour frame of the same frameset.

        Returns:
            DepthFrame: The decoded depth with both stamps and the depth CameraInfo.

        Raises:
            RobotError: No depth frame within the window, no colour frame, a frame without a capture stamp or older
                than the limit, stamps further apart than max_pair_dt_s (not the same frameset), or no CameraInfo.
        """
        self.touch()
        started = self.depth_cache.clock()
        msg = self.depth_cache.wait_newer(started, self.first_message_s)
        if msg is None:
            raise RobotError(f"no aligned depth frame on {self.depth_topic} within {self.first_message_s:g} s")
        deadline = time.monotonic() + self.first_message_s
        limit = self.cfg.max_pair_dt_s
        while True:
            msg = self.depth_cache.msg
            colour = self.colour_cache.msg
            if not header_stamp_s(msg):
                raise RobotError(f"aligned depth frame on {self.depth_topic} has no capture timestamp")
            if colour is None and time.monotonic() >= deadline:
                raise RobotError(f"no colour frame on {self.colour_topic} to pair the depth frame with")
            if colour is not None:
                delta = header_stamp_s(colour) - header_stamp_s(msg)
                if abs(delta) <= limit:
                    break
                if time.monotonic() >= deadline:
                    raise RobotError(
                        f"colour and aligned depth stamps differ by {abs(delta):.3f} s (limit {limit:.3f} s): "
                        "not the same frameset, so the depth does not belong to the colour picture"
                    )
            time.sleep(POLL_S)
        stamp = header_stamp_s(msg)
        age = self.ros_age(msg.header.stamp)
        if age > self.max_age_s:
            raise RobotError(
                f"newest aligned depth frame on {self.depth_topic} is {age:.1f} s old (limit {self.max_age_s:g} s)"
            )
        info = self.read_info()
        if info is None:
            raise RobotError(f"no CameraInfo for the aligned depth stream yet ({self.depth_topic})")
        return DepthFrame(decode_depth_image(msg), stamp, header_stamp_s(colour), round(age, 3), info)
