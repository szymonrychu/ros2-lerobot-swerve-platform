"""Serialize ROS2 messages to JSON-serializable dicts for the WebSocket bridge."""

from __future__ import annotations

import array
import base64
from typing import Any

import cv2
import numpy as np


def msg_to_dict(msg: Any) -> dict[str, Any]:
    """Convert a ROS2 message object to a JSON-serializable dict.

    Handles nested messages, arrays, and Image types with JPEG compression.
    Uses __slots__ introspection to walk the message tree without depending on
    rosidl_runtime_py (which requires PYTHONPATH from sourced ROS2 setup.bash).

    Args:
        msg: ROS2 message object (rclpy message).

    Returns:
        dict[str, Any]: JSON-serializable representation.
    """
    msg_type = type(msg).__name__
    if msg_type in ("Image", "CompressedImage"):
        return _serialize_image(msg)
    return _serialize_msg(msg)


def _serialize_msg(msg: Any) -> dict[str, Any]:
    """Recursively serialize a ROS2 message to a plain dict.

    Args:
        msg: ROS2 message with __slots__.

    Returns:
        dict[str, Any]: JSON-serializable dict.
    """
    result: dict[str, Any] = {}
    slots = getattr(msg, "__slots__", None)
    if slots is None:
        return result
    for slot in slots:
        # rclpy slot names start with underscore; strip it to get field name
        field = slot.lstrip("_")
        value = getattr(msg, field, None)
        result[field] = _serialize_value(value)
    return result


def _serialize_value(value: Any) -> Any:
    """Convert a single ROS2 field value to a JSON-serializable type.

    Args:
        value: Field value from a ROS2 message.

    Returns:
        Any: JSON-serializable representation.
    """
    if isinstance(value, array.array):
        return list(value)
    if isinstance(value, (bytes, bytearray)):
        return list(value)
    if isinstance(value, (list, tuple)):
        return [_serialize_value(v) for v in value]
    if isinstance(value, (int, float, bool, str)) or value is None:
        return value
    if hasattr(value, "__slots__"):
        return _serialize_msg(value)
    # numpy scalar / array
    try:
        return value.tolist()
    except AttributeError:
        pass
    return str(value)


def _serialize_image(msg: Any) -> dict[str, Any]:
    """Serialize Image or CompressedImage message to a dict with JPEG base64.

    For sensor_msgs/Image: decode raw bytes, compress to JPEG at 80% quality.
    For sensor_msgs/CompressedImage: pass through if already JPEG, else re-compress.

    Args:
        msg: Image or CompressedImage ROS2 message.

    Returns:
        dict[str, Any]: Dict with 'jpeg_b64' key containing base64-encoded JPEG.
    """
    try:
        msg_type = type(msg).__name__
        if msg_type == "CompressedImage":
            raw = bytes(msg.data)
            arr = np.frombuffer(raw, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        else:
            encoding = msg.encoding.lower()
            raw = bytes(msg.data)
            arr = np.frombuffer(raw, dtype=np.uint8)
            arr = arr.reshape((msg.height, msg.step // _bytes_per_pixel(encoding)))
            if encoding in ("rgb8", "rgb16"):
                img = cv2.cvtColor(arr[:, : msg.width], cv2.COLOR_RGB2BGR)
            elif encoding in ("bgr8", "bgr16"):
                img = arr[:, : msg.width]
            elif encoding in ("mono8", "mono16"):
                img = arr[:, : msg.width]
            else:
                img = arr[:, : msg.width]

        if img is None:
            return {"jpeg_b64": None, "error": "decode_failed"}

        success, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not success:
            return {"jpeg_b64": None, "error": "encode_failed"}

        return {"jpeg_b64": base64.b64encode(buf.tobytes()).decode("ascii")}
    except Exception as exc:  # pylint: disable=broad-except
        return {"jpeg_b64": None, "error": str(exc)}


def extractField_from_dict(data: dict[str, Any], path: str) -> float | None:  # noqa: N802
    """Extract a nested numeric field from a dict using dot-notation + array indexing.

    This is the Python equivalent of the TypeScript extractField utility, used for
    testing message serialization independently of the renderer.

    Args:
        data: Dict (e.g. serialized ROS2 message).
        path: Dot-notation path, e.g. "linear_acceleration.x" or "position[0]".

    Returns:
        float | None: Extracted numeric value, or None if not found / not numeric.
    """
    current: Any = data
    parts = path.replace("[", ".[").split(".")
    for part in parts:
        if current is None:
            return None
        if part.startswith("[") and part.endswith("]"):
            try:
                idx = int(part[1:-1])
                current = current[idx]
            except (IndexError, TypeError, ValueError):
                return None
        else:
            if not isinstance(current, dict):
                return None
            current = current.get(part)
    if isinstance(current, (int, float)) and not isinstance(current, bool):
        return float(current)
    return None


def _bytes_per_pixel(encoding: str) -> int:
    """Return bytes per pixel for a ROS2 image encoding.

    Args:
        encoding: ROS2 image encoding string (e.g. 'rgb8', 'mono16').

    Returns:
        int: Bytes per pixel.
    """
    encoding = encoding.lower()
    if "16" in encoding:
        return 2
    if encoding in ("rgba8", "bgra8"):
        return 4
    if encoding in ("rgb8", "bgr8"):
        return 3
    return 1
