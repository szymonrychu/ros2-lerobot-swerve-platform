"""ROS message serializer helpers."""

import json
import math
from array import array
from typing import Any

BINARY_DATA_TYPES = (
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/CompressedImage",
    "sensor_msgs/msg/PointCloud2",
)
SENSOR_QOS_TYPES = BINARY_DATA_TYPES
BINARY_FIELD = "data"


def ros_time_to_ns(stamp: Any) -> int | None:
    """Convert ROS time-like object to nanoseconds.

    Args:
        stamp: Object with `sec` and `nanosec` attributes.

    Returns:
        int | None: Nanoseconds since epoch-like origin, or None when unavailable.
    """

    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if sec is None or nanosec is None:
        return None
    try:
        return int(sec) * 1_000_000_000 + int(nanosec)
    except (TypeError, ValueError):
        return None


def ros_message_to_builtin(value: Any) -> Any:
    """Recursively convert ROS message object to JSON-safe builtin values.

    Args:
        value: Primitive, list, dict, or ROS message-like object.

    Returns:
        Any: JSON-safe object; non-finite floats (NaN, +/-inf) become None (JSON null).
    """

    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if value is None or isinstance(value, (str, int, bool)):
        return value
    if isinstance(value, list):
        return [ros_message_to_builtin(item) for item in value]
    if isinstance(value, tuple):
        return [ros_message_to_builtin(item) for item in value]
    if isinstance(value, array):
        return [ros_message_to_builtin(item) for item in value]
    if isinstance(value, dict):
        return {str(k): ros_message_to_builtin(v) for k, v in value.items()}
    if hasattr(value, "get_fields_and_field_types"):
        output: dict[str, Any] = {}
        fields = value.get_fields_and_field_types()
        for field_name in fields.keys():
            output[field_name] = ros_message_to_builtin(getattr(value, field_name))
        return output
    return str(value)


def message_to_payload(msg: Any, type_name: str) -> Any:
    """Convert a ROS message to JSON-safe builtins without expanding binary sensor data.

    For Image, CompressedImage and PointCloud2 the `data` field is never converted (a list of
    ints costs ~8 bytes per byte). It is replaced by `{"omitted": true, "byte_length": N}`;
    every other field is kept.

    Args:
        msg: ROS message-like object.
        type_name: ROS type name string (e.g. sensor_msgs/msg/Image).

    Returns:
        Any: JSON-safe object.
    """

    if type_name.strip() not in BINARY_DATA_TYPES or not hasattr(msg, "get_fields_and_field_types"):
        return ros_message_to_builtin(msg)
    output: dict[str, Any] = {}
    for field_name in msg.get_fields_and_field_types():
        value = getattr(msg, field_name)
        if field_name == BINARY_FIELD:
            output[field_name] = {"omitted": True, "byte_length": len(value)}
        else:
            output[field_name] = ros_message_to_builtin(value)
    return output


def dumps_strict(data: Any) -> str:
    """Serialize to strict JSON: non-finite floats anywhere in data become null.

    Args:
        data: Builtin JSON-like object (dict, list, str, int, float, bool, None).

    Returns:
        str: JSON text that never contains NaN or Infinity.
    """
    return json.dumps(ros_message_to_builtin(data), allow_nan=False)
