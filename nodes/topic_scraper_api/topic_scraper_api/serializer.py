"""ROS message serializer helpers."""

from typing import Any


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
        Any: JSON-safe object.
    """

    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, list):
        return [ros_message_to_builtin(item) for item in value]
    if isinstance(value, tuple):
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
