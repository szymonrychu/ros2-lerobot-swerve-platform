"""Shared helpers for ROS2 nodes (use type hints)."""


def clamp(value: float, low: float, high: float) -> float:
    """Clamp value to the closed interval [low, high].

    Args:
        value: Input value (float).
        low: Lower bound (float).
        high: Upper bound (float).

    Returns:
        float: value if low <= value <= high; otherwise low or high.
    """
    return max(low, min(high, value))
