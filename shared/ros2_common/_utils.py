"""Placeholder shared helpers (use type hints)."""


def clamp(value: float, low: float, high: float) -> float:
    """Clamp value to [low, high]."""
    return max(low, min(high, value))
