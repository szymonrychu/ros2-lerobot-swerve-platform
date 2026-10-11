"""Tilt guard consumer: parse /tilt_guard/state and build the message that refuses navigation goals."""

from __future__ import annotations

import json
from typing import Any

# std_msgs/String JSON {"level": "ok" | "warning" | "error", "roll_deg": float, "pitch_deg": float},
# published reliable + transient_local at 1 Hz by the tilt guard.
TILT_GUARD_STATE_TOPIC = "/tilt_guard/state"
LEVEL_OK = "ok"
LEVEL_WARNING = "warning"
LEVEL_ERROR = "error"
LEVELS = frozenset({LEVEL_OK, LEVEL_WARNING, LEVEL_ERROR})
ANGLE_KEYS = ("roll_deg", "pitch_deg")
# Source tag of the WebSocket error frame sent when a goal is refused.
TILT_GUARD_ERROR_SOURCE = "tilt_guard"


def parse_tilt_state(raw: str) -> dict[str, Any] | None:
    """Parse a tilt guard state JSON defensively.

    Args:
        raw (str): JSON text from a std_msgs/String.

    Returns:
        dict[str, Any] | None: The decoded object, or None when it is not valid JSON, not an object, has an
            unknown "level" or a present roll_deg/pitch_deg that is not a number.
    """
    try:
        data = json.loads(raw)
    except (ValueError, TypeError):
        return None
    if not isinstance(data, dict) or data.get("level") not in LEVELS:
        return None
    for key in ANGLE_KEYS:
        value = data.get(key)
        if value is not None and (isinstance(value, bool) or not isinstance(value, int | float)):
            return None
    return data


def nav_rejection_message(state: dict[str, Any] | None) -> str | None:
    """Return why navigation goals are refused for a tilt state.

    Args:
        state (dict[str, Any] | None): Parsed tilt state, or None when none was received.

    Returns:
        str | None: Refusal text at warning/error level, None when goals are allowed.
    """
    if state is None:
        return None
    level = state.get("level")
    if level == LEVEL_WARNING:
        roll, pitch = (state.get(key) for key in ANGLE_KEYS)
        return (
            f"Tilt warning: roll {_fmt(roll)} / pitch {_fmt(pitch)} deg - "
            "automatic navigation disabled, drive back manually"
        )
    if level == LEVEL_ERROR:
        return "Tilt error: base drive disabled until the robot is level"
    return None


def _fmt(angle: float | None) -> str:
    """Format an angle with one decimal, or "?" when missing."""
    return "?" if angle is None else f"{angle:.1f}"
