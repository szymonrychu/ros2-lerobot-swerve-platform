"""Position-to-steps conversion for joint commands (no ROS dependency).

STS3215 servo: 4096 steps across 360° (2π rad). Centre (midpoint) = step 2048.
Published positions and incoming commands use true radians centred at 0:
    radians = (ticks - STEP_CENTER) * RADIANS_PER_STEP
    ticks   = round(radians * STEPS_PER_RADIAN) + STEP_CENTER
"""

import math

STEPS_PER_REVOLUTION = 4096
STEP_CENTER = 2048
RADIANS_PER_STEP = (2 * math.pi) / STEPS_PER_REVOLUTION  # ~0.001534 rad/step
STEPS_PER_RADIAN = STEPS_PER_REVOLUTION / (2 * math.pi)  # ~651.9 steps/rad


def steps_to_radians(ticks: int) -> float:
    """Convert raw servo ticks to centred radians.

    Args:
        ticks: Raw servo position in steps (0-4095).

    Returns:
        float: Position in radians, 0.0 at servo centre (step 2048).
    """
    return (ticks - STEP_CENTER) * RADIANS_PER_STEP


def position_to_raw_steps(radians: float) -> int:
    """Convert centred radians to raw servo steps, clamped to 0..4095.

    Args:
        radians: Joint position in radians (0.0 = servo centre).

    Returns:
        int: Raw servo step value clamped to [0, 4095].
    """
    return max(0, min(4095, int(round(radians * STEPS_PER_RADIAN + STEP_CENTER))))


def map_position_to_steps(
    radians: float,
    source_min: int,
    source_max: int,
    cmd_min: int,
    cmd_max: int,
    source_inverted: bool = False,
) -> int:
    """Map incoming position (radians) to target steps in [cmd_min, cmd_max].

    Incoming radian value is converted to raw steps, then normalised within
    [source_min, source_max] and mapped linearly to [cmd_min, cmd_max].
    Returns steps clamped to [cmd_min, cmd_max]. Avoids division by zero when
    source range is degenerate.

    Args:
        radians: Incoming joint position in centred radians.
        source_min: Source servo range minimum in raw steps.
        source_max: Source servo range maximum in raw steps.
        cmd_min: Command servo range minimum in raw steps.
        cmd_max: Command servo range maximum in raw steps.
        source_inverted: If True, invert normalised progress before mapping.

    Returns:
        int: Target servo step clamped to [cmd_min, cmd_max].
    """
    raw_steps = radians * STEPS_PER_RADIAN + STEP_CENTER
    if source_max == source_min:
        return cmd_min
    normalized = max(0.0, min(1.0, (raw_steps - source_min) / (source_max - source_min)))
    if source_inverted:
        normalized = 1.0 - normalized
    target = round(cmd_min + normalized * (cmd_max - cmd_min))
    return max(cmd_min, min(cmd_max, target))
