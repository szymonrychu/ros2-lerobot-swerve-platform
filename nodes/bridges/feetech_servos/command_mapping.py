"""Position-to-steps range mapping for joint commands (no ROS dependency)."""

# Scale: joint_commands position (pseudo-radians) = steps / this value.
POSITION_RADIAN_TO_STEPS = 1000.0


def map_position_to_steps(
    position: float,
    source_min: int,
    source_max: int,
    cmd_min: int,
    cmd_max: int,
) -> int:
    """Map incoming position (pseudo-radians, steps/1000) to target steps in [cmd_min, cmd_max].

    Incoming value is interpreted as lying in [source_min, source_max] (in steps).
    Returns steps clamped to [cmd_min, cmd_max]. Avoids division by zero when source range is degenerate.
    """
    raw_steps = position * POSITION_RADIAN_TO_STEPS
    if source_max == source_min:
        return cmd_min
    normalized = max(0.0, min(1.0, (raw_steps - source_min) / (source_max - source_min)))
    target = round(cmd_min + normalized * (cmd_max - cmd_min))
    return max(cmd_min, min(cmd_max, target))
