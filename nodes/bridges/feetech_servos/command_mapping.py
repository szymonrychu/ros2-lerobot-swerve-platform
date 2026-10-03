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
# goal_speed / present_speed are sign-magnitude: bit 15 = negative direction, low 15 bits = steps/s.
SPEED_SIGN_BIT = 1 << 15
SPEED_MAGNITUDE_MASK = SPEED_SIGN_BIT - 1


def is_finite_command(value: float) -> bool:
    """Return True if a JointState command entry carries a real value.

    NaN marks "no command for this joint" in the combined command format (position NaN for velocity-mode
    joints, velocity NaN for position-mode joints); inf is never a valid command either.

    Args:
        value: JointState position or velocity entry.

    Returns:
        bool: True if value is finite.
    """
    return math.isfinite(value)


def require_finite(value: float, what: str) -> None:
    """Raise ValueError for a non-finite command value so it can never reach a servo register.

    Args:
        value: Command value to check.
        what: Name of the value for the error message.
    """
    if not math.isfinite(value):
        raise ValueError(f"{what} must be finite, got {value}")


def steps_to_radians(ticks: int, inverted: bool = False) -> float:
    """Convert raw servo ticks to centred radians.

    Args:
        ticks: Raw servo position in steps (0-4095).
        inverted: If True, the joint's positive direction is opposite to the servo's.

    Returns:
        float: Position in radians, 0.0 at servo centre (step 2048).
    """
    radians = (ticks - STEP_CENTER) * RADIANS_PER_STEP
    return -radians if inverted else radians


def position_to_raw_steps(radians: float, inverted: bool = False) -> int:
    """Convert centred radians to raw servo steps, clamped to 0..4095.

    Args:
        radians: Joint position in radians (0.0 = servo centre).
        inverted: If True, the joint's positive direction is opposite to the servo's.

    Returns:
        int: Raw servo step value clamped to [0, 4095].

    Raises:
        ValueError: If radians is NaN or infinite.
    """
    require_finite(radians, "position")
    if inverted:
        radians = -radians
    return max(0, min(4095, int(round(radians * STEPS_PER_RADIAN + STEP_CENTER))))


def velocity_to_speed_register(radians_per_s: float, max_velocity_rad_s: float, inverted: bool = False) -> int:
    """Convert a joint velocity to a sign-magnitude goal_speed register value (wheel mode).

    Args:
        radians_per_s: Desired joint velocity in rad/s.
        max_velocity_rad_s: Magnitude limit in rad/s; larger requests are clamped.
        inverted: If True, the joint's positive direction is opposite to the servo's.

    Returns:
        int: Raw goal_speed value (bit 15 set for negative direction, low bits in steps/s).

    Raises:
        ValueError: If radians_per_s is NaN or infinite (NaN would otherwise clamp to full speed).
    """
    require_finite(radians_per_s, "velocity")
    velocity = -radians_per_s if inverted else radians_per_s
    limit = max(0.0, max_velocity_rad_s)
    velocity = max(-limit, min(limit, velocity))
    magnitude = min(SPEED_MAGNITUDE_MASK, int(round(abs(velocity) * STEPS_PER_RADIAN)))
    if magnitude == 0:
        return 0
    return magnitude | SPEED_SIGN_BIT if velocity < 0 else magnitude


def speed_register_to_velocity(raw: int, inverted: bool = False) -> float:
    """Decode a sign-magnitude present_speed register value to rad/s.

    Args:
        raw: Raw present_speed value (bit 15 = negative direction, low bits in steps/s).
        inverted: If True, the joint's positive direction is opposite to the servo's.

    Returns:
        float: Joint velocity in rad/s.
    """
    velocity = (raw & SPEED_MAGNITUDE_MASK) * RADIANS_PER_STEP
    if raw & SPEED_SIGN_BIT:
        velocity = -velocity
    return -velocity if inverted else velocity


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

    Raises:
        ValueError: If radians is NaN or infinite.
    """
    require_finite(radians, "position")
    raw_steps = radians * STEPS_PER_RADIAN + STEP_CENTER
    if source_max == source_min:
        return cmd_min
    normalized = max(0.0, min(1.0, (raw_steps - source_min) / (source_max - source_min)))
    if source_inverted:
        normalized = 1.0 - normalized
    target = round(cmd_min + normalized * (cmd_max - cmd_min))
    return max(cmd_min, min(cmd_max, target))
