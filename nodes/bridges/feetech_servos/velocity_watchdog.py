"""Velocity-mode safety: find wheel joints whose last non-zero command has gone stale."""

from collections.abc import Callable


def expired_velocity_joints(
    last_commands: dict[int, tuple[float, float]],
    now: float,
    timeout_s: float,
) -> list[int]:
    """Return servos that are still commanded to move but have not received a command within timeout_s.

    Args:
        last_commands: servo_id -> (monotonic time of last successful command, last commanded velocity rad/s).
        now: Current monotonic time in seconds.
        timeout_s: Maximum command age before the servo must be stopped.

    Returns:
        list[int]: Servo IDs to stop (velocity != 0 and command older than timeout_s).
    """
    return [sid for sid, (stamp, velocity) in last_commands.items() if velocity != 0.0 and now - stamp > timeout_s]


def apply_velocity_command(
    write: Callable[[], bool],
    servo_id: int,
    velocity: float,
    now: float,
    last_commands: dict[int, tuple[float, float]],
) -> bool:
    """Write a velocity command and record it only if the write succeeded.

    A failed write (e.g. a watchdog stop hitting a bus error) leaves the previous record in place, so a
    still-moving servo stays expired and the stop is retried on the next cycle.

    Args:
        write: Performs the register write; returns True on success.
        servo_id: Servo ID being commanded.
        velocity: Commanded velocity, rad/s.
        now: Current monotonic time in seconds.
        last_commands: servo_id -> (time, velocity) of the last successful command; updated in place.

    Returns:
        bool: True if the write succeeded.
    """
    if not write():
        return False
    last_commands[servo_id] = (now, velocity)
    return True
