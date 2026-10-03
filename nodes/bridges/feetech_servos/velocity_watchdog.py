"""Velocity-mode safety: find wheel joints whose last drive command has gone stale.

Each record is (time of the last drive command received for the servo, velocity the servo is actually running at
= last successfully written value). The watchdog stops a wheel only when it is moving and no drive command for it
arrived within the timeout.
"""

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
    received: bool = False,
) -> bool:
    """Write a velocity command and update the watchdog record.

    On success the record becomes (now, velocity). On a failed write:
    - a watchdog stop (received=False) leaves the previous record in place, so a still-moving servo stays
      expired and the stop is retried on the next cycle;
    - a received drive command (received=True) refreshes the time (a command did arrive) but keeps the
      previously applied velocity, so if commands then stop, a still-moving servo is stopped after the timeout.

    Args:
        write: Performs the register write; returns True on success.
        servo_id: Servo ID being commanded.
        velocity: Commanded velocity, rad/s.
        now: Current monotonic time in seconds.
        last_commands: servo_id -> (time, velocity); updated in place.
        received: True when the command came from a joint_commands message (feeds the watchdog).

    Returns:
        bool: True if the write succeeded.
    """
    if not write():
        if received:
            _stamp, applied = last_commands.get(servo_id, (now, velocity))
            last_commands[servo_id] = (now, applied)
        return False
    last_commands[servo_id] = (now, velocity)
    return True
