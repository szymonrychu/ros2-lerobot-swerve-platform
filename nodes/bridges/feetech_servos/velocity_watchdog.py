"""Velocity-mode safety: find wheel joints whose last drive command has gone stale.

Each record is (receive time of the last drive command for the servo, velocity the servo is actually running at
= last successfully written value). Command age is measured from receipt, never from the (later) bus
write. The watchdog stops a wheel only when it is moving and no drive command for it arrived within the timeout.
"""

from collections.abc import Callable


def expired_velocity_joints(
    last_commands: dict[int, tuple[float, float]],
    now: float,
    timeout_s: float,
) -> list[int]:
    """Return servos that are still commanded to move but have not received a command within timeout_s.

    Args:
        last_commands: servo_id -> (receive time of last drive command, applied velocity rad/s).
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
    stamp: float,
    last_commands: dict[int, tuple[float, float]],
    received: bool = False,
) -> bool:
    """Write a velocity command and update the watchdog record.

    For a received drive command (received=True), stamp is the time the command was RECEIVED (not written), so
    the watchdog measures command age from receipt. The record time is refreshed whether or not the write succeeds
    (a command did arrive); the record velocity becomes the commanded value only on success, so after a failed
    write a still-moving servo keeps its applied velocity and is stopped once commands stop.

    For a watchdog stop (received=False) the record is only touched on success (velocity becomes 0, so it never
    expires again); a failed stop leaves the moving record in place, so the stop is retried on the next cycle.
    On success the record becomes (stamp, velocity).

    Args:
        write: Performs the register write; returns True on success.
        servo_id: Servo ID being commanded.
        velocity: Commanded velocity, rad/s.
        stamp: Receive time of the drive command (received=True) or current time (watchdog stop), seconds.
        last_commands: servo_id -> (time of last received drive command, applied velocity rad/s); updated in place.
        received: True when the command came from a joint_commands message (feeds the watchdog).

    Returns:
        bool: True if the write succeeded (or was skipped as unchanged).
    """
    if not write():
        if received:
            _previous_stamp, applied = last_commands.get(servo_id, (stamp, velocity))
            last_commands[servo_id] = (stamp, applied)
        return False
    last_commands[servo_id] = (stamp, velocity)
    return True
