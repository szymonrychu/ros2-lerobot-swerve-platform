"""Helpers for applying and verifying startup torque state."""

import time
from collections.abc import Callable


def set_startup_torque_state(
    joint_ids: list[int],
    torque_value: int,
    write_once: Callable[[int, int], bool],
    read_once: Callable[[int], int | None],
    attempts: int = 8,
    verify_sleep_s: float = 0.02,
) -> list[int]:
    """Set startup torque and verify register value for all joints.

    Args:
        joint_ids: Servo IDs to update.
        torque_value: Desired torque register value (0 or 1).
        write_once: Callback (servo_id, torque_value) -> write success.
        read_once: Callback (servo_id) -> current torque register value.
        attempts: Max retries per servo.
        verify_sleep_s: Delay between retries.

    Returns:
        list[int]: Servo IDs that failed to match desired torque value.
    """
    failed_ids: list[int] = []
    for sid in joint_ids:
        ok = False
        for _ in range(max(1, attempts)):
            wrote = write_once(sid, torque_value)
            if wrote:
                readback = read_once(sid)
                if readback == torque_value:
                    ok = True
                    break
            time.sleep(max(0.0, verify_sleep_s))
        if not ok:
            failed_ids.append(sid)
    return failed_ids
