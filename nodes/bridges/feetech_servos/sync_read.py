"""Read present position and speed for many servos in one bus transaction (STS sync read)."""

from collections.abc import Callable
from typing import Any

# present_position (2 bytes) is immediately followed by present_speed (2 bytes).
PRESENT_POSITION_ADDRESS = 56
PRESENT_SPEED_ADDRESS = 58
SYNC_READ_LENGTH = 4
COMM_SUCCESS = 0


def read_positions_and_speeds(
    servo_ids: list[int],
    group_factory: Callable[[], Any],
    fallback_read: Callable[[int], tuple[int, int] | None],
) -> dict[int, tuple[int, int]]:
    """Read raw (present_position, present_speed) for each servo, using one sync read when possible.

    Servos missing from the sync reply (or all of them when the transaction fails) are read
    individually via fallback_read. Servos that cannot be read at all are omitted from the result,
    so callers never see placeholder values.

    Args:
        servo_ids: Servo IDs to read.
        group_factory: Returns a fresh st3215.GroupSyncRead-compatible object for address 56, length 4.
        fallback_read: Per-servo read returning (position, speed) raw values, or None on failure.

    Returns:
        dict[int, tuple[int, int]]: servo_id -> (raw position steps, raw sign-magnitude speed).
    """
    result: dict[int, tuple[int, int]] = {}
    group = group_factory()
    for sid in servo_ids:
        group.addParam(sid)
    if group.txRxPacket() == COMM_SUCCESS:
        for sid in servo_ids:
            available, _error = group.isAvailable(sid, PRESENT_POSITION_ADDRESS, SYNC_READ_LENGTH)
            if available:
                result[sid] = (
                    group.getData(sid, PRESENT_POSITION_ADDRESS, 2),
                    group.getData(sid, PRESENT_SPEED_ADDRESS, 2),
                )
    for sid in servo_ids:
        if sid not in result:
            single = fallback_read(sid)
            if single is not None:
                result[sid] = single
    return result
