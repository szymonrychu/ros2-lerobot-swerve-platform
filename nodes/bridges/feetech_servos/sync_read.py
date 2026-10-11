"""Read present position, speed and (optionally) load for many servos in one bus transaction (STS sync read)."""

from collections.abc import Callable, Collection, Mapping, Sequence
from typing import Any, NamedTuple

from .config import JointEntry
from .registers import decode_present_load

# present_position (2 bytes) is immediately followed by present_speed (2 bytes) and present_load (2 bytes).
PRESENT_POSITION_ADDRESS = 56
SYNC_READ_LENGTH = 4
SYNC_READ_LENGTH_WITH_LOAD = 6
COMM_SUCCESS = 0


class ServoState(NamedTuple):
    """Raw state registers of one servo from one read.

    Attributes:
        position: present_position steps.
        speed: present_speed, raw sign-magnitude.
        load: present_load, raw sign-magnitude; None when the read did not cover it.
    """

    position: int
    speed: int
    load: int | None = None


def sync_read_length(with_load: bool) -> int:
    """Return the state read length: position + speed, plus present_load when efforts are published.

    Args:
        with_load (bool): True when any joint publishes effort.

    Returns:
        int: Bytes read from PRESENT_POSITION_ADDRESS per servo.
    """
    return SYNC_READ_LENGTH_WITH_LOAD if with_load else SYNC_READ_LENGTH


def decode_state(data: Sequence[int]) -> ServoState:
    """Decode the bytes read from PRESENT_POSITION_ADDRESS (little-endian 2-byte registers).

    Args:
        data (Sequence[int]): SYNC_READ_LENGTH or SYNC_READ_LENGTH_WITH_LOAD bytes.

    Returns:
        ServoState: Raw position, speed and load (None for a 4-byte read).
    """
    load = data[4] | (data[5] << 8) if len(data) >= SYNC_READ_LENGTH_WITH_LOAD else None
    return ServoState(data[0] | (data[1] << 8), data[2] | (data[3] << 8), load)


def read_servo_state(servo: Any, sts_id: int, length: int = SYNC_READ_LENGTH) -> ServoState | None:
    """Read one servo's state with one INST_READ (the per-servo fallback of the sync read).

    Args:
        servo (Any): ST3215-compatible bus (readTxRx).
        sts_id (int): Servo ID.
        length (int): SYNC_READ_LENGTH or SYNC_READ_LENGTH_WITH_LOAD.

    Returns:
        ServoState | None: The state, or None on no reply, short reply or a servo error byte.
    """
    data, comm, err = servo.readTxRx(sts_id, PRESENT_POSITION_ADDRESS, length)
    if comm != COMM_SUCCESS or err != 0 or len(data) < length:
        return None
    return decode_state(data)


def read_servo_states(
    servo_ids: list[int],
    group_factory: Callable[[], Any],
    fallback_read: Callable[[int], ServoState | None],
    length: int = SYNC_READ_LENGTH,
) -> dict[int, ServoState]:
    """Read the raw state of each servo, using one sync read when possible.

    Servos missing from the sync reply (or all of them when the transaction fails) are read
    individually via fallback_read. Servos that cannot be read at all are omitted from the result,
    so callers never see placeholder values.

    Args:
        servo_ids: Servo IDs to read.
        group_factory: Returns a fresh st3215.GroupSyncRead-compatible object for address 56 and length.
        fallback_read: Per-servo read returning the state, or None on failure.
        length: Bytes per servo, SYNC_READ_LENGTH or SYNC_READ_LENGTH_WITH_LOAD.

    Returns:
        dict[int, ServoState]: servo_id -> raw state.
    """
    result: dict[int, ServoState] = {}
    group = group_factory()
    for sid in servo_ids:
        group.addParam(sid)
    if group.txRxPacket() == COMM_SUCCESS:
        for sid in servo_ids:
            available, _error = group.isAvailable(sid, PRESENT_POSITION_ADDRESS, length)
            if available:
                # Decode bytes directly: st3215 GroupSyncRead.getData calls a non-existent scs_makeword.
                result[sid] = decode_state(group.data_dict[sid][1 : length + 1])
    for sid in servo_ids:
        if sid not in result:
            single = fallback_read(sid)
            if single is not None:
                result[sid] = single
    return result


def joint_efforts(
    joints: Sequence[JointEntry], readings: Mapping[int, ServoState], effort_joints: Collection[str]
) -> list[float]:
    """Decode JointState.effort for one group from the state read.

    Args:
        joints (Sequence[JointEntry]): Group joints, in message order (all present in readings).
        readings (Mapping[int, ServoState]): State read result.
        effort_joints (Collection[str]): publish_effort_joints names.

    Returns:
        list[float]: Signed load per joint (0.0 for joints not in effort_joints), or [] when no joint
            publishes effort.
    """
    if not effort_joints:
        return []
    efforts: list[float] = []
    for joint in joints:
        load = readings[joint.id].load if joint.name in effort_joints else None
        efforts.append(float(decode_present_load(load, joint.inverted)) if load is not None else 0.0)
    return efforts
