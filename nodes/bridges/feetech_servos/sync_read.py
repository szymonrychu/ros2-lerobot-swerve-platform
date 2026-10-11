"""Read present position, speed and (optionally) load for many servos in one bus transaction (STS sync read).

A sync read that times out (one servo did not answer) still keeps every reply that arrived: only the servos missing
from it are read again one by one. A servo that keeps failing is backed off (ReadBackoff): it is left out of the sync
read, so the healthy servos are not delayed by its timeout every cycle, and probed with one single read per retry
interval until it answers again.
"""

import math
import time
from collections.abc import Callable, Collection, Iterable, Mapping, Sequence
from typing import Any, NamedTuple

from .config import DEFAULT_READ_FAILURE_LIMIT, DEFAULT_READ_RETRY_INTERVAL_S, JointEntry
from .registers import decode_present_load

# present_position (2 bytes) is immediately followed by present_speed (2 bytes) and present_load (2 bytes).
PRESENT_POSITION_ADDRESS = 56
SYNC_READ_LENGTH = 4
SYNC_READ_LENGTH_WITH_LOAD = 6
COMM_SUCCESS = 0
UNMEASURED_EFFORT = math.nan  # JointState.effort for a joint without a measured load (never a placeholder 0.0)


class ServoState(NamedTuple):
    """Raw state registers of one servo from one read.

    Attributes:
        position: present_position steps.
        speed: present_speed, raw sign-magnitude.
        load: present_load, raw sign-magnitude; None when the read did not cover it.
        read_at_ns: Clock time the read was requested (when the servo sampled it), ns; None when not stamped.
    """

    position: int
    speed: int
    load: int | None = None
    read_at_ns: int | None = None


class ReadBackoff:
    """Per-servo read failure tracking: a servo failing failure_limit cycles in a row is backed off.

    A backed-off servo is left out of the sync read and probed with one single read every retry_interval_s, so a
    dead servo costs one packet timeout per retry interval instead of one (or more) every cycle.

    Attributes:
        failure_limit: Consecutive failed cycles before a servo is backed off.
        retry_interval_s: Seconds between probes of a backed-off servo.
        clock: Monotonic time source, seconds.
        failures: servo_id -> consecutive failed cycles.
        next_probe_at: servo_id -> clock time of the next probe of a backed-off servo.
    """

    def __init__(
        self,
        failure_limit: int = DEFAULT_READ_FAILURE_LIMIT,
        retry_interval_s: float = DEFAULT_READ_RETRY_INTERVAL_S,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        """Create the tracker.

        Args:
            failure_limit (int): Consecutive failed cycles before backing off (>= 1).
            retry_interval_s (float): Seconds between probes of a backed-off servo.
            clock (Callable[[], float]): Monotonic time source, seconds.
        """
        self.failure_limit = max(1, failure_limit)
        self.retry_interval_s = retry_interval_s
        self.clock = clock
        self.failures: dict[int, int] = {}
        self.next_probe_at: dict[int, float] = {}

    def is_backed_off(self, sid: int) -> bool:
        """Return True when the servo failed failure_limit cycles in a row.

        Args:
            sid (int): Servo ID.

        Returns:
            bool: True when backed off.
        """
        return self.failures.get(sid, 0) >= self.failure_limit

    def split(self, servo_ids: Iterable[int]) -> tuple[list[int], list[int]]:
        """Split the servos into the sync read set and the backed-off servos due for a probe.

        Args:
            servo_ids (Iterable[int]): Servo IDs to read this cycle.

        Returns:
            tuple[list[int], list[int]]: (servos for the sync read, backed-off servos to probe with a single read).
        """
        now = self.clock()
        sync_ids: list[int] = []
        probe_ids: list[int] = []
        for sid in servo_ids:
            if not self.is_backed_off(sid):
                sync_ids.append(sid)
            elif now >= self.next_probe_at.get(sid, now):
                probe_ids.append(sid)
        return sync_ids, probe_ids

    def record(self, sid: int, ok: bool) -> None:
        """Record the outcome of one cycle's read attempt of a servo.

        Args:
            sid (int): Servo ID.
            ok (bool): True when the servo was read.
        """
        if ok:
            self.failures.pop(sid, None)
            self.next_probe_at.pop(sid, None)
            return
        self.failures[sid] = self.failures.get(sid, 0) + 1
        if self.is_backed_off(sid):
            self.next_probe_at[sid] = self.clock() + self.retry_interval_s


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
    backoff: ReadBackoff | None = None,
    clock: Callable[[], int] | None = None,
) -> dict[int, ServoState]:
    """Read the raw state of each servo, using one sync read when possible.

    Every servo that answered the sync read is decoded from its reply, also when the transaction as a whole failed
    (one servo timed out); only the servos missing from the reply are read individually via fallback_read. With a
    backoff, servos that keep failing are left out of the sync read and only probed when due. Servos that cannot be
    read at all are omitted from the result, so callers never see placeholder values.

    Args:
        servo_ids: Servo IDs to read.
        group_factory: Returns a fresh st3215.GroupSyncRead-compatible object for address 56 and length.
        fallback_read: Per-servo read returning the state, or None on failure.
        length: Bytes per servo, SYNC_READ_LENGTH or SYNC_READ_LENGTH_WITH_LOAD.
        backoff: Per-servo failure tracking (updated in place); None reads every servo every cycle.
        clock: Time source in ns stamping each reading with its request time (read_at_ns); None leaves it unset.

    Returns:
        dict[int, ServoState]: servo_id -> raw state.
    """
    sync_ids, probe_ids = backoff.split(servo_ids) if backoff is not None else (list(servo_ids), [])
    result: dict[int, ServoState] = {}
    if sync_ids:
        group = group_factory()
        for sid in sync_ids:
            group.addParam(sid)
        sent_at = clock() if clock is not None else None
        # The result code is not checked: a timeout on one servo still leaves the other replies in data_dict.
        group.txRxPacket()
        for sid in sync_ids:
            available, _error = group.isAvailable(sid, PRESENT_POSITION_ADDRESS, length)
            if available:
                # Decode bytes directly: st3215 GroupSyncRead.getData calls a non-existent scs_makeword.
                result[sid] = decode_state(group.data_dict[sid][1 : length + 1])._replace(read_at_ns=sent_at)
    for sid in [s for s in sync_ids if s not in result] + probe_ids:
        sent_at = clock() if clock is not None else None
        single = fallback_read(sid)
        if single is not None:
            result[sid] = single._replace(read_at_ns=sent_at)
    if backoff is not None:
        for sid in sync_ids + probe_ids:
            backoff.record(sid, ok=sid in result)
    return result


def group_stamp_ns(joints: Sequence[JointEntry], readings: Mapping[int, ServoState]) -> int | None:
    """Return the header stamp of one group's sample: the request time of its oldest reading.

    A fallback read happens after the sync read, so the group's joints can be sampled at different times; the
    oldest one is used so the stamp never claims the sample is fresher than it is.

    Args:
        joints (Sequence[JointEntry]): Group joints (all present in readings).
        readings (Mapping[int, ServoState]): State read result.

    Returns:
        int | None: Stamp in ns, or None when a reading carries no read time.
    """
    times = [readings[j.id].read_at_ns for j in joints]
    if not times or any(t is None for t in times):
        return None
    return min(times)


def joint_efforts(
    joints: Sequence[JointEntry], readings: Mapping[int, ServoState], effort_joints: Collection[str]
) -> list[float]:
    """Decode JointState.effort for one group from the state read.

    Args:
        joints (Sequence[JointEntry]): Group joints, in message order (all present in readings).
        readings (Mapping[int, ServoState]): State read result.
        effort_joints (Collection[str]): publish_effort_joints names.

    Returns:
        list[float]: Signed load per joint, UNMEASURED_EFFORT (NaN) for joints not in effort_joints or without a
            load reading, or [] when no joint publishes effort.
    """
    if not effort_joints:
        return []
    efforts: list[float] = []
    for joint in joints:
        load = readings[joint.id].load if joint.name in effort_joints else None
        efforts.append(float(decode_present_load(load, joint.inverted)) if load is not None else UNMEASURED_EFFORT)
    return efforts
