"""Per-cycle state read: present load for publish_effort_joints folded into the position/speed sync read.

Runs the real st3215 GroupSyncRead and packet handler against the emulated bus (tests/sts_bus.py). Before,
each cycle sent one acknowledged goal write per joint plus two single present_load reads; now a cycle is one
goal sync write plus one sync read that also carries present_load.
"""

import math
from collections.abc import Sequence

from st3215.group_sync_read import GroupSyncRead

from feetech_servos.bridge_cycle import BridgeCycle
from feetech_servos.config import JointEntry, JointGroup
from feetech_servos.registers import decode_present_load, get_register_entry_by_name, read_register
from feetech_servos.sync_read import (
    PRESENT_POSITION_ADDRESS,
    SYNC_READ_LENGTH,
    SYNC_READ_LENGTH_WITH_LOAD,
    ReadBackoff,
    ServoState,
    group_stamp_ns,
    joint_efforts,
    read_servo_state,
    read_servo_states,
    sync_read_length,
)

from .sts_bus import PACKET_TIMEOUT_S, make_bus, put_word, register_table

ARM = [
    JointEntry("shoulder_pan", 1),
    JointEntry("shoulder_lift", 2),
    JointEntry("elbow_flex", 3),
    JointEntry("wrist_flex", 4),
    JointEntry("wrist_roll", 5, inverted=True),
    JointEntry("gripper", 6),
]
ARM_GROUP = JointGroup("follower", ARM)
EFFORT_JOINTS = {"wrist_roll", "gripper"}
LOADS = {1: 10, 2: 1030, 3: 0, 4: 2047, 5: 1076, 6: 52}  # raw sign-magnitude present_load per servo
SPEED_RAW = 0x8010  # sign-magnitude: -16
# The client bus: follower arm (1-6) plus the swerve servos (32-39) share one serial line.
CLIENT_IDS = [1, 2, 3, 4, 5, 6, 32, 33, 34, 35, 36, 37, 38, 39]
FOLLOWER_HZ = 95.8  # measured follower cycle rate (control_loop_hz 100) that a dead servo must not break
FAILURE_LIMIT = 3
RETRY_INTERVAL_S = 5.0
SIMULATED_S = 10.0


class FakeClock:
    """Manually advanced monotonic clock (seconds)."""

    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


def client_bus(missing: tuple[int, ...] = ()) -> tuple[object, object]:
    """Emulated client bus with every arm and swerve servo except the missing ones."""
    tables = {}
    for sid in CLIENT_IDS:
        if sid in missing:
            continue
        table = register_table()
        put_word(table, 56, 2000 + sid)
        tables[sid] = table
    return make_bus(tables)


def read_client(bus: object, backoff: ReadBackoff | None) -> dict[int, ServoState]:
    """The follower's per-cycle state read of the whole client bus."""
    return read_servo_states(
        CLIENT_IDS,
        lambda: GroupSyncRead(bus, PRESENT_POSITION_ADDRESS, SYNC_READ_LENGTH_WITH_LOAD),
        lambda sid: read_servo_state(bus, sid, SYNC_READ_LENGTH_WITH_LOAD),
        SYNC_READ_LENGTH_WITH_LOAD,
        backoff=backoff,
    )


class PartialSyncGroup:
    """GroupSyncRead double whose reply lacks some servos (and whose transaction reports a timeout)."""

    def __init__(self, states: dict[int, ServoState], absent: Sequence[int]) -> None:
        self.states = states
        self.absent = set(absent)
        self.data_dict: dict[int, list[int]] = {}

    def addParam(self, sid: int) -> bool:  # noqa: N802 - st3215 API
        self.data_dict[sid] = []
        return True

    def txRxPacket(self) -> int:  # noqa: N802 - st3215 API
        for sid in self.data_dict:
            if sid not in self.absent:
                state = self.states[sid]
                load = state.load or 0
                self.data_dict[sid] = [
                    0,
                    state.position & 0xFF,
                    state.position >> 8,
                    state.speed & 0xFF,
                    state.speed >> 8,
                    load & 0xFF,
                    load >> 8,
                ]
        return -6  # COMM_RX_TIMEOUT: the reply was incomplete

    def isAvailable(self, sid: int, _address: int, length: int) -> tuple[bool, int]:  # noqa: N802 - st3215 API
        data = self.data_dict.get(sid) or []
        return len(data) >= length + 1, 0


def arm_bus(missing: tuple[int, ...] = ()) -> tuple[object, object]:
    """Emulated arm bus: present_position 2000 + id, speed SPEED_RAW, load LOADS[id]."""
    tables = {}
    for joint in ARM:
        if joint.id in missing:
            continue
        table = register_table()
        put_word(table, 56, 2000 + joint.id)
        put_word(table, 58, SPEED_RAW)
        put_word(table, 60, LOADS[joint.id])
        tables[joint.id] = table
    return make_bus(tables)


def read_states(bus: object, length: int) -> dict[int, ServoState]:
    """The bridge's state read: one sync read, per-servo block read fallback."""
    return read_servo_states(
        [j.id for j in ARM],
        lambda: GroupSyncRead(bus, PRESENT_POSITION_ADDRESS, length),
        lambda sid: read_servo_state(bus, sid, length),
        length,
    )


def test_sync_read_length_includes_load_only_for_effort_joints() -> None:
    assert sync_read_length(with_load=False) == SYNC_READ_LENGTH == 4
    assert sync_read_length(with_load=True) == SYNC_READ_LENGTH_WITH_LOAD == 6


def test_one_cycle_is_one_sync_write_plus_one_sync_read() -> None:
    bus, port = arm_bus()
    cycle = BridgeCycle(
        servo=bus,
        goal_entry=get_register_entry_by_name("goal_position"),
        speed_goal_entry=get_register_entry_by_name("goal_speed"),
        velocity_joints=[],
        command_limits={},
        last_written={},
        velocity_command_timeout_s=0.3,
    )
    cycle.handle_command(ARM_GROUP, [j.name for j in ARM], [0.1, 0.2, 0.3, 0.4, 0.5, 0.6], [])
    cycle.write_pending_commands()
    readings = read_states(bus, SYNC_READ_LENGTH_WITH_LOAD)
    efforts = joint_efforts(ARM, readings, EFFORT_JOINTS)
    assert port.transactions == 2  # before: 6 goal writes + 1 sync read + 2 present_load reads
    assert [p.instruction for p in port.packets] == [0x83, 0x82]
    # Joints outside publish_effort_joints have no measured load: NaN, never a placeholder 0.0.
    assert all(math.isnan(e) for e in efforts[:4])
    assert efforts[4:] == [float(decode_present_load(1076, True)), float(decode_present_load(52))]


def test_folded_effort_is_decoded_identically_to_a_single_present_load_read() -> None:
    bus, _port = arm_bus()
    readings = read_states(bus, SYNC_READ_LENGTH_WITH_LOAD)
    load_entry = get_register_entry_by_name("present_load")
    efforts = joint_efforts(ARM, readings, {j.name for j in ARM})
    single = [float(decode_present_load(read_register(bus, j.id, load_entry), j.inverted)) for j in ARM]
    assert efforts == single
    for joint in ARM:
        assert readings[joint.id] == ServoState(2000 + joint.id, SPEED_RAW, LOADS[joint.id])


def test_position_and_speed_are_unchanged_by_the_longer_read() -> None:
    bus, _port = arm_bus()
    short = read_states(bus, SYNC_READ_LENGTH)
    long = read_states(bus, SYNC_READ_LENGTH_WITH_LOAD)
    assert {sid: (s.position, s.speed) for sid, s in short.items()} == {
        sid: (s.position, s.speed) for sid, s in long.items()
    }
    assert all(s.load is None for s in short.values())


def test_missing_servo_falls_back_to_one_block_read_and_is_omitted() -> None:
    bus, port = arm_bus(missing=(5,))
    readings = read_states(bus, SYNC_READ_LENGTH_WITH_LOAD)
    assert 5 not in readings
    assert readings[6] == ServoState(2006, SPEED_RAW, 52)
    # Sync read, then one fallback read of servo 5 (before: present_position + present_speed + present_load reads).
    assert port.transactions == 2
    assert port.packets[1].servo_id == 5 and port.packets[1].params == [PRESENT_POSITION_ADDRESS, 6]


def test_fallback_read_decodes_position_speed_and_load() -> None:
    bus, _port = arm_bus()
    assert read_servo_state(bus, 6, SYNC_READ_LENGTH_WITH_LOAD) == ServoState(2006, SPEED_RAW, 52)
    assert read_servo_state(bus, 6, SYNC_READ_LENGTH) == ServoState(2006, SPEED_RAW, None)
    assert read_servo_state(bus, 9, SYNC_READ_LENGTH) is None


def test_no_effort_joints_publish_no_effort() -> None:
    bus, _port = arm_bus()
    assert joint_efforts(ARM, read_states(bus, SYNC_READ_LENGTH), set()) == []


def test_unmeasured_load_of_an_effort_joint_is_nan_not_zero() -> None:
    readings = {j.id: ServoState(2000 + j.id, 0, None) for j in ARM}
    efforts = joint_efforts(ARM, readings, EFFORT_JOINTS)
    assert len(efforts) == len(ARM)
    assert all(math.isnan(e) for e in efforts)


def test_measured_zero_load_stays_zero() -> None:
    readings = {j.id: ServoState(2000 + j.id, 0, 0) for j in ARM}
    assert joint_efforts(ARM, readings, {"gripper"})[5] == 0.0


def test_dead_last_servo_others_come_from_the_sync_reply() -> None:
    """A timed-out sync read still delivers the servos that answered; only the dead one is re-read."""
    bus, port = arm_bus(missing=(6,))
    readings = read_states(bus, SYNC_READ_LENGTH_WITH_LOAD)
    assert sorted(readings) == [1, 2, 3, 4, 5]
    assert readings[5] == ServoState(2005, SPEED_RAW, LOADS[5])
    # Before: the sync read timed out on servo 6 and all six servos were re-read one by one.
    assert port.transactions == 2
    assert port.packets[1].servo_id == 6


def test_one_servo_dead_others_from_the_sync_reply_cycle_time_bounded() -> None:
    """one servo dead: others from the sync reply, cycle time bounded (follower rate kept on the client bus)."""
    period_s = 1.0 / FOLLOWER_HZ
    healthy_bus, healthy_port = client_bus()
    read_client(healthy_bus, None)
    healthy_s = healthy_port.wire_time_s()
    assert healthy_s < period_s  # the healthy read fits the cycle with room for the goal sync write

    clock = FakeClock()
    backoff = ReadBackoff(FAILURE_LIMIT, RETRY_INTERVAL_S, clock=clock)
    bus, port = client_bus(missing=(6,))
    cycle_times: list[float] = []
    while clock.now < SIMULATED_S:
        port.reset_counters()
        readings = read_client(bus, backoff)
        assert set(readings) == set(CLIENT_IDS) - {6}
        cycle_times.append(port.wire_time_s())
        clock.now += max(period_s, cycle_times[-1])
    # Until the backoff engages: one sync timeout plus ONE fallback timeout, not a read of every servo.
    assert all(t <= healthy_s + 2 * PACKET_TIMEOUT_S + 0.002 for t in cycle_times[:FAILURE_LIMIT])
    # Then the dead servo is left out of the sync read; it is probed once per RETRY_INTERVAL_S.
    slow = [t for t in cycle_times[FAILURE_LIMIT:] if t > period_s]
    assert len(slow) <= math.ceil(SIMULATED_S / RETRY_INTERVAL_S)
    rate_hz = len(cycle_times) / sum(max(period_s, t) for t in cycle_times)
    assert rate_hz >= 0.95 * FOLLOWER_HZ, f"{rate_hz:.1f} Hz"


def test_backed_off_servo_is_probed_and_rejoins_the_sync_read() -> None:
    clock = FakeClock()
    backoff = ReadBackoff(FAILURE_LIMIT, RETRY_INTERVAL_S, clock=clock)
    bus, port = client_bus(missing=(6,))
    for _ in range(FAILURE_LIMIT):
        read_client(bus, backoff)
    assert backoff.is_backed_off(6)
    port.reset_counters()
    read_client(bus, backoff)
    assert port.transactions == 1 and port.timeouts == 0  # healthy servos only, no wait on the dead one
    table = register_table()
    put_word(table, 56, 2006)
    port.servos[6] = table  # the servo comes back (e.g. after a brownout)
    clock.now += RETRY_INTERVAL_S
    readings = read_client(bus, backoff)
    assert readings[6].position == 2006  # probed with one single read
    assert not backoff.is_backed_off(6)
    port.reset_counters()
    readings = read_client(bus, backoff)
    assert 6 in readings and port.transactions == 1  # back in the one sync read


def test_backoff_needs_consecutive_failures() -> None:
    backoff = ReadBackoff(FAILURE_LIMIT, RETRY_INTERVAL_S, clock=FakeClock())
    for _ in range(FAILURE_LIMIT - 1):
        backoff.record(6, ok=False)
    backoff.record(6, ok=True)
    for _ in range(FAILURE_LIMIT - 1):
        backoff.record(6, ok=False)
    assert not backoff.is_backed_off(6)
    backoff.record(6, ok=False)
    assert backoff.is_backed_off(6)
    assert backoff.split([5, 6]) == ([5], [])  # not due for a probe yet


def test_each_reading_is_stamped_when_requested_and_the_group_stamp_is_the_oldest() -> None:
    states = {j.id: ServoState(2000 + j.id, 0, 0) for j in ARM}
    ticks = iter([100, 200, 300])
    readings = read_servo_states(
        [j.id for j in ARM],
        lambda: PartialSyncGroup(states, absent=[3]),
        lambda sid: states[sid],
        SYNC_READ_LENGTH_WITH_LOAD,
        clock=lambda: next(ticks),
    )
    assert sorted(readings) == [j.id for j in ARM]
    assert readings[1].read_at_ns == 100  # sampled when the sync read was sent
    assert readings[3].read_at_ns == 200  # the fallback read came later
    assert readings[3]._replace(read_at_ns=None) == states[3]
    assert group_stamp_ns(ARM, readings) == 100
    assert group_stamp_ns(ARM[2:3], readings) == 200
    assert group_stamp_ns(ARM[:1], {1: ServoState(1, 0)}) is None  # unstamped readings give no stamp
