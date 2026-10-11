"""Per-cycle state read: present load for publish_effort_joints folded into the position/speed sync read.

Runs the real st3215 GroupSyncRead and packet handler against the emulated bus (tests/sts_bus.py). Before,
each cycle sent one acknowledged goal write per joint plus two single present_load reads; now a cycle is one
goal sync write plus one sync read that also carries present_load.
"""

from st3215.group_sync_read import GroupSyncRead

from feetech_servos.bridge_cycle import BridgeCycle
from feetech_servos.config import JointEntry, JointGroup
from feetech_servos.registers import decode_present_load, get_register_entry_by_name, read_register
from feetech_servos.sync_read import (
    PRESENT_POSITION_ADDRESS,
    SYNC_READ_LENGTH,
    SYNC_READ_LENGTH_WITH_LOAD,
    ServoState,
    joint_efforts,
    read_servo_state,
    read_servo_states,
    sync_read_length,
)

from .sts_bus import make_bus, put_word, register_table

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
    assert efforts == [0.0, 0.0, 0.0, 0.0, float(decode_present_load(1076, True)), float(decode_present_load(52))]


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
