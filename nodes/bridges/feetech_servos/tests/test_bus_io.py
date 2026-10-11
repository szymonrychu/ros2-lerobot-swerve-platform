"""Bus transaction shape of the follower: block register dump and one sync write for all goal positions.

The real st3215 packet handler runs against the emulated bus in tests/sts_bus.py, so every assertion is on
the packets the library actually puts on the wire: transaction and byte counts, decoded SYNC_WRITE contents
and the resulting servo register tables.
"""

import json
import math
import random

from st3215.group_sync_read import GroupSyncRead

from feetech_servos.bridge_cycle import BridgeCycle
from feetech_servos.config import JointEntry, JointGroup
from feetech_servos.register_dump import RegisterDumpScheduler
from feetech_servos.registers import (
    REGISTER_MAP,
    STS_GOAL_POSITION_L,
    get_register_entry_by_name,
    read_all_registers,
    read_register,
    write_register,
)
from feetech_servos.sync_write import register_bytes, sync_write_register

from .sts_bus import FakeStsPort, make_bus, put_word, register_table

ARM = [
    JointEntry("shoulder_pan", 1),
    JointEntry("shoulder_lift", 2),
    JointEntry("elbow_flex", 3),
    JointEntry("wrist_flex", 4),
    JointEntry("wrist_roll", 5, inverted=True),
    JointEntry("gripper", 6),
]
STEER = [JointEntry("fl_steer", 33), JointEntry("fr_steer", 34), JointEntry("rl_steer", 37), JointEntry("rr_steer", 38)]
DRIVE = [
    JointEntry("fl_drive", 32, mode="velocity"),
    JointEntry("fr_drive", 35, mode="velocity"),
    JointEntry("rl_drive", 39, mode="velocity"),
    JointEntry("rr_drive", 36, mode="velocity"),
]
ARM_GROUP = JointGroup("follower", ARM)
SWERVE_GROUP = JointGroup("swerve_drive", STEER + DRIVE)
ALL_JOINTS = ARM + STEER + DRIVE
MAX_DUMP_TRANSACTIONS_PER_SERVO = 3
SYNC_READ_ADDRESS = 56
SYNC_READ_LENGTH = 6  # present_position, present_speed, present_load
RANDOM_TABLES = 20
GOAL_REFRESH_INTERVAL_S = 1.0
CYCLE_PERIOD_S = 0.01  # control_loop_hz 100
# Register values of a follower STS3215 (gripper, id 6), stored in address order; the rest of the table is zero.
RECORDED_REGISTERS = {
    "model": 777,
    "id": 6,
    "baudrate": 0,
    "min_angle_limit": 1979,
    "max_angle_limit": 3395,
    "max_temperature_limit": 70,
    "max_voltage_limit": 140,
    "min_voltage_limit": 40,
    "max_torque_limit": 1000,
    "phase": 12,
    "unloading_condition": 44,
    "led_alarm_condition": 47,
    "p_coefficient": 16,
    "d_coefficient": 32,
    "i_coefficient": 0,
    "min_startup_force": 16,
    "cw_dead_zone": 1,
    "ccw_dead_zone": 1,
    "protection_current": 250,
    "angular_resolution": 1,
    "offset": 2073,
    "mode": 0,
    "protective_torque": 20,
    "overload_torque": 80,
    "speed_p_coefficient": 10,
    "over_current_protection_time": 200,
    "torque_enable": 1,
    "acceleration": 0,
    "goal_position": 2731,
    "goal_time": 0,
    "goal_speed": 0,
    "torque_limit": 500,
    "lock": 1,
    "present_position": 2729,
    "present_speed": 0,
    "present_load": 1076,
    "present_voltage": 121,
    "present_temperature": 33,
    "status": 0,
    "moving": 0,
    "present_current": 14,
}


def recorded_table() -> bytearray:
    """Register table holding RECORDED_REGISTERS (2-byte values little-endian, in address order)."""
    table = register_table()
    for entry in REGISTER_MAP:
        value = RECORDED_REGISTERS.get(entry.name)
        if value is None:
            continue
        if entry.size == 2:
            put_word(table, entry.address, value)
        else:
            table[entry.address] = value
    return table


def single_read_decode(table: bytearray, servo_id: int) -> dict[str, int]:
    """Decode a register table the old way: one read1ByteTxRx / read2ByteTxRx per REGISTER_MAP entry."""
    bus, _port = make_bus({servo_id: bytearray(table)})
    out: dict[str, int] = {}
    for entry in REGISTER_MAP:
        value = read_register(bus, servo_id, entry)
        if value is not None:
            out[entry.name] = value
    return out


def follower_bus() -> tuple[object, FakeStsPort]:
    """Emulated follower bus: every arm and swerve servo present, present_position 2048."""
    tables = {}
    for joint in ALL_JOINTS:
        table = register_table()
        put_word(table, 56, 2048)
        tables[joint.id] = table
    return make_bus(tables)


class FakeClock:
    """Manually advanced monotonic clock (seconds)."""

    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


def make_cycle(bus: object, clock: FakeClock | None = None) -> BridgeCycle:
    """BridgeCycle over the bus with the swerve wheels in velocity mode (goal refresh every GOAL_REFRESH_INTERVAL_S)."""
    return BridgeCycle(
        servo=bus,
        goal_entry=get_register_entry_by_name("goal_position"),
        speed_goal_entry=get_register_entry_by_name("goal_speed"),
        velocity_joints=DRIVE,
        command_limits={},
        last_written={},
        velocity_command_timeout_s=0.3,
        clock=clock or FakeClock(),
        goal_refresh_interval_s=GOAL_REFRESH_INTERVAL_S,
    )


def goal_of(port: FakeStsPort, servo_id: int) -> int:
    """goal_position register value of an emulated servo."""
    table = port.servos[servo_id]
    return table[STS_GOAL_POSITION_L] | (table[STS_GOAL_POSITION_L + 1] << 8)


def sync_read(bus: object, servo_ids: list[int]) -> None:
    """One state sync read of every servo (the normal cycle's read) through the real GroupSyncRead."""
    group = GroupSyncRead(bus, SYNC_READ_ADDRESS, SYNC_READ_LENGTH)
    for sid in servo_ids:
        group.addParam(sid)
    group.txRxPacket()


# --- register dump ---


def test_register_dump_reads_one_servo_in_at_most_three_transactions() -> None:
    bus, port = make_bus({6: recorded_table()})
    read_all_registers(bus, 6)
    assert port.transactions <= MAX_DUMP_TRANSACTIONS_PER_SERVO


def test_register_dump_decodes_recorded_registers_identically_to_single_reads() -> None:
    bus, _port = make_bus({6: recorded_table()})
    blocks = read_all_registers(bus, 6)
    singles = single_read_decode(recorded_table(), 6)
    # Overlapping 2-byte entries (protection_time, velocity_i_coefficient) read their neighbour's byte as before.
    assert singles["model"] == 777 and singles["present_load"] == 1076 and singles["offset"] == 2073
    assert blocks == singles
    # Same key order too, so the /follower/servo_registers JSON is byte-identical.
    assert json.dumps(blocks, separators=(",", ":")) == json.dumps(singles, separators=(",", ":"))


def test_register_dump_decodes_random_tables_identically_to_single_reads() -> None:
    rng = random.Random(3215)
    for _ in range(RANDOM_TABLES):
        table = bytearray(rng.randrange(256) for _ in range(256))
        bus, _port = make_bus({9: bytearray(table)})
        assert json.dumps(read_all_registers(bus, 9)) == json.dumps(single_read_decode(table, 9))


def test_register_dump_of_missing_servo_is_empty_and_bounded() -> None:
    bus, port = make_bus({})
    assert read_all_registers(bus, 6) == {}
    assert port.transactions <= MAX_DUMP_TRANSACTIONS_PER_SERVO


def test_register_dump_omits_registers_while_the_servo_reports_an_error() -> None:
    bus, port = make_bus({6: recorded_table()})
    port.error_byte[6] = 0x20  # overload: every single read returned None before, so nothing was published
    assert read_all_registers(bus, 6) == {}
    assert port.transactions <= MAX_DUMP_TRANSACTIONS_PER_SERVO


def test_register_dump_cycle_costs_at_most_twice_a_normal_cycle() -> None:
    """One dump step (one servo) on top of a normal cycle stays within 2x the normal cycle's bus time."""
    bus, port = follower_bus()
    cycle = make_cycle(bus)
    ids = [j.id for j in ALL_JOINTS]
    scheduler = RegisterDumpScheduler([(j.name, j.id) for j in ALL_JOINTS], interval_s=10.0)

    def normal_cycle(position: float) -> None:
        cycle.handle_command(ARM_GROUP, [j.name for j in ARM], [position] * len(ARM), [])
        cycle.write_pending_commands()
        sync_read(bus, ids)

    normal_cycle(0.1)
    port.reset_counters()
    normal_cycle(0.2)
    normal_time = port.wire_time_s()
    port.reset_counters()
    normal_cycle(0.3)
    scheduler.step(0.0, lambda sid: read_all_registers(bus, sid))
    dump_time = port.wire_time_s()
    assert dump_time <= 2 * normal_time, f"dump cycle {dump_time * 1000:.2f} ms vs normal {normal_time * 1000:.2f} ms"


# --- goal position sync write ---


def test_goal_positions_of_a_group_go_out_as_one_sync_write_without_reply() -> None:
    bus, port = follower_bus()
    cycle = make_cycle(bus)
    cycle.handle_command(ARM_GROUP, [j.name for j in ARM], [0.1, -0.2, 0.3, -0.4, 0.5, 0.6], [])
    cycle.write_pending_commands()
    assert port.transactions == 1
    assert port.rx_bytes == 0  # broadcast: no status replies to wait for
    assert [e.servo_id for e in port.sync_writes] == [j.id for j in ARM]
    assert {e.address for e in port.sync_writes} == {STS_GOAL_POSITION_L}


def test_sync_write_carries_exactly_the_goal_values_single_writes_would() -> None:
    positions = [0.1, -0.2, 0.3, -0.4, 0.5, math.pi / 3]
    bus, port = follower_bus()
    cycle = make_cycle(bus)
    cycle.handle_command(ARM_GROUP, [j.name for j in ARM], positions, [])
    cycle.write_pending_commands()

    # Reference: the previous path, one write2ByteTxRx per joint with the same target steps.
    ref_bus, ref_port = follower_bus()
    goal = get_register_entry_by_name("goal_position")
    expected: dict[int, int] = {}
    for joint, position in zip(ARM, positions):
        steps = cycle.position_target_steps(joint, position)
        expected[joint.id] = steps
        assert write_register(ref_bus, joint.id, goal, steps, {})

    decoded = {e.servo_id: e.data[0] | (e.data[1] << 8) for e in port.sync_writes}
    assert decoded == expected
    for joint in ARM:
        assert port.servos[joint.id][42:44] == ref_port.servos[joint.id][42:44]


def test_steer_and_arm_targets_share_the_sync_write_and_wheels_keep_single_writes() -> None:
    bus, port = follower_bus()
    cycle = make_cycle(bus)
    cycle.handle_command(ARM_GROUP, ["gripper"], [0.2], [])
    names = [j.name for j in STEER + DRIVE]
    cycle.handle_command(SWERVE_GROUP, names, [0.3] * 4 + [math.nan] * 4, [math.nan] * 4 + [1.0] * 4)
    cycle.write_pending_commands()
    sync = [p for p in port.packets if p.instruction == 0x83]
    singles = [p for p in port.packets if p.instruction == 0x03]
    assert len(sync) == 1
    assert sorted(e.servo_id for e in port.sync_writes) == sorted([6] + [j.id for j in STEER])
    assert sorted(p.servo_id for p in singles) == sorted(j.id for j in DRIVE)
    assert {p.params[0] for p in singles} == {46}  # goal_speed, acknowledged write as before


def test_unchanged_goals_put_nothing_on_the_bus() -> None:
    bus, port = follower_bus()
    cycle = make_cycle(bus)
    cycle.handle_command(ARM_GROUP, ["gripper", "wrist_roll"], [0.2, 0.1], [])
    cycle.write_pending_commands()
    port.reset_counters()
    cycle.handle_command(ARM_GROUP, ["gripper", "wrist_roll"], [0.2, 0.15], [])
    cycle.write_pending_commands()
    assert [e.servo_id for e in port.sync_writes] == [5]  # only the changed joint is in the packet
    port.reset_counters()
    cycle.handle_command(ARM_GROUP, ["gripper", "wrist_roll"], [0.2, 0.15], [])
    cycle.write_pending_commands()
    assert port.transactions == 0


def test_failed_sync_write_is_not_cached_so_the_same_goal_is_sent_again() -> None:
    bus, port = follower_bus()
    cycle = make_cycle(bus)
    port.is_using = True  # the library refuses to send (COMM_PORT_BUSY), as for any failed transmit
    cycle.handle_command(ARM_GROUP, ["gripper"], [0.2], [])
    cycle.write_pending_commands()
    assert port.transactions == 0
    port.is_using = False
    cycle.handle_command(ARM_GROUP, ["gripper"], [0.2], [])
    cycle.write_pending_commands()
    assert [e.servo_id for e in port.sync_writes] == [6]


def test_register_bytes_are_little_endian_like_write2bytetxrx() -> None:
    assert register_bytes(0x0ABC, 2) == [0xBC, 0x0A]
    assert register_bytes(7, 1) == [7]


def test_sync_write_register_with_nothing_to_write_sends_nothing() -> None:
    bus, port = follower_bus()
    assert sync_write_register(bus, get_register_entry_by_name("goal_position"), {})
    assert port.transactions == 0


# --- goal refresh (a lost sync write is not cached forever) ---


def test_unchanged_goal_rewritten_after_refresh_interval() -> None:
    bus, port = follower_bus()
    clock = FakeClock()
    cycle = make_cycle(bus, clock)
    cycle.handle_command(ARM_GROUP, ["gripper", "wrist_roll"], [0.2, 0.1], [])
    cycle.write_pending_commands()
    port.reset_counters()
    clock.now = GOAL_REFRESH_INTERVAL_S / 2
    cycle.handle_command(ARM_GROUP, ["gripper", "wrist_roll"], [0.2, 0.1], [])
    cycle.write_pending_commands()
    assert port.transactions == 0  # within the interval the cache still rate-limits identical goals
    clock.now = GOAL_REFRESH_INTERVAL_S
    cycle.handle_command(ARM_GROUP, ["gripper", "wrist_roll"], [0.2, 0.1], [])
    cycle.write_pending_commands()
    assert port.transactions == 1  # both refreshed goals share one sync write
    assert sorted(e.servo_id for e in port.sync_writes) == [5, 6]


def test_lost_sync_write_is_corrected_by_the_refresh() -> None:
    """The packet is sent but a servo never applies it (corrupted on the wire): the refresh resends it."""
    bus, port = follower_bus()
    clock = FakeClock()
    cycle = make_cycle(bus, clock)
    del port.servos[6]  # the gripper misses this packet
    cycle.handle_command(ARM_GROUP, ["gripper"], [0.2], [])
    cycle.write_pending_commands()
    port.servos[6] = register_table()
    target = cycle.position_target_steps(ARM[5], 0.2)
    for _ in range(round(GOAL_REFRESH_INTERVAL_S / CYCLE_PERIOD_S) + 1):
        clock.now += CYCLE_PERIOD_S
        cycle.handle_command(ARM_GROUP, ["gripper"], [0.2], [])  # 5 Hz hold republish of the same goal
        cycle.write_pending_commands()
    assert goal_of(port, 6) == target


def test_emulated_servo_reset_corrected_within_refresh_cycles() -> None:
    """emulated servo reset -> corrected within N cycles (N = refresh interval / control period + 1)."""
    bus, port = follower_bus()
    clock = FakeClock()
    cycle = make_cycle(bus, clock)
    target = cycle.position_target_steps(ARM[5], 0.2)
    cycle.handle_command(ARM_GROUP, ["gripper"], [0.2], [])
    cycle.write_pending_commands()
    assert goal_of(port, 6) == target
    put_word(port.servos[6], STS_GOAL_POSITION_L, 2048)  # brownout reset: goal back to the power-on value
    max_cycles = round(GOAL_REFRESH_INTERVAL_S / CYCLE_PERIOD_S) + 1
    for cycles in range(1, max_cycles + 1):
        clock.now += CYCLE_PERIOD_S
        cycle.handle_command(ARM_GROUP, ["gripper"], [0.2], [])
        cycle.write_pending_commands()
        if goal_of(port, 6) == target:
            break
    assert goal_of(port, 6) == target, f"goal not restored within {max_cycles} cycles"
    assert cycles <= max_cycles


def test_refresh_disabled_with_zero_interval_keeps_the_pure_cache() -> None:
    bus, port = follower_bus()
    clock = FakeClock()
    cycle = make_cycle(bus, clock)
    cycle.goal_refresh_interval_s = 0.0
    cycle.handle_command(ARM_GROUP, ["gripper"], [0.2], [])
    cycle.write_pending_commands()
    port.reset_counters()
    clock.now = 100.0
    cycle.handle_command(ARM_GROUP, ["gripper"], [0.2], [])
    cycle.write_pending_commands()
    assert port.transactions == 0


def test_follower_cycle_bus_time_keeps_the_control_rate() -> None:
    """One full follower cycle (goal sync write incl. refresh + state sync read) fits the 100 Hz period."""
    bus, port = follower_bus()
    clock = FakeClock()
    cycle = make_cycle(bus, clock)
    ids = [j.id for j in ALL_JOINTS]
    cycle.handle_command(ARM_GROUP, [j.name for j in ARM], [0.1] * len(ARM), [])
    cycle.handle_command(SWERVE_GROUP, [j.name for j in STEER], [0.2] * len(STEER), [])
    cycle.write_pending_commands()
    clock.now = GOAL_REFRESH_INTERVAL_S  # every goal is due for a refresh: the worst case
    port.reset_counters()
    cycle.handle_command(ARM_GROUP, [j.name for j in ARM], [0.1] * len(ARM), [])
    cycle.handle_command(SWERVE_GROUP, [j.name for j in STEER], [0.2] * len(STEER), [])
    cycle.write_pending_commands()
    sync_read(bus, ids)
    assert len(port.sync_writes) == len(ARM) + len(STEER)
    assert port.wire_time_s() < CYCLE_PERIOD_S / 2
