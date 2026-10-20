"""TorqueControl on an emulated STS bus: per-group release (gate, then torque 0) and hold-present enable."""

from collections.abc import Callable
from dataclasses import dataclass, field

from st3215.protocol_packet_handler import protocol_packet_handler
from st3215.values import INST_READ, INST_WRITE

from feetech_servos.bridge_cycle import BridgeCycle
from feetech_servos.config import JointEntry, JointGroup
from feetech_servos.registers import (
    STS_GOAL_POSITION_L,
    STS_GOAL_SPEED_L,
    STS_PRESENT_POSITION_L,
    STS_TORQUE_ENABLE,
    get_register_entry_by_name,
)
from feetech_servos.torque_control import TorqueControl, TorqueResult

from .sts_bus import FakeStsPort, StsPacket, put_word, register_table

REFRESH_S = 1.0
ARM = JointGroup("follower", [JointEntry("shoulder_pan", 1), JointEntry("wrist_roll", 5), JointEntry("gripper", 6)])
STEER = [JointEntry("fl_steer", 33), JointEntry("fr_steer", 34), JointEntry("rl_steer", 37), JointEntry("rr_steer", 38)]
DRIVE = [
    JointEntry("fl_drive", 32, mode="velocity"),
    JointEntry("fr_drive", 35, mode="velocity"),
    JointEntry("rl_drive", 39, mode="velocity"),
    JointEntry("rr_drive", 36, mode="velocity"),
]
SWERVE = JointGroup("swerve_drive", STEER + DRIVE)
HEAD = JointGroup("head", [JointEntry("head_pan", 21), JointEntry("head_tilt", 22)])
GROUPS = [ARM, SWERVE, HEAD]
PRESENT = {1: 1500, 5: 2600, 6: 2100, 33: 2048, 34: 2010, 37: 2090, 38: 1990, 21: 2300, 22: 1800}


@dataclass
class FlakyPort(FakeStsPort):
    """Emulated bus whose servos can miss present_position reads or ignore torque_enable writes.

    Attributes:
        failing_reads: servo_id -> number of present_position reads still left unanswered (-1 = always).
        stuck_torque: Servo IDs whose torque_enable register ignores writes (the write is acknowledged).
        on_torque_write: Called with the servo ID of every torque_enable write, before it is applied.
    """

    failing_reads: dict[int, int] = field(default_factory=dict)
    stuck_torque: set[int] = field(default_factory=set)
    on_torque_write: Callable[[int], None] | None = None

    def apply(self, packet: StsPacket) -> None:
        """Drop a failing present_position read; keep torque_enable of a stuck servo; else behave normally."""
        sid, params = packet.servo_id, packet.params
        if packet.instruction == INST_READ and params[0] == STS_PRESENT_POSITION_L and self.failing_reads.get(sid):
            if self.failing_reads[sid] > 0:
                self.failing_reads[sid] -= 1
            return
        if packet.instruction == INST_WRITE and params[0] == STS_TORQUE_ENABLE:
            if self.on_torque_write is not None:
                self.on_torque_write(sid)
            if sid in self.stuck_torque:
                self.reply(sid, [])
                return
        super().apply(packet)


@dataclass
class FakeClock:
    """Settable monotonic clock."""

    now: float = 0.0

    def __call__(self) -> float:
        """Return the simulated time in seconds."""
        return self.now


@dataclass
class Rig:
    """One bus with every group, its BridgeCycle and TorqueControl."""

    port: FlakyPort
    cycle: BridgeCycle
    control: TorqueControl
    last_written: dict[int, dict[str, int]]
    clock: FakeClock


def make_rig(torque_on: bool = True) -> Rig:
    """Build the emulated bus (servos torqued, present positions PRESENT), cycle and TorqueControl.

    The register cache starts as the startup sequence leaves it: torque_enable cached for every servo.
    """
    tables = {}
    for joint in ARM.joints + SWERVE.joints + HEAD.joints:
        table = register_table({STS_TORQUE_ENABLE: 1 if torque_on else 0})
        put_word(table, STS_PRESENT_POSITION_L, PRESENT.get(joint.id, 0))
        tables[joint.id] = table
    port = FlakyPort(tables)
    servo = protocol_packet_handler(port)
    last_written = {sid: {"torque_enable": 1 if torque_on else 0} for sid in tables}
    clock = FakeClock()
    cycle = BridgeCycle(
        servo=servo,
        goal_entry=get_register_entry_by_name("goal_position"),
        speed_goal_entry=get_register_entry_by_name("goal_speed"),
        velocity_joints=DRIVE,
        command_limits={},
        last_written=last_written,
        velocity_command_timeout_s=0.3,
        clock=clock,
        goal_refresh_interval_s=REFRESH_S,
    )
    control = TorqueControl(servo, GROUPS, last_written, cycle, verify_sleep_s=0.0)
    return Rig(port, cycle, control, last_written, clock)


def register(port: FakeStsPort, sid: int, address: int, size: int = 2) -> int:
    """Read a register value straight from an emulated servo table."""
    table = port.servos[sid]
    return table[address] | (table[address + 1] << 8) if size == 2 else table[address]


def writes_to(port: FakeStsPort, address: int) -> list[tuple[int, list[int]]]:
    """(servo_id, data) of every acknowledged WRITE instruction to address, in order."""
    return [(p.servo_id, p.params[1:]) for p in port.packets if p.instruction == INST_WRITE and p.params[0] == address]


def goal_sync_ids(port: FakeStsPort) -> set[int]:
    """Servo IDs carried by any goal_position sync write."""
    return {w.servo_id for w in port.sync_writes if w.address == STS_GOAL_POSITION_L}


def command(rig: Rig, group: JointGroup, positions: dict[str, float], velocities: dict[str, float]) -> None:
    """Deliver one combined joint_commands message and run the write step of one loop iteration."""
    names = [j.name for j in group.joints]
    nan = float("nan")
    rig.cycle.handle_command(
        group, names, [positions.get(n, nan) for n in names], [velocities.get(n, nan) for n in names]
    )
    rig.cycle.write_pending_commands()


# --- release ---


def test_release_writes_torque_zero_although_cache_holds_zero_and_verifies_it() -> None:
    rig = make_rig()
    for joint in ARM.joints:
        rig.last_written[joint.id]["torque_enable"] = 0  # stale cache: a write_register would skip the write
    result = rig.control.release("follower")
    assert isinstance(result, TorqueResult)
    assert result.success, result.message
    assert sorted(sid for sid, _ in writes_to(rig.port, STS_TORQUE_ENABLE)) == [1, 5, 6]
    assert all(data == [0] for _, data in writes_to(rig.port, STS_TORQUE_ENABLE))
    assert all(register(rig.port, j.id, STS_TORQUE_ENABLE, 1) == 0 for j in ARM.joints)
    reads = [p.servo_id for p in rig.port.packets if p.instruction == INST_READ and p.params[0] == STS_TORQUE_ENABLE]
    assert sorted(reads) == [1, 5, 6]  # verified by readback
    assert result.joints == {"shoulder_pan": 0, "wrist_roll": 0, "gripper": 0}
    assert rig.control.released_ids == {1, 5, 6}
    assert rig.control.joint_torque == {1: 0, 5: 0, 6: 0}


def test_release_gates_the_group_before_any_torque_write() -> None:
    rig = make_rig()
    gated_at_write: list[bool] = []
    rig.port.on_torque_write = lambda sid: gated_at_write.append(sid in rig.cycle.released_ids)
    rig.control.release("follower")
    assert gated_at_write == [True, True, True]


def test_release_drops_pending_targets_of_the_group() -> None:
    rig = make_rig()
    rig.cycle.handle_command(ARM, ["wrist_roll"], [0.3], [])
    rig.control.release("follower")
    rig.cycle.write_pending_commands()
    assert goal_sync_ids(rig.port) == set()


def test_release_names_a_joint_whose_torque_off_is_not_verified() -> None:
    rig = make_rig()
    rig.port.stuck_torque.add(5)
    result = rig.control.release("follower")
    assert not result.success
    assert "wrist_roll" in result.message
    assert "shoulder_pan" not in result.message and "gripper" not in result.message
    assert result.joints["wrist_roll"] == 1  # the verified readback, not the wanted value
    assert 5 in rig.control.released_ids  # still gated: no goal reaches it


def test_release_of_released_group_succeeds_and_writes_torque_zero_again() -> None:
    rig = make_rig()
    assert rig.control.release("head").success
    rig.port.reset_counters()
    result = rig.control.release("head")
    assert result.success, result.message
    assert sorted(sid for sid, _ in writes_to(rig.port, STS_TORQUE_ENABLE)) == [21, 22]


def test_release_of_unknown_group_fails_without_bus_traffic() -> None:
    rig = make_rig()
    result = rig.control.release("nope")
    assert not result.success
    assert "nope" in result.message
    assert rig.port.packets == []


def test_no_bus_answers_failure() -> None:
    cycle = BridgeCycle(
        servo=None,
        goal_entry=get_register_entry_by_name("goal_position"),
        speed_goal_entry=get_register_entry_by_name("goal_speed"),
        velocity_joints=[],
        command_limits={},
        last_written={},
        velocity_command_timeout_s=0.3,
    )
    control = TorqueControl(None, GROUPS, {}, cycle)
    assert not control.release("head").success
    assert not control.enable("head").success


def test_released_group_gets_no_goal_over_cycles_with_refresh_due() -> None:
    rig = make_rig()
    command(rig, ARM, {"shoulder_pan": 0.1, "wrist_roll": 0.2, "gripper": 0.3}, {})
    assert goal_sync_ids(rig.port) == {1, 5, 6}
    rig.control.release("follower")
    rig.port.reset_counters()
    for _ in range(5):
        rig.clock.now += REFRESH_S * 2  # every cycle the goal refresh is due
        command(rig, ARM, {"shoulder_pan": 0.1, "wrist_roll": 0.2, "gripper": 0.3}, {})
        command(rig, ARM, {"shoulder_pan": 0.4, "wrist_roll": 0.5, "gripper": 0.6}, {})
    assert goal_sync_ids(rig.port) == set()
    assert writes_to(rig.port, STS_GOAL_POSITION_L) == []


# --- enable ---


def test_enable_writes_goal_present_before_torque_one_although_cache_holds_torque_one() -> None:
    rig = make_rig()
    rig.port.servos[5][STS_TORQUE_ENABLE] = 0  # the servo dropped torque by itself; the cache still says 1
    rig.last_written[5]["goal_position"] = PRESENT[5]  # cache already holds the present value: bypassed too
    result = rig.control.enable("follower")
    assert result.success, result.message
    order = [
        (p.params[0], p.servo_id)
        for p in rig.port.packets
        if p.instruction == INST_WRITE and p.params[0] in (STS_GOAL_POSITION_L, STS_TORQUE_ENABLE)
    ]
    for sid in (1, 5, 6):
        assert order.index((STS_GOAL_POSITION_L, sid)) < order.index((STS_TORQUE_ENABLE, sid))
    assert register(rig.port, 5, STS_TORQUE_ENABLE, 1) == 1
    assert all(register(rig.port, j.id, STS_GOAL_POSITION_L) == PRESENT[j.id] for j in ARM.joints)
    assert all(rig.last_written[j.id]["goal_position"] == PRESENT[j.id] for j in ARM.joints)
    assert result.joints == {"shoulder_pan": 1, "wrist_roll": 1, "gripper": 1}
    assert rig.control.joint_torque == {1: 1, 5: 1, 6: 1}


def test_enable_after_release_holds_present_and_lifts_the_gate() -> None:
    rig = make_rig()
    rig.control.release("follower")
    put_word(rig.port.servos[5], STS_PRESENT_POSITION_L, 3000)  # moved by hand while limp
    result = rig.control.enable("follower")
    assert result.success, result.message
    assert register(rig.port, 5, STS_GOAL_POSITION_L) == 3000
    assert rig.control.released_ids == set()
    assert rig.cycle.released_ids == set()
    rig.port.reset_counters()
    command(rig, ARM, {"wrist_roll": 0.2}, {})
    assert goal_sync_ids(rig.port) == {5}


def test_present_read_none_is_retried() -> None:
    rig = make_rig()
    rig.control.release("head")
    rig.port.failing_reads[21] = 2  # two missed replies, the third read answers
    result = rig.control.enable("head")
    assert result.success, result.message
    assert register(rig.port, 21, STS_GOAL_POSITION_L) == PRESENT[21]


def test_failing_present_read_leaves_joint_released_enables_the_rest_and_names_it() -> None:
    rig = make_rig()
    rig.control.release("follower")
    rig.port.failing_reads[5] = -1
    rig.port.reset_counters()
    result = rig.control.enable("follower")
    assert not result.success
    assert "wrist_roll" in result.message
    assert "shoulder_pan" not in result.message and "gripper" not in result.message
    assert rig.control.released_ids == {5}
    assert register(rig.port, 5, STS_TORQUE_ENABLE, 1) == 0
    assert 5 not in {sid for sid, _ in writes_to(rig.port, STS_GOAL_POSITION_L)}
    assert 5 not in {sid for sid, _ in writes_to(rig.port, STS_TORQUE_ENABLE)}
    assert register(rig.port, 1, STS_TORQUE_ENABLE, 1) == 1 and register(rig.port, 6, STS_TORQUE_ENABLE, 1) == 1
    assert result.joints == {"shoulder_pan": 1, "gripper": 1}
    rig.port.reset_counters()
    command(rig, ARM, {"shoulder_pan": 0.1, "wrist_roll": 0.2, "gripper": 0.3}, {})
    assert goal_sync_ids(rig.port) == {1, 6}


def test_unverified_torque_on_leaves_joint_released_and_names_it() -> None:
    rig = make_rig(torque_on=False)
    rig.port.stuck_torque.add(22)
    result = rig.control.enable("head")
    assert not result.success
    assert "head_tilt" in result.message and "head_pan" not in result.message
    assert rig.control.released_ids == {22}
    assert result.joints == {"head_pan": 1, "head_tilt": 0}


# --- base group: velocity joints stay torqued, are stopped and gated ---


def test_base_release_stops_wheels_keeps_their_torque_and_drops_their_commands() -> None:
    rig = make_rig()
    drive_on = {j.name: 2.0 for j in DRIVE}
    command(rig, SWERVE, {}, drive_on)
    assert all(register(rig.port, j.id, STS_GOAL_SPEED_L) != 0 for j in DRIVE)
    rig.port.reset_counters()
    result = rig.control.release("swerve_drive")
    assert result.success, result.message
    assert sorted(sid for sid, _ in writes_to(rig.port, STS_TORQUE_ENABLE)) == [33, 34, 37, 38]
    assert all(register(rig.port, j.id, STS_GOAL_SPEED_L) == 0 for j in DRIVE)
    assert all(register(rig.port, j.id, STS_TORQUE_ENABLE, 1) == 1 for j in DRIVE)
    assert rig.control.released_ids == {33, 34, 37, 38}
    assert set(result.joints) == {j.name for j in STEER}
    rig.port.reset_counters()
    for _ in range(3):
        rig.clock.now += REFRESH_S * 2
        command(rig, SWERVE, {j.name: 0.2 for j in STEER}, drive_on)
    assert writes_to(rig.port, STS_GOAL_SPEED_L) == []
    assert goal_sync_ids(rig.port) == set()


def test_base_enable_accepts_drive_commands_again_without_touching_wheel_torque() -> None:
    rig = make_rig()
    rig.control.release("swerve_drive")
    rig.port.reset_counters()
    result = rig.control.enable("swerve_drive")
    assert result.success, result.message
    assert sorted(sid for sid, _ in writes_to(rig.port, STS_TORQUE_ENABLE)) == [33, 34, 37, 38]
    command(rig, SWERVE, {}, {j.name: 2.0 for j in DRIVE})
    assert all(register(rig.port, j.id, STS_GOAL_SPEED_L) != 0 for j in DRIVE)


def test_other_group_on_same_bus_keeps_receiving_commands() -> None:
    rig = make_rig()
    rig.control.release("swerve_drive")
    rig.port.reset_counters()
    command(rig, HEAD, {"head_pan": 0.1, "head_tilt": -0.1}, {})
    assert goal_sync_ids(rig.port) == {21, 22}
    rig.control.release("head")
    rig.control.enable("swerve_drive")
    rig.port.reset_counters()
    command(rig, SWERVE, {j.name: 0.2 for j in STEER}, {j.name: 1.0 for j in DRIVE})
    assert goal_sync_ids(rig.port) == {33, 34, 37, 38}
    assert {sid for sid, _ in writes_to(rig.port, STS_GOAL_SPEED_L)} == {j.id for j in DRIVE}


# --- register dump readings ---


def test_note_register_dump_updates_joint_torque() -> None:
    rig = make_rig()
    rig.control.note_register_dump(5, {"torque_enable": 0, "present_position": 2000})
    assert rig.control.joint_torque == {5: 0}
    rig.control.note_register_dump(5, {"torque_enable": 1})
    assert rig.control.joint_torque == {5: 1}
    rig.control.note_register_dump(6, {"present_position": 2000})  # no torque_enable read: nothing recorded
    assert rig.control.joint_torque == {5: 1}
