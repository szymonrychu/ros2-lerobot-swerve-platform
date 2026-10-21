"""NeutralControl on an emulated STS bus: record neutral, save it as the EEPROM centre (DefineMiddle), stale marker."""

import importlib
import json
import os
from dataclasses import dataclass, field
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Any

import pytest
from st3215.protocol_packet_handler import protocol_packet_handler
from st3215.values import INST_READ, INST_WRITE

from feetech_servos import neutral as neutral_module
from feetech_servos.bridge_cycle import BridgeCycle
from feetech_servos.config import JointEntry, JointGroup
from feetech_servos.neutral import (
    CENTER_READ_ATTEMPTS,
    CENTER_SETTLE_S,
    LIMIT_READ_ATTEMPTS,
    LIMIT_SETTLE_S,
    NEUTRAL_MOVE_TOLERANCE_STEPS,
    STALE_ITEMS,
    NeutralControl,
    write_stale_marker,
)
from feetech_servos.registers import (
    STS_GOAL_POSITION_L,
    STS_GOAL_SPEED_L,
    STS_LOCK,
    STS_MAX_ANGLE_LIMIT_L,
    STS_MIN_ANGLE_LIMIT_L,
    STS_PRESENT_POSITION_L,
    STS_TORQUE_ENABLE,
    get_register_entry_by_name,
)
from feetech_servos.torque_control import TorqueControl, torque_state_payload

from .ros_stubs import install_service_stubs
from .sts_bus import FakeStsPort, StsPacket, put_word, register_table

install_service_stubs()

DEFINE_MIDDLE = 128
CENTER = 2048
SAVED_AT = "2026-10-20T12:00:00+00:00"
ARM_JOINTS = [
    JointEntry("shoulder_pan", 1),
    JointEntry("elbow_flex", 3),
    JointEntry("wrist_roll", 5),
    JointEntry("gripper", 6, command_min_steps=1951, command_max_steps=3377),
]
STEER = [JointEntry("fl_steer", 33), JointEntry("fr_steer", 34, command_min_steps=1500, command_max_steps=2600)]
DRIVE = [JointEntry("fl_drive", 32, mode="velocity"), JointEntry("fr_drive", 35, mode="velocity")]
HEAD = JointGroup("head", [JointEntry("head_pan", 21), JointEntry("head_tilt", 22)])
PRESENT = {1: 1500, 3: 2600, 5: 2100, 6: 2200, 33: 2148, 34: 1990, 21: 2300, 22: 1800, 32: 0, 35: 0}
LIMITS = {1: (500, 3500), 3: (700, 3300), 5: (0, 4095), 6: (1900, 3400), 33: (1000, 3000), 34: (1200, 2900)}
STALE_COMMAND_LIMITS = (0, 4095)
ARM_EEPROM_IDS = (1, 3, 5)


def to_word(data: list[int]) -> int:
    """Little-endian 2-byte value of a write's data bytes."""
    return data[0] | (data[1] << 8)


@dataclass
class NeutralPort(FakeStsPort):
    """Emulated bus whose servos implement DefineMiddle (torque_enable = 128): present_position becomes 2048.

    Attributes:
        events: (servo_id, event, value) per instruction: unlock, lock, define_middle, min_limit, max_limit,
            goal_speed, goal_position, torque.
        define_middle_error: Servos answering DefineMiddle with an error byte (nothing changes).
        no_shift: Servos acknowledging DefineMiddle without centring (the verify fails).
        limit_shift: servo_id -> steps the angle limits move by on DefineMiddle.
        stuck_limits: Servos acknowledging angle limit writes without storing them (the restore readback fails).
        limit_none_after_centre: servo_id -> limit register reads dropped right after its DefineMiddle.
        limit_misread_after_centre: servo_id -> min_angle_limit reads answering a wrong value after its DefineMiddle.
        limit_none_after_restore: Servos whose limit reads are dropped for good once a limit write arrived.
        unacked_speed: Servos that never answer a goal_speed write.
        failing_present_after_center: servo_id -> present_position reads dropped after its DefineMiddle.
        failing_present: servo_id -> present_position reads dropped from now on.
    """

    events: list[tuple[int, str, int]] = field(default_factory=list)
    define_middle_error: set[int] = field(default_factory=set)
    no_shift: set[int] = field(default_factory=set)
    limit_shift: dict[int, int] = field(default_factory=dict)
    stuck_limits: set[int] = field(default_factory=set)
    limit_none_after_centre: dict[int, int] = field(default_factory=dict)
    limit_misread_after_centre: dict[int, int] = field(default_factory=dict)
    limit_misreads: dict[int, int] = field(default_factory=dict)
    limit_none_after_restore: set[int] = field(default_factory=set)
    limit_reads_dropped: dict[int, int] = field(default_factory=dict)
    unacked_speed: set[int] = field(default_factory=set)
    failing_present_after_center: dict[int, int] = field(default_factory=dict)
    failing_present: dict[int, int] = field(default_factory=dict)

    def apply(self, packet: StsPacket) -> None:
        """Apply one instruction, emulating DefineMiddle and the configured faults."""
        sid, params = packet.servo_id, packet.params
        if packet.instruction == INST_READ and params[0] == STS_PRESENT_POSITION_L and self.failing_present.get(sid):
            self.failing_present[sid] -= 1
            return
        if packet.instruction == INST_READ and params[0] in (STS_MIN_ANGLE_LIMIT_L, STS_MAX_ANGLE_LIMIT_L):
            if self.limit_reads_dropped.get(sid):
                self.limit_reads_dropped[sid] -= 1
                return
            if params[0] == STS_MIN_ANGLE_LIMIT_L and self.limit_misreads.get(sid):
                self.limit_misreads[sid] -= 1
                table = self.servos[sid]
                true_value = word(table, STS_MIN_ANGLE_LIMIT_L)
                put_word(table, STS_MIN_ANGLE_LIMIT_L, true_value + 7)
                super().apply(packet)
                put_word(table, STS_MIN_ANGLE_LIMIT_L, true_value)
                return
        if packet.instruction != INST_WRITE:
            super().apply(packet)
            return
        address, data = params[0], params[1:]
        if address == STS_TORQUE_ENABLE and data[0] == DEFINE_MIDDLE:
            self.events.append((sid, "define_middle", DEFINE_MIDDLE))
            if sid in self.define_middle_error:
                self.error_byte[sid] = 1
                self.reply(sid, [])
                del self.error_byte[sid]
                return
            table = self.servos[sid]
            if sid not in self.no_shift:
                put_word(table, STS_PRESENT_POSITION_L, CENTER)
                self.failing_present.update({sid: self.failing_present_after_center.get(sid, 0)})
            self.limit_reads_dropped[sid] = self.limit_none_after_centre.get(sid, 0)
            self.limit_misreads[sid] = self.limit_misread_after_centre.get(sid, 0)
            shift = self.limit_shift.get(sid, 0)
            for address_l in (STS_MIN_ANGLE_LIMIT_L, STS_MAX_ANGLE_LIMIT_L):
                put_word(table, address_l, word(table, address_l) + shift)
            self.reply(sid, [])
            return
        names = {
            STS_LOCK: "lock" if data[0] else "unlock",
            STS_MIN_ANGLE_LIMIT_L: "min_limit",
            STS_MAX_ANGLE_LIMIT_L: "max_limit",
            STS_GOAL_SPEED_L: "goal_speed",
            STS_GOAL_POSITION_L: "goal_position",
            STS_TORQUE_ENABLE: "torque",
        }
        event = names.get(address, f"write_{address}")
        value = data[0] if len(data) == 1 else to_word(data)
        self.events.append((sid, event, value))
        if event in ("min_limit", "max_limit") and sid in self.limit_none_after_restore:
            self.limit_reads_dropped[sid] = 10**6
        if event in ("min_limit", "max_limit") and sid in self.stuck_limits:
            self.reply(sid, [])
            return
        if event == "goal_speed" and sid in self.unacked_speed:
            return
        super().apply(packet)


def word(table: bytearray, address: int) -> int:
    """Read a little-endian 2-byte value from a register table."""
    return table[address] | (table[address + 1] << 8)


class FakeServo(protocol_packet_handler):
    """st3215 packet handler with the EEPROM methods of st3215.ST3215 (same packets as the library's code).

    The st3215 package entry is mocked in conftest (ST3215 opens a serial port), so the three methods are mirrored
    here on the real packet handler.

    Attributes:
        raise_define_middle: Servo IDs whose DefineMiddle raises (e.g. a truncated reply).
    """

    def __init__(self, port: NeutralPort) -> None:
        super().__init__(port)
        self.raise_define_middle: set[int] = set()

    def UnLockEprom(self, sts_id: int) -> int:  # noqa: N802 - st3215 API
        """Unlock the EEPROM (lock register = 0, no reply awaited), as ST3215.UnLockEprom."""
        return self.write1ByteTxOnly(sts_id, STS_LOCK, 0)

    def LockEprom(self, sts_id: int) -> int:  # noqa: N802 - st3215 API
        """Lock the EEPROM (lock register = 1, no reply awaited), as ST3215.LockEprom."""
        return self.write1ByteTxOnly(sts_id, STS_LOCK, 1)

    def DefineMiddle(self, sts_id: int) -> bool | None:  # noqa: N802 - st3215 API
        """Write torque_enable = 128 (True on success, None on error) as ST3215.DefineMiddle, or raise if configured."""
        if sts_id in self.raise_define_middle:
            self.portHandler.events.append((sts_id, "define_middle", DEFINE_MIDDLE))
            raise IndexError("truncated status packet")
        comm, error = self.writeTxRx(sts_id, STS_TORQUE_ENABLE, 1, [DEFINE_MIDDLE])
        return True if comm == 0 and error == 0 else None


@dataclass
class Rig:
    """One bus with arm, steering (+ wheels) and head groups, the cycle, TorqueControl and NeutralControl."""

    port: NeutralPort
    servo: FakeServo
    cycle: BridgeCycle
    torque: TorqueControl
    neutral: NeutralControl
    command_limits: dict[int, tuple[int, int]]
    last_written: dict[int, dict[str, int]]
    marker: Path
    sleeps: list[float]
    errors: list[str]

    def events(self, sid: int, since: int = 0) -> list[str]:
        """Event names of one servo, in order."""
        return [event for s, event, _ in self.port.events[since:] if s == sid]


def make_rig(tmp_path: Path, marker: Path | None = None, arm_neutral: bool = True) -> Rig:
    """Build the bus (torque on, PRESENT, LIMITS), the cycle with wheels, TorqueControl and NeutralControl."""
    marker = marker if marker is not None else tmp_path / "arm" / "calibration_stale.json"
    arm = JointGroup(
        "follower",
        ARM_JOINTS,
        neutral_enabled=arm_neutral,
        neutral_exclude_joints=["gripper"],
        neutral_stale_marker=str(marker),
    )
    swerve = JointGroup("swerve_drive", STEER + DRIVE, neutral_enabled=True)
    groups = [arm, swerve, HEAD]
    tables = {}
    for joint in ARM_JOINTS + STEER + DRIVE + HEAD.joints:
        table = register_table({STS_TORQUE_ENABLE: 1})
        put_word(table, STS_PRESENT_POSITION_L, PRESENT[joint.id])
        low, high = LIMITS.get(joint.id, (0, 4095))
        put_word(table, STS_MIN_ANGLE_LIMIT_L, low)
        put_word(table, STS_MAX_ANGLE_LIMIT_L, high)
        tables[joint.id] = table
    port = NeutralPort(tables)
    servo = FakeServo(port)
    last_written = {sid: {"torque_enable": 1} for sid in tables}
    command_limits = {
        j.id: ((j.command_min_steps, j.command_max_steps) if j.command_min_steps is not None else STALE_COMMAND_LIMITS)
        for j in ARM_JOINTS + STEER + HEAD.joints
    }
    cycle = BridgeCycle(
        servo=servo,
        goal_entry=get_register_entry_by_name("goal_position"),
        speed_goal_entry=get_register_entry_by_name("goal_speed"),
        velocity_joints=DRIVE,
        command_limits=command_limits,
        last_written=last_written,
        velocity_command_timeout_s=0.3,
    )
    torque = TorqueControl(servo, groups, last_written, cycle, verify_sleep_s=0.0)
    torque.joint_torque.update({sid: 1 for sid in tables})
    sleeps: list[float] = []
    errors: list[str] = []
    neutral = NeutralControl(
        torque, command_limits, sleep=sleeps.append, now_iso=lambda: SAVED_AT, log_error=errors.append
    )
    return Rig(port, servo, cycle, torque, neutral, command_limits, last_written, marker, sleeps, errors)


def released_and_recorded(tmp_path: Path, namespace: str = "follower", **kwargs: Any) -> Rig:
    """Rig with the group released and its neutral recorded."""
    rig = make_rig(tmp_path, **kwargs)
    assert rig.torque.release(namespace).success
    assert rig.neutral.record(namespace).success
    return rig


# --- record ---------------------------------------------------------------------------------------------------------


def test_record_refused_when_not_released(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    result = rig.neutral.record("follower")
    assert result.success is False
    assert "release torque first" in result.message
    assert rig.neutral.captures == {}


def test_record_refused_when_a_neutral_joint_torque_is_not_verified_zero(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    assert rig.torque.release("follower").success
    rig.torque.joint_torque[3] = 1  # a register dump saw the servo torque back on
    result = rig.neutral.record("follower")
    assert (result.success, "release torque first" in result.message) == (False, True)


def test_record_captures_present_and_writes_nothing(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    assert rig.torque.release("follower").success
    events_before = len(rig.port.events)
    result = rig.neutral.record("follower")
    assert result.success is True
    assert rig.port.events[events_before:] == []  # no write to the bus
    assert rig.neutral.captures["follower"] == {"shoulder_pan": 1500, "elbow_flex": 2600, "wrist_roll": 2100}
    # shift present - 2048 in steps and degrees, per joint
    assert "shoulder_pan -548 steps (-48.2 deg)" in result.message
    assert "elbow_flex +552 steps (+48.5 deg)" in result.message
    assert "gripper" not in result.message


def test_record_retries_a_failed_present_read(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    assert rig.torque.release("follower").success
    rig.port.failing_present[3] = 2
    result = rig.neutral.record("follower")
    assert result.success is True
    assert rig.neutral.captures["follower"]["elbow_flex"] == 2600


def test_record_fails_when_present_never_reads(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    assert rig.torque.release("follower").success
    rig.port.failing_present[3] = 3
    result = rig.neutral.record("follower")
    assert result.success is False
    assert "elbow_flex" in result.message
    assert "follower" not in rig.neutral.captures


def test_record_and_save_refused_without_neutral_enabled(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    assert rig.torque.release("head").success
    assert rig.neutral.record("head").success is False
    assert rig.neutral.save("head").success is False


# --- save: refusals -------------------------------------------------------------------------------------------------


def test_save_refused_without_a_capture(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    assert rig.torque.release("follower").success
    result = rig.neutral.save("follower")
    assert result.success is False
    assert "record neutral first" in result.message
    assert rig.port.events[-1][1] == "torque"  # nothing after the release


def test_save_refused_when_group_not_released(tmp_path: Path) -> None:
    rig = released_and_recorded(tmp_path)
    assert rig.torque.enable("follower").success
    events_before = len(rig.port.events)
    result = rig.neutral.save("follower")
    assert result.success is False
    assert "release torque first" in result.message
    assert rig.port.events[events_before:] == []


def test_moved_since_record_refusal_writes_nothing(tmp_path: Path) -> None:
    rig = released_and_recorded(tmp_path)
    put_word(rig.port.servos[3], STS_PRESENT_POSITION_L, 2600 + NEUTRAL_MOVE_TOLERANCE_STEPS + 1)
    put_word(rig.port.servos[5], STS_PRESENT_POSITION_L, 2100 - NEUTRAL_MOVE_TOLERANCE_STEPS)  # within tolerance
    events_before = len(rig.port.events)
    result = rig.neutral.save("follower")
    assert result.success is False
    assert result.message == "/follower: moved since record: elbow_flex; record again"
    assert rig.port.events[events_before:] == []  # no wheel stop, no EEPROM write
    assert rig.neutral.last_saves == {}


def test_wheel_stop_not_acknowledged_refuses_before_any_eeprom_write(tmp_path: Path) -> None:
    rig = released_and_recorded(tmp_path)
    rig.port.unacked_speed.add(35)
    result = rig.neutral.save("follower")
    assert result.success is False
    assert "wheels not stopped: fr_drive" in result.message
    assert not any(event in ("unlock", "define_middle") for _, event, _ in rig.port.events)


# --- save: success --------------------------------------------------------------------------------------------------


def test_save_unlocks_defines_middle_locks_in_order(tmp_path: Path) -> None:
    rig = released_and_recorded(tmp_path)
    events_before = len(rig.port.events)
    result = rig.neutral.save("follower")
    assert result.success is True, result.message
    for sid in ARM_EEPROM_IDS:
        eeprom = [e for e in rig.events(sid, events_before) if e in ("unlock", "define_middle", "lock")]
        assert eeprom == ["unlock", "define_middle", "lock"]
    assert rig.sleeps.count(CENTER_SETTLE_S) == len(ARM_EEPROM_IDS)


def test_save_success_enables_torque_holding_present_and_clears_capture(tmp_path: Path) -> None:
    rig = released_and_recorded(tmp_path)
    result = rig.neutral.save("follower")
    assert result.success is True, result.message
    for sid in (1, 3, 5, 6):
        assert rig.port.servos[sid][STS_TORQUE_ENABLE] == 1
        assert word(rig.port.servos[sid], STS_GOAL_POSITION_L) == word(rig.port.servos[sid], STS_PRESENT_POSITION_L)
    assert word(rig.port.servos[1], STS_GOAL_POSITION_L) == CENTER  # held at the new centre, no jump
    assert rig.torque.released_ids == set()
    assert "follower" not in rig.neutral.captures
    assert rig.neutral.last_saves["follower"] == {
        "saved_at": SAVED_AT,
        "saved": ["shoulder_pan", "elbow_flex", "wrist_roll"],
        "failed": [],
    }


def test_excluded_joints_untouched(tmp_path: Path) -> None:
    rig = released_and_recorded(tmp_path)
    events_before = len(rig.port.events)
    assert rig.neutral.save("follower").success
    gripper_events = rig.events(6, events_before)
    assert not set(gripper_events) & {"unlock", "define_middle", "lock", "min_limit", "max_limit"}
    assert word(rig.port.servos[6], STS_PRESENT_POSITION_L) == PRESENT[6]
    assert rig.command_limits[6] == (1951, 3377)


def test_wheels_stopped_before_first_eeprom_write(tmp_path: Path) -> None:
    rig = released_and_recorded(tmp_path)
    rig.cycle.unrelease([32, 35])  # wheels of another group on this bus, driving
    rig.cycle.handle_command(JointGroup("swerve_drive", STEER + DRIVE), ["fl_drive"], [], [2.0])
    rig.cycle.write_pending_commands()
    assert word(rig.port.servos[32], STS_GOAL_SPEED_L) != 0
    events_before = len(rig.port.events)
    assert rig.neutral.save("follower").success
    events = rig.port.events[events_before:]
    first_eeprom = next(i for i, (_, event, _) in enumerate(events) if event in ("unlock", "define_middle"))
    stops = [i for i, (sid, event, value) in enumerate(events) if event == "goal_speed" and value == 0]
    stopped_ids = {events[i][0] for i in stops}
    assert stopped_ids == {32, 35}
    assert max(stops) < first_eeprom
    assert word(rig.port.servos[32], STS_GOAL_SPEED_L) == 0


def test_verify_retries_a_none_read(tmp_path: Path) -> None:
    rig = released_and_recorded(tmp_path)
    rig.port.failing_present_after_center[3] = 4
    result = rig.neutral.save("follower")
    assert result.success is True, result.message
    assert "elbow_flex" in rig.neutral.last_saves["follower"]["saved"]


def test_limits_not_rewritten_when_unchanged(tmp_path: Path) -> None:
    rig = released_and_recorded(tmp_path)
    assert rig.neutral.save("follower").success
    assert not any(event in ("min_limit", "max_limit") for _, event, _ in rig.port.events)


def test_limits_rewritten_when_changed_by_define_middle(tmp_path: Path) -> None:
    rig = released_and_recorded(tmp_path)
    rig.port.limit_shift[3] = 100
    events_before = len(rig.port.events)
    assert rig.neutral.save("follower").success
    assert [e for e in rig.events(3, events_before) if e.endswith("limit")] == ["min_limit", "max_limit"]
    assert (word(rig.port.servos[3], STS_MIN_ANGLE_LIMIT_L), word(rig.port.servos[3], STS_MAX_ANGLE_LIMIT_L)) == (
        LIMITS[3]
    )
    assert not any(e.endswith("limit") for e in rig.events(1, events_before))


def test_transient_none_limit_read_after_centre_is_not_a_failure(tmp_path: Path) -> None:
    rig = released_and_recorded(tmp_path)
    rig.port.limit_none_after_centre[1] = 2 * LIMIT_READ_ATTEMPTS  # two whole limit reads fail
    events_before = len(rig.port.events)
    result = rig.neutral.save("follower")
    assert result.success is True, result.message
    assert not any(e.endswith("limit") for e in rig.events(1, events_before))
    assert LIMIT_SETTLE_S in rig.sleeps


def test_transient_mismatching_limit_read_after_centre_writes_nothing(tmp_path: Path) -> None:
    rig = released_and_recorded(tmp_path)
    rig.port.limit_misread_after_centre[1] = 1
    events_before = len(rig.port.events)
    result = rig.neutral.save("follower")
    assert result.success is True, result.message
    assert not any(e.endswith("limit") for e in rig.events(1, events_before))
    assert LIMIT_SETTLE_S in rig.sleeps


def test_stable_limit_mismatch_is_restored_inside_unlock_and_lock(tmp_path: Path) -> None:
    rig = released_and_recorded(tmp_path)
    rig.port.limit_shift[3] = 100
    events_before = len(rig.port.events)
    assert rig.neutral.save("follower").success
    events = rig.events(3, events_before)
    restore = events[events.index("lock") + 1 :]  # after DefineMiddle's own lock
    assert restore[0] == "unlock"
    assert restore.index("unlock") < restore.index("min_limit") < restore.index("max_limit")
    assert restore.index("max_limit") < len(restore) - 1 - restore[::-1].index("lock")
    assert (word(rig.port.servos[3], STS_MIN_ANGLE_LIMIT_L), word(rig.port.servos[3], STS_MAX_ANGLE_LIMIT_L)) == (
        LIMITS[3]
    )


def test_restore_lock_written_when_a_limit_write_fails(tmp_path: Path) -> None:
    rig = released_and_recorded(tmp_path)
    rig.port.limit_shift[1] = 100
    rig.port.stuck_limits.add(1)
    events_before = len(rig.port.events)
    assert rig.neutral.save("follower").success is False
    events = rig.events(1, events_before)
    assert events.index("lock", events.index("define_middle")) < events.index("min_limit")
    last_lock = len(events) - 1 - events[::-1].index("lock")
    assert last_lock > events.index("min_limit")


def test_limit_readback_never_succeeding_after_restore_reports_the_first_observed_limits(tmp_path: Path) -> None:
    rig = released_and_recorded(tmp_path)
    rig.port.limit_shift[1] = 100
    rig.port.limit_none_after_restore.add(1)
    result = rig.neutral.save("follower")
    assert result.success is False
    shifted = (LIMITS[1][0] + 100, LIMITS[1][1] + 100)
    assert "angle limits not restored: read back None, wanted" in result.message
    assert f"first read after centre {shifted}" in result.message
    assert_group_left_released(rig)


def test_command_limits_refreshed_for_eeprom_limited_joints_only(tmp_path: Path) -> None:
    rig = released_and_recorded(tmp_path, namespace="swerve_drive")
    assert rig.command_limits[33] == STALE_COMMAND_LIMITS
    result = rig.neutral.save("swerve_drive")
    assert result.success is True, result.message
    assert rig.command_limits[33] == LIMITS[33]  # from the final limit readback
    assert rig.command_limits[34] == (1500, 2600)  # config steps kept


def test_command_limits_follow_the_restored_limits(tmp_path: Path) -> None:
    rig = released_and_recorded(tmp_path)
    rig.port.limit_shift[1] = -50
    assert rig.neutral.save("follower").success
    assert rig.command_limits[1] == LIMITS[1]
    assert rig.command_limits[3] == LIMITS[3]


def test_lock_written_when_define_middle_fails(tmp_path: Path) -> None:
    rig = released_and_recorded(tmp_path)
    rig.port.define_middle_error.add(1)
    events_before = len(rig.port.events)
    result = rig.neutral.save("follower")
    assert result.success is False
    assert [e for e in rig.events(1, events_before) if e in ("unlock", "define_middle", "lock")] == [
        "unlock",
        "define_middle",
        "lock",
    ]
    assert_group_left_released(rig)
    assert "shoulder_pan (DefineMiddle failed)" in result.message


def test_lock_written_when_define_middle_raises(tmp_path: Path) -> None:
    rig = released_and_recorded(tmp_path)
    rig.servo.raise_define_middle.add(1)
    events_before = len(rig.port.events)
    result = rig.neutral.save("follower")
    assert result.success is False
    assert [e for e in rig.events(1, events_before) if e in ("unlock", "define_middle", "lock")] == [
        "unlock",
        "define_middle",
        "lock",
    ]
    assert "shoulder_pan (DefineMiddle raised IndexError" in result.message


def assert_group_left_released(rig: Rig) -> None:
    """Every arm position joint (gripper included) is gated with torque 0."""
    for sid in (1, 3, 5, 6):
        assert rig.port.servos[sid][STS_TORQUE_ENABLE] == 0, sid
    assert {1, 3, 5, 6} <= rig.torque.released_ids


def test_failed_verify_aborts_the_rest_and_leaves_torque_off(tmp_path: Path) -> None:
    rig = released_and_recorded(tmp_path)
    rig.port.no_shift.add(3)
    rig.port.limit_shift[3] = 100  # limits moved, but the centre is not verified: never written
    events_before = len(rig.port.events)
    result = rig.neutral.save("follower")
    assert result.success is False
    assert not any(e.endswith("limit") for e in rig.events(3, events_before))
    assert not {"unlock", "define_middle", "lock"} & set(rig.events(5, events_before))  # not attempted
    assert_group_left_released(rig)
    assert rig.port.servos[5][STS_TORQUE_ENABLE] == 0
    assert "saved: shoulder_pan" in result.message
    assert "failed: elbow_flex (centre not verified" in result.message
    assert "not attempted: wrist_roll" in result.message
    assert "torque stays off" in result.message
    assert rig.neutral.captures["follower"] == {"elbow_flex": 2600, "wrist_roll": 2100}
    assert rig.neutral.last_saves["follower"] == {
        "saved_at": SAVED_AT,
        "saved": ["shoulder_pan"],
        "failed": ["elbow_flex"],
    }


def test_verify_gives_up_after_center_read_attempts(tmp_path: Path) -> None:
    rig = released_and_recorded(tmp_path)
    rig.port.failing_present_after_center[1] = CENTER_READ_ATTEMPTS
    result = rig.neutral.save("follower")
    assert result.success is False
    assert "failed: shoulder_pan (centre not verified: read failed" in result.message
    assert "not attempted: elbow_flex, wrist_roll" in result.message


def test_failed_limit_restore_aborts_the_rest_and_leaves_torque_off(tmp_path: Path) -> None:
    rig = released_and_recorded(tmp_path)
    rig.port.limit_shift[1] = 100
    rig.port.stuck_limits.add(1)
    events_before = len(rig.port.events)
    result = rig.neutral.save("follower")
    assert result.success is False
    assert "failed: shoulder_pan (centre changed, angle limits not restored" in result.message
    assert "not attempted: elbow_flex, wrist_roll" in result.message
    assert "unlock" not in rig.events(3, events_before)
    assert_group_left_released(rig)
    assert rig.command_limits[1] == STALE_COMMAND_LIMITS  # never refreshed from unverified limits


def test_partial_failure_keeps_group_released_and_names_joints(tmp_path: Path) -> None:
    rig = released_and_recorded(tmp_path)
    rig.port.define_middle_error.add(5)
    result = rig.neutral.save("follower")
    assert result.success is False
    assert "saved: shoulder_pan, elbow_flex" in result.message
    assert "failed: wrist_roll (DefineMiddle failed)" in result.message
    assert "not attempted: none" in result.message
    assert_group_left_released(rig)
    # The saved joints' centre is in the stale marker even though the save failed.
    marker = json.loads(rig.marker.read_text())
    assert set(marker["joints"]) == {"shoulder_pan", "elbow_flex"}
    # A second save after fixing the fault saves only the joints left, then enables the group.
    rig.port.define_middle_error.clear()
    events_before = len(rig.port.events)
    result = rig.neutral.save("follower")
    assert result.success is True, result.message
    assert "unlock" not in rig.events(1, events_before)
    assert rig.events(5, events_before).count("define_middle") == 1
    assert rig.torque.released_ids == set()


# --- stale marker ---------------------------------------------------------------------------------------------------


def test_marker_content_after_save(tmp_path: Path) -> None:
    rig = released_and_recorded(tmp_path)
    assert rig.neutral.save("follower").success
    assert json.loads(rig.marker.read_text()) == {
        "version": 1,
        "group": "follower",
        "saved_at": SAVED_AT,
        "joints": {
            "shoulder_pan": {"before_steps": 1500, "after_steps": CENTER},
            "elbow_flex": {"before_steps": 2600, "after_steps": CENTER},
            "wrist_roll": {"before_steps": 2100, "after_steps": CENTER},
        },
        "stale": ["joint_offsets", "home"],
    }


def test_no_marker_without_neutral_stale_marker(tmp_path: Path) -> None:
    rig = released_and_recorded(tmp_path, namespace="swerve_drive")
    assert rig.neutral.save("swerve_drive").success
    assert not rig.marker.exists()


def test_marker_merges_an_existing_marker(tmp_path: Path) -> None:
    marker = tmp_path / "calibration_stale.json"
    marker.write_text(
        json.dumps(
            {
                "version": 1,
                "group": "follower",
                "saved_at": "2026-10-01T00:00:00+00:00",
                "joints": {
                    "wrist_flex": {"before_steps": 1000, "after_steps": 2048},
                    "shoulder_pan": {"before_steps": 1, "after_steps": 2},
                },
                "stale": ["joint_offsets"],
            }
        )
    )
    write_stale_marker(marker, "follower", SAVED_AT, {"shoulder_pan": (1500, 2048)})
    assert json.loads(marker.read_text()) == {
        "version": 1,
        "group": "follower",
        "saved_at": SAVED_AT,
        "joints": {
            "wrist_flex": {"before_steps": 1000, "after_steps": 2048},
            "shoulder_pan": {"before_steps": 1500, "after_steps": 2048},
        },
        "stale": list(STALE_ITEMS),
    }


def test_marker_written_atomically_with_parents_created(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    marker = tmp_path / "a" / "b" / "calibration_stale.json"
    replaced: list[tuple[str, str]] = []
    real_replace = os.replace

    def recording_replace(src: str, dst: str) -> None:
        replaced.append((str(src), str(dst)))
        assert json.loads(Path(src).read_text())["joints"] == {"head_pan": {"before_steps": 10, "after_steps": 2048}}
        real_replace(src, dst)

    monkeypatch.setattr(neutral_module.os, "replace", recording_replace)
    write_stale_marker(marker, "head", SAVED_AT, {"head_pan": (10, 2048)})
    assert len(replaced) == 1
    src, dst = replaced[0]
    assert Path(src).parent == marker.parent and src != str(marker)
    assert dst == str(marker)
    assert [p.name for p in marker.parent.iterdir()] == [marker.name]  # no temp file left


def test_marker_failure_message_keeps_torque_enabled(tmp_path: Path) -> None:
    blocker = tmp_path / "not_a_dir"
    blocker.write_text("")
    rig = released_and_recorded(tmp_path, marker=blocker / "calibration_stale.json")
    result = rig.neutral.save("follower")
    assert result.success is False
    assert result.message.startswith("/follower: neutral saved, stale marker not written: ")
    assert rig.torque.released_ids == set()
    assert all(rig.port.servos[sid][STS_TORQUE_ENABLE] == 1 for sid in (1, 3, 5, 6))
    assert len(rig.errors) == 1 and "stale marker not written" in rig.errors[0]


# --- state payload --------------------------------------------------------------------------------------------------


def test_status_reports_capture_and_last_save(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    assert rig.neutral.status("follower") == {"neutral_capture": None, "last_save": None}
    assert rig.torque.release("follower").success
    assert rig.neutral.record("follower").success
    assert rig.neutral.status("follower")["neutral_capture"] == {
        "shoulder_pan": {"steps": 1500, "delta_steps": -548},
        "elbow_flex": {"steps": 2600, "delta_steps": 552},
        "wrist_roll": {"steps": 2100, "delta_steps": 52},
    }
    assert rig.neutral.save("follower").success
    assert rig.neutral.status("follower") == {
        "neutral_capture": None,
        "last_save": {"saved_at": SAVED_AT, "saved": ["shoulder_pan", "elbow_flex", "wrist_roll"], "failed": []},
    }


def test_payload_carries_neutral_fields() -> None:
    group = JointGroup("head", HEAD.joints, neutral_enabled=True)
    capture = {"head_pan": {"steps": 2300, "delta_steps": 252}}
    payload = torque_state_payload(group, {21: 0}, {21}, 7, neutral_capture=capture)
    assert payload == {
        "namespace": "head",
        "released": True,
        "joints": {"head_pan": 0},
        "stamp_ns": 7,
        "neutral_enabled": True,
        "neutral_capture": capture,
        "last_save": None,
    }
    assert torque_state_payload(HEAD, {21: 1}, set(), 8)["neutral_enabled"] is False


# --- bridge services ------------------------------------------------------------------------------------------------


class RecordingNode:
    """rclpy node double recording services and publishers."""

    def __init__(self) -> None:
        self.services: dict[str, Any] = {}
        self.publishers: dict[str, list[Any]] = {}

    def create_service(self, _srv_type: Any, name: str, callback: Any) -> SimpleNamespace:
        """Record a service callback by name."""
        self.services[name] = callback
        return SimpleNamespace()

    def create_publisher(self, _msg_type: Any, name: str, _qos: Any) -> SimpleNamespace:
        """Record a publisher; its messages land in publishers[name]."""
        self.publishers[name] = []
        return SimpleNamespace(publish=self.publishers[name].append)


@pytest.fixture
def bridge_module(monkeypatch: pytest.MonkeyPatch) -> ModuleType:
    """Import feetech_servos.bridge with ROS stubbed and String replaced by a plain message class."""
    module = importlib.import_module("feetech_servos.bridge")
    monkeypatch.setattr(module, "String", lambda data: SimpleNamespace(data=data))
    return module


def call(node: RecordingNode, name: str, callbacks_run: list[int]) -> SimpleNamespace:
    """Call a recorded service callback like the executor does."""
    response = SimpleNamespace(success=None, message=None)
    before = callbacks_run[0]
    node.services[name](SimpleNamespace(), response)
    assert callbacks_run[0] == before + 1
    return response


def test_neutral_services_only_for_neutral_enabled_groups(tmp_path: Path, bridge_module: ModuleType) -> None:
    rig = make_rig(tmp_path)
    node = RecordingNode()
    groups = list(rig.torque.groups.values())
    bridge_module.setup_torque_interfaces(node, groups, rig.torque, [0], 1.0, rig.neutral)
    assert "/follower/servo/record_neutral" in node.services
    assert "/follower/servo/save_neutral" in node.services
    assert "/swerve_drive/servo/record_neutral" in node.services
    assert "/swerve_drive/servo/save_neutral" in node.services
    assert "/head/servo/record_neutral" not in node.services
    assert "/head/servo/save_neutral" not in node.services
    assert "/head/servo/release_torque" in node.services


def test_no_neutral_services_when_disabled_everywhere(tmp_path: Path, bridge_module: ModuleType) -> None:
    rig = make_rig(tmp_path, arm_neutral=False)
    node = RecordingNode()
    arm = rig.torque.groups["follower"]
    bridge_module.setup_torque_interfaces(node, [arm, HEAD], rig.torque, [0], 1.0, rig.neutral)
    assert not [name for name in node.services if name.endswith("_neutral")]


def test_record_and_save_publish_torque_state(tmp_path: Path, bridge_module: ModuleType) -> None:
    rig = make_rig(tmp_path)
    node = RecordingNode()
    callbacks_run = [0]
    groups = list(rig.torque.groups.values())
    reporter = bridge_module.setup_torque_interfaces(node, groups, rig.torque, callbacks_run, 0.0, rig.neutral)
    reporter.step(0.0, 1)
    assert call(node, "/follower/servo/release_torque", callbacks_run).success
    reporter.step(0.1, 2)

    def states() -> list[dict[str, Any]]:
        return [json.loads(m.data) for m in node.publishers["/follower/torque_state"]]

    published = len(states())
    response = call(node, "/follower/servo/record_neutral", callbacks_run)
    assert response.success is True
    reporter.step(0.2, 3)
    assert len(states()) == published + 1
    assert states()[-1]["neutral_enabled"] is True
    assert states()[-1]["neutral_capture"]["elbow_flex"] == {"steps": 2600, "delta_steps": 552}
    # Recording the same pose again changes nothing in the payload but is still published.
    assert call(node, "/follower/servo/record_neutral", callbacks_run).success
    reporter.step(0.3, 4)
    assert len(states()) == published + 2

    response = call(node, "/follower/servo/save_neutral", callbacks_run)
    assert response.success is True, response.message
    reporter.step(0.4, 5)
    state = states()[-1]
    assert state["released"] is False
    assert state["neutral_capture"] is None
    assert state["last_save"]["saved"] == ["shoulder_pan", "elbow_flex", "wrist_roll"]
    head_states = [json.loads(m.data) for m in node.publishers["/head/torque_state"]]
    assert head_states[-1]["neutral_enabled"] is False
