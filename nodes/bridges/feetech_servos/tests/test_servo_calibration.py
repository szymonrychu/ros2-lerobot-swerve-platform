"""ServoCalibrationControl (record home, range capture, plan, status) on an emulated STS bus.

The rig holds the head (head_pan, head_tilt) and the base steering (fl_steer, fr_steer plus two drive wheels) on one
bus, both with a servo_calibration block; every servo reads present = (encoder - offset) mod 4096 (STS3215).
"""

import json
import os
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest
from feetech_mapping.command_mapping import position_to_raw_steps, steps_to_radians
from st3215.values import INST_WRITE

from feetech_servos.bridge_cycle import BridgeCycle
from feetech_servos.calib_event import CalibEventLog
from feetech_servos.config import JointEntry, JointGroup, ServoCalibrationConfig, ZeroRestoreConfig
from feetech_servos.registers import (
    STS_LOCK,
    STS_MAX_ANGLE_LIMIT_L,
    STS_MIN_ANGLE_LIMIT_L,
    STS_OFS_L,
    STS_PRESENT_POSITION_L,
    STS_TORQUE_ENABLE,
    get_register_entry_by_name,
)
from feetech_servos.servo_calibration import ServoCalibrationControl
from feetech_servos.servo_offsets import decode_offset, encode_offset, steps_to_degrees
from feetech_servos.torque_control import TorqueControl
from feetech_servos.zero_restore import ZeroRestoreControl, load_reference

from .offset_bus import Clock, OffsetPort
from .sts_bus import StsPacket, put_word, register_table
from .test_neutral import FakeServo, word

HEAD = [JointEntry("head_pan", 21), JointEntry("head_tilt", 22)]
STEER = [JointEntry("fl_steer", 33), JointEntry("fr_steer", 34)]
DRIVE = [JointEntry("fl_drive", 32, mode="velocity"), JointEntry("fr_drive", 35, mode="velocity")]
TARGET = position_to_raw_steps(0.0, False)
ENCODER = {21: TARGET + 52, 22: TARGET - 40, 33: TARGET + 30, 34: TARGET - 20, 32: 0, 35: 0}
OFFSETS = {21: 10, 22: -5, 33: 0, 34: 7}
LIMITS = {21: (1000, 3000), 22: (1200, 2800), 33: (0, 4095), 34: (0, 4095)}
NOW = datetime(2026, 11, 10, 12, 0, 0, tzinfo=UTC)
EEPROM_ADDRESSES = (STS_OFS_L, STS_MIN_ANGLE_LIMIT_L, STS_MAX_ANGLE_LIMIT_L, STS_LOCK)


@dataclass
class Rig:
    """Head and steering groups on one offset-aware bus with ServoCalibrationControl."""

    port: OffsetPort
    torque: TorqueControl
    cal: ServoCalibrationControl
    zero: ZeroRestoreControl
    clock: Clock
    lines: list[str]

    def calib_events(self) -> list[dict[str, Any]]:
        """Parsed calib_event lines."""
        return [json.loads(line.split(" ", 1)[1]) for line in self.lines if line.startswith("calib_event ")]


class WriteLog(OffsetPort):
    """OffsetPort recording every write instruction's start address.

    Attributes:
        writes: (servo_id, address) of every write instruction, in order.
        max_limit_skew: servo_id -> steps added to the next max_angle_limit write it stores (one-shot fault).
    """

    writes: list[tuple[int, int]]
    max_limit_skew: dict[int, int]

    def apply(self, packet: StsPacket) -> None:
        """Record (servo_id, address) of writes, then apply (skewing a faulted max_angle_limit write)."""
        if packet.instruction == INST_WRITE and packet.params:
            self.writes.append((packet.servo_id, packet.params[0]))
        super().apply(packet)
        skew = self.max_limit_skew.pop(packet.servo_id, None) if packet.params else None
        if skew is not None and packet.instruction == INST_WRITE and packet.params[0] == STS_MAX_ANGLE_LIMIT_L:
            table = self.servos[packet.servo_id]
            put_word(table, STS_MAX_ANGLE_LIMIT_L, word(table, STS_MAX_ANGLE_LIMIT_L) + skew)
        elif skew is not None:
            self.max_limit_skew[packet.servo_id] = skew


def make_config(reference: dict[str, float], tmp_path: Path, **changes: Any) -> ServoCalibrationConfig:
    """servo_calibration block with the given home references."""
    return ServoCalibrationConfig(home_reference_rad=reference, **{"record_dir": str(tmp_path), **changes})


def make_rig(
    tmp_path: Path,
    head_block: bool = True,
    released: tuple[str, ...] = (),
    overrides: dict[int, tuple[int, int]] | None = None,
    head_extra: dict[str, Any] | None = None,
    **changes: Any,
) -> Rig:
    """Build head + swerve_drive (torque on), optionally released, with their servo_calibration blocks.

    overrides are config command limits per servo id (they also seed the bridge command limits, as in the node).
    head_extra are further JointGroup fields of the head (neutral_stale_marker, zero_restore: the arm bookkeeping).
    """
    head = JointGroup(
        "head",
        list(HEAD),
        servo_calibration=make_config({"head_pan": 0.0, "head_tilt": 0.0}, tmp_path, **changes) if head_block else None,
        **(head_extra or {}),
    )
    swerve = JointGroup(
        "swerve_drive",
        STEER + DRIVE,
        servo_calibration=make_config({"fl_steer": 0.0, "fr_steer": 0.0}, tmp_path, **changes),
    )
    port = WriteLog(servos={})
    port.writes = []
    port.max_limit_skew = {}
    for joint in HEAD + STEER + DRIVE:
        table = register_table({STS_TORQUE_ENABLE: 1, STS_LOCK: 1})
        low, high = LIMITS.get(joint.id, (0, 4095))
        put_word(table, STS_MIN_ANGLE_LIMIT_L, low)
        put_word(table, STS_MAX_ANGLE_LIMIT_L, high)
        put_word(table, STS_OFS_L, encode_offset(OFFSETS.get(joint.id, 0)))
        port.servos[joint.id] = table
        port.encoder[joint.id] = ENCODER[joint.id] + OFFSETS.get(joint.id, 0)
        port.refresh_present(joint.id)
    servo = FakeServo(port)
    last_written = {sid: {"torque_enable": 1} for sid in port.servos}
    overrides = dict(overrides or {})
    command_limits = {j.id: overrides.get(j.id, (0, 4095)) for j in HEAD + STEER}
    cycle = BridgeCycle(
        servo=servo,
        goal_entry=get_register_entry_by_name("goal_position"),
        speed_goal_entry=get_register_entry_by_name("goal_speed"),
        velocity_joints=DRIVE,
        command_limits=command_limits,
        last_written=last_written,
        velocity_command_timeout_s=0.3,
    )
    torque = TorqueControl(servo, [head, swerve], last_written, cycle, verify_sleep_s=0.0)
    torque.joint_torque.update({sid: 1 for sid in port.servos})
    for namespace in released:
        assert torque.release(namespace).success
    clock = Clock()
    lines: list[str] = []
    zero = ZeroRestoreControl(torque, command_limits, sleep=lambda _s: None, monotonic=clock, now=lambda: NOW)
    cal = ServoCalibrationControl(
        torque,
        command_limits,
        overrides,
        sleep=lambda _s: None,
        monotonic=clock,
        now=lambda: NOW,
        events=CalibEventLog(lines.append, lines.append, lines.append),
        zero_restore=zero,
    )
    return Rig(port, torque, cal, zero, clock, lines)


def readings(**steps: int) -> dict[int, SimpleNamespace]:
    """Sync read readings (servo_id -> object with .position) from joint name -> steps."""
    ids = {j.name: j.id for j in HEAD + STEER}
    return {ids[name]: SimpleNamespace(position=value) for name, value in steps.items()}


def section(rig: Rig, namespace: str) -> dict[str, Any]:
    """servo_calibration status of a group."""
    status = rig.cal.status(namespace)["servo_calibration"]
    assert status is not None
    return status


def home_of(namespace: str) -> dict[str, int]:
    """Home readings of a group's joints on the rig bus."""
    joints = HEAD if namespace == "head" else STEER
    return {j.name: ENCODER[j.id] for j in joints}


# --- record home ------------------------------------------------------------------------------------------------------


def test_record_home_refused_when_not_released(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    result = rig.cal.record_home("head")
    assert result.success is False
    assert result.message.startswith("/head: ")
    assert "release torque first" in result.message
    assert section(rig, "head")["home"] is None


def test_record_home_refused_when_bus_reads_torque_on(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, released=("head",))
    rig.port.servos[22][STS_TORQUE_ENABLE] = 1
    result = rig.cal.record_home("head")
    assert result.success is False
    assert result.message.startswith("/head: torque not off")
    assert "head_tilt" in result.message
    assert section(rig, "head")["home"] is None


def test_record_home_refused_while_moving(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, released=("head",))
    rig.port.jitter[21] = [0, 4, 9, 12]
    result = rig.cal.record_home("head")
    assert result.success is False
    assert "head_pan moved" in result.message
    assert section(rig, "head")["home"] is None


def test_record_home_snapshots_presents_and_offsets_and_writes_nothing(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, released=("head",))
    rig.port.writes.clear()
    result = rig.cal.record_home("head")
    assert result.success, result.message
    assert "nothing written" in result.message
    eeprom_writes = [w for w in rig.port.writes if w[1] in EEPROM_ADDRESSES]
    assert eeprom_writes == []
    state = rig.cal.states["head"]
    assert state.home_steps == home_of("head")
    assert state.offsets == {"head_pan": 10, "head_tilt": -5}
    assert state.home_recorded_at == NOW.isoformat(timespec="seconds")
    home = section(rig, "head")["home"]
    assert home["recorded_at"] == "2026-11-10T12:00:00+00:00"
    assert home["joints"]["head_pan"]["steps"] == ENCODER[21]
    assert isinstance(home["joints"]["head_pan"]["rad"], float)
    steps = [e["step"] for e in rig.calib_events() if e["flow"] == "servo_calibration"]
    assert steps == ["record_home", "record_home"]


def test_record_home_discards_capture_and_plan(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, released=("head",))
    assert rig.cal.record_home("head").success
    assert rig.cal.capture_start("head").success
    rig.cal.observe(readings(head_pan=ENCODER[21] - 200, head_tilt=ENCODER[22] - 200))
    rig.cal.observe(readings(head_pan=ENCODER[21] + 200, head_tilt=ENCODER[22] + 200))
    assert rig.cal.capture_stop("head").success
    assert rig.cal.plan("head").success
    assert section(rig, "head")["plan"] is not None
    assert rig.cal.record_home("head").success
    status = section(rig, "head")
    assert status["capture"] is None
    assert status["plan"] is None
    assert rig.cal.states["head"].trackers == {}


# --- capture ----------------------------------------------------------------------------------------------------------


def test_capture_start_needs_home(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, released=("head",))
    result = rig.cal.capture_start("head")
    assert result.success is False
    assert result.message.startswith("/head: ")
    assert "record home first" in result.message
    assert section(rig, "head")["capture"] is None


def test_observe_folds_min_max_per_joint(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, released=("head",))
    assert rig.cal.record_home("head").success
    assert rig.cal.capture_start("head").success
    pan, tilt = ENCODER[21], ENCODER[22]
    rig.cal.observe(readings(head_pan=pan - 100, head_tilt=tilt + 30))
    rig.cal.observe(readings(head_pan=pan + 250, head_tilt=tilt - 80))
    rig.cal.observe(readings(head_pan=pan + 10))  # head_tilt missing: skipped
    trackers = rig.cal.states["head"].trackers
    assert (trackers["head_pan"].min_rel, trackers["head_pan"].max_rel) == (-100, 250)
    assert (trackers["head_tilt"].min_rel, trackers["head_tilt"].max_rel) == (-80, 30)
    assert trackers["head_tilt"].samples == 2
    capture = section(rig, "head")["capture"]
    assert capture["running"] is True
    assert capture["paused"] is False
    assert capture["samples"] == 3
    assert capture["joints"]["head_pan"] == {
        "present_steps": pan + 10,
        "min_steps": pan - 100,
        "max_steps": pan + 250,
        "range_deg": steps_to_degrees(350),
    }


def test_observe_pauses_while_torque_on(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, released=("head",))
    assert rig.cal.record_home("head").success
    assert rig.cal.capture_start("head").success
    rig.cal.observe(readings(head_pan=ENCODER[21] + 20, head_tilt=ENCODER[22]))
    assert rig.torque.enable("head").success
    rig.cal.observe(readings(head_pan=ENCODER[21] + 900, head_tilt=ENCODER[22] - 900))
    tracker = rig.cal.states["head"].trackers["head_pan"]
    assert tracker.max_rel == 20
    assert tracker.samples == 1
    rig.clock.now += 1.0
    capture = section(rig, "head")["capture"]
    assert capture["running"] is True
    assert capture["paused"] is True
    assert rig.torque.release("head").success
    rig.cal.observe(readings(head_pan=ENCODER[21] + 40, head_tilt=ENCODER[22]))
    assert tracker.max_rel == 40
    assert rig.cal.states["head"].paused is False


def test_two_groups_capture_at_once(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, released=("head", "swerve_drive"))
    for namespace in ("head", "swerve_drive"):
        assert rig.cal.record_home(namespace).success
        assert rig.cal.capture_start(namespace).success
    rig.cal.observe(
        readings(head_pan=ENCODER[21] - 300, head_tilt=ENCODER[22], fl_steer=ENCODER[33] + 400, fr_steer=ENCODER[34])
    )
    rig.cal.observe(
        readings(head_pan=ENCODER[21] + 300, head_tilt=ENCODER[22], fl_steer=ENCODER[33] - 400, fr_steer=ENCODER[34])
    )
    head = rig.cal.states["head"].trackers
    steer = rig.cal.states["swerve_drive"].trackers
    assert (head["head_pan"].min_rel, head["head_pan"].max_rel) == (-300, 300)
    assert (steer["fl_steer"].min_rel, steer["fl_steer"].max_rel) == (-400, 400)
    assert set(head) == {"head_pan", "head_tilt"}
    assert set(steer) == {"fl_steer", "fr_steer"}


def test_capture_stop_keeps_ranges(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, released=("head",))
    assert rig.cal.record_home("head").success
    assert rig.cal.capture_start("head").success
    rig.cal.observe(readings(head_pan=ENCODER[21] - 150, head_tilt=ENCODER[22] + 60))
    result = rig.cal.capture_stop("head")
    assert result.success, result.message
    assert result.message.startswith("/head: ")
    rig.cal.observe(readings(head_pan=ENCODER[21] - 900, head_tilt=ENCODER[22] + 900))
    tracker = rig.cal.states["head"].trackers["head_pan"]
    assert tracker.min_rel == -150
    capture = section(rig, "head")["capture"]
    assert capture["running"] is False
    assert capture["joints"]["head_pan"]["min_steps"] == ENCODER[21] - 150
    assert rig.cal.capture_stop("head").success is False


# --- plan -------------------------------------------------------------------------------------------------------------


def test_plan_without_capture_is_home_only(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, released=("head",))
    assert rig.cal.record_home("head").success
    rig.port.writes.clear()
    result = rig.cal.plan("head")
    assert result.success, result.message
    assert "nothing written" in result.message
    assert [w for w in rig.port.writes if w[1] in EEPROM_ADDRESSES] == []
    plan = section(rig, "head")["plan"]
    assert plan["ok"] is True
    assert plan["needs_confirm"] is False
    assert plan["created_at"] == "2026-11-10T12:00:00+00:00"
    pan = plan["joints"]["head_pan"]
    assert pan["offset_change_steps"] == 52
    assert pan["offset_before_steps"] == 10
    assert pan["new_offset_steps"] == 62
    assert pan["range_deg"] is None
    assert pan["limits_after"] == [1000 - 52, 3000 - 52]
    state = rig.cal.states["head"]
    assert state.plan is not None and state.plan.run_id == plan["run_id"]
    assert state.plan_mono == rig.clock.now
    assert state.plan.joints["head_pan"].limits_before == (1000, 3000)


def test_plan_refused_while_capture_running(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, released=("head",))
    assert rig.cal.record_home("head").success
    assert rig.cal.capture_start("head").success
    result = rig.cal.plan("head")
    assert result.success is False
    assert "stop the range capture first" in result.message
    assert section(rig, "head")["plan"] is None


def test_plan_refused_without_home(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, released=("head",))
    result = rig.cal.plan("head")
    assert result.success is False
    assert result.message == "/head: record home first; nothing written"


def test_plan_refused_when_offset_changed_since_home(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, released=("head",))
    assert rig.cal.record_home("head").success
    put_word(rig.port.servos[21], STS_OFS_L, encode_offset(99))
    result = rig.cal.plan("head")
    assert result.success is False
    assert "head_pan" in result.message
    assert "record home again" in result.message


def test_plan_reports_problems_and_fails(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, released=("head",))
    assert rig.cal.record_home("head").success
    assert rig.cal.capture_start("head").success
    rig.cal.observe(readings(head_pan=ENCODER[21] - 40, head_tilt=ENCODER[22] - 300))
    rig.cal.observe(readings(head_pan=ENCODER[21] + 40, head_tilt=ENCODER[22] + 300))
    assert rig.cal.capture_stop("head").success
    result = rig.cal.plan("head")
    assert result.success is False
    assert result.message.startswith("/head: ")
    assert "head_pan" in result.message
    assert "below 20 deg" in result.message
    plan = section(rig, "head")["plan"]
    assert plan["ok"] is False
    assert plan["joints"]["head_pan"]["problems"]
    assert plan["joints"]["head_tilt"]["problems"] == []
    assert plan["joints"]["head_tilt"]["limits_after"] == [TARGET - 300 + 23, TARGET + 300 - 23]


def test_plan_needs_confirm_for_large_change(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, released=("swerve_drive",), max_change_deg=1.0)
    assert rig.cal.record_home("swerve_drive").success
    result = rig.cal.plan("swerve_drive")
    assert result.success, result.message
    assert "confirm" in result.message
    plan = section(rig, "swerve_drive")["plan"]
    assert plan["ok"] is True
    assert plan["needs_confirm"] is True
    assert plan["joints"]["fl_steer"]["exceeds_max_change"] is True


def test_plan_needs_confirm_when_narrower_than_required(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, released=("head",), required_travel_rad={"head_pan": (-1.5708, 1.5708)})
    assert rig.cal.record_home("head").success
    result = rig.cal.plan("head")
    assert result.success, result.message
    plan = section(rig, "head")["plan"]
    assert plan["joints"]["head_pan"]["narrower_than_required"] is True
    assert plan["needs_confirm"] is True


# --- status -----------------------------------------------------------------------------------------------------------


def test_status_none_without_block(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, head_block=False)
    assert rig.cal.status("head") == {"servo_calibration": None}
    assert rig.cal.status("unknown") == {"servo_calibration": None}
    result = rig.cal.record_home("head")
    assert result.success is False
    assert "servo calibration not configured" in result.message


def test_status_before_any_reading(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    status = section(rig, "head")
    assert status["home"] is None
    assert status["capture"] is None
    assert status["plan"] is None
    assert status["last_result"] is None
    assert status["config"]["home_reference_rad"] == {"head_pan": 0.0, "head_tilt": 0.0}
    assert status["config"]["max_change_deg"] == 15.0
    assert status["config"]["capture_status_interval_s"] == 0.25


def test_status_capture_joint_fields_none_before_samples(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, released=("head",))
    assert rig.cal.record_home("head").success
    assert rig.cal.capture_start("head").success
    capture = section(rig, "head")["capture"]
    assert capture["samples"] == 0
    assert capture["joints"]["head_pan"] == {
        "present_steps": None,
        "min_steps": None,
        "max_steps": None,
        "range_deg": None,
    }


def test_status_capture_snapshot_throttled(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, released=("head",))
    assert rig.cal.record_home("head").success
    assert rig.cal.capture_start("head").success
    rig.cal.observe(readings(head_pan=ENCODER[21] + 10, head_tilt=ENCODER[22]))
    first = section(rig, "head")["capture"]
    rig.clock.now += 0.1
    rig.cal.observe(readings(head_pan=ENCODER[21] + 200, head_tilt=ENCODER[22]))
    rig.clock.now += 0.1
    second = section(rig, "head")["capture"]
    assert second == first
    assert second["joints"]["head_pan"]["max_steps"] == ENCODER[21] + 10
    rig.clock.now += 0.05
    third = section(rig, "head")["capture"]
    assert third["joints"]["head_pan"]["max_steps"] == ENCODER[21] + 200
    assert third["samples"] == 2


def test_service_call_marks_snapshot_dirty(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, released=("head",))
    assert rig.cal.record_home("head").success
    assert rig.cal.capture_start("head").success
    assert section(rig, "head")["capture"]["running"] is True
    assert rig.cal.capture_stop("head").success
    assert section(rig, "head")["capture"]["running"] is False


# --- carry-in fixes (Task 5 review) -----------------------------------------------------------------------------------


def test_record_home_refusals_say_nothing_written(tmp_path: Path) -> None:
    """The still check and the offset read refusals of record home carry the "nothing written" suffix."""
    rig = make_rig(tmp_path, released=("head",))
    rig.port.jitter[21] = [0, 4, 9, 12]
    moving = rig.cal.record_home("head")
    assert moving.success is False
    assert "joints not still: head_pan moved" in moving.message
    assert "arm" not in moving.message
    assert moving.message.endswith("; nothing written")
    rig.port.drop_offset_reads_after_writes[22] = 0
    unread = rig.cal.record_home("head")
    assert unread.success is False
    assert unread.message == "/head: offset read failed: head_tilt; nothing written"


def test_plan_flags_joint_without_range_samples(tmp_path: Path) -> None:
    """A capture ran but one joint got no samples: that joint gets a problem instead of a home-only plan."""
    rig = make_rig(tmp_path, released=("head",))
    assert rig.cal.record_home("head").success
    assert rig.cal.capture_start("head").success
    rig.cal.observe(readings(head_pan=ENCODER[21] - 300))
    rig.cal.observe(readings(head_pan=ENCODER[21] + 300))
    assert rig.cal.capture_stop("head").success
    result = rig.cal.plan("head")
    assert result.success is False
    plan = section(rig, "head")["plan"]
    assert plan["joints"]["head_tilt"]["problems"] == ["no range samples; record ranges again"]
    assert plan["joints"]["head_pan"]["problems"] == []
    assert "head_tilt: no range samples; record ranges again" in result.message


def test_observe_reanchors_to_home_after_pause(tmp_path: Path) -> None:
    """After a pause the first fold is relative to home, not the short way from the pre-pause sample."""
    rig = make_rig(tmp_path, released=("head",))
    assert rig.cal.record_home("head").success
    assert rig.cal.capture_start("head").success
    home = ENCODER[21]
    rig.cal.observe(readings(head_pan=(home + 700) % 4096, head_tilt=ENCODER[22]))
    rig.cal.observe(readings(head_pan=(home + 1500) % 4096, head_tilt=ENCODER[22]))
    assert rig.torque.enable("head").success
    rig.cal.observe(readings(head_pan=(home - 1000) % 4096, head_tilt=ENCODER[22]))
    assert rig.torque.release("head").success
    rig.cal.observe(readings(head_pan=(home - 1000) % 4096, head_tilt=ENCODER[22]))
    tracker = rig.cal.states["head"].trackers["head_pan"]
    assert tracker.rel == -1000
    assert (tracker.min_rel, tracker.max_rel) == (-1000, 1500)
    rig.cal.observe(readings(head_pan=(home - 1100) % 4096, head_tilt=ENCODER[22]))
    assert tracker.rel == -1100


# --- store ------------------------------------------------------------------------------------------------------------


def planned_rig(tmp_path: Path, namespace: str = "head", **kwargs: Any) -> Rig:
    """Rig with the group released, home recorded and a home-only plan; write log cleared."""
    rig = make_rig(tmp_path, released=(namespace,), **kwargs)
    assert rig.cal.record_home(namespace).success
    assert rig.cal.plan(namespace).message
    rig.port.writes.clear()
    rig.port.events.clear()
    return rig


def eeprom_writes(rig: Rig) -> list[tuple[int, int]]:
    """(servo_id, address) of every offset, angle limit or lock write."""
    return [w for w in rig.port.writes if w[1] in EEPROM_ADDRESSES]


def offset_of(rig: Rig, servo_id: int) -> int:
    """Offset register of a servo on the rig bus (steps)."""
    return decode_offset(word(rig.port.servos[servo_id], STS_OFS_L))


def limits_of(rig: Rig, servo_id: int) -> tuple[int, int]:
    """Angle limit registers of a servo on the rig bus."""
    table = rig.port.servos[servo_id]
    return word(table, STS_MIN_ANGLE_LIMIT_L), word(table, STS_MAX_ANGLE_LIMIT_L)


def present_of(rig: Rig, servo_id: int) -> int:
    """present_position of a servo on the rig bus."""
    return word(rig.port.servos[servo_id], STS_PRESENT_POSITION_L)


def assert_head_untouched(rig: Rig) -> None:
    """No EEPROM write was sent and the head registers are as built."""
    assert eeprom_writes(rig) == []
    assert (offset_of(rig, 21), offset_of(rig, 22)) == (10, -5)
    assert (limits_of(rig, 21), limits_of(rig, 22)) == (LIMITS[21], LIMITS[22])


def test_store_refused_without_plan(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, released=("head",))
    assert rig.cal.record_home("head").success
    rig.port.writes.clear()
    result = rig.cal.store("head")
    assert result.success is False
    assert result.message == "/head: plan first; nothing written"
    assert_head_untouched(rig)


def test_store_refused_when_plan_too_old(tmp_path: Path) -> None:
    rig = planned_rig(tmp_path)
    rig.clock.now += 301.0
    result = rig.cal.store("head")
    assert result.success is False
    assert "plan is 301 s old (max 300 s)" in result.message
    assert result.message.endswith("nothing written")
    assert_head_untouched(rig)
    assert section(rig, "head")["plan"] is None


def test_store_refused_when_torque_on(tmp_path: Path) -> None:
    rig = planned_rig(tmp_path)
    rig.port.servos[22][STS_TORQUE_ENABLE] = 1
    on_bus = rig.cal.store("head")
    assert on_bus.success is False
    assert on_bus.message.startswith("/head: torque not off: head_tilt")
    assert_head_untouched(rig)
    rig.port.servos[22][STS_TORQUE_ENABLE] = 0
    assert rig.torque.enable("head").success
    rig.port.writes.clear()
    enabled = rig.cal.store("head")
    assert enabled.success is False
    assert "release torque first" in enabled.message
    assert_head_untouched(rig)
    assert section(rig, "head")["plan"] is not None


def test_store_refused_when_registers_changed_since_plan(tmp_path: Path) -> None:
    rig = planned_rig(tmp_path)
    put_word(rig.port.servos[22], STS_MAX_ANGLE_LIMIT_L, 2700)
    result = rig.cal.store("head")
    assert result.success is False
    assert "changed since the plan" in result.message
    assert "head_tilt" in result.message
    assert "head_pan" not in result.message
    assert result.message.endswith("nothing written")
    assert eeprom_writes(rig) == []
    assert section(rig, "head")["plan"] is None


def test_store_refused_when_joints_not_still(tmp_path: Path) -> None:
    rig = planned_rig(tmp_path)
    rig.port.jitter[22] = [0, 5, 9]
    result = rig.cal.store("head")
    assert result.success is False
    assert "joints not still: head_tilt moved" in result.message
    assert result.message.endswith("nothing written")
    assert_head_untouched(rig)


def test_store_refused_with_problems(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, released=("head",))
    assert rig.cal.record_home("head").success
    assert rig.cal.capture_start("head").success
    rig.cal.observe(readings(head_pan=ENCODER[21] - 40, head_tilt=ENCODER[22] - 300))
    rig.cal.observe(readings(head_pan=ENCODER[21] + 40, head_tilt=ENCODER[22] + 300))
    assert rig.cal.capture_stop("head").success
    assert rig.cal.plan("head").success is False
    rig.port.writes.clear()
    for confirmed in (False, True):
        result = rig.cal.store("head", confirmed=confirmed)
        assert result.success is False
        assert "cannot store" in result.message
        assert "head_pan" in result.message
        assert "below 20 deg" in result.message
        assert_head_untouched(rig)


def test_store_refused_for_large_change_without_confirm(tmp_path: Path) -> None:
    rig = planned_rig(tmp_path, max_change_deg=1.0)
    result = rig.cal.store("head")
    assert result.success is False
    assert "head_pan (large change +4.6 deg, above 1 deg)" in result.message
    assert "head_tilt (large change -3.5 deg, above 1 deg)" in result.message
    assert "store_confirmed" in result.message
    assert result.message.endswith("nothing written")
    assert_head_untouched(rig)
    assert section(rig, "head")["plan"] is not None


def test_store_refused_when_narrower_than_required_without_confirm(tmp_path: Path) -> None:
    rig = planned_rig(tmp_path, required_travel_rad={"head_pan": (-1.5708, 1.5708)})
    result = rig.cal.store("head")
    assert result.success is False
    assert "head_pan (limits narrower than the required travel)" in result.message
    assert "head_tilt" not in result.message
    assert_head_untouched(rig)
    confirmed = rig.cal.store("head", confirmed=True)
    assert confirmed.success, confirmed.message
    assert limits_of(rig, 21) == (948, 2948)


def test_store_confirmed_applies_large_change(tmp_path: Path) -> None:
    rig = planned_rig(tmp_path, max_change_deg=1.0)
    result = rig.cal.store("head", confirmed=True)
    assert result.success, result.message
    assert (offset_of(rig, 21), offset_of(rig, 22)) == (62, -45)
    assert rig.cal.states["head"].last_result["action"] == "store_confirmed"


def test_store_head_writes_offsets_and_limits(tmp_path: Path) -> None:
    rig = planned_rig(tmp_path)
    plan = rig.cal.states["head"].plan
    assert plan is not None
    result = rig.cal.store("head")
    assert result.success, result.message
    assert result.message.startswith("/head: calibration stored for head_pan, head_tilt")
    assert "torque stays off" in result.message
    for joint in HEAD:
        planned = plan.joints[joint.name]
        assert offset_of(rig, joint.id) == planned.new_offset_steps
        assert limits_of(rig, joint.id) == planned.limits_after
        assert present_of(rig, joint.id) == TARGET
    assert limits_of(rig, 21) == (948, 2948)
    assert limits_of(rig, 22) == (1240, 2840)
    state = rig.cal.states["head"]
    assert state.plan is None
    assert state.home_steps == {}
    assert state.trackers == {}
    status = section(rig, "head")
    assert status["home"] is None
    assert status["plan"] is None
    last = status["last_result"]
    assert last["success"] is True
    assert last["action"] == "store"
    assert last["plan_run_id"] == plan.run_id
    assert last["failure"] is None
    assert last["not_attempted"] == []
    pan = last["joints"]["head_pan"]
    assert pan["applied"] is True
    assert pan["written"] is True
    assert pan["present_before_steps"] == TARGET + 52
    assert pan["present_after_steps"] == TARGET
    assert pan["offset_after_steps"] == 62
    assert pan["limits_after"] == [948, 2948]
    assert pan["rolled_back"] is None
    json.dumps(last)


def test_store_verifies_the_predicted_reading_away_from_home(tmp_path: Path) -> None:
    """The user moved head_pan 300 steps away from home after the plan: verify against p0 - change."""
    rig = planned_rig(tmp_path)
    rig.port.encoder[21] += 300
    rig.port.refresh_present(21)
    result = rig.cal.store("head")
    assert result.success, result.message
    assert offset_of(rig, 21) == 62
    assert present_of(rig, 21) == TARGET + 300
    pan = rig.cal.states["head"].last_result["joints"]["head_pan"]
    assert pan["present_before_steps"] == TARGET + 352
    assert pan["present_after_steps"] == TARGET + 300


def test_store_skips_unchanged_joint(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, released=("head",))
    rig.port.encoder[22] = TARGET + OFFSETS[22]
    rig.port.refresh_present(22)
    assert rig.cal.record_home("head").success
    assert rig.cal.plan("head").success
    rig.port.writes.clear()
    result = rig.cal.store("head")
    assert result.success, result.message
    assert "unchanged: head_tilt" in result.message
    assert [w for w in rig.port.writes if w[0] == 22] == []
    assert (offset_of(rig, 22), limits_of(rig, 22)) == (-5, LIMITS[22])
    tilt = rig.cal.states["head"].last_result["joints"]["head_tilt"]
    assert tilt["written"] is False
    assert tilt["applied"] is True
    assert tilt["reason"] is None


def test_store_rolls_back_on_verify_failure(tmp_path: Path) -> None:
    """head_pan drops the offset effect (acknowledged, not stored): old offset and limits kept, head_tilt not tried."""
    rig = planned_rig(tmp_path)
    rig.port.stuck_offset.add(21)
    result = rig.cal.store("head")
    assert result.success is False
    assert "failed: head_pan (not verified" in result.message
    assert "not attempted: head_tilt" in result.message
    assert (offset_of(rig, 21), limits_of(rig, 21)) == (10, LIMITS[21])
    assert (offset_of(rig, 22), limits_of(rig, 22)) == (-5, LIMITS[22])
    assert [w for w in eeprom_writes(rig) if w[0] == 22] == []
    last = rig.cal.states["head"].last_result
    assert last["success"] is False
    assert last["failure"][0] == "head_pan"
    assert last["not_attempted"] == ["head_tilt"]
    pan = last["joints"]["head_pan"]
    assert pan["written"] is True
    assert pan["applied"] is False
    assert pan["rolled_back"] is True
    assert "head_tilt" not in last["joints"]
    assert rig.cal.states["head"].plan is None
    assert rig.cal.states["head"].home_steps == home_of("head")
    assert rig.cal.command_limits[21] == (0, 4095)


def test_store_rolls_back_on_limit_readback_mismatch(tmp_path: Path) -> None:
    """head_pan's max limit lands 7 steps off: the old offset and both old limits are rewritten and read back."""
    rig = planned_rig(tmp_path)
    rig.port.max_limit_skew[21] = 7
    result = rig.cal.store("head")
    assert result.success is False
    assert "failed: head_pan" in result.message
    assert "2955" in result.message
    assert (offset_of(rig, 21), limits_of(rig, 21)) == (10, LIMITS[21])
    assert present_of(rig, 21) == TARGET + 52
    assert (offset_of(rig, 22), limits_of(rig, 22)) == (-5, LIMITS[22])
    last = rig.cal.states["head"].last_result
    assert last["not_attempted"] == ["head_tilt"]
    assert last["joints"]["head_pan"]["rolled_back"] is True
    rollback = [e for e in rig.calib_events() if e["step"] == "rollback"]
    assert len(rollback) == 1
    assert rollback[0]["outcome"] == "ok"
    assert rig.cal.command_limits[21] == (0, 4095)


def test_store_refreshes_command_limits_but_keeps_overrides(tmp_path: Path) -> None:
    rig = planned_rig(tmp_path, overrides={22: (1300, 2700)})
    assert rig.cal.store("head").success
    assert rig.cal.command_limits[21] == (948, 2948)
    assert rig.cal.command_limits[22] == (1300, 2700)
    assert rig.torque.cycle.command_limits[21] == (948, 2948)


def test_store_reports_wider_override(tmp_path: Path) -> None:
    rig = planned_rig(tmp_path, overrides={22: (1000, 3000)})
    result = rig.cal.store("head")
    assert result.success, result.message
    note = "head_tilt: override 1000..3000 still clamps commands; the servo now stops at 1240..2840"
    assert note in result.message
    assert rig.cal.states["head"].last_result["override_notes"] == [note]
    latest = json.loads((tmp_path / "servo_calibration-head-latest.json").read_text())
    assert latest["override_notes"] == [note]


def test_store_keeps_torque_off(tmp_path: Path) -> None:
    rig = planned_rig(tmp_path)
    assert rig.cal.store("head").success
    assert [w for w in rig.port.writes if w[1] == STS_TORQUE_ENABLE] == []
    for joint in HEAD:
        assert rig.port.servos[joint.id][STS_TORQUE_ENABLE] == 0
        assert joint.id in rig.torque.released_ids
    assert rig.cal.not_released(HEAD) == []


def test_store_writes_record_and_latest(tmp_path: Path) -> None:
    rig = planned_rig(tmp_path)
    plan_run_id = rig.cal.states["head"].plan.run_id
    assert rig.cal.store("head").success
    last = rig.cal.states["head"].last_result
    audit = tmp_path / f"servo_calibration-20261110T120000Z-{last['run_id']}.json"
    assert last["record_path"] == str(audit)
    record = json.loads(audit.read_text())
    latest = json.loads((tmp_path / "servo_calibration-head-latest.json").read_text())
    assert record == latest
    assert record["success"] is True
    assert record["group"] == "head"
    assert record["plan_run_id"] == plan_run_id
    assert record["confirmed"] is False
    assert record["changed_at"] == "2026-11-10T12:00:00+00:00"
    assert set(record["joints"]) == {"head_pan", "head_tilt"}
    assert record["joints"]["head_pan"]["plan"]["new_offset_steps"] == 62
    assert record["joints"]["head_pan"]["offset_after_steps"] == 62
    assert record["home_rad"]["head_pan"] == 0.0
    steps = [e["step"] for e in rig.calib_events() if e["flow"] == "servo_calibration"]
    assert steps[-1] == "store"
    assert "record" in steps


def test_failed_store_writes_audit_record_only(tmp_path: Path) -> None:
    rig = planned_rig(tmp_path)
    rig.port.stuck_offset.add(21)
    assert rig.cal.store("head").success is False
    last = rig.cal.states["head"].last_result
    record = json.loads(Path(last["record_path"]).read_text())
    assert record["success"] is False
    assert record["failure"][0] == "head_pan"
    assert record["not_attempted"] == ["head_tilt"]
    assert not (tmp_path / "servo_calibration-head-latest.json").exists()


def test_store_never_sends_define_middle(tmp_path: Path) -> None:
    rig = planned_rig(tmp_path)
    assert rig.cal.store("head").success
    assert not any(event == "define_middle" for _, event, _ in rig.port.events)


# --- arm bookkeeping (spec 3.6) ---------------------------------------------------------------------------------------


def arm_extra(
    tmp_path: Path,
    rad: float = 0.0,
    tilt_rad: float = 0.0,
    write_file: bool = True,
    seed: dict[str, float] | None = None,
) -> dict[str, Any]:
    """Head group fields that make it carry the arm bookkeeping: a stale marker and a zero_restore block.

    rad / tilt_rad are the rest reference file's readings (no file with write_file False); seed is the
    seed_reference_rad (default 0.0 for both joints).
    """
    path = tmp_path / "zero" / "rest_reference.json"
    path.parent.mkdir()
    if write_file:
        reference = {
            "version": 1,
            "group": "head",
            "saved_at": "2026-11-01T00:00:00+00:00",
            "joints": {"head_pan": rad, "head_tilt": tilt_rad},
        }
        path.write_text(json.dumps(reference))
    return {
        "neutral_stale_marker": str(tmp_path / "zero" / "calibration_stale.json"),
        "zero_restore": ZeroRestoreConfig(
            reference_file=str(path),
            record_dir=str(tmp_path),
            seed_reference_rad=seed or {"head_pan": 0.0, "head_tilt": 0.0},
        ),
    }


def test_arm_store_marks_joint_offsets_and_home_stale(tmp_path: Path) -> None:
    extra = arm_extra(tmp_path)
    rig = planned_rig(tmp_path, head_extra=extra)
    assert rig.cal.store("head").success
    marker = json.loads(Path(extra["neutral_stale_marker"]).read_text())
    assert marker["stale"] == ["joint_offsets", "home"]
    assert marker["joints"] == {
        "head_pan": {"before_steps": 10, "after_steps": 62},
        "head_tilt": {"before_steps": -5, "after_steps": -45},
    }
    zero_change = json.loads((Path(extra["neutral_stale_marker"]).parent / "zero_changed.json").read_text())
    assert zero_change["changed_at"] == "2026-11-10T12:00:00+00:00"


def test_marker_saved_at_equals_record_changed_at(tmp_path: Path) -> None:
    extra = arm_extra(tmp_path)
    rig = planned_rig(tmp_path, head_extra=extra)
    assert rig.cal.store("head").success
    marker = json.loads(Path(extra["neutral_stale_marker"]).read_text())
    record = json.loads((tmp_path / "servo_calibration-head-latest.json").read_text())
    assert marker["saved_at"] == record["changed_at"]
    assert record["bookkeeping_errors"] == []


def test_store_converts_rest_reference_into_new_frame(tmp_path: Path) -> None:
    extra = arm_extra(tmp_path, rad=0.1)
    reference_path = Path(extra["zero_restore"].reference_file)
    rig = planned_rig(tmp_path, head_extra=extra)
    rig.zero.state(rig.torque.groups["head"])
    rig.zero.states["head"].plan = object()
    assert rig.cal.store("head").success
    record = json.loads((tmp_path / "servo_calibration-head-latest.json").read_text())
    reference = json.loads(reference_path.read_text())
    for name, rad in (("head_pan", 0.1), ("head_tilt", 0.0)):
        change = record["joints"][name]["plan"]["offset_change_steps"]
        assert change != 0
        expected = (position_to_raw_steps(rad, False) - change) % 4096
        assert reference["steps"][name] == expected
        assert reference["joints"][name] == steps_to_radians(expected, False)
    assert reference["saved_at"] == "2026-11-10T12:00:00+00:00"
    assert rig.zero.states["head"].plan is None


def test_arm_store_names_servo_calibration_in_the_zero_change_record(tmp_path: Path) -> None:
    extra = arm_extra(tmp_path)
    rig = planned_rig(tmp_path, head_extra=extra)
    assert rig.cal.store("head").success
    zero_change = json.loads((Path(extra["neutral_stale_marker"]).parent / "zero_changed.json").read_text())
    assert zero_change["source"] == "servo_calibration"


def test_store_writes_the_rest_reference_from_the_seed_when_absent(tmp_path: Path) -> None:
    extra = arm_extra(tmp_path, write_file=False, seed={"head_pan": 0.1, "head_tilt": 0.0})
    reference_path = Path(extra["zero_restore"].reference_file)
    rig = planned_rig(tmp_path, head_extra=extra)
    assert rig.cal.store("head").success
    record = json.loads((tmp_path / "servo_calibration-head-latest.json").read_text())
    reference = json.loads(reference_path.read_text())
    assert (reference["version"], reference["group"]) == (1, "head")
    assert reference["saved_at"] == "2026-11-10T12:00:00+00:00"
    for name, rad in (("head_pan", 0.1), ("head_tilt", 0.0)):
        change = record["joints"][name]["plan"]["offset_change_steps"]
        expected = (position_to_raw_steps(rad, False) - change) % 4096
        assert reference["steps"][name] == expected
        assert reference["joints"][name] == steps_to_radians(expected, False)


@pytest.mark.parametrize("with_file", [True, False], ids=["reference_file", "seed_only"])
def test_zero_restore_plans_no_change_after_an_arm_store(tmp_path: Path, with_file: bool) -> None:
    """The rest reference (file, or seed without one) equals the resting readings before the store; after the store
    the zero restore plan of the resting group changes no offset (it would not undo the calibration)."""
    pan, tilt = steps_to_radians(ENCODER[21], False), steps_to_radians(ENCODER[22], False)
    extra = arm_extra(tmp_path, pan, tilt, write_file=with_file, seed={"head_pan": pan, "head_tilt": tilt})
    rig = planned_rig(tmp_path, head_extra=extra)
    group = rig.torque.groups["head"]
    before, problems = rig.zero.plan_joints(
        group,
        load_reference(extra["zero_restore"], ["head_pan", "head_tilt"])[0],
        {
            "head_pan": present_of(rig, 21),
            "head_tilt": present_of(rig, 22),
        },
    )
    assert problems == [] and all(abs(j.offset_change_steps) <= 1 for j in before.values())
    assert rig.cal.store("head").success
    assert (offset_of(rig, 21), offset_of(rig, 22)) == (62, -45)
    reference, error = load_reference(extra["zero_restore"], ["head_pan", "head_tilt"])
    assert error is None and reference["source"] == "file"
    presents = {"head_pan": present_of(rig, 21), "head_tilt": present_of(rig, 22)}
    planned, problems = rig.zero.plan_joints(group, reference, presents)
    assert problems == []
    assert {name: abs(j.offset_change_steps) <= 1 for name, j in planned.items()} == {
        "head_pan": True,
        "head_tilt": True,
    }


def test_zero_restore_refused_after_an_arm_store(tmp_path: Path) -> None:
    extra = arm_extra(tmp_path)
    rig = planned_rig(tmp_path, head_extra=extra)
    assert rig.cal.store("head").success
    marker_path = Path(extra["neutral_stale_marker"])
    marker_before = marker_path.read_text()
    for result in (rig.zero.dry_run("head"), rig.zero.apply("head", confirmed=True)):
        assert result.success is False
        assert "set by a servo calibration at 2026-11-10T12:00:00+00:00" in result.message
        assert "redo the arm-to-camera calibration" in result.message
    assert marker_path.read_text() == marker_before
    assert json.loads(marker_before)["stale"] == ["joint_offsets", "home"]
    assert (offset_of(rig, 21), offset_of(rig, 22)) == (62, -45)


def test_store_reports_rest_reference_failure_without_undoing_the_writes(tmp_path: Path) -> None:
    extra = arm_extra(tmp_path)
    Path(extra["zero_restore"].reference_file).write_text("not json")
    rig = planned_rig(tmp_path, head_extra=extra)
    result = rig.cal.store("head")
    assert result.success is False
    assert "rest reference" in result.message
    assert offset_of(rig, 21) == 62
    record = json.loads(next(tmp_path.glob("servo_calibration-2026*.json")).read_text())
    assert record["bookkeeping_errors"] and "rest reference" in record["bookkeeping_errors"][0]


def test_failed_arm_store_writes_the_zero_change_record(tmp_path: Path) -> None:
    extra = arm_extra(tmp_path)
    rig = planned_rig(tmp_path, head_extra=extra)
    rig.port.stuck_offset.add(21)
    assert rig.cal.store("head").success is False
    directory = Path(extra["neutral_stale_marker"]).parent
    assert json.loads((directory / "zero_changed.json").read_text())["changed_at"] == "2026-11-10T12:00:00+00:00"
    assert not (directory / "calibration_stale.json").exists()


def test_store_with_unknown_offset_marks_the_joint_stale(tmp_path: Path) -> None:
    extra = arm_extra(tmp_path)
    rig = planned_rig(tmp_path, head_extra=extra)
    rig.port.drop_offset_reads_after_writes[21] = 1
    result = rig.cal.store("head")
    assert result.success is False
    marker = json.loads(Path(extra["neutral_stale_marker"]).read_text())
    assert list(marker["joints"]) == ["head_pan"]
    assert marker["joints"]["head_pan"]["before_steps"] == 10
    assert marker["stale"] == ["joint_offsets", "home"]


def test_store_with_bookkeeping_error_writes_no_latest_record(tmp_path: Path) -> None:
    extra = arm_extra(tmp_path)
    Path(extra["zero_restore"].reference_file).write_text("not json")
    rig = planned_rig(tmp_path, head_extra=extra)
    result = rig.cal.store("head")
    assert result.success is False
    last = rig.cal.states["head"].last_result
    assert last["bookkeeping_errors"]
    assert json.loads(Path(last["record_path"]).read_text())["success"] is False
    assert not (tmp_path / "servo_calibration-head-latest.json").exists()


def test_store_still_check_refusal_hints_to_return_home(tmp_path: Path) -> None:
    rig = planned_rig(tmp_path)
    rig.port.jitter[21] = [0, 40, -40, 40]
    result = rig.cal.store("head")
    assert result.success is False
    assert "return the joints to home and let go" in result.message
    assert_head_untouched(rig)


def test_store_stops_when_torque_comes_on_after_the_first_joint(tmp_path: Path) -> None:
    rig = planned_rig(tmp_path)
    rig.port.torque_on_after_offset[21] = 22
    result = rig.cal.store("head")
    assert result.success is False
    assert "head_tilt (torque not off before the write" in result.message
    assert offset_of(rig, 21) == 62
    assert offset_of(rig, 22) == -5
    assert not [w for w in eeprom_writes(rig) if w[0] == 22]
    assert rig.cal.states["head"].last_result["joints"]["head_pan"]["applied"] is True


def test_head_store_writes_no_marker(tmp_path: Path) -> None:
    rig = planned_rig(tmp_path)
    assert rig.cal.store("head").success
    assert sorted(p.name for p in tmp_path.iterdir() if p.is_file()) == sorted(
        [
            f"servo_calibration-20261110T120000Z-{rig.cal.states['head'].last_result['run_id']}.json",
            "servo_calibration-head-latest.json",
        ]
    )


@pytest.mark.skipif(os.geteuid() == 0, reason="root ignores directory permissions")
def test_store_refused_when_record_dir_not_writable(tmp_path: Path) -> None:
    locked = tmp_path / "locked"
    locked.mkdir()
    rig = planned_rig(tmp_path, record_dir=str(locked))
    locked.chmod(0o500)
    try:
        result = rig.cal.store("head")
    finally:
        locked.chmod(0o700)
    assert result.success is False
    assert "not writable" in result.message
    assert result.message.endswith("nothing written")
    assert_head_untouched(rig)
    assert section(rig, "head")["plan"] is not None
