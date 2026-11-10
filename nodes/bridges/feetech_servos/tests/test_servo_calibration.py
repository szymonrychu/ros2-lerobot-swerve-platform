"""ServoCalibrationControl (record home, range capture, plan, status) on an emulated STS bus.

The rig holds the head (head_pan, head_tilt) and the base steering (fl_steer, fr_steer plus two drive wheels) on one
bus, both with a servo_calibration block; every servo reads present = (encoder - offset) mod 4096 (STS3215).
"""

import json
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from types import SimpleNamespace
from typing import Any

from feetech_mapping.command_mapping import position_to_raw_steps
from st3215.values import INST_WRITE

from feetech_servos.bridge_cycle import BridgeCycle
from feetech_servos.calib_event import CalibEventLog
from feetech_servos.config import JointEntry, JointGroup, ServoCalibrationConfig
from feetech_servos.registers import (
    STS_LOCK,
    STS_MAX_ANGLE_LIMIT_L,
    STS_MIN_ANGLE_LIMIT_L,
    STS_OFS_L,
    STS_TORQUE_ENABLE,
    get_register_entry_by_name,
)
from feetech_servos.servo_calibration import ServoCalibrationControl
from feetech_servos.servo_offsets import encode_offset, steps_to_degrees
from feetech_servos.torque_control import TorqueControl

from .offset_bus import Clock, OffsetPort
from .sts_bus import StsPacket, put_word, register_table
from .test_neutral import FakeServo

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
    clock: Clock
    lines: list[str]

    def calib_events(self) -> list[dict[str, Any]]:
        """Parsed calib_event lines."""
        return [json.loads(line.split(" ", 1)[1]) for line in self.lines if line.startswith("calib_event ")]


class WriteLog(OffsetPort):
    """OffsetPort recording every write instruction's start address."""

    writes: list[tuple[int, int]]

    def apply(self, packet: StsPacket) -> None:
        """Record (servo_id, address) of writes, then apply."""
        if packet.instruction == INST_WRITE and packet.params:
            self.writes.append((packet.servo_id, packet.params[0]))
        super().apply(packet)


def make_config(reference: dict[str, float], tmp_path: Path, **changes: Any) -> ServoCalibrationConfig:
    """servo_calibration block with the given home references."""
    return ServoCalibrationConfig(home_reference_rad=reference, record_dir=str(tmp_path), **changes)


def make_rig(tmp_path: Path, head_block: bool = True, released: tuple[str, ...] = (), **changes: Any) -> Rig:
    """Build head + swerve_drive (torque on), optionally released, with their servo_calibration blocks."""
    head = JointGroup(
        "head",
        list(HEAD),
        servo_calibration=make_config({"head_pan": 0.0, "head_tilt": 0.0}, tmp_path, **changes) if head_block else None,
    )
    swerve = JointGroup(
        "swerve_drive",
        STEER + DRIVE,
        servo_calibration=make_config({"fl_steer": 0.0, "fr_steer": 0.0}, tmp_path, **changes),
    )
    port = WriteLog(servos={})
    port.writes = []
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
    command_limits = {j.id: (0, 4095) for j in HEAD + STEER}
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
    cal = ServoCalibrationControl(
        torque,
        command_limits,
        {},
        sleep=lambda _s: None,
        monotonic=clock,
        now=lambda: NOW,
        events=CalibEventLog(lines.append, lines.append, lines.append),
    )
    return Rig(port, torque, cal, clock, lines)


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
