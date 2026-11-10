"""ZeroRestoreControl on an emulated STS bus whose servos read present = (encoder - offset) mod 4096 (STS3215).

The rig seeds the follower arm after the 2026-11 incident: each joint read its rest reference with ORIGINAL_OFFSET
before a neutral save in the wrong pose raised each offset by the logged shift (INCIDENT_SHIFT), so the resting arm
now reads reference - shift. The gripper is excluded (neutral_exclude_joints) and must never be touched.
"""

import importlib
import json
import math
from dataclasses import dataclass, field
from datetime import UTC, datetime
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Any

import pytest
from feetech_mapping.command_mapping import STEPS_PER_REVOLUTION, position_to_raw_steps, steps_to_radians
from st3215.values import INST_READ, INST_WRITE

from feetech_servos import zero_restore as zero_restore_module
from feetech_servos.bridge_cycle import BridgeCycle
from feetech_servos.calib_event import CalibEventLog
from feetech_servos.config import JointEntry, JointGroup, ZeroRestoreConfig
from feetech_servos.neutral import NeutralControl
from feetech_servos.registers import (
    STS_LOCK,
    STS_MAX_ANGLE_LIMIT_L,
    STS_MIN_ANGLE_LIMIT_L,
    STS_OFS_L,
    STS_PRESENT_POSITION_L,
    STS_TORQUE_ENABLE,
    get_register_entry_by_name,
)
from feetech_servos.torque_control import TorqueControl, torque_state_payload
from feetech_servos.zero_record import STALE_ITEMS, ZERO_RECORD_FILE, write_stale_marker
from feetech_servos.zero_restore import (
    OFFSET_MAX_STEPS,
    STILL_SAMPLE_INTERVAL_S,
    ZeroRestoreControl,
    decode_offset,
    encode_offset,
    load_reference,
    offset_delta_steps,
    steps_to_degrees,
    wrap_offset_steps,
)

from .ros_stubs import install_service_stubs
from .sts_bus import StsPacket, put_word, register_table
from .test_neutral import FakeServo, NeutralPort, word
from .test_torque_services import RecordingNode

SEED = {"shoulder_pan": -0.018, "shoulder_lift": -1.542, "elbow_flex": 1.516, "wrist_flex": 0.862, "wrist_roll": -0.072}
ARM = [
    JointEntry("shoulder_pan", 1),
    JointEntry("shoulder_lift", 2),
    JointEntry("elbow_flex", 3),
    JointEntry("wrist_flex", 4),
    JointEntry("wrist_roll", 5),
    JointEntry("gripper", 6, command_min_steps=1900, command_max_steps=3377),
]
ARM_IDS = (1, 2, 3, 4, 5)
GRIPPER_ID = 6
# Logged shifts of the wrong re-zero ("neutral recorded: <joint> +N steps"); the restore's offset change is the inverse.
INCIDENT_SHIFT = {"shoulder_pan": -11, "shoulder_lift": -901, "elbow_flex": 1029, "wrist_flex": -47, "wrist_roll": 14}
ORIGINAL_OFFSET = {"shoulder_pan": 120, "shoulder_lift": -300, "elbow_flex": 450, "wrist_flex": -20, "wrist_roll": 0}
SMALL_SHIFT = {"shoulder_pan": -11, "shoulder_lift": 20, "elbow_flex": -30, "wrist_flex": -47, "wrist_roll": 14}
LIMITS = {1: (500, 3500), 2: (0, 4095), 3: (0, 4095), 4: (0, 4095), 5: (0, 4095), 6: (1900, 3377)}
STALE_COMMAND_LIMITS = (0, 4095)
NOW = datetime(2026, 11, 9, 12, 0, 0, tzinfo=UTC)
RECORD_GLOB = "zero_restore-20261109T120000Z-*.json"
STEP_RAD = 2 * math.pi / STEPS_PER_REVOLUTION


@dataclass
class OffsetPort(NeutralPort):
    """NeutralPort whose servos read present_position = (encoder - offset) mod 4096, like the STS3215.

    Attributes:
        encoder: servo_id -> raw encoder steps of the resting arm (the arm never moves).
        offset_sign: +1 for the STS3215 semantics; -1 models a servo adding the offset (wrong convention).
        stuck_offset: Servos acknowledging offset writes without storing them.
        limit_shift_on_offset: Steps the angle limit registers move by on an offset write.
        jitter: servo_id -> per-read present_position deltas (popped per read; the arm trembling).
        torque_on_after_offset: servo_id -> servo whose torque turns on after that servo's offset write.
        unacked_offset: Servos storing an offset write without sending the status reply.
        drop_offset_reads_after_writes: servo_id -> offset writes after which its offset reads never answer.
        offset_writes_seen: servo_id -> offset writes received so far.
    """

    encoder: dict[int, int] = field(default_factory=dict)
    offset_sign: int = 1
    stuck_offset: set[int] = field(default_factory=set)
    limit_shift_on_offset: int = 0
    jitter: dict[int, list[int]] = field(default_factory=dict)
    torque_on_after_offset: dict[int, int] = field(default_factory=dict)
    unacked_offset: set[int] = field(default_factory=set)
    drop_offset_reads_after_writes: dict[int, int] = field(default_factory=dict)
    offset_writes_seen: dict[int, int] = field(default_factory=dict)

    def refresh_present(self, servo_id: int) -> None:
        """Recompute present_position of one servo from its encoder and offset register."""
        table = self.servos[servo_id]
        offset = decode_offset(word(table, STS_OFS_L))
        put_word(table, STS_PRESENT_POSITION_L, (self.encoder[servo_id] - self.offset_sign * offset) % 4096)

    def apply(self, packet: StsPacket) -> None:
        """Apply the instruction, then model the offset register's effect and the configured faults."""
        sid, params = packet.servo_id, packet.params
        if packet.instruction == INST_READ and params[0] == STS_PRESENT_POSITION_L and self.jitter.get(sid):
            table = self.servos[sid]
            true_value = word(table, STS_PRESENT_POSITION_L)
            put_word(table, STS_PRESENT_POSITION_L, true_value + self.jitter[sid].pop(0))
            super().apply(packet)
            put_word(table, STS_PRESENT_POSITION_L, true_value)
            return
        is_offset_write = packet.instruction == INST_WRITE and params and params[0] == STS_OFS_L
        is_offset_read = packet.instruction == INST_READ and params[0] == STS_OFS_L
        if is_offset_read and self.offset_writes_seen.get(sid, 0) >= self.drop_offset_reads_after_writes.get(
            sid, 10**9
        ):
            return
        if is_offset_write:
            self.offset_writes_seen[sid] = self.offset_writes_seen.get(sid, 0) + 1
        if is_offset_write and sid in self.unacked_offset:
            self.servos[sid][STS_OFS_L : STS_OFS_L + 2] = bytes(params[1:3])
            self.events.append((sid, "offset", decode_offset(word(self.servos[sid], STS_OFS_L))))
            self.refresh_present(sid)
            return
        if is_offset_write and sid in self.stuck_offset:
            self.events.append((sid, "offset", word(bytearray(params[1:3]), 0)))
            self.reply(sid, [])
            return
        super().apply(packet)
        if is_offset_write and sid in self.servos:
            self.events.append((sid, "offset", decode_offset(word(self.servos[sid], STS_OFS_L))))
            self.refresh_present(sid)
            if self.limit_shift_on_offset:
                table = self.servos[sid]
                put_word(table, STS_MIN_ANGLE_LIMIT_L, LIMITS[sid][0] + self.limit_shift_on_offset)
                put_word(table, STS_MAX_ANGLE_LIMIT_L, LIMITS[sid][1] - self.limit_shift_on_offset)
            if sid in self.torque_on_after_offset:
                self.servos[self.torque_on_after_offset[sid]][STS_TORQUE_ENABLE] = 1


@dataclass
class Clock:
    """Injected monotonic clock."""

    now: float = 1000.0

    def __call__(self) -> float:
        return self.now


@dataclass
class Rig:
    """Arm group on an offset-aware bus with TorqueControl and ZeroRestoreControl."""

    port: OffsetPort
    torque: TorqueControl
    zero: ZeroRestoreControl
    command_limits: dict[int, tuple[int, int]]
    last_written: dict[int, dict[str, int]]
    config: ZeroRestoreConfig
    marker: Path
    sleeps: list[float]
    clock: Clock
    lines: list[str]

    def offset(self, sid: int) -> int:
        """Decoded offset register of one servo."""
        return decode_offset(word(self.port.servos[sid], STS_OFS_L))

    def present(self, sid: int) -> int:
        """present_position of one servo."""
        return word(self.port.servos[sid], STS_PRESENT_POSITION_L)

    def offset_writes(self) -> list[int]:
        """Servo IDs of every offset write, in order."""
        return [sid for sid, event, _ in self.port.events if event == "offset"]

    def calib_events(self) -> list[dict[str, Any]]:
        """Parsed calib_event lines."""
        return [json.loads(line.split(" ", 1)[1]) for line in self.lines if line.startswith("calib_event ")]


def make_rig(
    tmp_path: Path,
    shift: dict[str, int] | None = None,
    with_block: bool = True,
    released: bool = True,
    **config_changes: Any,
) -> Rig:
    """Build the post-incident arm at rest (torque on), optionally released, with its zero restore block."""
    shift = shift or INCIDENT_SHIFT
    marker = tmp_path / "arm" / "calibration_stale.json"
    config = ZeroRestoreConfig(
        reference_file=str(tmp_path / "arm" / "rest_reference.json"),
        record_dir=str(tmp_path / "arm"),
        seed_reference_rad=dict(SEED),
        **config_changes,
    )
    group = JointGroup(
        "follower",
        list(ARM),
        neutral_enabled=True,
        neutral_exclude_joints=["gripper"],
        neutral_stale_marker=str(marker),
        zero_restore=config if with_block else None,
    )
    port = OffsetPort(servos={})
    for joint in ARM:
        table = register_table({STS_TORQUE_ENABLE: 1, STS_LOCK: 1})
        put_word(table, STS_MIN_ANGLE_LIMIT_L, LIMITS[joint.id][0])
        put_word(table, STS_MAX_ANGLE_LIMIT_L, LIMITS[joint.id][1])
        port.servos[joint.id] = table
        if joint.name in SEED:
            target = position_to_raw_steps(SEED[joint.name], joint.inverted)
            port.encoder[joint.id] = target + ORIGINAL_OFFSET[joint.name]
            put_word(table, STS_OFS_L, encode_offset(ORIGINAL_OFFSET[joint.name] + shift[joint.name]))
        else:
            port.encoder[joint.id] = 2600
        port.refresh_present(joint.id)
    servo = FakeServo(port)
    last_written = {sid: {"torque_enable": 1, "offset": 1, "goal_position": 2048} for sid in port.servos}
    command_limits = {
        j.id: (j.command_min_steps, j.command_max_steps) if j.command_min_steps is not None else STALE_COMMAND_LIMITS
        for j in ARM
    }
    cycle = BridgeCycle(
        servo=servo,
        goal_entry=get_register_entry_by_name("goal_position"),
        speed_goal_entry=get_register_entry_by_name("goal_speed"),
        velocity_joints=[],
        command_limits=command_limits,
        last_written=last_written,
        velocity_command_timeout_s=0.3,
    )
    torque = TorqueControl(servo, [group], last_written, cycle, verify_sleep_s=0.0)
    torque.joint_torque.update({sid: 1 for sid in port.servos})
    if released:
        assert torque.release("follower").success
    sleeps: list[float] = []
    clock = Clock()
    lines: list[str] = []
    zero = ZeroRestoreControl(
        torque,
        command_limits,
        sleep=sleeps.append,
        monotonic=clock,
        now=lambda: NOW,
        events=CalibEventLog(lines.append, lines.append, lines.append),
    )
    return Rig(port, torque, zero, command_limits, last_written, config, marker, sleeps, clock, lines)


def write_marker(rig: Rig) -> None:
    """Write the stale marker the wrong re-zero left (joint_offsets and home stale)."""
    write_stale_marker(rig.marker, "follower", "2026-11-09T10:00:00+00:00", {"elbow_flex": (3077, 2048)})


def only_record(rig: Rig) -> Path:
    """The single apply record in the record dir."""
    records = list(Path(rig.config.record_dir).glob(RECORD_GLOB))
    assert len(records) == 1
    return records[0]


def plan_of(rig: Rig) -> dict[str, Any]:
    """The published plan of the follower group."""
    plan = rig.zero.status("follower")["zero_restore"]["plan"]
    assert plan is not None
    return plan


# --- sign-magnitude offset encoding (STS3215: magnitude bits 0-10, sign bit 11) --------------------------------------


@pytest.mark.parametrize(
    ("steps", "raw"),
    [(0, 0x000), (1, 0x001), (-1, 0x801), (901, 0x385), (-1029, 0xC05), (2047, 0x7FF), (-2047, 0xFFF)],
)
def test_offset_encoding_round_trip(steps: int, raw: int) -> None:
    assert encode_offset(steps) == raw
    assert decode_offset(raw) == steps


def test_negative_zero_decodes_to_zero() -> None:
    assert decode_offset(0x800) == 0


@pytest.mark.parametrize("steps", [2048, -2048, 4000])
def test_encode_rejects_magnitude_over_2047(steps: int) -> None:
    with pytest.raises(ValueError):
        encode_offset(steps)


@pytest.mark.parametrize("raw", [0x1000, 0xF000])
def test_decode_rejects_bits_above_sign(raw: int) -> None:
    with pytest.raises(ValueError):
        decode_offset(raw)


def test_offset_max_is_2047() -> None:
    assert OFFSET_MAX_STEPS == 2047


def test_offset_delta_is_present_minus_target() -> None:
    assert offset_delta_steps(2949, 2048) == 901
    assert offset_delta_steps(1019, 2048) == -1029


def test_offset_delta_takes_the_short_way_across_the_wrap() -> None:
    assert offset_delta_steps(10, 4090) == 16
    assert offset_delta_steps(4090, 10) == -16


@pytest.mark.parametrize(
    ("steps", "wrapped"),
    [(2469, -1627), (-2242, 1854), (-2601, 1495), (0, 0), (2047, 2047), (-2047, -2047), (2048, -2048), (-2048, -2048)],
)
def test_wrap_offset_steps(steps: int, wrapped: int) -> None:
    assert wrap_offset_steps(steps) == wrapped
    assert (steps - wrapped) % STEPS_PER_REVOLUTION == 0


@pytest.mark.parametrize("steps", [2469, -2242, -2601])
def test_wrapped_user_offsets_encode(steps: int) -> None:
    assert decode_offset(encode_offset(wrap_offset_steps(steps))) == wrap_offset_steps(steps)


def test_wrapped_2048_is_not_encodable() -> None:
    with pytest.raises(ValueError):
        encode_offset(wrap_offset_steps(2048))


def test_steps_to_degrees() -> None:
    assert steps_to_degrees(1024) == 90.0
    assert steps_to_degrees(-901) == pytest.approx(-79.189, abs=1e-3)


# --- reference --------------------------------------------------------------------------------------------------------


def test_reference_is_the_seed_without_a_file(tmp_path: Path) -> None:
    config = ZeroRestoreConfig(str(tmp_path / "r.json"), str(tmp_path), dict(SEED))
    reference, error = load_reference(config, list(SEED))
    assert error is None
    assert reference == {"source": "seed", "saved_at": None, "joints": SEED}


def test_reference_file_wins_over_the_seed(tmp_path: Path) -> None:
    path = tmp_path / "r.json"
    joints = {name: value + 0.01 for name, value in SEED.items()}
    path.write_text(json.dumps({"version": 1, "group": "follower", "saved_at": "T", "joints": joints}))
    reference, error = load_reference(ZeroRestoreConfig(str(path), str(tmp_path), dict(SEED)), list(SEED))
    assert error is None
    assert reference == {"source": "file", "saved_at": "T", "joints": joints}


@pytest.mark.parametrize(
    "content",
    [
        "{not json",
        "[]",
        json.dumps({"joints": {"shoulder_pan": 0.0}}),
        json.dumps({"joints": {**SEED, "elbow_flex": "x"}}),
        json.dumps({"joints": {**SEED, "elbow_flex": None}}),
    ],
)
def test_invalid_reference_file_is_an_error_never_the_seed(tmp_path: Path, content: str) -> None:
    path = tmp_path / "r.json"
    path.write_text(content)
    reference, error = load_reference(ZeroRestoreConfig(str(path), str(tmp_path), dict(SEED)), list(SEED))
    assert reference is None
    assert error is not None and str(path) in error


# --- dry run ----------------------------------------------------------------------------------------------------------


def test_dry_run_refused_without_a_zero_restore_block(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, with_block=False)
    result = rig.zero.dry_run("follower")
    assert result.success is False
    assert "zero restore not configured" in result.message
    assert rig.zero.status("follower") == {"zero_restore": None}


def test_dry_run_refused_when_not_released(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, released=False)
    result = rig.zero.dry_run("follower")
    assert (result.success, "release torque first" in result.message) == (False, True)
    assert plan_or_none(rig) is None


def plan_or_none(rig: Rig) -> dict[str, Any] | None:
    """The published plan, or None."""
    return rig.zero.status("follower")["zero_restore"]["plan"]


def test_dry_run_refused_when_a_servo_reads_torque_on(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    rig.port.servos[3][STS_TORQUE_ENABLE] = 1  # the bridge's cache still says 0
    result = rig.zero.dry_run("follower")
    assert result.success is False
    assert "torque not off: elbow_flex (torque_enable 1)" in result.message
    assert plan_or_none(rig) is None


def test_dry_run_refused_while_the_arm_moves(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    rig.port.jitter[2] = [0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0]
    result = rig.zero.dry_run("follower")
    assert result.success is False
    assert "not still: shoulder_lift moved 6 steps (tolerance 3)" in result.message
    assert plan_or_none(rig) is None


def test_dry_run_samples_the_still_window(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, still_window_s=0.5)
    rig.port.jitter[2] = [0, 3, 0, 3]  # within the tolerance
    assert rig.zero.dry_run("follower").success
    assert rig.sleeps == [STILL_SAMPLE_INTERVAL_S] * 5


def test_dry_run_plans_the_inverse_of_the_incident_shift_and_writes_nothing(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    writes_before = [p for p in rig.port.packets if p.instruction == INST_WRITE]
    result = rig.zero.dry_run("follower")
    assert result.success is True
    assert [p for p in rig.port.packets if p.instruction == INST_WRITE] == writes_before
    plan = plan_of(rig)
    assert list(plan["joints"]) == list(SEED)  # config order, gripper excluded
    for name, joint in plan["joints"].items():
        assert joint["offset_change_steps"] == -INCIDENT_SHIFT[name]
        assert joint["offset_before_steps"] == ORIGINAL_OFFSET[name] + INCIDENT_SHIFT[name]
        assert joint["new_offset_steps"] == ORIGINAL_OFFSET[name]
        assert joint["target_steps"] == position_to_raw_steps(SEED[name])
        assert joint["target_rad"] == pytest.approx(SEED[name], abs=STEP_RAD)
        assert joint["present_rad"] == pytest.approx(steps_to_radians(joint["present_steps"]))
        assert joint["change_deg"] == pytest.approx(steps_to_degrees(INCIDENT_SHIFT[name]), abs=1e-6)
    assert [n for n, j in plan["joints"].items() if j["exceeds_max_change"]] == ["shoulder_lift", "elbow_flex"]
    assert plan["needs_confirm"] is True
    assert plan["reference_source"] == "seed"
    assert plan["created_at"] == NOW.isoformat(timespec="seconds")
    assert plan["max_age_s"] == 60.0
    assert "confirm" in result.message and "shoulder_lift" in result.message
    assert rig.port.servos[GRIPPER_ID][STS_TORQUE_ENABLE] == 0
    assert not [p for p in rig.port.packets if p.servo_id == GRIPPER_ID and p.params[:1] == [STS_OFS_L]]


def test_dry_run_refused_when_torque_cannot_be_read(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    original_apply = rig.port.apply

    def drop_torque_reads(packet: StsPacket) -> None:
        if packet.instruction == INST_READ and packet.params[0] == STS_TORQUE_ENABLE and packet.servo_id == 4:
            return
        original_apply(packet)

    rig.port.apply = drop_torque_reads  # type: ignore[method-assign]
    result = rig.zero.dry_run("follower")
    assert result.success is False
    assert "wrist_flex (torque read failed)" in result.message


def test_inverted_joint_target_follows_the_bridge_mapping(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, shift=SMALL_SHIFT)
    group = rig.torque.groups["follower"]
    group.joints[2] = JointEntry("elbow_flex", 3, inverted=True)
    target = position_to_raw_steps(SEED["elbow_flex"], inverted=True)
    rig.port.encoder[3] = target + 25 + rig.offset(3)
    rig.port.refresh_present(3)
    assert rig.zero.dry_run("follower").success
    joint = plan_of(rig)["joints"]["elbow_flex"]
    assert joint["target_steps"] == target
    assert joint["target_rad"] == pytest.approx(SEED["elbow_flex"], abs=STEP_RAD)
    assert joint["offset_change_steps"] == 25
    assert joint["present_rad"] == pytest.approx(steps_to_radians(target + 25, inverted=True))


def test_dry_run_small_change_needs_no_confirm(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, shift=SMALL_SHIFT)
    result = rig.zero.dry_run("follower")
    assert result.success is True
    assert plan_of(rig)["needs_confirm"] is False


def test_dry_run_wraps_a_new_offset_past_2047(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, shift=SMALL_SHIFT)
    put_word(rig.port.servos[3], STS_OFS_L, encode_offset(2000))
    # the resting servo reads target + 100: the offset rises by 100 to 2100, the same as -1996 modulo 4096
    target = position_to_raw_steps(SEED["elbow_flex"])
    rig.port.encoder[3] = (target + 100 + 2000) % 4096
    rig.port.refresh_present(3)
    result = rig.zero.dry_run("follower")
    assert result.success is True, result.message
    joint = plan_of(rig)["joints"]["elbow_flex"]
    assert joint["offset_before_steps"] == 2000
    assert joint["offset_change_steps"] == 100
    assert joint["new_offset_steps"] == -1996
    assert joint["exceeds_max_change"] is False
    assert rig.zero.apply("follower").success
    assert rig.offset(3) == -1996
    assert rig.present(3) == target  # the wrapped offset gives the same reading


def test_dry_run_refuses_a_joint_whose_wrapped_offset_is_2048(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, shift=SMALL_SHIFT)
    put_word(rig.port.servos[3], STS_OFS_L, encode_offset(2000))
    # the offset must rise by 48 to 2048 (== -2048): the sign-magnitude register cannot hold it
    target = position_to_raw_steps(SEED["elbow_flex"])
    rig.port.encoder[3] = (target + 48 + 2000) % 4096
    rig.port.refresh_present(3)
    result = rig.zero.dry_run("follower")
    assert result.success is False
    assert "elbow_flex: new offset +-2048 cannot be encoded (max +-2047)" in result.message
    assert "nothing written" in result.message


def test_dry_run_uses_the_reference_file_when_present(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    joints = {name: steps_to_radians(rig.present(sid)) for name, sid in zip(SEED, ARM_IDS, strict=True)}
    Path(rig.config.reference_file).parent.mkdir(parents=True, exist_ok=True)
    Path(rig.config.reference_file).write_text(json.dumps({"version": 1, "saved_at": "T", "joints": joints}))
    assert rig.zero.dry_run("follower").success
    plan = plan_of(rig)
    assert plan["reference_source"] == "file"
    assert all(j["offset_change_steps"] == 0 for j in plan["joints"].values())
    assert plan["needs_confirm"] is False


def test_dry_run_refused_on_an_invalid_reference_file(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    Path(rig.config.reference_file).parent.mkdir(parents=True, exist_ok=True)
    Path(rig.config.reference_file).write_text("{broken")
    result = rig.zero.dry_run("follower")
    assert result.success is False
    assert "rest reference" in result.message
    status = rig.zero.status("follower")["zero_restore"]
    assert status["reference"] is None and status["reference_error"] is not None


def test_dry_run_logs_calib_events(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    rig.zero.dry_run("follower")
    events = rig.calib_events()
    assert [(e["flow"], e["step"], e["outcome"]) for e in events] == [
        ("zero_restore", "dry_run", "start"),
        ("zero_restore", "dry_run", "ok"),
    ]
    data = events[-1]["data"]
    assert data["needs_confirm"] is True
    assert data["joints"]["elbow_flex"]["offset_change_steps"] == -1029
    assert data["joints"]["elbow_flex"]["limits"] == [0, 4095]
    assert data["torque"] == {name: 0 for name in SEED}
    assert len({e["run_id"] for e in events}) == 1


# --- apply ------------------------------------------------------------------------------------------------------------


def test_apply_refused_without_a_dry_run(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    result = rig.zero.apply("follower", confirmed=True)
    assert (result.success, "dry run first" in result.message) == (False, True)
    assert rig.offset_writes() == []


def test_apply_refused_when_the_dry_run_is_too_old(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    assert rig.zero.dry_run("follower").success
    rig.clock.now += 61.0
    result = rig.zero.apply("follower", confirmed=True)
    assert result.success is False
    assert "dry run is 61 s old (max 60 s)" in result.message
    assert rig.offset_writes() == []
    assert plan_or_none(rig) is None


def test_apply_refuses_a_large_change_without_the_confirm(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    assert rig.zero.dry_run("follower").success
    result = rig.zero.apply("follower")
    assert result.success is False
    assert "offset change above 15 deg: shoulder_lift" in result.message
    assert "zero_restore_apply_confirmed" in result.message
    assert rig.offset_writes() == []
    assert plan_or_none(rig) is not None  # kept for the confirmed apply


def test_apply_confirmed_restores_the_reference(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    write_marker(rig)
    assert rig.zero.dry_run("follower").success
    result = rig.zero.apply("follower", confirmed=True)
    assert result.success is True, result.message
    for name, sid in zip(SEED, ARM_IDS, strict=True):
        assert rig.offset(sid) == ORIGINAL_OFFSET[name]
        assert rig.present(sid) == position_to_raw_steps(SEED[name])
        assert rig.port.servos[sid][STS_LOCK] == 1
        assert rig.port.servos[sid][STS_TORQUE_ENABLE] == 0  # torque stays off
    assert rig.offset_writes() == list(ARM_IDS)
    assert not [e for e in rig.port.events if e[1] == "torque" and e[0] in ARM_IDS and e[2] != 0]
    assert rig.torque.released_ids >= set(ARM_IDS)  # still gated: enable_torque is the next step
    assert "torque stays off" in result.message
    assert not rig.marker.exists()  # joint_offsets and home cleared: nothing left
    assert plan_or_none(rig) is None


def test_apply_without_confirm_for_a_small_change(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, shift=SMALL_SHIFT)
    assert rig.zero.dry_run("follower").success
    result = rig.zero.apply("follower")
    assert result.success is True, result.message
    assert all(rig.offset(sid) == ORIGINAL_OFFSET[n] for n, sid in zip(SEED, ARM_IDS, strict=True))


def test_apply_unlocks_before_and_locks_after_each_offset_write(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    assert rig.zero.dry_run("follower").success
    start = len(rig.port.events)
    assert rig.zero.apply("follower", confirmed=True).success
    for sid in ARM_IDS:
        events = [event for s, event, _ in rig.port.events[start:] if s == sid and not event.startswith("write_")]
        index = events.index("offset")
        assert events[index - 1] == "unlock"
        assert events[index + 1] == "lock"


def test_apply_clears_the_bridge_caches_and_refreshes_command_limits(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    assert rig.zero.dry_run("follower").success
    assert rig.zero.apply("follower", confirmed=True).success
    for sid in ARM_IDS:
        assert "goal_position" not in rig.last_written[sid]
        assert rig.last_written[sid].get("offset") == encode_offset(rig.offset(sid))
    assert rig.command_limits[1] == LIMITS[1]  # read back from the servo
    assert rig.command_limits[2] == (0, 4095)
    assert rig.command_limits[GRIPPER_ID] == (1900, 3377)  # config limits untouched


def test_apply_writes_the_json_record(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    assert rig.zero.dry_run("follower").success
    result = rig.zero.apply("follower", confirmed=True)
    record_path = only_record(rig)
    run_id = rig.zero.status("follower")["zero_restore"]["last_result"]["run_id"]
    assert record_path.name == f"zero_restore-20261109T120000Z-{run_id}.json"
    assert str(record_path) in result.message
    record = json.loads(record_path.read_text())
    assert record["success"] is True and record["confirmed"] is True
    assert record["reference_source"] == "seed"
    entry = record["joints"]["elbow_flex"]
    assert entry["applied"] is True
    assert entry["offset_before_steps"] == ORIGINAL_OFFSET["elbow_flex"] + INCIDENT_SHIFT["elbow_flex"]
    assert entry["offset_after_steps"] == ORIGINAL_OFFSET["elbow_flex"]
    assert entry["present_after_rad"] == pytest.approx(SEED["elbow_flex"], abs=STEP_RAD)
    assert entry["limits_before"] == [0, 4095] and entry["limits_after"] == [0, 4095]
    last = rig.zero.status("follower")["zero_restore"]["last_result"]
    assert last["success"] is True and last["record_path"] == str(record_path)
    assert last["joints"]["elbow_flex"]["applied"] is True
    assert last["not_attempted"] == []


def test_apply_restores_angle_limits_moved_by_the_offset_write(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    rig.port.limit_shift_on_offset = 7
    assert rig.zero.dry_run("follower").success
    assert rig.zero.apply("follower", confirmed=True).success
    for sid in ARM_IDS:
        table = rig.port.servos[sid]
        assert (word(table, STS_MIN_ANGLE_LIMIT_L), word(table, STS_MAX_ANGLE_LIMIT_L)) == LIMITS[sid]


def test_apply_verify_failure_rolls_back_and_stops(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    write_marker(rig)
    rig.port.stuck_offset = {2}
    assert rig.zero.dry_run("follower").success
    result = rig.zero.apply("follower", confirmed=True)
    assert result.success is False
    assert rig.offset_writes() == [1, 2, 2]  # shoulder_pan, shoulder_lift, its rollback; nothing after
    assert rig.offset(1) == ORIGINAL_OFFSET["shoulder_pan"]
    assert "failed: shoulder_lift (not verified (reads " in result.message
    assert "not attempted: elbow_flex, wrist_flex, wrist_roll" in result.message
    assert json.loads(rig.marker.read_text())["stale"] == list(STALE_ITEMS)  # not cleared
    last = rig.zero.status("follower")["zero_restore"]["last_result"]
    assert last["success"] is False
    assert last["not_attempted"] == ["elbow_flex", "wrist_flex", "wrist_roll"]
    record = json.loads(only_record(rig).read_text())
    assert record["joints"]["shoulder_pan"]["applied"] is True
    assert record["joints"]["shoulder_lift"]["applied"] is False
    assert any(e["step"] == "rollback" for e in rig.calib_events())


def test_apply_on_a_servo_adding_the_offset_rolls_back(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    rig.port.offset_sign = -1
    for sid in ARM_IDS:  # re-seed so the resting reading is the same post-incident reading
        offset = rig.offset(sid)
        rig.port.encoder[sid] = (rig.present(sid) - offset) % 4096
        rig.port.refresh_present(sid)
    before = {sid: rig.offset(sid) for sid in ARM_IDS}
    assert rig.zero.dry_run("follower").success
    assert rig.zero.apply("follower", confirmed=True).success is False
    assert {sid: rig.offset(sid) for sid in ARM_IDS} == before


def test_apply_stops_at_a_joint_whose_torque_came_on(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    rig.port.torque_on_after_offset = {1: 2}
    assert rig.zero.dry_run("follower").success
    result = rig.zero.apply("follower", confirmed=True)
    assert result.success is False
    assert "shoulder_lift (torque_enable 1)" in result.message
    assert rig.offset_writes() == [1]


def test_apply_refused_when_the_arm_moved_since_the_dry_run(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    assert rig.zero.dry_run("follower").success
    rig.port.encoder[3] += 40
    rig.port.refresh_present(3)
    result = rig.zero.apply("follower", confirmed=True)
    assert result.success is False
    assert "moved since the dry run: elbow_flex" in result.message
    assert rig.offset_writes() == []
    assert plan_or_none(rig) is None


def test_apply_refused_when_torque_was_enabled_after_the_dry_run(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    assert rig.zero.dry_run("follower").success
    assert rig.torque.enable("follower").success
    result = rig.zero.apply("follower", confirmed=True)
    assert (result.success, "release torque first" in result.message) == (False, True)
    assert rig.offset_writes() == []


def test_apply_consumes_the_plan(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    assert rig.zero.dry_run("follower").success
    assert rig.zero.apply("follower", confirmed=True).success
    result = rig.zero.apply("follower", confirmed=True)
    assert (result.success, "dry run first" in result.message) == (False, True)


def test_apply_never_touches_the_gripper(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    assert rig.zero.dry_run("follower").success
    start = len(rig.port.packets)
    assert rig.zero.apply("follower", confirmed=True).success
    assert not [p for p in rig.port.packets[start:] if p.servo_id == GRIPPER_ID]


def test_apply_marks_home_stale_when_it_was_recorded_in_the_wrong_zero(tmp_path: Path) -> None:
    """Only joint_offsets listed: home was re-recorded (arm_set_home) after the wrong re-zero, so it is wrong now."""
    rig = make_rig(tmp_path)
    write_marker(rig)
    content = json.loads(rig.marker.read_text())
    rig.marker.write_text(json.dumps({**content, "stale": ["joint_offsets"]}))
    assert rig.zero.dry_run("follower").success
    assert rig.zero.apply("follower", confirmed=True).success
    assert json.loads(rig.marker.read_text())["stale"] == ["home"]


def test_apply_without_a_marker_marks_a_real_zero_change_stale(tmp_path: Path) -> None:
    """No marker: the calibrations matched the old zero, so a change beyond plan_tolerance_steps makes them stale."""
    rig = make_rig(tmp_path, shift=SMALL_SHIFT)
    assert rig.zero.dry_run("follower").success
    assert rig.zero.apply("follower").success
    marker = json.loads(rig.marker.read_text())
    assert marker["stale"] == list(STALE_ITEMS)
    assert set(marker["joints"]) == set(SEED)  # every SMALL_SHIFT change is above 5 steps


def test_apply_without_a_marker_and_within_tolerance_writes_none(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, shift={name: 2 for name in SEED})
    assert rig.zero.dry_run("follower").success
    assert rig.zero.apply("follower").success
    assert not rig.marker.exists() and not rig.marker.with_name(ZERO_RECORD_FILE).exists()


def zero_record(rig: Any) -> dict[str, Any]:
    """The zero change record next to the rig's marker."""
    return json.loads(rig.marker.with_name(ZERO_RECORD_FILE).read_text())


def test_a_restore_that_clears_joint_offsets_records_the_zero_change(tmp_path: Path) -> None:
    """The marker may be deleted by the restore, but the zero changed: offsets fitted in between no longer hold."""
    rig = make_rig(tmp_path)
    write_marker(rig)
    assert rig.zero.dry_run("follower").success
    assert rig.zero.apply("follower", confirmed=True).success
    assert not rig.marker.exists()
    assert zero_record(rig) == {"version": 1, "group": "follower", "changed_at": NOW.isoformat(timespec="seconds")}


def test_a_real_zero_change_records_it(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, shift=SMALL_SHIFT)
    assert rig.zero.dry_run("follower").success
    assert rig.zero.apply("follower").success
    assert zero_record(rig)["changed_at"] == NOW.isoformat(timespec="seconds")


def test_a_partial_apply_records_the_zero_change(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    rig.port.stuck_offset = {3}
    assert rig.zero.dry_run("follower").success
    assert rig.zero.apply("follower", confirmed=True).success is False
    assert zero_record(rig)["changed_at"] == NOW.isoformat(timespec="seconds")


def test_partial_apply_without_a_marker_marks_the_written_joints_stale(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    rig.port.stuck_offset = {3}
    assert rig.zero.dry_run("follower").success
    assert rig.zero.apply("follower", confirmed=True).success is False
    marker = json.loads(rig.marker.read_text())
    assert marker["stale"] == list(STALE_ITEMS)
    assert set(marker["joints"]) == {"shoulder_pan", "shoulder_lift"}


def test_an_apply_failing_at_the_limits_records_the_zero_change(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The offset landed and was verified but the limits were not restored: the zero changed, so it is recorded."""
    rig = make_rig(tmp_path)
    assert rig.zero.dry_run("follower").success
    monkeypatch.setattr(rig.zero, "restore_limits", lambda servo_id, wanted: (0, 1))
    assert rig.zero.apply("follower", confirmed=True).success is False
    assert zero_record(rig)["changed_at"] == NOW.isoformat(timespec="seconds")
    assert "shoulder_pan" in json.loads(rig.marker.read_text())["joints"]


def test_the_restore_writes_the_zero_record_before_deleting_the_marker(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    rig = make_rig(tmp_path)
    write_marker(rig)
    content = json.loads(rig.marker.read_text())
    assert content["stale"] == list(STALE_ITEMS)
    assert rig.zero.dry_run("follower").success

    def broken(*args: Any, **kwargs: Any) -> None:
        raise OSError("disk full")

    monkeypatch.setattr(zero_restore_module, "record_zero_change", broken)
    rig.zero.apply("follower", confirmed=True)
    assert rig.marker.exists()  # the record failed first, so the marker still says the zero is suspect


def test_unacknowledged_offset_write_is_rolled_back_and_verified(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    rig.port.unacked_offset = {1}  # the write lands but no status reply comes back
    assert rig.zero.dry_run("follower").success
    result = rig.zero.apply("follower", confirmed=True)
    assert result.success is False
    assert "shoulder_pan (offset write failed; old offset 109 rewritten and read back)" in result.message
    assert rig.offset(1) == ORIGINAL_OFFSET["shoulder_pan"] + INCIDENT_SHIFT["shoulder_pan"]
    assert rig.offset_writes() == [1, 1]


def test_rollback_not_read_back_is_reported(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    rig.port.stuck_offset = {2}
    rig.port.drop_offset_reads_after_writes = {2: 2}  # the read back after the rollback never answers
    assert rig.zero.dry_run("follower").success
    result = rig.zero.apply("follower", confirmed=True)
    assert "REWRITE NOT VERIFIED" in result.message


def test_apply_logs_a_calib_event_per_step(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    assert rig.zero.dry_run("follower").success
    rig.lines.clear()
    assert rig.zero.apply("follower", confirmed=True).success
    steps = [(e["step"], e["outcome"]) for e in rig.calib_events()]
    assert steps[0] == ("apply", "start")
    assert steps.count(("joint_write", "ok")) == 5
    assert steps.count(("joint_verify", "ok")) == 5
    assert ("record", "ok") in steps and ("marker_update", "ok") in steps
    assert steps[-1] == ("apply", "ok")


# --- save rest reference ----------------------------------------------------------------------------------------------


def test_save_rest_reference_refused_while_the_zero_is_suspect(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    write_marker(rig)
    result = rig.zero.save_rest_reference("follower")
    assert result.success is False
    assert "joint_offsets" in result.message
    assert not Path(rig.config.reference_file).exists()


def test_save_rest_reference_refused_on_an_unreadable_marker(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    rig.marker.parent.mkdir(parents=True, exist_ok=True)
    rig.marker.write_text("{broken")
    assert rig.zero.save_rest_reference("follower").success is False


def test_save_rest_reference_refused_when_not_released(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, released=False)
    result = rig.zero.save_rest_reference("follower")
    assert (result.success, "release torque first" in result.message) == (False, True)


def test_save_rest_reference_stores_the_present_readings(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, shift=SMALL_SHIFT)
    assert rig.zero.dry_run("follower").success
    result = rig.zero.save_rest_reference("follower")
    assert result.success is True, result.message
    content = json.loads(Path(rig.config.reference_file).read_text())
    expected = {name: steps_to_radians(rig.present(sid)) for name, sid in zip(SEED, ARM_IDS, strict=True)}
    assert content["joints"] == pytest.approx(expected)
    assert content["saved_at"] == NOW.isoformat(timespec="seconds")
    assert content["group"] == "follower"
    status = rig.zero.status("follower")["zero_restore"]
    assert status["reference"]["source"] == "file"
    assert status["plan"] is None  # a new reference invalidates the plan
    assert [p.name for p in Path(rig.config.reference_file).parent.iterdir()] == ["rest_reference.json"]


def test_save_rest_reference_refused_while_the_arm_moves(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    rig.port.jitter[4] = [0, 9]
    assert rig.zero.save_rest_reference("follower").success is False
    assert not Path(rig.config.reference_file).exists()


# --- status -----------------------------------------------------------------------------------------------------------


def test_status_reports_the_seed_reference_and_the_stale_flag(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    status = rig.zero.status("follower")["zero_restore"]
    assert status["max_change_deg"] == 15.0
    assert status["reference"] == {"source": "seed", "saved_at": None, "joints": SEED}
    assert status["reference_error"] is None
    assert status["stale_joint_offsets"] is False
    assert status["plan"] is None and status["last_result"] is None


def test_status_stale_flag_follows_the_marker_after_its_refresh(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    assert rig.zero.status("follower")["zero_restore"]["stale_joint_offsets"] is False
    write_marker(rig)
    rig.clock.now += 2.0
    assert rig.zero.status("follower")["zero_restore"]["stale_joint_offsets"] is True


def test_status_of_an_unknown_group(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    assert rig.zero.status("nope") == {"zero_restore": None}


# --- bridge wiring: services and torque_state -------------------------------------------------------------------------


ZERO_SERVICES = {
    "/follower/servo/zero_restore_dry_run",
    "/follower/servo/zero_restore_apply",
    "/follower/servo/zero_restore_apply_confirmed",
    "/follower/servo/save_rest_reference",
}


@pytest.fixture
def bridge(monkeypatch: pytest.MonkeyPatch) -> ModuleType:
    """Import feetech_servos.bridge with ROS stubbed and String replaced by a plain message class."""
    install_service_stubs()
    module = importlib.import_module("feetech_servos.bridge")
    monkeypatch.setattr(module, "String", lambda data: SimpleNamespace(data=data))
    return module


def wire(bridge: ModuleType, rig: Rig) -> tuple[RecordingNode, Any, list[int]]:
    """setup_torque_interfaces on the rig's bus with neutral and zero restore controls."""
    node = RecordingNode()
    callbacks_run = [0]
    neutral = NeutralControl(rig.torque, rig.command_limits, sleep=rig.sleeps.append)
    reporter = bridge.setup_torque_interfaces(
        node, list(rig.torque.groups.values()), rig.torque, callbacks_run, 1.0, neutral, rig.zero
    )
    return node, reporter, callbacks_run


def call(node: RecordingNode, service: str) -> Any:
    """Call a recorded Trigger service callback."""
    return node.services[service](SimpleNamespace(), SimpleNamespace(success=None, message=None))


def test_zero_restore_services_exist_for_a_group_with_the_block(bridge: ModuleType, tmp_path: Path) -> None:
    node, _, _ = wire(bridge, make_rig(tmp_path))
    assert ZERO_SERVICES <= set(node.services)


def test_no_zero_restore_services_without_the_block(bridge: ModuleType, tmp_path: Path) -> None:
    node, _, _ = wire(bridge, make_rig(tmp_path, with_block=False))
    assert not ZERO_SERVICES & set(node.services)


def test_dry_run_service_publishes_the_plan_in_torque_state(bridge: ModuleType, tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    node, reporter, callbacks_run = wire(bridge, rig)
    reporter.step(0.0, 1)
    first = json.loads(node.publishers["/follower/torque_state"][-1].data)
    assert first["zero_restore"]["plan"] is None
    response = call(node, "/follower/servo/zero_restore_dry_run")
    assert response.success is True and callbacks_run[0] == 1
    reporter.step(0.01, 2)  # requested: published inside the interval
    state = json.loads(node.publishers["/follower/torque_state"][-1].data)
    assert state["zero_restore"]["plan"]["needs_confirm"] is True
    assert list(state["zero_restore"]["plan"]["joints"]) == list(SEED)


def test_apply_services_map_to_the_confirm(bridge: ModuleType, tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    node, _, _ = wire(bridge, rig)
    assert call(node, "/follower/servo/zero_restore_dry_run").success
    refused = call(node, "/follower/servo/zero_restore_apply")
    assert refused.success is False and "zero_restore_apply_confirmed" in refused.message
    applied = call(node, "/follower/servo/zero_restore_apply_confirmed")
    assert applied.success is True, applied.message
    assert rig.offset(3) == ORIGINAL_OFFSET["elbow_flex"]


def test_save_rest_reference_service(bridge: ModuleType, tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    node, _, _ = wire(bridge, rig)
    assert call(node, "/follower/servo/save_rest_reference").success is True
    assert Path(rig.config.reference_file).exists()


def test_payload_without_zero_restore_has_no_key() -> None:
    group = JointGroup("head", [JointEntry("head_pan", 21)])
    assert "zero_restore" not in torque_state_payload(group, {21: 1}, set(), 1)
    payload = torque_state_payload(group, {21: 1}, set(), 1, zero_restore={"plan": None})
    assert payload["zero_restore"] == {"plan": None}
