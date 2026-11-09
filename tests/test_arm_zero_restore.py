"""Unit tests for scripts/arm_zero_restore.py on the feetech bridge's emulated STS bus (no hardware)."""

import json
import math
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
# feetech_servos is imported as a package from nodes/bridges, as the follower's launcher does (PYTHONPATH).
sys.path.insert(0, str(REPO_ROOT / "nodes" / "bridges"))

from feetech_mapping.command_mapping import STEPS_PER_REVOLUTION, position_to_raw_steps  # noqa: E402
from feetech_servos.registers import (  # noqa: E402
    STS_LOCK,
    STS_MAX_ANGLE_LIMIT_L,
    STS_MIN_ANGLE_LIMIT_L,
    STS_OFS_L,
    STS_PRESENT_POSITION_L,
    STS_TORQUE_ENABLE,
)
from feetech_servos.tests.sts_bus import FakeStsPort, StsPacket, put_word, register_table  # noqa: E402
from st3215 import ST3215  # noqa: E402
from st3215.protocol_packet_handler import protocol_packet_handler  # noqa: E402
from st3215.values import INST_WRITE  # noqa: E402

import scripts.arm_zero_restore as restore_module  # noqa: E402
from scripts.arm_zero_restore import (  # noqa: E402
    EXIT_FAIL,
    EXIT_OK,
    OFFSET_MAX_STEPS,
    RestoreConfig,
    decode_offset,
    degrees_to_steps,
    encode_offset,
    load_config,
    offset_delta_steps,
    run_restore,
)

CONFIG_PATH = REPO_ROOT / "scripts" / "arm_zero_restore.yaml"
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
FIXED_NOW = "20261109T120000Z"
LIMITS = (0, 4095)
# Logged shifts of the wrong re-zero ("neutral recorded: <joint> +N steps"), the inverse of the cross-check deltas.
INCIDENT_SHIFT = {"shoulder_pan": -11, "shoulder_lift": -901, "elbow_flex": 1029, "wrist_flex": -47, "wrist_roll": 14}
# Offsets before the incident (arbitrary, sign-magnitude range).
ORIGINAL_OFFSET = {"shoulder_pan": 120, "shoulder_lift": -300, "elbow_flex": 450, "wrist_flex": -20, "wrist_roll": 0}


@dataclass
class OffsetPort(FakeStsPort):
    """Emulated STS bus whose servos read present_position = (encoder - offset) mod 4096, like the STS3215.

    Attributes:
        encoder: servo_id -> raw encoder steps of the resting arm (does not change: the arm never moves).
        offset_sign: +1 for the STS3215 semantics; -1 models a servo adding the offset (wrong convention).
        stuck_offset: Servos acknowledging offset writes without storing them.
        limit_shift_on_offset: Steps the angle limit registers move by on an offset write (limit restore path).
    """

    encoder: dict[int, int] = field(default_factory=dict)
    offset_sign: int = 1
    stuck_offset: set[int] = field(default_factory=set)
    limit_shift_on_offset: int = 0

    def refresh_present(self, servo_id: int) -> None:
        """Recompute present_position of one servo from its encoder and offset register."""
        table = self.servos[servo_id]
        offset = decode_offset(table[STS_OFS_L] | (table[STS_OFS_L + 1] << 8))
        put_word(table, STS_PRESENT_POSITION_L, (self.encoder[servo_id] - self.offset_sign * offset) % 4096)

    def apply(self, packet: StsPacket) -> None:
        """Apply the instruction, then model the offset register's effect."""
        is_offset_write = packet.instruction == INST_WRITE and packet.params and packet.params[0] == STS_OFS_L
        if is_offset_write and packet.servo_id in self.stuck_offset:
            self.reply(packet.servo_id, [])
            return
        super().apply(packet)
        if is_offset_write and packet.servo_id in self.servos:
            self.refresh_present(packet.servo_id)
            if self.limit_shift_on_offset:
                table = self.servos[packet.servo_id]
                put_word(table, STS_MIN_ANGLE_LIMIT_L, LIMITS[0] + self.limit_shift_on_offset)
                put_word(table, STS_MAX_ANGLE_LIMIT_L, LIMITS[1] - self.limit_shift_on_offset)


def config_with(tmp_path: Path, **changes: object) -> RestoreConfig:
    """Load the repo config with record_path in tmp_path and optional top-level overrides.

    Args:
        tmp_path (Path): Test directory.
        **changes (object): Top-level keys to replace.

    Returns:
        RestoreConfig: The config.
    """
    data = yaml.safe_load(CONFIG_PATH.read_text())
    data["record_path"] = str(tmp_path / "zero_restore-{timestamp}.json")
    data.update(changes)
    return RestoreConfig(**data)


def incident_bus(
    config: RestoreConfig, torque: dict[int, int] | None = None, shift: dict[str, int] | None = None
) -> tuple[ST3215, OffsetPort]:
    """Build the follower arm after the incident: at rest, offsets moved by the logged DefineMiddle shifts.

    Before the incident each joint read its reference at rest with ORIGINAL_OFFSET. DefineMiddle at a reading of
    2048 + shift raised the offset by shift, so the resting arm now reads reference - shift.

    Args:
        config (RestoreConfig): Restore config (joints, references).
        torque (dict[int, int] | None): servo_id -> torque_enable; default 0 for all.
        shift (dict[str, int] | None): Incident shift per joint; default INCIDENT_SHIFT.

    Returns:
        tuple[ST3215, OffsetPort]: The library's ST3215 (its UnLockEprom / LockEprom) on the emulated port.
    """
    shift = shift or INCIDENT_SHIFT
    port = OffsetPort(servos={})
    for joint in config.joints:
        target = position_to_raw_steps(joint.reference_rad, joint.inverted)
        table = register_table()
        put_word(table, STS_MIN_ANGLE_LIMIT_L, LIMITS[0])
        put_word(table, STS_MAX_ANGLE_LIMIT_L, LIMITS[1])
        table[STS_TORQUE_ENABLE] = (torque or {}).get(joint.id, 0)
        table[STS_LOCK] = 1
        port.encoder[joint.id] = target + ORIGINAL_OFFSET[joint.name]
        put_word(table, STS_OFS_L, encode_offset(ORIGINAL_OFFSET[joint.name] + shift[joint.name]))
        port.servos[joint.id] = table
        port.refresh_present(joint.id)
    return bus_on(port), port


def bus_on(port: FakeStsPort) -> ST3215:
    """Return a real st3215.ST3215 talking to the emulated port (its __init__ would open a serial device).

    Args:
        port (FakeStsPort): Emulated bus.

    Returns:
        ST3215: The library class with the emulated port handler.
    """
    servo = ST3215.__new__(ST3215)
    protocol_packet_handler.__init__(servo, port)
    return servo


def offset_writes(port: FakeStsPort) -> list[StsPacket]:
    """Return every write to the offset register."""
    return [p for p in port.packets if p.instruction == INST_WRITE and p.params[0] == STS_OFS_L]


def eeprom_writes(port: FakeStsPort) -> list[StsPacket]:
    """Return every write to anything but the lock register (EEPROM data or torque)."""
    return [p for p in port.packets if p.instruction == INST_WRITE and p.params[0] != STS_LOCK]


def read_offset(port: FakeStsPort, servo_id: int) -> int:
    """Decoded offset register of one emulated servo."""
    table = port.servos[servo_id]
    return decode_offset(table[STS_OFS_L] | (table[STS_OFS_L + 1] << 8))


def read_present(port: FakeStsPort, servo_id: int) -> int:
    """present_position of one emulated servo."""
    table = port.servos[servo_id]
    return table[STS_PRESENT_POSITION_L] | (table[STS_PRESENT_POSITION_L + 1] << 8)


def only_record(tmp_path: Path) -> dict:
    """Parse the single JSON record the run wrote in tmp_path."""
    records = list(tmp_path.glob("zero_restore-*.json"))
    assert len(records) == 1
    assert records[0].name == f"zero_restore-{FIXED_NOW}.json"
    return json.loads(records[0].read_text())


# --- sign-magnitude offset encoding (STS3215: magnitude bits 0-10, sign bit 11) ---


@pytest.mark.parametrize(
    ("steps", "raw"),
    [(0, 0x000), (1, 0x001), (-1, 0x801), (901, 0x385), (-1029, 0xC05), (2047, 0x7FF), (-2047, 0xFFF)],
)
def test_offset_encoding_round_trip(steps: int, raw: int) -> None:
    """Encoding sets bit 11 for negatives and keeps the magnitude in bits 0-10; decoding inverts it."""
    assert encode_offset(steps) == raw
    assert decode_offset(raw) == steps


def test_negative_zero_decodes_to_zero() -> None:
    """0x800 (sign bit, magnitude 0) is zero."""
    assert decode_offset(0x800) == 0


@pytest.mark.parametrize("steps", [2048, -2048, 4095])
def test_encode_rejects_magnitude_over_2047(steps: int) -> None:
    """The 11-bit magnitude cannot hold 2048 or more: refused, never clamped."""
    with pytest.raises(ValueError):
        encode_offset(steps)


@pytest.mark.parametrize("raw", [0x1000, 0x8001, 0xF000])
def test_decode_rejects_bits_above_sign(raw: int) -> None:
    """Bits 12-15 are not part of the register value: a read with them set is refused."""
    with pytest.raises(ValueError):
        decode_offset(raw)


def test_offset_max_is_2047() -> None:
    """The largest offset magnitude is 2047 steps."""
    assert OFFSET_MAX_STEPS == 2047


# --- delta computation ---


def test_degrees_to_steps() -> None:
    """5 deg = 4096 * 5 / 360 steps."""
    assert degrees_to_steps(5.0) == pytest.approx(56.888, abs=1e-3)


def test_offset_delta_is_present_minus_target() -> None:
    """present = encoder - offset, so raising the offset by present - target makes present read target."""
    assert offset_delta_steps(present=1944, target=1043) == 901
    assert offset_delta_steps(present=1000, target=2029) == -1029


def test_offset_delta_takes_the_short_way_across_the_wrap() -> None:
    """A reading on the other side of 0/4095 gives the short delta, not a near-full turn."""
    assert offset_delta_steps(present=4090, target=10) == -16
    assert offset_delta_steps(present=10, target=4090) == 16


def test_inverted_joint_target_follows_the_bridge_mapping(tmp_path: Path) -> None:
    """An inverted joint's target is the bridge's position_to_raw_steps(rad, inverted=True): delta sign flips."""
    data = yaml.safe_load(CONFIG_PATH.read_text())
    data["record_path"] = str(tmp_path / "zero_restore-{timestamp}.json")
    data["joints"] = [dict(data["joints"][3], inverted=True, expected_offset_delta_steps=47)]
    config = RestoreConfig(**data)
    servo, port = incident_bus(config)
    joint = config.joints[0]
    assert read_present(port, joint.id) == position_to_raw_steps(-0.862) - INCIDENT_SHIFT["wrist_flex"]

    code = run_restore(config, servo, apply=True, now=lambda: FIXED_NOW, sleep=lambda _s: None)

    assert code == EXIT_OK
    assert read_present(port, joint.id) == position_to_raw_steps(0.862, inverted=True)
    assert read_offset(port, joint.id) == ORIGINAL_OFFSET["wrist_flex"]


def test_dry_run_computes_the_inverse_of_the_incident_shift(tmp_path: Path) -> None:
    """On the post-incident arm the planned offset change per joint is minus the logged shift."""
    config = config_with(tmp_path)
    servo, port = incident_bus(config)

    code = run_restore(config, servo, apply=False, now=lambda: FIXED_NOW, sleep=lambda _s: None)

    assert code == EXIT_OK
    record = only_record(tmp_path)
    assert record["mode"] == "dry_run" and record["result"] == "ok"
    for joint in config.joints:
        entry = record["joints"][joint.name]
        assert entry["delta_steps"] == -INCIDENT_SHIFT[joint.name]
        assert entry["new_offset_steps"] == ORIGINAL_OFFSET[joint.name]
        assert entry["offset_before_steps"] == ORIGINAL_OFFSET[joint.name] + INCIDENT_SHIFT[joint.name]
        assert entry["target_steps"] == position_to_raw_steps(joint.reference_rad)
        assert entry["present_after_steps"] is None and entry["applied"] is False


# --- refusals: torque, cross-check, range, reads ---


def test_refuses_when_a_servo_has_torque_on(tmp_path: Path) -> None:
    """Any arm servo with torque_enable != 0 refuses the run before any write, even with --apply."""
    config = config_with(tmp_path)
    servo, port = incident_bus(config, torque={3: 1})

    code = run_restore(config, servo, apply=True, now=lambda: FIXED_NOW, sleep=lambda _s: None)

    assert code == EXIT_FAIL
    assert eeprom_writes(port) == []
    record = only_record(tmp_path)
    assert record["result"] == "refused" and "elbow_flex" in record["message"]


def test_refuses_when_torque_cannot_be_read(tmp_path: Path) -> None:
    """A servo that does not answer counts as not verified torque off: refused, nothing written."""
    config = config_with(tmp_path)
    servo, port = incident_bus(config)
    del port.servos[5]

    code = run_restore(config, servo, apply=True, now=lambda: FIXED_NOW, sleep=lambda _s: None)

    assert code == EXIT_FAIL
    assert eeprom_writes(port) == []
    assert "wrist_roll" in only_record(tmp_path)["message"]


def test_cross_check_mismatch_aborts_writing_nothing(tmp_path: Path) -> None:
    """A computed delta more than tolerance_deg from the expected delta aborts the whole run, all joints unwritten."""
    config = config_with(tmp_path)
    shift = dict(INCIDENT_SHIFT, wrist_flex=-47 - 60)  # 60 steps = 5.3 deg off the expected +47
    servo, port = incident_bus(config, shift=shift)

    code = run_restore(config, servo, apply=True, now=lambda: FIXED_NOW, sleep=lambda _s: None)

    assert code == EXIT_FAIL
    assert eeprom_writes(port) == []
    record = only_record(tmp_path)
    assert record["result"] == "refused" and "wrist_flex" in record["message"]
    assert record["joints"]["wrist_flex"]["deviation_steps"] == 60


def test_cross_check_within_tolerance_passes(tmp_path: Path) -> None:
    """A delta 56 steps (4.9 deg) from the expected one is accepted."""
    config = config_with(tmp_path)
    servo, _port = incident_bus(config, shift=dict(INCIDENT_SHIFT, wrist_flex=-47 - 56))

    assert run_restore(config, servo, apply=False, now=lambda: FIXED_NOW, sleep=lambda _s: None) == EXIT_OK


def test_new_offset_out_of_range_aborts(tmp_path: Path) -> None:
    """A new offset beyond +-2047 cannot be encoded: refused, nothing written."""
    data = yaml.safe_load(CONFIG_PATH.read_text())
    data["record_path"] = str(tmp_path / "zero_restore-{timestamp}.json")
    config = RestoreConfig(**data)
    servo, port = incident_bus(config)
    joint = config.joints[1]  # shoulder_lift, delta +901
    table = port.servos[joint.id]
    put_word(table, STS_OFS_L, encode_offset(1900))
    port.encoder[joint.id] += 1900 - (ORIGINAL_OFFSET[joint.name] + INCIDENT_SHIFT[joint.name])
    port.refresh_present(joint.id)

    code = run_restore(config, servo, apply=True, now=lambda: FIXED_NOW, sleep=lambda _s: None)

    assert code == EXIT_FAIL
    assert eeprom_writes(port) == []
    assert "shoulder_lift" in only_record(tmp_path)["message"]


# --- dry run and apply ---


def test_dry_run_writes_nothing(tmp_path: Path) -> None:
    """Without --apply only reads reach the bus: no write packet at all, not even the EEPROM lock."""
    config = config_with(tmp_path)
    servo, port = incident_bus(config)

    run_restore(config, servo, apply=False, now=lambda: FIXED_NOW, sleep=lambda _s: None)

    assert [p for p in port.packets if p.instruction == INST_WRITE] == []


def test_apply_writes_offsets_and_verifies_reference(tmp_path: Path) -> None:
    """--apply writes each offset inside UnLockEprom/LockEprom; every joint then reads its reference target."""
    config = config_with(tmp_path)
    servo, port = incident_bus(config)

    code = run_restore(config, servo, apply=True, now=lambda: FIXED_NOW, sleep=lambda _s: None)

    assert code == EXIT_OK
    for joint in config.joints:
        assert read_offset(port, joint.id) == ORIGINAL_OFFSET[joint.name]
        assert read_present(port, joint.id) == position_to_raw_steps(joint.reference_rad)
        assert port.servos[joint.id][STS_LOCK] == 1
        assert port.servos[joint.id][STS_TORQUE_ENABLE] == 0
    assert [p.servo_id for p in offset_writes(port)] == [j.id for j in config.joints]
    assert not [p for p in eeprom_writes(port) if p.params[0] == STS_TORQUE_ENABLE]
    record = only_record(tmp_path)
    assert record["mode"] == "apply" and record["result"] == "ok"
    for joint in config.joints:
        entry = record["joints"][joint.name]
        assert entry["applied"] is True
        assert entry["offset_after_steps"] == ORIGINAL_OFFSET[joint.name]
        assert entry["present_after_steps"] == position_to_raw_steps(joint.reference_rad)
        assert entry["present_after_rad"] == pytest.approx(joint.reference_rad, abs=2 * math.pi / STEPS_PER_REVOLUTION)
        assert entry["limits_before"] == list(LIMITS) and entry["limits_after"] == list(LIMITS)


def test_apply_unlocks_before_and_locks_after_each_offset_write(tmp_path: Path) -> None:
    """Per joint: lock 0 write before the offset write, lock 1 write after it (EEPROM relocked)."""
    config = config_with(tmp_path)
    servo, port = incident_bus(config)

    run_restore(config, servo, apply=True, now=lambda: FIXED_NOW, sleep=lambda _s: None)

    for joint in config.joints:
        writes = [p for p in port.packets if p.instruction == INST_WRITE and p.servo_id == joint.id]
        index = next(i for i, p in enumerate(writes) if p.params[0] == STS_OFS_L)
        assert writes[index - 1].params == [STS_LOCK, 0]
        assert writes[index + 1].params[0] == STS_LOCK
        assert writes[-1].params == [STS_LOCK, 1]


def test_apply_restores_angle_limits_moved_by_the_offset_write(tmp_path: Path) -> None:
    """Limits that read differently after the offset write are written back to their pre-write values (neutral.py)."""
    config = config_with(tmp_path)
    servo, port = incident_bus(config)
    port.limit_shift_on_offset = 7

    code = run_restore(config, servo, apply=True, now=lambda: FIXED_NOW, sleep=lambda _s: None)

    assert code == EXIT_OK
    for joint in config.joints:
        table = port.servos[joint.id]
        assert table[STS_MIN_ANGLE_LIMIT_L] | (table[STS_MIN_ANGLE_LIMIT_L + 1] << 8) == LIMITS[0]
        assert table[STS_MAX_ANGLE_LIMIT_L] | (table[STS_MAX_ANGLE_LIMIT_L + 1] << 8) == LIMITS[1]


def test_apply_verify_failure_rolls_back_and_stops(tmp_path: Path) -> None:
    """A servo that does not take the offset fails the verify: its old offset is rewritten, later joints untouched."""
    config = config_with(tmp_path)
    servo, port = incident_bus(config)
    port.stuck_offset = {2}

    code = run_restore(config, servo, apply=True, now=lambda: FIXED_NOW, sleep=lambda _s: None)

    assert code == EXIT_FAIL
    assert read_offset(port, 1) == ORIGINAL_OFFSET["shoulder_pan"]
    written = [p.servo_id for p in offset_writes(port)]
    assert written == [1, 2, 2]  # shoulder_pan, shoulder_lift, its rollback; nothing after
    record = only_record(tmp_path)
    assert record["result"] == "failed" and "shoulder_lift" in record["message"]
    assert record["joints"]["shoulder_pan"]["applied"] is True
    assert record["joints"]["shoulder_lift"]["applied"] is False
    assert record["joints"]["elbow_flex"]["applied"] is False


def test_apply_on_a_servo_adding_the_offset_rolls_back(tmp_path: Path) -> None:
    """Wrong sign convention (present = encoder + offset): the verify catches it and the old offset is restored."""
    config = config_with(tmp_path)
    servo, port = incident_bus(config)
    port.offset_sign = -1
    for joint in config.joints:  # re-seed so the resting reading is the same post-incident reading
        offset = read_offset(port, joint.id)
        port.encoder[joint.id] = (read_present(port, joint.id) - offset) % 4096
        port.refresh_present(joint.id)
    before = {j.id: read_offset(port, j.id) for j in config.joints}

    code = run_restore(config, servo, apply=True, now=lambda: FIXED_NOW, sleep=lambda _s: None)

    assert code == EXIT_FAIL
    assert read_offset(port, 1) == before[1]
    assert {j.id: read_offset(port, j.id) for j in config.joints} == before


def test_apply_refuses_a_joint_whose_torque_came_on(tmp_path: Path) -> None:
    """Torque is re-read right before each joint's write; a servo found on stops the run without writing it."""
    config = config_with(tmp_path)
    servo, port = incident_bus(config)
    original_apply = port.apply

    def torque_on_after_first_offset(packet: StsPacket) -> None:
        original_apply(packet)
        if packet.instruction == INST_WRITE and packet.params[0] == STS_OFS_L and packet.servo_id == 1:
            port.servos[2][STS_TORQUE_ENABLE] = 1

    port.apply = torque_on_after_first_offset  # type: ignore[method-assign]

    code = run_restore(config, servo, apply=True, now=lambda: FIXED_NOW, sleep=lambda _s: None)

    assert code == EXIT_FAIL
    assert [p.servo_id for p in offset_writes(port)] == [1]


# --- config ---


def test_repo_config_matches_the_follower_bridge_config() -> None:
    """Device, joint names, ids and inverted flags equal the lerobot_follower entry of client.yml."""
    config = load_config(CONFIG_PATH)
    nodes = yaml.safe_load(CLIENT_VARS.read_text())["ros2_nodes"]
    follower = yaml.safe_load(next(n for n in nodes if n["name"] == "lerobot_follower")["config"])
    bridge = {j["name"]: j for j in follower["joint_names"]}
    assert config.device == follower["device"]
    for joint in config.joints:
        assert bridge[joint.name]["id"] == joint.id
        assert bool(bridge[joint.name].get("inverted", False)) == joint.inverted
    assert [j.name for j in config.joints] == [n for n in bridge if n not in follower["neutral_exclude_joints"]]


def test_repo_config_values() -> None:
    """The reference readings, cross-check deltas and tolerances of the incident."""
    config = load_config(CONFIG_PATH)
    assert config.tolerance_deg == 5.0
    assert config.verify_tolerance_steps == 3
    assert {j.name: j.reference_rad for j in config.joints} == {
        "shoulder_pan": -0.018,
        "shoulder_lift": -1.542,
        "elbow_flex": 1.516,
        "wrist_flex": 0.862,
        "wrist_roll": -0.072,
    }
    assert {j.name: j.expected_offset_delta_steps for j in config.joints} == {
        name: -shift for name, shift in INCIDENT_SHIFT.items()
    }
    assert config.record_path == "/var/lib/ros2/arm/zero_restore-{timestamp}.json"


def test_config_rejects_duplicate_ids(tmp_path: Path) -> None:
    """Two joints on one servo id are refused at load."""
    data = yaml.safe_load(CONFIG_PATH.read_text())
    data["joints"][1]["id"] = 1
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        load_config(path)


def test_config_rejects_unknown_keys(tmp_path: Path) -> None:
    """Unknown keys are refused (typos never silently ignored)."""
    data = yaml.safe_load(CONFIG_PATH.read_text())
    data["tolerence_deg"] = 1.0
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        load_config(path)


# --- bus ownership guard ---


def test_main_refuses_while_the_follower_bridge_runs(monkeypatch: pytest.MonkeyPatch) -> None:
    """With ros2-lerobot_follower active the serial port is never opened."""
    opened: list[str] = []
    monkeypatch.setattr(restore_module, "service_active", lambda name: name == "ros2-lerobot_follower")
    monkeypatch.setattr(restore_module, "ST3215", lambda device: opened.append(device))

    assert restore_module.main(["--config", str(CONFIG_PATH)]) == EXIT_FAIL
    assert opened == []


def test_service_active_asks_systemctl(monkeypatch: pytest.MonkeyPatch) -> None:
    """systemctl is-active exit 0 means active; any other exit code means not active."""
    calls: list[list[str]] = []

    def fake_run(cmd: list[str], check: bool) -> subprocess.CompletedProcess:
        calls.append(cmd)
        return subprocess.CompletedProcess(cmd, 0 if cmd[-1] == "running" else 3)

    monkeypatch.setattr(restore_module.subprocess, "run", fake_run)
    assert restore_module.service_active("running") is True
    assert restore_module.service_active("stopped") is False
    assert calls[0] == ["systemctl", "is-active", "--quiet", "running"]


def test_repo_config_names_the_follower_service() -> None:
    """The guarded unit is the follower bridge's systemd service."""
    assert load_config(CONFIG_PATH).bridge_service == "ros2-lerobot_follower"
