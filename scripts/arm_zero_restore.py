#!/usr/bin/env python3
"""Restore the SO-101 follower arm zero (STS3215 EEPROM offset) without moving the arm.

One-off repair after a neutral save (DefineMiddle) recorded in the wrong pose. With the arm resting folded and torque
OFF, each servo's offset register is set so that its present reading, mapped to joint radians exactly as the
feetech bridge publishes /follower/joint_states (feetech_mapping steps_to_radians / position_to_raw_steps with the
joint's inverted flag), equals the reference reading taken before the incident.

Sign convention (STS3215, sign-magnitude offset at address 31, sign bit 11): present = (encoder - offset) mod 4096.
DefineMiddle at a reading of 2048 + N ("neutral recorded: <joint> +N steps", neutral.py shift_text) raises the
offset by N so the servo reads 2048. To make a resting servo read target, the offset changes by
present - target (offset_delta_steps); the expected change is the inverse of the logged shift.

Steps: refuse unless every listed servo reads torque_enable 0; read present_position, the offset register and the
angle limits; compute per joint the target, the offset change and the new offset; refuse (writing nothing) when any
change differs from expected_offset_delta_steps by more than tolerance_deg, a new offset cannot be encoded (+-2047)
or a read fails. Dry run (default) stops there. --apply per joint, in config order: re-read torque (refuse when on),
UnLockEprom, write the offset, LockEprom (as neutral.py does around its EEPROM writes), settle, verify present within
verify_tolerance_steps of target (else rewrite the old offset and stop), restore the angle limit registers if they
read differently (as neutral.py does after DefineMiddle). Torque is never written. A JSON record with before/after per
joint goes to record_path for every run that reached the bus.

Runs on the client RPi with ros2-lerobot_follower STOPPED (the bus must be free; the script refuses while
bridge_service is active), in the follower's venv:

    sudo systemctl stop ros2-lerobot_follower
    cd /opt/ros2-lerobot-swerve-platform
    PYTHONPATH=nodes/bridges /opt/ros2-nodes/lerobot_follower/venv/bin/python3 scripts/arm_zero_restore.py
    PYTHONPATH=nodes/bridges /opt/ros2-nodes/lerobot_follower/venv/bin/python3 scripts/arm_zero_restore.py --apply

Procedure: docs/runbooks/servo-torque-neutral.md "Arm zero restore".
"""

import argparse
import json
import logging
import subprocess
import sys
import time
from collections.abc import Callable
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import yaml
from feetech_mapping.command_mapping import STEPS_PER_REVOLUTION, position_to_raw_steps, steps_to_radians
from feetech_servos.registers import get_register_entry_by_name, read_register, write_register
from pydantic import BaseModel, Field
from st3215 import ST3215

LOG = logging.getLogger("arm_zero_restore")

DEFAULT_CONFIG = Path(__file__).with_name("arm_zero_restore.yaml")
TIMESTAMP_FORMAT = "%Y%m%dT%H%M%SZ"
DEGREES_PER_TURN = 360.0
HALF_TURN_STEPS = STEPS_PER_REVOLUTION // 2
# STS3215 offset register: sign-magnitude, magnitude in bits 0-10, sign (negative) in bit 11.
OFFSET_SIGN_BIT = 0x800
OFFSET_MAGNITUDE_MASK = 0x7FF
OFFSET_MAX_STEPS = OFFSET_MAGNITUDE_MASK
OFFSET_VALID_BITS = OFFSET_SIGN_BIT | OFFSET_MAGNITUDE_MASK
TORQUE_OFF = 0
READ_ATTEMPTS = 3
SETTLE_S = 0.5
VERIFY_READ_ATTEMPTS = 10
VERIFY_RETRY_SLEEP_S = 0.1
OFFSET_REGISTER = "offset"
PRESENT_REGISTER = "present_position"
TORQUE_REGISTER = "torque_enable"
MIN_LIMIT_REGISTER = "min_angle_limit"
MAX_LIMIT_REGISTER = "max_angle_limit"
MODE_DRY_RUN = "dry_run"
MODE_APPLY = "apply"
RESULT_OK = "ok"
RESULT_REFUSED = "refused"
RESULT_FAILED = "failed"
EXIT_OK = 0
EXIT_FAIL = 1


class Forbid(BaseModel, extra="forbid"):
    """Base model rejecting unknown keys (class keyword config: pydantic 1.10 on the Pi and 2 in the dev venv)."""


class JointConfig(Forbid):
    """One arm joint to restore.

    Attributes:
        name: Joint name as in /follower/joint_states.
        id: Servo ID on the follower bus.
        inverted: The bridge's inverted flag of the joint.
        reference_rad: Resting reading before the incident (rad, joint frame).
        expected_offset_delta_steps: Expected offset change (new - current), the inverse of the logged shift.
    """

    name: str
    id: int = Field(ge=0, le=253)
    inverted: bool = False
    reference_rad: float
    expected_offset_delta_steps: int


class RestoreConfig(Forbid):
    """Config of the restore (scripts/arm_zero_restore.yaml).

    Attributes:
        device: Follower bus serial device.
        bridge_service: systemd unit of the follower bridge; the run refuses while it is active (bus owner).
        record_path: JSON record path; {timestamp} is replaced by the UTC run time.
        tolerance_deg: Largest allowed |delta - expected delta| (deg).
        verify_tolerance_steps: Largest allowed |present - target| after a write (steps).
        joints: Joints in write order.
    """

    device: str
    bridge_service: str
    record_path: str
    tolerance_deg: float = Field(gt=0)
    verify_tolerance_steps: int = Field(ge=0)
    joints: list[JointConfig]


@dataclass
class JointRecord:
    """Before/after of one joint (the JSON record entry).

    Attributes:
        id: Servo ID.
        reference_rad: Reference reading (rad).
        target_steps: present_position that reads reference_rad through the bridge mapping.
        present_before_steps: present_position read before any write.
        present_before_rad: The same in joint radians.
        offset_raw_before: Raw offset register before.
        offset_before_steps: Decoded offset before.
        delta_steps: Offset change that makes present read target.
        expected_delta_steps: Cross-check offset change from the config.
        deviation_steps: delta_steps - expected_delta_steps.
        new_offset_steps: offset_before_steps + delta_steps.
        limits_before: (min, max) angle limits before.
        applied: True when the new offset was written and verified.
        offset_after_steps: Decoded offset read back after the write.
        present_after_steps: present_position read after the write.
        present_after_rad: The same in joint radians.
        limits_after: (min, max) angle limits after the write.
    """

    id: int
    reference_rad: float
    target_steps: int
    present_before_steps: int | None = None
    present_before_rad: float | None = None
    offset_raw_before: int | None = None
    offset_before_steps: int | None = None
    delta_steps: int | None = None
    expected_delta_steps: int | None = None
    deviation_steps: int | None = None
    new_offset_steps: int | None = None
    limits_before: tuple[int, int] | None = None
    applied: bool = False
    offset_after_steps: int | None = None
    present_after_steps: int | None = None
    present_after_rad: float | None = None
    limits_after: tuple[int, int] | None = None


def load_config(path: Path) -> RestoreConfig:
    """Load and validate the restore config.

    Args:
        path (Path): YAML file.

    Returns:
        RestoreConfig: The config.

    Raises:
        ValueError: Invalid content (pydantic ValidationError is a ValueError), no joints, duplicate names or ids.
    """
    config = RestoreConfig(**yaml.safe_load(path.read_text()))
    names = [j.name for j in config.joints]
    ids = [j.id for j in config.joints]
    if not names or len(set(names)) != len(names) or len(set(ids)) != len(ids):
        raise ValueError(f"{path}: joints must be non-empty with unique names and ids (names {names}, ids {ids})")
    return config


def encode_offset(steps: int) -> int:
    """Encode an offset in steps as the STS3215 sign-magnitude register value.

    Args:
        steps (int): Offset, -2047..2047.

    Returns:
        int: Raw register value (bit 11 set for negative, magnitude in bits 0-10).

    Raises:
        ValueError: |steps| > 2047 (never clamped).
    """
    if abs(steps) > OFFSET_MAX_STEPS:
        raise ValueError(f"offset {steps} outside +-{OFFSET_MAX_STEPS}")
    return (-steps | OFFSET_SIGN_BIT) if steps < 0 else steps


def decode_offset(raw: int) -> int:
    """Decode a raw STS3215 offset register value.

    Args:
        raw (int): Raw 2-byte value.

    Returns:
        int: Offset in steps (0x800, negative zero, is 0).

    Raises:
        ValueError: Bits above the sign bit are set (not an offset value).
    """
    if raw & ~OFFSET_VALID_BITS:
        raise ValueError(f"offset register 0x{raw:04x} has bits above bit 11 set")
    magnitude = raw & OFFSET_MAGNITUDE_MASK
    return -magnitude if raw & OFFSET_SIGN_BIT else magnitude


def degrees_to_steps(degrees: float) -> float:
    """Convert degrees to servo steps.

    Args:
        degrees (float): Angle in degrees.

    Returns:
        float: Steps (4096 per turn).
    """
    return degrees * STEPS_PER_REVOLUTION / DEGREES_PER_TURN


def offset_delta_steps(present: int, target: int) -> int:
    """Offset change that makes a resting servo reading present read target (present = encoder - offset).

    Args:
        present (int): Current present_position (0..4095).
        target (int): Wanted present_position (0..4095).

    Returns:
        int: present - target, the short way round the 0/4095 wrap (-2048..2047).
    """
    return (present - target + HALF_TURN_STEPS) % STEPS_PER_REVOLUTION - HALF_TURN_STEPS


def read_retried(servo: Any, servo_id: int, register: str, attempts: int = READ_ATTEMPTS) -> int | None:
    """Read one register by name, retrying a failed read.

    Args:
        servo (Any): ST3215-compatible bus.
        servo_id (int): Servo ID.
        register (str): Register name (feetech_servos.registers).
        attempts (int): Maximum reads.

    Returns:
        int | None: Raw value, or None when every read failed.
    """
    entry = get_register_entry_by_name(register)
    for _ in range(attempts):
        value = read_register(servo, servo_id, entry)
        if value is not None:
            return value
    return None


def read_limits(servo: Any, servo_id: int) -> tuple[int, int] | None:
    """Read (min_angle_limit, max_angle_limit).

    Args:
        servo (Any): ST3215-compatible bus.
        servo_id (int): Servo ID.

    Returns:
        tuple[int, int] | None: The limits, or None when a read failed.
    """
    low = read_retried(servo, servo_id, MIN_LIMIT_REGISTER)
    high = read_retried(servo, servo_id, MAX_LIMIT_REGISTER)
    return None if low is None or high is None else (low, high)


def torque_refusals(servo: Any, joints: list[JointConfig]) -> list[str]:
    """Name every joint whose torque_enable is not verified 0.

    Args:
        servo (Any): ST3215-compatible bus.
        joints (list[JointConfig]): Joints to check.

    Returns:
        list[str]: "<name> (torque_enable N)" or "<name> (torque read failed)" per refused joint.
    """
    refused = []
    for joint in joints:
        torque = read_retried(servo, joint.id, TORQUE_REGISTER)
        if torque is None:
            refused.append(f"{joint.name} (torque read failed)")
        elif torque != TORQUE_OFF:
            refused.append(f"{joint.name} (torque_enable {torque})")
    return refused


def plan_joint(servo: Any, joint: JointConfig, tolerance_steps: float, record: JointRecord) -> str | None:
    """Read one joint and fill its planned offset change into record.

    Args:
        servo (Any): ST3215-compatible bus.
        joint (JointConfig): The joint.
        tolerance_steps (float): Largest allowed |delta - expected delta|.
        record (JointRecord): Filled in place.

    Returns:
        str | None: Why this joint refuses the run; None when the plan is valid.
    """
    present = read_retried(servo, joint.id, PRESENT_REGISTER)
    raw_offset = read_retried(servo, joint.id, OFFSET_REGISTER)
    limits = read_limits(servo, joint.id)
    if present is None or raw_offset is None or limits is None:
        return f"{joint.name}: present/offset/limit read failed"
    record.present_before_steps = present
    record.present_before_rad = steps_to_radians(present, joint.inverted)
    record.offset_raw_before = raw_offset
    record.limits_before = limits
    try:
        record.offset_before_steps = decode_offset(raw_offset)
    except ValueError as exc:
        return f"{joint.name}: {exc}"
    delta = offset_delta_steps(present, record.target_steps)
    record.delta_steps = delta
    record.expected_delta_steps = joint.expected_offset_delta_steps
    record.deviation_steps = delta - joint.expected_offset_delta_steps
    record.new_offset_steps = record.offset_before_steps + delta
    if abs(record.deviation_steps) > tolerance_steps:
        return (
            f"{joint.name}: offset change {delta:+d} differs from the expected {joint.expected_offset_delta_steps:+d} "
            f"by {record.deviation_steps:+d} steps (tolerance {tolerance_steps:.1f})"
        )
    if abs(record.new_offset_steps) > OFFSET_MAX_STEPS:
        return f"{joint.name}: new offset {record.new_offset_steps} outside +-{OFFSET_MAX_STEPS}"
    return None


def write_offset(servo: Any, servo_id: int, steps: int) -> bool:
    """Write the offset register inside UnLockEprom / LockEprom (always relocked), as neutral.py writes EEPROM.

    Args:
        servo (Any): ST3215-compatible bus.
        servo_id (int): Servo ID.
        steps (int): Offset in steps (-2047..2047).

    Returns:
        bool: True when the write was acknowledged.
    """
    servo.UnLockEprom(servo_id)
    try:
        return write_register(servo, servo_id, get_register_entry_by_name(OFFSET_REGISTER), encode_offset(steps), {})
    finally:
        servo.LockEprom(servo_id)


def verify_present(servo: Any, servo_id: int, target: int, tolerance: int, sleep: Callable[[float], Any]) -> int | None:
    """Read present_position until it is within tolerance of target.

    Args:
        servo (Any): ST3215-compatible bus.
        servo_id (int): Servo ID.
        target (int): Wanted present_position.
        tolerance (int): Allowed |present - target|.
        sleep (Callable[[float], Any]): Sleep function (seconds).

    Returns:
        int | None: The last reading (in tolerance when verified), None when every read failed.
    """
    last = None
    for attempt in range(VERIFY_READ_ATTEMPTS):
        reading = read_register(servo, servo_id, get_register_entry_by_name(PRESENT_REGISTER))
        if reading is not None:
            last = reading
            if abs(offset_delta_steps(reading, target)) <= tolerance:
                return reading
        if attempt < VERIFY_READ_ATTEMPTS - 1:
            sleep(VERIFY_RETRY_SLEEP_S)
    return last


def restore_limits(servo: Any, servo_id: int, wanted: tuple[int, int]) -> tuple[int, int] | None:
    """Write the angle limit registers back when they read differently from wanted (as neutral.py after a centre).

    Args:
        servo (Any): ST3215-compatible bus.
        servo_id (int): Servo ID.
        wanted (tuple[int, int]): Limits read before the offset write.

    Returns:
        tuple[int, int] | None: The limits read at the end (None when a read failed).
    """
    limits = read_limits(servo, servo_id)
    if limits == wanted:
        return limits
    servo.UnLockEprom(servo_id)
    try:
        for name, value in ((MIN_LIMIT_REGISTER, wanted[0]), (MAX_LIMIT_REGISTER, wanted[1])):
            write_register(servo, servo_id, get_register_entry_by_name(name), value, {})
    finally:
        servo.LockEprom(servo_id)
    return read_limits(servo, servo_id)


def apply_joint(
    servo: Any, joint: JointConfig, record: JointRecord, tolerance: int, sleep: Callable[[float], Any]
) -> str | None:
    """Write, verify (rolling back on failure) and keep the limits of one joint.

    Args:
        servo (Any): ST3215-compatible bus.
        joint (JointConfig): The joint.
        record (JointRecord): Planned joint; the after fields are filled in place.
        tolerance (int): verify_tolerance_steps.
        sleep (Callable[[float], Any]): Sleep function (seconds).

    Returns:
        str | None: Why the joint failed (the run stops); None when applied and verified.
    """
    refused = torque_refusals(servo, [joint])
    if refused:
        return f"torque not off before the write: {refused[0]}"
    if not write_offset(servo, joint.id, record.new_offset_steps):
        return f"{joint.name}: offset write failed"
    sleep(SETTLE_S)
    present = verify_present(servo, joint.id, record.target_steps, tolerance, sleep)
    record.present_after_steps = present
    record.present_after_rad = None if present is None else steps_to_radians(present, joint.inverted)
    raw_after = read_retried(servo, joint.id, OFFSET_REGISTER)
    record.offset_after_steps = None if raw_after is None else decode_offset(raw_after)
    if present is None or abs(offset_delta_steps(present, record.target_steps)) > tolerance:
        rolled_back = write_offset(servo, joint.id, record.offset_before_steps)
        reading = "read failed" if present is None else f"reads {present}"
        return (
            f"{joint.name}: not verified ({reading}, target {record.target_steps} +- {tolerance}); "
            f"old offset {record.offset_before_steps} {'rewritten' if rolled_back else 'REWRITE FAILED'}"
        )
    record.limits_after = restore_limits(servo, joint.id, record.limits_before)
    if record.limits_after != record.limits_before:
        return f"{joint.name}: offset applied, angle limits not restored: {record.limits_after}"
    record.applied = True
    return None


def write_record(path_template: str, now: str, payload: dict[str, Any]) -> Path:
    """Write the JSON record.

    Args:
        path_template (str): record_path with {timestamp}.
        now (str): UTC timestamp text.
        payload (dict[str, Any]): Record content.

    Returns:
        Path: The written file.
    """
    path = Path(path_template.format(timestamp=now))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n")
    return path


def log_table(records: dict[str, JointRecord]) -> None:
    """Log the per-joint table.

    Args:
        records (dict[str, JointRecord]): joint name -> record.
    """
    LOG.info(
        "%-14s %4s %8s %9s %7s %6s %7s %6s %6s %7s %7s",
        "joint", "id", "ref_rad", "now_rad", "present", "target", "offset", "delta", "expect", "new_ofs", "after",
    )  # fmt: skip
    for name, rec in records.items():
        LOG.info(
            "%-14s %4d %8.3f %9s %7s %6d %7s %6s %6s %7s %7s",
            name,
            rec.id,
            rec.reference_rad,
            "-" if rec.present_before_rad is None else f"{rec.present_before_rad:.3f}",
            rec.present_before_steps,
            rec.target_steps,
            rec.offset_before_steps,
            "-" if rec.delta_steps is None else f"{rec.delta_steps:+d}",
            "-" if rec.expected_delta_steps is None else f"{rec.expected_delta_steps:+d}",
            rec.new_offset_steps,
            "-" if rec.present_after_steps is None else rec.present_after_steps,
        )


def run_restore(
    config: RestoreConfig,
    servo: Any,
    apply: bool,
    now: Callable[[], str],
    sleep: Callable[[float], Any] = time.sleep,
) -> int:
    """Check, plan and (with apply) write the offsets; always writes the JSON record.

    Args:
        config (RestoreConfig): Restore config.
        servo (Any): ST3215-compatible bus (the follower bus, bridge stopped).
        apply (bool): Write the offsets; False is a dry run (reads only).
        now (Callable[[], str]): UTC timestamp text for the record name.
        sleep (Callable[[float], Any]): Sleep function (seconds).

    Returns:
        int: EXIT_OK when the plan is valid (dry run) or every joint applied and verified; else EXIT_FAIL.
    """
    tolerance_steps = degrees_to_steps(config.tolerance_deg)
    records = {
        j.name: JointRecord(j.id, j.reference_rad, position_to_raw_steps(j.reference_rad, j.inverted))
        for j in config.joints
    }
    result, message = RESULT_OK, ""
    refused = torque_refusals(servo, config.joints)
    if refused:
        result, message = RESULT_REFUSED, f"torque not off: {', '.join(refused)}; nothing written"
    else:
        problems = [p for j in config.joints if (p := plan_joint(servo, j, tolerance_steps, records[j.name]))]
        if problems:
            result, message = RESULT_REFUSED, "; ".join(problems) + "; nothing written"
        elif not apply:
            message = "dry run: plan valid, nothing written; run again with --apply to write"
        else:
            for joint in config.joints:
                failure = apply_joint(servo, joint, records[joint.name], config.verify_tolerance_steps, sleep)
                if failure is not None:
                    applied = [n for n, r in records.items() if r.applied]
                    result = RESULT_FAILED
                    message = f"{failure}; applied: {', '.join(applied) or 'none'}; later joints not attempted"
                    break
            else:
                message = "every joint applied and verified; torque stays off"
    log_table(records)
    payload = {
        "created_at": now(),
        "mode": MODE_APPLY if apply else MODE_DRY_RUN,
        "device": config.device,
        "result": result,
        "message": message,
        "joints": {name: asdict(rec) for name, rec in records.items()},
    }
    path = write_record(config.record_path, payload["created_at"], payload)
    (LOG.info if result == RESULT_OK else LOG.error)("%s: %s (record %s)", result, message, path)
    return EXIT_OK if result == RESULT_OK else EXIT_FAIL


def utc_timestamp() -> str:
    """Return the current UTC time as YYYYmmddTHHMMSSZ.

    Returns:
        str: e.g. "20261109T120000Z".
    """
    return datetime.now(UTC).strftime(TIMESTAMP_FORMAT)


def service_active(name: str) -> bool:
    """Return True when a systemd unit is active (systemctl is-active exits 0).

    Args:
        name (str): Unit name.

    Returns:
        bool: True when active.
    """
    return subprocess.run(["systemctl", "is-active", "--quiet", name], check=False).returncode == 0


def main(argv: list[str] | None = None) -> int:
    """CLI entry point: --config (default next to this script), --apply; refuses while the bridge service runs.

    Args:
        argv (list[str] | None): Arguments; None reads sys.argv.

    Returns:
        int: Exit code.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="restore config YAML")
    parser.add_argument("--apply", action="store_true", help="write the offsets (default: dry run, reads only)")
    args = parser.parse_args(argv)
    logging.basicConfig(stream=sys.stdout, level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    config = load_config(args.config)
    if service_active(config.bridge_service):
        LOG.error("%s is active and owns the bus: stop it first", config.bridge_service)
        return EXIT_FAIL
    servo = ST3215(config.device)
    return run_restore(config, servo, args.apply, utc_timestamp)


if __name__ == "__main__":
    sys.exit(main())
