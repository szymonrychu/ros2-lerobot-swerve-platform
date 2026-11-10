"""Servo calibration record files and arm bookkeeping (spec 3.5 and 3.6): the audit record, the latest record and the
rest reference conversion into the new zero frame.

Record files: `<record_dir>/servo_calibration-<YYYYmmddTHHMMSSZ>-<run_id>.json` (every run) and, on full success only,
`<record_dir>/servo_calibration-<namespace>-latest.json`. No rclpy, no bus.
"""

import json
import logging
import math
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from feetech_mapping.command_mapping import STEPS_PER_REVOLUTION, position_to_raw_steps, steps_to_radians

from .calibration_plan import JointCalibrationPlan
from .zero_record import write_json_atomic

LOGGER = logging.getLogger(__name__)
RECORD_VERSION = 1
RECORD_PREFIX = "servo_calibration"
STAMP_FORMAT = "%Y%m%dT%H%M%SZ"
SAVED_AT_FIELD = "saved_at"


def record_payload(
    namespace: str,
    run_id: str,
    plan_run_id: str,
    created_at: str,
    confirmed: bool,
    joints: dict[str, dict[str, Any]],
    failure: tuple[str, str] | None,
    not_attempted: list[str],
    home_rad: dict[str, float],
    override_notes: list[str],
) -> dict[str, Any]:
    """Build the audit record content of one servo calibration store.

    Args:
        namespace (str): Group namespace.
        run_id (str): Id of the store run.
        plan_run_id (str): Id of the plan the store applied.
        created_at (str): ISO-8601 UTC time of the record (also its changed_at).
        confirmed (bool): The store was the confirmed variant.
        joints (dict[str, dict[str, Any]]): joint name -> per-joint outcome.
        failure (tuple[str, str] | None): (joint, reason) of the first failure; None on full success.
        not_attempted (list[str]): Joints not reached after a failure.
        home_rad (dict[str, float]): joint name -> recorded home reading in rad (the gripper reading, spec 3.6).
        override_notes (list[str]): Joints whose config command override is wider than the new hardware limits.

    Returns:
        dict[str, Any]: The record; success is true exactly when there is no failure.
    """
    return {
        "version": RECORD_VERSION,
        "group": namespace,
        "run_id": run_id,
        "plan_run_id": plan_run_id,
        "created_at": created_at,
        "changed_at": created_at,
        "confirmed": confirmed,
        "success": failure is None,
        "failure": list(failure) if failure is not None else None,
        "not_attempted": list(not_attempted),
        "joints": joints,
        "home_rad": home_rad,
        "override_notes": list(override_notes),
    }


def audit_path(record_dir: Path, stamp: datetime, run_id: str) -> Path:
    """Path of the audit record of one run.

    Args:
        record_dir (Path): Directory of the record files.
        stamp (datetime): Time of the run (named in UTC).
        run_id (str): Id of the run.

    Returns:
        Path: `<record_dir>/servo_calibration-<YYYYmmddTHHMMSSZ>-<run_id>.json`.
    """
    return record_dir / f"{RECORD_PREFIX}-{stamp.astimezone(UTC).strftime(STAMP_FORMAT)}-{run_id}.json"


def write_records(record_dir: Path, namespace: str, stamp: datetime, payload: dict[str, Any]) -> list[str]:
    """Write the audit record and, on full success, the latest record.

    Args:
        record_dir (Path): Directory of the record files (created when missing).
        namespace (str): Group namespace (names the latest file).
        stamp (datetime): Time of the run; the audit file name carries it in UTC.
        payload (dict[str, Any]): record_payload content (run_id and success are read from it).

    Returns:
        list[str]: One "... not written: ..." text per failed write; empty when everything was written.
    """
    errors: list[str] = []
    targets = [audit_path(record_dir, stamp, payload["run_id"])]
    if payload["success"]:
        targets.append(record_dir / f"{RECORD_PREFIX}-{namespace}-latest.json")
    for path in targets:
        try:
            write_json_atomic(path, payload)
        except OSError as exc:
            errors.append(f"calibration record {path} not written: {type(exc).__name__}: {exc}")
    return errors


def convert_rest_reference(path: Path, changes: dict[str, tuple[int, bool]], saved_at: str) -> str | None:
    """Rewrite the arm rest reference into the new zero frame after offset changes.

    Each changed joint's steps become (steps - change) % 4096 and its rad the matching reading; other joints and
    other top-level fields are kept; the saved_at field is set to saved_at.

    Args:
        path (Path): Rest reference file (zero_restore format: version, group, saved_at, joints rad, steps).
        changes (dict[str, tuple[int, bool]]): joint name -> (offset_change_steps, inverted).
        saved_at (str): ISO-8601 UTC time of the conversion.

    Returns:
        str | None: Error text; None on success or when the file is absent (nothing is written).
    """
    try:
        data = json.loads(path.read_text())
    except FileNotFoundError:
        return None
    except (OSError, json.JSONDecodeError) as exc:
        return f"rest reference {path} unreadable: {type(exc).__name__}: {exc}"
    joints = data.get("joints") if isinstance(data, dict) else None
    if not isinstance(joints, dict):
        return f"rest reference {path} has no joints"
    steps = data.get("steps")
    steps = dict(steps) if isinstance(steps, dict) else {}
    new_joints = dict(joints)
    new_steps = dict(steps)
    for name, (change, inverted) in changes.items():
        rad = joints.get(name)
        if isinstance(rad, bool) or not isinstance(rad, int | float) or not math.isfinite(rad):
            continue  # not a restore joint of this file (e.g. the gripper): nothing to convert
        old = steps.get(name)
        if isinstance(old, bool) or not isinstance(old, int):
            old = position_to_raw_steps(float(rad), inverted)
        converted = (old - change) % STEPS_PER_REVOLUTION
        new_steps[name] = converted
        new_joints[name] = steps_to_radians(converted, inverted)
    converted_data = {**data, "joints": new_joints, SAVED_AT_FIELD: saved_at}
    if new_steps:
        converted_data["steps"] = new_steps
    try:
        write_json_atomic(path, converted_data)
    except OSError as exc:
        return f"rest reference {path} not written: {type(exc).__name__}: {exc}"
    return None


def home_rad(plans: dict[str, JointCalibrationPlan]) -> dict[str, float]:
    """Return the recorded home readings in rad from the plans' target steps (inverted-aware).

    Args:
        plans (dict[str, JointCalibrationPlan]): joint name -> plan; target_steps equals home_steps for the joints
            excluded from the offset change (the gripper).

    Returns:
        dict[str, float]: joint name -> reading in rad.
    """
    return {name: steps_to_radians(plan.target_steps, plan.inverted) for name, plan in plans.items()}
