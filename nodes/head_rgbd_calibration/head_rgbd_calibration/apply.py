"""Apply a passing result to the robot and package a result for download (design D6).

The applied files live in <state_dir>/applied/ and are what the head nodes and the Ansible path unit read; the unit
restart is triggered outside this node, which only reads unit state with an unprivileged systemctl show.
"""

import io
import json
import logging
import os
import subprocess
import zipfile
from collections.abc import Callable, Sequence
from datetime import datetime
from typing import Any

from head_rgbd_calibration.compute import (
    EXTRINSICS_FILE,
    REPORT_FILE,
    RGB_CAMERA_FILE,
    TOF_CAMERA_FILE,
    TOF_JOINT_FILE,
    URDF_FILE,
)
from head_rgbd_calibration.store import RESULT_FILE, TEMP_PREFIX, SessionStore, write_json_atomic

LOGGER = logging.getLogger(__name__)

APPLIED_FILES = (RGB_CAMERA_FILE, TOF_CAMERA_FILE, EXTRINSICS_FILE, TOF_JOINT_FILE)
APPLIED_MARKER = "applied.json"
APPLIED_DIR = "applied"
YAML_FILES = (RGB_CAMERA_FILE, TOF_CAMERA_FILE, EXTRINSICS_FILE)
DOWNLOAD_FILES = (*YAML_FILES, TOF_JOINT_FILE, URDF_FILE, REPORT_FILE)
SYSTEMCTL_SHOW = ["systemctl", "show", "--timestamp=unix", "-p", "ActiveEnterTimestamp", "-p", "ActiveState"]
SYSTEMCTL_TIMEOUT_S = 5.0

RunFn = Callable[..., subprocess.CompletedProcess]


class NotApplicable(ValueError):
    """The result cannot be applied or downloaded (maps to 409)."""


def apply_result(store: SessionStore, sid: str, rid: str, now: datetime) -> dict[str, Any]:
    """Copy a passing result into <state_dir>/applied/ atomically, writing applied.json last.

    Args:
        store: The session store.
        sid: Session id.
        rid: Result id.
        now: Apply time, recorded as applied_at.

    Returns:
        The applied.json content (session, result, applied_at, files).

    Raises:
        NotApplicable: result.json is missing or does not say passed: true, or a file to apply is missing.
        NotFound, InvalidId: from the store for unknown or malformed ids.
    """
    source = store.result_dir(sid, rid)
    result_path = source / RESULT_FILE
    if not result_path.is_file() or json.loads(result_path.read_text()).get("passed") is not True:
        raise NotApplicable(f"result {rid} did not pass and cannot be applied")
    missing = [name for name in APPLIED_FILES if not (source / name).is_file()]
    if missing:
        raise NotApplicable(f"result {rid} lacks {', '.join(missing)}")
    with store.lock:
        target = store.state_dir / APPLIED_DIR
        target.mkdir(parents=True, exist_ok=True)
        try:
            for name in APPLIED_FILES:
                tmp = target / f"{TEMP_PREFIX}{name}"
                tmp.write_bytes((source / name).read_bytes())
                os.replace(tmp, target / name)
            info = {"session": sid, "result": rid, "applied_at": now.isoformat(), "files": list(APPLIED_FILES)}
            write_json_atomic(target / APPLIED_MARKER, info)
        finally:
            for leftover in target.glob(f"{TEMP_PREFIX}*"):
                leftover.unlink(missing_ok=True)
    return info


def unit_state(unit: str, applied_at: float, run: RunFn) -> dict[str, Any]:
    """State of one systemd unit.

    Args:
        unit: Unit name.
        applied_at: Apply time as a unix timestamp.
        run: subprocess.run-compatible callable.

    Returns:
        {name, active_state, restarted}; restarted is True when ActiveEnterTimestamp is not older than applied_at.
    """
    proc = run([*SYSTEMCTL_SHOW, unit], capture_output=True, text=True, check=False, timeout=SYSTEMCTL_TIMEOUT_S)
    props = dict(line.split("=", 1) for line in proc.stdout.splitlines() if "=" in line)
    state = props.get("ActiveState", "unknown")
    stamp = props.get("ActiveEnterTimestamp", "").strip().lstrip("@")
    try:
        restarted = float(stamp) >= applied_at
    except ValueError:
        restarted = False
    return {"name": unit, "active_state": state, "restarted": restarted}


def applied_state(store: SessionStore, units: Sequence[str], run: RunFn = subprocess.run) -> dict[str, Any] | None:
    """The applied.json content plus per-unit state, None when nothing is applied.

    Args:
        store: The session store.
        units: Systemd unit names to report.
        run: subprocess.run-compatible callable (injectable for tests).

    Returns:
        applied.json fields plus units: [{name, active_state, restarted}].
    """
    marker = store.state_dir / APPLIED_DIR / APPLIED_MARKER
    if not marker.is_file():
        return None
    info = json.loads(marker.read_text())
    applied_at = datetime.fromisoformat(info["applied_at"]).timestamp()
    return {**info, "units": [unit_state(unit, applied_at, run) for unit in units]}


def download_zip(store: SessionStore, sid: str, rid: str) -> tuple[str, bytes]:
    """Package a result's YAMLs, tof_joint.json, head.urdf and report.md.

    Returns:
        (file name <sid>-<rid>.zip, zip bytes).

    Raises:
        NotApplicable: the result holds no YAML files.
    """
    source = store.result_dir(sid, rid)
    if not any((source / name).is_file() for name in YAML_FILES):
        raise NotApplicable(f"result {rid} has no calibration files")
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w", zipfile.ZIP_DEFLATED) as archive:
        for name in DOWNLOAD_FILES:
            if (source / name).is_file():
                archive.write(source / name, name)
    return f"{sid}-{rid}.zip", buffer.getvalue()
