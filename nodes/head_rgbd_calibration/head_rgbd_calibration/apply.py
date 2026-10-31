"""Accept a passed D435 intrinsics check and package a result for download (design D6, D10).

Accepting records the check in <state_dir>/applied/applied.json and copies no file into applied/: the D435 factory
intrinsics stay the ones every consumer reads. The records other applies keep in applied.json (the head chain) are
left as they are. Every writer of applied.json (this accept, the head chain applies, the CSI applies) goes through
update_applied, one lock for the read-modify-write. A unit restart is triggered outside this node, which only reads unit state with an unprivileged
systemctl show.
"""

import io
import json
import logging
import subprocess
import threading
import zipfile
from collections.abc import Callable, Sequence
from datetime import datetime
from pathlib import Path
from typing import Any

from head_rgbd_calibration.compute import REPORT_FILE
from head_rgbd_calibration.store import (
    KIND_INTRINSICS,
    RESULT_FILE,
    SessionStore,
    write_bytes_atomic,
    write_json_atomic,
)

LOGGER = logging.getLogger(__name__)

APPLIED_MARKER = "applied.json"
APPLIED_DIR = "applied"
# applied.json keys of the accepted check; "files" was the RGB + ToF calibration's list of copied files.
CHECK_RECORD_KEYS = ("kind", "session", "result", "applied_at", "files")
# applied.json key of the CSI camera records (csi_intrinsics.py): camera name -> record; they accept no D435 check.
CSI_APPLIED_KEY = "csi_intrinsics"
# The reviewed client.yml offsets snippet of an arm result; downloaded, never copied into applied/.
SNIPPET_FILE = "joint_offsets_snippet.yaml"
DOWNLOAD_FILES = (REPORT_FILE, RESULT_FILE, SNIPPET_FILE)
SYSTEMCTL_SHOW = ["systemctl", "show", "--timestamp=unix", "-p", "ActiveEnterTimestamp", "-p", "ActiveState"]
SYSTEMCTL_TIMEOUT_S = 5.0

RunFn = Callable[..., subprocess.CompletedProcess]
# The one lock of every applied.json read-modify-write in the process (update_applied).
APPLIED_LOCK = threading.Lock()


class NotApplicable(ValueError):
    """The result cannot be applied or downloaded (maps to 409)."""


def apply_result(store: SessionStore, sid: str, rid: str, now: datetime) -> dict[str, Any]:
    """Record a passed intrinsics check in applied.json (atomically); no file is copied.

    Args:
        store: The session store.
        sid: Session id.
        rid: Result id.
        now: Accept time, recorded as applied_at.

    Returns:
        The check record: {kind: "intrinsics", session, result, applied_at}.

    Raises:
        NotApplicable: result.json is missing, is not an intrinsics check, or did not pass.
        NotFound, InvalidId: from the store for unknown or malformed ids.
    """
    result_path = store.result_dir(sid, rid) / RESULT_FILE
    if not result_path.is_file():
        raise NotApplicable(f"result {rid} has no result.json")
    result = json.loads(result_path.read_text())
    if result.get("kind") != KIND_INTRINSICS:
        raise NotApplicable(f"result {rid} is not a {KIND_INTRINSICS} check result")
    if result.get("passed") is not True:
        raise NotApplicable(f"result {rid} did not pass and cannot be applied")
    info = {"kind": KIND_INTRINSICS, "session": sid, "result": rid, "applied_at": now.isoformat()}

    def keep_others(kept: dict[str, Any]) -> dict[str, Any]:
        return {**info, **{key: value for key, value in kept.items() if key not in CHECK_RECORD_KEYS}}

    update_applied(store.state_dir / APPLIED_DIR, keep_others)
    return info


def update_applied(
    target: Path, change: Callable[[dict[str, Any]], dict[str, Any]], files: Sequence[tuple[str, bytes]] = ()
) -> dict[str, Any]:
    """Read-modify-write applied/applied.json under APPLIED_LOCK, landing the applied files first.

    Args:
        target: The applied/ directory (created when missing, after change accepted).
        change: Maps the current applied.json content ({} when absent) to the new one; it may raise to refuse, then
            nothing is written.
        files: (name, bytes) written into target atomically, in order, before applied.json (the apply path unit fires
            on applied.json and compares the files).

    Returns:
        dict[str, Any]: The written applied.json content.
    """
    with APPLIED_LOCK:
        marker = target / APPLIED_MARKER
        updated = change(json.loads(marker.read_text()) if marker.is_file() else {})
        target.mkdir(parents=True, exist_ok=True)
        for name, data in files:
            write_bytes_atomic(target / name, data)
        write_json_atomic(marker, updated)
        return updated


def has_check_record(info: dict[str, Any]) -> bool:
    """Whether applied.json holds more than the CSI camera records (the D435 check and what was applied after it).

    Args:
        info: The applied.json content.

    Returns:
        bool: True when a key other than CSI_APPLIED_KEY is present.
    """
    return any(key != CSI_APPLIED_KEY for key in info)


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
    """The applied.json content plus per-unit state, None when nothing but CSI camera records is applied.

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
    if not has_check_record(info):
        return None
    applied_at = datetime.fromisoformat(info["applied_at"]).timestamp()
    return {**info, "units": [unit_state(unit, applied_at, run) for unit in units]}


def download_zip(store: SessionStore, sid: str, rid: str) -> tuple[str, bytes]:
    """Package a result's report.md, result.json and, for an arm result, joint_offsets_snippet.yaml.

    Returns:
        (file name <sid>-<rid>.zip, zip bytes).

    Raises:
        NotApplicable: the result holds no report.
    """
    source = store.result_dir(sid, rid)
    if not (source / REPORT_FILE).is_file():
        raise NotApplicable(f"result {rid} has no report")
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w", zipfile.ZIP_DEFLATED) as archive:
        for name in DOWNLOAD_FILES:
            if (source / name).is_file():
                archive.write(source / name, name)
    return f"{sid}-{rid}.zip", buffer.getvalue()
