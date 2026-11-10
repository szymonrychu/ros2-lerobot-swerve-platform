"""Servo zero change files: the stale marker and the zero change record (standard library only).

One module owns the format of both files, so every writer of a servo zero change (the bridge's neutral save and zero
restore, and scripts/calibrate_servos.py) writes them the same way:

- The stale marker (a group's `neutral_stale_marker`, e.g. /var/lib/ros2/arm/calibration_stale.json):
  {version, group, saved_at, joints: {name: {before_steps, after_steps}}, stale: [items]}. Items are the calibrations
  taken in the old zero (STALE_ITEMS: joint_offsets, home); mcp_server and web_ui read it, mcp_server clears items.
- The zero change record (ZERO_RECORD_FILE next to the marker): {version, group, changed_at}, replaced on every zero
  change and kept when the marker is cleared. head_rgbd_calibration's Accept and the stale marker clearing refuse when
  it names a change at or after a capture (ros2_common arm_calibration_file.zero_change_refusal).

write_zero_change writes both for one event, the record first (it is the readers' fail-safe if the marker write
fails). record_dir_refusal is the pre-flight check of a writer that may run as another user than the bridge service
(the calibrate_servos CLI). No relative imports: the CLI imports this module from the node directory.
"""

import json
import logging
import os
import tempfile
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

LOGGER = logging.getLogger(__name__)
MARKER_VERSION = 1
STALE_ITEMS = ("joint_offsets", "home")
ZERO_RECORD_FILE = "zero_changed.json"  # next to the stale marker: the time of the last servo zero change
ZERO_RECORD_VERSION = 1


def utc_now_iso() -> str:
    """Return the current UTC time as ISO-8601 with seconds (the saved_at / changed_at format).

    Returns:
        str: e.g. "2026-10-20T12:00:00+00:00".
    """
    return datetime.now(UTC).isoformat(timespec="seconds")


def merge_marker(
    existing: dict[str, Any] | None, group: str, saved_at: str, joints: dict[str, tuple[int, int]]
) -> dict[str, Any]:
    """Build the stale marker content, merging the joints of an existing marker.

    Args:
        existing (dict[str, Any] | None): Parsed existing marker; None or a malformed one starts afresh.
        group (str): Namespace of the saved group.
        saved_at (str): ISO-8601 UTC time of the save.
        joints (dict[str, tuple[int, int]]): joint name -> (before_steps, after_steps) of the joints whose centre changed.

    Returns:
        dict[str, Any]: {"version", "group", "saved_at", "joints", "stale"}; stale is reset to both STALE_ITEMS.
    """
    old_joints = existing.get("joints") if isinstance(existing, dict) else None
    merged = dict(old_joints) if isinstance(old_joints, dict) else {}
    for name, (before, after) in joints.items():
        merged[name] = {"before_steps": before, "after_steps": after}
    return {
        "version": MARKER_VERSION,
        "group": group,
        "saved_at": saved_at,
        "joints": merged,
        "stale": list(STALE_ITEMS),
    }


def write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    """Write JSON atomically: temp file in the same directory (fsynced), then os.replace; parents are created.

    Args:
        path (Path): Destination file.
        payload (dict[str, Any]): JSON content (indented, trailing newline).

    Raises:
        OSError: When the directory, the temp file or the replace fails (the temp file is removed).
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    content = json.dumps(payload, indent=2) + "\n"
    fd, temp_name = tempfile.mkstemp(dir=path.parent, prefix=f".{path.name}.", suffix=".tmp")
    try:
        with os.fdopen(fd, "w") as handle:
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temp_name, path)
    except BaseException:
        Path(temp_name).unlink(missing_ok=True)
        raise


def read_marker_json(path: Path) -> dict[str, Any] | None:
    """Read the stale marker JSON.

    Args:
        path (Path): Marker path.

    Returns:
        dict[str, Any] | None: The parsed marker; None when absent.

    Raises:
        ValueError: The file exists but is unreadable, not JSON or not an object with a "stale" list.
    """
    try:
        text = path.read_text()
    except FileNotFoundError:
        return None
    except OSError as exc:
        raise ValueError(f"stale marker {path} unreadable: {exc}") from exc
    try:
        data = json.loads(text)
    except json.JSONDecodeError as exc:
        raise ValueError(f"stale marker {path} is not valid JSON: {exc}") from exc
    if not isinstance(data, dict) or not isinstance(data.get("stale"), list):
        raise ValueError(f"stale marker {path} has no stale list")
    return data


def read_stale_items(path: Path) -> list[str] | None:
    """Items the stale marker lists (as mcp_server calibration_marker.load_marker reads them).

    Args:
        path (Path): Marker path.

    Returns:
        list[str] | None: The "stale" items; None when there is no marker; every STALE_ITEMS entry for an unreadable
            or invalid marker (a suspect marker is never read as clean).
    """
    try:
        data = read_marker_json(path)
    except ValueError as exc:
        LOGGER.error("%s; treating every item as stale", exc)
        return list(STALE_ITEMS)
    return None if data is None else [str(item) for item in data["stale"]]


def set_stale_items(path: Path, items: list[str], group: str, saved_at: str | None) -> None:
    """Set the marker's stale list atomically, keeping its other fields; delete the marker when the list is empty.

    An absent marker is created ({version, group, saved_at, joints: {}, stale}); an unreadable one is rewritten.

    Args:
        path (Path): Marker path.
        items (list[str]): The new stale items.
        group (str): Group namespace for a new marker.
        saved_at (str | None): ISO-8601 UTC time for a new marker (None: unknown).

    Raises:
        OSError: When the rewrite or the delete fails.
    """
    if not items:
        path.unlink(missing_ok=True)
        return
    try:
        data = read_marker_json(path)
    except ValueError as exc:
        LOGGER.error("%s; rewriting it", exc)
        data = None
    if data is None:
        data = {"version": MARKER_VERSION, "group": group, "saved_at": saved_at, "joints": {}}
    write_json_atomic(path, {**data, "stale": list(items)})


def record_zero_change(marker: Path, group: str, changed_at: str) -> None:
    """Record a servo zero change atomically in ZERO_RECORD_FILE next to the stale marker (replaced on every change).

    Args:
        marker (Path): The group's stale marker path (its directory holds the record).
        group (str): Group namespace.
        changed_at (str): ISO-8601 UTC time of the change.

    Raises:
        OSError: When the write fails.
    """
    record = {"version": ZERO_RECORD_VERSION, "group": group, "changed_at": changed_at}
    write_json_atomic(marker.with_name(ZERO_RECORD_FILE), record)


def write_stale_marker(path: Path, group: str, saved_at: str, joints: dict[str, tuple[int, int]]) -> None:
    """Write (merge) the stale marker atomically: temp file in the same directory, then os.replace.

    Missing parent directories are created. An existing marker that is not valid JSON is replaced.

    Args:
        path (Path): Marker path.
        group (str): Namespace of the saved group.
        saved_at (str): ISO-8601 UTC time of the save.
        joints (dict[str, tuple[int, int]]): joint name -> (before_steps, after_steps).

    Raises:
        OSError: When the directory, the temp file or the replace fails.
    """
    existing = None
    if path.exists():
        try:
            existing = json.loads(path.read_text())
        except json.JSONDecodeError:
            LOGGER.warning("stale marker %s is not valid JSON; replacing it", path)
    write_json_atomic(path, merge_marker(existing, group, saved_at, joints))


def write_zero_change(marker: Path, group: str, changed_at: str, joints: dict[str, tuple[int, int]]) -> list[str]:
    """Record one servo zero change: the zero change record first (the readers' fail-safe), then the stale marker for
    the joints whose zero verifiably changed.

    Called for every event that may have changed a servo zero (a DefineMiddle or offset write was sent, even one that
    failed): the record is always written; the marker only when joints is not empty (a verified change).

    Args:
        marker (Path): The group's stale marker path; the record goes next to it.
        group (str): Group namespace.
        changed_at (str): ISO-8601 UTC time of the change (utc_now_iso).
        joints (dict[str, tuple[int, int]]): joint name -> (before_steps, after_steps) of the verified changes.

    Returns:
        list[str]: "stale marker not written: ..." and / or "zero change record not written: ..." for each failed
            write (in that order); empty when everything was written.
    """
    record_error = marker_error = None
    try:
        record_zero_change(marker, group, changed_at)
    except OSError as exc:
        record_error = f"zero change record not written: {type(exc).__name__}: {exc}"
    if joints:
        try:
            write_stale_marker(marker, group, changed_at, joints)
        except OSError as exc:
            marker_error = f"stale marker not written: {type(exc).__name__}: {exc}"
    return [e for e in (marker_error, record_error) if e is not None]


def record_dir_refusal(marker: Path, euid: int) -> str | None:
    """Why a process running as euid must not write the zero change files of this marker (None when it may).

    The directory (or, while it does not exist, its nearest existing ancestor that write_json_atomic would create it
    in) must be writable by and owned by euid: the files are created mode 0600, so a file written by another user
    (e.g. root through sudo) could not be read or replaced by the bridge service and the readers.

    Args:
        marker (Path): The stale marker path; the record goes in the same directory.
        euid (int): Effective user id of the writer (os.geteuid()).

    Returns:
        str | None: The refusal text naming the directory, its owner and the fix; None when the writer may write.
    """
    directory = marker.parent
    existing = directory
    while not existing.exists() and existing != existing.parent:
        existing = existing.parent
    try:
        owner = existing.stat().st_uid
    except OSError as exc:
        return f"cannot check {existing} for the zero change record: {type(exc).__name__}: {exc}"
    if owner != euid:
        return (
            f"{existing} is owned by uid {owner} but this process runs as uid {euid}: the zero change record and "
            f"stale marker in {directory} would be unwritable or unreadable for the bridge; run it as the owner of "
            f"{existing} (the bridge service user, e.g. `sudo -u <user>`)"
        )
    if not os.access(existing, os.W_OK | os.X_OK):
        return f"{existing} is not writable by uid {euid}: the zero change record in {directory} cannot be written"
    return None
