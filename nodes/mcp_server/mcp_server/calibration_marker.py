"""Arm calibration stale marker written by the follower bridge after a runtime neutral save.

The bridge (feetech_servos neutral.py) writes {version, group, saved_at, joints, stale} whenever a servo centre moved.
Items in "stale" are calibrations taken in the old zero: "joint_offsets" (arm.joint_offsets_rad) and "home" (the
stored home pose). This module reads the marker and clears items once they were re-recorded.
"""

import json
import logging
import os
import tempfile
from pathlib import Path

from pydantic import BaseModel, ConfigDict, ValidationError

LOGGER = logging.getLogger(__name__)

STALE_ITEMS = ("joint_offsets", "home")
SERVO_CALIBRATION_AT_FIELD = "servo_calibration_at"  # marker key: ISO time of the arm servo calibration store


class MarkerJoint(BaseModel):
    """Centre change of one joint in raw servo steps."""

    model_config = ConfigDict(extra="ignore")

    before_steps: int
    after_steps: int


class CalibrationMarker(BaseModel):
    """Content of the stale marker file."""

    model_config = ConfigDict(extra="ignore")

    version: int
    group: str
    saved_at: str | None
    joints: dict[str, MarkerJoint]
    stale: list[str]
    servo_calibration_at: str | None = None  # set by an arm servo calibration store; absent on disk otherwise


def servo_calibration_redo(marker: CalibrationMarker | None) -> bool:
    """Whether joint_offsets is stale because an arm servo calibration store set the arm zero (not an accident).

    Args:
        marker (CalibrationMarker | None): The stale marker, or None when absent.

    Returns:
        bool: True when the marker lists joint_offsets and carries the servo calibration time.
    """
    return marker is not None and "joint_offsets" in marker.stale and isinstance(marker.servo_calibration_at, str)


def unreadable_marker() -> CalibrationMarker:
    """Marker standing in for an unreadable or invalid file: both items stale, no time.

    Returns:
        CalibrationMarker: The stand-in.
    """
    return CalibrationMarker(version=0, group="unknown", saved_at=None, joints={}, stale=list(STALE_ITEMS))


def load_marker(path: Path) -> CalibrationMarker | None:
    """Read the stale marker.

    Args:
        path (Path): Marker file.

    Returns:
        CalibrationMarker | None: None when the file is absent; an unreadable or invalid file is logged and reported
            as both items stale (without a time).
    """
    try:
        text = path.read_text()
    except FileNotFoundError:
        return None
    except OSError as exc:
        LOGGER.error("calibration stale marker %s unreadable: %s", path, exc)
        return unreadable_marker()
    try:
        return CalibrationMarker.model_validate(json.loads(text))
    except (ValueError, ValidationError) as exc:
        LOGGER.error("calibration stale marker %s invalid: %s", path, exc)
        return unreadable_marker()


def clear_stale(path: Path, items: list[str], only_if_saved_at: str | None = None) -> bool:
    """Remove items from the marker's stale list, atomically; delete the file when none remain.

    The servo_calibration_at reason is kept while joint_offsets stays listed and dropped with it (an accepted
    arm-to-camera calibration clears joint_offsets); it is never written as null.

    Args:
        path (Path): Marker file; absent is a no-op.
        items (list[str]): Items to clear ("joint_offsets", "home").
        only_if_saved_at (str | None): Clear only while the marker's saved_at is still this (the re-zero the caller
            checked; a newer re-zero written meanwhile is kept); None clears regardless.

    Returns:
        bool: True when the marker was rewritten or deleted, False when absent or of another re-zero.
    """
    marker = load_marker(path)
    if marker is None or (only_if_saved_at is not None and marker.saved_at != only_if_saved_at):
        return False
    marker.stale = [item for item in marker.stale if item not in items]
    if not marker.stale:
        path.unlink(missing_ok=True)
        return True
    document = marker.model_dump()
    if marker.servo_calibration_at is None or "joint_offsets" not in marker.stale:
        document.pop(SERVO_CALIBRATION_AT_FIELD)
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=".stale-", suffix=".json")
    try:
        with os.fdopen(fd, "w") as fh:
            json.dump(document, fh, indent=2)
        os.replace(tmp, path)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise
    return True
