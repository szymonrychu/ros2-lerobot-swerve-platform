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


def clear_stale(path: Path, items: list[str]) -> None:
    """Remove items from the marker's stale list, atomically; delete the file when none remain.

    Args:
        path (Path): Marker file; absent is a no-op.
        items (list[str]): Items to clear ("joint_offsets", "home").
    """
    marker = load_marker(path)
    if marker is None:
        return
    marker.stale = [item for item in marker.stale if item not in items]
    if not marker.stale:
        path.unlink(missing_ok=True)
        return
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=".stale-", suffix=".json")
    try:
        with os.fdopen(fd, "w") as fh:
            json.dump(marker.model_dump(), fh, indent=2)
        os.replace(tmp, path)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise
