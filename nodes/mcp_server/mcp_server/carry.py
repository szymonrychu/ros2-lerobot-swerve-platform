"""Carried-object record behind the one carrying signal (ControlState.arm.carrying), persisted next to the home pose.

The record keeps what mcp_server knows about an object in the gripper: the last close held it (grasp_held: a
'grasped' close or an 'unverified' empty-band hold, grasp_kind says which) and the carried_object_radius_m given at
the last keep_position arm release. It is written atomically to <home_file dir>/carry.yaml on every change, so a
restarted mcp_server still refuses to park the arm with the object in the jaws. A missing or unreadable file reads as
nothing carried (logged); a failed write keeps the in-memory record (logged).
"""

import logging
import os
import tempfile
import threading
import time
from collections.abc import Callable
from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, ValidationError

CARRY_FILE_NAME = "carry.yaml"

GraspKind = Literal["grasped", "unverified"]

LOGGER = logging.getLogger(__name__)


class CarryRecord(BaseModel):
    """Persisted carried-object facts (the live jaw and gripper load are read separately)."""

    grasp_held: bool = False  # the last close holds an object and no open succeeded since
    grasp_kind: GraspKind | None = None  # grasped (verified contact) or unverified (empty-band hold)
    radius_m: float | None = None  # carried_object_radius_m of the last keep_position arm release (> 0)
    since: float | None = None  # wall clock (s since the epoch) of the last change


def carry_path(home_file: Path) -> Path:
    """Carry record file next to the home pose file.

    Args:
        home_file (Path): arm.home_file.

    Returns:
        Path: <home_file dir>/carry.yaml.
    """
    return home_file.parent / CARRY_FILE_NAME


class CarryStore:
    """Thread-safe carried-object record with atomic YAML persistence."""

    def __init__(self, path: Path, wall: Callable[[], float] = time.time) -> None:
        """Load the record (nothing carried when the file is missing or unreadable).

        Args:
            path (Path): Record file.
            wall (Callable[[], float]): Wall clock for `since`.
        """
        self.path = path
        self.wall = wall
        self.lock = threading.Lock()
        self.record = self.load()

    def load(self) -> CarryRecord:
        """Read the record file.

        Returns:
            CarryRecord: The stored record, or an empty one when missing or unreadable.
        """
        if not self.path.is_file():
            return CarryRecord()
        try:
            return CarryRecord.model_validate(yaml.safe_load(self.path.read_text()) or {})
        except (OSError, yaml.YAMLError, ValidationError) as exc:
            LOGGER.warning("carry record %s unreadable, treated as nothing carried: %s", self.path, exc)
            return CarryRecord()

    def get(self) -> CarryRecord:
        """Current record.

        Returns:
            CarryRecord: A copy.
        """
        with self.lock:
            return self.record.model_copy()

    def update(self, **changes: object) -> CarryRecord:
        """Change fields and persist when anything changed.

        Args:
            **changes (object): CarryRecord fields to set.

        Returns:
            CarryRecord: The record afterwards.
        """
        with self.lock:
            current = self.record.model_dump()
            merged = current | changes
            if merged == current:
                return self.record.model_copy()
            merged["since"] = self.wall()
            self.record = CarryRecord.model_validate(merged)
            self.save_locked()
            return self.record.model_copy()

    def clear(self) -> CarryRecord:
        """Nothing carried any more (an open, an empty jaw).

        Returns:
            CarryRecord: The empty record.
        """
        return self.update(grasp_held=False, grasp_kind=None, radius_m=None)

    def save_locked(self) -> None:
        """Write the record atomically (temp file + rename); failures are logged (lock held)."""
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            fd, tmp = tempfile.mkstemp(dir=self.path.parent, prefix=".carry-", suffix=".yaml")
            try:
                with os.fdopen(fd, "w") as fh:
                    yaml.safe_dump(self.record.model_dump(), fh, sort_keys=False)
                os.replace(tmp, self.path)
            except BaseException:
                Path(tmp).unlink(missing_ok=True)
                raise
        except OSError as exc:
            LOGGER.warning("carry record not persisted to %s: %s", self.path, exc)
