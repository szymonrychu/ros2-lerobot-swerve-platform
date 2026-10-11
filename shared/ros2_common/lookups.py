"""Camera lookup contract shared by mcp_server (writer) and web_ui (read-only reader).

One definition of the index.jsonl record (LookupRecord), the list filters (LookupQuery, with ISO-8601 or unix time
bounds), the JSON metadata (record_summary) and an index reader that re-parses the file only when it changed
(IndexCache). Index lines that do not parse are skipped and logged once per distinct line.
"""

import hashlib
import json
import logging
import math
import threading
import time
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from pydantic import BaseModel, Field, ValidationError, field_validator

LOGGER = logging.getLogger("ros2_common.lookups")
INDEX_FILE = "index.jsonl"
MAX_QUERY_LIMIT = 20
DEFAULT_QUERY_LIMIT = 10
ISO_TIME_FORMAT = "%Y-%m-%dT%H:%M:%SZ"


class PlanarPose(BaseModel):
    """Robot pose in the map frame when the image was taken."""

    x: float
    y: float
    yaw: float


class CameraPose(BaseModel):
    """Camera body pose in base_link (fixed-axis roll/pitch/yaw, rad)."""

    xyz: list[float] = Field(min_length=3, max_length=3)
    rpy: list[float] = Field(min_length=3, max_length=3)


class LookupRecord(BaseModel):
    """One index.jsonl line: a recorded camera image and where the robot was."""

    id: str
    ts: float = Field(description="Unix time the image was recorded (s)")
    tool: str
    camera: str
    width: int
    height: int
    file: str = Field(description="JPEG file name, relative to the store directory")
    bytes: int
    base_pose_map: PlanarPose | None = None
    arm_joints: dict[str, float] | None = None
    camera_pose_base: CameraPose | None = None
    mcp_session: str | None = None
    note: str = ""
    poi_ids: list[str] = Field(default_factory=list)


class NearFilter(BaseModel):
    """Lookups taken within radius_m of a map point."""

    x: float = Field(description="map x (m)")
    y: float = Field(description="map y (m)")
    radius_m: float = Field(gt=0.0, description="Search radius (m)")


def parse_time(value: Any) -> float | None:
    """Unix seconds from a unix number, a numeric string or an ISO-8601 string (naive means UTC).

    Args:
        value (Any): None, int/float, or str ("1760000000", "2025-10-09T08:53:20Z", "2025-10-09T10:53:20+02:00").

    Returns:
        float | None: Unix seconds, None for None.

    Raises:
        ValueError: The value is neither a number nor an ISO-8601 time.
    """
    if value is None:
        return None
    if isinstance(value, bool):
        raise ValueError("a time must be unix seconds or an ISO-8601 string")
    if isinstance(value, int | float):
        return float(value)
    if not isinstance(value, str):
        raise ValueError("a time must be unix seconds or an ISO-8601 string")
    text = value.strip()
    try:
        return float(text)
    except ValueError:
        pass
    try:
        moment = datetime.fromisoformat(text)
    except ValueError as exc:
        raise ValueError(f"{value!r} is neither unix seconds nor an ISO-8601 time (e.g. 2025-10-09T08:53:20Z)") from exc
    return (moment if moment.tzinfo else moment.replace(tzinfo=UTC)).timestamp()


class LookupQuery(BaseModel):
    """list_lookups filters; every given filter must match (AND). since / until take unix seconds or ISO-8601."""

    since: float | None = None
    until: float | None = None
    near: NearFilter | None = None
    poi_id: str | None = None
    text: str | None = None
    camera: str | None = None
    mcp_session: str | None = None
    limit: int = Field(default=DEFAULT_QUERY_LIMIT, ge=1, le=MAX_QUERY_LIMIT)

    @field_validator("since", "until", mode="before")
    @classmethod
    def time_bound(cls, value: Any) -> float | None:
        """Accept unix seconds or an ISO-8601 string.

        Args:
            value (Any): Raw bound.

        Returns:
            float | None: Unix seconds.
        """
        return parse_time(value)

    def matches(self, rec: LookupRecord) -> bool:
        """Whether a record passes every filter of this query.

        Args:
            rec (LookupRecord): Candidate record.

        Returns:
            bool: True when it matches.
        """
        if self.since is not None and rec.ts < self.since:
            return False
        if self.until is not None and rec.ts > self.until:
            return False
        if self.camera is not None and rec.camera != self.camera:
            return False
        if self.mcp_session is not None and rec.mcp_session != self.mcp_session:
            return False
        if self.poi_id is not None and self.poi_id not in rec.poi_ids:
            return False
        if self.text is not None and self.text.casefold() not in rec.note.casefold():
            return False
        if self.near is not None:
            pose = rec.base_pose_map
            if pose is None or math.hypot(pose.x - self.near.x, pose.y - self.near.y) > self.near.radius_m:
                return False
        return True


def parse_line(line: str) -> LookupRecord | None:
    """Parse one index line.

    Args:
        line (str): Raw line (without the newline).

    Returns:
        LookupRecord | None: The record, None for a blank or corrupt line.
    """
    if not line.strip():
        return None
    try:
        return LookupRecord.model_validate_json(line)
    except ValidationError:
        return None


def record_summary(rec: LookupRecord) -> dict[str, object]:
    """JSON metadata of a record: the record plus an ISO time.

    Args:
        rec (LookupRecord): Record.

    Returns:
        dict[str, object]: Metadata.
    """
    data: dict[str, object] = json.loads(rec.model_dump_json())
    data["time"] = time.strftime(ISO_TIME_FORMAT, time.gmtime(rec.ts))
    return data


class IndexCache:
    """Parsed index.jsonl, re-read only when the file changed (inode, mtime, size); thread-safe.

    Attributes:
        path (Path): The index file.
    """

    def __init__(self, path: Path) -> None:
        """Create an empty cache.

        Args:
            path (Path): The index file (may not exist yet).
        """
        self.path = path
        self.lock = threading.Lock()
        self.key: tuple[int, int, int] | None = None
        self.parsed: list[tuple[str, LookupRecord | None]] = []
        self.warned: set[str] = set()

    def stat_key(self) -> tuple[int, int, int] | None:
        """Identity of the file as it is now.

        Returns:
            tuple[int, int, int] | None: (inode, mtime_ns, size), None when missing.
        """
        try:
            st = self.path.stat()
        except FileNotFoundError:
            return None
        return st.st_ino, st.st_mtime_ns, st.st_size

    def refresh(self) -> None:
        """Re-read the file when it changed (caller holds the lock)."""
        key = self.stat_key()
        if key is not None and key == self.key:
            return
        if key is None:
            self.key, self.parsed = None, []
            return
        try:
            lines = self.path.read_text().splitlines()
        except FileNotFoundError:
            self.key, self.parsed = None, []
            return
        parsed: list[tuple[str, LookupRecord | None]] = []
        for number, line in enumerate(lines, start=1):
            rec = parse_line(line)
            parsed.append((line, rec))
            if rec is None and line.strip():
                digest = hashlib.sha1(line.encode(), usedforsecurity=False).hexdigest()
                if digest not in self.warned:
                    self.warned.add(digest)
                    LOGGER.warning("lookup index %s line %d is corrupt, skipped", self.path, number)
        self.key, self.parsed = key, parsed

    def entries(self) -> list[tuple[str, LookupRecord | None]]:
        """Every index line with its parsed record (None for blank and corrupt lines).

        Returns:
            list[tuple[str, LookupRecord | None]]: (raw line, record) in file order.
        """
        with self.lock:
            self.refresh()
            return list(self.parsed)

    def records(self) -> list[LookupRecord]:
        """Every valid record, oldest first.

        Returns:
            list[LookupRecord]: Records.
        """
        return [rec for _, rec in self.entries() if rec is not None]

    def appended(self, rec: LookupRecord) -> None:
        """Account for a line this process just appended after a fresh entries() call (saves a re-parse).

        Args:
            rec (LookupRecord): The record whose line was appended.
        """
        with self.lock:
            self.parsed.append((rec.model_dump_json(), rec))
            self.key = self.stat_key()

    def invalidate(self) -> None:
        """Forget the cached parse (the next read re-parses)."""
        with self.lock:
            self.key = None
