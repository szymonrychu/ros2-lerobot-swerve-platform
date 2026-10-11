"""Camera lookup store: every camera image the agent received, as <id>.jpg files plus one index.jsonl line each.

The store lives next to poi_store's poi.json (/var/lib/ros2/poi/lookups by default) so lookups can be linked to POI
ids, but poi_store never reads it. Every add prunes oldest first until the retention limits (record count, total JPEG
bytes, age) hold again; the record just written is never pruned. Index lines that do not parse are skipped (logged)
when reading and kept untouched when annotate rewrites another line; a prune rewrite drops them.
"""

import logging
import math
import os
import tempfile
import threading
import time
import uuid
from collections.abc import Callable
from pathlib import Path

from pydantic import BaseModel, Field, ValidationError

LOGGER = logging.getLogger("mcp_server.lookups")
INDEX_FILE = "index.jsonl"
DAY_S = 86400.0
MAX_QUERY_LIMIT = 20
DEFAULT_QUERY_LIMIT = 10


class LookupNotFoundError(KeyError):
    """No lookup with the given id."""


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


class LookupQuery(BaseModel):
    """list_lookups filters; every given filter must match (AND)."""

    since: float | None = None
    until: float | None = None
    near: NearFilter | None = None
    poi_id: str | None = None
    text: str | None = None
    camera: str | None = None
    mcp_session: str | None = None
    limit: int = Field(default=DEFAULT_QUERY_LIMIT, ge=1, le=MAX_QUERY_LIMIT)

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


def write_atomic(path: Path, data: bytes) -> None:
    """Write a file atomically (temp file in the same directory + rename).

    Args:
        path (Path): Target file.
        data (bytes): Content.
    """
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=f".{path.name}-")
    try:
        with os.fdopen(fd, "wb") as fh:
            fh.write(data)
        os.replace(tmp, path)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise


class LookupStore:
    """Thread-safe JPEG + index.jsonl store with oldest-first retention.

    Attributes:
        root (Path): Store directory (created on the first add).
        max_records (int): Most records kept.
        max_bytes (int): Most total JPEG bytes kept.
        max_age_s (float): Oldest record age kept (s).
    """

    def __init__(
        self,
        root: Path,
        max_records: int,
        max_bytes: int,
        max_age_days: float,
        clock: Callable[[], float] = time.time,
    ) -> None:
        """Create the store (nothing is touched on disk until the first add).

        Args:
            root (Path): Store directory.
            max_records (int): Most records kept.
            max_bytes (int): Most total JPEG bytes kept.
            max_age_days (float): Oldest record age kept (days).
            clock (Callable[[], float]): Unix time source (tests pass a fake).
        """
        self.root = root
        self.max_records = max_records
        self.max_bytes = max_bytes
        self.max_age_s = max_age_days * DAY_S
        self.clock = clock
        self.lock = threading.Lock()

    @property
    def index_path(self) -> Path:
        """Path of index.jsonl.

        Returns:
            Path: The index file.
        """
        return self.root / INDEX_FILE

    def read_lines(self) -> list[str]:
        """Raw index lines (no newlines); empty when the index does not exist.

        Returns:
            list[str]: Lines in file order (oldest first).
        """
        try:
            return self.index_path.read_text().splitlines()
        except FileNotFoundError:
            return []

    def records(self) -> list[LookupRecord]:
        """Every valid record, oldest first; corrupt lines are logged and skipped.

        Returns:
            list[LookupRecord]: Records.
        """
        out: list[LookupRecord] = []
        for number, line in enumerate(self.read_lines(), start=1):
            rec = parse_line(line)
            if rec is not None:
                out.append(rec)
            elif line.strip():
                LOGGER.warning("lookup index %s line %d is corrupt, skipped", self.index_path, number)
        return out

    def add(
        self,
        jpeg: bytes,
        tool: str,
        camera: str,
        width: int,
        height: int,
        base_pose_map: dict[str, float] | None = None,
        arm_joints: dict[str, float] | None = None,
        camera_pose_base: dict[str, list[float]] | None = None,
        mcp_session: str | None = None,
    ) -> LookupRecord:
        """Store one image, append its index line, then prune to the retention limits.

        Args:
            jpeg (bytes): Exactly the JPEG the model received.
            tool (str): Camera tool that produced it.
            camera (str): Camera name.
            width (int): Image width (px).
            height (int): Image height (px).
            base_pose_map (dict[str, float] | None): {x, y, yaw} in map, None when unknown.
            arm_joints (dict[str, float] | None): Measured arm joints (rad), None when unknown.
            camera_pose_base (dict[str, list[float]] | None): {xyz, rpy} in base_link, None when unknown.
            mcp_session (str | None): MCP session id of the call.

        Returns:
            LookupRecord: The stored record.
        """
        lookup_id = uuid.uuid4().hex
        rec = LookupRecord(
            id=lookup_id,
            ts=self.clock(),
            tool=tool,
            camera=camera,
            width=width,
            height=height,
            file=f"{lookup_id}.jpg",
            bytes=len(jpeg),
            base_pose_map=None if base_pose_map is None else PlanarPose.model_validate(base_pose_map),
            arm_joints=arm_joints,
            camera_pose_base=None if camera_pose_base is None else CameraPose.model_validate(camera_pose_base),
            mcp_session=mcp_session,
        )
        with self.lock:
            self.root.mkdir(parents=True, exist_ok=True)
            image = self.root / rec.file
            write_atomic(image, jpeg)
            try:
                with self.index_path.open("a") as fh:
                    fh.write(rec.model_dump_json() + "\n")
            except BaseException:
                image.unlink(missing_ok=True)
                raise
            self.prune(rec.id)
        return rec

    def prune(self, keep_id: str) -> None:
        """Drop the oldest records until count, bytes and age are within the limits (caller holds the lock).

        Args:
            keep_id (str): Record that is never dropped (the one just written).
        """
        recs = self.records()
        now = self.clock()
        count = len(recs)
        total = sum(r.bytes for r in recs)
        dropped: set[str] = set()
        for rec in sorted(recs, key=lambda r: r.ts):
            too_old = now - rec.ts > self.max_age_s
            if rec.id == keep_id or not (count > self.max_records or total > self.max_bytes or too_old):
                continue
            dropped.add(rec.id)
            count -= 1
            total -= rec.bytes
        if not dropped:
            return
        kept = [r for r in recs if r.id not in dropped]
        write_atomic(self.index_path, "".join(r.model_dump_json() + "\n" for r in kept).encode())
        for rec in recs:
            if rec.id in dropped:
                (self.root / rec.file).unlink(missing_ok=True)
        LOGGER.info("lookup store pruned %d record(s), %d kept (%d bytes)", len(dropped), count, total)

    def query(self, q: LookupQuery) -> list[LookupRecord]:
        """Records matching a query, newest first, at most q.limit.

        Args:
            q (LookupQuery): Filters and limit.

        Returns:
            list[LookupRecord]: Matches.
        """
        matches = [r for r in self.records() if q.matches(r)]
        matches.sort(key=lambda r: r.ts, reverse=True)
        return matches[: q.limit]

    def get(self, lookup_id: str) -> LookupRecord | None:
        """One record by id.

        Args:
            lookup_id (str): Record id.

        Returns:
            LookupRecord | None: The record, None when unknown.
        """
        return next((r for r in self.records() if r.id == lookup_id), None)

    def image(self, rec: LookupRecord) -> bytes:
        """JPEG bytes of a record.

        Args:
            rec (LookupRecord): Record.

        Returns:
            bytes: The stored JPEG.
        """
        return (self.root / rec.file).read_bytes()

    def annotate(self, lookup_id: str, note: str | None = None, poi_ids: list[str] | None = None) -> LookupRecord:
        """Set a record's note and/or POI links, rewriting only its index line.

        Args:
            lookup_id (str): Record id.
            note (str | None): New note; None keeps the current one.
            poi_ids (list[str] | None): New POI ids (replace the list); None keeps the current ones.

        Returns:
            LookupRecord: The updated record.
        """
        with self.lock:
            lines = self.read_lines()
            for index, line in enumerate(lines):
                rec = parse_line(line)
                if rec is None or rec.id != lookup_id:
                    continue
                update: dict[str, object] = {}
                if note is not None:
                    update["note"] = note
                if poi_ids is not None:
                    update["poi_ids"] = list(dict.fromkeys(poi_ids))
                updated = rec.model_copy(update=update)
                lines[index] = updated.model_dump_json()
                write_atomic(self.index_path, "".join(f"{text}\n" for text in lines).encode())
                return updated
        raise LookupNotFoundError(lookup_id)


def record_summary(rec: LookupRecord) -> dict[str, object]:
    """JSON metadata of a record for the tools (the record plus an ISO time).

    Args:
        rec (LookupRecord): Record.

    Returns:
        dict[str, object]: Metadata.
    """
    data = rec.model_dump(mode="json")
    data["time"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(rec.ts))
    return data
