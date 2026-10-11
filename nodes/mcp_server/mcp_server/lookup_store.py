"""Camera lookup store: every camera image the agent received, as <id>.jpg files plus one index.jsonl line each.

The store lives next to poi_store's poi.json (/var/lib/ros2/poi/lookups by default) so lookups can be linked to POI
ids, but poi_store never reads it. The record, query and index-cache types are shared with web_ui
(ros2_common.lookups). Retention (record count, total JPEG bytes, age) is enforced oldest first once a limit is
exceeded by PRUNE_SLACK; the record just written is never pruned. Index lines that do not parse are skipped (logged
once) when reading and kept untouched when annotate rewrites another line; a prune rewrite drops them.
"""

import logging
import os
import threading
import time
import uuid
from collections.abc import Callable
from pathlib import Path

from ros2_common.lookups import (
    DEFAULT_QUERY_LIMIT,
    INDEX_FILE,
    MAX_QUERY_LIMIT,
    CameraPose,
    IndexCache,
    LookupQuery,
    LookupRecord,
    NearFilter,
    PlanarPose,
    record_summary,
)
from ros2_common.photos import FILE_MODE, write_atomic

__all__ = [
    "DEFAULT_QUERY_LIMIT",
    "INDEX_FILE",
    "MAX_QUERY_LIMIT",
    "LookupNotFoundError",
    "LookupQuery",
    "LookupRecord",
    "LookupStore",
    "NearFilter",
    "record_summary",
    "write_atomic",
]

LOGGER = logging.getLogger("mcp_server.lookups")
DAY_S = 86400.0
# Pruning rewrites the whole index, so it only starts once a limit is exceeded by this fraction and then goes back to
# the limit: at the limit a camera call no longer rewrites ~1 MB on the SD card every time.
PRUNE_SLACK = 0.05
# A JPEG without an index line younger than this may still be between its write and its index append.
ORPHAN_MIN_AGE_S = 60.0


class LookupNotFoundError(KeyError):
    """No lookup with the given id."""


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
        self.cache = IndexCache(root / INDEX_FILE)
        self.swept = False

    @property
    def index_path(self) -> Path:
        """Path of index.jsonl.

        Returns:
            Path: The index file.
        """
        return self.root / INDEX_FILE

    def records(self) -> list[LookupRecord]:
        """Every valid record, oldest first (the index is re-parsed only when it changed; corrupt lines are logged
        once and skipped).

        Returns:
            list[LookupRecord]: Records.
        """
        return self.cache.records()

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
                self.append_line(rec)
            except BaseException:
                image.unlink(missing_ok=True)
                raise
            self.prune(rec.id)
        return rec

    def append_line(self, rec: LookupRecord) -> None:
        """Append one index line, first terminating a trailing line a crash left without newline; fsynced (caller
        holds the lock).

        Args:
            rec (LookupRecord): Record to append.
        """
        fd = os.open(self.index_path, os.O_RDWR | os.O_APPEND | os.O_CREAT, FILE_MODE)
        with os.fdopen(fd, "r+b") as fh:
            os.fchmod(fh.fileno(), FILE_MODE)
            if fh.seek(0, os.SEEK_END) > 0:
                fh.seek(-1, os.SEEK_END)
                if fh.read(1) != b"\n":
                    fh.write(b"\n")
                    fh.flush()
            self.cache.entries()  # bring the cache up to date with the repaired file before the append
            fh.write((rec.model_dump_json() + "\n").encode())
            fh.flush()
            os.fsync(fh.fileno())
            self.cache.appended(rec)

    def prune(self, keep_id: str) -> None:
        """Drop the oldest records until count, bytes and age are within the limits (caller holds the lock).

        Nothing is rewritten until a limit is exceeded by PRUNE_SLACK; then records are dropped back to the limit
        itself. Image files without an index line (crash between the JPEG and the index append) are removed on the
        first call of a process and with every rewrite.

        Args:
            keep_id (str): Record that is never dropped (the one just written).
        """
        recs = self.records()
        now = self.clock()
        count = len(recs)
        total = sum(r.bytes for r in recs)
        slack = 1.0 + PRUNE_SLACK
        oldest_age = max((now - r.ts for r in recs), default=0.0)
        if count > self.max_records * slack or total > self.max_bytes * slack or oldest_age > self.max_age_s * slack:
            self.drop_over_limit(recs, keep_id, now)
        elif not self.swept:
            self.remove_orphans({r.file for r in recs})
        self.swept = True

    def drop_over_limit(self, recs: list[LookupRecord], keep_id: str, now: float) -> None:
        """Rewrite the index without the oldest records that bring count, bytes and age back to the limits.

        Args:
            recs (list[LookupRecord]): Every valid record.
            keep_id (str): Record that is never dropped.
            now (float): Current unix time (s).
        """
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
        kept = [r for r in recs if r.id not in dropped]
        if dropped:
            write_atomic(self.index_path, "".join(r.model_dump_json() + "\n" for r in kept).encode())
            self.cache.invalidate()
            for rec in recs:
                if rec.id in dropped:
                    (self.root / rec.file).unlink(missing_ok=True)
            LOGGER.info("lookup store pruned %d record(s), %d kept (%d bytes)", len(dropped), count, total)
        self.remove_orphans({r.file for r in kept})

    def remove_orphans(self, indexed_files: set[str]) -> None:
        """Delete JPEGs in the store that no index line names (skipping ones younger than ORPHAN_MIN_AGE_S).

        Args:
            indexed_files (set[str]): File names that stay.
        """
        cutoff = time.time() - ORPHAN_MIN_AGE_S
        for path in self.root.glob("*.jpg"):
            try:
                if path.name not in indexed_files and path.stat().st_mtime < cutoff:
                    path.unlink(missing_ok=True)
                    LOGGER.info("lookup store removed orphan image %s", path.name)
            except FileNotFoundError:
                continue

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
            entries = self.cache.entries()
            for index, (_, rec) in enumerate(entries):
                if rec is None or rec.id != lookup_id:
                    continue
                update: dict[str, object] = {}
                if note is not None:
                    update["note"] = note
                if poi_ids is not None:
                    update["poi_ids"] = list(dict.fromkeys(poi_ids))
                updated = rec.model_copy(update=update)
                if updated == rec:
                    return rec
                lines = [line for line, _ in entries]
                lines[index] = updated.model_dump_json()
                write_atomic(self.index_path, "".join(f"{text}\n" for text in lines).encode())
                self.cache.invalidate()
                return updated
        raise LookupNotFoundError(lookup_id)
