"""Photo store: photo POIs as <id>.jpg files plus one index.jsonl line each; poi_store is the only index writer.

mcp_server writes the JPEG, then asks poi_store to add it (PhotoAdd); this module validates the file, appends the line
and enforces retention. Retention (record count, total JPEG bytes, age) drops the oldest records once a limit is
exceeded by PRUNE_SLACK and then goes back to the limit itself; the record just added is never dropped. The same
pruning runs from a timer (prune_now) so the age limit holds without camera activity. Index lines that do not parse
are skipped when reading, kept by update/delete rewrites and dropped by prune rewrites. JPEGs no index line names
(a crash between the JPEG write and the add) are swept, once they are ORPHAN_MIN_AGE_S old, on the first prune of the
process and on every index rewrite. The revision moves only when the index really changed.
"""

import logging
import os
import shutil
import threading
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any

from pydantic import ValidationError
from ros2_common.lookups import INDEX_FILE, IndexCache, LookupRecord
from ros2_common.photos import FILE_MODE, PHOTO_KIND, PhotoAdd, PhotoSelector, write_atomic

from .config import PhotoSettings
from .metrics import PHOTO_BYTES, PHOTO_COUNT, PHOTO_PRUNED

LOGGER = logging.getLogger("poi_store.photos")
DAY_S = 86400.0
# Pruning rewrites the whole index, so it only starts once a limit is exceeded by this fraction and then goes back to
# the limit: at the limit a camera call no longer rewrites ~1 MB on the SD card every time.
PRUNE_SLACK = 0.05
# A JPEG without an index line younger than this may still be between its write and its add.
ORPHAN_MIN_AGE_S = 60.0
IMMUTABLE_MESSAGE = "photo capture data is immutable"
EDITABLE_FIELDS = frozenset({"name", "note", "status", "poi_ids"})


class PhotoStoreError(ValueError):
    """A photo add or update was refused (the message says why)."""


class PhotoNotFoundError(KeyError):
    """No photo with the given id."""


class PhotoStore:
    """Thread-safe photo index (index.jsonl) and JPEG directory with oldest-first retention.

    Attributes:
        root (Path): Store directory.
        max_records (int): Most records kept.
        max_bytes (int): Most total JPEG bytes kept.
        max_age_days (float | None): Oldest record age kept (days); None never drops a record for its age.
        max_image_bytes (int): Largest JPEG an add accepts.
        clock (Callable[[], float]): Unix time source.
        revision (int): Bumped on every change of the index.
    """

    def __init__(
        self,
        root: Path,
        max_records: int,
        max_bytes: int,
        max_age_days: float | None,
        max_image_bytes: int,
        clock: Callable[[], float] = time.time,
    ) -> None:
        """Create the store (nothing is touched on disk until the first change).

        Args:
            root (Path): Store directory.
            max_records (int): Most records kept.
            max_bytes (int): Most total JPEG bytes kept.
            max_age_days (float | None): Oldest record age kept (days); None never drops a record for its age.
            max_image_bytes (int): Largest JPEG an add accepts.
            clock (Callable[[], float]): Unix time source (tests pass a fake).
        """
        self.root = root
        self.max_records = max_records
        self.max_bytes = max_bytes
        self.max_age_days = max_age_days
        self.max_image_bytes = max_image_bytes
        self.clock = clock
        self.revision = 0
        self.lock = threading.Lock()
        self.cache = IndexCache(root / INDEX_FILE)
        self.swept = False

    @classmethod
    def from_settings(cls, settings: PhotoSettings, clock: Callable[[], float] = time.time) -> "PhotoStore":
        """Build the store from the node config.

        Args:
            settings (PhotoSettings): photos section of PoiStoreConfig.
            clock (Callable[[], float]): Unix time source.

        Returns:
            PhotoStore: The store.
        """
        return cls(
            Path(settings.store_dir),
            max_records=settings.max_records,
            max_bytes=settings.max_bytes,
            max_age_days=settings.max_age_days,
            max_image_bytes=settings.max_image_bytes,
            clock=clock,
        )

    @property
    def index_path(self) -> Path:
        """Path of index.jsonl.

        Returns:
            Path: The index file.
        """
        return self.root / INDEX_FILE

    def records(self) -> list[LookupRecord]:
        """Every valid record, oldest first in file order (re-parsed only when the file changed).

        Returns:
            list[LookupRecord]: Records.
        """
        return self.cache.records()

    def get(self, photo_id: str) -> LookupRecord | None:
        """One record by id.

        Args:
            photo_id (str): Photo id.

        Returns:
            LookupRecord | None: The record, None when unknown.
        """
        return next((r for r in self.records() if r.id == photo_id), None)

    def __contains__(self, photo_id: object) -> bool:
        """Whether the index holds a photo with this id.

        Args:
            photo_id (object): Candidate id.

        Returns:
            bool: True when known.
        """
        return isinstance(photo_id, str) and self.get(photo_id) is not None

    def add(self, photo: PhotoAdd) -> LookupRecord:
        """Index a JPEG that is already in the store directory, then prune to the retention limits.

        Args:
            photo (PhotoAdd): Validated add payload (file is '<id>.jpg').

        Returns:
            LookupRecord: The stored record (bytes from the file size).

        Raises:
            PhotoStoreError: Duplicate id, or the file is outside the store, missing, not a file or too large.
        """
        with self.lock:
            size = self.image_size(photo.file)
            if self.get(photo.id) is not None:
                raise PhotoStoreError(f"photo {photo.id} already exists")
            data = photo.model_dump()
            data["poi_ids"] = list(dict.fromkeys(photo.poi_ids))
            rec = LookupRecord(**data, bytes=size)
            self.append_line(rec)
            self.revision += 1
            self.prune(rec.id)
            self.track()
        return rec

    def image_size(self, file_name: str) -> int:
        """Size of a JPEG that must resolve inside the store directory.

        Args:
            file_name (str): File name relative to root.

        Returns:
            int: File size (bytes).

        Raises:
            PhotoStoreError: Outside the store, missing, not a regular file, or above max_image_bytes.
        """
        root = self.root.resolve()
        path = (self.root / file_name).resolve()
        if path.parent != root:
            raise PhotoStoreError(f"photo file {file_name} resolves outside the photo store {self.root}")
        try:
            st = path.stat()
        except FileNotFoundError:
            raise PhotoStoreError(f"photo file {file_name} does not exist in {self.root}") from None
        if not path.is_file():
            raise PhotoStoreError(f"photo file {file_name} is not a regular file")
        if st.st_size > self.max_image_bytes:
            raise PhotoStoreError(f"photo file {file_name} has {st.st_size} bytes, above max_image_bytes")
        return st.st_size

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

    def prune_now(self) -> int:
        """Apply the retention limits now (the timer path).

        Returns:
            int: Records removed.
        """
        with self.lock:
            if not self.root.is_dir():
                return 0
            removed = self.prune(None)
            if removed:
                self.revision += 1
                self.track()
            return removed

    def prune(self, keep_id: str | None) -> int:
        """Drop the oldest records until count, bytes and age are within the limits (caller holds the lock).

        Nothing is rewritten until a limit is exceeded by PRUNE_SLACK; then records are dropped back to the limit
        itself. Orphan JPEGs are swept on the first call of the process and with every rewrite.

        Args:
            keep_id (str | None): Record that is never dropped (the one just added), None for none.

        Returns:
            int: Records removed.
        """
        recs = self.records()
        now = self.clock()
        max_age_s = None if self.max_age_days is None else self.max_age_days * DAY_S
        total = sum(r.bytes for r in recs)
        slack = 1.0 + PRUNE_SLACK
        oldest_age = max((now - r.ts for r in recs), default=0.0)
        removed = 0
        if (
            len(recs) > self.max_records * slack
            or total > self.max_bytes * slack
            or (max_age_s is not None and oldest_age > max_age_s * slack)
        ):
            removed = self.drop_over_limit(recs, keep_id, now)
        elif not self.swept:
            self.remove_orphans({r.file for r in recs})
        self.swept = True
        return removed

    def drop_over_limit(self, recs: list[LookupRecord], keep_id: str | None, now: float) -> int:
        """Rewrite the index without the oldest records that bring count, bytes and age back to the limits.

        Args:
            recs (list[LookupRecord]): Every valid record.
            keep_id (str | None): Record that is never dropped.
            now (float): Current unix time (s).

        Returns:
            int: Records removed.
        """
        max_age_s = None if self.max_age_days is None else self.max_age_days * DAY_S
        count = len(recs)
        total = sum(r.bytes for r in recs)
        dropped: set[str] = set()
        for rec in sorted(recs, key=lambda r: r.ts):
            too_old = max_age_s is not None and now - rec.ts > max_age_s
            if rec.id == keep_id or not (count > self.max_records or total > self.max_bytes or too_old):
                continue
            dropped.add(rec.id)
            count -= 1
            total -= rec.bytes
        if not dropped:
            self.remove_orphans({r.file for r in recs})
            return 0
        kept = [r for r in recs if r.id not in dropped]
        self.rewrite([r.model_dump_json() for r in kept])
        self.unlink_images([r for r in recs if r.id in dropped])
        PHOTO_PRUNED.inc(len(dropped))
        LOGGER.info("photo store pruned %d photo(s), %d kept (%d bytes)", len(dropped), count, total)
        return len(dropped)

    def rewrite(self, lines: list[str]) -> None:
        """Replace the index with these lines atomically, then sweep orphan JPEGs (caller holds the lock).

        Args:
            lines (list[str]): Index lines without newlines (corrupt ones included when they are to be kept).
        """
        write_atomic(self.index_path, "".join(f"{text}\n" for text in lines).encode())
        self.cache.invalidate()
        self.remove_orphans({r.file for r in self.records()})

    def unlink_images(self, recs: list[LookupRecord]) -> None:
        """Delete the JPEGs of removed records (already missing ones are fine).

        Args:
            recs (list[LookupRecord]): Removed records.
        """
        for rec in recs:
            (self.root / rec.file).unlink(missing_ok=True)

    def remove_orphans(self, indexed_files: set[str]) -> None:
        """Delete JPEGs no index line names, skipping ones younger than ORPHAN_MIN_AGE_S (file mtime, wall clock).

        Args:
            indexed_files (set[str]): File names that stay.
        """
        cutoff = time.time() - ORPHAN_MIN_AGE_S
        for path in self.root.glob("*.jpg"):
            try:
                if path.name not in indexed_files and path.stat().st_mtime < cutoff:
                    path.unlink(missing_ok=True)
                    LOGGER.info("photo store removed orphan image %s", path.name)
            except FileNotFoundError:
                continue

    def update(self, photo_id: str, changes: dict[str, Any]) -> LookupRecord:
        """Change a photo's name, note, status and/or POI links, rewriting only its index line.

        Args:
            photo_id (str): Photo id.
            changes (dict[str, Any]): Fields to set; "id" (equal to photo_id) and "kind" ("photo") are ignored.

        Returns:
            LookupRecord: The updated record (unchanged, with no rewrite, when nothing differs).

        Raises:
            PhotoStoreError: A capture field was given, or a value is invalid.
            PhotoNotFoundError: Unknown id.
        """
        fields = {
            k: v for k, v in changes.items() if not (k == "id" and v == photo_id or k == "kind" and v == PHOTO_KIND)
        }
        if not set(fields) <= EDITABLE_FIELDS:
            raise PhotoStoreError(f"{IMMUTABLE_MESSAGE}: {', '.join(sorted(set(fields) - EDITABLE_FIELDS))}")
        if isinstance(fields.get("poi_ids"), list):
            fields["poi_ids"] = list(dict.fromkeys(fields["poi_ids"]))
        with self.lock:
            entries = self.cache.entries()
            for index, (_, rec) in enumerate(entries):
                if rec is None or rec.id != photo_id:
                    continue
                try:
                    updated = LookupRecord.model_validate({**rec.model_dump(), **fields})
                except ValidationError as exc:
                    raise PhotoStoreError(str(exc)) from exc
                if updated == rec:
                    return rec
                updated = updated.model_copy(update={"updated_at": self.clock()})
                lines = [line for line, _ in entries]
                lines[index] = updated.model_dump_json()
                self.rewrite(lines)
                self.revision += 1
                return updated
        raise PhotoNotFoundError(photo_id)

    def delete(self, photo_id: str) -> LookupRecord:
        """Remove a photo's index line and its JPEG.

        Args:
            photo_id (str): Photo id.

        Returns:
            LookupRecord: The removed record.

        Raises:
            PhotoNotFoundError: Unknown id.
        """
        with self.lock:
            rec = self.get(photo_id)
            if rec is None:
                raise PhotoNotFoundError(photo_id)
            self.remove({photo_id})
        return rec

    def remove(self, ids: set[str]) -> None:
        """Rewrite the index without these records (corrupt lines kept) and unlink their JPEGs (caller holds the
        lock; every id is known).

        Args:
            ids (set[str]): Photo ids to remove.
        """
        entries = self.cache.entries()
        self.rewrite([line for line, rec in entries if rec is None or rec.id not in ids])
        self.unlink_images([rec for _, rec in entries if rec is not None and rec.id in ids])
        self.revision += 1
        self.track()

    def select(self, selector: PhotoSelector) -> tuple[list[LookupRecord], list[str]]:
        """Records a selector picks, and the requested ids it did not find.

        ids picks those records (ids unknown or excluded by created_by/camera are missing); older_than_s picks the
        records older than that; keep_newest picks all but the newest N. created_by and camera narrow the candidates
        first, so keep_newest counts within them.

        Args:
            selector (PhotoSelector): Validated selector.

        Returns:
            tuple[list[LookupRecord], list[str]]: (selected records, missing ids).
        """
        candidates = [
            r
            for r in self.records()
            if (selector.created_by is None or r.created_by == selector.created_by)
            and (selector.camera is None or r.camera == selector.camera)
        ]
        if selector.ids is not None:
            wanted = list(dict.fromkeys(selector.ids))
            by_id = {r.id: r for r in candidates}
            return [by_id[i] for i in wanted if i in by_id], [i for i in wanted if i not in by_id]
        if selector.older_than_s is not None:
            cutoff = self.clock() - selector.older_than_s
            return [r for r in candidates if r.ts < cutoff], []
        newest_first = sorted(candidates, key=lambda r: r.ts, reverse=True)
        return newest_first[selector.keep_newest :], []

    def delete_many(self, selector: PhotoSelector) -> dict[str, Any]:
        """Bulk delete with one index rewrite, or report what would go (dry_run, nothing changes).

        Args:
            selector (PhotoSelector): Which photos.

        Returns:
            dict[str, Any]: {removed, bytes_freed, missing} or, for dry_run, {would_remove, bytes}.
        """
        with self.lock:
            selected, missing = self.select(selector)
            freed = sum(r.bytes for r in selected)
            if selector.dry_run:
                return {"would_remove": len(selected), "bytes": freed}
            if selected:
                self.remove({r.id for r in selected})
                LOGGER.info("photo store deleted %d photo(s) (%d bytes)", len(selected), freed)
            return {"removed": len(selected), "bytes_freed": freed, "missing": missing}

    def export(self, dest: Path, ids: set[str] | None = None) -> list[LookupRecord]:
        """Hard-link (copy when linking fails) the JPEGs of every record, or only ids, into dest with an index.jsonl.

        The store and its revision are not changed. A record whose JPEG is missing is skipped and logged.

        Args:
            dest (Path): Target directory (created when missing); stale files of the same names are replaced.
            ids (set[str] | None): Only these photo ids, None for all.

        Returns:
            list[LookupRecord]: The exported records in index order.
        """
        with self.lock:
            dest.mkdir(parents=True, exist_ok=True)
            exported: list[LookupRecord] = []
            for rec in self.records():
                if ids is not None and rec.id not in ids:
                    continue
                source = self.root / rec.file
                if not source.is_file():
                    LOGGER.warning("photo export skipped %s: %s is missing", rec.id, rec.file)
                    continue
                link_or_copy(source, dest / rec.file)
                exported.append(rec)
            write_atomic(dest / INDEX_FILE, "".join(f"{r.model_dump_json()}\n" for r in exported).encode())
            return exported

    def replace_all(self, records: list[LookupRecord], source: Path | None) -> tuple[int, int]:
        """Make the index exactly these records, all or nothing, and delete the JPEGs of records that are gone.

        A JPEG of an already indexed id in root is kept (capture data is immutable); any other is linked (copied
        when linking fails) from source with a fresh mtime so the orphan sweep keeps it. bytes comes from the file.

        Args:
            records (list[LookupRecord]): The new photo set, in index order.
            source (Path | None): Directory holding the JPEGs of records not yet in root, None for none.

        Returns:
            tuple[int, int]: (records now indexed, records removed).

        Raises:
            PhotoStoreError: A record's JPEG is in neither root nor source, a file name is not a plain name, or an
                id repeats; nothing is changed.
        """
        with self.lock:
            old = self.records()
            known = {r.id for r in old}
            if len({r.id for r in records}) != len(records):
                raise PhotoStoreError("duplicate photo id in the new photo set")
            plan: list[tuple[LookupRecord, Path | None]] = []
            for rec in records:
                if Path(rec.file).name != rec.file:
                    raise PhotoStoreError(f"photo file {rec.file} is not a plain file name")
                in_root = self.root / rec.file
                from_source = None if source is None else source / rec.file
                if rec.id in known and in_root.is_file():
                    plan.append((rec, None))
                elif from_source is not None and from_source.is_file():
                    plan.append((rec, from_source))
                elif in_root.is_file():
                    plan.append((rec, None))
                else:
                    raise PhotoStoreError(f"photo file {rec.file} is in neither {self.root} nor the source")
            self.root.mkdir(parents=True, exist_ok=True)
            placed: list[Path] = []
            try:
                new: list[LookupRecord] = []
                for rec, origin in plan:
                    target = self.root / rec.file
                    if origin is not None:
                        existed = target.exists()
                        link_or_copy(origin, target)
                        os.utime(target)
                        if not existed:
                            placed.append(target)
                    new.append(rec.model_copy(update={"bytes": target.stat().st_size}))
                if new != old:
                    self.rewrite([r.model_dump_json() for r in new])
                    self.revision += 1
            except BaseException:
                for path in placed:
                    path.unlink(missing_ok=True)
                raise
            kept_ids = {r.id for r in new}
            gone = [r for r in old if r.id not in kept_ids]
            self.unlink_images(gone)
            self.track()
            return len(new), len(gone)

    def strip_links(self, poi_id: str) -> int:
        """Remove a (deleted) POI id from every photo's poi_ids; rewrites only when some photo references it.

        Args:
            poi_id (str): POI id.

        Returns:
            int: Photos changed.
        """
        with self.lock:
            entries = self.cache.entries()
            lines: list[str] = []
            changed = 0
            for line, rec in entries:
                if rec is not None and poi_id in rec.poi_ids:
                    rec = rec.model_copy(update={"poi_ids": [p for p in rec.poi_ids if p != poi_id]})
                    line = rec.model_dump_json()
                    changed += 1
                lines.append(line)
            if changed:
                self.rewrite(lines)
                self.revision += 1
            return changed

    def summary(self) -> dict[str, Any]:
        """The /poi/photos payload; also refreshes the count and bytes gauges.

        Returns:
            dict[str, Any]: {revision, count, bytes, oldest_ts, newest_ts, limits:{max_records, max_bytes,
            max_age_days}}; the timestamps are None when the store is empty.

        Raises:
            OSError: The index exists but cannot be read.
        """
        with self.lock:
            recs = self.records()
            total = self.track(recs)
            return {
                "revision": self.revision,
                "count": len(recs),
                "bytes": total,
                "oldest_ts": min((r.ts for r in recs), default=None),
                "newest_ts": max((r.ts for r in recs), default=None),
                "limits": {
                    "max_records": self.max_records,
                    "max_bytes": self.max_bytes,
                    "max_age_days": self.max_age_days,
                },
            }

    def track(self, recs: list[LookupRecord] | None = None) -> int:
        """Set the poi_photo_count and poi_photo_bytes gauges.

        Args:
            recs (list[LookupRecord] | None): Current records; read from the index when None.

        Returns:
            int: Total JPEG bytes.
        """
        recs = self.records() if recs is None else recs
        total = sum(r.bytes for r in recs)
        PHOTO_COUNT.set(len(recs))
        PHOTO_BYTES.set(total)
        return total


def link_or_copy(source: Path, target: Path) -> None:
    """Hard-link source to target (replacing a file already there); copy when linking is not possible.

    Args:
        source (Path): Existing file.
        target (Path): New name, same or another directory.
    """
    target.unlink(missing_ok=True)
    try:
        os.link(source, target)
    except OSError:
        shutil.copyfile(source, target)
