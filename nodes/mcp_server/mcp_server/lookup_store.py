"""Photo store facade (read-only index): the <id>.jpg files mcp_server writes and the index.jsonl poi_store owns.

A photo is a POI of kind "photo" (E11). poi_store is the only writer of index.jsonl (adds, annotations, deletes and
retention); mcp_server writes the JPEG before asking poi_store to add the photo, and removes it again when poi_store
refuses it. Reads (records, get, query, image) go through the shared stat-keyed IndexCache (ros2_common.lookups), so
corrupt index lines are skipped and logged once.
"""

from pathlib import Path

from ros2_common.lookups import (
    DEFAULT_QUERY_LIMIT,
    INDEX_FILE,
    MAX_QUERY_LIMIT,
    IndexCache,
    LookupQuery,
    LookupRecord,
    NearFilter,
    record_summary,
)
from ros2_common.photos import PHOTO_ID_RE, write_atomic

__all__ = [
    "DEFAULT_QUERY_LIMIT",
    "INDEX_FILE",
    "MAX_QUERY_LIMIT",
    "LookupQuery",
    "LookupRecord",
    "LookupStore",
    "NearFilter",
    "record_summary",
]


def image_name(photo_id: str) -> str:
    """JPEG file name of a photo id.

    Args:
        photo_id (str): Photo id (32 lowercase hex characters).

    Returns:
        str: '<id>.jpg'.

    Raises:
        ValueError: The id is not 32 lowercase hex characters (it would name a file outside the store).
    """
    if not PHOTO_ID_RE.match(photo_id):
        raise ValueError(f"photo id must be 32 lowercase hex characters, got {photo_id!r}")
    return f"{photo_id}.jpg"


class LookupStore:
    """Read-only view of the photo index plus the JPEG writes that precede a photo add.

    Attributes:
        root (Path): Store directory (created by the first write_image).
        cache (IndexCache): Parsed index.jsonl, re-read only when the file changed.
    """

    def __init__(self, root: Path) -> None:
        """Create the store (nothing is touched on disk until the first write_image).

        Args:
            root (Path): Store directory.
        """
        self.root = root
        self.cache = IndexCache(root / INDEX_FILE)

    def records(self) -> list[LookupRecord]:
        """Every valid record, oldest first (corrupt lines are logged once and skipped).

        Returns:
            list[LookupRecord]: Records.
        """
        return self.cache.records()

    def get(self, lookup_id: str) -> LookupRecord | None:
        """One record by id.

        Args:
            lookup_id (str): Record id.

        Returns:
            LookupRecord | None: The record, None when unknown.
        """
        return next((r for r in self.records() if r.id == lookup_id), None)

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

    def image(self, rec: LookupRecord) -> bytes:
        """JPEG bytes of a record.

        Args:
            rec (LookupRecord): Record.

        Returns:
            bytes: The stored JPEG.
        """
        return (self.root / rec.file).read_bytes()

    def write_image(self, photo_id: str, jpeg: bytes) -> Path:
        """Write a photo's JPEG atomically (mode 0644), creating the store directory when needed.

        Args:
            photo_id (str): Photo id (32 lowercase hex characters).
            jpeg (bytes): Exactly the JPEG the model received.

        Returns:
            Path: The written file.
        """
        path = self.root / image_name(photo_id)
        self.root.mkdir(parents=True, exist_ok=True)
        write_atomic(path, jpeg)
        return path

    def remove_image(self, photo_id: str) -> None:
        """Delete a photo's JPEG (no error when it is already gone).

        Args:
            photo_id (str): Photo id (32 lowercase hex characters).
        """
        (self.root / image_name(photo_id)).unlink(missing_ok=True)
