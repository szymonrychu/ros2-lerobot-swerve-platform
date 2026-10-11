"""Read-only access to the mcp_server camera lookup store (index.jsonl + <id>.jpg files).

The store is written by mcp_server (nodes/mcp_server/mcp_server/lookup_store.py) into a directory that web_ui reads
from the same host. The record, the filters (ISO-8601 or unix since/until) and the index reader come from ros2_common.lookups,
the same code mcp_server writes with: an index line that does not parse is skipped (logged once), filters are ANDed,
results are newest first, and the index is re-parsed only when it changed. Images are only served for ids present in
the index, and only when the indexed file resolves inside the store directory; ?max_px= serves a downscaled copy.
"""

from __future__ import annotations

import threading
from functools import lru_cache
from pathlib import Path

import cv2
import numpy as np
import structlog
from fastapi import FastAPI, Query
from fastapi.responses import FileResponse, JSONResponse, Response
from pydantic import ValidationError
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

from .config import AppConfig

log = structlog.get_logger(__name__)

__all__ = ["INDEX_FILE", "LookupQuery", "LookupRecord", "image_path", "query_records", "read_records", "record_summary"]

JPEG_MEDIA_TYPE = "image/jpeg"
# Stored JPEGs never change once written (a record id is a uuid), so browsers may cache them.
LOOKUP_IMAGE_CACHE_CONTROL = "private, max-age=3600"
HTTP_NOT_FOUND = 404
HTTP_UNPROCESSABLE = 422
MIN_THUMB_PX = 16
MAX_THUMB_PX = 8192
THUMB_JPEG_QUALITY = 80
THUMB_CACHE_ENTRIES = 256

CACHES: dict[Path, IndexCache] = {}
CACHES_LOCK = threading.Lock()


def index_cache(root: Path) -> IndexCache:
    """The parsed-index cache of a store directory (one per directory, shared by all requests).

    Args:
        root (Path): Store directory.

    Returns:
        IndexCache: The cache.
    """
    with CACHES_LOCK:
        cache = CACHES.get(root)
        if cache is None:
            cache = CACHES[root] = IndexCache(root / INDEX_FILE)
        return cache


def read_records(root: Path) -> list[LookupRecord]:
    """Every valid record of the store, oldest first; the index is re-parsed only when it changed and corrupt
    lines are logged once.

    Args:
        root (Path): Store directory.

    Returns:
        list[LookupRecord]: Records; empty when the index does not exist or cannot be read.
    """
    try:
        return index_cache(root).records()
    except OSError as exc:
        log.warning("lookup_index_unreadable", root=str(root), error=str(exc))
        return []


def query_records(root: Path, query: LookupQuery) -> list[LookupRecord]:
    """Records matching the filters, newest first, at most query.limit.

    Args:
        root (Path): Store directory.
        query (LookupQuery): Filters and limit.

    Returns:
        list[LookupRecord]: Matches.
    """
    matches = [r for r in read_records(root) if query.matches(r)]
    matches.sort(key=lambda r: r.ts, reverse=True)
    return matches[: query.limit]


def image_path(root: Path, lookup_id: str) -> Path | None:
    """Path of the JPEG of an indexed lookup.

    Args:
        root (Path): Store directory.
        lookup_id (str): Record id from the URL.

    Returns:
        Path | None: The file, or None when the id is not in the index, the indexed file name resolves outside the
        store directory or the file does not exist.
    """
    rec = next((r for r in read_records(root) if r.id == lookup_id), None)
    if rec is None:
        return None
    base = root.resolve()
    path = (base / rec.file).resolve()
    if not path.is_relative_to(base):
        log.warning("lookup_path_traversal_attempt", lookup_id=lookup_id, file=rec.file)
        return None
    return path if path.is_file() else None


@lru_cache(maxsize=THUMB_CACHE_ENTRIES)
def scaled_jpeg(path: str, mtime_ns: int, max_px: int) -> bytes | None:
    """The JPEG downscaled so its longest side is at most max_px (the stored bytes when already small enough).

    Args:
        path (str): JPEG file.
        mtime_ns (int): File modification time (cache key: a changed file is re-encoded).
        max_px (int): Longest side limit (px).

    Returns:
        bytes | None: JPEG bytes, None when the file cannot be decoded.
    """
    raw = Path(path).read_bytes()
    img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return None
    height, width = img.shape[:2]
    if max(width, height) <= max_px:
        return raw
    scale = max_px / max(width, height)
    size = (max(1, round(width * scale)), max(1, round(height * scale)))
    ok, buf = cv2.imencode(
        ".jpg", cv2.resize(img, size, interpolation=cv2.INTER_AREA), [cv2.IMWRITE_JPEG_QUALITY, THUMB_JPEG_QUALITY]
    )
    return buf.tobytes() if ok else None


def register_lookup_routes(app: FastAPI, config: AppConfig) -> None:
    """Add GET /api/lookups and GET /api/lookups/{id}.jpg, reading the first map_nav tab's lookups_dir.

    Args:
        app (FastAPI): Application.
        config (AppConfig): Validated configuration; without a map_nav tab no route is added.
    """
    tab = next(iter(config.map_nav_tabs()), None)
    if tab is None or tab.lookups_dir is None:
        return
    root = Path(tab.lookups_dir)

    # Plain `def` routes: FastAPI runs them in its thread pool, so index reads and JPEG scaling never block the event
    # loop that also serves the teleop WebSocket.
    @app.get("/api/lookups")
    def list_lookups(
        since: str | None = None,
        until: str | None = None,
        near_x: float | None = None,
        near_y: float | None = None,
        radius_m: float | None = Query(default=None, gt=0),
        poi_id: str | None = None,
        text: str | None = None,
        camera: str | None = None,
        mcp_session: str | None = None,
        limit: int = Query(default=DEFAULT_QUERY_LIMIT, ge=1, le=MAX_QUERY_LIMIT),
    ) -> Response:
        near = (near_x, near_y, radius_m)
        if any(v is not None for v in near) and any(v is None for v in near):
            return JSONResponse({"error": "near needs near_x, near_y and radius_m together"}, HTTP_UNPROCESSABLE)
        try:
            query = LookupQuery(
                since=since,  # type: ignore[arg-type]  # validated: unix seconds or ISO-8601
                until=until,  # type: ignore[arg-type]
                near=None if radius_m is None else NearFilter(x=near_x, y=near_y, radius_m=radius_m),  # type: ignore[arg-type]
                poi_id=poi_id,
                text=text,
                camera=camera,
                mcp_session=mcp_session,
                limit=limit,
            )
        except ValidationError as exc:
            return JSONResponse({"error": "; ".join(str(err["msg"]) for err in exc.errors())}, HTTP_UNPROCESSABLE)
        return JSONResponse({"lookups": [record_summary(r) for r in query_records(root, query)]})

    @app.get("/api/lookups/{lookup_id}.jpg")
    def get_lookup_image(
        lookup_id: str, max_px: int | None = Query(default=None, ge=MIN_THUMB_PX, le=MAX_THUMB_PX)
    ) -> Response:
        path = image_path(root, lookup_id)
        if path is None:
            return JSONResponse({"error": "unknown lookup"}, HTTP_NOT_FOUND)
        headers = {"Cache-Control": LOOKUP_IMAGE_CACHE_CONTROL}
        if max_px is None:
            return FileResponse(path, media_type=JPEG_MEDIA_TYPE, headers=headers)
        try:
            data = scaled_jpeg(str(path), path.stat().st_mtime_ns, max_px)
        except OSError:
            data = None
        if data is None:
            return JSONResponse({"error": "lookup image cannot be decoded"}, HTTP_NOT_FOUND)
        return Response(data, media_type=JPEG_MEDIA_TYPE, headers=headers)
