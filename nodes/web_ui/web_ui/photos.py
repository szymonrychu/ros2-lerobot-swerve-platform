"""Read-only access to the photo store (index.jsonl + <id>.jpg files) written by poi_store and mcp_server.

A photo is a POI of kind "photo" (ros2_common.photos.photo_as_poi). The record and the index reader come from
ros2_common.lookups, the same code poi_store and mcp_server use: an index line that does not parse is skipped (logged
once) and the index is re-parsed only when it changed. GET /api/photos pages the photos newest first with filters and a
cursor; GET /api/photos/{id}.jpg serves an image, only for ids present in the index and only when the indexed file
resolves inside the store directory; ?max_px= serves a downscaled copy.
"""

from __future__ import annotations

import math
import threading
from functools import lru_cache
from pathlib import Path

import cv2
import numpy as np
import structlog
from fastapi import FastAPI, Query
from fastapi.responses import FileResponse, JSONResponse, Response
from ros2_common.lookups import INDEX_FILE, IndexCache, LookupRecord, parse_time
from ros2_common.photos import PhotoCreator, photo_as_poi

from .config import AppConfig

log = structlog.get_logger(__name__)

__all__ = ["INDEX_FILE", "LookupRecord", "image_path", "photo_as_poi", "read_records", "select_photos"]

DEFAULT_PAGE_LIMIT = 50
MAX_PAGE_LIMIT = 1000
JPEG_MEDIA_TYPE = "image/jpeg"
# Stored JPEGs never change once written (a record id is a uuid), so browsers may cache them.
PHOTO_IMAGE_CACHE_CONTROL = "private, max-age=3600"
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
        log.warning("photo_index_unreadable", root=str(root), error=str(exc))
        return []


def parse_cursor(before: str) -> tuple[float, str]:
    """Parse the paging cursor '<ts>:<id>'.

    Args:
        before (str): Cursor from a previous next_before.

    Returns:
        tuple[float, str]: (ts, id).

    Raises:
        ValueError: The cursor is not '<finite float>:<non-empty id>'.
    """
    ts_text, sep, photo_id = before.partition(":")
    if not sep or not photo_id:
        raise ValueError("before must be '<ts>:<id>'")
    ts = float(ts_text)
    if not math.isfinite(ts):
        raise ValueError("before must be '<ts>:<id>'")
    return ts, photo_id


def select_photos(
    records: list[LookupRecord],
    *,
    since: float | None = None,
    until: float | None = None,
    near: tuple[float, float, float] | None = None,
    poi_id: str | None = None,
    text: str | None = None,
    camera: str | None = None,
    created_by: str | None = None,
) -> list[LookupRecord]:
    """Photos passing every given filter (AND), newest first (ties broken by id, descending).

    Args:
        records (list[LookupRecord]): Index records.
        since (float | None): Earliest ts (unix s).
        until (float | None): Latest ts (unix s).
        near (tuple[float, float, float] | None): (x, y, radius_m) in the map; photos without a pose never match.
        poi_id (str | None): The photo must be linked to this POI.
        text (str | None): Case-insensitive substring of the name or the note.
        camera (str | None): Exact camera name.
        created_by (str | None): "agent" or "user".

    Returns:
        list[LookupRecord]: Matches, newest first.
    """
    needle = None if text is None else text.casefold()
    matches: list[LookupRecord] = []
    for rec in records:
        if (
            (since is not None and rec.ts < since)
            or (until is not None and rec.ts > until)
            or (camera is not None and rec.camera != camera)
            or (created_by is not None and rec.created_by != created_by)
            or (poi_id is not None and poi_id not in rec.poi_ids)
            or (needle is not None and needle not in rec.note.casefold() and needle not in rec.name.casefold())
        ):
            continue
        if near is not None:
            poi = photo_as_poi(rec)
            if poi["x"] is None or math.hypot(poi["x"] - near[0], poi["y"] - near[1]) > near[2]:
                continue
        matches.append(rec)
    matches.sort(key=lambda r: (r.ts, r.id), reverse=True)
    return matches


def image_path(root: Path, photo_id: str) -> Path | None:
    """Path of the JPEG of an indexed photo.

    Args:
        root (Path): Store directory.
        photo_id (str): Record id from the URL.

    Returns:
        Path | None: The file, or None when the id is not in the index, the indexed file name resolves outside the
        store directory or the file does not exist.
    """
    rec = next((r for r in read_records(root) if r.id == photo_id), None)
    if rec is None:
        return None
    base = root.resolve()
    path = (base / rec.file).resolve()
    if not path.is_relative_to(base):
        log.warning("photo_path_traversal_attempt", photo_id=photo_id, file=rec.file)
        return None
    return path if path.is_file() else None


@lru_cache(maxsize=THUMB_CACHE_ENTRIES)
def scaled_jpeg(path: str, _mtime_ns: int, max_px: int) -> bytes | None:
    """The JPEG downscaled so its longest side is at most max_px (the stored bytes when already small enough).

    Args:
        path (str): JPEG file.
        _mtime_ns (int): File modification time (cache key: a changed file is re-encoded).
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


def register_photo_routes(app: FastAPI, config: AppConfig) -> None:
    """Add GET /api/photos and GET /api/photos/{id}.jpg, reading the first map_nav tab's lookups_dir.

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
    @app.get("/api/photos")
    def list_photos(
        since: str | None = None,
        until: str | None = None,
        near_x: float | None = None,
        near_y: float | None = None,
        radius_m: float | None = Query(default=None, gt=0),
        poi_id: str | None = None,
        text: str | None = None,
        camera: str | None = None,
        created_by: PhotoCreator | None = None,
        before: str | None = None,
        limit: int = Query(default=DEFAULT_PAGE_LIMIT, ge=1, le=MAX_PAGE_LIMIT),
    ) -> Response:
        if any(v is not None for v in (near_x, near_y, radius_m)) and any(
            v is None for v in (near_x, near_y, radius_m)
        ):
            return JSONResponse({"error": "near needs near_x, near_y and radius_m together"}, HTTP_UNPROCESSABLE)
        try:
            since_ts, until_ts = parse_time(since), parse_time(until)
            cursor = None if before is None else parse_cursor(before)
        except ValueError as exc:
            return JSONResponse({"error": str(exc)}, HTTP_UNPROCESSABLE)
        near = None if radius_m is None else (near_x, near_y, radius_m)
        matches = select_photos(
            read_records(root),
            since=since_ts,
            until=until_ts,
            near=near,  # type: ignore[arg-type]  # all three given: checked above
            poi_id=poi_id,
            text=text,
            camera=camera,
            created_by=created_by,
        )
        page = matches if cursor is None else [r for r in matches if (r.ts, r.id) < cursor]
        shown = page[:limit]
        next_before = f"{shown[-1].ts!r}:{shown[-1].id}" if len(page) > limit else None
        return JSONResponse(
            {"photos": [photo_as_poi(r) for r in shown], "next_before": next_before, "total_matching": len(matches)}
        )

    @app.get("/api/photos/{photo_id}.jpg")
    def get_photo_image(
        photo_id: str, max_px: int | None = Query(default=None, ge=MIN_THUMB_PX, le=MAX_THUMB_PX)
    ) -> Response:
        path = image_path(root, photo_id)
        if path is None:
            return JSONResponse({"error": "unknown photo"}, HTTP_NOT_FOUND)
        headers = {"Cache-Control": PHOTO_IMAGE_CACHE_CONTROL}
        if max_px is None:
            return FileResponse(path, media_type=JPEG_MEDIA_TYPE, headers=headers)
        try:
            data = scaled_jpeg(str(path), path.stat().st_mtime_ns, max_px)
        except OSError:
            data = None
        if data is None:
            return JSONResponse({"error": "photo image cannot be decoded"}, HTTP_NOT_FOUND)
        return Response(data, media_type=JPEG_MEDIA_TYPE, headers=headers)
