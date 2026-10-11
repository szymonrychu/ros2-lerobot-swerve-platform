"""Read-only access to the mcp_server camera lookup store (index.jsonl + <id>.jpg files).

The store is written by mcp_server (nodes/mcp_server/mcp_server/lookup_store.py) into a directory that web_ui reads
from the same host. The record shape and the parsing rules mirror it: an index line that does not parse is skipped,
filters are ANDed, results are newest first. Images are only served for ids present in the index, and only when the
indexed file resolves inside the store directory.
"""

from __future__ import annotations

import math
import time
from pathlib import Path

import structlog
from fastapi import FastAPI, Query
from fastapi.responses import FileResponse, JSONResponse, Response
from pydantic import BaseModel, Field, ValidationError

from .config import AppConfig

log = structlog.get_logger(__name__)

INDEX_FILE = "index.jsonl"
MAX_QUERY_LIMIT = 20
DEFAULT_QUERY_LIMIT = 10
JPEG_MEDIA_TYPE = "image/jpeg"
# Stored JPEGs never change once written (a record id is a uuid), so browsers may cache them.
LOOKUP_IMAGE_CACHE_CONTROL = "private, max-age=3600"
HTTP_NOT_FOUND = 404
HTTP_UNPROCESSABLE = 422


class LookupRecord(BaseModel):
    """One index.jsonl line (the fields of mcp_server's LookupRecord that web_ui reads or passes on)."""

    id: str
    ts: float
    tool: str
    camera: str
    width: int
    height: int
    file: str
    bytes: int
    base_pose_map: dict[str, float] | None = None
    arm_joints: dict[str, float] | None = None
    camera_pose_base: dict[str, list[float]] | None = None
    mcp_session: str | None = None
    note: str = ""
    poi_ids: list[str] = Field(default_factory=list)


class LookupFilters(BaseModel):
    """GET /api/lookups filters (the list_lookups arguments of mcp_server); every given filter must match."""

    since: float | None = None
    until: float | None = None
    near_x: float | None = None
    near_y: float | None = None
    radius_m: float | None = None
    poi_id: str | None = None
    text: str | None = None
    camera: str | None = None
    mcp_session: str | None = None
    limit: int = DEFAULT_QUERY_LIMIT

    def matches(self, rec: LookupRecord) -> bool:
        """Whether a record passes every filter.

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
        if self.near_x is not None and self.near_y is not None and self.radius_m is not None:
            pose = rec.base_pose_map
            if pose is None or math.hypot(pose["x"] - self.near_x, pose["y"] - self.near_y) > self.radius_m:
                return False
        return True


def read_records(root: Path) -> list[LookupRecord]:
    """Every valid record of the store, oldest first; corrupt lines are logged and skipped.

    Args:
        root (Path): Store directory.

    Returns:
        list[LookupRecord]: Records; empty when the index does not exist.
    """
    try:
        lines = (root / INDEX_FILE).read_text().splitlines()
    except FileNotFoundError:
        return []
    except OSError as exc:
        log.warning("lookup_index_unreadable", root=str(root), error=str(exc))
        return []
    out: list[LookupRecord] = []
    for number, line in enumerate(lines, start=1):
        if not line.strip():
            continue
        try:
            out.append(LookupRecord.model_validate_json(line))
        except ValidationError:
            log.warning("lookup_index_line_corrupt", root=str(root), line=number)
    return out


def query_records(root: Path, filters: LookupFilters) -> list[LookupRecord]:
    """Records matching the filters, newest first, at most filters.limit.

    Args:
        root (Path): Store directory.
        filters (LookupFilters): Filters and limit.

    Returns:
        list[LookupRecord]: Matches.
    """
    matches = [r for r in read_records(root) if filters.matches(r)]
    matches.sort(key=lambda r: r.ts, reverse=True)
    return matches[: filters.limit]


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


def record_summary(rec: LookupRecord) -> dict[str, object]:
    """JSON metadata of a record: the record plus an ISO time (same as mcp_server's tools return).

    Args:
        rec (LookupRecord): Record.

    Returns:
        dict[str, object]: Metadata.
    """
    data: dict[str, object] = rec.model_dump(mode="json")
    data["time"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(rec.ts))
    return data


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

    @app.get("/api/lookups")
    async def list_lookups(
        since: float | None = None,
        until: float | None = None,
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
        filters = LookupFilters(
            since=since,
            until=until,
            near_x=near_x,
            near_y=near_y,
            radius_m=radius_m,
            poi_id=poi_id,
            text=text,
            camera=camera,
            mcp_session=mcp_session,
            limit=limit,
        )
        return JSONResponse({"lookups": [record_summary(r) for r in query_records(root, filters)]})

    @app.get("/api/lookups/{lookup_id}.jpg")
    async def get_lookup_image(lookup_id: str) -> Response:
        path = image_path(root, lookup_id)
        if path is None:
            return JSONResponse({"error": "unknown lookup"}, HTTP_NOT_FOUND)
        return FileResponse(path, media_type=JPEG_MEDIA_TYPE, headers={"Cache-Control": LOOKUP_IMAGE_CACHE_CONTROL})
