"""Map tile proxy: fetch XYZ raster tiles server-side with a disk cache, so the browser loads them from 'self'.

The browser requests /api/tiles/{z}/{x}/{y}.png; the backend serves the tile from its disk cache or fetches it
from the configured tile_url (subdomain rotation for {s}), stores it and returns it. Cached tiles are served
without touching the network, so previously viewed areas keep working offline. The cache is capped in size;
when it grows past the cap the oldest-written tiles are evicted first.
"""

from __future__ import annotations

import asyncio
import os
import threading
from pathlib import Path

import httpx
import structlog

log = structlog.get_logger(__name__)

# Highest zoom level accepted by the proxy (no raster provider serves deeper tiles).
TILE_MAX_ZOOM = 22
# Tile servers' usage policies require an identifying User-Agent.
TILE_USER_AGENT = "web_ui-tile-proxy/0.1 (ros2-lerobot-sverve-platform robot dashboard)"
# Upstream fetch timeout (seconds); a slow or unreachable tile server yields 504 instead of hanging the browser.
TILE_FETCH_TIMEOUT_S = 5.0
# Browser cache lifetime of a served tile (seconds).
TILE_BROWSER_MAX_AGE_S = 86400
TILE_MEDIA_TYPE = "image/png"
HTTP_OK = 200
HTTP_NOT_FOUND = 404
HTTP_GATEWAY_TIMEOUT = 504
BYTES_PER_MB = 1024 * 1024
# Eviction deletes oldest tiles down to this fraction of the cap, so the full scan runs rarely.
TILE_CACHE_LOW_WATER_FRACTION = 0.9


def validate_tile(z: int, x: int, y: int) -> bool:
    """Check XYZ tile coordinates are inside the Web Mercator tile pyramid.

    Args:
        z (int): Zoom level.
        x (int): Tile column.
        y (int): Tile row.

    Returns:
        bool: True when 0 <= z <= TILE_MAX_ZOOM and 0 <= x, y < 2**z.
    """
    if not 0 <= z <= TILE_MAX_ZOOM:
        return False
    n = 1 << z
    return 0 <= x < n and 0 <= y < n


def build_tile_url(template: str, subdomains: str, z: int, x: int, y: int) -> str:
    """Fill an XYZ tile URL template.

    {s} rotates through subdomains by (x + y) so neighbouring tiles spread over the provider's hosts; {r}
    (retina suffix) is dropped.

    Args:
        template (str): URL template with {z}, {x}, {y} and optionally {s} and {r}.
        subdomains (str): Characters substituted for {s} (may be empty when the template has no {s}).
        z (int): Zoom level.
        x (int): Tile column.
        y (int): Tile row.

    Returns:
        str: The tile URL.
    """
    s = subdomains[(x + y) % len(subdomains)] if subdomains else ""
    return (
        template.replace("{s}", s)
        .replace("{r}", "")
        .replace("{z}", str(z))
        .replace("{x}", str(x))
        .replace("{y}", str(y))
    )


class TileCache:
    """Disk cache of tiles at <root>/<z>/<x>/<y>.png, capped at max_bytes (oldest-written evicted first).

    Filesystem errors (missing permissions, full disk) are logged and treated as cache misses, never raised.
    """

    def __init__(self, root: Path, max_bytes: int) -> None:
        """Initialise the cache (no filesystem access until first use).

        Args:
            root (Path): Cache directory, created on first write.
            max_bytes (int): Size cap in bytes.
        """
        self.root = root
        self.max_bytes = max_bytes
        self.total: int | None = None
        self.lock = threading.Lock()

    def path(self, z: int, x: int, y: int) -> Path:
        """Return the cache file path of a tile.

        Args:
            z (int): Zoom level.
            x (int): Tile column.
            y (int): Tile row.

        Returns:
            Path: <root>/<z>/<x>/<y>.png.
        """
        return self.root / str(z) / str(x) / f"{y}.png"

    def get(self, z: int, x: int, y: int) -> bytes | None:
        """Read a cached tile.

        Args:
            z (int): Zoom level.
            x (int): Tile column.
            y (int): Tile row.

        Returns:
            bytes | None: Tile bytes, or None when not cached or unreadable.
        """
        try:
            return self.path(z, x, y).read_bytes()
        except OSError:
            return None

    def put(self, z: int, x: int, y: int, data: bytes) -> None:
        """Store a tile (atomic rename), then, when the running size estimate exceeds max_bytes, evict the oldest
        tiles down to the low-water mark. Blocking: call from a worker thread in async code.

        Args:
            z (int): Zoom level.
            x (int): Tile column.
            y (int): Tile row.
            data (bytes): Tile bytes.
        """
        target = self.path(z, x, y)
        try:
            target.parent.mkdir(parents=True, exist_ok=True)
            tmp = target.with_suffix(".tmp")
            tmp.write_bytes(data)
            os.replace(tmp, target)
        except OSError as exc:
            log.warning("tile_cache_write_failed", path=str(target), error=str(exc))
            return
        with self.lock:
            self.total = self.total_bytes() if self.total is None else self.total + len(data)
            if self.total > self.max_bytes:
                self.evict()

    def files(self) -> list[tuple[float, int, Path]]:
        """List cached tiles.

        Returns:
            list[tuple[float, int, Path]]: (mtime, size, path) per cached tile, oldest first.
        """
        entries: list[tuple[float, int, Path]] = []
        for path in self.root.rglob("*.png"):
            try:
                st = path.stat()
            except OSError:
                continue
            entries.append((st.st_mtime, st.st_size, path))
        return sorted(entries)

    def total_bytes(self) -> int:
        """Return the cache size by scanning the directory.

        Returns:
            int: Sum of cached tile sizes in bytes.
        """
        return sum(size for _, size, _ in self.files())

    def evict(self) -> None:
        """Delete the oldest-written tiles until the cache is within the low-water mark (90% of max_bytes)."""
        entries = self.files()
        total = sum(size for _, size, _ in entries)
        target = int(self.max_bytes * TILE_CACHE_LOW_WATER_FRACTION)
        for _, size, path in entries:
            if total <= target:
                break
            try:
                path.unlink()
            except OSError as exc:
                log.warning("tile_cache_evict_failed", path=str(path), error=str(exc))
                continue
            total -= size
        self.total = total
        log.info("tile_cache_evicted", total_bytes=total, max_bytes=self.max_bytes)


class TileProxy:
    """Serves tiles from a TileCache, fetching misses from the upstream tile server with httpx."""

    def __init__(
        self,
        tile_url: str,
        subdomains: str,
        cache: TileCache,
        timeout_s: float = TILE_FETCH_TIMEOUT_S,
        transport: httpx.AsyncBaseTransport | None = None,
    ) -> None:
        """Initialise the proxy (the HTTP client is created on first fetch).

        Args:
            tile_url (str): XYZ URL template (see build_tile_url).
            subdomains (str): Characters rotated into {s}.
            cache (TileCache): Disk cache.
            timeout_s (float): Upstream request timeout in seconds.
            transport (httpx.AsyncBaseTransport | None): Custom httpx transport (tests), None for the network.
        """
        self.tile_url = tile_url
        self.subdomains = subdomains
        self.cache = cache
        self.timeout_s = timeout_s
        self.transport = transport
        self.client: httpx.AsyncClient | None = None

    async def get(self, z: int, x: int, y: int) -> tuple[int, bytes | None]:
        """Return a tile from the cache, or fetch, cache and return it.

        Args:
            z (int): Zoom level (validated by the caller).
            x (int): Tile column.
            y (int): Tile row.

        Returns:
            tuple[int, bytes | None]: (200, png bytes); (404, None) when the upstream has no such tile or
                answers with a non-image; (504, None) when the upstream times out or is unreachable.
        """
        cached = await asyncio.to_thread(self.cache.get, z, x, y)
        if cached is not None:
            return HTTP_OK, cached
        url = build_tile_url(self.tile_url, self.subdomains, z, x, y)
        if self.client is None:
            self.client = httpx.AsyncClient(
                transport=self.transport,
                timeout=self.timeout_s,
                headers={"User-Agent": TILE_USER_AGENT},
                follow_redirects=True,
            )
        try:
            resp = await self.client.get(url)
        except httpx.TransportError as exc:
            log.warning("tile_fetch_failed", url=url, error=type(exc).__name__)
            return HTTP_GATEWAY_TIMEOUT, None
        content_type = resp.headers.get("content-type", "")
        if resp.status_code != HTTP_OK or not content_type.startswith("image/"):
            log.info("tile_unavailable", url=url, status=resp.status_code, content_type=content_type)
            return HTTP_NOT_FOUND, None
        await asyncio.to_thread(self.cache.put, z, x, y, resp.content)
        return HTTP_OK, resp.content

    async def aclose(self) -> None:
        """Close the HTTP client (application shutdown)."""
        if self.client is not None:
            await self.client.aclose()
            self.client = None
