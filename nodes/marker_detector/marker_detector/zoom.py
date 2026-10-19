"""Zoom still planner: rate-limited automatic requests and still pairing (no ROS)."""

import time
from collections.abc import Callable
from dataclasses import dataclass

from marker_detector.config import ZoomConfig

GRID_CELLS = 8
ZOOM_STEPS = (2, 4, 8)
UNDECODED_ZOOM = 4
OK_STATUSES = frozenset({"ok", "af_failed"})
MAX_BUFFERED_IMAGES = 4


@dataclass(frozen=True)
class ZoomTarget:
    """A candidate for a zoom still.

    Attributes:
        u: Normalised x of the quad centre in the stream image (0..1).
        v: Normalised y of the quad centre (0..1).
        module_px: Decoded module size in pixels, or None for an undecoded quad.
    """

    u: float
    v: float
    module_px: float | None = None

    @property
    def region(self) -> tuple[int, int]:
        """Grid cell (1/8 of the image) containing the target."""
        return (
            min(GRID_CELLS - 1, max(0, int(self.u * GRID_CELLS))),
            min(GRID_CELLS - 1, max(0, int(self.v * GRID_CELLS))),
        )

    def zoom_for(self, target_module_px: float) -> int:
        """Smallest zoom of 2/4/8 bringing the module to ``target_module_px``.

        Args:
            target_module_px: Desired module size after zoom.
        Returns:
            The zoom factor; 4 for an undecoded quad.
        """
        if self.module_px is None or self.module_px <= 0:
            return UNDECODED_ZOOM
        needed = target_module_px / self.module_px
        return next((z for z in ZOOM_STEPS if z >= needed), ZOOM_STEPS[-1])


@dataclass(frozen=True)
class ZoomFrame:
    """A zoom still paired with its result."""

    request_id: str
    stamp_ns: int
    status: str
    jpeg: bytes
    target: ZoomTarget | None


@dataclass
class PendingZoom:
    """An accepted result waiting for (or already holding) its image."""

    request_id: str
    stamp_ns: int
    status: str
    target: ZoomTarget | None
    frame: ZoomFrame | None = None


class ZoomPlanner:
    """Decides when to request a zoom still and pairs results with images."""

    def __init__(self, config: ZoomConfig, clock: Callable[[], float] = time.monotonic) -> None:
        """Create a planner.

        Args:
            config: Zoom settings.
            clock: Monotonic seconds source used when a call is not given ``now``.
        """
        self.config = config
        self.clock = clock
        self.outstanding: tuple[str, ZoomTarget, float] | None = None
        self.last_request_at: float | None = None
        self.region_at: dict[tuple[int, int], float] = {}
        self.pending: dict[int, PendingZoom] = {}
        self.images: dict[int, tuple[bytes, float]] = {}

    def propose(self, targets: list[ZoomTarget], now: float) -> ZoomTarget | None:
        """Pick at most one target to request, honouring every limit.

        Args:
            targets: Small decoded candidates and undecoded quads.
            now: Current time in seconds.
        Returns:
            The chosen target or None.
        """
        cfg = self.config
        if not cfg.auto_zoom or self.outstanding is not None:
            return None
        if self.last_request_at is not None and now - self.last_request_at < cfg.min_interval_s:
            return None
        ready = [t for t in targets if now - self.region_at.get(t.region, float("-inf")) >= cfg.cooldown_s]
        if not ready:
            return None
        return min(ready, key=lambda t: (t.module_px is not None, t.module_px or 0.0))

    def sent(self, request_id: str, target: ZoomTarget, now: float) -> None:
        """Record that a request was published.

        Args:
            request_id: Id sent on the command topic.
            target: The target requested.
            now: Current time in seconds.
        """
        self.outstanding = (request_id, target, now)
        self.last_request_at = now
        self.region_at[target.region] = now

    def on_result(self, result: dict, now: float) -> PendingZoom | None:
        """Handle a zoom result message.

        Args:
            result: Dict with ``request_id``, ``status`` and (when ok) ``stamp_ns``.
            now: Current time in seconds.
        Returns:
            A PendingZoom (with ``frame`` set if the image already arrived) for
            ok/af_failed results we process, else None.
        """
        request_id = str(result.get("request_id", ""))
        status = str(result.get("status", ""))
        own = self.outstanding is not None and self.outstanding[0] == request_id
        target = self.outstanding[1] if own and self.outstanding else None
        if own:
            self.outstanding = None
        if status not in OK_STATUSES or "stamp_ns" not in result:
            return None
        if not own and not self.config.process_stills:
            return None
        stamp_ns = int(result["stamp_ns"])
        pending = PendingZoom(request_id, stamp_ns, status, target)
        buffered = self.images.pop(stamp_ns, None)
        if buffered is not None:
            pending.frame = ZoomFrame(request_id, stamp_ns, status, buffered[0], target)
        else:
            self.pending[stamp_ns] = pending
            while len(self.pending) > MAX_BUFFERED_IMAGES:
                del self.pending[next(iter(self.pending))]
        return pending

    def on_image(self, stamp_ns: int, jpeg_bytes: bytes) -> ZoomFrame | None:
        """Handle a zoom still image.

        Args:
            stamp_ns: Image stamp, matching the result's ``stamp_ns``.
            jpeg_bytes: Compressed image data.
        Returns:
            The paired frame, or None if no result matches yet (image is buffered).
        """
        pending = self.pending.pop(stamp_ns, None)
        if pending is not None:
            return ZoomFrame(pending.request_id, stamp_ns, pending.status, jpeg_bytes, pending.target)
        if self.config.process_stills or self.outstanding is not None:
            self.images[stamp_ns] = (jpeg_bytes, self.clock())
            while len(self.images) > MAX_BUFFERED_IMAGES:
                del self.images[next(iter(self.images))]
        return None

    def expire(self, now: float) -> None:
        """Drop an outstanding request and stale buffered stills after ``timeout_s``.

        Args:
            now: Current time in seconds.
        """
        timeout = self.config.timeout_s
        if self.outstanding is not None and now - self.outstanding[2] >= timeout:
            self.outstanding = None
        self.images = {k: v for k, v in self.images.items() if now - v[1] < timeout}
