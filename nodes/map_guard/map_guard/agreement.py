"""Scan-to-map agreement for map_guard: the pure functions live in ros2_common.scan_agreement (shared with
map_manager); they are re-exported here, and MismatchMonitor (which needs map_guard's HoldTimer) stays."""

import statistics
from collections import deque

from ros2_common.scan_agreement import (
    UNKNOWN,
    AgreementResult,
    GridMap,
    PreparedMap,
    agreement,
    dilate,
    prepare_map,
    quaternion_matrix,
    scan_endpoints,
    transform_points,
    yaw_quaternion,
)

from .timing import HoldTimer

__all__ = [
    "UNKNOWN",
    "AgreementResult",
    "GridMap",
    "MismatchMonitor",
    "PreparedMap",
    "agreement",
    "dilate",
    "prepare_map",
    "quaternion_matrix",
    "scan_endpoints",
    "transform_points",
    "yaw_quaternion",
]


class MismatchMonitor:
    """Mismatch: the rolling median ratio stays below mismatch_ratio for mismatch_s on an established map.

    A map only counts once it has min_map_cells_known known cells and the median was at least healthy_ratio on it
    (armed), so a fresh or empty map never triggers.
    """

    def __init__(
        self, mismatch_ratio: float, healthy_ratio: float, mismatch_s: float, min_map_cells_known: int, window: int
    ) -> None:
        """Create the monitor.

        Args:
            mismatch_ratio (float): Median below this is a mismatch.
            healthy_ratio (float): Median at or above this arms the monitor.
            mismatch_s (float): Time the mismatch must hold, s.
            min_map_cells_known (int): Known cells needed before a mismatch can count.
            window (int): Samples in the rolling median.
        """
        self.mismatch_ratio = mismatch_ratio
        self.healthy_ratio = healthy_ratio
        self.min_map_cells_known = min_map_cells_known
        self.samples: deque[float] = deque(maxlen=window)
        self.hold = HoldTimer(mismatch_s)
        self.armed = False

    @property
    def median(self) -> float | None:
        """Rolling median of the ratio, None before the first sample."""
        return statistics.median(self.samples) if self.samples else None

    def update(self, now: float, ratio: float, known_cells: int) -> bool:
        """Add one agreement sample.

        Args:
            now (float): Sample time, s.
            ratio (float): Agreement ratio.
            known_cells (int): Known cells of the map the sample was scored against.

        Returns:
            bool: True while the mismatch condition has held for mismatch_s.
        """
        self.samples.append(ratio)
        median = self.median
        assert median is not None
        big_enough = known_cells >= self.min_map_cells_known
        if big_enough and median >= self.healthy_ratio:
            self.armed = True
        return self.hold.update(now, self.armed and big_enough and median < self.mismatch_ratio)

    def reset_hold(self) -> None:
        """Restart the mismatch hold (after the map was found healthy again)."""
        self.hold.reset()

    def reset(self) -> None:
        """Forget everything (the map was reset): disarmed until the new map is healthy."""
        self.samples.clear()
        self.hold.reset()
        self.armed = False
