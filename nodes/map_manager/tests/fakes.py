"""In-memory fakes of the map_manager ports with one shared call log, plus room geometry for the candidate search.

Every fake appends ``(name, args)`` to the shared :class:`CallLog`, so a test can assert the order of calls across
slam_toolbox, poi_store and the sensors.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from ros2_common.maps import Pose2D
from ros2_common.scan_agreement import GridMap

RESOLUTION = 0.05
LASER_FRAME = "laser_frame"
POSEGRAPH_SUFFIXES = (".posegraph", ".data")


@dataclass
class CallLog:
    """Ordered record of port calls.

    Attributes:
        calls: ``(name, args)`` tuples in call order, e.g. ``("slam.deserialize", (base, pose))``.
    """

    calls: list[tuple[str, tuple[Any, ...]]] = field(default_factory=list)

    def add(self, name: str, *args: Any) -> None:
        """Record one call.

        Args:
            name (str): ``<port>.<method>``.
            *args (Any): Call arguments worth asserting.
        """
        self.calls.append((name, args))

    def names(self) -> list[str]:
        """Call names in order.

        Returns:
            list[str]: The names.
        """
        return [name for name, _ in self.calls]

    def args_of(self, name: str) -> list[tuple[Any, ...]]:
        """Arguments of every call with that name.

        Args:
            name (str): Call name.

        Returns:
            list[tuple[Any, ...]]: Arguments in call order.
        """
        return [args for call, args in self.calls if call == name]


@dataclass
class FakeClock:
    """Monotonic fake time that advances by ``step`` on every read.

    Attributes:
        now: Current time, s.
        step: Advance per read, s.
    """

    now: float = 1_700_000_000.0
    step: float = 0.5

    def __call__(self) -> float:
        """Read and advance the time.

        Returns:
            float: The time before the advance, s.
        """
        value = self.now
        self.now += self.step
        return value


@dataclass
class FakeSlam:
    """slam_toolbox fake: serialize writes the pose graph files, pause is a real toggle.

    Attributes:
        log: Shared call log.
        serialize_ok: serialize writes the files and answers True.
        deserialize_ok: Answer of deserialize.
        paused: Current new-measurement pause state.
        pause_readable: query_paused returns the state (else None).
        toggle_ok: toggle_pause flips the state and answers True (else changes nothing and answers False).
        confirmed_pose: Pose returned by wait_pose_after (None: the load is never confirmed).
        covariance: Diagonal returned by pose_covariance.
        on_deserialize: Hook run inside deserialize (used to issue a concurrent command).
        deserialize_error: Raised by deserialize after recording the call (None: answer deserialize_ok).
    """

    log: CallLog
    serialize_ok: bool = True
    deserialize_ok: bool = True
    paused: bool = False
    pause_readable: bool = True
    toggle_ok: bool = True
    confirmed_pose: Pose2D | None = None
    covariance: list[float] | None = field(default_factory=lambda: [0.01, 0.02, 0.003])
    on_deserialize: Any = None
    deserialize_error: Exception | None = None

    def serialize(self, base: str) -> bool:
        """Write ``<base>.posegraph`` and ``<base>.data`` when serialize_ok."""
        self.log.add("slam.serialize", base)
        if not self.serialize_ok:
            return False
        for suffix in POSEGRAPH_SUFFIXES:
            Path(base + suffix).write_bytes(b"graph")
        return True

    def deserialize(self, base: str, pose: Pose2D) -> bool:
        """Record the call and answer deserialize_ok."""
        self.log.add("slam.deserialize", base, pose)
        if self.on_deserialize is not None:
            self.on_deserialize()
        if self.deserialize_error is not None:
            raise self.deserialize_error
        return self.deserialize_ok

    def query_paused(self) -> bool | None:
        """Pause state, or None when unreadable."""
        self.log.add("slam.query_paused")
        return self.paused if self.pause_readable else None

    def toggle_pause(self) -> bool:
        """Flip the pause state when toggle_ok."""
        self.log.add("slam.toggle_pause")
        if not self.toggle_ok:
            return False
        self.paused = not self.paused
        return True

    def wait_pose_after(self, stamp: float, timeout_s: float) -> Pose2D | None:
        """The confirmed pose (None: not confirmed)."""
        self.log.add("slam.wait_pose_after", stamp, timeout_s)
        return self.confirmed_pose

    def pose_covariance(self) -> list[float] | None:
        """Covariance diagonal of the last pose."""
        self.log.add("slam.pose_covariance")
        return self.covariance


@dataclass
class FakePoi:
    """poi_store fake.

    Attributes:
        log: Shared call log.
        pois: Live POI set (None: no /poi/list received yet).
        revision: Live revision.
        replace_ok: replace swaps the set and answers ok.
    """

    log: CallLog
    pois: list[dict[str, Any]] | None = field(default_factory=lambda: [{"id": "live-1", "name": "sofa"}])
    revision: int = 3
    replace_ok: bool = True

    def latest_list(self) -> tuple[list[dict[str, Any]], int] | None:
        """Live POIs and revision, None before the first list."""
        self.log.add("poi.latest_list")
        if self.pois is None:
            return None
        return [dict(poi) for poi in self.pois], self.revision

    def replace(self, pois: list[dict[str, Any]]) -> tuple[bool, str]:
        """Swap the live set when replace_ok."""
        self.log.add("poi.replace", [dict(poi) for poi in pois])
        if not self.replace_ok:
            return False, "replace rejected: store write failed"
        self.pois = [dict(poi) for poi in pois]
        self.revision += 1
        return True, f"replaced with {len(pois)} POIs"


@dataclass
class FakeSensors:
    """TF, /map, /scan_filtered and agreement fake.

    Attributes:
        log: Shared call log.
        pose: Robot pose (None: TF missing or stale).
        grid: Latest /map (None: none received).
        points: Scan endpoints in the base frame (None: no scan).
        samples: Agreement samples returned for the verify window.
        frame: Laser frame (None: unknown).
        active: Between start_operation and end_operation.
    """

    log: CallLog
    pose: Pose2D | None = None
    grid: GridMap | None = None
    points: np.ndarray | None = None
    samples: list[float] = field(default_factory=lambda: [0.9] * 6)
    frame: str | None = LASER_FRAME
    active: bool = False

    def robot_pose(self) -> Pose2D | None:
        """Robot pose or None."""
        self.log.add("sensors.robot_pose")
        return self.pose

    def latest_grid(self) -> GridMap | None:
        """Latest grid or None."""
        self.log.add("sensors.latest_grid")
        return self.grid

    def scan_points_base(self) -> np.ndarray | None:
        """Scan points or None."""
        self.log.add("sensors.scan_points_base")
        return self.points

    def agreement_samples(self, window_s: float) -> list[float]:
        """The configured samples."""
        self.log.add("sensors.agreement_samples", window_s)
        return list(self.samples)

    def laser_frame(self) -> str | None:
        """The laser frame or None."""
        self.log.add("sensors.laser_frame")
        return self.frame

    def start_operation(self) -> None:
        """Mark the heavy subscriptions as created."""
        self.log.add("sensors.start_operation")
        self.active = True

    def end_operation(self) -> None:
        """Mark the heavy subscriptions as destroyed."""
        self.log.add("sensors.end_operation")
        self.active = False


def room_grid(width_m: float, height_m: float, margin_m: float = 0.5) -> GridMap:
    """Axis-aligned rectangular room: free interior, one-cell occupied wall, unknown outside.

    Args:
        width_m (float): Interior width along x, m.
        height_m (float): Interior height along y, m.
        margin_m (float): Unknown margin around the walls, m.

    Returns:
        GridMap: Grid with the room's interior lower-left corner at (0, 0).
    """
    margin = round(margin_m / RESOLUTION)
    inner_w = round(width_m / RESOLUTION)
    inner_h = round(height_m / RESOLUTION)
    cells = np.full((inner_h + 2 * margin + 2, inner_w + 2 * margin + 2), -1, dtype=np.int8)
    cells[margin : margin + inner_h + 2, margin : margin + inner_w + 2] = 100
    cells[margin + 1 : margin + inner_h + 1, margin + 1 : margin + inner_w + 1] = 0
    origin = -(margin + 1) * RESOLUTION
    return GridMap(resolution=RESOLUTION, origin_x=origin, origin_y=origin, origin_yaw=0.0, cells=cells)


def wall_points_base(grid: GridMap, pose: Pose2D) -> np.ndarray:
    """Every occupied cell centre of the grid seen from pose, in the base frame (a convex room sees all walls).

    Args:
        grid (GridMap): The room.
        pose (Pose2D): Robot pose in the map frame.

    Returns:
        np.ndarray: (N, 2) endpoints in the base frame, m.
    """
    rows, cols = np.nonzero(grid.cells >= 100)
    xs = grid.origin_x + (cols + 0.5) * grid.resolution - pose.x
    ys = grid.origin_y + (rows + 0.5) * grid.resolution - pose.y
    cos_yaw, sin_yaw = math.cos(pose.yaw), math.sin(pose.yaw)
    return np.column_stack((cos_yaw * xs + sin_yaw * ys, -sin_yaw * xs + cos_yaw * ys))
