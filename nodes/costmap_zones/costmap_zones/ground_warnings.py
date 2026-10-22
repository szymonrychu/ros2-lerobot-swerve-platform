"""The /ground/warning_areas input of costmap_zones: pydantic model of the JSON and a per-cell TTL store. No ROS.

Contract `ground-warning-areas` of ground-state-json (nodes/ground_estimator_cpp state.hpp
to_warning_areas_string): {"stamp_ns": int, "frame": str, "areas": [{kind, polygon, height_m, slope_deg, source}]}.
"""

import hashlib
import json
import time
from collections.abc import Callable
from typing import Any

import numpy as np
from pydantic import BaseModel, ConfigDict, Field, ValidationError

from costmap_zones.raster import MAX_YAW_RAD, MapGeometry, fill_zone, geometry_key

REASON_BAD_MESSAGE = "bad_message"
REASON_FRAME_MISMATCH = "frame_mismatch"
REASON_NO_MAP = "no_map"
MIN_POLYGON_VERTICES = 3
VERTEX_LENGTH = 2


class GroundArea(BaseModel):
    """One warning area (clear to ride through at reduced speed).

    Attributes:
        kind: Area kind from the ground node (step, ramp, colour, ...).
        polygon: Vertices [x, y] in the message frame, m (at least 3).
        height_m: Edge step height, m; null: not measured (kind no_return).
        slope_deg: Surface slope, deg; null: not measured (kind no_return).
        source: Sensor that produced it (tof, colour, ...).
    """

    model_config = ConfigDict(extra="forbid")

    kind: str
    polygon: list[tuple[float, float]] = Field(min_length=MIN_POLYGON_VERTICES)
    height_m: float | None
    slope_deg: float | None
    source: str


class GroundWarningMessage(BaseModel):
    """The /ground/warning_areas document.

    Attributes:
        stamp_ns: Source time, ns.
        frame: Frame of every polygon; must equal the /map header frame.
        areas: Warning areas; empty is a real "no warning" state.
    """

    model_config = ConfigDict(extra="forbid")

    stamp_ns: int
    frame: str = Field(min_length=1)
    areas: list[GroundArea]


def parse_warning_areas(data: str) -> GroundWarningMessage:
    """Parse and validate the JSON document.

    Args:
        data (str): std_msgs/String data.

    Returns:
        GroundWarningMessage: The validated message.

    Raises:
        ValueError: Not JSON or not matching the contract (pydantic ValidationError is a ValueError).
    """
    try:
        return GroundWarningMessage.model_validate(json.loads(data))
    except (json.JSONDecodeError, ValidationError) as exc:
        raise ValueError(f"bad /ground/warning_areas message: {exc}") from exc


class GroundWarningStore:
    """Keeps, per map cell, the monotonic time an accepted message last covered it (cells expire ttl_s later)."""

    def __init__(self, ttl_s: float, logger: Any, monotonic: Callable[[], float] = time.monotonic) -> None:
        """Create the store.

        Args:
            ttl_s (float): Seconds a map cell stays active after the last message that covered it.
            logger (Any): Object with a warning method.
            monotonic (Callable[[], float]): Monotonic clock, seconds.
        """
        self.ttl_s = ttl_s
        self.logger = logger
        self.monotonic = monotonic
        self.geometry_id: tuple | None = None
        self.expiry: np.ndarray = np.empty((0, 0), dtype=np.float64)
        self.area_count = 0
        self.warned_frames: set[str] = set()

    def use_geometry(self, geometry: MapGeometry) -> None:
        """Clear the cells when the map grid changed (they would point at other places).

        Args:
            geometry (MapGeometry): Current /map grid.
        """
        key = geometry_key(geometry)
        if key != self.geometry_id:
            self.geometry_id = key
            self.expiry = np.full((geometry.height, geometry.width), -np.inf, dtype=np.float64)
            self.area_count = 0

    def receive(self, data: str, map_frame: str | None, geometry: MapGeometry | None) -> str | None:
        """Parse a message and mark the map cells covered by its areas.

        Args:
            data (str): std_msgs/String data.
            map_frame (str | None): header.frame_id of the latest /map; None before the first map.
            geometry (MapGeometry | None): Grid of the latest /map; None before the first map.

        Returns:
            str | None: None when stored, else the ignore reason (REASON_*).
        """
        try:
            message = parse_warning_areas(data)
        except ValueError as exc:
            self.logger.warning(str(exc))
            return REASON_BAD_MESSAGE
        if map_frame is None or geometry is None:
            return REASON_NO_MAP
        if message.frame != map_frame:
            if message.frame not in self.warned_frames:
                self.warned_frames.add(message.frame)
                self.logger.warning(f"ignoring ground warning areas in frame {message.frame!r}, map is {map_frame!r}")
            return REASON_FRAME_MISMATCH
        self.use_geometry(geometry)
        self.area_count = len(message.areas)
        if abs(geometry.origin_yaw) <= MAX_YAW_RAD:  # a rotated map gets no masks (RotatedMapError in the node)
            expires = self.monotonic() + self.ttl_s
            for area in message.areas:
                covered = fill_zone(geometry, tuple(area.polygon))
                self.expiry[covered] = expires
        return None

    def active_cells(self, geometry: MapGeometry) -> np.ndarray:
        """Drop expired cells and return the active ones.

        Args:
            geometry (MapGeometry): Current /map grid (a change clears the store).

        Returns:
            np.ndarray: bool array (height, width), True for cells covered less than ttl_s ago.
        """
        self.use_geometry(geometry)
        active = self.expiry > self.monotonic()
        self.expiry[~active] = -np.inf
        if not active.any():
            self.area_count = 0
        return active


def active_key(cells: np.ndarray) -> bytes:
    """Digest of an active cell mask.

    Args:
        cells (np.ndarray): bool array (height, width).

    Returns:
        bytes: blake2b digest of the shape and the packed cells.
    """
    return hashlib.blake2b(np.packbits(cells).tobytes() + repr(cells.shape).encode(), digest_size=16).digest()
