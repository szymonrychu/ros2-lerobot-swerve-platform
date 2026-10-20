"""The /ground/warning_areas input of costmap_zones: pydantic model of the JSON and a TTL store. No ROS.

Contract `ground-warning-areas` of ground-state-json (nodes/ground_estimator_cpp state.hpp
to_warning_areas_string): {"stamp_ns": int, "frame": str, "areas": [{kind, polygon, height_m, slope_deg, source}]}.
"""

import json
import time
from collections.abc import Callable
from typing import Any

from pydantic import BaseModel, ConfigDict, Field, ValidationError

REASON_BAD_MESSAGE = "bad_message"
REASON_FRAME_MISMATCH = "frame_mismatch"
REASON_NO_MAP = "no_map"
DIGEST_DECIMALS = 3
MIN_POLYGON_VERTICES = 3
VERTEX_LENGTH = 2

PolygonT = tuple[tuple[float, float], ...]
AreaKey = tuple[str, PolygonT]


class GroundArea(BaseModel):
    """One warning area (clear to ride through at reduced speed).

    Attributes:
        kind: Area kind from the ground node (step, ramp, colour, ...).
        polygon: Vertices [x, y] in the message frame, m (at least 3).
        height_m: Edge step height, m.
        slope_deg: Surface slope, deg.
        source: Sensor that produced it (tof, colour, ...).
    """

    model_config = ConfigDict(extra="forbid")

    kind: str
    polygon: list[tuple[float, float]] = Field(min_length=MIN_POLYGON_VERTICES)
    height_m: float
    slope_deg: float
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


def area_key(area: GroundArea) -> AreaKey:
    """Identity of an area for change detection (polygon rounded to mm).

    Args:
        area (GroundArea): Area.

    Returns:
        AreaKey: (kind, rounded polygon).
    """
    return (area.kind, tuple((round(x, DIGEST_DECIMALS), round(y, DIGEST_DECIMALS)) for x, y in area.polygon))


class GroundWarningStore:
    """Keeps every received message's areas until ttl_s after receipt (monotonic clock)."""

    def __init__(self, ttl_s: float, logger: Any, monotonic: Callable[[], float] = time.monotonic) -> None:
        """Create the store.

        Args:
            ttl_s (float): Seconds an area stays active after the message that carried it was received.
            logger (Any): Object with a warning method.
            monotonic (Callable[[], float]): Monotonic clock, seconds.
        """
        self.ttl_s = ttl_s
        self.logger = logger
        self.monotonic = monotonic
        self.entries: list[tuple[float, list[GroundArea]]] = []
        self.warned_frames: set[str] = set()

    def receive(self, data: str, map_frame: str | None) -> str | None:
        """Parse a message and store its areas.

        Args:
            data (str): std_msgs/String data.
            map_frame (str | None): header.frame_id of the latest /map; None before the first map.

        Returns:
            str | None: None when stored, else the ignore reason (REASON_*).
        """
        try:
            message = parse_warning_areas(data)
        except ValueError as exc:
            self.logger.warning(str(exc))
            return REASON_BAD_MESSAGE
        if map_frame is None:
            return REASON_NO_MAP
        if message.frame != map_frame:
            if message.frame not in self.warned_frames:
                self.warned_frames.add(message.frame)
                self.logger.warning(f"ignoring ground warning areas in frame {message.frame!r}, map is {map_frame!r}")
            return REASON_FRAME_MISMATCH
        self.entries.append((self.monotonic() + self.ttl_s, message.areas))
        return None

    def active(self) -> list[GroundArea]:
        """Drop expired messages and return the areas of the rest.

        Returns:
            list[GroundArea]: Areas of messages received less than ttl_s ago.
        """
        now = self.monotonic()
        self.entries = [entry for entry in self.entries if entry[0] > now]
        return [area for _, areas in self.entries for area in areas]


def active_key(areas: list[GroundArea]) -> frozenset[AreaKey]:
    """Order-free identity of an active area set.

    Args:
        areas (list[GroundArea]): Active areas.

    Returns:
        frozenset[AreaKey]: Set of area keys.
    """
    return frozenset(area_key(area) for area in areas)
