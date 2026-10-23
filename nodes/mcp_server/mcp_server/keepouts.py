"""Person-drawn keep-out zones from the poi list (no ROS): parsing, point-in-polygon and a memory of the last zones.

A keep-out is an open POI of kind area with zone "keepout", parsed as nodes/costmap_zones/costmap_zones/zones.py does.
"""

import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any, Protocol

import numpy as np

from .models import RobotError

ZONE_KEEPOUT = "keepout"
MIN_POLYGON_VERTICES = 3


@dataclass(frozen=True)
class KeepOut:
    """One person-drawn keep-out area (an open area POI with zone "keepout"), map frame.

    Attributes:
        id (str): POI id.
        polygon (tuple[tuple[float, float], ...]): Vertices (x, y) in metres, at least 3.
    """

    id: str
    polygon: tuple[tuple[float, float], ...]


def parse_keepouts(pois: Sequence[dict[str, Any]]) -> list[KeepOut]:
    """Active keep-out zones of a /poi/list, parsed as costmap_zones does (open, kind area, zone keepout, >= 3 vertices).

    Args:
        pois (Sequence[dict[str, Any]]): POI JSON objects.

    Returns:
        list[KeepOut]: One per valid zone in list order; malformed entries are skipped.
    """
    zones: list[KeepOut] = []
    for poi in pois:
        if poi.get("kind") != "area" or poi.get("status") != "open" or poi.get("zone") != ZONE_KEEPOUT:
            continue
        raw = poi.get("polygon")
        if not isinstance(raw, list) or len(raw) < MIN_POLYGON_VERTICES:
            continue
        try:
            polygon = tuple((float(v[0]), float(v[1])) for v in raw)
        except (TypeError, ValueError, IndexError):
            continue
        if all(map(math.isfinite, (c for v in polygon for c in v))):
            zones.append(KeepOut(id=str(poi.get("id", "")), polygon=polygon))
    return zones


def points_in_polygon(xs: np.ndarray, ys: np.ndarray, polygon: Sequence[Sequence[float]]) -> np.ndarray:
    """Vectorised ray casting point-in-polygon test (same rule as poi_client.point_in_polygon).

    Args:
        xs (np.ndarray): Point x values, shape N.
        ys (np.ndarray): Point y values, shape N.
        polygon (Sequence[Sequence[float]]): Vertices [[x, y], ...].

    Returns:
        np.ndarray: bool array of shape N, True where the point is inside.
    """
    inside = np.zeros(np.shape(xs), dtype=bool)
    j = len(polygon) - 1
    for i, (xi, yi) in enumerate(polygon):
        xj, yj = polygon[j]
        if yi != yj:
            crosses = (yi > ys) != (yj > ys)
            inside ^= crosses & (xs < (xj - xi) * (ys - yi) / (yj - yi) + xi)
        j = i
    return inside


class PoiSource(Protocol):
    """What KeepoutMemory reads from the robot (RobotApi provides it)."""

    def poi_list(self) -> tuple[list[dict[str, Any]], int]:
        """Latest /poi/list (POIs, revision)."""
        ...

    def poi_list_age_s(self) -> float | None:
        """Seconds since /poi/list was last received, None when never."""
        ...


class KeepoutMemory:
    """Keep-out zones from the poi list, remembered so a stale or missing list keeps the last zones in force."""

    def __init__(self) -> None:
        """Start with no remembered zones (None: no list was ever read)."""
        self.zones: tuple[KeepOut, ...] | None = None

    def read(self, source: PoiSource, stale_s: float) -> tuple[tuple[KeepOut, ...], list[str]]:
        """Current zones and the status notes explaining any fallback to remembered zones.

        Args:
            source (PoiSource): poi list provider.
            stale_s (float): A poi list older than this keeps the remembered zones.

        Returns:
            tuple[tuple[KeepOut, ...], list[str]]: Zones in force (empty without a list) and notes.
        """
        try:
            pois, _ = source.poi_list()
            age = source.poi_list_age_s()
        except RobotError as exc:
            if self.zones is None:
                return (), []
            return self.zones, [
                f"keep-out zones: poi list unavailable ({exc}); {len(self.zones)} zone(s) from the last list still apply"
            ]
        if age is not None and age > stale_s and self.zones is not None:
            return self.zones, [
                f"keep-out zones: poi list is stale ({age:.0f} s old, limit {stale_s:g} s); {len(self.zones)} zone(s) from the last list still apply"
            ]
        self.zones = tuple(parse_keepouts(pois))
        return self.zones, []
