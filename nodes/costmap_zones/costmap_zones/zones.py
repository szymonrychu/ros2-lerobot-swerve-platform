"""Active zones from the poi_store /poi/list JSON, and a digest that changes only when the zones change. No ROS."""

import hashlib
import json
from typing import Annotated, Any, Literal, NamedTuple

from pydantic import BaseModel, ConfigDict, Field, ValidationError, model_validator

ZONE_KEEPOUT = "keepout"
ZONE_SLOW = "slow"
ACTIVE_KIND = "area"
ACTIVE_STATUS = "open"
MIN_POLYGON_VERTICES = 3
MIN_SPEED_LIMIT_PCT = 10
MAX_SPEED_LIMIT_PCT = 100

Coordinate = Annotated[float, Field(allow_inf_nan=False)]
SpeedLimitPct = Annotated[int, Field(ge=MIN_SPEED_LIMIT_PCT, le=MAX_SPEED_LIMIT_PCT)]


class PoiListError(ValueError):
    """The /poi/list payload is not JSON {"pois": [...]}; the caller keeps its previous zones."""


class Zone(BaseModel):
    """One active zone (an open area POI with zone keepout or slow), map frame.

    Attributes:
        id: POI id.
        zone: "keepout" (no entry) or "slow" (speed cap).
        speed_limit_pct: Zone "slow": speed cap in percent (10..100); None for keepout.
        polygon: Vertices (x, y) in metres, at least 3.
    """

    model_config = ConfigDict(frozen=True)

    id: str
    zone: Literal["keepout", "slow"]
    speed_limit_pct: SpeedLimitPct | None = None
    polygon: tuple[tuple[Coordinate, Coordinate], ...] = Field(min_length=MIN_POLYGON_VERTICES)

    @model_validator(mode="after")
    def check_speed_limit(self) -> "Zone":
        """A slow zone needs speed_limit_pct; a keepout drops it.

        Returns:
            Zone: Self.

        Raises:
            ValueError: Slow zone without speed_limit_pct.
        """
        if self.zone == ZONE_SLOW:
            if self.speed_limit_pct is None:
                raise ValueError("zone 'slow' needs speed_limit_pct")
        elif self.speed_limit_pct is not None:
            object.__setattr__(self, "speed_limit_pct", None)
        return self


class PoiRecord(BaseModel):
    """The fields of one /poi/list entry that decide whether it is an active zone (others are ignored).

    Attributes:
        id: POI id.
        kind: "point", "area" or "object" (unknown kinds are kept and never active).
        status: "open", "done" or "cancelled".
        zone: "none", "keepout" or "slow"; missing (older poi_store) means "none".
        speed_limit_pct: Speed cap percent or None.
        polygon: Area vertices.
    """

    model_config = ConfigDict(extra="ignore")

    id: str
    kind: str
    status: str
    zone: Literal["none", "keepout", "slow"] = "none"
    speed_limit_pct: SpeedLimitPct | None = None
    polygon: list[tuple[Coordinate, Coordinate]] = Field(default_factory=list)


class ParsedPoiList(NamedTuple):
    """Result of parse_poi_list.

    Attributes:
        zones: Active zones, in list order.
        invalid: Entries skipped because they failed validation.
    """

    zones: list[Zone]
    invalid: int


def parse_entry(entry: Any) -> Zone | None:
    """Turn one /poi/list entry into an active zone.

    Args:
        entry (Any): Decoded JSON entry.

    Returns:
        Zone | None: The zone, or None when the entry is valid but not an active zone.

    Raises:
        ValidationError: The entry is malformed or an active zone is invalid.
    """
    record = PoiRecord.model_validate(entry)
    if record.kind != ACTIVE_KIND or record.status != ACTIVE_STATUS or record.zone == "none":
        return None
    return Zone(id=record.id, zone=record.zone, speed_limit_pct=record.speed_limit_pct, polygon=tuple(record.polygon))


def parse_poi_list(raw: str) -> ParsedPoiList:
    """Extract the active zones (kind area, zone keepout or slow, status open) from a /poi/list payload.

    Args:
        raw (str): JSON {"pois": [...], "revision": int}.

    Returns:
        ParsedPoiList: Active zones and the count of skipped invalid entries.

    Raises:
        PoiListError: The payload is not JSON or has no "pois" list; the caller keeps its previous zones.
    """
    try:
        data = json.loads(raw)
    except (json.JSONDecodeError, TypeError) as exc:
        raise PoiListError(f"/poi/list is not JSON: {exc}") from exc
    if not isinstance(data, dict) or not isinstance(data.get("pois"), list):
        raise PoiListError('/poi/list is not an object with a "pois" list')
    zones: list[Zone] = []
    invalid = 0
    for entry in data["pois"]:
        try:
            zone = parse_entry(entry)
        except ValidationError:
            invalid += 1
            continue
        if zone is not None:
            zones.append(zone)
    return ParsedPoiList(zones=zones, invalid=invalid)


def zones_digest(zones: list[Zone]) -> str:
    """Digest of the zone content (id, zone, speed limit, polygon), independent of the list order.

    Args:
        zones (list[Zone]): Active zones.

    Returns:
        str: sha256 hex digest.
    """
    rows = sorted(
        json.dumps([zone.id, zone.zone, zone.speed_limit_pct, zone.polygon], separators=(",", ":")) for zone in zones
    )
    return hashlib.sha256("\n".join(rows).encode()).hexdigest()
