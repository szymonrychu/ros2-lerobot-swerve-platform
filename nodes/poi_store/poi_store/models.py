"""Pydantic models of the POI contract (map frame) and of the /poi/command messages."""

import re
from typing import Annotated, Any, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

ID_PATTERN = re.compile(r"^[0-9a-f]{32}$")
MAX_NAME_LEN = 60
MAX_NOTE_LEN = 2000
MIN_POLYGON_VERTICES = 3
DEFAULT_RADIUS_M = 0.2
AREA_EPSILON = 1e-12

Coordinate = Annotated[float, Field(allow_inf_nan=False)]
Timestamp = Annotated[float, Field(allow_inf_nan=False, ge=0)]


def polygon_centroid(polygon: list[tuple[float, float]]) -> tuple[float, float]:
    """Area centroid of a simple polygon; the vertex mean when the polygon is degenerate (zero area).

    Args:
        polygon: Vertices (x, y), at least one.

    Returns:
        tuple[float, float]: Centroid (x, y).
    """
    twice_area = 0.0
    cx = 0.0
    cy = 0.0
    for i, (x0, y0) in enumerate(polygon):
        x1, y1 = polygon[(i + 1) % len(polygon)]
        cross = x0 * y1 - x1 * y0
        twice_area += cross
        cx += (x0 + x1) * cross
        cy += (y0 + y1) * cross
    if abs(twice_area) < AREA_EPSILON:
        n = len(polygon)
        return sum(p[0] for p in polygon) / n, sum(p[1] for p in polygon) / n
    return cx / (3.0 * twice_area), cy / (3.0 * twice_area)


class Poi(BaseModel):
    """One point or area of interest in the map frame.

    Attributes:
        id: uuid4 hex; empty only before the store assigns it.
        kind: "point", "area" or "object" (a remembered object: a point-like POI with sighting fields).
        frame: Always "map".
        x: Point position, or the polygon centroid for an area (computed), m.
        y: As x.
        polygon: Area vertices [[x, y], ...] (>= 3); empty for a point.
        radius_m: Point radius, m (> 0); kept at the default for an area.
        name: Short label.
        note: Free text.
        status: "open", "done" or "cancelled".
        created_by: "agent" or "user".
        created_at: Unix seconds; 0 before the store assigns it.
        updated_at: Unix seconds; 0 before the store assigns it.
        times_seen: Objects: how often it was sighted (0 for other kinds).
        first_seen: Objects: unix seconds of the first sighting (0 for other kinds).
        last_seen: Objects: unix seconds of the latest sighting (0 for other kinds).
        confidence: Objects: how sure the sighting was, 0..1 (1 for other kinds).
    """

    model_config = ConfigDict(extra="ignore")

    id: str = ""
    kind: Literal["point", "area", "object"]
    frame: Literal["map"] = "map"
    x: Coordinate = 0.0
    y: Coordinate = 0.0
    polygon: list[tuple[Coordinate, Coordinate]] = Field(default_factory=list)
    radius_m: Annotated[float, Field(allow_inf_nan=False, gt=0)] = DEFAULT_RADIUS_M
    name: str = Field(default="", max_length=MAX_NAME_LEN)
    note: str = Field(default="", max_length=MAX_NOTE_LEN)
    status: Literal["open", "done", "cancelled"] = "open"
    created_by: Literal["agent", "user"] = "agent"
    created_at: Timestamp = 0.0
    updated_at: Timestamp = 0.0
    times_seen: Annotated[int, Field(ge=0)] = 0
    first_seen: Timestamp = 0.0
    last_seen: Timestamp = 0.0
    confidence: Annotated[float, Field(allow_inf_nan=False, ge=0, le=1)] = 1.0

    @field_validator("id")
    @classmethod
    def check_id(_cls, value: str) -> str:
        """Require an empty id or 32 lowercase hex characters.

        Args:
            value: Candidate id.

        Returns:
            str: The id.
        """
        if value and not ID_PATTERN.match(value):
            raise ValueError("id must be a 32-character lowercase hex string")
        return value

    @model_validator(mode="after")
    def check_geometry(self) -> "Poi":
        """Enforce the per-kind geometry: a point or object has no polygon, an area has >= 3
        vertices and a computed centroid.

        Returns:
            Poi: Self.
        """
        if self.kind != "area":
            self.polygon = []
        else:
            if len(self.polygon) < MIN_POLYGON_VERTICES:
                raise ValueError(f"area needs at least {MIN_POLYGON_VERTICES} polygon vertices")
            self.x, self.y = polygon_centroid(self.polygon)
        return self

    def to_json_dict(self) -> dict[str, Any]:
        """Contract JSON form (polygon as lists).

        Returns:
            dict[str, Any]: JSON-serialisable dict.
        """
        return self.model_dump(mode="json")


class Command(BaseModel):
    """A /poi/command message.

    Attributes:
        op: "add", "update", "delete" or "clear".
        request_id: Echoed in /poi/result.
        poi: add: the POI; update: id plus changed fields; delete: id; clear: unused.
        created_by: clear: delete every POI created by this creator ("agent" or "user").
    """

    model_config = ConfigDict(extra="ignore")

    op: Literal["add", "update", "delete", "clear"]
    request_id: str = ""
    poi: dict[str, Any] = Field(default_factory=dict)
    created_by: Literal["agent", "user"] | None = None

    @model_validator(mode="after")
    def check_clear_creator(self) -> "Command":
        """Require created_by for the clear op.

        Returns:
            Command: Self.
        """
        if self.op == "clear" and self.created_by is None:
            raise ValueError("clear needs created_by ('agent' or 'user')")
        return self
