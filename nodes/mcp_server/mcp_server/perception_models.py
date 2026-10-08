"""Result models of the perception/memory tools (structured tool output)."""

from typing import Any

from pydantic import BaseModel, Field


class ObjectRecord(BaseModel):
    """A remembered object (map frame), stored as a POI of kind 'object' in poi_store."""

    id: str
    label: str
    x: float
    y: float
    note: str = ""
    confidence: float = Field(ge=0.0, le=1.0)
    first_seen: float = Field(description="Unix seconds of the first sighting")
    last_seen: float = Field(description="Unix seconds of the latest sighting")
    times_seen: int = Field(ge=1)


class ObjectView(ObjectRecord):
    """ObjectRecord seen from the robot: distance and bearing (null while the robot pose is unknown)."""

    distance_m: float | None = None
    bearing_deg: float | None = Field(default=None, description="Relative to the robot heading: 0 ahead, + left")


class ObjectList(BaseModel):
    """list_objects result."""

    objects: list[ObjectView]
    robot_pose: dict[str, float] | None = None
    notes: list[str] = Field(default_factory=list)


class PoiView(BaseModel):
    """A POI plus its distance and bearing from the robot (null while the robot pose is unknown)."""

    poi: dict[str, Any]
    id: str
    name: str
    kind: str
    status: str
    distance_m: float | None = Field(default=None, description="From the robot to the POI position (area: centroid)")
    bearing_deg: float | None = Field(default=None, description="Relative to the robot heading: 0 ahead, + left")
    inside: bool | None = Field(default=None, description="Areas only: whether the robot stands inside the polygon")


class PoiList(BaseModel):
    """list_pois result."""

    pois: list[PoiView]
    revision: int
    notes: list[str] = Field(default_factory=list)


class PoiResult(BaseModel):
    """add_poi / update_poi / delete_poi result: the poi_store answer."""

    ok: bool
    message: str
    poi: dict[str, Any] | None = None


class HeadingSummary(BaseModel):
    """What look_around saw at one stop."""

    index: int
    heading_deg: int = Field(description="Degrees turned counter-clockwise from the start heading")
    label: str
    nearest_m: float | None = Field(description="Nearest lidar return in any direction (m from base_link)")
    nearest_sector: str | None = None
    sectors: dict[str, float | None] = Field(default_factory=dict, description="Nearest return per 45 deg sector")
    image_captured: bool = True


class StepRecord(BaseModel):
    """One rotation step of look_around."""

    index: int
    heading_deg: int
    status: str
    expected_yaw: float | None = None
    achieved_yaw: float | None = None
    message: str = ""


class LookAroundResult(BaseModel):
    """look_around result (the images travel as content blocks)."""

    status: str = Field(description="completed, interrupted, failed, stopped or aborted_obstacle")
    message: str = ""
    returned_to_start: bool = False
    interrupted_by: str | None = Field(default=None, description="Critical robot event that ended the turn early")
    expected: dict[str, Any] = Field(default_factory=dict)
    achieved: dict[str, Any] = Field(default_factory=dict)
    headings: list[HeadingSummary] = Field(default_factory=list)
    steps: list[StepRecord] = Field(default_factory=list)
    notes: list[str] = Field(default_factory=list)
    topdown: dict[str, Any] | None = None
    duration_s: float = 0.0


class TopdownMeta(BaseModel):
    """get_topdown_view metadata (the image travels as a content block)."""

    pose: dict[str, float] | None = Field(description="Robot pose in the map frame (x, y, yaw)")
    scale_m_per_px: float
    radius_m: float
    px: int
    orientation: str
    scale_bar_m: float = 0.5
    layers_requested: list[str]
    layers_present: list[str]
    layers_missing: dict[str, str] = Field(description="Layer -> why it is not drawn (never fabricated)")
    data_ages: dict[str, float] = Field(default_factory=dict, description="Age in seconds per data source")
