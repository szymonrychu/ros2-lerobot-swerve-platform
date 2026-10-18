"""Named maps topic contract of the map_manager node (no ROS imports).

map_manager, web_ui and mcp_server share these topic names and pydantic models. One operation runs at a time: a
second command while busy is answered ``MapResult(ok=False, message="busy: <op> <name>")``.
"""

from __future__ import annotations

import math
import re
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

COMMAND_TOPIC = "/map_manager/command"
RESULT_TOPIC = "/map_manager/result"
STATUS_TOPIC = "/map_manager/status"
LOCALISED_TOPIC = "/map_manager/localised"

MAP_NAME_PATTERN = r"^[A-Za-z0-9][A-Za-z0-9_-]{0,39}$"
MAP_NAME_RULE = "1-40 characters: letters, digits, '_' and '-', starting with a letter or digit"
MAX_NOTE_CHARS = 500
NAMED_OPS = ("save", "load", "delete")

MapOp = Literal["save", "load", "delete", "relocalise"]
LocalisationState = Literal["unknown", "loading", "localised", "not_localised", "ambiguous"]

_NAME_RE = re.compile(MAP_NAME_PATTERN)


def validate_map_name(name: str) -> str:
    """Check a map name against ``MAP_NAME_PATTERN``.

    Args:
        name: Candidate map name (str).

    Returns:
        str: The unchanged name.

    Raises:
        ValueError: The name breaks the rule (message names the rule).
    """
    if not isinstance(name, str) or not _NAME_RE.fullmatch(name):
        raise ValueError(f"invalid map name {name!r}: must be {MAP_NAME_RULE}")
    return name


class Pose2D(BaseModel):
    """Planar pose in the map frame: ``x``, ``y`` in metres, ``yaw`` in radians; all finite."""

    x: float
    y: float
    yaw: float

    @field_validator("x", "y", "yaw")
    @classmethod
    def finite(cls, value: float) -> float:
        """Reject NaN and infinity."""
        if not math.isfinite(value):
            raise ValueError("must be finite")
        return value


class MapCommand(BaseModel):
    """A command on ``COMMAND_TOPIC``.

    ``name`` is required and validated for save/load/delete and must be empty for relocalise. ``pose`` is load
    only, ``note`` and ``overwrite`` are save only (overwrite is refused on relocalise), ``since`` and
    ``discard_on_fail`` are relocalise only.
    """

    model_config = ConfigDict(extra="forbid")

    op: MapOp
    request_id: str = ""
    name: str = ""
    overwrite: bool = False
    pose: Pose2D | None = None
    note: str = Field(default="", max_length=MAX_NOTE_CHARS)
    since: float | None = None
    discard_on_fail: bool = False

    @model_validator(mode="after")
    def check_op_fields(self) -> MapCommand:
        """Enforce which fields each op accepts."""
        if self.op in NAMED_OPS:
            validate_map_name(self.name)
        elif self.name:
            raise ValueError("relocalise takes no map name")
        if self.pose is not None and self.op != "load":
            raise ValueError("pose is only allowed on load")
        if self.note and self.op != "save":
            raise ValueError("note is only allowed on save")
        if self.op == "relocalise" and self.overwrite:
            raise ValueError("overwrite is not allowed on relocalise")
        if self.op != "relocalise" and (self.since is not None or self.discard_on_fail):
            raise ValueError("since and discard_on_fail are only allowed on relocalise")
        if self.since is not None and not math.isfinite(self.since):
            raise ValueError("since must be finite")
        return self


class MapResult(BaseModel):
    """Answer on ``RESULT_TOPIC``; a load answers after the relocalisation verdict."""

    request_id: str
    ok: bool
    message: str
    data: dict[str, Any] | None = None


class Localisation(BaseModel):
    """Relocalisation verdict; ``ratio`` (0..1) and ``pose`` are None until measured."""

    state: LocalisationState
    ratio: float | None = Field(default=None, ge=0.0, le=1.0)
    pose: Pose2D | None = None
    message: str = ""
    updated_at: float | None = None


class MapSummary(BaseModel):
    """One saved map; ``error`` is non-empty only for an invalid bundle."""

    name: str
    saved_at: float
    note: str
    poi_count: int
    last_pose: Pose2D
    valid: bool
    error: str = ""


class MapStatus(BaseModel):
    """Latched status on ``STATUS_TOPIC``."""

    maps: list[MapSummary]
    active_map: str | None
    load_seq: int = Field(ge=0)
    localisation: Localisation
