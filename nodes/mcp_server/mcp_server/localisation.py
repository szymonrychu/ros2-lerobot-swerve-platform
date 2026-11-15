"""Localisation health from map_guard_cpp's /map_guard/state (std_msgs/String JSON, latched; no ROS imports).

The guard publishes its state on every transition and about once a second. get_robot_state reports it as the
``localisation`` object with a ``reliable`` verdict, so the agent sees a bad map pose (a robot carried or turned by hand)
before it navigates. A missing or stale state is never invented: the object is null and a note says why.
"""

import logging
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, ValidationError

from .models import LocalisationHealth

LOGGER = logging.getLogger(__name__)
GuardStateName = Literal["ok", "disturbed", "settling", "evaluating", "relocalising", "resumed", "reset"]
# map_guard states in which the map pose is being questioned or repaired.
UNRELIABLE_STATES = frozenset({"disturbed", "settling", "evaluating", "relocalising"})
NAV_ADVICE = (
    "Do not use navigate_to_pose or move_relative until it is reliable: call relocalise (moves nothing), or ask a "
    "person to place the robot or confirm where it is."
)
MISSING_NOTE = "no /map_guard/state received (map_guard_cpp not running?): localisation unknown"
STALE_NOTE = "/map_guard/state stale ({age:.1f} s): localisation unknown"


class MapGuardStatus(BaseModel):
    """One /map_guard/state message (map_guard_cpp state_wire.hpp)."""

    model_config = ConfigDict(extra="ignore")

    state: GuardStateName
    reasons: list[str] = Field(default_factory=list)
    agreement_ratio: float | None = Field(default=None, ge=0.0, le=1.0)
    ratio_age_s: float | None = Field(default=None, ge=0.0)
    median_ratio: float | None = Field(default=None, ge=0.0, le=1.0)
    healthy_ratio: float = Field(ge=0.0, le=1.0)
    dry_run: bool
    held: bool = False
    stamp: float


def parse_map_guard_state(payload: str) -> MapGuardStatus | None:
    """Parse one /map_guard/state payload.

    Args:
        payload (str): The std_msgs/String data.

    Returns:
        MapGuardStatus | None: The state, None when the payload breaks the contract (logged).
    """
    try:
        return MapGuardStatus.model_validate_json(payload)
    except ValidationError as exc:
        LOGGER.warning("map_guard state dropped (not the /map_guard/state contract): %s", exc.errors()[:1])
        return None


def unreliable_reason(status: MapGuardStatus, stale_s: float) -> str | None:
    """Why the map pose of a fresh guard state cannot be trusted.

    Args:
        status (MapGuardStatus): The guard state.
        stale_s (float): Oldest agreement sample still judged, s.

    Returns:
        str | None: The reason with the next step, None when the pose is reliable.
    """
    if status.state == "relocalising":
        return (
            "map_guard is relocalising the robot on the map: wait (wait_for_event or get_robot_state again) before "
            "navigate_to_pose or move_relative."
        )
    if status.state in UNRELIABLE_STATES:
        reasons = ", ".join(status.reasons) or "unknown"
        return f"map_guard is {status.state} (reasons: {reasons}): the map pose is in doubt. {NAV_ADVICE}"
    if status.held:
        return f"map_manager reports the loaded map as not localised. {NAV_ADVICE}"
    ratio = status.median_ratio if status.median_ratio is not None else status.agreement_ratio
    if ratio is None:
        return f"no scan-to-map agreement measured yet: the map pose is unchecked. {NAV_ADVICE}"
    if status.ratio_age_s is not None and status.ratio_age_s > stale_s:
        return f"the scan-to-map agreement is {status.ratio_age_s:.1f} s old: the map pose is unchecked. {NAV_ADVICE}"
    if ratio < status.healthy_ratio:
        return (
            f"scan-to-map agreement {ratio:.2f} is below the healthy {status.healthy_ratio:.2f}: the map pose is "
            f"probably wrong (robot moved by hand?). {NAV_ADVICE}"
        )
    return None


def localisation_health(
    status: MapGuardStatus | None, age_s: float | None, stale_s: float
) -> tuple[LocalisationHealth | None, list[str]]:
    """The localisation object of get_robot_state and its notes.

    Args:
        status (MapGuardStatus | None): Latest guard state, None when none arrived.
        age_s (float | None): Seconds since it was received, None without one.
        stale_s (float): Older states (and older agreement samples inside a state) are not trusted, s.

    Returns:
        tuple[LocalisationHealth | None, list[str]]: The health (None when missing or stale) and the notes for
            RobotState.notes (empty when reliable).
    """
    if status is None or age_s is None:
        return None, [MISSING_NOTE]
    if age_s > stale_s:
        return None, [STALE_NOTE.format(age=age_s)]
    note = unreliable_reason(status, stale_s)
    health = LocalisationHealth(
        state=status.state,
        reasons=status.reasons,
        agreement_ratio=status.agreement_ratio,
        median_ratio=status.median_ratio,
        healthy_ratio=status.healthy_ratio,
        dry_run=status.dry_run,
        held=status.held,
        age_s=round(age_s, 3),
        reliable=note is None,
        note=note,
    )
    return health, [] if note is None else [note]
