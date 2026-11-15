"""Localisation health from map_guard_cpp's /map_guard/state (std_msgs/String JSON, latched; no ROS imports).

The guard publishes its state on every transition and about once a second. get_robot_state reports it as the
``localisation`` object with a ``reliable`` verdict, so the agent sees a bad map pose (a robot carried or turned by hand)
before it navigates. reliable is null (unknown) when map_guard is settled but cannot score the scan (unmapped or open
space): that is no evidence of a wrong pose. A missing or stale state is never invented: the object is null and a note
says why.
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
# A settled guard without a fresh agreement sample: unknown, not bad; no relocalise advice.
UNSCORABLE_NOTE = (
    "map agreement not scorable here (unmapped/open space), not evidence of a wrong pose: {count} recent scans had too "
    "few endpoints on mapped cells. The map pose is unchecked until the scan sees mapped walls again; compare the "
    "camera view with the map if in doubt."
)
NO_SCORE_NOTE = (
    "no scan scored against the map lately (no fresh agreement sample), not evidence of a wrong pose: the map pose is "
    "unchecked; compare the camera view with the map if in doubt."
)


class MapGuardStatus(BaseModel):
    """One /map_guard/state message (map_guard_cpp state_wire.hpp)."""

    model_config = ConfigDict(extra="ignore")

    state: GuardStateName
    reasons: list[str] = Field(default_factory=list)
    agreement_ratio: float | None = Field(default=None, ge=0.0, le=1.0)
    ratio_age_s: float | None = Field(default=None, ge=0.0)
    unscored_scans: int = Field(ge=0)
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


def judge(status: MapGuardStatus, age_s: float, stale_s: float) -> tuple[bool | None, str | None]:
    """How far the map pose of a fresh guard state can be trusted.

    Only positive evidence makes it false: a disturbed, settling, evaluating or relocalising guard, a held map, or a
    fresh agreement (the latest sample or its median, whichever is lower) below healthy. A settled guard without a
    fresh sample (the robot faces unmapped or open space, too few endpoints on known cells) is unknown, not bad.

    Args:
        status (MapGuardStatus): The guard state.
        age_s (float): Age of the message, s.
        stale_s (float): Oldest agreement sample (its ratio_age_s plus the message age) still judged, s.

    Returns:
        tuple[bool | None, str | None]: reliable (None when unknown) and the note with the next step (None when
            reliable).
    """
    if status.state == "relocalising":
        return False, (
            "map_guard is relocalising the robot on the map: wait (wait_for_event or get_robot_state again) before "
            "navigate_to_pose or move_relative."
        )
    if status.state in UNRELIABLE_STATES:
        reasons = ", ".join(status.reasons) or "unknown"
        return False, f"map_guard is {status.state} (reasons: {reasons}): the map pose is in doubt. {NAV_ADVICE}"
    if status.held:
        return False, f"map_manager reports the loaded map as not localised. {NAV_ADVICE}"
    sample_age = None if status.ratio_age_s is None else status.ratio_age_s + age_s
    if status.agreement_ratio is None or sample_age is None or sample_age > stale_s:
        if status.unscored_scans > 0:
            return None, UNSCORABLE_NOTE.format(count=status.unscored_scans)
        return None, NO_SCORE_NOTE
    ratio = min(r for r in (status.agreement_ratio, status.median_ratio) if r is not None)
    if ratio < status.healthy_ratio:
        median = "none" if status.median_ratio is None else f"{status.median_ratio:.2f}"
        return False, (
            f"scan-to-map agreement {ratio:.2f} (latest {status.agreement_ratio:.2f}, median {median}) is below the "
            f"healthy {status.healthy_ratio:.2f}: the map pose is probably wrong (robot moved by hand?). {NAV_ADVICE}"
        )
    return True, None


def message_age(receive_age_s: float, stamp: float, wall_now: float) -> float:
    """Age of a /map_guard/state message: the older of its receipt and its stamp (same host clock), so a latched
    sample delivered late is not treated as fresh.

    Args:
        receive_age_s (float): Seconds since it was received.
        stamp (float): Its stamp, epoch s.
        wall_now (float): Wall clock now, epoch s.

    Returns:
        float: The age, s.
    """
    return max(receive_age_s, wall_now - stamp)


def reliable_since(status: MapGuardStatus, age_s: float, stale_s: float) -> float | None:
    """Wall time of the agreement sample that made a fresh state reliable (for relocalise's ``since``).

    Args:
        status (MapGuardStatus): The guard state.
        age_s (float): Age of the message, s.
        stale_s (float): Older messages are not judged, s.

    Returns:
        float | None: Epoch seconds of the sample, None when the state is stale or not reliable.
    """
    if age_s > stale_s or judge(status, age_s, stale_s)[0] is not True:
        return None
    return status.stamp - (status.ratio_age_s or 0.0)


def localisation_health(
    status: MapGuardStatus | None, age_s: float | None, stale_s: float
) -> tuple[LocalisationHealth | None, list[str]]:
    """The localisation object of get_robot_state and its notes.

    Args:
        status (MapGuardStatus | None): Latest guard state, None when none arrived.
        age_s (float | None): Age of the message (message_age), None without one.
        stale_s (float): Older states (and older agreement samples, ratio_age_s plus age_s) are not trusted, s.

    Returns:
        tuple[LocalisationHealth | None, list[str]]: The health (None when missing or stale) and the notes for
            RobotState.notes (empty when reliable).
    """
    if status is None or age_s is None:
        return None, [MISSING_NOTE]
    if age_s > stale_s:
        return None, [STALE_NOTE.format(age=age_s)]
    reliable, note = judge(status, age_s, stale_s)
    health = LocalisationHealth(
        state=status.state,
        reasons=status.reasons,
        agreement_ratio=status.agreement_ratio,
        median_ratio=status.median_ratio,
        healthy_ratio=status.healthy_ratio,
        dry_run=status.dry_run,
        held=status.held,
        age_s=round(age_s, 3),
        ratio_age_s=None if status.ratio_age_s is None else round(status.ratio_age_s + age_s, 3),
        unscored_scans=status.unscored_scans,
        reliable=reliable,
        note=note,
    )
    return health, [] if note is None else [note]
