"""Auto stuck points: a collision_stop or a Nav2 abort at a fresh map pose becomes a point POI in poi_store (no ROS here).

The marker only ever adds points ("stuck: <reason>", created_by "agent") or updates the note of an existing stuck POI
within merge_radius_m; it never creates an area or a keepout (a keepout would block Nav2). The agent sees the POI id as
auto_poi_id on the digest event and can upgrade it to a keepout or slow area itself.
"""

import logging
import math
import re
import time
from collections.abc import Callable
from typing import Any

from .config import AutoPoiSettings
from .models import BasePose, RobotEvent
from .monitor import RobotMonitor
from .object_memory import MAX_LABEL_LEN, POI_FRAME, PoiBackend
from .poi_client import point_in_polygon

LOGGER = logging.getLogger("mcp_server.auto_poi")
AUTO_POI_ID_KEY = "auto_poi_id"
STUCK_PREFIX = "stuck:"
STUCK_CREATED_BY = "agent"
POINT_KIND = "point"
AREA_KIND = "area"
OPEN_STATUS = "open"
TIME_FORMAT = "%Y-%m-%d %H:%M:%S"
# Short reason per event type in the POI name; other configured types use the event type itself.
REASONS = {"collision_stop": "collision stop", "nav_abort": "Nav2 abort"}
COUNT_PATTERN = re.compile(r"stuck (\d+) times?")


def stuck_reason(event: RobotEvent) -> str:
    """Short reason of a stuck event, used in the POI name.

    Args:
        event (RobotEvent): The event.

    Returns:
        str: e.g. "collision stop" or "Nav2 abort".
    """
    return REASONS.get(event.type, event.type.replace("_", " "))


def event_detail(event: RobotEvent) -> str:
    """What the event says about the stop: the collision polygon or the Nav2 message.

    Args:
        event (RobotEvent): The event.

    Returns:
        str: e.g. "collision stop, polygon FootprintStop" or "Nav2 abort, navigate_to_pose: robot in inflation".
    """
    reason = stuck_reason(event)
    polygon = event.data.get("polygon")
    if polygon:
        return f"{reason}, polygon {polygon}"
    tool = event.data.get("tool")
    message = event.data.get("message")
    if tool or message:
        return f"{reason}, {': '.join(str(part) for part in (tool, message) if part)}"
    return reason


def stuck_note(count: int, event: RobotEvent) -> str:
    """Note of a stuck POI: how often the robot got stuck there, when last and why.

    Args:
        count (int): Times stuck at this spot (>= 1).
        event (RobotEvent): The latest stuck event (its wall-clock ts is the time).

    Returns:
        str: e.g. "stuck 2 times, last 2026-10-12 14:03:05 (Nav2 abort, navigate_to_pose: ...). Auto-marked; ...".
    """
    when = time.strftime(TIME_FORMAT, time.localtime(event.ts))
    times = "time" if count == 1 else "times"
    return (
        f"stuck {count} {times}, last {when} ({event_detail(event)}). Auto-marked by mcp_server; upgrade it to a "
        "keepout or slow area if the spot should be avoided."
    )


def stuck_count(note: str) -> int:
    """Times stuck recorded in a stuck POI note (1 when the note no longer says).

    Args:
        note (str): POI note.

    Returns:
        int: Count (>= 1).
    """
    match = COUNT_PATTERN.search(note)
    return max(1, int(match.group(1))) if match else 1


class AutoPoiMarker:
    """Turns stuck events (auto_poi.event_types) at a fresh map pose into point POIs, merging repeats."""

    def __init__(self, robot: PoiBackend, settings: AutoPoiSettings) -> None:
        """Create the marker.

        Args:
            robot (PoiBackend): poi_list / poi_request provider.
            settings (AutoPoiSettings): enabled, merge radius, pose age limit and event types.
        """
        self.robot = robot
        self.cfg = settings

    def wants(self, event: RobotEvent) -> bool:
        """Whether this event can become a stuck POI (enabled and a configured type).

        Args:
            event (RobotEvent): The event.

        Returns:
            bool: True for a stuck event while the marker is enabled.
        """
        return self.cfg.enabled and event.type in self.cfg.event_types

    def fresh(self, pose: BasePose | None) -> bool:
        """Whether a pose is a map pose no older than max_pose_age_s (an unknown age is not fresh).

        Args:
            pose (BasePose | None): Robot pose.

        Returns:
            bool: True when the POI may be placed there.
        """
        return (
            pose is not None
            and pose.frame == POI_FRAME
            and pose.age_s is not None
            and pose.age_s <= self.cfg.max_pose_age_s
            and math.isfinite(pose.x)
            and math.isfinite(pose.y)
        )

    def nearest_stuck(self, pois: list[dict[str, Any]], x: float, y: float) -> dict[str, Any] | None:
        """The open agent stuck POI this spot belongs to: a point within merge_radius_m (nearest first), else an area
        the agent drew from one that contains the spot.

        Args:
            pois (list[dict[str, Any]]): POIs from poi_store.
            x (float): Map x (m).
            y (float): Map y (m).

        Returns:
            dict[str, Any] | None: The POI, or None when this is a new spot.
        """
        stuck = [
            p
            for p in pois
            if str(p.get("name", "")).startswith(STUCK_PREFIX)
            and p.get("created_by") == STUCK_CREATED_BY
            and p.get("status", OPEN_STATUS) == OPEN_STATUS
        ]
        points = [
            (dist, p)
            for p in stuck
            if p.get("kind") == POINT_KIND
            and (dist := math.hypot(float(p["x"]) - x, float(p["y"]) - y)) <= self.cfg.merge_radius_m
        ]
        if points:
            return min(points, key=lambda item: item[0])[1]
        areas = [p for p in stuck if p.get("kind") == AREA_KIND and point_in_polygon(x, y, p.get("polygon") or [])]
        return areas[0] if areas else None

    def on_event(self, event: RobotEvent, pose: BasePose | None) -> str | None:
        """Add or update the stuck POI of one event; poi_store failures are logged and give None.

        Args:
            event (RobotEvent): The event (only auto_poi.event_types count).
            pose (BasePose | None): Robot map pose when the event was handled; None or stale means no POI.

        Returns:
            str | None: Id of the added or updated POI, None when nothing was written.
        """
        if not self.wants(event):
            return None
        if pose is None or not self.fresh(pose):
            LOGGER.info("auto POI skipped for %s #%d: no fresh map pose", event.type, event.seq)
            return None
        try:
            pois, _ = self.robot.poi_list()
            match = self.nearest_stuck(pois, pose.x, pose.y)
            if match is None:
                fields = {
                    "kind": POINT_KIND,
                    "frame": POI_FRAME,
                    "x": pose.x,
                    "y": pose.y,
                    "name": f"{STUCK_PREFIX} {stuck_reason(event)}"[:MAX_LABEL_LEN],
                    "note": stuck_note(1, event),
                    "created_by": STUCK_CREATED_BY,
                }
                result = self.robot.poi_request("add", fields)
            else:
                count = stuck_count(str(match.get("note", ""))) + 1
                result = self.robot.poi_request("update", {"id": match["id"], "note": stuck_note(count, event)})
        except Exception as exc:  # the event path and the tool result must survive any poi_store failure
            LOGGER.warning("auto POI for %s #%d failed: %s", event.type, event.seq, exc)
            return None
        poi_id = (result.get("poi") or {}).get("id") or (None if match is None else match["id"])
        LOGGER.info("auto POI %s for %s #%d at (%.2f, %.2f)", poi_id, event.type, event.seq, pose.x, pose.y)
        return None if poi_id is None else str(poi_id)


def attach_auto_poi(monitor: RobotMonitor, marker: AutoPoiMarker, pose_source: Callable[[], BasePose | None]) -> None:
    """Add the marker as a monitor sink: a stuck event gets its POI and the POI id as auto_poi_id in the digest.

    Args:
        monitor (RobotMonitor): Event source (the sink fan-out) and digest.
        marker (AutoPoiMarker): The marker.
        pose_source (Callable[[], BasePose | None]): Fresh map pose of the robot (RobotApi.robot_pose).
    """

    def sink(event: RobotEvent) -> None:
        if not marker.wants(event):
            return
        poi_id = marker.on_event(event, pose_source())
        if poi_id is not None:
            monitor.annotate(event.seq, {AUTO_POI_ID_KEY: poi_id})

    monitor.add_sink(sink)
