"""Client side of the poi_store topics (no ROS): request/response matching by request_id, /poi/list parsing, geometry."""

import json
import math
import threading
import uuid
from collections.abc import Callable, Sequence
from typing import Any

from .geometry import normalize_angle
from .models import RobotError
from .perception_models import PoiView


class PoiRequests:
    """Publishes /poi/command messages and matches /poi/result answers to them by request_id."""

    def __init__(
        self,
        publish: Callable[[str], None],
        store_running: Callable[[], bool],
        timeout_s: float,
        new_id: Callable[[], str] = lambda: uuid.uuid4().hex,
    ) -> None:
        """Create the client.

        Args:
            publish (Callable[[str], None]): Publishes one JSON string on /poi/command.
            store_running (Callable[[], bool]): Whether poi_store listens (a subscriber exists on /poi/command).
            timeout_s (float): How long to wait for the matching result.
            new_id (Callable[[], str]): Request id factory.
        """
        self.publish = publish
        self.store_running = store_running
        self.timeout_s = timeout_s
        self.new_id = new_id
        self.lock = threading.Lock()
        self.pending: dict[str, tuple[threading.Event, list[dict[str, Any]]]] = {}

    def pending_count(self) -> int:
        """Number of requests still waiting for an answer.

        Returns:
            int: Pending request count.
        """
        with self.lock:
            return len(self.pending)

    def on_result(self, payload: str) -> None:
        """Feed one /poi/result JSON string; results of unknown request ids and malformed payloads are dropped.

        Args:
            payload (str): The std_msgs/String data.
        """
        try:
            result = json.loads(payload)
            request_id = result["request_id"]
        except (ValueError, KeyError, TypeError):
            return
        with self.lock:
            entry = self.pending.get(request_id)
        if entry is not None:
            entry[1].append(result)
            entry[0].set()

    def request(self, op: str, poi: dict[str, Any]) -> dict[str, Any]:
        """Send one command and wait for its result.

        Args:
            op (str): "add", "update" or "delete".
            poi (dict[str, Any]): POI payload as defined by poi_store.

        Returns:
            dict[str, Any]: The accepted result {request_id, ok: True, message, poi}.
        """
        if not self.store_running():
            raise RobotError("poi_store is not running (nothing listens on /poi/command); start the node and retry")
        request_id = self.new_id()
        event: threading.Event = threading.Event()
        answers: list[dict[str, Any]] = []
        with self.lock:
            self.pending[request_id] = (event, answers)
        try:
            self.publish(json.dumps({"op": op, "request_id": request_id, "poi": poi}))
            if not event.wait(self.timeout_s):
                raise RobotError(f"poi_store did not answer within {self.timeout_s:g} s (request {request_id})")
        finally:
            with self.lock:
                self.pending.pop(request_id, None)
        result = answers[0]
        if not result.get("ok"):
            raise RobotError(f"poi_store rejected the {op}: {result.get('message', 'no reason given')}")
        return result


def parse_poi_list(payload: str) -> tuple[list[dict[str, Any]], int]:
    """Parse a /poi/list message.

    Args:
        payload (str): JSON {"pois": [...], "revision": int}.

    Returns:
        tuple[list[dict[str, Any]], int]: POIs and the store revision.
    """
    data = json.loads(payload)
    if not isinstance(data, dict) or not isinstance(data.get("pois"), list):
        raise ValueError("/poi/list payload must be an object with a 'pois' list")
    return data["pois"], int(data.get("revision", 0))


def point_in_polygon(x: float, y: float, polygon: Sequence[Sequence[float]]) -> bool:
    """Ray casting point-in-polygon test.

    Args:
        x (float): Point x.
        y (float): Point y.
        polygon (Sequence[Sequence[float]]): Vertices [[x, y], ...].

    Returns:
        bool: True when the point is inside.
    """
    inside = False
    j = len(polygon) - 1
    for i, (xi, yi) in enumerate(polygon):
        xj, yj = polygon[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def describe_pois(
    pois: Sequence[dict[str, Any]],
    pose: tuple[float, float, float] | None,
    status: str | None = None,
    near_radius_m: float | None = None,
) -> list[PoiView]:
    """Filter POIs and add distance/bearing from the robot.

    Args:
        pois (Sequence[dict[str, Any]]): POI JSON objects.
        pose (tuple[float, float, float] | None): Robot (x, y, yaw) in the map frame, None when unknown.
        status (str | None): Keep only this status.
        near_radius_m (float | None): When set, keep only POIs within this distance and sort nearest first.

    Returns:
        list[PoiView]: Views in store order, or nearest first with near_radius_m.
    """
    views: list[PoiView] = []
    for poi in pois:
        if status is not None and poi.get("status") != status:
            continue
        view = PoiView(
            poi=poi,
            id=str(poi.get("id", "")),
            name=str(poi.get("name", "")),
            kind=str(poi.get("kind", "")),
            status=str(poi.get("status", "")),
        )
        if pose is not None:
            view.distance_m = round(math.hypot(poi["x"] - pose[0], poi["y"] - pose[1]), 3)
            bearing = normalize_angle(math.atan2(poi["y"] - pose[1], poi["x"] - pose[0]) - pose[2])
            view.bearing_deg = round(math.degrees(bearing), 1)
            if poi.get("kind") == "area" and poi.get("polygon"):
                view.inside = point_in_polygon(pose[0], pose[1], poi["polygon"])
        views.append(view)
    if near_radius_m is not None and pose is not None:
        views = sorted((v for v in views if (v.distance_m or 0.0) <= near_radius_m), key=lambda v: v.distance_m or 0.0)
    return views
