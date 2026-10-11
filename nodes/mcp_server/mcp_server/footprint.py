"""Navigation footprint at the arm release (no ROS): what Nav2, the collision monitor and the laser_filter treat as
the robot.

static: the H body outline (footprint_geometry.body_outline over config.body, the outline scripts/gen_nav_footprint.py
writes into nav2_params.yaml). Published at mcp_server start, on take_control(subsystem='arm') (the arm may move from
then on) and after an arm release that parked the arm at home.

dynamic: published by release_control(subsystem='arm', keep_position=true): the arm does not move while released, so
the footprint is the union of the static outline, the ground shadow of the frozen arm (convex hull of every arm part
point of self_collision.part_points projected to the floor, the link capsules with their radius) and, when given,
the carried object: a circle of carried_object_radius_m around the tool point. The arm shadow and the carried circle
are simplified by simplify_m; the static outline is kept exact.

Consumers (through a FootprintSink, implemented by ros_iface.RosRobot):
- the Nav2 local and global costmap footprint inputs (geometry_msgs/Polygon on `<costmap>/footprint`): the footprint;
- the laser_filter `polygon` parameter (parameter service): the footprint + laser_filter_margin_m, so the lidar does
  not see the robot's own arm or carried object;
- the collision_monitor ArmStop polygon (PolygonStamped, transient local) and its `ArmStop.enabled` parameter: the arm
  shadow and carried circle grown by stopbox_margin_m, enabled only while the footprint is dynamic. The StopBox
  velocity polygons are static parameters (Jazzy collision_monitor reads them once at configure), so they keep
  covering the body and ArmStop covers the arm.

The current state is re-sent every republish_s, so a restarted consumer gets it again.
"""

import logging
import threading
from collections.abc import Callable
from typing import Literal, Protocol

import numpy as np
from shapely.geometry import MultiPoint, Polygon
from shapely.ops import unary_union

from .arm_geometry import LINK_CAPSULE_RADIUS_M
from .config import BodySettings, NavFootprintSettings
from .footprint_geometry import Point2, body_outline, circle_polygon, offset_polygon, polygon_text, to_points
from .models import FootprintState
from .self_collision import CAPSULES

LOGGER = logging.getLogger(__name__)
TOOL_POINT = "tool_point"
# Capsule parts of self_collision.part_points: their first two rows are the capsule axis ends.
CAPSULE_PARTS = tuple(name for name, *_ in CAPSULES)
FootprintKind = Literal["static", "dynamic"]


class FootprintSink(Protocol):
    """Where the footprint goes (ros_iface.RosRobot; faked in tests)."""

    def publish_polygon(self, topic: str, points: list[Point2]) -> None:
        """Publish a geometry_msgs/Polygon (a Nav2 costmap footprint input)."""
        ...

    def publish_stamped_polygon(self, topic: str, frame: str, points: list[Point2]) -> None:
        """Publish a geometry_msgs/PolygonStamped (the collision_monitor ArmStop shape)."""
        ...

    def set_node_parameter(self, node: str, name: str, value: str | bool) -> None:
        """Set one parameter of another node through its parameter service (asynchronous, failures logged)."""
        ...

    def every(self, period_s: float, callback: Callable[[], None]) -> None:
        """Call callback periodically (a ROS timer)."""
        ...


def arm_shadow(parts: dict[str, np.ndarray], circle_segments: int) -> Polygon:
    """Ground shadow of an arm pose: convex hull of every part point projected to the floor, the link capsules
    widened by their radius around both axis ends.

    Args:
        parts (dict[str, np.ndarray]): self_collision part_points of the pose (base_link, m).
        circle_segments (int): Vertices per capsule end circle.

    Returns:
        Polygon: The shadow (base_link, m).
    """
    xy = [np.asarray(points, dtype=np.float64)[:, :2] for points in parts.values()]
    for name in CAPSULE_PARTS:
        if name in parts:
            for x, y in np.asarray(parts[name], dtype=np.float64)[:2, :2]:
                xy.append(np.asarray(circle_polygon(x, y, LINK_CAPSULE_RADIUS_M, circle_segments).exterior.coords))
    return MultiPoint(np.vstack(xy)).convex_hull


def carried_circle(parts: dict[str, np.ndarray], radius_m: float, circle_segments: int) -> Polygon | None:
    """Carried object: a circle of radius_m around the tool point projected to the floor.

    Args:
        parts (dict[str, np.ndarray]): self_collision part_points (its tool_point row).
        radius_m (float): Object radius (m); 0 = no circle.
        circle_segments (int): Vertices of the circle.

    Returns:
        Polygon | None: The circle, None for radius 0.
    """
    if radius_m <= 0.0:
        return None
    x, y = np.asarray(parts[TOOL_POINT], dtype=np.float64)[0, :2]
    return circle_polygon(float(x), float(y), radius_m, circle_segments)


def dynamic_outline(static: Polygon, extra: Polygon, simplify_m: float) -> Polygon:
    """Union of the exact static outline and the simplified arm extension, holes filled.

    Args:
        static (Polygon): Static H body outline.
        extra (Polygon): Arm shadow (and carried circle).
        simplify_m (float): Simplification tolerance of extra (m).

    Returns:
        Polygon: One polygon (when extra does not touch the body the union's convex hull joins them).
    """
    union = unary_union([static, extra.simplify(simplify_m, preserve_topology=True)])
    if union.geom_type != "Polygon":
        LOGGER.warning("arm shadow does not touch the body outline: using the convex hull of both")
        union = union.convex_hull
    return Polygon(union.exterior)


class NavFootprint:
    """The published navigation footprint: static or dynamic, and its consumers."""

    def __init__(self, settings: NavFootprintSettings, body: BodySettings, sink: FootprintSink) -> None:
        """Compute the static outline (nothing is published until start / publish_*).

        Args:
            settings (NavFootprintSettings): Geometry numbers and consumer names.
            body (BodySettings): Body parts (the H outline source).
            sink (FootprintSink): Publisher and parameter client.
        """
        self.cfg = settings
        self.sink = sink
        self.static = body_outline(body.boxes, body.cylinders, settings.circle_segments, settings.close_gap_m)
        self.static_points = to_points(self.static, settings.decimals)
        self.static_laser_points = self.laser_points(self.static_points)
        self.lock = threading.Lock()
        self.kind: FootprintKind = "static"
        self.points = self.static_points
        self.laser = self.static_laser_points
        self.arm_stop: list[Point2] | None = None
        self.carried_radius_m: float | None = None

    def laser_points(self, points: list[Point2]) -> list[Point2]:
        """laser_filter self polygon of a footprint.

        Args:
            points (list[Point2]): Footprint.

        Returns:
            list[Point2]: Footprint + laser_filter_margin_m.
        """
        return offset_polygon(points, self.cfg.laser_filter_margin_m, self.cfg.decimals)

    def start(self) -> None:
        """Publish the static footprint (mcp_server start) and re-send the current one every republish_s."""
        self.publish_static()
        self.sink.every(self.cfg.republish_s, self.apply)

    def publish_static(self) -> FootprintState:
        """Publish the static H body footprint and disable ArmStop.

        Returns:
            FootprintState: The published state.
        """
        with self.lock:
            self.kind = "static"
            self.points = self.static_points
            self.laser = self.static_laser_points
            self.arm_stop = None
            self.carried_radius_m = None
        self.apply()
        return self.state()

    def publish_dynamic(self, parts: dict[str, np.ndarray], carried_radius_m: float | None) -> FootprintState:
        """Publish the footprint of a frozen arm pose (and carried object) and enable ArmStop.

        Args:
            parts (dict[str, np.ndarray]): self_collision part_points of the frozen pose (base_link, m).
            carried_radius_m (float | None): Carried object radius at the tool point (m), None without one.

        Returns:
            FootprintState: The published state.
        """
        segments, decimals = self.cfg.circle_segments, self.cfg.decimals
        extra = arm_shadow(parts, segments)
        circle = carried_circle(parts, carried_radius_m or 0.0, segments)
        if circle is not None:
            extra = unary_union([extra, circle]).convex_hull
        points = to_points(dynamic_outline(self.static, extra, self.cfg.simplify_m), decimals)
        reach = to_points(extra.simplify(self.cfg.simplify_m, preserve_topology=True), decimals)
        with self.lock:
            self.kind = "dynamic"
            self.points = points
            self.laser = self.laser_points(points)
            self.arm_stop = offset_polygon(reach, self.cfg.stopbox_margin_m, decimals)
            self.carried_radius_m = carried_radius_m if circle is not None else None
        self.apply()
        return self.state()

    def apply(self) -> None:
        """Send the current footprint to every consumer (also the periodic re-send)."""
        with self.lock:
            points, laser, arm_stop = self.points, self.laser, self.arm_stop
        for topic in self.cfg.costmap_footprint_topics:
            self.sink.publish_polygon(topic, points)
        if arm_stop is not None:
            self.sink.publish_stamped_polygon(self.cfg.arm_stop_topic, self.cfg.frame_id, arm_stop)
        self.sink.set_node_parameter(self.cfg.laser_filter_node, self.cfg.laser_polygon_param, polygon_text(laser))
        self.sink.set_node_parameter(
            self.cfg.collision_monitor_node, f"{self.cfg.arm_stop_polygon}.enabled", arm_stop is not None
        )

    def state(self) -> FootprintState:
        """What is published now.

        Returns:
            FootprintState: Kind, vertex count and carried radius.
        """
        with self.lock:
            return FootprintState(
                kind=self.kind, vertices=len(self.points), carried_object_radius_m=self.carried_radius_m
            )
