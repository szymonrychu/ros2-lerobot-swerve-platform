"""Navigation footprint (no ROS): what Nav2, the collision monitor and the laser_filter treat as the robot.

static: the H body outline (footprint_geometry.body_outline over config.body) united with the ground shadow of the arm
parked at its stored home (nav_footprint.parked_arm_shadow), the outline scripts/gen_nav_footprint.py writes into
nav2_params.yaml. Published while autonomy holds the arm (take_control('arm'): the arm may move from then on), after a
parked release, after a converged /arm/home (its tick sees the parked arm) and whenever the measured arm shadow lies
inside it.

dynamic: the arm is released out of the static outline. The footprint is the union of the static outline, the ground
shadow of the measured arm pose (convex hull of every arm part point of self_collision.part_points projected to the
floor, the link capsules with their radius) and, when given, the carried object: a circle of carried_object_radius_m
around the tool point. The shadow and circle are grown by simplify_m and then simplified by it (never trimmed).

conservative: the arm is released but its pose is unknown (no fresh joint states at mcp_server start, or since an
unverified static footprint): a disc of conservative_reach_m (+ the carried radius) around the arm mount.

Triggers: release_control publishes the released pose (publish_dynamic; publish_conservative without joint states),
take_control and a parked release publish static (publish_static), a converged /arm/home runs the tick at once. Every
republish_s the tick follows the arm while autonomy does not hold it (the leader arm or web UI may move it,
mcp_server may have restarted): it recomputes from the fresh measured pose (replacing the footprint only when it moved
by more than refresh_threshold_m), keeps the last footprint when the sample is stale, and falls back to conservative when the last
footprint never saw the released pose.

Consumers (through a FootprintSink, implemented by ros_iface.RosRobot):
- the Nav2 costmaps: the footprint on `<ns>/footprint` (geometry_msgs/Polygon) and their inflation layer's
  inflation_radius (circumscribed radius rounded up to a cell) and cost_scaling_factor (steepest that keeps the cost
  >= 1 at the circumscribed radius). A footprint change makes a costmap re-inflate, so the tick republishes to a
  costmap only when its `<ns>/published_footprint` differs (restarted Nav2) or never arrived;
- the laser_filter `polygon` parameter: the footprint + laser_filter_margin_m (re-sent every tick);
- the collision_monitor ArmStop polygon (PolygonStamped, transient local; the arm part outside the static outline,
  grown by stopbox_margin_m) and its `ArmStop.enabled` parameter (true unless static; re-sent every tick).
A consumer whose parameter service comes up again (restart) gets its values at once (consumer_ready). The last outcome
per consumer is in FootprintState.consumers. One re-entrant lock covers a state change and its sends, so a tick never
sends a footprint a tool call has already replaced.

`python -m mcp_server.footprint <client.yml> <joints json> <circle_segments> <simplify_m> <decimals>` prints the
parked-arm shadow (JSON [[x, y], ...]) for scripts/gen_nav_footprint.py (the arm FK lives in this uv environment).
"""

import json
import logging
import sys
import threading
from collections.abc import Callable
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Literal, Protocol

import numpy as np
import yaml
from shapely import get_coordinates
from shapely.geometry import MultiPoint, Point, Polygon
from shapely.ops import unary_union

from .arm_geometry import LINK_CAPSULE_RADIUS_M
from .config import BodySettings, McpServerConfig, NavFootprintSettings
from .floor_guard import JawModel
from .footprint_geometry import (
    Point2,
    body_outline,
    circle_polygon,
    cost_scaling_factor,
    grow_extension,
    inflation_radius,
    offset_polygon,
    polygon_text,
    same_shape,
    static_outline,
    to_points,
    union_outline,
)
from .ik import ArmKinematics
from .models import FootprintState
from .self_collision import CAPSULES, SelfCollisionChecker

LOGGER = logging.getLogger(__name__)
TOOL_POINT = "tool_point"
# Capsule parts of self_collision.part_points: their first two rows are the capsule axis ends.
CAPSULE_PARTS = tuple(name for name, *_ in CAPSULES)
FootprintKind = Literal["static", "dynamic", "conservative"]
PUBLISHED_FOOTPRINT = "published_footprint"
INFLATION_RADIUS = "inflation_radius"
COST_SCALING_FACTOR = "cost_scaling_factor"
OK = "ok"
PENDING = "pending"
# Channel of a costmap consumer that is its footprint topic (the others are parameter names).
FOOTPRINT_CHANNEL = "footprint"
ParamValue = str | bool | float


class FootprintSink(Protocol):
    """Where the footprint goes and where the arm pose comes from (ros_iface.RosRobot; faked in tests)."""

    def publish_polygon(self, topic: str, points: list[Point2]) -> None:
        """Publish a geometry_msgs/Polygon (a Nav2 costmap footprint input)."""
        ...

    def publish_stamped_polygon(self, topic: str, frame: str, points: list[Point2]) -> None:
        """Publish a geometry_msgs/PolygonStamped (the collision_monitor ArmStop shape)."""
        ...

    def set_node_parameter(
        self, node: str, name: str, value: ParamValue, report: Callable[[str], None] | None = None
    ) -> None:
        """Set one parameter of another node (asynchronous); report gets ok | unavailable: | refused: | timeout:."""
        ...

    def observed_polygon(self, topic: str) -> list[Point2] | None:
        """Last polygon received on a costmap's published_footprint topic, None before the first."""
        ...

    def every(self, period_s: float, callback: Callable[[], None]) -> None:
        """Call callback every period_s (a ROS timer)."""
        ...

    def on_consumer_ready(self, callback: Callable[[str], None]) -> None:
        """Call callback(node) when a consumer node's parameter service becomes available."""
        ...

    def arm_lease_held(self) -> bool:
        """Whether autonomy (this server) holds the arm lease."""
        ...

    def arm_parts(self) -> dict[str, np.ndarray] | None:
        """self_collision part_points of the fresh measured arm pose, None when the joint states are stale."""
        ...

    def arm_mount_xy(self) -> Point2:
        """Arm mount (shoulder pan axis) in base_link (m)."""
        ...


@dataclass(frozen=True)
class Footprint:
    """One consistent published footprint (every consumer value derives from it)."""

    kind: FootprintKind
    points: list[Point2]
    laser: list[Point2]
    arm_stop: list[Point2] | None  # None: ArmStop disabled
    inflation_radius: float
    cost_scaling_factor: float
    carried_radius_m: float | None
    pose_known: bool  # computed from (or confirmed by) the measured released pose / a converged home


def costmap_namespace(topic: str) -> str:
    """Namespace of a costmap footprint topic (/local_costmap/footprint -> /local_costmap).

    Args:
        topic (str): Footprint topic.

    Returns:
        str: Namespace.
    """
    return topic.rsplit("/", 1)[0]


def costmap_node(topic: str) -> str:
    """Costmap2DROS node of a footprint topic (/local_costmap/footprint -> /local_costmap/local_costmap).

    Args:
        topic (str): Footprint topic.

    Returns:
        str: Fully qualified node name.
    """
    ns = costmap_namespace(topic)
    return f"{ns}/{ns.rsplit('/', 1)[-1]}"


def published_footprint_topic(topic: str) -> str:
    """Topic on which a costmap republishes the footprint it uses (in its global frame).

    Args:
        topic (str): Footprint input topic.

    Returns:
        str: `<ns>/published_footprint`.
    """
    return f"{costmap_namespace(topic)}/{PUBLISHED_FOOTPRINT}"


def consumer_name(node: str) -> str:
    """Short consumer name of a node (/local_costmap/local_costmap -> local_costmap).

    Args:
        node (str): Fully qualified node name.

    Returns:
        str: Last path segment.
    """
    return node.rsplit("/", 1)[-1]


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


def arm_extension(parts: dict[str, np.ndarray], carried_radius_m: float | None, circle_segments: int) -> Polygon:
    """Exact area the released arm (and carried object) covers: convex hull of its shadow and the carried circle.

    Args:
        parts (dict[str, np.ndarray]): self_collision part_points.
        carried_radius_m (float | None): Carried object radius (m), None without one.
        circle_segments (int): Vertices per circle.

    Returns:
        Polygon: The extension.
    """
    extra = arm_shadow(parts, circle_segments)
    circle = carried_circle(parts, carried_radius_m or 0.0, circle_segments)
    if circle is not None:
        extra = unary_union([extra, circle]).convex_hull
    return extra


def dynamic_outline(static: Polygon, extra: Polygon, simplify_m: float) -> Polygon:
    """Union of the exact static outline and the grown-then-simplified arm extension, holes filled.

    Args:
        static (Polygon): Static outline.
        extra (Polygon): Arm shadow (and carried circle).
        simplify_m (float): Simplification tolerance of extra (m).

    Returns:
        Polygon: One polygon (when extra does not touch the body the union's convex hull joins them).
    """
    return union_outline(static, grow_extension(extra, simplify_m))


def parked_arm_shadow(
    config: McpServerConfig, joints: dict[str, float], circle_segments: int, simplify_m: float, decimals: int
) -> list[Point2]:
    """Grown and simplified ground shadow of a parked arm pose (the static footprint's arm part).

    Args:
        config (McpServerConfig): Node configuration (arm mount, joint offsets, tool offset, URDF).
        joints (dict[str, float]): Parked pose, joint name -> rad (measured space).
        circle_segments (int): Vertices per capsule end circle.
        simplify_m (float): Grow / simplify tolerance (m).
        decimals (int): Decimal places of the vertices.

    Returns:
        list[Point2]: Vertices (base_link, m).
    """
    kin = ArmKinematics(
        config.arm.urdf_path,
        margin=config.limits.arm_limit_margin_rad,
        joint_offsets=config.arm.joint_offsets_rad.model_dump(),
        tool_offset=tuple(config.arm.tool_offset_m.model_dump().values()),
        limit_overrides=config.arm.joint_limit_overrides_rad,
    )
    jaw = JawModel(kin, config.arm.jaw_open_axis, config.arm.gripper_closed_rad)
    parts = SelfCollisionChecker.from_config(kin, config, jaw).part_points(joints)
    return to_points(grow_extension(arm_shadow(parts, circle_segments), simplify_m), decimals)


class NavFootprint:
    """The published navigation footprint and its consumers (see the module docstring)."""

    def __init__(self, settings: NavFootprintSettings, body: BodySettings, sink: FootprintSink) -> None:
        """Compute the static outline (nothing is published until start / publish_*).

        Args:
            settings (NavFootprintSettings): Geometry numbers and consumer names.
            body (BodySettings): Body parts (the H outline source).
            sink (FootprintSink): Publisher, parameter client and arm pose source.
        """
        self.cfg = settings
        self.sink = sink
        outline = body_outline(body.boxes, body.cylinders, settings.circle_segments, settings.close_gap_m)
        self.static = static_outline(outline, settings.parked_arm_shadow)
        self.static_points = to_points(self.static, settings.decimals)
        self.costmaps = [(topic, costmap_node(topic)) for topic in settings.costmap_footprint_topics]
        # One re-entrant lock across a state change and its sends (a tick never sends a replaced footprint).
        self.lock = threading.RLock()
        self.current = self.static_footprint(None, pose_known=False)
        # Consumer outcomes per (consumer, channel) and the values each node acknowledged (own lock: outcomes are
        # reported from ROS callback threads).
        self.status_lock = threading.Lock()
        self.outcomes: dict[str, dict[str, str]] = {consumer_name(node): {} for node in self.consumer_nodes()}
        self.acked: dict[tuple[str, str], ParamValue] = {}

    def consumer_nodes(self) -> list[str]:
        """Every consumer node (costmaps, laser_filter, collision_monitor).

        Returns:
            list[str]: Fully qualified node names.
        """
        return [node for _, node in self.costmaps] + [self.cfg.laser_filter_node, self.cfg.collision_monitor_node]

    # --- footprint construction ------------------------------------------------------------------------------------

    def footprint(
        self,
        kind: FootprintKind,
        points: list[Point2],
        arm_stop: list[Point2] | None,
        carried: float | None,
        pose_known: bool,
    ) -> Footprint:
        """Footprint with its derived consumer values.

        Args:
            kind (FootprintKind): static | dynamic | conservative.
            points (list[Point2]): Costmap footprint.
            arm_stop (list[Point2] | None): ArmStop polygon, None = disabled.
            carried (float | None): Carried object radius (m).
            pose_known (bool): The released pose was measured (or the arm homed).

        Returns:
            Footprint: The footprint.
        """
        return Footprint(
            kind=kind,
            points=points,
            laser=offset_polygon(points, self.cfg.laser_filter_margin_m, self.cfg.decimals),
            arm_stop=arm_stop,
            inflation_radius=inflation_radius(points, self.cfg.costmap_resolution),
            cost_scaling_factor=cost_scaling_factor(points, self.cfg.max_cost_scaling_factor),
            carried_radius_m=carried,
            pose_known=pose_known,
        )

    def static_footprint(self, carried: float | None, pose_known: bool) -> Footprint:
        """The static outline, ArmStop off.

        Args:
            carried (float | None): Carried object radius (m).
            pose_known (bool): Confirmed by a measured pose / converged home.

        Returns:
            Footprint: Static footprint.
        """
        return self.footprint("static", self.static_points, None, carried, pose_known)

    def extended_footprint(
        self, kind: FootprintKind, extra: Polygon, carried: float | None, pose_known: bool
    ) -> Footprint:
        """Static outline grown by an extension (arm shadow or reach disc); static when it lies inside the outline.

        Args:
            kind (FootprintKind): dynamic | conservative.
            extra (Polygon): Exact extension.
            carried (float | None): Carried object radius (m).
            pose_known (bool): Whether the extension comes from a measured pose.

        Returns:
            Footprint: The footprint.
        """
        simplify, decimals = self.cfg.simplify_m, self.cfg.decimals
        if self.static.buffer(simplify).contains(extra):
            return self.static_footprint(carried, pose_known)
        grown = grow_extension(extra, simplify)
        points = to_points(union_outline(self.static, grown), decimals)
        return self.footprint(kind, points, self.arm_stop_points(grown), carried, pose_known)

    def arm_stop_points(self, extra: Polygon) -> list[Point2] | None:
        """ArmStop polygon: the part of the extension outside the static outline, grown by stopbox_margin_m (the
        StopBox already covers the body; an obstacle next to the body must not stop the robot through ArmStop). When
        the outside part falls apart (the arm also spans a notch) the whole extension is used: collision_monitor takes
        one polygon, and the body inside it holds no obstacle (the laser_filter removes it from the scan).

        Args:
            extra (Polygon): Grown extension.

        Returns:
            list[Point2] | None: Polygon, None when nothing sticks out.
        """
        outside = extra.difference(self.static)
        pieces = [g for g in getattr(outside, "geoms", [outside]) if g.geom_type == "Polygon"]
        pieces = [g for g in pieces if g.area > self.cfg.simplify_m**2]
        if not pieces:
            return None
        shape = pieces[0] if len(pieces) == 1 else extra
        return offset_polygon(to_points(shape, self.cfg.decimals), self.cfg.stopbox_margin_m, self.cfg.decimals)

    def conservative_footprint(self, carried: float | None) -> Footprint:
        """Arm pose unknown: a disc of conservative_reach_m (+ carried radius) around the arm mount.

        Args:
            carried (float | None): Carried object radius (m).

        Returns:
            Footprint: The footprint.
        """
        x, y = self.sink.arm_mount_xy()
        disc = circle_polygon(x, y, self.cfg.conservative_reach_m + (carried or 0.0), self.cfg.circle_segments)
        return self.extended_footprint("conservative", disc, carried, pose_known=False)

    def measured_footprint(self, parts: dict[str, np.ndarray], carried: float | None) -> Footprint:
        """Footprint of a measured released arm pose.

        Args:
            parts (dict[str, np.ndarray]): self_collision part_points of the pose.
            carried (float | None): Carried object radius (m).

        Returns:
            Footprint: The footprint.
        """
        extra = arm_extension(parts, carried, self.cfg.circle_segments)
        return self.extended_footprint("dynamic", extra, carried, pose_known=True)

    def moved(self, new: Footprint) -> bool:
        """Whether a recomputed footprint differs enough from the current one to be published.

        Args:
            new (Footprint): Recomputed footprint.

        Returns:
            bool: True to publish it.
        """
        cur = self.current
        if (new.kind, new.carried_radius_m, new.arm_stop is None) != (
            cur.kind,
            cur.carried_radius_m,
            cur.arm_stop is None,
        ):
            return True
        if new.points == cur.points:
            return False
        return Polygon(new.points).hausdorff_distance(Polygon(cur.points)) > self.cfg.refresh_threshold_m

    # --- triggers --------------------------------------------------------------------------------------------------

    def start(self) -> None:
        """mcp_server start: publish the footprint of the measured arm pose (static at home, dynamic with the arm out,
        conservative without joint states), then run the tick every republish_s and resend on consumer restarts."""
        with self.lock:
            parts = None if self.sink.arm_lease_held() else self.sink.arm_parts()
            if self.sink.arm_lease_held():
                self.set(self.static_footprint(None, pose_known=False))
            elif parts is None:
                self.set(self.conservative_footprint(None))
            else:
                self.set(self.measured_footprint(parts, None))
        self.sink.every(self.cfg.republish_s, self.tick)
        self.sink.on_consumer_ready(self.consumer_ready)

    def publish_static(self) -> FootprintState:
        """Publish the static footprint and disable ArmStop (take_control('arm'), a parked release).

        Returns:
            FootprintState: The published state.
        """
        with self.lock:
            self.set(self.static_footprint(None, pose_known=False))
            return self.state()

    def publish_dynamic(self, parts: dict[str, np.ndarray], carried_radius_m: float | None) -> FootprintState:
        """Publish the footprint of a measured released arm pose (and carried object).

        Args:
            parts (dict[str, np.ndarray]): self_collision part_points of the pose (base_link, m).
            carried_radius_m (float | None): Carried object radius at the tool point (m), None without one.

        Returns:
            FootprintState: The published state (static when the arm lies inside the static outline).
        """
        with self.lock:
            self.set(self.measured_footprint(parts, carried_radius_m))
            return self.state()

    def publish_conservative(self, carried_radius_m: float | None) -> FootprintState:
        """Publish the conservative footprint: released without fresh joint states, the arm pose is unknown.

        Args:
            carried_radius_m (float | None): Carried object radius (m), None without one.

        Returns:
            FootprintState: The published state.
        """
        with self.lock:
            self.set(self.conservative_footprint(carried_radius_m))
            return self.state()

    def tick(self) -> None:
        """Republish tick: follow the released arm, then resend where a consumer needs it."""
        with self.lock:
            if not self.sink.arm_lease_held():
                parts = self.sink.arm_parts()
                carried = self.current.carried_radius_m
                if parts is not None:
                    new = self.measured_footprint(parts, carried)
                    if self.moved(new):
                        self.set(new)
                        return
                    self.current = replace(self.current, pose_known=True)
                elif not self.current.pose_known:
                    new = self.conservative_footprint(carried)
                    if self.moved(new):
                        self.set(new)
                        return
            self.resend()

    def consumer_ready(self, node: str) -> None:
        """A consumer's parameter service came up (restarted): send it the current footprint now.

        Args:
            node (str): Fully qualified node name.
        """
        with self.lock:
            fp = self.current
            for topic, costmap in self.costmaps:
                if costmap == node:
                    self.publish_costmap(topic, fp)
                    self.send_inflation(costmap, fp, force=True)
            if node == self.cfg.laser_filter_node:
                self.send_laser(fp)
            if node == self.cfg.collision_monitor_node:
                self.send_arm_stop_enabled(fp)

    # --- sending ---------------------------------------------------------------------------------------------------

    def set(self, new: Footprint) -> None:
        """Make new the current footprint and send it to every consumer (caller holds the lock).

        Args:
            new (Footprint): Footprint.
        """
        self.current = new
        for topic, node in self.costmaps:
            self.publish_costmap(topic, new)
            self.send_inflation(node, new, force=False)
        if new.arm_stop is not None:
            self.sink.publish_stamped_polygon(self.cfg.arm_stop_topic, self.cfg.frame_id, new.arm_stop)
        self.send_laser(new)
        self.send_arm_stop_enabled(new)

    def resend(self) -> None:
        """Tick resend (caller holds the lock): the footprint to each costmap whose published_footprint differs or
        never arrived (with its inflation), inflation not yet acknowledged, the laser_filter and ArmStop parameters."""
        fp = self.current
        for topic, node in self.costmaps:
            observed_topic = published_footprint_topic(topic)
            observed = self.sink.observed_polygon(observed_topic)
            consumer = consumer_name(node)
            if observed is None:
                self.sink.publish_polygon(topic, fp.points)
                self.record(consumer, FOOTPRINT_CHANNEL, f"unavailable: nothing received on {observed_topic}")
                self.send_inflation(node, fp, force=False)
            elif not same_shape(observed, fp.points):
                self.sink.publish_polygon(topic, fp.points)
                self.record(consumer, FOOTPRINT_CHANNEL, f"republished: {observed_topic} differed (costmap restarted?)")
                self.send_inflation(node, fp, force=True)
            else:
                self.record(consumer, FOOTPRINT_CHANNEL, OK)
                self.send_inflation(node, fp, force=False)
        self.send_laser(fp)
        self.send_arm_stop_enabled(fp)

    def publish_costmap(self, topic: str, fp: Footprint) -> None:
        """Publish the footprint on one costmap input.

        Args:
            topic (str): Costmap footprint topic.
            fp (Footprint): Footprint.
        """
        self.sink.publish_polygon(topic, fp.points)

    def send_inflation(self, node: str, fp: Footprint, force: bool) -> None:
        """Set a costmap's inflation radius and cost scaling (each change re-inflates: only when not acknowledged).

        Args:
            node (str): Costmap node.
            fp (Footprint): Footprint.
            force (bool): Send even when the node acknowledged the value (it restarted).
        """
        layer = self.cfg.inflation_layer
        for name, value in (
            (f"{layer}.{INFLATION_RADIUS}", fp.inflation_radius),
            (f"{layer}.{COST_SCALING_FACTOR}", fp.cost_scaling_factor),
        ):
            with self.status_lock:
                acked = self.acked.get((node, name))
            if force or acked != value:
                self.send_param(node, name, value)

    def send_laser(self, fp: Footprint) -> None:
        """Set the laser_filter self polygon.

        Args:
            fp (Footprint): Footprint.
        """
        self.send_param(self.cfg.laser_filter_node, self.cfg.laser_polygon_param, polygon_text(fp.laser))

    def send_arm_stop_enabled(self, fp: Footprint) -> None:
        """Enable ArmStop unless the footprint is static.

        Args:
            fp (Footprint): Footprint.
        """
        name = f"{self.cfg.arm_stop_polygon}.enabled"
        self.send_param(self.cfg.collision_monitor_node, name, fp.arm_stop is not None)

    def send_param(self, node: str, name: str, value: ParamValue) -> None:
        """Set one consumer parameter and record its outcome when reported.

        Args:
            node (str): Consumer node.
            name (str): Parameter name.
            value (ParamValue): Value.
        """

        def report(outcome: str) -> None:
            with self.status_lock:
                if outcome == OK:
                    self.acked[(node, name)] = value
                else:
                    self.acked.pop((node, name), None)
            self.record(consumer_name(node), name, outcome)

        self.sink.set_node_parameter(node, name, value, report)

    def record(self, consumer: str, channel: str, outcome: str) -> None:
        """Store a consumer outcome.

        Args:
            consumer (str): Consumer name.
            channel (str): footprint or parameter name.
            outcome (str): ok | unavailable: | refused: | timeout: | republished:.
        """
        with self.status_lock:
            self.outcomes.setdefault(consumer, {})[channel] = outcome

    def consumers(self) -> dict[str, str]:
        """Outcome per consumer: its non-ok outcomes joined, ok, or pending before any.

        Returns:
            dict[str, str]: Consumer -> outcome.
        """
        with self.status_lock:
            out = {}
            for consumer, channels in self.outcomes.items():
                bad = sorted({o for o in channels.values() if o != OK})
                out[consumer] = "; ".join(bad) if bad else (OK if channels else PENDING)
            return out

    def arm_outside_static_m(self, parts: dict[str, np.ndarray]) -> float:
        """Largest distance of an arm pose's ground shadow outside the static outline.

        Args:
            parts (dict[str, np.ndarray]): self_collision part_points of the pose (base_link, m).

        Returns:
            float: Metres; 0 when the whole shadow is inside the static outline. The shadow is convex, so the
                farthest point is one of its vertices.
        """
        shadow = arm_shadow(parts, self.cfg.circle_segments)
        return max(Point(x, y).distance(self.static) for x, y in get_coordinates(shadow))

    def circumscribed_radius(self) -> float:
        """Radius the Nav2 costmaps inflate by for the footprint published now.

        Returns:
            float: The circumscribed radius rounded up to a costmap cell (m).
        """
        with self.lock:
            return self.current.inflation_radius

    def state(self) -> FootprintState:
        """What is published now.

        Returns:
            FootprintState: Kind, vertex count, carried radius and consumer outcomes.
        """
        with self.lock:
            fp = self.current
            return FootprintState(
                kind=fp.kind,
                vertices=len(fp.points),
                carried_object_radius_m=fp.carried_radius_m,
                consumers=self.consumers(),
            )


def main(argv: list[str]) -> int:
    """Print the parked-arm shadow for scripts/gen_nav_footprint.py.

    Args:
        argv (list[str]): client.yml path, joints JSON, circle_segments, simplify_m, decimals.

    Returns:
        int: Exit code.
    """
    client_vars, joints, segments, simplify_m, decimals = argv
    nodes = yaml.safe_load(Path(client_vars).read_text())["ros2_nodes"]
    raw = next(n for n in nodes if n["name"] == "mcp_server")["config"]
    data = dict(yaml.safe_load(raw) if isinstance(raw, str) else raw)
    data.pop("nav_footprint", None)  # generated from this output: not needed, and maybe not regenerated yet
    config = McpServerConfig.model_validate(data)
    # The deployed URDF path is the robot's install path: use this checkout's packaged URDF.
    config.arm.urdf_path = McpServerConfig().arm.urdf_path
    shadow = parked_arm_shadow(config, json.loads(joints), int(segments), float(simplify_m), int(decimals))
    print(json.dumps([[x, y] for x, y in shadow]))
    return 0


if __name__ == "__main__":
    logging.basicConfig(level=logging.WARNING)
    sys.exit(main(sys.argv[1:]))
