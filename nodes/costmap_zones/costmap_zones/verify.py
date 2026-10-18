"""Motion-free robot verification of the costmap_zones chain (run on the client in the node venv with rclpy).

python -m costmap_zones.verify (normally via scripts/costmap_zones_verify.sh, which also runs step 0: the
nav2_costmap_2d version and the service states). It reads verify.yaml (env COSTMAP_ZONES_VERIFY_CONFIG, default
the file next to this package) and runs, relative to the robot pose (map -> base_link) at the start:

1. Both masks are received, their grid equals /map, keepout holds only -1, the slow-zone mask value (node config
   slow_zone_mask_value, else verify.yaml, default 60) and 100, and speed only 0..100.
2. Control: compute_path_to_pose (robot -> goal ahead, use_start false) succeeds and crosses the test rectangle.
3. A keepout POI on the rectangle: the mask has 100 at its centre, the global costmap 99/100 there, and the
   planner fails or plans around it.
4. A keepout POI around the robot: the planner still finds a path out (override_lethal_cost).
5. A slow POI on the test rectangle: the keepout mask holds the slow value at its centre, the global costmap there
   equals the published translation of round(v * 254 / 100) (60 -> 152 -> 59), never 99/100, and
   compute_path_to_pose still succeeds (whether the path crosses the rectangle is printed, not judged).
6. A slow POI around the robot: /speed_limit reports slow_pct with percentage true, and 0 after deleting it.
7. Always: every POI named poi_name_prefix* is deleted, and the masks and costmap return to background there.

It only publishes /poi/command and calls the compute_path_to_pose action: the base never moves. It refuses to run
while /navigate_to_pose has an active goal. One PASS/FAIL line per step, then a RESULT line; the exit code is 0 only
when every step passed (1 on a failure, 2 when it refused or could not load its config).
"""

import json
import math
import os
import signal
import sys
import time
import uuid
from collections.abc import Callable, Iterable, Sequence
from dataclasses import dataclass
from functools import partial
from pathlib import Path
from typing import Any, NamedTuple

import cv2
import numpy as np
import rclpy
import yaml
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import OccupancyGrid
from pydantic import BaseModel, ConfigDict, Field, model_validator
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .config import (
    DEFAULT_SLOW_ZONE_MASK_VALUE,
    MAX_SLOW_ZONE_MASK_VALUE,
    MIN_SLOW_ZONE_MASK_VALUE,
    load_config_from_env,
)
from .raster import KEEPOUT_BACKGROUND, KEEPOUT_INSIDE, SPEED_BACKGROUND, MapGeometry

ENV_VERIFY_CONFIG_KEY = "COSTMAP_ZONES_VERIFY_CONFIG"
DEFAULT_VERIFY_CONFIG_PATH = Path(__file__).resolve().parent.parent / "verify.yaml"
NODE_NAME = "costmap_zones_verify"
EXIT_PASS = 0
EXIT_FAIL = 1
EXIT_REFUSED = 2
SPIN_STEP_S = 0.1
SPEED_LIMIT_TOLERANCE = 0.5
MIN_SPEED_LIMIT_PCT = 10
MAX_SPEED_LIMIT_PCT = 100
# action_msgs/GoalStatus: ACCEPTED 1, EXECUTING 2, CANCELING 3 are live; SUCCEEDED 4, CANCELED 5, ABORTED 6 are done.
ACTIVE_GOAL_STATUSES = frozenset({1, 2, 3})
# Costmap -> OccupancyGrid translation (nav2 costmap_2d_publisher): LETHAL 254 -> 100, INSCRIBED 253 -> 99,
# 1..252 -> 1 + 97 * (cost - 1) / 251, so the KeepoutFilter lethal_override_cost 252 (robot inside) -> 98.
KEEPOUT_GRID_VALUES = frozenset({99, 100})
KEEPOUT_GRID_MIN_ROBOT_INSIDE = 98
COSTMAP_INSCRIBED = 253
COSTMAP_LETHAL_COST = 254
COSTMAP_NO_INFORMATION = 255
GRID_SPAN = 97  # OccupancyGrid values 1..98 carry the costs 1..252
COST_SPAN = 251
MASK_SCALE_NUM = 254  # KeepoutFilter: mask value v -> cost round(v * 254 / 100)
MASK_SCALE_DEN = 100
COSTMAP_LETHAL = 100

LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
# Volatile matches both a volatile and a transient_local publisher (costmap, /speed_limit, /poi/result).
VOLATILE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=20,
)

Point = tuple[float, float]


class Timeouts(BaseModel):
    """Waits of the verification, seconds.

    Attributes:
        nav_status_s: Listen for /navigate_to_pose goal status before deciding nothing is active.
        discovery_s: poi_store and planner discovery.
        tf_s: map -> base_link lookup.
        poi_s: /poi/result answer to one command.
        mask_s: costmap_zones republishes the masks.
        costmap_s: The global costmap shows a change.
        plan_s: One compute_path_to_pose call.
        speed_limit_s: /speed_limit reacts to a slow zone.
    """

    model_config = ConfigDict(extra="forbid")

    nav_status_s: float = Field(default=2.0, gt=0)
    discovery_s: float = Field(default=5.0, gt=0)
    tf_s: float = Field(default=5.0, gt=0)
    poi_s: float = Field(default=5.0, gt=0)
    mask_s: float = Field(default=10.0, gt=0)
    costmap_s: float = Field(default=15.0, gt=0)
    plan_s: float = Field(default=20.0, gt=0)
    speed_limit_s: float = Field(default=3.0, gt=0)


class VerifyConfig(BaseModel):
    """verify.yaml: test geometry, topics and timeouts.

    Attributes:
        goal_ahead_m: Planning goal distance straight ahead of the robot, m.
        rect_ahead_m: Centre of the test rectangle on the robot -> goal line, m.
        rect_depth_m: Rectangle size along the heading, m.
        rect_width_m: Rectangle size across the heading, m.
        robot_box_m: Side of the square zone centred on base_link (steps 4 and 6), m.
        slow_pct: Speed limit of the step 6 slow zone, percent (10..100).
        slow_zone_mask_value: Keepout-mask value of slow zones when the costmap_zones node config is unavailable.
        poi_name_prefix: Name prefix of every POI this run creates (and step 7 deletes).
        map_frame: Global frame.
        base_frame: Robot frame.
        map_topic: SLAM map.
        keepout_mask_topic: costmap_zones keepout mask.
        speed_mask_topic: costmap_zones speed mask.
        global_costmap_topic: Global costmap OccupancyGrid.
        speed_limit_topic: nav2_msgs/SpeedLimit from the SpeedFilter.
        poi_command_topic: poi_store command topic.
        poi_result_topic: poi_store result topic.
        poi_list_topic: poi_store latched list.
        compute_path_action: Planner action.
        navigate_status_topic: GoalStatusArray of the navigate_to_pose action.
        timeouts: Waits, seconds.
    """

    model_config = ConfigDict(extra="forbid")

    goal_ahead_m: float = Field(default=1.2, gt=0)
    rect_ahead_m: float = Field(default=0.6, gt=0)
    rect_depth_m: float = Field(default=0.3, gt=0)
    rect_width_m: float = Field(default=1.0, gt=0)
    robot_box_m: float = Field(default=0.6, gt=0)
    slow_pct: int = Field(default=40, ge=MIN_SPEED_LIMIT_PCT, le=MAX_SPEED_LIMIT_PCT)
    slow_zone_mask_value: int = Field(
        default=DEFAULT_SLOW_ZONE_MASK_VALUE, ge=MIN_SLOW_ZONE_MASK_VALUE, le=MAX_SLOW_ZONE_MASK_VALUE
    )
    poi_name_prefix: str = Field(default="E7-verify-", min_length=1)
    map_frame: str = Field(default="map", min_length=1)
    base_frame: str = Field(default="base_link", min_length=1)
    map_topic: str = Field(default="/map", min_length=1)
    keepout_mask_topic: str = Field(default="/costmap_zones/keepout_mask", min_length=1)
    speed_mask_topic: str = Field(default="/costmap_zones/speed_mask", min_length=1)
    global_costmap_topic: str = Field(default="/global_costmap/costmap", min_length=1)
    speed_limit_topic: str = Field(default="/speed_limit", min_length=1)
    poi_command_topic: str = Field(default="/poi/command", min_length=1)
    poi_result_topic: str = Field(default="/poi/result", min_length=1)
    poi_list_topic: str = Field(default="/poi/list", min_length=1)
    compute_path_action: str = Field(default="/compute_path_to_pose", min_length=1)
    navigate_status_topic: str = Field(default="/navigate_to_pose/_action/status", min_length=1)
    timeouts: Timeouts = Field(default_factory=Timeouts)

    @model_validator(mode="after")
    def check_layout(self) -> "VerifyConfig":
        """The rectangle must lie between the robot box and the goal, or the control path cannot cross it.

        Returns:
            VerifyConfig: Self.

        Raises:
            ValueError: The rectangle overlaps the robot box or reaches the goal.
        """
        near = self.rect_ahead_m - self.rect_depth_m / 2
        far = self.rect_ahead_m + self.rect_depth_m / 2
        if near <= self.robot_box_m / 2 or far >= self.goal_ahead_m:
            raise ValueError(
                f"test rectangle [{near:.2f}, {far:.2f}] m ahead must lie between the robot box "
                f"({self.robot_box_m / 2:.2f} m) and the goal ({self.goal_ahead_m:.2f} m)"
            )
        return self


class Pose2D(NamedTuple):
    """Planar pose in the map frame.

    Attributes:
        x: m.
        y: m.
        yaw: rad.
    """

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class StepResult:
    """Outcome of one verification step.

    Attributes:
        number: Step number (1..7; step 0 is in the shell script).
        name: Short step name.
        passed: True when the step passed.
        detail: What was observed.
    """

    number: int
    name: str
    passed: bool
    detail: str


class PlanOutcome(NamedTuple):
    """Result of one compute_path_to_pose call.

    Attributes:
        succeeded: Action status SUCCEEDED with a non-empty path.
        points: Path poses (x, y) in the map frame.
        detail: Status, error code and pose count.
    """

    succeeded: bool
    points: list[Point]
    detail: str


class VerifyError(RuntimeError):
    """A step could not observe what it needs (timeout, missing pose, rejected command); the step fails."""


def verify_config_path() -> Path:
    """Config path from COSTMAP_ZONES_VERIFY_CONFIG, else verify.yaml next to the package.

    Returns:
        Path: Config file path.
    """
    value = os.environ.get(ENV_VERIFY_CONFIG_KEY, "").strip()
    return Path(value) if value else DEFAULT_VERIFY_CONFIG_PATH


def load_verify_config(path: Path) -> VerifyConfig:
    """Load and validate verify.yaml.

    Args:
        path (Path): YAML file.

    Returns:
        VerifyConfig: Parsed config (an empty file gives the defaults).

    Raises:
        FileNotFoundError: The file does not exist.
        pydantic.ValidationError: A value is invalid or a key is unknown.
    """
    data = yaml.safe_load(path.read_text()) or {}
    return VerifyConfig(**data)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Yaw of a quaternion.

    Args:
        x (float): Quaternion x.
        y (float): Quaternion y.
        z (float): Quaternion z.
        w (float): Quaternion w.

    Returns:
        float: Yaw, rad.
    """
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def point_ahead(pose: Pose2D, distance: float) -> Point:
    """The point at a distance along the pose heading.

    Args:
        pose (Pose2D): Start pose.
        distance (float): Distance, m.

    Returns:
        Point: (x, y), m.
    """
    return pose.x + distance * math.cos(pose.yaw), pose.y + distance * math.sin(pose.yaw)


def rectangle_ahead(pose: Pose2D, ahead_m: float, depth_m: float, width_m: float) -> list[Point]:
    """Rectangle centred ahead_m along the heading, depth_m along it and width_m across it (ahead_m 0: around the pose).

    Args:
        pose (Pose2D): Robot pose.
        ahead_m (float): Distance of the rectangle centre along the heading, m.
        depth_m (float): Size along the heading, m.
        width_m (float): Size across the heading, m.

    Returns:
        list[Point]: Four vertices, counter-clockwise.
    """
    cx, cy = point_ahead(pose, ahead_m)
    fx, fy = math.cos(pose.yaw), math.sin(pose.yaw)
    lx, ly = -fy, fx
    corners = ((-1, -1), (1, -1), (1, 1), (-1, 1))
    return [
        (cx + a * depth_m / 2 * fx + b * width_m / 2 * lx, cy + a * depth_m / 2 * fy + b * width_m / 2 * ly)
        for a, b in corners
    ]


def point_in_polygon(point: Point, polygon: Sequence[Point]) -> bool:
    """Whether a point is inside a polygon or on its edge (cv2.pointPolygonTest).

    Args:
        point (Point): (x, y).
        polygon (Sequence[Point]): Vertices.

    Returns:
        bool: True inside or on the boundary.
    """
    contour = np.asarray(polygon, dtype=np.float32).reshape(-1, 1, 2)
    return cv2.pointPolygonTest(contour, (float(point[0]), float(point[1])), False) >= 0


def orientation(a: Point, b: Point, c: Point) -> float:
    """Signed area of the triangle a, b, c (> 0 counter-clockwise).

    Args:
        a (Point): First point.
        b (Point): Second point.
        c (Point): Third point.

    Returns:
        float: Twice the signed area.
    """
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def on_segment(a: Point, b: Point, p: Point) -> bool:
    """Whether a point collinear with segment a-b lies within its bounding box.

    Args:
        a (Point): Segment start.
        b (Point): Segment end.
        p (Point): Collinear point.

    Returns:
        bool: True when p lies on the segment.
    """
    return min(a[0], b[0]) <= p[0] <= max(a[0], b[0]) and min(a[1], b[1]) <= p[1] <= max(a[1], b[1])


def segments_intersect(p1: Point, p2: Point, q1: Point, q2: Point) -> bool:
    """Whether segments p1-p2 and q1-q2 share a point (touching and collinear overlap count).

    Args:
        p1 (Point): First segment start.
        p2 (Point): First segment end.
        q1 (Point): Second segment start.
        q2 (Point): Second segment end.

    Returns:
        bool: True when they intersect.
    """
    d1 = orientation(q1, q2, p1)
    d2 = orientation(q1, q2, p2)
    d3 = orientation(p1, p2, q1)
    d4 = orientation(p1, p2, q2)
    if ((d1 > 0) != (d2 > 0) and d1 != 0 and d2 != 0) and ((d3 > 0) != (d4 > 0) and d3 != 0 and d4 != 0):
        return True
    return any(
        d == 0 and on_segment(a, b, p)
        for d, a, b, p in ((d1, q1, q2, p1), (d2, q1, q2, p2), (d3, p1, p2, q1), (d4, p1, p2, q2))
    )


def path_crosses_polygon(path: Sequence[Point], polygon: Sequence[Point]) -> bool:
    """Whether a path enters a polygon: a pose inside it, or a segment between poses crossing an edge.

    Args:
        path (Sequence[Point]): Path poses (x, y).
        polygon (Sequence[Point]): Vertices.

    Returns:
        bool: True when the path touches the polygon.
    """
    if any(point_in_polygon(p, polygon) for p in path):
        return True
    edges = [(polygon[k - 1], polygon[k]) for k in range(len(polygon))]
    return any(segments_intersect(path[k - 1], path[k], a, b) for k in range(1, len(path)) for a, b in edges)


def grid_geometry(grid: Any) -> MapGeometry:
    """Geometry of an OccupancyGrid message.

    Args:
        grid (Any): nav_msgs/OccupancyGrid.

    Returns:
        MapGeometry: Frame and info.
    """
    return MapGeometry.from_map_info(grid.header.frame_id, grid.info)


def cell_value(geometry: MapGeometry, data: Sequence[int], x: float, y: float) -> int | None:
    """Value of the grid cell holding a map-frame point (row-major, row 0 = lowest y).

    Args:
        geometry (MapGeometry): Grid geometry (axis-aligned).
        data (Sequence[int]): OccupancyGrid data.
        x (float): Map x, m.
        y (float): Map y, m.

    Returns:
        int | None: Cell value, or None outside the grid.
    """
    i = math.floor((x - geometry.origin_x) / geometry.resolution)
    j = math.floor((y - geometry.origin_y) / geometry.resolution)
    if not (0 <= i < geometry.width and 0 <= j < geometry.height):
        return None
    index = j * geometry.width + i
    if index >= len(data):
        return None
    return int(data[index])


def mask_problems(
    map_geometry: MapGeometry,
    keepout_geometry: MapGeometry,
    keepout_data: Sequence[int],
    speed_geometry: MapGeometry,
    speed_data: Sequence[int],
    slow_value: int = DEFAULT_SLOW_ZONE_MASK_VALUE,
) -> list[str]:
    """Step 1 checks: both masks on the /map grid, keepout values only -1/slow_value/100, speed only 0..100.

    Args:
        map_geometry (MapGeometry): /map geometry.
        keepout_geometry (MapGeometry): Keepout mask geometry.
        keepout_data (Sequence[int]): Keepout mask data.
        speed_geometry (MapGeometry): Speed mask geometry.
        speed_data (Sequence[int]): Speed mask data.
        slow_value (int): Keepout-mask value of slow zones.

    Returns:
        list[str]: One message per problem; empty when the masks are right.
    """
    problems: list[str] = []
    for name, geometry, data in (("keepout", keepout_geometry, keepout_data), ("speed", speed_geometry, speed_data)):
        if geometry != map_geometry:
            problems.append(f"{name} mask grid {geometry} differs from /map {map_geometry}")
            continue
        if len(data) != geometry.width * geometry.height:
            problems.append(f"{name} mask has {len(data)} cells, expected {geometry.width * geometry.height}")
            continue
        values = set(np.unique(np.asarray(data, dtype=np.int16)).tolist())
        if name == "keepout":
            bad = sorted(values - {KEEPOUT_BACKGROUND, slow_value, KEEPOUT_INSIDE})
        else:
            bad = sorted(v for v in values if not SPEED_BACKGROUND <= v <= MAX_SPEED_LIMIT_PCT)
        if bad:
            problems.append(f"{name} mask holds invalid values {bad}")
    return problems


def keepout_cost(mask_value: int) -> int:
    """Costmap cost the KeepoutFilter writes for a keepout-mask value (costmap_filter getMaskCost).

    Args:
        mask_value (int): Mask value 1..100.

    Returns:
        int: round(mask_value * 254 / 100), e.g. 60 -> 152, 100 -> 254.
    """
    return (2 * mask_value * MASK_SCALE_NUM + MASK_SCALE_DEN) // (2 * MASK_SCALE_DEN)


def published_grid_value(cost: int) -> int:
    """OccupancyGrid value nav2 costmap_2d_publisher publishes for a costmap cost.

    Args:
        cost (int): Costmap cost 0..255.

    Returns:
        int: 0 -> 0, 253 -> 99, 254 -> 100, 255 -> -1, 1..252 -> 1 + (97 * (cost - 1)) // 251 (152 -> 59).
    """
    if cost == COSTMAP_NO_INFORMATION:
        return -1
    if cost == COSTMAP_LETHAL_COST:
        return COSTMAP_LETHAL
    if cost == COSTMAP_INSCRIBED:
        return 99
    if cost == 0:
        return 0
    return 1 + (GRID_SPAN * (cost - 1)) // COST_SPAN


def resolve_slow_value(config: VerifyConfig) -> int:
    """The slow-zone mask value in force: the costmap_zones node config, else verify.yaml.

    Args:
        config (VerifyConfig): Verification config.

    Returns:
        int: slow_zone_mask_value of the node config when it loads, else config.slow_zone_mask_value.
    """
    try:
        node_config = load_config_from_env()
    except Exception:  # an unreadable node config falls back to verify.yaml
        node_config = None
    return node_config.slow_zone_mask_value if node_config else config.slow_zone_mask_value


def keepout_marked(value: int | None, robot_inside: bool) -> bool:
    """Whether a global costmap OccupancyGrid cell shows a keepout.

    Args:
        value (int | None): Cell value (None outside the grid).
        robot_inside (bool): The robot is inside the zone, so override_lethal_cost writes 252 (grid value 98).

    Returns:
        bool: True for 99/100, or >= 98 when the robot is inside.
    """
    if value is None:
        return False
    if robot_inside:
        return value >= KEEPOUT_GRID_MIN_ROBOT_INSIDE
    return value in KEEPOUT_GRID_VALUES


def speed_limit_matches(speed_limit: float, percentage: bool, expected_pct: int) -> bool:
    """Whether a /speed_limit message is the one the zone state implies.

    Args:
        speed_limit (float): SpeedLimit.speed_limit.
        percentage (bool): SpeedLimit.percentage.
        expected_pct (int): Expected percent; 0 means no limit (percentage then does not matter).

    Returns:
        bool: True when it matches.
    """
    if expected_pct == 0:
        return abs(speed_limit) < SPEED_LIMIT_TOLERANCE
    return percentage and abs(speed_limit - expected_pct) < SPEED_LIMIT_TOLERANCE


def goal_is_active(statuses: Iterable[int]) -> bool:
    """Whether any navigate_to_pose goal is accepted, executing or canceling.

    Args:
        statuses (Iterable[int]): action_msgs/GoalStatus.status values.

    Returns:
        bool: True when a goal is live.
    """
    return any(status in ACTIVE_GOAL_STATUSES for status in statuses)


def zone_poi(name: str, zone: str, polygon: Sequence[Point], speed_limit_pct: int | None = None) -> dict[str, Any]:
    """POI fields of a test zone (an open area created by "user", so an agent session reset leaves it alone).

    Args:
        name (str): POI name (poi_name_prefix + suffix).
        zone (str): "keepout" or "slow".
        polygon (Sequence[Point]): Vertices, map frame.
        speed_limit_pct (int | None): Slow zones: percent.

    Returns:
        dict[str, Any]: The "poi" of an add command.
    """
    poi: dict[str, Any] = {
        "kind": "area",
        "name": name,
        "status": "open",
        "created_by": "user",
        "zone": zone,
        "polygon": [[float(x), float(y)] for x, y in polygon],
    }
    if speed_limit_pct is not None:
        poi["speed_limit_pct"] = speed_limit_pct
    return poi


def poi_command(op: str, request_id: str, poi: dict[str, Any]) -> str:
    """JSON of one /poi/command message.

    Args:
        op (str): "add" or "delete".
        request_id (str): Echoed on /poi/result.
        poi (dict[str, Any]): add: the POI fields; delete: {"id": ...}.

    Returns:
        str: JSON payload.
    """
    return json.dumps({"op": op, "request_id": request_id, "poi": poi})


def parse_result(raw: str, request_id: str) -> dict[str, Any] | None:
    """The /poi/result answer to one request.

    Args:
        raw (str): /poi/result JSON.
        request_id (str): Request to match.

    Returns:
        dict[str, Any] | None: The decoded result, or None when it is unparsable or for another request.
    """
    try:
        data = json.loads(raw)
    except (json.JSONDecodeError, TypeError):
        return None
    if not isinstance(data, dict) or data.get("request_id") != request_id:
        return None
    return data


def verify_poi_ids(raw: str, prefix: str) -> list[str] | None:
    """Ids of the POIs whose name starts with the verification prefix.

    Args:
        raw (str): /poi/list JSON {"pois": [...]}.
        prefix (str): Name prefix.

    Returns:
        list[str] | None: Ids in list order, or None when the list is unparsable.
    """
    try:
        data = json.loads(raw)
    except (json.JSONDecodeError, TypeError):
        return None
    if not isinstance(data, dict) or not isinstance(data.get("pois"), list):
        return None
    return [
        str(entry["id"])
        for entry in data["pois"]
        if isinstance(entry, dict) and "id" in entry and str(entry.get("name", "")).startswith(prefix)
    ]


def format_result(result: StepResult) -> str:
    """One PASS/FAIL line.

    Args:
        result (StepResult): Step outcome.

    Returns:
        str: "PASS step N name: detail" or "FAIL step N name: detail".
    """
    return f"{'PASS' if result.passed else 'FAIL'} step {result.number} {result.name}: {result.detail}"


def summarise(results: Sequence[StepResult]) -> tuple[list[str], int]:
    """Step lines plus a RESULT line, and the exit code.

    Args:
        results (Sequence[StepResult]): Step outcomes in run order.

    Returns:
        tuple[list[str], int]: Lines to print and EXIT_PASS only when steps ran and all passed, else EXIT_FAIL.
    """
    if not results:
        return ["RESULT: FAIL (no steps ran)"], EXIT_FAIL
    lines = [format_result(result) for result in results]
    failed = [result.number for result in results if not result.passed]
    if failed:
        numbers = ", ".join(str(n) for n in failed)
        lines.append(f"RESULT: FAIL ({len(failed)} of {len(results)} steps failed: {numbers})")
        return lines, EXIT_FAIL
    lines.append(f"RESULT: PASS ({len(results)}/{len(results)} steps)")
    return lines, EXIT_PASS


class Verifier:
    """Runs steps 1-7 against the live graph. Creates exactly one publisher (/poi/command); never a velocity one."""

    def __init__(self, node: Any, config: VerifyConfig) -> None:
        """Create the publisher, subscriptions, planner client and TF listener.

        Args:
            node (Any): rclpy node.
            config (VerifyConfig): Verification config.
        """
        self.node = node
        self.config = config
        self.latest: dict[str, Any] = {}
        self.poi_results: list[str] = []
        self.results: list[StepResult] = []
        self.pose: Pose2D | None = None
        self.slow_value = resolve_slow_value(config)
        self.command_pub = node.create_publisher(String, config.poi_command_topic, VOLATILE_QOS)
        node.create_subscription(String, config.poi_result_topic, self.on_poi_result, VOLATILE_QOS)
        for key, msg_type, topic, qos in (
            ("map", OccupancyGrid, config.map_topic, LATCHED_QOS),
            ("keepout", OccupancyGrid, config.keepout_mask_topic, LATCHED_QOS),
            ("speed", OccupancyGrid, config.speed_mask_topic, LATCHED_QOS),
            ("poi_list", String, config.poi_list_topic, LATCHED_QOS),
            ("nav_status", GoalStatusArray, config.navigate_status_topic, LATCHED_QOS),
            ("costmap", OccupancyGrid, config.global_costmap_topic, VOLATILE_QOS),
            ("speed_limit", SpeedLimit, config.speed_limit_topic, VOLATILE_QOS),
        ):
            node.create_subscription(msg_type, topic, partial(self.store, key), qos)
        self.plan_client = ActionClient(node, ComputePathToPose, config.compute_path_action)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

    def store(self, key: str, msg: Any) -> None:
        """Keep the latest message of one subscription.

        Args:
            key (str): Subscription key.
            msg (Any): Message.
        """
        self.latest[key] = msg

    def on_poi_result(self, msg: String) -> None:
        """Keep every /poi/result payload for request matching.

        Args:
            msg (String): JSON result.
        """
        self.poi_results.append(msg.data)

    def log(self, text: str) -> None:
        """Progress line on stderr (stdout carries only the PASS/FAIL lines).

        Args:
            text (str): Message.
        """
        print(text, file=sys.stderr, flush=True)

    def wait_until(self, predicate: Callable[[], Any], timeout_s: float) -> Any:
        """Spin until the predicate returns a truthy value or the timeout passes.

        Args:
            predicate (Callable[[], Any]): Checked before each spin.
            timeout_s (float): Maximum wait, s.

        Returns:
            Any: The truthy predicate value, or None on timeout.
        """
        deadline = time.monotonic() + timeout_s
        while True:
            value = predicate()
            if value:
                return value
            if time.monotonic() >= deadline:
                return None
            rclpy.spin_once(self.node, timeout_sec=SPIN_STEP_S)

    def latest_cell(self, key: str, point: Point) -> int | None:
        """Cell value of the latest grid of one subscription at a map point.

        Args:
            key (str): "keepout", "speed" or "costmap".
            point (Point): (x, y), map frame.

        Returns:
            int | None: Value, or None without a grid or outside it.
        """
        grid = self.latest.get(key)
        if grid is None:
            return None
        return cell_value(grid_geometry(grid), grid.data, point[0], point[1])

    def navigation_active(self) -> bool:
        """Whether /navigate_to_pose has an accepted, executing or canceling goal (latched status).

        Returns:
            bool: True when a goal is live; False when none is or no status arrived within nav_status_s.
        """
        msg = self.wait_until(lambda: self.latest.get("nav_status"), self.config.timeouts.nav_status_s)
        if msg is None:
            return False
        return goal_is_active(status.status for status in msg.status_list)

    def robot_pose(self) -> Pose2D | None:
        """Current map -> base_frame pose.

        Returns:
            Pose2D | None: Pose, or None when the transform is not available yet.
        """
        try:
            transform = self.tf_buffer.lookup_transform(self.config.map_frame, self.config.base_frame, Time())
        except TransformException:
            return None
        t = transform.transform
        q = t.rotation
        return Pose2D(t.translation.x, t.translation.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def poi_request(self, op: str, poi: dict[str, Any]) -> dict[str, Any]:
        """Send one /poi/command and wait for its /poi/result.

        Args:
            op (str): "add" or "delete".
            poi (dict[str, Any]): Command POI.

        Returns:
            dict[str, Any]: The ok result.

        Raises:
            VerifyError: poi_store not discovered, no answer, or ok false.
        """
        config = self.config
        discovered = self.wait_until(
            lambda: (
                self.node.count_subscribers(config.poi_command_topic) > 0
                and self.node.count_publishers(config.poi_result_topic) > 0
            ),
            config.timeouts.discovery_s,
        )
        if not discovered:
            raise VerifyError(f"poi_store not discovered on {config.poi_command_topic} / {config.poi_result_topic}")
        request_id = uuid.uuid4().hex
        self.command_pub.publish(String(data=poi_command(op, request_id, poi)))

        def answer() -> dict[str, Any] | None:
            for raw in self.poi_results:
                result = parse_result(raw, request_id)
                if result is not None:
                    return result
            return None

        result = self.wait_until(answer, config.timeouts.poi_s)
        if result is None:
            raise VerifyError(f"no /poi/result for {op} within {config.timeouts.poi_s} s")
        if not result.get("ok"):
            raise VerifyError(f"poi_store refused {op}: {result.get('message')}")
        return result

    def add_zone(self, suffix: str, zone: str, polygon: Sequence[Point], speed_limit_pct: int | None = None) -> str:
        """Add a test zone POI.

        Args:
            suffix (str): Name after the prefix.
            zone (str): "keepout" or "slow".
            polygon (Sequence[Point]): Vertices.
            speed_limit_pct (int | None): Slow zones: percent.

        Returns:
            str: POI id.

        Raises:
            VerifyError: The add failed.
        """
        name = self.config.poi_name_prefix + suffix
        result = self.poi_request("add", zone_poi(name, zone, polygon, speed_limit_pct))
        poi = result.get("poi") or {}
        if not poi.get("id"):
            raise VerifyError(f"poi_store returned no id for {name}")
        return str(poi["id"])

    def delete_poi(self, poi_id: str) -> None:
        """Delete one POI.

        Args:
            poi_id (str): POI id.

        Raises:
            VerifyError: The delete failed.
        """
        self.poi_request("delete", {"id": poi_id})

    def delete_verify_pois(self) -> int:
        """Delete every POI whose name starts with poi_name_prefix and wait until /poi/list has none.

        Returns:
            int: Number deleted.

        Raises:
            VerifyError: /poi/list missing or unparsable, a delete failed, or the list still has them.
        """
        prefix = self.config.poi_name_prefix
        msg = self.wait_until(lambda: self.latest.get("poi_list"), self.config.timeouts.discovery_s)
        if msg is None:
            raise VerifyError(f"no {self.config.poi_list_topic} received")
        ids = verify_poi_ids(msg.data, prefix)
        if ids is None:
            raise VerifyError(f"{self.config.poi_list_topic} is not a POI list")
        for poi_id in ids:
            self.delete_poi(poi_id)
        gone = self.wait_until(
            lambda: verify_poi_ids(self.latest["poi_list"].data, prefix) == [], self.config.timeouts.poi_s
        )
        if not gone:
            raise VerifyError(f"{prefix}* POIs still listed after delete")
        return len(ids)

    def plan(self, goal: Point, yaw: float) -> PlanOutcome:
        """compute_path_to_pose from the current robot pose (use_start false) to a map-frame goal.

        Args:
            goal (Point): Goal (x, y).
            yaw (float): Goal heading, rad.

        Returns:
            PlanOutcome: Success, path points and detail.

        Raises:
            VerifyError: Planner unavailable or no answer in time.
        """
        timeouts = self.config.timeouts
        if not self.plan_client.wait_for_server(timeout_sec=timeouts.discovery_s):
            raise VerifyError(f"{self.config.compute_path_action} server not available")
        request = ComputePathToPose.Goal()
        request.goal = PoseStamped()
        request.goal.header.frame_id = self.config.map_frame
        request.goal.header.stamp = self.node.get_clock().now().to_msg()
        request.goal.pose.position.x = float(goal[0])
        request.goal.pose.position.y = float(goal[1])
        request.goal.pose.orientation.z = math.sin(yaw / 2)
        request.goal.pose.orientation.w = math.cos(yaw / 2)
        request.use_start = False
        sent = self.plan_client.send_goal_async(request)
        rclpy.spin_until_future_complete(self.node, sent, timeout_sec=timeouts.plan_s)
        if not sent.done():
            raise VerifyError(f"compute_path_to_pose goal not answered within {timeouts.plan_s} s")
        handle = sent.result()
        if not handle.accepted:
            return PlanOutcome(False, [], "goal rejected")
        pending = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, pending, timeout_sec=timeouts.plan_s)
        if not pending.done():
            handle.cancel_goal_async()
            raise VerifyError(f"compute_path_to_pose gave no result within {timeouts.plan_s} s")
        wrapped = pending.result()
        points = [(p.pose.position.x, p.pose.position.y) for p in wrapped.result.path.poses]
        succeeded = wrapped.status == GoalStatus.STATUS_SUCCEEDED and bool(points)
        detail = f"status {wrapped.status}, error_code {wrapped.result.error_code}, {len(points)} poses"
        return PlanOutcome(succeeded, points, detail)

    def require_pose(self) -> Pose2D:
        """The robot pose recorded in step 2.

        Returns:
            Pose2D: Pose.

        Raises:
            VerifyError: Step 2 could not look it up.
        """
        if self.pose is None:
            raise VerifyError(f"no robot pose ({self.config.map_frame} -> {self.config.base_frame} lookup failed)")
        return self.pose

    def goal(self) -> Point:
        """Planning goal ahead of the step 2 pose.

        Returns:
            Point: (x, y).
        """
        return point_ahead(self.require_pose(), self.config.goal_ahead_m)

    def rectangle(self) -> list[Point]:
        """Test rectangle ahead of the step 2 pose.

        Returns:
            list[Point]: Vertices.
        """
        c = self.config
        return rectangle_ahead(self.require_pose(), c.rect_ahead_m, c.rect_depth_m, c.rect_width_m)

    def robot_box(self) -> list[Point]:
        """Square around the step 2 pose.

        Returns:
            list[Point]: Vertices.
        """
        return rectangle_ahead(self.require_pose(), 0.0, self.config.robot_box_m, self.config.robot_box_m)

    def step_masks(self) -> str:
        """Step 1: both masks received, on the /map grid, with valid values.

        Returns:
            str: Detail.

        Raises:
            VerifyError: Missing message or a mask problem after mask_s.
        """
        problems: list[str] = []

        def masks_ok() -> bool:
            grids = [self.latest.get(key) for key in ("map", "keepout", "speed")]
            if any(grid is None for grid in grids):
                problems[:] = [f"no {key} yet" for key, grid in zip(("map", "keepout", "speed"), grids) if grid is None]
                return False
            map_grid, keepout, speed = grids
            problems[:] = mask_problems(
                grid_geometry(map_grid),
                grid_geometry(keepout),
                keepout.data,
                grid_geometry(speed),
                speed.data,
                self.slow_value,
            )
            return not problems

        if not self.wait_until(masks_ok, self.config.timeouts.mask_s):
            raise VerifyError("; ".join(problems))
        geometry = grid_geometry(self.latest["map"])
        keepout_cells = int(np.count_nonzero(np.asarray(self.latest["keepout"].data) == KEEPOUT_INSIDE))
        speed_cells = int(np.count_nonzero(np.asarray(self.latest["speed"].data) != SPEED_BACKGROUND))
        return (
            f"masks match /map {geometry.width}x{geometry.height} @ {geometry.resolution} m; "
            f"{keepout_cells} keepout cells, {speed_cells} slow cells"
        )

    def step_control(self) -> str:
        """Step 2: record the pose; compute_path_to_pose to the goal succeeds and crosses the rectangle.

        Returns:
            str: Detail.

        Raises:
            VerifyError: No pose, failed plan, or a path that misses the rectangle.
        """
        self.pose = self.wait_until(self.robot_pose, self.config.timeouts.tf_s)
        pose = self.require_pose()
        outcome = self.plan(self.goal(), pose.yaw)
        if not outcome.succeeded:
            raise VerifyError(f"control plan failed ({outcome.detail}); is the space ahead free?")
        if not path_crosses_polygon(outcome.points, self.rectangle()):
            raise VerifyError(f"control path misses the test rectangle ({outcome.detail})")
        return f"robot ({pose.x:.2f}, {pose.y:.2f}, yaw {pose.yaw:.2f}); path crosses the rectangle ({outcome.detail})"

    def step_keepout_ahead(self) -> str:
        """Step 3: keepout on the rectangle -> mask 100, global costmap 99/100 at its centre, plan fails or avoids it.

        Returns:
            str: Detail.

        Raises:
            VerifyError: Any expectation unmet.
        """
        rect = self.rectangle()
        centre = point_ahead(self.require_pose(), self.config.rect_ahead_m)
        timeouts = self.config.timeouts
        poi_id = self.add_zone("keepout", "keepout", rect)
        try:
            if not self.wait_until(lambda: self.latest_cell("keepout", centre) == KEEPOUT_INSIDE, timeouts.mask_s):
                raise VerifyError(f"keepout mask centre cell is {self.latest_cell('keepout', centre)}, not 100")
            marked = self.wait_until(
                lambda: keepout_marked(self.latest_cell("costmap", centre), robot_inside=False), timeouts.costmap_s
            )
            if not marked:
                raise VerifyError(f"global costmap centre cell is {self.latest_cell('costmap', centre)}, not 99/100")
            outcome = self.plan(self.goal(), self.require_pose().yaw)
            if outcome.succeeded and path_crosses_polygon(outcome.points, rect):
                raise VerifyError(f"path still crosses the keepout ({outcome.detail})")
            costmap = self.latest_cell("costmap", centre)
            verdict = "planner refused" if not outcome.succeeded else "path avoids the keepout"
            return f"mask 100, costmap {costmap} at the centre; {verdict} ({outcome.detail})"
        finally:
            self.delete_poi(poi_id)
            self.wait_until(lambda: self.latest_cell("keepout", centre) != KEEPOUT_INSIDE, timeouts.mask_s)

    def step_keepout_robot(self) -> str:
        """Step 4: keepout around the robot -> costmap marked under it, a path out still succeeds.

        Returns:
            str: Detail.

        Raises:
            VerifyError: Any expectation unmet.
        """
        pose = self.require_pose()
        robot = (pose.x, pose.y)
        timeouts = self.config.timeouts
        poi_id = self.add_zone("robot-keepout", "keepout", self.robot_box())
        try:
            if not self.wait_until(lambda: self.latest_cell("keepout", robot) == KEEPOUT_INSIDE, timeouts.mask_s):
                raise VerifyError(f"keepout mask robot cell is {self.latest_cell('keepout', robot)}, not 100")
            marked = self.wait_until(
                lambda: keepout_marked(self.latest_cell("costmap", robot), robot_inside=True), timeouts.costmap_s
            )
            if not marked:
                raise VerifyError(f"global costmap robot cell is {self.latest_cell('costmap', robot)}, not >= 98")
            outcome = self.plan(self.goal(), pose.yaw)
            if not outcome.succeeded:
                raise VerifyError(f"no path out of the keepout ({outcome.detail})")
            return f"costmap {self.latest_cell('costmap', robot)} under the robot; path out succeeds ({outcome.detail})"
        finally:
            self.delete_poi(poi_id)
            self.wait_until(lambda: self.latest_cell("keepout", robot) != KEEPOUT_INSIDE, timeouts.mask_s)

    def speed_limit_is(self, expected_pct: int) -> bool:
        """Whether the latest /speed_limit matches.

        Args:
            expected_pct (int): Expected percent, 0 for no limit.

        Returns:
            bool: True when a message arrived and matches.
        """
        msg = self.latest.get("speed_limit")
        return msg is not None and speed_limit_matches(msg.speed_limit, msg.percentage, expected_pct)

    def describe_speed_limit(self) -> str:
        """The latest /speed_limit for a failure message.

        Returns:
            str: "speed_limit X, percentage Y" or "no message".
        """
        msg = self.latest.get("speed_limit")
        if msg is None:
            return "no /speed_limit message"
        return f"speed_limit {msg.speed_limit}, percentage {msg.percentage}"

    def step_slow_cost(self) -> str:
        """Step 5: slow zone on the rectangle -> mask holds the slow value, costmap is the non-lethal translation.

        Returns:
            str: Detail.

        Raises:
            VerifyError: Any expectation unmet, or the plan fails.
        """
        rect = self.rectangle()
        centre = point_ahead(self.require_pose(), self.config.rect_ahead_m)
        timeouts = self.config.timeouts
        value = self.slow_value
        cost = keepout_cost(value)
        expected = published_grid_value(cost)
        poi_id = self.add_zone("slow-cost", "slow", rect, speed_limit_pct=self.config.slow_pct)
        try:
            if not self.wait_until(lambda: self.latest_cell("keepout", centre) == value, timeouts.mask_s):
                raise VerifyError(f"keepout mask centre cell is {self.latest_cell('keepout', centre)}, not {value}")
            if not self.wait_until(lambda: self.latest_cell("costmap", centre) == expected, timeouts.costmap_s):
                raise VerifyError(
                    f"global costmap centre cell is {self.latest_cell('costmap', centre)}, "
                    f"expected {expected} (cost {cost}), never 99/100"
                )
            outcome = self.plan(self.goal(), self.require_pose().yaw)
            if not outcome.succeeded:
                raise VerifyError(f"plan through the slow zone failed ({outcome.detail})")
            crossing = "crosses" if path_crosses_polygon(outcome.points, rect) else "avoids"
            return (
                f"mask {value}, costmap {expected} (cost {cost}) at the centre; plan succeeds and {crossing} "
                f"the slow zone ({outcome.detail})"
            )
        finally:
            self.delete_poi(poi_id)
            self.wait_until(lambda: self.latest_cell("keepout", centre) != value, timeouts.mask_s)

    def step_slow_robot(self) -> str:
        """Step 6: slow zone around the robot -> /speed_limit slow_pct percent, then 0 after deleting it.

        Returns:
            str: Detail.

        Raises:
            VerifyError: Either expectation unmet within speed_limit_s.
        """
        pct = self.config.slow_pct
        timeout = self.config.timeouts.speed_limit_s
        self.latest.pop("speed_limit", None)
        poi_id = self.add_zone("slow", "slow", self.robot_box(), speed_limit_pct=pct)
        try:
            if not self.wait_until(lambda: self.speed_limit_is(pct), timeout):
                raise VerifyError(f"expected {pct} % within {timeout} s, got {self.describe_speed_limit()}")
        finally:
            self.latest.pop("speed_limit", None)
            self.delete_poi(poi_id)
        if not self.wait_until(lambda: self.speed_limit_is(0), timeout):
            raise VerifyError(f"expected 0 within {timeout} s after delete, got {self.describe_speed_limit()}")
        return f"/speed_limit {pct} % (percentage true), then 0 after delete"

    def step_cleanup(self) -> str:
        """Step 7: delete every prefixed POI; masks back to background and the costmap centre cell no longer 100.

        Returns:
            str: Detail.

        Raises:
            VerifyError: A delete failed or a cell kept its zone value.
        """
        deleted = self.delete_verify_pois()
        if self.pose is None:
            return f"deleted {deleted} POIs (no robot pose: cell checks skipped)"
        timeouts = self.config.timeouts
        centre = point_ahead(self.pose, self.config.rect_ahead_m)
        robot = (self.pose.x, self.pose.y)
        checks = (
            ("keepout mask at the rectangle", lambda: self.latest_cell("keepout", centre) == KEEPOUT_BACKGROUND),
            ("keepout mask at the robot", lambda: self.latest_cell("keepout", robot) == KEEPOUT_BACKGROUND),
            ("speed mask at the robot", lambda: self.latest_cell("speed", robot) == SPEED_BACKGROUND),
        )
        for label, check in checks:
            if not self.wait_until(check, timeouts.mask_s):
                raise VerifyError(f"{label} did not return to background")
        if not self.wait_until(
            lambda: self.latest_cell("costmap", centre) not in (None, COSTMAP_LETHAL), timeouts.costmap_s
        ):
            raise VerifyError(f"global costmap centre cell still {self.latest_cell('costmap', centre)}")
        return f"deleted {deleted} POIs; masks at background, costmap centre {self.latest_cell('costmap', centre)}"

    def run_step(self, number: int, name: str, step: Callable[[], str]) -> StepResult:
        """Run one step; any error becomes a FAIL with its message.

        Args:
            number (int): Step number.
            name (str): Step name.
            step (Callable[[], str]): Returns the PASS detail or raises.

        Returns:
            StepResult: Outcome (also appended to self.results).
        """
        self.log(f"step {number} {name} ...")
        try:
            result = StepResult(number, name, True, step())
        except VerifyError as exc:
            result = StepResult(number, name, False, str(exc))
        except Exception as exc:  # a crash in one step is a FAIL, cleanup still runs
            result = StepResult(number, name, False, f"{type(exc).__name__}: {exc}")
        self.results.append(result)
        self.log(format_result(result))
        return result

    def run(self) -> list[StepResult]:
        """Remove leftovers of an interrupted run, then steps 1-6, then always step 7.

        Returns:
            list[StepResult]: Outcomes in run order.
        """
        try:
            try:
                leftovers = self.delete_verify_pois()
                if leftovers:
                    self.log(f"removed {leftovers} leftover {self.config.poi_name_prefix}* POIs")
            except VerifyError as exc:
                self.log(f"leftover cleanup failed: {exc}")
            self.run_step(1, "masks", self.step_masks)
            self.run_step(2, "control path", self.step_control)
            self.run_step(3, "keepout ahead", self.step_keepout_ahead)
            self.run_step(4, "keepout around robot", self.step_keepout_robot)
            self.run_step(5, "slow zone cost", self.step_slow_cost)
            self.run_step(6, "slow zone", self.step_slow_robot)
        finally:
            self.run_step(7, "cleanup", self.step_cleanup)
        return self.results


def raise_interrupt(_signum: int, _frame: Any) -> None:
    """SIGTERM/SIGHUP (ssh dropped) -> KeyboardInterrupt, so the cleanup step still runs.

    Raises:
        KeyboardInterrupt: Always.
    """
    raise KeyboardInterrupt


def main() -> int:
    """Load the config, refuse during navigation, run the steps and print the PASS/FAIL lines.

    Returns:
        int: EXIT_PASS, EXIT_FAIL or EXIT_REFUSED.
    """
    path = verify_config_path()
    try:
        config = load_verify_config(path)
    except FileNotFoundError:
        print(f"REFUSED: verify config {path} not found (set {ENV_VERIFY_CONFIG_KEY})", flush=True)
        return EXIT_REFUSED
    # Our own signal handling: rclpy's would shut the context down and break the cleanup step.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    for signum in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, raise_interrupt)
    node = rclpy.create_node(NODE_NAME)
    verifier = Verifier(node, config)
    interrupted = False
    try:
        if verifier.navigation_active():
            print("REFUSED: /navigate_to_pose has an active goal; cancel it before verifying", flush=True)
            return EXIT_REFUSED
        verifier.run()
    except KeyboardInterrupt:
        interrupted = True
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    lines, code = summarise(verifier.results)
    for line in lines:
        print(line, flush=True)
    if interrupted:
        print("INTERRUPTED: not every step ran", flush=True)
        return EXIT_FAIL
    return code


if __name__ == "__main__":
    sys.exit(main())
