"""RosRobot.move_relative near obstacles (robot agent retrospective 2026-10-22 16:28, "Explore"), without ROS: the ROS
packages are mocks (conftest ros_iface fixture), the base is a simulated pose that integrates the published twists.

Covers: a short backing move (and a sideways slide) runs as one straight twist on cmd_vel_nav, no Nav2 goal and no
in-place turn first, and every twist it publishes selects the shipped StopBox `back` sub-polygon (not `rotate`), also
with a front-left return at 0.34 m; a turn (dyaw != 0, or a Nav2 goal behind the robot that starts with an in-place
turn) is refused before any command when the lidar shows a return inside the rotation circle (footprint circumscribed
radius + look_around.clearance_margin_m), naming the nearest return and a translation away; no scan refuses it; a clear
sweep still sends the Nav2 goal; with the shipped nav2_params.yaml polygons a 4-point cluster 0.34 m front-left lies
outside the `back` and `back_left` sub-polygons (and inside `rotate`, the polygon an in-place turn checks); the tool
description states the backing and the refusal.
"""

import math
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import cv2
import numpy as np
import pytest
import yaml

from mcp_server.config import McpServerConfig
from mcp_server.look_around import required_clearance_m
from mcp_server.models import BasePose, MapSummary, NavigationResult, RobotError, SectorObstacle
from mcp_server.tools import build_mcp_server

from .test_tools import TOKEN, FakeRobot, tool_descriptions

NAV2_PARAMS = Path(__file__).resolve().parents[3] / "nodes" / "nav2_bringup" / "config" / "nav2_params.yaml"
SECTOR_NAMES = ("front", "front_left", "left", "rear_left", "rear", "rear_right", "right", "front_right")
RETRO_BEARING_RAD = math.radians(30.0)  # front_left return of the retrospective, 0.34 m from base_link
RETRO_DISTANCE_M = 0.34
CLUSTER_SPREAD_DEG = (-1.5, -0.5, 0.5, 1.5)  # 4 points (StopBox min_points) about 2 cm apart along the arc
START = BasePose(frame="map", x=-2.58, y=1.61, yaw=2.73)


def stopbox() -> dict[str, Any]:
    """StopBox parameters of the shipped collision_monitor config.

    Returns:
        dict[str, Any]: The StopBox parameter block.
    """
    params = yaml.safe_load(NAV2_PARAMS.read_text())
    return params["collision_monitor"]["ros__parameters"]["StopBox"]


def sub_polygon(name: str) -> np.ndarray:
    """Points of one StopBox sub-polygon.

    Args:
        name (str): Sub-polygon name.

    Returns:
        np.ndarray: Nx2 float32 points (base_link, m).
    """
    return np.array(yaml.safe_load(stopbox()[name]["points"]), dtype=np.float32)


def inside(polygon: np.ndarray, x: float, y: float) -> bool:
    """Whether a point is inside or on the polygon.

    Args:
        polygon (np.ndarray): Nx2 float32 points.
        x (float): Point x (m).
        y (float): Point y (m).

    Returns:
        bool: True when inside or on the edge.
    """
    return cv2.pointPolygonTest(polygon.reshape(-1, 1, 2), (x, y), False) >= 0


def cluster(bearing_rad: float, distance_m: float) -> list[tuple[float, float]]:
    """A 4-point obstacle cluster at a distance from base_link around a bearing.

    Args:
        bearing_rad (float): Centre bearing (rad, 0 = ahead, + = left).
        distance_m (float): Distance of every point from base_link (m).

    Returns:
        list[tuple[float, float]]: (x, y) points in base_link.
    """
    return [
        (distance_m * math.cos(bearing_rad + math.radians(d)), distance_m * math.sin(bearing_rad + math.radians(d)))
        for d in CLUSTER_SPREAD_DEG
    ]


def selected_sub_polygon(vx: float, vy: float, wz: float) -> str:
    """StopBox sub-polygon nav2 VelocityPolygon picks for a command (holonomic: speed magnitude, direction
    atan2(vy, vx), yaw rate; first match in velocity_polygons order wins).

    Args:
        vx (float): Forward velocity (m/s).
        vy (float): Left velocity (m/s).
        wz (float): Yaw rate (rad/s).

    Returns:
        str: Sub-polygon name.
    """
    box = stopbox()
    speed = math.hypot(vx, vy)
    direction = math.atan2(vy, vx)
    for name in box["velocity_polygons"]:
        p = box[name]
        start, end = p["direction_start_angle"], p["direction_end_angle"]
        in_direction = start <= direction <= end if start <= end else direction >= start or direction <= end
        if p["linear_min"] <= speed <= p["linear_max"] and p["theta_min"] <= wz <= p["theta_max"] and in_direction:
            return str(name)
    raise AssertionError(f"no StopBox sub-polygon for ({vx}, {vy}, {wz})")


def sectors_with(sector: str | None = None, distance_m: float = 0.0, bearing_rad: float = 0.0) -> list[SectorObstacle]:
    """Lidar sectors: every sector free at 2 m, one optional close return.

    Args:
        sector (str | None): Sector of the close return, None for none.
        distance_m (float): Its distance (m).
        bearing_rad (float): Its bearing (rad).

    Returns:
        list[SectorObstacle]: The eight sectors.
    """
    out = [SectorObstacle(sector=n, nearest_m=2.0, bearing_rad=i * math.pi / 4) for i, n in enumerate(SECTOR_NAMES)]
    if sector is not None:
        out[SECTOR_NAMES.index(sector)] = SectorObstacle(sector=sector, nearest_m=distance_m, bearing_rad=bearing_rad)
    return out


class SimBase:
    """Simulated holonomic base: the latest published twist (base_link frame) moves the map pose on every sleep."""

    def __init__(self, pose: BasePose) -> None:
        self.x, self.y, self.yaw = pose.x, pose.y, pose.yaw
        self.frame = pose.frame
        self.clock = 0.0
        self.twist = (0.0, 0.0, 0.0)
        self.published: list[tuple[float, float, float]] = []

    def publish(self, vx: float, vy: float, wz: float) -> None:
        self.twist = (vx, vy, wz)
        self.published.append(self.twist)

    def monotonic(self) -> float:
        return self.clock

    def sleep(self, seconds: float) -> None:
        vx, vy, wz = self.twist
        self.x += (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * seconds
        self.y += (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * seconds
        self.yaw += wz * seconds
        self.clock += seconds

    def pose(self) -> BasePose:
        return BasePose(frame=self.frame, x=self.x, y=self.y, yaw=self.yaw, age_s=0.01)


def make_robot(module: ModuleType, monkeypatch: pytest.MonkeyPatch, obstacles: list[SectorObstacle] | None) -> Any:
    """RosRobot without __init__: default config, a simulated base, a mocked Nav2 navigate and a fixed lidar summary.

    Args:
        module (ModuleType): The ros_iface module (mocked ROS).
        monkeypatch (pytest.MonkeyPatch): Patcher (the module's clock is the simulated one).
        obstacles (list[SectorObstacle] | None): Lidar sectors map_summary returns, None for no fresh scan.

    Returns:
        Any: The robot; robot.sim is the base, robot.navigate and robot.map_summary are mocks.
    """
    robot = module.RosRobot.__new__(module.RosRobot)
    robot.cfg = McpServerConfig()
    robot.base_motion = module.threading.Lock()
    robot.base_stop = module.threading.Event()
    robot.monitor = MagicMock()
    robot.monitor.watch.return_value.check.return_value = None
    robot.motion_rate = MagicMock()
    sim = SimBase(START)
    robot.sim = sim
    robot.robot_pose = sim.pose
    robot.publish_twist = sim.publish
    robot.navigate = MagicMock(return_value=NavigationResult(status="succeeded", message=""))
    robot.map_summary = MagicMock(return_value=(MapSummary(obstacles=obstacles), None))
    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=sim.monotonic, sleep=sim.sleep))
    return robot


@pytest.mark.parametrize("bearing_deg", range(0, 61, 5))
def test_back_polygon_clear_of_front_left_obstacle(bearing_deg: int) -> None:
    points = cluster(math.radians(bearing_deg), RETRO_DISTANCE_M)
    for name in ("back", "back_left"):
        polygon = sub_polygon(name)
        assert not any(inside(polygon, x, y) for x, y in points), f"{name} holds the cluster at {bearing_deg} deg"
    # The retrospective's return (0.34 m) was held by the old 0.35 m rotation circle (why the turns stopped); `rotate` is
    # now the footprint + 2 cm, which holds no return at that distance.
    rotate = sub_polygon("rotate")
    assert not any(inside(rotate, x, y) for x, y in points), f"rotate holds the cluster at {bearing_deg} deg"


def test_backing_move_does_not_turn_first(ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch) -> None:
    robot = make_robot(ros_iface, monkeypatch, sectors_with("front_left", RETRO_DISTANCE_M, RETRO_BEARING_RAD))
    result = robot.move_relative(-0.35, 0.0, 0.0, 30.0)
    robot.navigate.assert_not_called()  # no Nav2 goal: RotationShim would turn the robot to face backwards first
    moving = [t for t in robot.sim.published if t != (0.0, 0.0, 0.0)]
    assert moving, "the backing move published no twist"
    assert all(wz == 0.0 for _, _, wz in robot.sim.published)
    assert {selected_sub_polygon(*t) for t in moving} == {"back"}
    assert robot.sim.published[-1] == (0.0, 0.0, 0.0)
    assert result.status == "succeeded"
    assert result.final_pose is not None
    travelled = math.hypot(result.final_pose.x - START.x, result.final_pose.y - START.y)
    assert travelled == pytest.approx(0.35, abs=robot.cfg.nav.intermediate_xy_tolerance_m)
    assert result.final_pose.yaw == pytest.approx(START.yaw)
    # It moved backwards along the robot's own -x axis.
    assert (result.final_pose.x - START.x) * math.cos(START.yaw) + (result.final_pose.y - START.y) * math.sin(
        START.yaw
    ) < 0.0


def test_sideways_move_slides_without_turn(ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch) -> None:
    robot = make_robot(ros_iface, monkeypatch, sectors_with("front_left", RETRO_DISTANCE_M, RETRO_BEARING_RAD))
    result = robot.move_relative(0.0, -0.12, 0.0, 30.0, precise=True)
    robot.navigate.assert_not_called()
    moving = [t for t in robot.sim.published if t != (0.0, 0.0, 0.0)]
    assert {selected_sub_polygon(*t) for t in moving} == {"right"}
    assert result.status == "succeeded"
    assert result.final_pose is not None
    travelled = math.hypot(result.final_pose.x - START.x, result.final_pose.y - START.y)
    assert travelled == pytest.approx(0.12, abs=robot.cfg.nav.goal_xy_tolerance_m)


def test_long_backing_move_needs_turn_and_is_checked(ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch) -> None:
    robot = make_robot(ros_iface, monkeypatch, sectors_with("front_left", RETRO_DISTANCE_M, RETRO_BEARING_RAD))
    with pytest.raises(RobotError, match="rotation"):
        robot.move_relative(-0.8, 0.0, 0.0, 30.0)  # beyond the straight-move limit: Nav2 turns to face it first
    robot.navigate.assert_not_called()
    assert robot.sim.published == []


def test_turn_refused_when_obstacle_in_sweep(ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch) -> None:
    robot = make_robot(ros_iface, monkeypatch, sectors_with("front_left", RETRO_DISTANCE_M, RETRO_BEARING_RAD))
    with pytest.raises(RobotError) as refused:
        robot.move_relative(0.0, 0.0, -0.6, 30.0)
    robot.navigate.assert_not_called()
    assert robot.sim.published == []
    message = str(refused.value)
    assert "0.34 m" in message and "+30 deg" in message and "front_left" in message
    required = required_clearance_m(
        robot.cfg.footprint.length_m, robot.cfg.footprint.width_m, robot.cfg.look_around.clearance_margin_m
    )
    assert f"{required:.2f} m" in message
    # The suggested translation points away from the return (behind-right of the robot) and is a straight move.
    assert "move_relative(dx=-" in message and ", dy=-" in message


def test_turn_refused_without_scan(ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch) -> None:
    robot = make_robot(ros_iface, monkeypatch, None)
    with pytest.raises(RobotError, match="lidar scan missing or stale"):
        robot.move_relative(0.0, 0.0, 0.6, 30.0)
    robot.navigate.assert_not_called()
    assert robot.sim.published == []


def test_turn_allowed_when_sweep_clear(ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch) -> None:
    robot = make_robot(ros_iface, monkeypatch, sectors_with())
    result = robot.move_relative(0.0, 0.0, -0.6, 30.0, precise=True)
    robot.map_summary.assert_called()  # the sweep was checked against the scan before the goal
    robot.navigate.assert_called_once()
    x, y, yaw, frame, timeout_s, precise = robot.navigate.call_args.args
    assert (x, y, frame, timeout_s, precise) == (pytest.approx(START.x), pytest.approx(START.y), "map", 30.0, True)
    assert math.remainder(yaw - (START.yaw - 0.6), 2 * math.pi) == pytest.approx(0.0, abs=1e-9)
    assert result.status == "succeeded"


def test_forward_move_goes_to_nav2_without_sweep_check(ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch) -> None:
    robot = make_robot(ros_iface, monkeypatch, None)  # no scan: a forward goal needs no turn, so it is not refused
    robot.move_relative(0.8, 0.1, 0.0, 30.0)
    robot.navigate.assert_called_once()
    robot.map_summary.assert_not_called()


def test_tool_description_states_backing_and_turn_refusal(tmp_path: Path) -> None:
    docs = tool_descriptions(build_mcp_server(FakeRobot(tmp_path), McpServerConfig(), TOKEN))
    text = docs["move_relative"]
    assert "0.5 m" in text and "straight" in text and "without turning first" in text
    assert "refused before moving" in text and "rotation circle" in text and "translate away" in text
