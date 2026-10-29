"""Tests for frontier_tools: list_frontiers, plan_frontier_visit, go_to_frontier and look_toward_frontier (FakeRobot)."""

import json
import math
import time
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from mcp.server.mcpserver import MCPServer
from mcp.server.mcpserver.exceptions import ToolError
from prometheus_client import REGISTRY

from mcp_server import frontier_tools
from mcp_server.config import McpServerConfig
from mcp_server.frontier_tools import map_age_s, view_clearance_m
from mcp_server.frontiers import FrontierTracker, MapGrid
from mcp_server.look_around import required_clearance_m
from mcp_server.models import BasePose, MapSummary, NavigationResult, PathPlan, SectorObstacle
from mcp_server.tools import build_mcp_server, register_tools

from .test_tools import TOKEN, FakeRobot, call, take, tool_schema_text

RES = 0.05
SIZE = 80
ROOM_ROWS = (10, 70)  # free rows [10, 70): y 0.5 .. 3.5 m
ROOM_COLS = (10, 50)  # free cols [10, 50): x 0.5 .. 2.5 m
OPENING = (30, 50)  # rows of the unknown gap in the right wall (col 50): one frontier of 20 cells (1.0 m)
FRONTIER_X = 2.525  # centre x of the opening column
ROOM_CENTRE = (1.5, 2.0)


def room(opening: tuple[int, int] | None = OPENING, stamp: float | None = None) -> MapGrid:
    """A walled room in unknown space with an unknown gap in its right wall (None: closed)."""
    data = np.full((SIZE, SIZE), -1, dtype=np.int16)
    (r0, r1), (c0, c1) = ROOM_ROWS, ROOM_COLS
    data[r0 - 1 : r1 + 1, c0 - 1 : c1 + 1] = 100
    data[r0:r1, c0:c1] = 0
    if opening is not None:
        data[opening[0] : opening[1], c1] = -1
    now = time.time() if stamp is None else stamp
    sec = int(now)
    version = (sec, int((now - sec) * 1e9), SIZE, SIZE, 0.0, float(opening[0] if opening else -1))
    return MapGrid.from_array(data, RES, 0.0, 0.0, 0.0, version)


class FrontierRobot(FakeRobot):
    """FakeRobot whose move_relative turns the pose and whose lidar summary is settable."""

    def __init__(self, tmp_path: Path) -> None:
        super().__init__(tmp_path)
        self.pose = BasePose(frame="map", x=ROOM_CENTRE[0], y=ROOM_CENTRE[1], yaw=0.0, age_s=0.05)
        self.map = room()
        self.nearest_m: float | None = 1.2
        self.rotation_status = "succeeded"
        self.interrupted_by: str | None = None
        self.map_after_navigate: MapGrid | None = None

    def navigate(
        self, x: float, y: float, yaw: float, frame: str, timeout_s: float, precise: bool = False
    ) -> NavigationResult:
        result = super().navigate(x, y, yaw, frame, timeout_s, precise)
        if self.map_after_navigate is not None:
            self.map = self.map_after_navigate
        return result

    def map_summary(self, include_png: bool, radius_m: float, png_max_px: int) -> tuple[MapSummary, bytes | None]:
        self.calls.append(("map_summary", (include_png, radius_m, png_max_px)))
        if self.nearest_m is None:
            return MapSummary(obstacles=None), None
        return MapSummary(obstacles=[SectorObstacle(sector="left", nearest_m=self.nearest_m, bearing_rad=1.57)]), None

    def move_relative(
        self, dx: float, dy: float, dyaw: float, timeout_s: float, precise: bool = False
    ) -> NavigationResult:
        self.calls.append(("move_relative", (dx, dy, dyaw, timeout_s, precise)))
        if self.rotation_status == "succeeded" and self.pose is not None:
            self.pose = self.pose.model_copy(update={"yaw": self.pose.yaw + dyaw})
        return NavigationResult(status=self.rotation_status, interrupted_by=self.interrupted_by)


@pytest.fixture
def robot(tmp_path: Path) -> FrontierRobot:
    return FrontierRobot(tmp_path)


@pytest.fixture
def server(robot: FrontierRobot) -> Any:
    return build_mcp_server(robot, McpServerConfig(), TOKEN)


def out(result: Any) -> dict[str, Any]:
    return result.structured_content


def motion_calls(robot: FakeRobot) -> list[str]:
    return [c[0] for c in robot.calls if c[0] in ("navigate", "move_relative", "drive", "spin", "camera_image")]


def sample(name: str, **labels: str) -> float:
    return REGISTRY.get_sample_value(name, labels) or 0.0


# --- list_frontiers -----------------------------------------------------------------------------------------------


def test_list_frontiers_reports_every_field(server: Any, robot: FrontierRobot) -> None:
    res = out(call(server, "list_frontiers", {}))
    assert res["epoch"] == 1 and res["map_version"] == list(robot.map.version)
    assert 0.0 <= res["map_age_s"] < 5.0
    assert res["robot_pose"] == {"x": 1.5, "y": 2.0, "yaw": 0.0}
    assert res["retired_since_last_call"] == []
    assert {"extract", "geodesic", "total"} <= set(res["compute_ms"])
    [f] = res["frontiers"]
    assert f["id"] == 1 and f["cells"] == 20 and f["length_m"] == pytest.approx(1.0)
    assert f["centroid"]["x"] == pytest.approx(FRONTIER_X) and f["centroid"]["y"] == pytest.approx(2.0, abs=0.03)
    assert f["target"]["x"] == pytest.approx(FRONTIER_X)
    assert f["reachable"] is True
    assert f["straight_distance_m"] == pytest.approx(1.025, abs=0.03)
    assert f["bearing_deg"] == pytest.approx(0.0, abs=3.0)
    pose = f["observation_pose"]
    assert pose is not None and 0.5 < pose["x"] < 2.5 and 0.5 < pose["y"] < 3.5
    assert abs(pose["yaw"]) < 0.6  # facing the gap
    assert f["path_distance_m"] is not None and f["path_distance_m"] >= 0.0
    assert f["nearest_poi"] is None and f["last_lookup_facing"] is None


def test_list_frontiers_without_a_map_is_a_tool_error(server: Any, robot: FrontierRobot) -> None:
    robot.map = None
    with pytest.raises(ToolError, match="/map"):
        call(server, "list_frontiers", {})


def test_list_frontiers_without_pose_nulls_distance_and_bearing_with_a_note(server: Any, robot: FrontierRobot) -> None:
    robot.pose = None
    res = out(call(server, "list_frontiers", {}))
    [f] = res["frontiers"]
    assert f["reachable"] is None and f["path_distance_m"] is None
    assert f["straight_distance_m"] is None and f["bearing_deg"] is None
    assert f["observation_pose"] is not None  # ranked by standoff only, still never in unknown space
    assert res["robot_pose"] is None
    assert any("robot pose unavailable" in n for n in res["notes"])


def test_list_frontiers_unreachable_robot_outside_traversable_space(server: Any, robot: FrontierRobot) -> None:
    robot.pose = BasePose(frame="map", x=3.5, y=3.5, yaw=0.0)  # in unknown space outside the room
    res = out(call(server, "list_frontiers", {}))
    [f] = res["frontiers"]
    assert f["reachable"] is False and f["path_distance_m"] is None and f["observation_pose"] is None
    assert any("unreachable" in n for n in res["notes"])
    assert out(call(server, "list_frontiers", {"reachable_only": True}))["frontiers"] == []


def test_list_frontiers_joins_the_nearest_poi(server: Any, robot: FrontierRobot) -> None:
    robot.pois.append({"id": "p1", "name": "door", "kind": "point", "x": 2.4, "y": 2.0, "status": "open"})
    [f] = out(call(server, "list_frontiers", {}))["frontiers"]
    assert f["nearest_poi"]["id"] == "p1" and f["nearest_poi"]["distance_m"] == pytest.approx(0.125, abs=0.03)


def test_list_frontiers_poi_store_down_is_null_plus_a_note(server: Any, robot: FrontierRobot) -> None:
    robot.poi_store_up = False
    res = out(call(server, "list_frontiers", {}))
    assert res["frontiers"][0]["nearest_poi"] is None
    assert any("poi_store" in n for n in res["notes"])


def test_second_call_is_a_cache_hit_with_the_same_ids(server: Any, robot: FrontierRobot) -> None:
    first = out(call(server, "list_frontiers", {}))
    second = out(call(server, "list_frontiers", {}))
    assert [f["id"] for f in first["frontiers"]] == [f["id"] for f in second["frontiers"]]
    assert second["compute_ms"]["extract"] == 0.0 and second["compute_ms"]["geodesic"] == 0.0


def test_retired_ids_are_reported_once(server: Any, robot: FrontierRobot) -> None:
    call(server, "list_frontiers", {})
    robot.map = room(None)  # gap closed: explored
    res = out(call(server, "list_frontiers", {}))
    assert res["frontiers"] == []
    assert res["retired_since_last_call"] == [{"id": 1, "reason": "explored", "successor": None}]
    assert out(call(server, "list_frontiers", {}))["retired_since_last_call"] == []


def test_list_frontiers_caps_the_count_with_a_note(server: Any, robot: FrontierRobot) -> None:
    data = room().data.copy()
    data[20:25, 50] = -1  # a second gap, 0.25 m: below min_length_m, not listed
    data[55:65, 50] = -1  # a third gap, 0.5 m
    robot.map = MapGrid.from_array(data, RES, 0.0, 0.0, 0.0, (int(time.time()), 0, 1))
    full = out(call(server, "list_frontiers", {}))["frontiers"]
    assert len(full) == 2
    capped = out(call(server, "list_frontiers", {"max_count": 1}))
    assert len(capped["frontiers"]) == 1 and any("1 more" in n for n in capped["notes"])


def test_list_frontiers_sets_the_metrics(server: Any, robot: FrontierRobot) -> None:
    extract = sample("mcp_frontier_compute_seconds_count", stage="extract")
    geodesic = sample("mcp_frontier_compute_seconds_count", stage="geodesic")
    call(server, "list_frontiers", {})
    assert sample("mcp_frontier_compute_seconds_count", stage="extract") == extract + 1
    assert sample("mcp_frontier_compute_seconds_count", stage="geodesic") == geodesic + 1
    assert sample("mcp_frontier_count") == 1.0
    call(server, "list_frontiers", {})  # cache hit: nothing observed again
    assert sample("mcp_frontier_compute_seconds_count", stage="extract") == extract + 1


def test_map_age_from_the_version_stamp_or_none() -> None:
    assert map_age_s(room(stamp=time.time() - 3.0)) == pytest.approx(3.0, abs=0.5)
    assert map_age_s(MapGrid.from_array(np.zeros((2, 2)), RES, 0.0, 0.0, 0.0, ("opaque",))) is None


def test_view_clearance_uses_the_nav_footprint_or_the_static_half_diagonal(robot: FrontierRobot) -> None:
    cfg = McpServerConfig()
    margin = cfg.frontiers.view_clearance_margin_m
    static = required_clearance_m(cfg.footprint.length_m, cfg.footprint.width_m, margin)
    assert view_clearance_m(cfg, None) == pytest.approx(static)

    class Footprint:
        def circumscribed_radius(self) -> float:
            return 0.5

    assert view_clearance_m(cfg, Footprint()) == pytest.approx(0.5 + margin)


# --- plan_frontier_visit ------------------------------------------------------------------------------------------


def test_plan_frontier_visit_checks_the_pose_with_nav2(server: Any, robot: FrontierRobot) -> None:
    robot.path_plan = PathPlan(
        status="succeeded", length_m=0.9, planning_time_s=0.2, goal=BasePose(frame="map", x=0, y=0, yaw=0)
    )
    robot.pose = robot.pose.model_copy(update={"yaw": math.pi / 2})
    res = out(call(server, "plan_frontier_visit", {"id": 1}))
    pose = res["observation_pose"]
    assert robot.path_requests == [(pose["x"], pose["y"], pose["yaw"], 5.0)]
    assert res["nav2_check"]["status"] == "succeeded" and res["nav2_check"]["length_m"] == 0.9
    assert res["reachable"] is True and res["path_distance_m"] is not None
    assert res["rotation_deg"] == pytest.approx(-90.0, abs=3.0)
    assert res["rotation_clearance"]["ok"] is True and res["rotation_clearance"]["refusal"] is None
    assert motion_calls(robot) == []


def test_plan_frontier_visit_skips_nav2_while_navigating(server: Any, robot: FrontierRobot) -> None:
    robot.nav_busy = True
    res = out(call(server, "plan_frontier_visit", {"id": 1}))
    assert res["nav2_check"] is None and robot.path_requests == []
    assert "navigation active: Nav2 check skipped (would preempt the planner)" in res["notes"]


def test_plan_frontier_visit_planner_down_is_null_plus_a_note(server: Any, robot: FrontierRobot) -> None:
    robot.path_plan = None
    res = out(call(server, "plan_frontier_visit", {"id": 1}))
    assert res["nav2_check"] is None and any("Nav2 check unavailable" in n for n in res["notes"])


def test_plan_frontier_visit_needs_no_lease(server: Any, robot: FrontierRobot) -> None:
    call(server, "plan_frontier_visit", {"id": 1})  # no take_control: sensor tool


def test_plan_frontier_visit_reports_a_blocked_rotation(server: Any, robot: FrontierRobot) -> None:
    robot.nearest_m = 0.1
    res = out(call(server, "plan_frontier_visit", {"id": 1}))
    assert res["rotation_clearance"]["ok"] is False and "too close" in res["rotation_clearance"]["refusal"]


def test_retired_id_names_the_successor(server: Any, robot: FrontierRobot) -> None:
    call(server, "list_frontiers", {})
    robot.map = room((50, 66))  # the gap moved 0.9 m: id 1 retired, the new gap is id 2
    with pytest.raises(ToolError, match=r"frontier 1 is gone \(split into 2\); nearest current frontier: 2"):
        call(server, "plan_frontier_visit", {"id": 1})


def test_explored_id_says_so(server: Any, robot: FrontierRobot) -> None:
    call(server, "list_frontiers", {})
    robot.map = room(None)
    with pytest.raises(ToolError, match=r"frontier 1 is gone \(explored\); no current frontier replaces it"):
        call(server, "plan_frontier_visit", {"id": 1})


def test_unknown_id_lists_the_current_ids(server: Any, robot: FrontierRobot) -> None:
    with pytest.raises(ToolError, match=r"frontier 9 is unknown.*current ids: \[1\]"):
        call(server, "plan_frontier_visit", {"id": 9})


# --- go_to_frontier -----------------------------------------------------------------------------------------------


def test_go_to_frontier_needs_the_base_lease(server: Any, robot: FrontierRobot) -> None:
    with pytest.raises(ToolError, match="base"):
        call(server, "go_to_frontier", {"id": 1})
    assert motion_calls(robot) == []


def test_go_to_frontier_navigates_to_the_observation_pose(server: Any, robot: FrontierRobot) -> None:
    take(server, "base")
    listed = out(call(server, "list_frontiers", {}))["frontiers"][0]["observation_pose"]
    res = out(call(server, "go_to_frontier", {"id": 1}))
    [(x, y, yaw, frame, timeout, precise)] = [c[1] for c in robot.calls if c[0] == "navigate"]
    assert (x, y, yaw) == pytest.approx((listed["x"], listed["y"], listed["yaw"]), abs=1e-3)
    assert frame == "map" and timeout == McpServerConfig().timeouts.nav_default_timeout_s and precise is False
    assert res["status"] == "succeeded" and res["frontier_id"] == 1
    assert res["observation_pose"] == listed
    cell = (int(y / RES), int(x / RES))
    assert robot.map.data[cell] == 0  # the goal is known free, never unknown
    assert res["frontier_still_present"] is None  # the map did not change during the move
    assert any("map not updated" in n for n in res["notes"])


def test_go_to_frontier_reports_whether_the_frontier_is_still_there(server: Any, robot: FrontierRobot) -> None:
    take(server, "base")

    robot.map_after_navigate = room(None)  # the gap gets explored while driving there
    res = out(call(server, "go_to_frontier", {"id": 1}))
    assert res["frontier_still_present"] is False


def test_go_to_frontier_timeout_bounded(server: Any, robot: FrontierRobot) -> None:
    take(server, "base")
    with pytest.raises(ToolError, match="timeout_s must be <="):
        call(server, "go_to_frontier", {"id": 1, "timeout_s": 1e6})
    call(server, "go_to_frontier", {"id": 1, "timeout_s": 12.0})
    assert [c[1][4] for c in robot.calls if c[0] == "navigate"] == [12.0]


def test_go_to_frontier_refuses_without_pose(server: Any, robot: FrontierRobot) -> None:
    take(server, "base")
    robot.pose = None
    with pytest.raises(ToolError, match="robot pose unavailable"):
        call(server, "go_to_frontier", {"id": 1})
    assert motion_calls(robot) == []


def test_go_to_frontier_without_reachable_pose_does_not_move(server: Any, robot: FrontierRobot) -> None:
    take(server, "base")
    robot.pose = BasePose(frame="map", x=3.5, y=3.5, yaw=0.0)
    with pytest.raises(ToolError, match="no reachable observation pose"):
        call(server, "go_to_frontier", {"id": 1})
    assert motion_calls(robot) == []


def test_go_to_frontier_retired_id_is_an_error(server: Any, robot: FrontierRobot) -> None:
    take(server, "base")
    call(server, "list_frontiers", {})
    robot.map = room(None)
    with pytest.raises(ToolError, match="frontier 1 is gone"):
        call(server, "go_to_frontier", {"id": 1})
    assert motion_calls(robot) == []


# --- look_toward_frontier -----------------------------------------------------------------------------------------


@pytest.mark.parametrize(("yaw", "sign"), [(math.pi / 2, -1.0), (-math.pi / 2, 1.0)])
def test_look_toward_frontier_rotates_with_the_right_sign(
    server: Any, robot: FrontierRobot, yaw: float, sign: float
) -> None:
    take(server, "base")
    robot.pose = robot.pose.model_copy(update={"yaw": yaw})
    res = call(server, "look_toward_frontier", {"id": 1})
    [(dx, dy, dyaw, timeout, precise)] = [c[1] for c in robot.calls if c[0] == "move_relative"]
    assert (dx, dy) == (0.0, 0.0) and precise is True and timeout == McpServerConfig().frontiers.look_timeout_s
    assert dyaw == pytest.approx(sign * math.pi / 2, abs=0.05)
    data = out(res)
    assert data["status"] == "succeeded" and data["rotation_deg"] == pytest.approx(sign * 90.0, abs=3.0)
    assert abs(data["heading_error_deg"]) < 3.0
    assert data["image_captured"] is True and data["camera"] == "front"
    assert [c.type for c in res.content][:2] == ["image", "text"]


def test_look_toward_frontier_records_the_capture_as_a_lookup(server: Any, robot: FrontierRobot) -> None:
    take(server, "base")
    robot.pose = robot.pose.model_copy(update={"yaw": math.pi / 2})
    data = out(call(server, "look_toward_frontier", {"id": 1}))
    assert data["lookup_id"]
    lookups = out(call(server, "list_lookups", {}))
    [record] = lookups["lookups"]
    assert record["tool"] == "look_toward_frontier" and record["id"] == data["lookup_id"]
    [f] = out(call(server, "list_frontiers", {}))["frontiers"]
    assert f["last_lookup_facing"]["lookup_id"] == data["lookup_id"]


def test_look_toward_frontier_skips_the_rotation_within_the_yaw_tolerance(server: Any, robot: FrontierRobot) -> None:
    take(server, "base")
    data = out(call(server, "look_toward_frontier", {"id": 1}))
    assert [c for c in robot.calls if c[0] == "move_relative"] == []
    assert data["status"] == "skipped" and data["rotation_deg"] == pytest.approx(0.0, abs=3.0)
    assert data["image_captured"] is True


def test_look_toward_frontier_refused_when_the_rotation_circle_is_blocked(server: Any, robot: FrontierRobot) -> None:
    take(server, "base")
    robot.pose = robot.pose.model_copy(update={"yaw": math.pi / 2})
    robot.nearest_m = 0.1
    with pytest.raises(ToolError, match="look_toward_frontier refused: obstacle too close"):
        call(server, "look_toward_frontier", {"id": 1})
    assert motion_calls(robot) == []


def test_look_toward_frontier_refused_without_a_scan(server: Any, robot: FrontierRobot) -> None:
    take(server, "base")
    robot.pose = robot.pose.model_copy(update={"yaw": math.pi / 2})
    robot.nearest_m = None
    with pytest.raises(ToolError, match="lidar scan missing"):
        call(server, "look_toward_frontier", {"id": 1})
    assert motion_calls(robot) == []


@pytest.mark.parametrize(("status", "event"), [("aborted", None), ("interrupted", "overheat"), ("canceled", None)])
def test_no_capture_after_a_failed_or_interrupted_rotation(
    server: Any, robot: FrontierRobot, status: str, event: str | None
) -> None:
    take(server, "base")
    robot.pose = robot.pose.model_copy(update={"yaw": math.pi / 2})
    robot.rotation_status, robot.interrupted_by = status, event
    res = call(server, "look_toward_frontier", {"id": 1})
    data = out(res)
    assert data["status"] == status and data["interrupted_by"] == event
    assert data["image_captured"] is False and data["lookup_id"] is None
    assert f"rotation {status}: no capture" in data["notes"]
    assert [c for c in robot.calls if c[0] == "camera_image"] == []
    assert [c.type for c in res.content][0] == "text"


def test_look_toward_frontier_never_moves_the_arm(server: Any, robot: FrontierRobot) -> None:
    take(server, "base")
    robot.pose = robot.pose.model_copy(update={"yaw": math.pi / 2})
    before = len(robot.arm_backend.commands)
    data = out(call(server, "look_toward_frontier", {"id": 1, "camera": "front"}))
    assert len(robot.arm_backend.commands) == before
    assert data["camera"] == "front"


def test_look_toward_frontier_uses_front(server: Any, robot: FrontierRobot) -> None:
    text = tool_schema_text(server, "look_toward_frontier")
    assert '"enum": ["front"]' in text or '"const": "front"' in text
    assert "D435" in text and "gripper" not in text
    assert not hasattr(frontier_tools, "GRIPPER_NOTE")
    take(server, "base")
    robot.pose = robot.pose.model_copy(update={"yaw": math.pi / 2})
    with pytest.raises(ToolError):
        call(server, "look_toward_frontier", {"id": 1, "camera": "gripper"})
    assert motion_calls(robot) == []
    data = out(call(server, "look_toward_frontier", {"id": 1}))
    assert data["camera"] == "front" and data["image_captured"] is True
    assert [c[1][0] for c in robot.calls if c[0] == "camera_image"] == ["front"]
    assert not any("gripper" in n for n in data["notes"])


def test_look_toward_frontier_needs_the_base_lease(server: Any, robot: FrontierRobot) -> None:
    with pytest.raises(ToolError, match="base"):
        call(server, "look_toward_frontier", {"id": 1})
    assert motion_calls(robot) == []


def test_look_toward_frontier_refuses_without_pose(server: Any, robot: FrontierRobot) -> None:
    take(server, "base")
    robot.pose = None
    with pytest.raises(ToolError, match="robot pose unavailable"):
        call(server, "look_toward_frontier", {"id": 1})
    assert motion_calls(robot) == []


def test_look_toward_frontier_result_text_is_json(server: Any, robot: FrontierRobot) -> None:
    take(server, "base")
    res = call(server, "look_toward_frontier", {"id": 1})
    text = next(c.text for c in res.content if c.type == "text")
    assert json.loads(text)["frontier_id"] == 1


def test_register_tools_creates_the_tracker(robot: FrontierRobot) -> None:
    seen: list[Any] = []
    register_tools(MCPServer("t"), robot, McpServerConfig(), extra_modules=(lambda ctx: seen.append(ctx.frontiers),))
    assert isinstance(seen[0], FrontierTracker)


def test_look_toward_frontier_camera_failure_keeps_the_rotation_result(server: Any, robot: FrontierRobot) -> None:
    take(server, "base")
    robot.camera_error = "no frame"
    data = out(call(server, "look_toward_frontier", {"id": 1}))
    assert data["status"] == "skipped" and data["image_captured"] is False and data["lookup_id"] is None
    assert any("camera capture failed: no frame" in n for n in data["notes"])
