"""Tests for keep-out aware frontiers: person-drawn keepout areas hide ghost frontiers (mirror mitigation)."""

from pathlib import Path
from typing import Any

import numpy as np
import pytest
from mcp.server.mcpserver.exceptions import ToolError

from mcp_server.config import ExplorationSettings, McpServerConfig
from mcp_server.frontiers import FrontierTracker
from mcp_server.keepouts import KeepOut, parse_keepouts, points_in_polygon
from mcp_server.perception_tools import frontier_markers
from mcp_server.tools import build_mcp_server
from mcp_server.topdown import COLOR_FRONTIER, COLOR_FRONTIER_KEEPOUT, KEEPOUT_LEGEND, TopdownInputs

from .test_frontier_tools import FrontierRobot, motion_calls, out
from .test_tools import TOKEN, call, take
from .test_topdown import CENTRE, SCALE, has_color, render

SPOT = {"id": "s1", "name": "survey spot", "kind": "point", "x": 1.5, "y": 2.0, "status": "open",
        "created_by": "agent", "note": "#survey #survey-spot\nfirst"}  # fmt: skip


def keepout_poi(poi_id: str, x0: float, y0: float, x1: float, y1: float, **extra: Any) -> dict[str, Any]:
    """An open keepout area POI covering the rectangle."""
    return {
        "id": poi_id, "name": "mirror", "kind": "area", "zone": "keepout", "status": "open",
        "x": (x0 + x1) / 2, "y": (y0 + y1) / 2, "polygon": [[x0, y0], [x1, y0], [x1, y1], [x0, y1]], **extra,
    }  # fmt: skip


AROUND_TARGET = (2.4, 1.3, 2.8, 2.7)  # covers the frontier column (x 2.525, y 1.5..2.5), no observation pose
OVER_POSES = (1.2, 0.3, 2.5, 3.7)  # covers every observation pose, not the frontier column


@pytest.fixture
def robot(tmp_path: Path) -> FrontierRobot:
    return FrontierRobot(tmp_path)


def make_server(robot: FrontierRobot) -> Any:
    config = McpServerConfig(exploration=ExplorationSettings(survey_radius_m=5.0))
    return build_mcp_server(robot, config, TOKEN)


def test_parse_keepouts_takes_only_open_keepout_areas() -> None:
    good = keepout_poi("k1", 0, 0, 1, 1)
    pois = [
        good,
        {**good, "id": "done", "status": "done"},
        {**good, "id": "slow", "zone": "slow", "speed_limit_pct": 50},
        {**good, "id": "point", "kind": "point"},
        {**good, "id": "line", "polygon": [[0, 0], [1, 1]]},
        {**good, "id": "nan", "polygon": [[0, 0], [1, "x"], [1, 1]]},
    ]
    assert parse_keepouts(pois) == [KeepOut(id="k1", polygon=((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)))]


def test_points_in_polygon_matches_the_square() -> None:
    square = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    inside = points_in_polygon(np.array([0.5, 1.5, 0.5]), np.array([0.5, 0.5, -0.1]), square)
    assert inside.tolist() == [True, False, False]


def test_frontier_inside_keepout_excluded(robot: FrontierRobot) -> None:
    server = make_server(robot)
    assert len(out(call(server, "list_frontiers", {}))["frontiers"]) == 1
    robot.pois.append(keepout_poi("mirror1", *AROUND_TARGET))
    listed = out(call(server, "list_frontiers", {}))
    assert listed["frontiers"] == []
    assert listed["excluded_keepout"] == [{"id": 1, "zone_poi_id": "mirror1"}]
    robot.pois.append(dict(SPOT))
    status = out(call(server, "get_exploration_status", {}))
    assert status["frontiers"] == [] and status["reachable_frontiers"] == 0 and status["unreachable_frontiers"] == 0
    assert status["excluded_keepout"] == [{"id": 1, "zone_poi_id": "mirror1"}]


def test_no_zones_means_no_exclusion(robot: FrontierRobot) -> None:
    robot.pois.append({**keepout_poi("k", *AROUND_TARGET), "status": "done"})
    listed = out(call(make_server(robot), "list_frontiers", {}))
    assert len(listed["frontiers"]) == 1 and listed["excluded_keepout"] == []


def test_observation_pose_inside_keepout_skipped(robot: FrontierRobot) -> None:
    server = make_server(robot)
    before = out(call(server, "list_frontiers", {}))["frontiers"][0]
    assert before["observation_pose"] is not None
    robot.pois.append(keepout_poi("k2", *OVER_POSES))
    [after] = out(call(server, "list_frontiers", {}))["frontiers"]
    assert after["observation_pose"] is None and after["reachable"] is False
    # a keepout that only covers the best pose moves the pose out of it
    robot.pois[:] = [
        keepout_poi("k3", before["observation_pose"]["x"] - 0.1, 0.3, before["observation_pose"]["x"] + 0.1, 3.7)
    ]
    [moved] = out(call(server, "list_frontiers", {}))["frontiers"]
    pose = moved["observation_pose"]
    assert pose is not None and abs(pose["x"] - before["observation_pose"]["x"]) > 0.1 - 1e-6


def test_go_to_frontier_refused_in_keepout(robot: FrontierRobot) -> None:
    server = make_server(robot)
    take(server, "base")
    robot.pois.append(keepout_poi("mirror1", *AROUND_TARGET))
    with pytest.raises(ToolError, match="keep-out.*mirror1"):
        call(server, "go_to_frontier", {"id": 1})
    assert motion_calls(robot) == []


def test_complete_ignores_keepout_frontiers(robot: FrontierRobot) -> None:
    server = make_server(robot)
    robot.pois.append(dict(SPOT))
    assert out(call(server, "get_exploration_status", {}))["complete"] is False
    robot.pois.append(keepout_poi("mirror1", *AROUND_TARGET))
    status = out(call(server, "get_exploration_status", {}))
    assert status["complete"] is True and status["reasons"] == []


def test_stale_poi_list_keeps_last_zones_and_says_so(robot: FrontierRobot) -> None:
    server = make_server(robot)
    robot.pois.append(keepout_poi("mirror1", *AROUND_TARGET))
    assert out(call(server, "list_frontiers", {}))["frontiers"] == []
    robot.pois.clear()  # the zone vanishes from a list that is now older than the limit
    robot.poi_age_s = 1e6
    listed = out(call(server, "list_frontiers", {}))
    assert listed["frontiers"] == [] and listed["excluded_keepout"] == [{"id": 1, "zone_poi_id": "mirror1"}]
    assert any("poi list" in n and "stale" in n for n in listed["notes"])


def test_poi_store_down_keeps_last_zones(robot: FrontierRobot) -> None:
    server = make_server(robot)
    robot.pois.append(keepout_poi("mirror1", *AROUND_TARGET))
    call(server, "list_frontiers", {})
    robot.poi_store_up = False
    listed = out(call(server, "list_frontiers", {}))
    assert listed["frontiers"] == [] and any("keep-out" in n for n in listed["notes"])


# --- topdown ------------------------------------------------------------------------------------------------------


def test_topdown_draws_excluded_frontiers_grey_with_a_legend_row() -> None:
    inputs = TopdownInputs(pose=(0.0, 0.0, 0.0))
    marker = {"id": 4, "target_xy": (1.0, 0.0), "cells_xy": np.array([(1.0, 0.0)]), "excluded": True}
    result = render(inputs, ["frontiers"], frontiers=[marker])
    v = CENTRE - 1.0 / SCALE
    assert has_color(result.image, COLOR_FRONTIER_KEEPOUT, CENTRE, v) and not has_color(
        result.image, COLOR_FRONTIER, CENTRE, v
    )
    assert KEEPOUT_LEGEND[0] == "in keep-out"
    legend_row_y = 4 + 14 + 6  # second legend row swatch
    assert has_color(result.image, COLOR_FRONTIER_KEEPOUT, 7, legend_row_y, tol_px=1)
    plain = render(inputs, ["frontiers"], frontiers=[{**marker, "excluded": False}])
    assert has_color(plain.image, COLOR_FRONTIER, CENTRE, v) and not has_color(
        plain.image, COLOR_FRONTIER_KEEPOUT, 7, legend_row_y, tol_px=1
    )


def test_frontier_markers_flag_keepout_frontiers(robot: FrontierRobot) -> None:
    tracker = FrontierTracker(McpServerConfig().frontiers)
    [plain] = frontier_markers(tracker, robot)
    assert plain["excluded"] is False
    robot.pois.append(keepout_poi("mirror1", *AROUND_TARGET))
    [marker] = frontier_markers(tracker, robot)
    assert marker["excluded"] is True
