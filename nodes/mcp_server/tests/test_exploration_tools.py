"""Tests for get_exploration_status and mark_survey_spot (frontier_tools; FakeRobot, fake guard, POI list)."""

from pathlib import Path
from typing import Any

import pytest
from mcp.server.mcpserver.exceptions import ToolError
from ros2_common.battery import BatteryConfig, BatteryGuard

from mcp_server.config import ExplorationSettings, McpServerConfig
from mcp_server.tools import ALWAYS_ALLOWED_TOOLS, build_mcp_server

from .test_frontier_tools import FrontierRobot, room
from .test_tools import TOKEN, call

STOP_MARGIN = 0.3
SPOT = {"id": "s1", "name": "survey spot", "kind": "point", "x": 1.5, "y": 2.0, "status": "open",
        "created_by": "agent", "note": "#survey #survey-spot\nfirst"}  # fmt: skip


@pytest.fixture
def robot(tmp_path: Path) -> FrontierRobot:
    return FrontierRobot(tmp_path)


def make_server(robot: FrontierRobot, guard: BatteryGuard | None = None) -> Any:
    config = McpServerConfig(exploration=ExplorationSettings(survey_radius_m=5.0, stop_margin_cell_v=STOP_MARGIN))
    return build_mcp_server(robot, config, TOKEN, guard)


def make_guard(voltage: float | None) -> BatteryGuard:
    guard = BatteryGuard.from_config(BatteryConfig())
    if voltage is not None:
        guard.update(voltage)
    return guard


def status(server: Any) -> dict[str, Any]:
    return call(server, "get_exploration_status", {}).structured_content


def test_tools_are_registered_and_always_allowed() -> None:
    assert {"get_exploration_status", "mark_survey_spot"} <= ALWAYS_ALLOWED_TOOLS


def test_complete_only_with_no_reachable_frontier_and_full_coverage(robot: FrontierRobot) -> None:
    robot.map = room(None)  # closed room
    robot.pois.append(dict(SPOT))
    res = status(make_server(robot))
    assert res["complete"] is True and res["reasons"] == []
    assert res["reachable_frontiers"] == 0 and res["frontiers"] == []
    assert res["coverage"]["fraction"] == pytest.approx(1.0) and res["coverage"]["unsurveyed_areas"] == []
    assert len(res["survey_spots"]) == 1


def test_open_frontier_blocks_completion_and_is_named(robot: FrontierRobot) -> None:
    robot.pois.append(dict(SPOT))
    res = status(make_server(robot))
    assert res["complete"] is False
    [f] = res["frontiers"]
    assert f["id"] == 1 and f["path_distance_m"] is not None and f["looked"] is False
    assert res["reachable_frontiers"] == 1 and res["unlooked_frontiers"] == 1 and res["unreachable_frontiers"] == 0
    assert any("frontier" in r and "1" in r for r in res["reasons"])


def test_looked_frontier_is_counted(robot: FrontierRobot) -> None:
    robot.pois.append(dict(SPOT))
    server = make_server(robot)
    ctx_frontier = call(server, "list_frontiers", {}).structured_content["frontiers"][0]
    assert ctx_frontier["last_lookup_facing"] is None
    from mcp_server import frontier_tools

    original = frontier_tools.FrontierContext.describe

    def describe(self: Any, frontiers: list[Any]) -> list[Any]:
        return [v.model_copy(update={"last_lookup_facing": {"id": "l1"}}) for v in original(self, frontiers)]

    frontier_tools.FrontierContext.describe = describe  # type: ignore[method-assign]
    try:
        res = status(server)
    finally:
        frontier_tools.FrontierContext.describe = original  # type: ignore[method-assign]
    assert res["frontiers"][0]["looked"] is True and res["unlooked_frontiers"] == 0


def test_unsurveyed_area_blocks_completion_and_is_named(robot: FrontierRobot) -> None:
    robot.map = room(None)
    res = status(make_server(robot))  # no survey spot yet
    assert res["complete"] is False and res["survey_spots"] == []
    assert res["coverage"]["fraction"] == 0.0 and len(res["coverage"]["unsurveyed_areas"]) == 1
    assert any("unsurveyed" in r for r in res["reasons"])


def test_only_survey_spot_pois_count(robot: FrontierRobot) -> None:
    robot.map = room(None)
    robot.pois += [
        {**SPOT, "id": "a", "note": "#survey"},  # survey sighting, not a spot
        {**SPOT, "id": "b", "created_by": "user"},
        {**SPOT, "id": "c", "status": "done"},
        {**SPOT, "id": "d", "note": "survey-spot in words"},
    ]
    res = status(make_server(robot))
    assert res["survey_spots"] == [] and res["complete"] is False
    robot.pois.append(dict(SPOT))
    res = status(make_server(robot))
    assert [s["x"] for s in res["survey_spots"]] == [1.5] and res["complete"] is True


def test_battery_limit_flips_below_stop_v(robot: FrontierRobot) -> None:
    robot.map = room(None)
    guard = make_guard(None)
    cfg = BatteryConfig()
    stop_v = cfg.cells * (cfg.cutoff_cell_v + STOP_MARGIN)
    server = make_server(robot, guard)
    guard.update(stop_v + 0.5)
    b = status(server)["battery"]
    assert b["stop_v"] == pytest.approx(stop_v) and b["pack_v"] == pytest.approx(stop_v + 0.5)
    assert b["limit_reached"] is False
    guard.update(stop_v - 0.1)
    assert status(server)["battery"]["limit_reached"] is True


def test_battery_null_with_note_without_guard_or_reading(robot: FrontierRobot) -> None:
    robot.map = room(None)
    res = status(make_server(robot))
    assert res["battery"] == {"pack_v": None, "stop_v": None, "limit_reached": None}
    assert any("battery" in n for n in res["notes"])
    res = status(make_server(robot, make_guard(None)))
    assert res["battery"]["pack_v"] is None and any("battery" in n for n in res["notes"])


def test_null_with_note_without_map(robot: FrontierRobot) -> None:
    robot.map = None
    res = status(make_server(robot))
    assert res["complete"] is None and res["coverage"] is None and res["frontiers"] is None
    assert any("/map" in n for n in res["notes"])


def test_null_with_note_without_pose(robot: FrontierRobot) -> None:
    robot.pose = None
    res = status(make_server(robot))
    assert res["complete"] is None and res["coverage"] is None and res["frontiers"] is None
    assert any("pose" in n for n in res["notes"])


def test_status_never_moves(robot: FrontierRobot) -> None:
    status(make_server(robot))
    assert not [c for c in robot.calls if c[0] in ("navigate", "move_relative", "drive", "spin")]


# --- mark_survey_spot ---------------------------------------------------------------------------------------------


def test_mark_survey_spot_at_robot_pose(robot: FrontierRobot) -> None:
    res = call(make_server(robot), "mark_survey_spot", {"note": "kitchen door"}).structured_content
    assert res["ok"] is True
    [(op, poi)] = robot.poi_commands
    assert op == "add" and poi["name"] == "survey spot" and poi["kind"] == "point" and poi["created_by"] == "agent"
    assert (poi["x"], poi["y"]) == (1.5, 2.0)
    assert poi["note"] == "#survey #survey-spot\nkitchen door"
    assert res["poi"]["id"] == robot.pois[0]["id"]


def test_mark_survey_spot_without_note_has_only_tags(robot: FrontierRobot) -> None:
    call(make_server(robot), "mark_survey_spot", {})
    assert robot.poi_commands[0][1]["note"] == "#survey #survey-spot"


def test_mark_survey_spot_at_given_xy(robot: FrontierRobot) -> None:
    robot.pose = None
    call(make_server(robot), "mark_survey_spot", {"x": 0.7, "y": 1.1})
    poi = robot.poi_commands[0][1]
    assert (poi["x"], poi["y"]) == (0.7, 1.1)


def test_mark_survey_spot_errors_without_pose_and_coordinates(robot: FrontierRobot) -> None:
    robot.pose = None
    with pytest.raises(ToolError, match="pose"):
        call(make_server(robot), "mark_survey_spot", {})
    assert robot.poi_commands == []


def test_mark_survey_spot_needs_both_coordinates(robot: FrontierRobot) -> None:
    with pytest.raises(ToolError, match="both"):
        call(make_server(robot), "mark_survey_spot", {"x": 1.0})
