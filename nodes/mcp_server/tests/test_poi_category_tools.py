"""POI category arguments of add_poi, update_poi, remember_object and list_pois."""

import json
from pathlib import Path
from typing import Any

import pytest
from mcp.server.mcpserver.exceptions import ToolError

from mcp_server.config import McpServerConfig
from mcp_server.tools import build_mcp_server

from .test_tools import TOKEN, FakeRobot, call

SQUARE = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]


def payload(res: Any) -> dict[str, Any]:
    for block in res.content:
        if block.type == "text":
            data = json.loads(block.text)
            if "robot_events_since_last_call" not in data:
                return data
    raise AssertionError("no payload text block")


@pytest.fixture
def robot(tmp_path: Path) -> FakeRobot:
    return FakeRobot(tmp_path)


@pytest.fixture
def server(robot: FakeRobot) -> Any:
    return build_mcp_server(robot, McpServerConfig(), TOKEN)


def add(server: Any, **kwargs: Any) -> dict[str, Any]:
    return payload(call(server, "add_poi", {"name": "n", **kwargs}))["poi"]


@pytest.mark.parametrize(
    ("args", "note", "zone"),
    [
        ({"kind": "point", "x": 1.0, "y": 2.0, "category": "target"}, "#target", "none"),
        ({"kind": "point", "x": 1.0, "y": 2.0, "category": "detail", "note": "text"}, "#detail\ntext", "none"),
        ({"kind": "area", "polygon": SQUARE, "category": "keepout", "zone": "keepout"}, "#keepout", "keepout"),
        (
            {"kind": "area", "polygon": SQUARE, "category": "slow", "zone": "slow", "speed_limit_pct": 40},
            "#slow",
            "slow",
        ),
        (
            {"kind": "point", "x": 1.0, "y": 2.0, "category": "problem", "problem_type": "window"},
            "#problem/window",
            "none",
        ),
        (
            {"kind": "point", "x": 1.0, "y": 2.0, "category": "target", "survey": True},
            "#target #survey",
            "none",
        ),
    ],
)
def test_add_poi_writes_category_tags(server: Any, robot: FakeRobot, args: dict, note: str, zone: str) -> None:
    poi = add(server, **args)
    assert poi["note"] == note
    assert poi["zone"] == zone


@pytest.mark.parametrize(
    "args",
    [
        {"kind": "point", "x": 1.0, "y": 2.0, "category": "keepout"},
        {"kind": "area", "polygon": SQUARE, "category": "keepout"},
        {"kind": "area", "polygon": SQUARE, "category": "slow", "zone": "slow"},
        {"kind": "area", "polygon": SQUARE, "category": "slow", "speed_limit_pct": 40},
        {"kind": "point", "x": 1.0, "y": 2.0, "category": "problem"},
        {"kind": "point", "x": 1.0, "y": 2.0, "category": "target", "problem_type": "window"},
        {"kind": "point", "x": 1.0, "y": 2.0, "problem_type": "window"},
    ],
)
def test_invalid_combinations_are_rejected_before_any_request(server: Any, robot: FakeRobot, args: dict) -> None:
    with pytest.raises(ToolError):
        call(server, "add_poi", {"name": "n", **args})
    assert robot.poi_commands == []


def test_survey_without_category_adds_only_the_survey_tag(server: Any) -> None:
    assert add(server, kind="point", x=1.0, y=2.0, survey=True)["note"] == "#survey"


def test_update_poi_changing_category_keeps_free_text_and_survey(server: Any, robot: FakeRobot) -> None:
    poi = add(server, kind="point", x=1.0, y=2.0, category="target", survey=True, note="by the door")
    assert poi["note"] == "#target #survey\nby the door"
    call(server, "update_poi", {"id": poi["id"], "category": "problem", "problem_type": "mirror"})
    assert robot.pois[0]["note"] == "#problem/mirror #survey\nby the door"


def test_update_poi_validates_against_the_stored_values(server: Any, robot: FakeRobot) -> None:
    poi = add(server, kind="point", x=1.0, y=2.0)
    robot.poi_commands.clear()
    with pytest.raises(ToolError):
        call(server, "update_poi", {"id": poi["id"], "category": "keepout", "zone": "keepout"})
    assert robot.poi_commands == []
    area = add(server, kind="area", polygon=SQUARE)
    with pytest.raises(ToolError):
        call(server, "update_poi", {"id": area["id"], "category": "slow"})
    call(server, "update_poi", {"id": area["id"], "category": "slow", "zone": "slow", "speed_limit_pct": 30})
    assert robot.pois[1]["note"] == "#slow"
    assert robot.pois[1]["zone"] == "slow"


def test_update_poi_survey_true_adds_the_tag(server: Any, robot: FakeRobot) -> None:
    poi = add(server, kind="point", x=1.0, y=2.0, note="free")
    call(server, "update_poi", {"id": poi["id"], "survey": True})
    assert robot.pois[0]["note"] == "#survey\nfree"


def test_remember_object_survey_tag(server: Any, robot: FakeRobot) -> None:
    call(server, "remember_object", {"label": "cup", "x": 1.0, "y": 1.0, "note": "on table", "survey": True})
    assert robot.pois[0]["note"] == "#target #survey\non table"


def test_list_pois_returns_category_fields(server: Any, robot: FakeRobot) -> None:
    add(server, kind="point", x=1.0, y=2.0, category="problem", problem_type="tight", survey=True)
    add(server, kind="point", x=3.0, y=2.0)
    views = payload(call(server, "list_pois", {}))["pois"]
    assert (views[0]["category"], views[0]["problem_type"], views[0]["survey"]) == ("problem", "tight", True)
    assert (views[1]["category"], views[1]["problem_type"], views[1]["survey"]) == (None, None, False)
