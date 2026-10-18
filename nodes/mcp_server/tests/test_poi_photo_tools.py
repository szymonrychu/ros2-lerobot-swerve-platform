"""Photo POIs in the agent's POI tools: clear_pois counts, list_pois and describe_pois skip photos."""

import json
from pathlib import Path
from typing import Any

import pytest

from mcp_server.config import McpServerConfig
from mcp_server.poi_client import describe_pois
from mcp_server.tools import build_mcp_server

from .test_tools import TOKEN, FakeRobot, call


def payload(res: Any) -> dict[str, Any]:
    for block in res.content:
        if block.type == "text":
            data = json.loads(block.text)
            if "robot_events_since_last_call" not in data:
                return data
    raise AssertionError("no payload text block")


@pytest.fixture
def robot(tmp_path: Path) -> FakeRobot:
    r = FakeRobot(tmp_path)
    r.pois = [
        {"id": "a", "name": "a", "created_by": "agent", "kind": "point", "status": "open", "x": 1.0, "y": 2.0},
        {"id": "p1", "name": "p1", "created_by": "agent", "kind": "photo", "status": "open", "x": 0.5, "y": 0.5},
        {"id": "p2", "name": "p2", "created_by": "user", "kind": "photo", "status": "open", "x": None, "y": None},
    ]
    return r


@pytest.fixture
def server(robot: FakeRobot) -> Any:
    return build_mcp_server(robot, McpServerConfig(), TOKEN)


def test_clear_photo_kind_sends_kinds_and_reports_both_counts(server: Any, robot: FakeRobot) -> None:
    assert payload(call(server, "clear_pois", {"kinds": ["photo"]})) == {"removed": 1, "pois": 0, "photos": 1}
    assert robot.poi_clear_args == [{"created_by": "agent", "kinds": ["photo"], "keep_zones": True}]


def test_default_clear_sends_no_kinds_and_reports_both_counts(server: Any, robot: FakeRobot) -> None:
    assert payload(call(server, "clear_pois", {})) == {"removed": 2, "pois": 1, "photos": 1}
    assert robot.poi_clear_args[0]["kinds"] is None


def test_list_pois_returns_only_non_photo_pois(server: Any) -> None:
    res = payload(call(server, "list_pois", {}))
    assert [p["id"] for p in res["pois"]] == ["a"]


def test_list_pois_near_ignores_photo_without_coordinates(server: Any, robot: FakeRobot) -> None:
    res = payload(call(server, "list_pois", {"near": True}))
    assert all(p["kind"] != "photo" for p in res["pois"])


def test_describe_pois_skips_photos_and_non_numeric_coordinates() -> None:
    pois = [
        {"id": "p", "kind": "photo", "x": 1.0, "y": 1.0},
        {"id": "n", "kind": "point", "x": None, "y": 2.0},
        {"id": "ok", "kind": "point", "x": 1.0, "y": 1.0},
    ]
    views = describe_pois(pois, (0.0, 0.0, 0.0))
    assert [v.id for v in views] == ["ok"]


def test_descriptions_cover_photos(server: Any) -> None:
    import anyio

    tools = {t.name: t.description for t in anyio.run(server.list_tools)}
    assert "photo" in tools["clear_pois"] and "never cleared" not in tools["clear_pois"]
    assert "keeps the photos linked" in tools["delete_poi"]
    assert "list_lookups" in tools["list_pois"]
