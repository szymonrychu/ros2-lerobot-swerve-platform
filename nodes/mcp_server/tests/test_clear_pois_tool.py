"""clear_pois tool and the PoiRequests.clear payloads."""

import json
from pathlib import Path
from typing import Any

import pytest
from mcp.server.mcpserver.exceptions import ToolError

from mcp_server.config import McpServerConfig
from mcp_server.poi_client import PoiRequests, PoiTimeout
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
        {"id": "a", "created_by": "agent", "kind": "point", "status": "open"},
        {"id": "z", "created_by": "agent", "kind": "area", "zone": "keepout", "status": "open"},
        {"id": "o", "created_by": "agent", "kind": "object", "status": "open"},
        {"id": "u", "created_by": "user", "kind": "point", "status": "open"},
    ]
    return r


@pytest.fixture
def server(robot: FakeRobot) -> Any:
    return build_mcp_server(robot, McpServerConfig(), TOKEN)


def ids(robot: FakeRobot) -> list[str]:
    return [p["id"] for p in robot.pois]


def sent(**kwargs: Any) -> dict:
    out: list[dict] = []
    reqs = PoiRequests(lambda p: out.append(json.loads(p)), lambda: True, 0.01, lambda: "r1")
    with pytest.raises(PoiTimeout):
        reqs.clear(**kwargs)
    return out[0]


def test_default_clears_the_agents_pois_and_keeps_zones(server: Any, robot: FakeRobot) -> None:
    assert payload(call(server, "clear_pois", {})) == {"removed": 2}
    assert robot.poi_clear_args == [{"created_by": "agent", "kinds": None, "keep_zones": True}]
    assert ids(robot) == ["z", "u"]


def test_user_and_all_creators(server: Any, robot: FakeRobot) -> None:
    assert payload(call(server, "clear_pois", {"creator": "user"})) == {"removed": 1}
    assert robot.poi_clear_args[-1]["created_by"] == "user"
    assert payload(call(server, "clear_pois", {"creator": "all"})) == {"removed": 2}
    assert robot.poi_clear_args[-1] == {"created_by": None, "kinds": None, "keep_zones": True}
    assert ids(robot) == ["z"]


def test_kinds_filter(server: Any, robot: FakeRobot) -> None:
    assert payload(call(server, "clear_pois", {"kinds": ["object"]})) == {"removed": 1}
    assert robot.poi_clear_args == [{"created_by": "agent", "kinds": ["object"], "keep_zones": True}]


def test_include_zones_sends_no_keep_zones(server: Any, robot: FakeRobot) -> None:
    assert payload(call(server, "clear_pois", {"include_zones": True})) == {"removed": 3}
    assert robot.poi_clear_args[0]["keep_zones"] is False


def test_invalid_creator_and_kind_rejected(server: Any, robot: FakeRobot) -> None:
    with pytest.raises(ToolError):
        call(server, "clear_pois", {"creator": "bob"})
    with pytest.raises(ToolError):
        call(server, "clear_pois", {"kinds": ["bogus"]})
    assert robot.poi_clear_args == []


def test_poi_store_down(server: Any, robot: FakeRobot) -> None:
    robot.poi_store_up = False
    with pytest.raises(ToolError):
        call(server, "clear_pois", {})


def test_description_states_the_consent_rules(server: Any) -> None:
    import anyio

    tools = {t.name: t.description for t in anyio.run(server.list_tools)}
    text = tools["clear_pois"]
    assert "only when the person asked" in text and "photos are never cleared" in text


def test_client_payloads_send_only_given_fields() -> None:
    admin = sent(created_by="agent")
    assert admin["created_by"] == "agent" and admin["op"] == "clear"
    assert not {"kinds", "keep_zones", "statuses"} & set(admin)
    everyone = sent(created_by=None, kinds=["area"], keep_zones=True)
    assert "created_by" not in everyone and everyone["kinds"] == ["area"] and everyone["keep_zones"] is True
