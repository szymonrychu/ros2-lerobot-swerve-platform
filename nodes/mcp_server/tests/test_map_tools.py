"""Named map tools (save_map, list_maps, load_map, delete_map) and the MapRequests client."""

import json
from pathlib import Path
from typing import Any

import pytest
from mcp.server.mcpserver.exceptions import ToolError
from ros2_common.maps import COMMAND_TOPIC, RESULT_TOPIC, STATUS_TOPIC, Localisation, MapStatus, MapSummary, Pose2D

from mcp_server.config import McpServerConfig
from mcp_server.map_client import MapManagerDown, MapRequests, MapTimeout
from mcp_server.models import RobotError
from mcp_server.tools import ALWAYS_ALLOWED_TOOLS, TOOL_NAMES, build_mcp_server

from .test_tools import TOKEN, FakeRobot, call

MAP_TOOLS = {"save_map", "list_maps", "load_map", "delete_map"}
VERDICT = {"state": "localised", "ratio": 0.82, "pose": {"x": 1.0, "y": 2.0, "yaw": 0.3}, "reason": ""}


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


def status(state: str = "localised") -> MapStatus:
    return MapStatus(
        maps=[
            MapSummary(
                name="kitchen",
                saved_at=1.0,
                note="n",
                poi_count=3,
                last_pose=Pose2D(x=1, y=2, yaw=0),
                valid=True,
            )
        ],
        active_map="kitchen",
        load_seq=1,
        localisation=Localisation(state=state, ratio=0.8),
    )


def test_topics_come_from_the_shared_contract() -> None:
    topics = McpServerConfig().maps
    assert (topics.command, topics.result, topics.status) == (COMMAND_TOPIC, RESULT_TOPIC, STATUS_TOPIC)
    assert (topics.save_timeout_s, topics.load_timeout_s, topics.delete_timeout_s) == (30.0, 120.0, 10.0)


def test_tools_are_registered_and_not_motion(server: Any) -> None:
    assert MAP_TOOLS <= set(TOOL_NAMES) and MAP_TOOLS <= ALWAYS_ALLOWED_TOOLS


def test_save_map_sends_name_overwrite_and_note(server: Any, robot: FakeRobot) -> None:
    robot.map_script = lambda cmd: {"ok": True, "message": "saved kitchen", "data": {"poi_count": 3}}
    res = call(server, "save_map", {"name": "kitchen", "overwrite": True, "note": "tidy"})
    (sent,) = robot.map_commands
    assert sent["op"] == "save" and sent["name"] == "kitchen" and sent["overwrite"] is True and sent["note"] == "tidy"
    assert sent["request_id"]
    body = payload(res)
    assert body["ok"] is True and body["message"] == "saved kitchen" and body["data"] == {"poi_count": 3}


def test_save_map_defaults(server: Any, robot: FakeRobot) -> None:
    robot.map_script = lambda cmd: {"ok": True, "message": "saved"}
    call(server, "save_map", {"name": "a"})
    assert robot.map_commands[0]["overwrite"] is False and robot.map_commands[0]["note"] == ""


def test_save_map_bad_name_is_refused_without_sending(server: Any, robot: FakeRobot) -> None:
    with pytest.raises(ToolError, match="invalid map name"):
        call(server, "save_map", {"name": "../x"})
    assert robot.map_commands == []


def test_rejected_command_carries_map_managers_message(server: Any, robot: FakeRobot) -> None:
    robot.map_script = lambda cmd: {"ok": False, "message": "map kitchen exists; pass overwrite"}
    with pytest.raises(ToolError, match="map kitchen exists; pass overwrite"):
        call(server, "save_map", {"name": "kitchen"})


def test_map_manager_down_is_an_error_without_publishing(server: Any, robot: FakeRobot) -> None:
    robot.map_up = False
    with pytest.raises(ToolError, match="map_manager is not running"):
        call(server, "save_map", {"name": "kitchen"})
    assert robot.map_commands == []


def test_timeout_is_an_error(server: Any, robot: FakeRobot) -> None:
    robot.map_script = lambda cmd: None
    with pytest.raises(ToolError, match="did not answer"):
        call(server, "load_map", {"name": "kitchen"})


def test_list_maps_returns_maps_active_map_and_localisation(server: Any, robot: FakeRobot) -> None:
    robot.map_status_msg = status()
    body = payload(call(server, "list_maps", {}))
    assert body["maps"][0]["name"] == "kitchen" and body["maps"][0]["poi_count"] == 3
    assert body["active_map"] == "kitchen" and body["localisation"]["state"] == "localised"
    assert robot.map_commands == []


def test_list_maps_before_any_status_is_an_error(server: Any, robot: FakeRobot) -> None:
    robot.map_status_msg = None
    with pytest.raises(ToolError, match="no /map_manager/status"):
        call(server, "list_maps", {})


def test_load_map_stops_first_then_returns_the_verdict(server: Any, robot: FakeRobot) -> None:
    order: list[str] = []
    robot.map_script = lambda cmd: order.append("load") or {"ok": True, "message": "loaded", "data": VERDICT}
    robot.stop_hook = lambda: order.append("stop")
    body = payload(call(server, "load_map", {"name": "kitchen"}))
    assert order == ["stop", "load"]
    (sent,) = robot.map_commands
    assert sent["op"] == "load" and sent["name"] == "kitchen" and sent["pose"] is None
    assert body["state"] == "localised" and body["ratio"] == 0.82 and body["pose"] == VERDICT["pose"]
    assert body["name"] == "kitchen"


def test_load_map_with_full_pose_sends_it(server: Any, robot: FakeRobot) -> None:
    robot.map_script = lambda cmd: {
        "ok": True,
        "message": "m",
        "data": {**VERDICT, "state": "not_localised", "reason": "low"},
    }
    body = payload(call(server, "load_map", {"name": "kitchen", "x": 1.5, "y": -2.0, "yaw": 0.25}))
    assert robot.map_commands[0]["pose"] == {"x": 1.5, "y": -2.0, "yaw": 0.25}
    assert body["state"] == "not_localised" and body["reason"] == "low"


@pytest.mark.parametrize("given", [{"x": 1.0}, {"x": 1.0, "y": 2.0}, {"yaw": 0.1}, {"y": 1.0, "yaw": 0.0}])
def test_load_map_partial_pose_is_an_error_before_stop_or_send(server: Any, robot: FakeRobot, given: dict) -> None:
    with pytest.raises(ToolError, match="x, y and yaw"):
        call(server, "load_map", {"name": "kitchen", **given})
    assert robot.map_commands == [] and robot.stops == 0


def test_delete_map_without_confirm_sends_nothing(server: Any, robot: FakeRobot) -> None:
    with pytest.raises(ToolError, match="ask the person"):
        call(server, "delete_map", {"name": "kitchen"})
    assert robot.map_commands == []


def test_delete_map_with_confirm_sends_delete(server: Any, robot: FakeRobot) -> None:
    robot.map_script = lambda cmd: {"ok": True, "message": "deleted kitchen"}
    body = payload(call(server, "delete_map", {"name": "kitchen", "confirm": True}))
    assert robot.map_commands[0]["op"] == "delete" and robot.map_commands[0]["name"] == "kitchen"
    assert body["ok"] is True


def test_descriptions_state_the_consequences(server: Any) -> None:
    tools = {t.name: t.description for t in __import__("anyio").run(server.list_tools)}
    assert "replaces the current map and POIs" in tools["load_map"] and "autosave" in tools["load_map"]
    assert "do not navigate" in tools["load_map"] and "pose hint" in tools["load_map"]
    assert "POIs" in tools["delete_map"] and "cannot be undone" in tools["delete_map"]


class Wire:
    def __init__(self, answer: Any, up: bool = True) -> None:
        self.sent: list[dict] = []
        self.answer = answer
        self.client = MapRequests(self.publish, lambda: up, lambda op: 0.2)

    def publish(self, text: str) -> None:
        msg = json.loads(text)
        self.sent.append(msg)
        reply = self.answer(msg)
        if reply is not None:
            self.client.on_result(json.dumps({"request_id": msg["request_id"], **reply}))


def test_client_ignores_other_request_ids_and_malformed_payloads() -> None:
    wire = Wire(lambda m: {"ok": True, "message": "ok"})
    wire.client.on_result("not json")
    wire.client.on_result(json.dumps({"request_id": "other", "ok": False, "message": "x"}))
    assert wire.client.send({"op": "save", "name": "a"}).message == "ok"
    assert wire.client.pending_count() == 0


def test_client_errors_are_robot_errors() -> None:
    assert issubclass(MapManagerDown, RobotError) and issubclass(MapTimeout, RobotError)
    with pytest.raises(MapManagerDown):
        Wire(lambda m: None, up=False).client.send({"op": "save", "name": "a"})
    with pytest.raises(MapTimeout):
        Wire(lambda m: None).client.send({"op": "save", "name": "a"})
