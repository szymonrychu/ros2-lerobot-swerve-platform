"""Named maps backend: /api/maps routes and the bridge's map_manager topics."""

from __future__ import annotations

import asyncio
import json
import threading
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock, patch

import httpx
import pytest
from ros2_common.maps import COMMAND_TOPIC, RESULT_TOPIC, STATUS_TOPIC, MapCommand, MapStatus

from web_ui import server
from web_ui.config import AppConfig, TabConfig
from web_ui.server import build_app

SETTLE_S = 0.05
GOAL_TOPIC = "/goal"
MAP_TOPIC = "/slam/map"
NAV_ACTION = "/nav"


def status_payload(load_seq: int = 0, poi_count: int = 2) -> dict[str, Any]:
    return {
        "maps": [
            {
                "name": "lab",
                "saved_at": 1.0,
                "note": "",
                "poi_count": poi_count,
                "last_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
                "valid": True,
            }
        ],
        "active_map": "lab",
        "load_seq": load_seq,
        "localisation": {"state": "unknown"},
    }


def done_future(result: Any) -> Future:
    fut: Future = Future()
    fut.set_result(result)
    return fut


def cancel_ok() -> Future:
    return done_future(SimpleNamespace(return_code=0, goals_canceling=[]))


@pytest.fixture
def client_factory(tmp_path: Path, urdf_dir: Path) -> Any:
    config = AppConfig(
        tabs=[
            TabConfig(
                id="map",
                type="map_nav",
                label="Map",
                map_topic=MAP_TOPIC,
                goal_topic=GOAL_TOPIC,
                map_reset_service="/slam/reset",
                navigate_action=NAV_ACTION,
            ),
            TabConfig(id="other", type="map_nav", label="Other", map_topic="/m2"),
        ]
    )

    def factory(bridge: Any) -> httpx.AsyncClient:
        app = build_app(config=config, urdf_dir=urdf_dir, static_dir=tmp_path / "none", bridge_node=bridge)
        return httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://test")

    return factory


def make_bridge(result: dict[str, Any] | None = None) -> MagicMock:
    bridge = MagicMock()
    bridge.cancel_all_goals_async.return_value = cancel_ok()
    bridge.map_request_async.return_value = done_future(result or {"request_id": "x", "ok": True, "message": "done"})
    return bridge


def test_timeouts() -> None:
    assert server.MAP_SAVE_TIMEOUT_S == 30.0
    assert server.MAP_LOAD_TIMEOUT_S == 120.0
    assert server.MAP_DELETE_TIMEOUT_S == 10.0


async def test_status_503_before_first_message(client_factory: Any) -> None:
    bridge = make_bridge()
    bridge.map_status.return_value = None
    async with client_factory(bridge) as client:
        resp = await client.get("/api/maps?tab=map")
    assert resp.status_code == 503
    assert "no map_manager status yet" in resp.json()["message"]


async def test_status_returns_cached(client_factory: Any) -> None:
    bridge = make_bridge()
    bridge.map_status.return_value = MapStatus.model_validate(status_payload())
    async with client_factory(bridge) as client:
        resp = await client.get("/api/maps?tab=map")
    assert resp.status_code == 200
    assert resp.json()["maps"][0]["name"] == "lab"
    assert resp.json()["load_seq"] == 0


@pytest.mark.parametrize(
    ("method", "path"),
    [("get", "/api/maps"), ("post", "/api/maps/save"), ("post", "/api/maps/load"), ("post", "/api/maps/delete")],
)
async def test_unknown_tab_404(client_factory: Any, method: str, path: str) -> None:
    async with client_factory(make_bridge()) as client:
        resp = await getattr(client, method)(
            f"{path}?tab=nope", **({"json": {"name": "a"}} if method == "post" else {})
        )
    assert resp.status_code == 404


@pytest.mark.parametrize(
    ("method", "path"),
    [("get", "/api/maps"), ("post", "/api/maps/save"), ("post", "/api/maps/load"), ("post", "/api/maps/delete")],
)
async def test_bridge_missing_503(client_factory: Any, method: str, path: str) -> None:
    async with client_factory(None) as client:
        resp = await getattr(client, method)(f"{path}?tab=map", **({"json": {"name": "a"}} if method == "post" else {}))
    assert resp.status_code == 503


async def test_save_success_sends_command(client_factory: Any) -> None:
    bridge = make_bridge()
    async with client_factory(bridge) as client:
        resp = await client.post("/api/maps/save?tab=map", json={"name": "lab", "overwrite": True, "note": "hi"})
    assert resp.status_code == 200
    assert resp.json() == {"ok": True, "message": "done"}
    command = bridge.map_request_async.call_args.args[0]
    assert isinstance(command, MapCommand)
    assert (command.op, command.name, command.overwrite, command.note) == ("save", "lab", True, "hi")
    bridge.cancel_all_goals_async.assert_not_called()


async def test_save_result_not_ok_is_400(client_factory: Any) -> None:
    bridge = make_bridge({"request_id": "x", "ok": False, "message": "exists"})
    async with client_factory(bridge) as client:
        resp = await client.post("/api/maps/save?tab=map", json={"name": "lab"})
    assert resp.status_code == 400
    assert resp.json() == {"ok": False, "message": "exists"}


@pytest.mark.parametrize("path", ["save", "load", "delete"])
async def test_validation_error_422(client_factory: Any, path: str) -> None:
    bridge = make_bridge()
    async with client_factory(bridge) as client:
        resp = await client.post(f"/api/maps/{path}?tab=map", json={"name": "bad name!"})
    assert resp.status_code == 422
    assert "invalid map name" in resp.json()["message"]
    bridge.map_request_async.assert_not_called()


async def test_non_object_body_422(client_factory: Any) -> None:
    async with client_factory(make_bridge()) as client:
        resp = await client.post("/api/maps/save?tab=map", json=["x"])
    assert resp.status_code == 422


@pytest.mark.parametrize("path", ["save", "load", "delete"])
async def test_map_manager_down_503(client_factory: Any, path: str) -> None:
    bridge = make_bridge()
    bridge.map_request_async.return_value = None
    async with client_factory(bridge) as client:
        resp = await client.post(f"/api/maps/{path}?tab=map", json={"name": "lab"})
    assert resp.status_code == 503
    assert "map_manager is not running" in resp.json()["message"]


@pytest.mark.parametrize(
    ("path", "attr"),
    [("save", "MAP_SAVE_TIMEOUT_S"), ("load", "MAP_LOAD_TIMEOUT_S"), ("delete", "MAP_DELETE_TIMEOUT_S")],
)
async def test_timeout_504(client_factory: Any, path: str, attr: str) -> None:
    bridge = make_bridge()
    pending: Future = Future()
    bridge.map_request_async.return_value = pending
    with patch.object(server, attr, 0.05):
        async with client_factory(bridge) as client:
            resp = await client.post(f"/api/maps/{path}?tab=map", json={"name": "lab"})
    assert resp.status_code == 504
    assert pending.cancelled()


async def test_delete_success_not_battery_guarded_and_no_cancel(client_factory: Any) -> None:
    bridge = make_bridge()
    async with client_factory(bridge) as client:
        resp = await client.post("/api/maps/delete?tab=map", json={"name": "lab"})
    assert resp.status_code == 200
    assert bridge.map_request_async.call_args.args[0].op == "delete"
    bridge.cancel_all_goals_async.assert_not_called()


async def test_load_cancels_goals_first(client_factory: Any) -> None:
    order: list[str] = []
    bridge = make_bridge()
    bridge.cancel_all_goals_async.side_effect = lambda action: (order.append("cancel"), cancel_ok())[1]
    bridge.map_request_async.side_effect = lambda cmd: (
        order.append("load"),
        done_future({"request_id": "x", "ok": True, "message": "loaded"}),
    )[1]
    async with client_factory(bridge) as client:
        resp = await client.post(
            "/api/maps/load?tab=map", json={"name": "lab", "pose": {"x": 1.0, "y": 2.0, "yaw": 0.5}}
        )
    assert resp.status_code == 200
    assert order == ["cancel", "load"]
    bridge.cancel_all_goals_async.assert_called_once_with(NAV_ACTION)
    assert bridge.map_request_async.call_args.args[0].pose.x == 1.0


@pytest.mark.parametrize("cancel", [None, SimpleNamespace(return_code=1, goals_canceling=[])])
async def test_load_stops_when_cancel_fails(client_factory: Any, cancel: Any) -> None:
    bridge = make_bridge()
    bridge.cancel_all_goals_async.return_value = None if cancel is None else done_future(cancel)
    async with client_factory(bridge) as client:
        resp = await client.post("/api/maps/load?tab=map", json={"name": "lab"})
    assert resp.status_code in (500, 503)
    assert resp.json()["ok"] is False
    bridge.map_request_async.assert_not_called()


async def test_concurrent_identical_loads_share_one_command(client_factory: Any) -> None:
    bridge = make_bridge()
    pending: Future = Future()
    bridge.map_request_async.return_value = pending
    async with client_factory(bridge) as client:
        first = asyncio.create_task(client.post("/api/maps/load?tab=map", json={"name": "lab"}))
        await asyncio.sleep(SETTLE_S)
        second = asyncio.create_task(client.post("/api/maps/load?tab=map", json={"name": "lab"}))
        await asyncio.sleep(SETTLE_S)
        pending.set_result({"request_id": "x", "ok": True, "message": "loaded"})
        r1, r2 = await asyncio.gather(first, second)
    assert bridge.map_request_async.call_count == 1
    assert r1.status_code == r2.status_code == 200


# ---- bridge ----


def make_node(**attrs: Any) -> Any:
    from web_ui.bridge import BridgeNode

    node = BridgeNode.__new__(BridgeNode)
    node._latest = {}
    node._dirty = set()
    node._pending = {}
    node._cleared = set()
    node._lock = threading.Lock()
    node._gps_anchor = None
    node._map_topics = [MAP_TOPIC]
    node._map_status = None
    node._map_pending = {}
    node._map_command_pub = None
    for key, value in attrs.items():
        setattr(node, key, value)
    return node


def status_msg(**kwargs: Any) -> Any:
    return SimpleNamespace(data=json.dumps(status_payload(**kwargs)))


def test_bridge_subscribes_to_map_manager_topics() -> None:
    from web_ui.bridge import BridgeNode

    created: list[tuple[Any, str, Any, Any]] = []
    publishers: list[str] = []
    with (
        patch("web_ui.bridge.Node.__init__", return_value=None),
        patch.object(
            BridgeNode,
            "create_subscription",
            create=True,
            side_effect=lambda c, t, cb, q: created.append((c, t, cb, q)),
        ),
        patch.object(
            BridgeNode, "create_publisher", create=True, side_effect=lambda c, t, q: publishers.append(t) or MagicMock()
        ),
        patch.object(BridgeNode, "create_timer", create=True),
        patch.object(BridgeNode, "get_logger", create=True),
        patch.object(BridgeNode, "create_client", create=True),
    ):
        node = BridgeNode(topics=["/map"], allowed_publish_topics=set(), topic_roles={"/map": "map"})
    by_topic = {t: (cb, q) for _, t, cb, q in created}
    assert by_topic[STATUS_TOPIC][0] == node.on_map_status
    assert by_topic[RESULT_TOPIC][0] == node.on_map_result
    from rclpy.qos import DurabilityPolicy

    assert by_topic[STATUS_TOPIC][1].durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert COMMAND_TOPIC in publishers


def test_status_cached_and_invalid_ignored() -> None:
    node = make_node()
    assert node.map_status() is None
    node.on_map_status(SimpleNamespace(data="not json"))
    node.on_map_status(SimpleNamespace(data=json.dumps({"maps": "x"})))
    assert node.map_status() is None
    node.on_map_status(status_msg())
    assert node.map_status().maps[0].name == "lab"
    node.on_map_status(SimpleNamespace(data="garbage"))
    assert node.map_status().maps[0].name == "lab"


def test_first_status_does_not_clear_map() -> None:
    anchor = MagicMock()
    node = make_node(_gps_anchor=anchor)
    node._latest[MAP_TOPIC] = {"topic": MAP_TOPIC, "data": {"x": 1}}
    node.on_map_status(status_msg(load_seq=3))
    anchor.reset.assert_not_called()
    assert MAP_TOPIC in node._latest


def test_unchanged_load_seq_does_not_clear() -> None:
    anchor = MagicMock()
    node = make_node(_gps_anchor=anchor)
    node.on_map_status(status_msg(load_seq=3))
    node._latest[MAP_TOPIC] = {"topic": MAP_TOPIC, "data": {"x": 1}}
    node.on_map_status(status_msg(load_seq=3, poi_count=5))
    anchor.reset.assert_not_called()
    assert MAP_TOPIC in node._latest


def test_load_seq_change_clears_map_and_gps_anchor() -> None:
    from web_ui.bridge import GPS_ANCHOR_TOPIC

    anchor = MagicMock()
    node = make_node(_gps_anchor=anchor)
    node.on_map_status(status_msg(load_seq=3))
    node._latest[MAP_TOPIC] = {"topic": MAP_TOPIC, "data": {"x": 1}}
    node._latest[GPS_ANCHOR_TOPIC] = {"topic": GPS_ANCHOR_TOPIC, "data": {"lat": 1.0}}
    node.on_map_status(status_msg(load_seq=4))
    anchor.reset.assert_called_once()
    assert {env["topic"]: env["data"] for env in node.flush_dirty()} == {MAP_TOPIC: None, GPS_ANCHOR_TOPIC: None}


def test_map_request_async_none_without_subscriber() -> None:
    node = make_node()
    cmd = MapCommand(op="delete", name="lab")
    assert node.map_request_async(cmd) is None
    pub = MagicMock()
    pub.get_subscription_count.return_value = 0
    node._map_command_pub = pub
    assert node.map_request_async(cmd) is None
    pub.publish.assert_not_called()


def test_map_request_matches_result_by_request_id() -> None:
    pub = MagicMock()
    pub.get_subscription_count.return_value = 1
    node = make_node(_map_command_pub=pub)
    f1 = node.map_request_async(MapCommand(op="save", name="a"))
    f2 = node.map_request_async(MapCommand(op="delete", name="b"))
    sent = [json.loads(c.args[0].data) for c in pub.publish.call_args_list]
    assert sent[0]["request_id"] != sent[1]["request_id"]
    assert sent[1]["op"] == "delete"
    node.on_map_result(
        SimpleNamespace(data=json.dumps({"request_id": sent[1]["request_id"], "ok": True, "message": "m"}))
    )
    assert not f1.done()
    assert f2.result()["message"] == "m"
    node.on_map_result(SimpleNamespace(data="junk"))
    node.on_map_result(SimpleNamespace(data=json.dumps({"request_id": "unknown", "ok": True, "message": ""})))
    assert not f1.done()


def test_map_reset_keeps_saved_maps_listed() -> None:
    """D13: map_guard's reset clears the map and anchor; the status (maps list) is untouched until map_manager updates it."""
    node = make_node()
    node.on_map_status(status_msg(poi_count=2))
    node.on_map_guard_reset(object())
    assert [m.name for m in node.map_status().maps] == ["lab"]
    node.on_map_status(status_msg(poi_count=0))
    status = node.map_status()
    assert [m.name for m in status.maps] == ["lab"]
    assert status.maps[0].poi_count == 0


async def test_unrecoverable_move_adds_no_route(client_factory: Any) -> None:
    async with client_factory(make_bridge()) as client:
        resp = await client.post("/api/maps/clear?tab=map")
    assert resp.status_code in (404, 405)
