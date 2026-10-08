"""Tests for the POI layer backend: config roles, /poi/list forwarding, POST /api/poi."""

from __future__ import annotations

import json
import threading
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock, patch

import pytest
from fastapi.testclient import TestClient

from web_ui.config import AppConfig, TabConfig

LIST_PAYLOAD = {"pois": [{"id": "a" * 32, "kind": "point", "x": 1.0, "y": 2.0}], "revision": 4}


def make_bridge(**attrs: Any) -> Any:
    """Build a BridgeNode without rclpy initialisation."""
    from web_ui.bridge import BridgeNode

    node = BridgeNode.__new__(BridgeNode)
    node._latest = {}
    node._dirty = set()
    node._cleared = set()
    node._lock = threading.Lock()
    node._topic_last_rx = {}
    node._poi_pending = {}
    node._poi_command_pub = MagicMock()
    node._poi_command_pub.get_subscription_count.return_value = 1
    for key, value in attrs.items():
        setattr(node, key, value)
    return node


def test_map_nav_poi_topic_defaults() -> None:
    tab = TabConfig(id="m", type="map_nav", label="Map")
    assert (tab.poi_list_topic, tab.poi_command_topic, tab.poi_result_topic) == (
        "/poi/list",
        "/poi/command",
        "/poi/result",
    )


def test_poi_roles_and_subscriptions() -> None:
    cfg = AppConfig(tabs=[TabConfig(id="m", type="map_nav", label="Map")])
    roles = cfg.topic_roles()
    assert roles["/poi/list"] == "poi_list" and roles["/poi/result"] == "poi_result"
    assert "/poi/command" not in roles
    assert "/poi/list" in cfg.all_subscribed_topics()
    assert cfg.poi_command_topic() == "/poi/command"


def test_poi_command_topic_none_without_map_nav() -> None:
    assert AppConfig(tabs=[]).poi_command_topic() is None


def test_subscription_spec_poi_list_is_latched() -> None:
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

    from web_ui.bridge import subscription_spec
    from web_ui.msg_serializer import serialize_poi_list

    _, qos, serializer = subscription_spec("/poi/list", "poi_list")
    assert qos.reliability is ReliabilityPolicy.RELIABLE
    assert qos.durability is DurabilityPolicy.TRANSIENT_LOCAL
    assert serializer is serialize_poi_list


def test_serialize_poi_list() -> None:
    from web_ui.msg_serializer import serialize_poi_list

    assert serialize_poi_list(SimpleNamespace(data=json.dumps(LIST_PAYLOAD))) == LIST_PAYLOAD


def test_serialize_poi_list_keeps_object_pois_with_their_sighting_fields() -> None:
    from web_ui.msg_serializer import serialize_poi_list

    cup = {"id": "b" * 32, "kind": "object", "name": "cup", "x": 1.0, "y": 2.0, "times_seen": 3, "last_seen": 9.0}
    payload = {"pois": [*LIST_PAYLOAD["pois"], cup], "revision": 5}
    assert serialize_poi_list(SimpleNamespace(data=json.dumps(payload))) == payload


@pytest.mark.parametrize("raw", ["{oops", "[]", json.dumps({"pois": "x"}), json.dumps({"revision": 1})])
def test_serialize_poi_list_rejects_malformed(raw: str) -> None:
    from web_ui.msg_serializer import serialize_poi_list

    with pytest.raises(ValueError):
        serialize_poi_list(SimpleNamespace(data=raw))


def test_poi_request_publishes_command_with_request_id() -> None:
    node = make_bridge()
    future = node.poi_request_async({"op": "add", "poi": {"kind": "point"}})
    sent = json.loads(node._poi_command_pub.publish.call_args[0][0].data)
    assert sent["op"] == "add" and sent["request_id"] and sent["poi"] == {"kind": "point"}
    assert sent["request_id"] in node._poi_pending
    node.on_poi_result(
        SimpleNamespace(data=json.dumps({"request_id": sent["request_id"], "ok": True, "message": "m", "poi": None}))
    )
    assert future.result()["ok"] is True
    assert node._poi_pending == {}


def test_poi_request_none_when_store_not_running() -> None:
    node = make_bridge()
    node._poi_command_pub.get_subscription_count.return_value = 0
    assert node.poi_request_async({"op": "delete", "poi": {"id": "a"}}) is None
    node._poi_command_pub = None
    assert node.poi_request_async({"op": "delete", "poi": {"id": "a"}}) is None


def test_poi_result_unknown_or_malformed_ignored() -> None:
    node = make_bridge()
    node.on_poi_result(SimpleNamespace(data=json.dumps({"request_id": "zzz", "ok": True})))
    node.on_poi_result(SimpleNamespace(data="{bad"))
    assert node._poi_pending == {}


def test_cancelled_request_is_forgotten() -> None:
    node = make_bridge()
    future = node.poi_request_async({"op": "add", "poi": {}})
    future.cancel()
    assert node._poi_pending == {}


class FakeBridge:
    """Stand-in bridge: returns a canned future (or None) for POI requests."""

    def __init__(self, result: dict[str, Any] | None = None, never: bool = False, none: bool = False) -> None:
        self.result = result
        self.never = never
        self.none = none
        self.payloads: list[dict[str, Any]] = []

    def poi_request_async(self, payload: dict[str, Any]) -> Future | None:
        self.payloads.append(payload)
        if self.none:
            return None
        future: Future = Future()
        if not self.never:
            future.set_result(self.result)
        return future


def make_client(tmp_path: Path, bridge: Any, battery_guard: Any = None) -> TestClient:
    from web_ui.server import build_app

    config = AppConfig(tabs=[TabConfig(id="map", type="map_nav", label="Map")])
    app = build_app(
        config=config, urdf_dir=tmp_path, static_dir=tmp_path / "none", bridge_node=bridge, battery_guard=battery_guard
    )
    return TestClient(app)


def test_api_poi_returns_store_result(tmp_path: Path) -> None:
    result = {"request_id": "x", "ok": True, "message": "added", "poi": {"id": "a" * 32}}
    bridge = FakeBridge(result)
    resp = make_client(tmp_path, bridge).post("/api/poi?tab=map", json={"op": "add", "poi": {"kind": "point"}})
    assert resp.status_code == 200 and resp.json() == result
    assert bridge.payloads == [{"op": "add", "poi": {"kind": "point"}}]


def test_api_poi_store_rejection_is_400(tmp_path: Path) -> None:
    result = {"request_id": "x", "ok": False, "message": "unknown poi id", "poi": None}
    resp = make_client(tmp_path, FakeBridge(result)).post("/api/poi?tab=map", json={"op": "delete", "poi": {"id": "a"}})
    assert resp.status_code == 400 and resp.json()["ok"] is False


def test_api_poi_503_when_store_not_running(tmp_path: Path) -> None:
    resp = make_client(tmp_path, FakeBridge(none=True)).post("/api/poi?tab=map", json={"op": "add", "poi": {}})
    assert resp.status_code == 503 and resp.json()["ok"] is False


def test_api_poi_503_without_bridge(tmp_path: Path) -> None:
    assert make_client(tmp_path, None).post("/api/poi?tab=map", json={"op": "add", "poi": {}}).status_code == 503


def test_api_poi_timeout(tmp_path: Path) -> None:
    with patch("web_ui.server.POI_TIMEOUT_S", 0.05):
        resp = make_client(tmp_path, FakeBridge(never=True)).post("/api/poi?tab=map", json={"op": "add", "poi": {}})
    assert resp.status_code == 504


@pytest.mark.parametrize(
    "body", [{"op": "explode", "poi": {}}, {"op": "add"}, {"op": "add", "poi": []}, {"poi": {}}, ["x"]]
)
def test_api_poi_bad_body_is_422(tmp_path: Path, body: Any) -> None:
    assert make_client(tmp_path, FakeBridge({})).post("/api/poi?tab=map", json=body).status_code == 422


def test_api_poi_unknown_tab_is_404(tmp_path: Path) -> None:
    assert (
        make_client(tmp_path, FakeBridge({})).post("/api/poi?tab=nope", json={"op": "add", "poi": {}}).status_code
        == 404
    )


def test_api_poi_allowed_during_battery_cutoff(tmp_path: Path) -> None:
    guard = MagicMock()
    guard.is_cutoff.return_value = True
    result = {"request_id": "x", "ok": True, "message": "added", "poi": None}
    resp = make_client(tmp_path, FakeBridge(result), guard).post("/api/poi?tab=map", json={"op": "add", "poi": {}})
    assert resp.status_code == 200
