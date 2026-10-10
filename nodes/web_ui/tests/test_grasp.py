"""Tests for the grasp backend: config, /grasp/result role, bridge request correlation, POST /api/grasp[/stop]."""

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

OBJECT = {"frame": "arm", "x": 0.22, "y": 0.0, "support_z": -0.15, "width_m": 0.03, "depth_m": 0.03, "height_m": 0.04}
PLAN_BODY = {"action": "plan", "object": OBJECT, "strategy": "auto"}
EXEC_BODY = {"action": "execute", "object": OBJECT, "strategy": "scoop"}
PLANNED = {
    "ok": True,
    "request_id": "r1",
    "action": "plan",
    "result": {"outcome": "planned", "reasons": [], "plan": {"strategy": "scoop", "feasible": True, "waypoints": []}},
}


def make_bridge(**attrs: Any) -> Any:
    """Build a BridgeNode without rclpy initialisation."""
    from web_ui.bridge import BridgeNode

    node = BridgeNode.__new__(BridgeNode)
    node._latest = {}
    node._dirty = set()
    node._cleared = set()
    node._lock = threading.Lock()
    node._topic_last_rx = {}
    node._grasp_pending = {}
    node._grasp_command_pub = MagicMock()
    node._grasp_command_pub.get_subscription_count.return_value = 1
    for key, value in attrs.items():
        setattr(node, key, value)
    return node


def sent_json(node: Any) -> dict[str, Any]:
    return json.loads(node._grasp_command_pub.publish.call_args[0][0].data)


def test_map_nav_grasp_defaults() -> None:
    tab = TabConfig(id="m", type="map_nav", label="Map")
    assert (tab.grasp_command_topic, tab.grasp_result_topic) == ("/grasp/command", "/grasp/result")
    assert tab.grasp_timeout_s == 30.0
    assert tab.grasp_accept_wait_s == 1.0


@pytest.mark.parametrize("field", ["grasp_timeout_s", "grasp_accept_wait_s"])
def test_grasp_timeouts_must_be_positive(field: str) -> None:
    with pytest.raises(ValueError):
        TabConfig(id="m", type="map_nav", label="Map", **{field: 0})


def test_grasp_roles_and_command_topic() -> None:
    cfg = AppConfig(tabs=[TabConfig(id="m", type="map_nav", label="Map")])
    assert cfg.topic_roles()["/grasp/result"] == "grasp_result"
    assert "/grasp/command" not in cfg.topic_roles()
    assert "/grasp/result" in cfg.all_subscribed_topics()
    assert cfg.grasp_command_topic() == "/grasp/command"
    assert AppConfig(tabs=[]).grasp_command_topic() is None


def test_grasp_request_publishes_with_fresh_request_id_and_overrides_client_id() -> None:
    node = make_bridge()
    request_id, future = node.grasp_request_async({**PLAN_BODY, "request_id": "evil"})
    sent = sent_json(node)
    assert sent["request_id"] == request_id != "evil"
    assert sent["action"] == "plan" and sent["object"] == OBJECT and sent["strategy"] == "auto"
    assert request_id in node._grasp_pending
    node.on_grasp_result(SimpleNamespace(data=json.dumps({**PLANNED, "request_id": request_id})))
    assert future.result()["result"]["outcome"] == "planned"
    assert node._grasp_pending == {}


def test_grasp_request_none_when_mcp_server_not_running() -> None:
    node = make_bridge()
    node._grasp_command_pub.get_subscription_count.return_value = 0
    assert node.grasp_request_async(PLAN_BODY) is None
    node._grasp_command_pub = None
    assert node.grasp_request_async(PLAN_BODY) is None


def test_plan_result_is_not_streamed_to_ws() -> None:
    node = make_bridge()
    request_id, _ = node.grasp_request_async(PLAN_BODY)
    node.on_grasp_result(SimpleNamespace(data=json.dumps({**PLANNED, "request_id": request_id})))
    assert "/web_ui/grasp_result" not in node._latest


def test_execute_publishes_running_then_streams_result() -> None:
    node = make_bridge()
    request_id, future = node.grasp_request_async(EXEC_BODY)
    assert node._latest["/web_ui/grasp_result"]["data"] == {
        "request_id": request_id,
        "action": "execute",
        "state": "running",
    }
    done = {"ok": True, "request_id": request_id, "action": "execute", "result": {"outcome": "grasped", "steps": []}}
    node.on_grasp_result(SimpleNamespace(data=json.dumps(done)))
    assert node._latest["/web_ui/grasp_result"]["data"] == {**done, "state": "done"}
    assert future.result() == done
    assert "/web_ui/grasp_result" in node._dirty


def test_execute_rejection_streams_done_with_error() -> None:
    node = make_bridge()
    request_id, _ = node.grasp_request_async(EXEC_BODY)
    err = {"ok": False, "request_id": request_id, "action": "execute", "error": "another grasp action is running"}
    node.on_grasp_result(SimpleNamespace(data=json.dumps(err)))
    assert node._latest["/web_ui/grasp_result"]["data"] == {**err, "state": "done"}


def test_stop_result_does_not_overwrite_streamed_execute_state() -> None:
    node = make_bridge()
    request_id, _ = node.grasp_request_async(EXEC_BODY)
    stop_id, stop_future = node.grasp_request_async({"action": "stop"})
    stop = {"ok": True, "request_id": stop_id, "action": "stop", "result": {"arm_held": True, "message": "held"}}
    node.on_grasp_result(SimpleNamespace(data=json.dumps(stop)))
    assert stop_future.result() == stop
    assert node._latest["/web_ui/grasp_result"]["data"]["state"] == "running"
    assert node._latest["/web_ui/grasp_result"]["data"]["request_id"] == request_id


def test_grasp_result_unknown_or_malformed_ignored() -> None:
    node = make_bridge()
    node.on_grasp_result(SimpleNamespace(data=json.dumps({"request_id": "zzz", "ok": True, "action": "plan"})))
    node.on_grasp_result(SimpleNamespace(data="{bad"))
    node.on_grasp_result(SimpleNamespace(data="[]"))
    assert node._grasp_pending == {} and node._latest == {}


def test_cancelled_grasp_request_is_forgotten() -> None:
    node = make_bridge()
    _, future = node.grasp_request_async(PLAN_BODY)
    future.cancel()
    assert node._grasp_pending == {}


class FakeBridge:
    """Stand-in bridge: canned (request_id, future), or None."""

    def __init__(self, result: dict[str, Any] | None = None, never: bool = False, none: bool = False) -> None:
        self.result = result
        self.never = never
        self.none = none
        self.payloads: list[dict[str, Any]] = []
        self.futures: list[Future] = []

    def grasp_request_async(self, payload: dict[str, Any]) -> tuple[str, Future] | None:
        self.payloads.append(payload)
        if self.none:
            return None
        future: Future = Future()
        if not self.never:
            future.set_result(self.result)
        self.futures.append(future)
        return "rid", future


def make_client(tmp_path: Path, bridge: Any, battery_guard: Any = None, **tab: Any) -> TestClient:
    from web_ui.server import build_app

    config = AppConfig(tabs=[TabConfig(id="map", type="map_nav", label="Map", **tab)])
    app = build_app(
        config=config, urdf_dir=tmp_path, static_dir=tmp_path / "none", bridge_node=bridge, battery_guard=battery_guard
    )
    return TestClient(app)


def cutoff_guard() -> Any:
    guard = MagicMock()
    guard.is_cutoff.return_value = True
    guard.rejection_message.return_value = "battery below cut-off"
    return guard


def test_api_grasp_plan_returns_matching_result(tmp_path: Path) -> None:
    bridge = FakeBridge(PLANNED)
    resp = make_client(tmp_path, bridge).post("/api/grasp?tab=map", json=PLAN_BODY)
    assert resp.status_code == 200 and resp.json() == PLANNED
    assert bridge.payloads == [PLAN_BODY]


def test_api_grasp_plan_not_battery_blocked(tmp_path: Path) -> None:
    resp = make_client(tmp_path, FakeBridge(PLANNED), cutoff_guard()).post("/api/grasp?tab=map", json=PLAN_BODY)
    assert resp.status_code == 200


def test_api_grasp_plan_timeout_uses_tab_setting(tmp_path: Path) -> None:
    resp = make_client(tmp_path, FakeBridge(never=True), grasp_timeout_s=0.05).post(
        "/api/grasp?tab=map", json=PLAN_BODY
    )
    assert resp.status_code == 504 and resp.json()["ok"] is False


def test_api_grasp_mcp_error_is_400_with_result_body(tmp_path: Path) -> None:
    err = {"ok": False, "request_id": "rid", "action": "plan", "error": "invalid request: ..."}
    resp = make_client(tmp_path, FakeBridge(err)).post("/api/grasp?tab=map", json=PLAN_BODY)
    assert resp.status_code == 400 and resp.json() == err


def test_api_grasp_execute_returns_accepted_immediately_and_keeps_waiting(tmp_path: Path) -> None:
    bridge = FakeBridge(never=True)
    resp = make_client(tmp_path, bridge, grasp_accept_wait_s=0.05).post("/api/grasp?tab=map", json=EXEC_BODY)
    assert resp.status_code == 202
    body = resp.json()
    assert body["ok"] is True and body["accepted"] is True and body["request_id"] == "rid"
    assert body["action"] == "execute"
    assert not bridge.futures[0].cancelled()


def test_api_grasp_execute_immediate_rejection_is_returned(tmp_path: Path) -> None:
    err = {"ok": False, "request_id": "rid", "action": "execute", "error": "another grasp action is running"}
    resp = make_client(tmp_path, FakeBridge(err)).post("/api/grasp?tab=map", json=EXEC_BODY)
    assert resp.status_code == 400 and resp.json() == err


@pytest.mark.parametrize("body", [EXEC_BODY, {"action": "release"}])
def test_api_grasp_motion_blocked_in_battery_cutoff(tmp_path: Path, body: dict[str, Any]) -> None:
    bridge = FakeBridge(PLANNED)
    resp = make_client(tmp_path, bridge, cutoff_guard()).post("/api/grasp?tab=map", json=body)
    assert resp.status_code == 503 and resp.json()["ok"] is False
    assert resp.json()["message"] == "battery below cut-off"
    assert bridge.payloads == []


def test_api_grasp_release_forwarded(tmp_path: Path) -> None:
    bridge = FakeBridge(never=True)
    resp = make_client(tmp_path, bridge, grasp_accept_wait_s=0.05).post(
        "/api/grasp?tab=map", json={"action": "release"}
    )
    assert resp.status_code == 202 and bridge.payloads == [{"action": "release"}]


@pytest.mark.parametrize("grip", ["gentle", "firm", {"base": "gentle", "squeeze_rad": 0.01}])
def test_api_grasp_execute_passes_the_grip_profile_through(tmp_path: Path, grip: Any) -> None:
    bridge = FakeBridge(never=True)
    body = EXEC_BODY | {"grip_profile": grip}
    resp = make_client(tmp_path, bridge, grasp_accept_wait_s=0.05).post("/api/grasp?tab=map", json=body)
    assert resp.status_code == 202
    assert bridge.payloads == [body]


@pytest.mark.parametrize("grip", [3, ["gentle"], "", True])
def test_api_grasp_bad_grip_profile_is_422(tmp_path: Path, grip: Any) -> None:
    bridge = FakeBridge(PLANNED)
    resp = make_client(tmp_path, bridge).post("/api/grasp?tab=map", json=EXEC_BODY | {"grip_profile": grip})
    assert resp.status_code == 422
    assert bridge.payloads == []


def test_api_grasp_503_when_mcp_server_not_running(tmp_path: Path) -> None:
    resp = make_client(tmp_path, FakeBridge(none=True)).post("/api/grasp?tab=map", json=PLAN_BODY)
    assert resp.status_code == 503 and resp.json()["ok"] is False


def test_api_grasp_503_without_bridge(tmp_path: Path) -> None:
    assert make_client(tmp_path, None).post("/api/grasp?tab=map", json=PLAN_BODY).status_code == 503


@pytest.mark.parametrize(
    "body",
    [
        {"action": "explode"},
        {"action": "stop"},
        {"action": "plan"},
        {"action": "execute"},
        {},
        ["x"],
        {"action": "plan", "object": []},
    ],
)
def test_api_grasp_bad_body_is_422(tmp_path: Path, body: Any) -> None:
    bridge = FakeBridge(PLANNED)
    assert make_client(tmp_path, bridge).post("/api/grasp?tab=map", json=body).status_code == 422
    assert bridge.payloads == []


def test_api_grasp_unknown_tab_is_404(tmp_path: Path) -> None:
    assert make_client(tmp_path, FakeBridge(PLANNED)).post("/api/grasp?tab=nope", json=PLAN_BODY).status_code == 404


def test_api_grasp_stop_returns_result_and_is_allowed_in_cutoff(tmp_path: Path) -> None:
    stop = {"ok": True, "request_id": "rid", "action": "stop", "result": {"arm_held": True, "message": "held"}}
    bridge = FakeBridge(stop)
    resp = make_client(tmp_path, bridge, cutoff_guard()).post("/api/grasp/stop?tab=map")
    assert resp.status_code == 200 and resp.json() == stop
    assert bridge.payloads == [{"action": "stop"}]


def test_api_grasp_stop_503_and_504(tmp_path: Path) -> None:
    assert make_client(tmp_path, FakeBridge(none=True)).post("/api/grasp/stop?tab=map").status_code == 503
    with patch("web_ui.server.GRASP_STOP_TIMEOUT_S", 0.05):
        assert make_client(tmp_path, FakeBridge(never=True)).post("/api/grasp/stop?tab=map").status_code == 504
    assert make_client(tmp_path, None).post("/api/grasp/stop?tab=map").status_code == 503
    assert make_client(tmp_path, FakeBridge()).post("/api/grasp/stop?tab=nope").status_code == 404
