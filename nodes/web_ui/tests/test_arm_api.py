"""Tests for the Arm tab backend: jog / take control / release / home routes, config roles, bridge active source."""

from __future__ import annotations

from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock, patch

import pytest
from fastapi.testclient import TestClient

from web_ui.bridge import GRASP_MOTION_ACTIONS, ROLE_SPECS, String
from web_ui.config import AppConfig, TabConfig

JOG = {"kind": "cartesian", "direction": "up", "step": 0.01}
JOG_DONE = {"ok": True, "request_id": "rid", "action": "jog", "result": {"kind": "cartesian", "status": "converged"}}
JOG_REFUSED = {"ok": False, "request_id": "rid", "action": "jog", "error": "jog refused: agent holds the arm"}


class FakeBridge:
    """Stand-in bridge: canned grasp answer and Trigger answer."""

    def __init__(
        self,
        result: dict[str, Any] | None = None,
        trigger: SimpleNamespace | None = None,
        none: bool = False,
        never: bool = False,
        no_service: bool = False,
    ) -> None:
        self.result = result
        self.trigger = trigger or SimpleNamespace(success=True, message="done")
        self.none = none
        self.never = never
        self.no_service = no_service
        self.payloads: list[dict[str, Any]] = []
        self.services: list[str] = []

    def grasp_request_async(self, payload: dict[str, Any]) -> tuple[str, Future] | None:
        self.payloads.append(payload)
        if self.none:
            return None
        future: Future = Future()
        if not self.never:
            future.set_result(self.result)
        return "rid", future

    def trigger_async(self, service: str) -> Future | None:
        self.services.append(service)
        if self.no_service:
            return None
        future: Future = Future()
        future.set_result(self.trigger)
        return future


def make_client(tmp_path: Path, bridge: Any, battery_guard: Any = None) -> TestClient:
    from web_ui.server import build_app

    config = AppConfig(
        tabs=[
            TabConfig(id="map", type="map_nav", label="Map"),
            TabConfig(id="arm", type="arm", label="Arm"),
            TabConfig(id="cam", type="camera", label="Cam", topic="/cam"),
        ]
    )
    app = build_app(
        config=config, urdf_dir=tmp_path, static_dir=tmp_path / "none", bridge_node=bridge, battery_guard=battery_guard
    )
    return TestClient(app)


def cutoff_guard() -> Any:
    guard = MagicMock()
    guard.is_cutoff.return_value = True
    guard.rejection_message.return_value = "battery below cut-off"
    return guard


def test_arm_tab_roles_and_topics() -> None:
    cfg = AppConfig(tabs=[TabConfig(id="arm", type="arm", label="Arm")])
    assert cfg.topic_roles()["/filter/active_source"] == "active_source"
    assert cfg.topic_roles()["/grasp/result"] == "grasp_result"
    assert "/filter/active_source" in cfg.all_subscribed_topics()
    assert cfg.grasp_command_topic() == "/grasp/command"
    assert {"/arm/home", "/arm/take_control", "/arm/release_control"} <= set(cfg.trigger_services())


def test_bridge_jog_is_motion_and_active_source_role_serializes_string() -> None:
    assert "jog" in GRASP_MOTION_ACTIONS
    msg_cls, _qos, serializer = ROLE_SPECS["active_source"]
    assert serializer(SimpleNamespace(data="leader")) == {"data": "leader"}
    assert msg_cls is String


def test_jog_forwards_request_and_returns_answer(tmp_path: Path) -> None:
    bridge = FakeBridge(JOG_DONE)
    resp = make_client(tmp_path, bridge).post("/api/arm/jog?tab=arm", json=JOG)
    assert resp.status_code == 200 and resp.json() == JOG_DONE
    assert bridge.payloads == [{"action": "jog", "jog": JOG}]


def test_jog_refusal_is_400(tmp_path: Path) -> None:
    resp = make_client(tmp_path, FakeBridge(JOG_REFUSED)).post("/api/arm/jog?tab=arm", json=JOG)
    assert resp.status_code == 400 and resp.json()["error"].startswith("jog refused")


@pytest.mark.parametrize("body", [{}, [], {"kind": 3}, {"direction": "up"}, {"kind": "cartesian"}])
def test_jog_bad_body_is_422(tmp_path: Path, body: Any) -> None:
    bridge = FakeBridge(JOG_DONE)
    assert make_client(tmp_path, bridge).post("/api/arm/jog?tab=arm", json=body).status_code == 422
    assert bridge.payloads == []


def test_jog_blocked_in_cutoff(tmp_path: Path) -> None:
    bridge = FakeBridge(JOG_DONE)
    resp = make_client(tmp_path, bridge, cutoff_guard()).post("/api/arm/jog?tab=arm", json=JOG)
    assert resp.status_code == 503 and "battery" in resp.json()["message"]
    assert bridge.payloads == []


@pytest.mark.parametrize("tab", ["nope", "map", "cam"])
def test_routes_404_for_unknown_or_wrong_type_tab(tmp_path: Path, tab: str) -> None:
    client = make_client(tmp_path, FakeBridge(JOG_DONE))
    assert client.post(f"/api/arm/jog?tab={tab}", json=JOG).status_code == 404
    for route in ("take_control", "release"):
        assert client.post(f"/api/arm/{route}?tab={tab}").status_code == 404


def test_jog_503_without_bridge_or_mcp_server_and_504(tmp_path: Path) -> None:
    assert make_client(tmp_path, None).post("/api/arm/jog?tab=arm", json=JOG).status_code == 503
    assert make_client(tmp_path, FakeBridge(none=True)).post("/api/arm/jog?tab=arm", json=JOG).status_code == 503
    with patch("web_ui.server.JOG_TIMEOUT_S", 0.05):
        assert make_client(tmp_path, FakeBridge(never=True)).post("/api/arm/jog?tab=arm", json=JOG).status_code == 504


@pytest.mark.parametrize(
    ("route", "service"), [("take_control", "/arm/take_control"), ("release", "/arm/release_control")]
)
def test_control_routes_call_trigger(tmp_path: Path, route: str, service: str) -> None:
    bridge = FakeBridge(trigger=SimpleNamespace(success=True, message="lease taken"))
    resp = make_client(tmp_path, bridge).post(f"/api/arm/{route}?tab=arm")
    assert resp.status_code == 200 and resp.json()["ok"] is True and resp.json()["message"] == "lease taken"
    assert bridge.services == [service]


def test_control_route_failure_is_500(tmp_path: Path) -> None:
    bridge = FakeBridge(trigger=SimpleNamespace(success=False, message="agent holds the arm"))
    resp = make_client(tmp_path, bridge).post("/api/arm/take_control?tab=arm")
    assert resp.status_code == 500 and resp.json()["message"] == "agent holds the arm"


@pytest.mark.parametrize("route", ["take_control", "release", "home"])
def test_control_routes_503_without_bridge_service_and_in_cutoff(tmp_path: Path, route: str) -> None:
    assert make_client(tmp_path, None).post(f"/api/arm/{route}?tab=arm").status_code == 503
    assert make_client(tmp_path, FakeBridge(no_service=True)).post(f"/api/arm/{route}?tab=arm").status_code == 503
    bridge = FakeBridge()
    assert make_client(tmp_path, bridge, cutoff_guard()).post(f"/api/arm/{route}?tab=arm").status_code == 503
    assert bridge.services == []


def test_home_accepts_arm_and_map_nav_tabs(tmp_path: Path) -> None:
    bridge = FakeBridge()
    client = make_client(tmp_path, bridge)
    assert client.post("/api/arm/home?tab=arm").status_code == 200
    assert client.post("/api/arm/home?tab=map").status_code == 200
    assert client.post("/api/arm/home?tab=cam").status_code == 404
    assert bridge.services == ["/arm/home", "/arm/home"]
