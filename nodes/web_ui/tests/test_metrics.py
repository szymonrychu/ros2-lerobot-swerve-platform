"""Prometheus metrics of web_ui: GET /metrics, HTTP middleware, WebSocket, broadcaster, bridge code points."""

from __future__ import annotations

import time
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest
from fastapi.testclient import TestClient
from prometheus_client import REGISTRY
from tf2_ros import TransformException

from web_ui.bridge import BridgeNode
from web_ui.config import AppConfig
from web_ui.throttle import WarnThrottle

from .test_battery import LOW_V, battery_msg, make_guard
from .test_battery import make_bridge as make_battery_bridge
from .test_map_nav import make_bridge, make_transform


def val(name: str, labels: dict[str, str] | None = None) -> float | None:
    """Read a sample from the default registry.

    Args:
        name (str): Sample name.
        labels (dict[str, str] | None): Label set.

    Returns:
        float | None: Current value, None when the sample is absent.
    """
    return REGISTRY.get_sample_value(name, labels or {})


def num(name: str, labels: dict[str, str] | None = None) -> float:
    """Read a sample, 0 when absent.

    Args:
        name (str): Sample name.
        labels (dict[str, str] | None): Label set.

    Returns:
        float: Current value.
    """
    return val(name, labels) or 0.0


@pytest.fixture
def build(tmp_path: Path, urdf_dir: Path) -> Any:
    """Factory building the app over a static dir that holds an index.html (the SPA catch-all is mounted)."""
    from web_ui.server import build_app

    static_dir = tmp_path / "static"
    static_dir.mkdir()
    (static_dir / "index.html").write_text("<html><body>spa</body></html>")

    def factory(bridge: Any = None, hz: float = 10.0) -> Any:
        config = AppConfig(http_port=8080, tabs=[], overlays=[], ws_broadcast_hz=hz)
        return build_app(config=config, urdf_dir=urdf_dir, static_dir=static_dir, bridge_node=bridge)

    return factory


def test_metrics_route_is_not_shadowed_by_the_spa_catch_all(build: Any) -> None:
    """GET /metrics returns Prometheus text, not index.html, although "/" is a static mount."""
    resp = TestClient(build()).get("/metrics")
    assert resp.status_code == 200
    assert resp.headers["content-type"].startswith("text/plain")
    assert "spa" not in resp.text
    assert 'robot_node_info{node="web_ui"} 1.0' in resp.text


def test_requests_are_counted_by_route_template_and_status(build: Any) -> None:
    """The route label is the matched template, never the raw path."""
    client = TestClient(build())
    urdf = {"route": "/api/urdf/{path:path}", "status": "200"}
    tile = {"route": "/api/tiles/{z}/{x}/{y}.png", "status": "404"}
    spa = {"route": "static", "status": "200"}
    before = [num("webui_http_requests_total", labels) for labels in (urdf, tile, spa)]
    client.get("/api/urdf/robot.urdf")
    client.get("/api/tiles/1/2/3.png")
    client.get("/some/spa/route")
    after = [num("webui_http_requests_total", labels) for labels in (urdf, tile, spa)]
    assert [a - b for a, b in zip(after, before, strict=True)] == [1, 1, 1]
    assert val("webui_http_requests_total", {"route": "/api/urdf/robot.urdf", "status": "200"}) is None


def test_ws_clients_gauge_and_disconnects(build: Any) -> None:
    """A WebSocket client raises webui_ws_clients while connected; leaving counts one disconnect."""
    client = TestClient(build())
    disconnects = num("webui_ws_disconnects_total")
    with client.websocket_connect("/ws"):
        assert num("webui_ws_clients") == 1
    assert num("webui_ws_clients") == 0
    assert num("webui_ws_disconnects_total") == disconnects + 1


def test_broadcast_duration_and_slow_cycles(build: Any) -> None:
    """Each broadcast cycle with work is observed; one over the slow threshold also bumps the slow counter."""

    def slow_flush() -> list[dict[str, Any]]:
        time.sleep(0.08)
        return [{"topic": "/live", "data": {"v": 1}}]

    bridge = MagicMock()
    bridge.latest_envelopes.return_value = []
    bridge.flush_dirty.side_effect = slow_flush
    observed, slow = num("webui_broadcast_duration_seconds_count"), num("webui_broadcaster_slow_total")
    with TestClient(build(bridge, hz=200.0)) as client, client.websocket_connect("/ws") as ws:
        ws.receive_json()
        ws.receive_json()
    assert num("webui_broadcast_duration_seconds_count") >= observed + 1
    assert num("webui_broadcaster_slow_total") >= slow + 1


def test_topic_stale_counts_the_transition_once_per_topic(monkeypatch: pytest.MonkeyPatch) -> None:
    """A topic that goes stale is counted once, again only after it recovered and went stale again."""
    import web_ui.bridge as bridge

    clock = {"t": 1000.0}
    monkeypatch.setattr(bridge.time, "monotonic", lambda: clock["t"])
    labels = {"topic": "/metrics_test_topic"}
    before = num("webui_topic_stale_total", labels)
    fake = SimpleNamespace(_topic_last_rx={"/metrics_test_topic": 0.0}, _stale_warn=WarnThrottle(), _stale_topics=set())
    BridgeNode._check_topic_health(fake)
    BridgeNode._check_topic_health(fake)
    assert num("webui_topic_stale_total", labels) == before + 1
    fake._topic_last_rx["/metrics_test_topic"] = clock["t"]
    BridgeNode._check_topic_health(fake)
    fake._topic_last_rx["/metrics_test_topic"] = 0.0
    BridgeNode._check_topic_health(fake)
    assert num("webui_topic_stale_total", labels) == before + 2


def test_map_updates_and_age() -> None:
    """A stored map message bumps webui_map_updates_total and starts webui_map_age_seconds."""
    node = make_bridge()
    callback = node._make_callback("/map", lambda _msg: {"png_b64": "x"}, "map")
    updates = num("webui_map_updates_total")
    callback(object())
    assert num("webui_map_updates_total") == updates + 1
    assert val("webui_map_age_seconds") == pytest.approx(0, abs=1)
    other = node._make_callback("/path", lambda _msg: {"points": []}, "path")
    other(object())
    assert num("webui_map_updates_total") == updates + 1


def test_robot_pose_ok_follows_tf() -> None:
    """webui_robot_pose_ok is 1 with a fresh map->base_link transform and 0 when it is missing."""
    tf_buffer = MagicMock()
    tf_buffer.lookup_transform.return_value = make_transform(1.0, -1.0, 0.3)
    node = make_bridge(_tf_buffer=tf_buffer, _robot_pose_frames=("map", "base_link"))
    node.update_robot_pose()
    assert val("webui_robot_pose_ok") == 1
    tf_buffer.lookup_transform.side_effect = TransformException("no map frame")
    node.update_robot_pose()
    assert val("webui_robot_pose_ok") == 0


def test_battery_cutoff_active_follows_the_guard() -> None:
    """webui_battery_cutoff_active is 1 below the cut-off voltage and 0 above."""
    node = make_battery_bridge(make_guard())
    node.on_battery("/battery_state", battery_msg(LOW_V))
    assert val("webui_battery_cutoff_active") == 1
    node.on_battery("/battery_state", battery_msg(12.5))
    assert val("webui_battery_cutoff_active") == 0
