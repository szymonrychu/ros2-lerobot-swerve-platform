"""Tests for the battery feature: config, serializer, guard hysteresis, bridge callback and server rejection."""

from __future__ import annotations

import math
import threading
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock, patch

import pytest
from fastapi.testclient import TestClient
from pydantic import ValidationError

from web_ui.battery_guard import BatteryGuard
from web_ui.config import AppConfig, BatteryConfig, TabConfig, load_config
from web_ui.msg_serializer import serialize_battery

CELLS = 3
CUTOFF_CELL_V = 2.8
RESUME_CELL_V = 2.9
STALE_S = 5.0
LOW_V = 8.21  # 2.74 V/cell
OK_V = 11.4


class FakeClock:
    """Manually advanced monotonic clock."""

    def __init__(self) -> None:
        self.now = 100.0

    def __call__(self) -> float:
        return self.now


def make_guard(clock: FakeClock | None = None) -> BatteryGuard:
    """Build a guard with the default thresholds and an optional fake clock."""
    return BatteryGuard(
        cells=CELLS,
        cutoff_cell_v=CUTOFF_CELL_V,
        resume_cell_v=RESUME_CELL_V,
        stale_s=STALE_S,
        clock=clock or FakeClock(),
    )


# ---------------------------------------------------------------- config


def test_battery_config_defaults() -> None:
    cfg = BatteryConfig()
    assert (cfg.topic, cfg.cells, cfg.cutoff_cell_v, cfg.resume_cell_v, cfg.stale_s) == (
        "/battery_state",
        3,
        2.8,
        2.9,
        5.0,
    )


def test_app_config_battery_absent_means_off() -> None:
    assert AppConfig().battery is None


def test_battery_config_rejects_resume_below_cutoff() -> None:
    with pytest.raises(ValidationError):
        BatteryConfig(cutoff_cell_v=3.0, resume_cell_v=2.9)


def test_battery_config_allows_equal_thresholds() -> None:
    assert BatteryConfig(cutoff_cell_v=2.8, resume_cell_v=2.8).resume_cell_v == 2.8


@pytest.mark.parametrize("cells", [0, -1])
def test_battery_config_rejects_bad_cells(cells: int) -> None:
    with pytest.raises(ValidationError):
        BatteryConfig(cells=cells)


def test_battery_config_loaded_from_yaml_and_in_api_config(tmp_path: Path, urdf_dir: Path) -> None:
    from web_ui.server import build_app

    p = tmp_path / "c.yaml"
    p.write_text("battery:\n  topic: /pack\n  cells: 4\n")
    cfg = load_config(p)
    assert cfg.battery is not None and cfg.battery.topic == "/pack" and cfg.battery.cells == 4
    body = TestClient(build_app(cfg, urdf_dir, tmp_path / "none")).get("/api/config").json()
    assert body["battery"]["topic"] == "/pack"
    assert body["battery"]["cutoff_cell_v"] == 2.8


def test_battery_topic_is_subscribed_with_battery_role() -> None:
    cfg = AppConfig(battery=BatteryConfig(topic="/pack"))
    assert "/pack" in cfg.all_subscribed_topics()
    assert cfg.topic_roles()["/pack"] == "battery"
    assert "/battery_state" not in AppConfig().all_subscribed_topics()


# ------------------------------------------------------------ serializer


def test_serialize_battery() -> None:
    msg = SimpleNamespace(voltage=11.4, header=SimpleNamespace(stamp=SimpleNamespace(sec=12, nanosec=500_000_000)))
    msg.header.frame_id = "base_link"
    data = serialize_battery(msg, cells=3)
    assert data["voltage"] == pytest.approx(11.4)
    assert data["cells"] == 3
    assert data["cell_voltage"] == pytest.approx(3.8)
    assert data["stamp"] == pytest.approx(12.5)
    assert data["frame_id"] == "base_link"


# ------------------------------------------------------------------ guard


def test_guard_unknown_without_reading() -> None:
    guard = make_guard()
    assert guard.is_cutoff() is False
    assert guard.state()["voltage"] is None


def test_guard_enters_cutoff_below_threshold() -> None:
    guard = make_guard()
    guard.update(8.41)
    assert guard.is_cutoff() is False
    guard.update(8.39)
    assert guard.is_cutoff() is True


def test_guard_hysteresis_leaves_only_above_resume() -> None:
    guard = make_guard()
    guard.update(8.0)
    guard.update(8.6)  # above cut-off (8.4) but below resume (8.7)
    assert guard.is_cutoff() is True
    guard.update(8.7)  # not strictly above
    assert guard.is_cutoff() is True
    guard.update(8.71)
    assert guard.is_cutoff() is False
    guard.update(8.5)  # between thresholds after resume: stays released
    assert guard.is_cutoff() is False


def test_guard_stale_reading_is_not_blocked() -> None:
    clock = FakeClock()
    guard = make_guard(clock)
    guard.update(8.0)
    assert guard.is_cutoff() is True
    clock.now += STALE_S + 0.1
    assert guard.is_cutoff() is False
    assert guard.state()["stale"] is True


def test_guard_ignores_invalid_voltage() -> None:
    guard = make_guard()
    for bad in (math.nan, math.inf, 0.0, -1.0):
        assert guard.update(bad) is False
    assert guard.state()["voltage"] is None
    guard.update(8.0)
    guard.update(math.nan)
    assert guard.state()["voltage"] == 8.0


def test_guard_state_and_rejection_message() -> None:
    guard = make_guard()
    guard.update(LOW_V)
    state = guard.state()
    assert state["cutoff"] is True
    assert state["voltage"] == pytest.approx(LOW_V)
    assert state["cells"] == CELLS
    assert state["cell_voltage"] == pytest.approx(LOW_V / CELLS)
    assert state["cutoff_cell_v"] == CUTOFF_CELL_V and state["resume_cell_v"] == RESUME_CELL_V
    assert state["cutoff_v"] == pytest.approx(8.4) and state["resume_v"] == pytest.approx(8.7)
    assert guard.rejection_message() == ("battery below cut-off: 8.21 V (2.74 V/cell < 2.80 V/cell); commands rejected")


def test_guard_is_thread_safe() -> None:
    guard = make_guard()

    def writer() -> None:
        for i in range(2000):
            guard.update(8.0 if i % 2 else 11.0)

    threads = [threading.Thread(target=writer) for _ in range(4)]
    for t in threads:
        t.start()
    for _ in range(2000):
        guard.is_cutoff()
        guard.state()
    for t in threads:
        t.join()
    assert guard.state()["voltage"] in (8.0, 11.0)


def test_guard_from_config() -> None:
    guard = BatteryGuard.from_config(BatteryConfig(cells=4, cutoff_cell_v=2.5, resume_cell_v=2.6))
    guard.update(9.9)
    assert guard.is_cutoff() is True  # 4 x 2.5 = 10.0


# ----------------------------------------------------------------- bridge


def make_bridge(guard: BatteryGuard | None) -> Any:
    with patch("web_ui.bridge.rclpy"), patch("web_ui.bridge.Node.__init__", return_value=None):
        from web_ui.bridge import BridgeNode

        node = BridgeNode.__new__(BridgeNode)
        node._latest = {}
        node._dirty = set()
        node._cleared = set()
        node._lock = threading.Lock()
        node._topic_last_rx = {}
        node._battery_guard = guard
        return node


def battery_msg(voltage: float) -> SimpleNamespace:
    return SimpleNamespace(
        voltage=voltage, header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0), frame_id="base_link")
    )


def test_bridge_battery_callback_stores_payload_with_guard_state() -> None:
    node = make_bridge(make_guard())
    node.on_battery("/battery_state", battery_msg(LOW_V))
    env = node.flush_dirty()
    assert [e["topic"] for e in env] == ["/battery_state"]
    data = env[0]["data"]
    assert data["voltage"] == pytest.approx(LOW_V)
    assert data["cutoff"] is True
    assert data["cell_voltage"] == pytest.approx(LOW_V / CELLS)


def test_bridge_battery_callback_drops_invalid_voltage() -> None:
    node = make_bridge(make_guard())
    node.on_battery("/battery_state", battery_msg(math.nan))
    assert node.flush_dirty() == []


# ----------------------------------------------------------------- server


@pytest.fixture
def make_client(tmp_path: Path, urdf_dir: Path) -> Any:
    from web_ui.server import build_app

    config = AppConfig(
        tabs=[TabConfig(id="map", type="map_nav", label="Map", goal_topic="/goal", navigate_action="/nav")],
        battery=BatteryConfig(),
    )

    def factory(guard: BatteryGuard | None, bridge: Any = None) -> TestClient:
        app = build_app(
            config=config, urdf_dir=urdf_dir, static_dir=tmp_path / "none", bridge_node=bridge, battery_guard=guard
        )
        return TestClient(app)

    return factory


def low_guard() -> BatteryGuard:
    guard = make_guard()
    guard.update(LOW_V)
    return guard


PUBLISH = {"type": "publish", "topic": "/goal", "msg_type": "geometry_msgs/PoseStamped", "data": {}}


def test_ws_publish_rejected_in_cutoff(make_client: Any) -> None:
    bridge = MagicMock()
    with make_client(low_guard(), bridge).websocket_connect("/ws") as ws:
        ws.send_text(__import__("json").dumps(PUBLISH))
        frame = ws.receive_json()
    assert frame == {
        "type": "error",
        "source": "battery",
        "message": "battery below cut-off: 8.21 V (2.74 V/cell < 2.80 V/cell); commands rejected",
    }
    bridge.create_publisher_for.assert_not_called()
    bridge.publish_dict.assert_not_called()


@pytest.mark.parametrize("guard_factory", [lambda: None, make_guard])
def test_ws_publish_accepted_without_cutoff(make_client: Any, guard_factory: Any) -> None:
    bridge = MagicMock()
    guard = guard_factory()
    if guard is not None:
        guard.update(OK_V)
    with make_client(guard, bridge).websocket_connect("/ws") as ws:
        ws.send_text(__import__("json").dumps(PUBLISH))
        ws.send_text("{}")  # round trip: both frames processed in order
    bridge.create_publisher_for.assert_called_once_with("/goal", "geometry_msgs/PoseStamped")
    bridge.publish_dict.assert_called_once_with("/goal", {})


@pytest.mark.parametrize(
    "path",
    ["/api/map/save?tab=map", "/api/map/reset?tab=map", "/api/arm/home?tab=map", "/api/arm/set_home?tab=map"],
)
def test_post_actions_rejected_with_503_in_cutoff(make_client: Any, path: str) -> None:
    bridge = MagicMock()
    resp = make_client(low_guard(), bridge).post(path)
    assert resp.status_code == 503
    body = resp.json()
    assert body["ok"] is False
    assert body["message"].startswith("battery below cut-off: 8.21 V")
    assert bridge.method_calls == []


def test_nav_stop_allowed_in_cutoff(make_client: Any) -> None:
    bridge = MagicMock()
    bridge.cancel_all_goals_async.return_value = None  # service unavailable -> 503 from the service, not the battery
    resp = make_client(low_guard(), bridge).post("/api/nav/stop?tab=map")
    bridge.cancel_all_goals_async.assert_called_once_with("/nav")
    assert "battery" not in resp.json()["message"]


def test_post_actions_not_blocked_without_battery_config(make_client: Any) -> None:
    bridge = MagicMock()
    bridge.serialize_map_async.return_value = None
    resp = make_client(None, bridge).post("/api/map/save?tab=map")
    assert "battery" not in resp.json()["message"]
    bridge.serialize_map_async.assert_called_once()


def test_post_actions_not_blocked_when_battery_ok_or_unknown(make_client: Any) -> None:
    bridge = MagicMock()
    bridge.serialize_map_async.return_value = None
    unknown = make_guard()
    ok = make_guard()
    ok.update(OK_V)
    for guard in (unknown, ok):
        resp = make_client(guard, bridge).post("/api/map/save?tab=map")
        assert "battery" not in resp.json()["message"]
