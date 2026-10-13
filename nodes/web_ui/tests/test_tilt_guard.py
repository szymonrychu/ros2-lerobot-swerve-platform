"""Tests for the tilt guard feature: state parsing, bridge subscription/callback and nav goal refusal."""

from __future__ import annotations

import json
import threading
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock, patch

import pytest
from fastapi.testclient import TestClient
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

from web_ui.config import AppConfig, TabConfig
from web_ui.tilt_guard import (
    TILT_GUARD_STATE_TOPIC,
    nav_rejection_message,
    parse_tilt_state,
)

WARNING = {"level": "warning", "roll_deg": 12.5, "pitch_deg": -3.0}
ERROR = {"level": "error", "roll_deg": 25.0, "pitch_deg": 4.0}
OK = {"level": "ok", "roll_deg": 0.5, "pitch_deg": 0.2}


def test_topic_name() -> None:
    assert TILT_GUARD_STATE_TOPIC == "/tilt_guard/state"


def test_parse_valid_state() -> None:
    assert parse_tilt_state(json.dumps(WARNING)) == WARNING


@pytest.mark.parametrize(
    "raw",
    ["not json", "[1]", "null", "", '{"level": "bogus"}', '{"roll_deg": 1}', '{"level": "warning", "roll_deg": "x"}'],
)
def test_parse_rejects_invalid_state(raw: str) -> None:
    assert parse_tilt_state(raw) is None


def test_nav_rejection_message_by_level() -> None:
    assert nav_rejection_message(None) is None
    assert nav_rejection_message(OK) is None
    warning = nav_rejection_message(WARNING)
    assert warning is not None
    assert "Tilt warning" in warning
    assert "roll 12.5" in warning
    assert "pitch -3.0" in warning
    assert "automatic navigation disabled" in warning
    assert "drive back manually" in warning
    error = nav_rejection_message(ERROR)
    assert error is not None
    assert "Tilt error" in error
    assert "base drive disabled until the robot is level" in error


def test_nav_rejection_warning_with_base_drive_inhibited() -> None:
    state = {**WARNING, "base_drive_inhibited": True}
    message = nav_rejection_message(state)
    assert message is not None
    assert "base drive still inhibited" in message
    assert "wait until the robot is level" in message
    assert "drive back manually" not in message


def test_nav_rejection_when_imu_not_ok_at_ok_level() -> None:
    message = nav_rejection_message({**OK, "imu_ok": False})
    assert message is not None
    assert "IMU" in message
    assert nav_rejection_message({**OK, "imu_ok": True}) is None


def test_nav_rejection_when_state_is_stale() -> None:
    message = nav_rejection_message({**OK, "stale": True})
    assert message is not None
    assert "stale" in message
    assert nav_rejection_message({**OK, "stale": False}) is None


# ----------------------------------------------------------------- bridge


def make_bridge() -> Any:
    with patch("web_ui.bridge.rclpy"), patch("web_ui.bridge.Node.__init__", return_value=None):
        from web_ui.bridge import BridgeNode

        node = BridgeNode.__new__(BridgeNode)
        node._latest = {}
        node._dirty = set()
        node._pending = {}
        node._color_small = False
        node._cleared = set()
        node._lock = threading.Lock()
        node._topic_last_rx = {}
        return node


def test_bridge_tilt_state_stores_and_broadcasts() -> None:
    node = make_bridge()
    node.on_tilt_guard_state(SimpleNamespace(data=json.dumps(WARNING)))
    assert node.flush_dirty() == [{"topic": TILT_GUARD_STATE_TOPIC, "data": WARNING}]
    assert node.tilt_state() == WARNING


def test_bridge_tilt_state_drops_invalid_json() -> None:
    node = make_bridge()
    node.on_tilt_guard_state(SimpleNamespace(data="{oops"))
    assert node.flush_dirty() == []
    assert node.tilt_state() is None


def test_bridge_subscribes_reliable_transient_local_and_tracks_staleness() -> None:
    with patch("web_ui.bridge.rclpy"), patch("web_ui.bridge.Node.__init__", return_value=None):
        from web_ui.bridge import BridgeNode

        node = BridgeNode.__new__(BridgeNode)
        node._topic_last_rx = {}
        node.create_subscription = MagicMock()
        node.subscribe_tilt_guard()
    args = node.create_subscription.call_args.args
    assert args[1] == TILT_GUARD_STATE_TOPIC
    assert args[2] == node.on_tilt_guard_state
    assert args[3].reliability is ReliabilityPolicy.RELIABLE
    assert args[3].durability is DurabilityPolicy.TRANSIENT_LOCAL
    assert TILT_GUARD_STATE_TOPIC in node._topic_last_rx


def test_bridge_marks_state_stale_after_the_limit_and_broadcasts_it() -> None:
    node = make_bridge()
    node._tilt_stale_s = 5.0
    with patch("web_ui.bridge.time.monotonic", return_value=100.0):
        node.on_tilt_guard_state(SimpleNamespace(data=json.dumps(OK)))
        node.flush_dirty()
        assert "stale" not in node.tilt_state()
    with patch("web_ui.bridge.time.monotonic", return_value=104.0):
        assert node.flush_dirty() == []
    with patch("web_ui.bridge.time.monotonic", return_value=106.0):
        assert node.tilt_state() == {**OK, "stale": True}
        assert node.flush_dirty() == [{"topic": TILT_GUARD_STATE_TOPIC, "data": {**OK, "stale": True}}]
        assert node.flush_dirty() == []


def test_bridge_fresh_message_clears_stale() -> None:
    node = make_bridge()
    node._tilt_stale_s = 5.0
    with patch("web_ui.bridge.time.monotonic", return_value=100.0):
        node.on_tilt_guard_state(SimpleNamespace(data=json.dumps(OK)))
    with patch("web_ui.bridge.time.monotonic", return_value=110.0):
        assert node.tilt_state()["stale"] is True
        node.on_tilt_guard_state(SimpleNamespace(data=json.dumps(OK)))
        assert node.tilt_state() == OK


def test_config_tilt_guard_stale_s_default_and_validation() -> None:
    from pydantic import ValidationError

    assert AppConfig().tilt_guard_stale_s == 5.0
    assert AppConfig(tilt_guard_stale_s=3).tilt_guard_stale_s == 3
    with pytest.raises(ValidationError):
        AppConfig(tilt_guard_stale_s=0)


# ----------------------------------------------------------------- server

GOAL = {"type": "publish", "topic": "/goal", "msg_type": "geometry_msgs/PoseStamped", "data": {}}
ARM = {"type": "publish", "topic": "/arm", "msg_type": "sensor_msgs/JointState", "data": {}}


@pytest.fixture
def make_client(tmp_path: Path, urdf_dir: Path) -> Any:
    from web_ui.server import build_app

    config = AppConfig(
        tabs=[TabConfig(id="map", type="map_nav", label="Map", goal_topic="/goal", navigate_action="/nav")],
    )

    def factory(state: dict[str, Any] | None) -> tuple[TestClient, Any]:
        bridge = MagicMock()
        bridge.tilt_state.return_value = state
        app = build_app(config=config, urdf_dir=urdf_dir, static_dir=tmp_path / "none", bridge_node=bridge)
        return TestClient(app), bridge

    return factory


@pytest.mark.parametrize("state", [WARNING, ERROR])
def test_ws_goal_publish_refused_with_message(make_client: Any, state: dict[str, Any]) -> None:
    client, bridge = make_client(state)
    with client.websocket_connect("/ws") as ws:
        ws.send_text(json.dumps(GOAL))
        frame = ws.receive_json()
    assert frame["type"] == "error"
    assert frame["source"] == "tilt_guard"
    assert frame["message"] == nav_rejection_message(state)
    bridge.publish_dict.assert_not_called()


@pytest.mark.parametrize("state", [None, OK])
def test_ws_goal_publish_allowed_when_level(make_client: Any, state: dict[str, Any] | None) -> None:
    client, bridge = make_client(state)
    with client.websocket_connect("/ws") as ws:
        ws.send_text(json.dumps(GOAL))
        ws.send_text("{}")
    bridge.publish_dict.assert_called_once_with("/goal", {})


def test_ws_non_goal_publish_allowed_during_error(make_client: Any) -> None:
    client, bridge = make_client(ERROR)
    with client.websocket_connect("/ws") as ws:
        ws.send_text(json.dumps(ARM))
        ws.send_text("{}")
    bridge.publish_dict.assert_called_once_with("/arm", {})


@pytest.mark.parametrize(
    "state",
    [{**OK, "stale": True}, {**OK, "imu_ok": False}],
    ids=["state older than the limit", "imu_ok false at ok"],
)
def test_ws_goal_publish_refused_when_state_stale_or_imu_dead(make_client: Any, state: dict[str, Any]) -> None:
    client, bridge = make_client(state)
    with client.websocket_connect("/ws") as ws:
        ws.send_text(json.dumps(GOAL))
        frame = ws.receive_json()
    assert frame["source"] == "tilt_guard"
    assert frame["message"] == nav_rejection_message(state)
    bridge.publish_dict.assert_not_called()
