"""Head command backend: config fields, publish allowlist, frame_id default and the WS range refusal."""

from __future__ import annotations

import json
import math
import signal
from pathlib import Path
from typing import Any
from unittest.mock import MagicMock

import pytest
from fastapi.testclient import TestClient
from pydantic import ValidationError
from ros2_common.battery import BatteryConfig, BatteryGuard
from sensor_msgs.msg import JointState

from web_ui.bridge import TOPIC_TYPE_HINTS
from web_ui.config import AppConfig, TabConfig

CMD = "/head/joint_commands"
MSG_TYPE = "sensor_msgs/JointState"
WS_REPLY_TIMEOUT_S = 15


def head_tab(**kw: Any) -> TabConfig:
    return TabConfig(id="head", type="head", label="Head", **kw)


def test_head_tab_defaults() -> None:
    tab = head_tab()
    assert tab.head_command_topic == CMD
    assert tab.head_joint_states_topic == "/head/joint_states"
    assert tab.head_pan_range_deg == (-90.0, 90.0)
    assert tab.head_tilt_range_deg == (-30.0, 45.0)
    assert tab.head_home_deg == (0.0, 30.0)


def test_head_tab_overrides() -> None:
    tab = head_tab(head_pan_range_deg=(-45, 45), head_tilt_range_deg=(0, 20), head_home_deg=(10, 5))
    assert tab.head_pan_range_deg == (-45.0, 45.0)
    assert tab.head_home_deg == (10.0, 5.0)


@pytest.mark.parametrize(
    "kw",
    [
        {"head_pan_range_deg": (10, -10)},
        {"head_tilt_range_deg": (5, 5)},
        {"head_home_deg": (120, 30)},
        {"head_home_deg": (0, 60)},
        {"head_command_topic": " "},
    ],
)
def test_head_tab_validation(kw: dict[str, Any]) -> None:
    with pytest.raises(ValidationError):
        head_tab(**kw)


def test_other_tabs_have_no_head_command_fields() -> None:
    tab = TabConfig(id="arm", type="arm", label="Arm")
    assert tab.head_command_topic is None
    assert tab.head_pan_range_deg is None


def test_publish_topics_and_frame_id_defaults() -> None:
    config = AppConfig(tabs=[head_tab()])
    assert CMD in config.publish_topics()
    assert config.frame_id_defaults()[CMD] == "web_ui"


def test_map_nav_head_fields() -> None:
    tab = TabConfig(id="map", type="map_nav", label="Map")
    assert tab.head_urdf == "head.urdf"
    assert tab.head_joint_states_topic == "/head/joint_states"
    assert "/head/joint_states" in AppConfig(tabs=[tab]).all_subscribed_topics()


def test_head_joint_states_subscription_type_hint() -> None:
    assert TOPIC_TYPE_HINTS["/head/joint_states"] is JointState
    assert "/head/joint_states" in AppConfig(tabs=[head_tab()]).all_subscribed_topics()


def make_guard(voltage: float | None) -> BatteryGuard | None:
    if voltage is None:
        return None
    guard = BatteryGuard(cells=3, cutoff_cell_v=2.8, resume_cell_v=3.0, stale_s=10.0)
    guard.update(voltage)
    return guard


@pytest.fixture(autouse=True)
def fail_instead_of_hanging() -> Any:
    """A missing WS reply blocks receive_json forever; turn that into a test failure."""

    def on_alarm(_signum: int, _frame: Any) -> None:
        raise TimeoutError("no WebSocket reply within WS_REPLY_TIMEOUT_S")

    previous = signal.signal(signal.SIGALRM, on_alarm)
    signal.alarm(WS_REPLY_TIMEOUT_S)
    yield
    signal.alarm(0)
    signal.signal(signal.SIGALRM, previous)


@pytest.fixture
def make_client(tmp_path: Path, urdf_dir: Path) -> Any:
    from web_ui.server import build_app

    config = AppConfig(tabs=[head_tab()], battery=BatteryConfig())

    def factory(bridge: Any, guard: BatteryGuard | None = None) -> TestClient:
        app = build_app(
            config=config, urdf_dir=urdf_dir, static_dir=tmp_path / "none", bridge_node=bridge, battery_guard=guard
        )
        return TestClient(app)

    return factory


def send(client: TestClient, pan: Any, tilt: Any, topic: str = CMD) -> dict[str, Any] | None:
    data = {"name": ["head_pan", "head_tilt"], "position": [pan, tilt]}
    frame = {"type": "publish", "topic": topic, "msg_type": MSG_TYPE, "data": data}
    with client.websocket_connect("/ws") as ws:
        ws.send_text(json.dumps(frame, allow_nan=True))
        ws.send_text("{}")  # round trip: both frames processed in order
        return None


def error_of(client: TestClient, pan: Any, tilt: Any) -> dict[str, Any]:
    data = {"name": ["head_pan", "head_tilt"], "position": [pan, tilt]}
    frame = {"type": "publish", "topic": CMD, "msg_type": MSG_TYPE, "data": data}
    with client.websocket_connect("/ws") as ws:
        ws.send_text(json.dumps(frame, allow_nan=True))
        return ws.receive_json()


def test_in_range_command_published(make_client: Any) -> None:
    bridge = MagicMock()
    send(make_client(bridge), math.radians(90), math.radians(-30))
    bridge.create_publisher_for.assert_called_once_with(CMD, MSG_TYPE)
    bridge.publish_dict.assert_called_once()
    assert bridge.publish_dict.call_args.args[0] == CMD


@pytest.mark.parametrize(
    ("pan", "tilt"),
    [
        (math.radians(91), 0.0),
        (0.0, math.radians(46)),
        (0.0, math.radians(-31)),
        (float("nan"), 0.0),
        (0.0, float("inf")),
        ("x", 0.0),
    ],
)
def test_out_of_range_or_non_finite_refused(make_client: Any, pan: Any, tilt: Any) -> None:
    bridge = MagicMock()
    frame = error_of(make_client(bridge), pan, tilt)
    assert frame["type"] == "error"
    assert frame["source"] == "head"
    assert frame["message"]
    bridge.create_publisher_for.assert_not_called()
    bridge.publish_dict.assert_not_called()


def test_unknown_joint_name_refused(make_client: Any) -> None:
    bridge = MagicMock()
    client = make_client(bridge)
    frame = {"type": "publish", "topic": CMD, "msg_type": MSG_TYPE, "data": {"name": ["x"], "position": [0.0]}}
    with client.websocket_connect("/ws") as ws:
        ws.send_text(json.dumps(frame))
        assert ws.receive_json()["source"] == "head"
    bridge.publish_dict.assert_not_called()


def test_battery_cutoff_refuses_first(make_client: Any) -> None:
    bridge = MagicMock()
    frame = error_of(make_client(bridge, make_guard(8.0)), 99.0, 0.0)
    assert frame["source"] == "battery"
    bridge.publish_dict.assert_not_called()


def test_head_command_not_refused_for_lease(make_client: Any) -> None:
    """R1: no lease check on the web UI path; a valid command is published."""
    bridge = MagicMock()
    send(make_client(bridge), 0.0, 0.5)
    bridge.publish_dict.assert_called_once()
