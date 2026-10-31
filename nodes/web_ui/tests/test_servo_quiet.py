"""Tests for quieting a released servo group: WS publishes and arm routes refused while torque is off."""

from __future__ import annotations

import json
import signal
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock, patch

import pytest
from fastapi.testclient import TestClient

from web_ui.config import AppConfig, CalibrationConfig, ServoGroupConfig, TabConfig
from web_ui.servo_calibration import released_command_error

ARM_TOPIC = "/filter/web_ui_joint_commands"
HEAD_TOPIC = "/head/joint_commands"
GOAL_TOPIC = "/goal_pose"
NOW = 100.0
RELEASED = {"namespace": "follower", "released": True, "joints": {}, "stamp_ns": 1}
ENABLED = {**RELEASED, "released": False}


def make_config() -> AppConfig:
    groups = [
        ServoGroupConfig(id="arm", label="Arm", namespace="follower", command_topics=[ARM_TOPIC]),
        ServoGroupConfig(id="head", label="Head", namespace="head", command_topics=[HEAD_TOPIC]),
        ServoGroupConfig(id="base", label="Base", namespace="swerve", command_topics=[GOAL_TOPIC]),
    ]
    return AppConfig(
        tabs=[
            TabConfig(id="map", type="map_nav", label="Map", goal_topic=GOAL_TOPIC),
            TabConfig(id="arm", type="arm", label="Arm", arm_command_topic=ARM_TOPIC),
        ],
        calibration=CalibrationConfig(servo_groups=groups),
    )


class FakeBridge(MagicMock):
    """MagicMock bridge with canned torque samples and Trigger / grasp answers."""

    def setup(self, samples: dict[str, tuple[dict[str, Any], float]]) -> FakeBridge:
        self.servo_torque_sample.side_effect = samples.get
        self.tilt_state.return_value = None
        done: Future = Future()
        done.set_result(SimpleNamespace(success=True, message="done"))
        self.trigger_async.return_value = done
        grasp: Future = Future()
        grasp.set_result({"ok": True})
        self.grasp_request_async.return_value = ("rid", grasp)
        return self


def bridge_with(**states: dict[str, Any] | None) -> FakeBridge:
    samples = {ns: (state, NOW - 1.0) for ns, state in states.items() if state is not None}
    return FakeBridge().setup(samples)


def make_client(tmp_path: Path, bridge: Any) -> TestClient:
    from web_ui.server import build_app

    return TestClient(
        build_app(config=make_config(), urdf_dir=tmp_path, static_dir=tmp_path / "none", bridge_node=bridge)
    )


def publish(client: TestClient, topic: str) -> dict[str, Any] | None:
    frame = {"type": "publish", "topic": topic, "msg_type": "sensor_msgs/JointState", "data": {}}
    with client.websocket_connect("/ws") as ws:
        ws.send_text(json.dumps(frame))
        ws.send_text("{}")
    return None


def error_of(client: TestClient, topic: str) -> dict[str, Any]:
    frame = {"type": "publish", "topic": topic, "msg_type": "sensor_msgs/JointState", "data": {}}
    with client.websocket_connect("/ws") as ws:
        ws.send_text(json.dumps(frame))
        return ws.receive_json()


@pytest.fixture(autouse=True)
def fail_instead_of_hanging() -> Any:
    """A missing WS reply blocks receive_json forever; turn that into a test failure."""

    def on_alarm(_signum: int, _frame: Any) -> None:
        raise TimeoutError("no WebSocket reply within 10 s")

    previous = signal.signal(signal.SIGALRM, on_alarm)
    signal.alarm(10)
    yield
    signal.alarm(0)
    signal.signal(signal.SIGALRM, previous)


@pytest.fixture(autouse=True)
def fixed_clock() -> Any:
    with patch("web_ui.servo_calibration.time.monotonic", return_value=NOW):
        yield


def test_released_command_error_cases() -> None:
    config = make_config()
    assert released_command_error(config, bridge_with(follower=RELEASED), ARM_TOPIC) == (
        "Arm servo torque is off (Calibrate page)"
    )
    assert released_command_error(config, bridge_with(follower=ENABLED), ARM_TOPIC) is None
    assert released_command_error(config, bridge_with(), ARM_TOPIC) is None
    assert released_command_error(config, None, ARM_TOPIC) is None
    assert released_command_error(config, bridge_with(follower=RELEASED), "/other") is None
    stale = FakeBridge().setup({"follower": (RELEASED, NOW - 60.0)})
    assert released_command_error(config, stale, ARM_TOPIC) is None
    assert released_command_error(AppConfig(), bridge_with(follower=RELEASED), ARM_TOPIC) is None


@pytest.mark.parametrize(("topic", "ns"), [(ARM_TOPIC, "follower"), (HEAD_TOPIC, "head"), (GOAL_TOPIC, "swerve")])
def test_ws_publish_rejected_while_released(tmp_path: Path, topic: str, ns: str) -> None:
    bridge = bridge_with(**{ns: RELEASED})
    frame = error_of(make_client(tmp_path, bridge), topic)
    assert frame["type"] == "error"
    assert frame["source"] == "head"
    assert "torque is off" in frame["message"]
    bridge.publish_dict.assert_not_called()


@pytest.mark.parametrize("state", [ENABLED, None])
def test_ws_publish_allowed_when_enabled_or_unknown(tmp_path: Path, state: dict[str, Any] | None) -> None:
    bridge = bridge_with(follower=state, head=state, swerve=state)
    for topic in (ARM_TOPIC, HEAD_TOPIC, GOAL_TOPIC):  # one client each: a TestClient serves one WS session
        publish(make_client(tmp_path, bridge), topic)
    assert [c.args[0] for c in bridge.publish_dict.call_args_list] == [ARM_TOPIC, HEAD_TOPIC, GOAL_TOPIC]


def test_ws_publish_allowed_when_stale(tmp_path: Path) -> None:
    bridge = FakeBridge().setup({"head": (RELEASED, NOW - 60.0)})
    publish(make_client(tmp_path, bridge), HEAD_TOPIC)
    bridge.publish_dict.assert_called_once()


@pytest.mark.parametrize(
    ("path", "body"),
    [
        ("/api/arm/home?tab=arm", None),
        ("/api/arm/take_control?tab=arm", None),
        ("/api/arm/jog?tab=arm", {"kind": "cartesian", "direction": "up"}),
        ("/api/grasp?tab=map", {"action": "release"}),
        ("/api/grasp?tab=arm", {"action": "release"}),
    ],
)
def test_arm_routes_409_while_released(tmp_path: Path, path: str, body: Any) -> None:
    bridge = bridge_with(follower=RELEASED)
    response = make_client(tmp_path, bridge).post(path, json=body)
    assert response.status_code == 409
    assert response.json() == {"ok": False, "message": "Arm servo torque is off (Calibrate page)"}
    bridge.trigger_async.assert_not_called()
    bridge.grasp_request_async.assert_not_called()


def test_arm_routes_allowed_when_enabled(tmp_path: Path) -> None:
    bridge = bridge_with(follower=ENABLED)
    assert make_client(tmp_path, bridge).post("/api/arm/home?tab=arm").status_code == 200


def test_release_and_stop_allowed_while_released(tmp_path: Path) -> None:
    bridge = bridge_with(follower=RELEASED)
    client = make_client(tmp_path, bridge)
    assert client.post("/api/arm/release?tab=arm").status_code == 200
    assert client.post("/api/arm/set_home?tab=arm").status_code == 200
    assert client.post("/api/grasp/stop?tab=map").status_code == 200
