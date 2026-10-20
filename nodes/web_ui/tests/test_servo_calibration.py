"""Tests for the servo torque / neutral routes: config, state cache, GET groups and POST actions."""

from __future__ import annotations

import json
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import patch

import pytest
from fastapi.testclient import TestClient
from pydantic import ValidationError

from web_ui.config import AppConfig, CalibrationConfig, ServoGroupConfig

STATE = {"namespace": "follower", "released": True, "joints": {"a": 0}, "stamp_ns": 1}


class FakeBridge:
    """Stand-in bridge: canned torque samples and Trigger answer."""

    def __init__(self, samples: dict[str, tuple[dict[str, Any], float]] | None = None, **kwargs: Any) -> None:
        self.samples = samples or {}
        self.trigger = kwargs.get("trigger") or SimpleNamespace(success=True, message="done")
        self.no_service = kwargs.get("no_service", False)
        self.services: list[str] = []

    def servo_torque_sample(self, namespace: str) -> tuple[dict[str, Any], float] | None:
        return self.samples.get(namespace)

    def imu_calibration_sample(self) -> None:
        return None

    def trigger_async(self, service: str) -> Future | None:
        self.services.append(service)
        if self.no_service:
            return None
        future: Future = Future()
        future.set_result(self.trigger)
        return future


class FakeGuard:
    def is_cutoff(self) -> bool:
        return True

    def rejection_message(self) -> str:
        return "battery cut-off"


def groups(marker: Path | None = None) -> list[ServoGroupConfig]:
    return [
        ServoGroupConfig(
            id="arm",
            label="Arm",
            namespace="follower",
            neutral=True,
            stale_marker=str(marker) if marker else None,
        ),
        ServoGroupConfig(id="head", label="Head", namespace="head"),
    ]


def make_client(tmp_path: Path, bridge: Any, marker: Path | None = None, guard: Any = None) -> TestClient:
    from web_ui.server import build_app

    config = AppConfig(calibration=CalibrationConfig(servo_groups=groups(marker)))
    return TestClient(
        build_app(
            config=config, urdf_dir=tmp_path, static_dir=tmp_path / "none", bridge_node=bridge, battery_guard=guard
        )
    )


def test_group_ids_unique() -> None:
    with pytest.raises(ValidationError):
        CalibrationConfig(
            servo_groups=[
                ServoGroupConfig(id="arm", label="a", namespace="x"),
                ServoGroupConfig(id="arm", label="b", namespace="y"),
            ]
        )


def test_group_id_and_timeouts_validated() -> None:
    with pytest.raises(ValidationError):
        ServoGroupConfig(id="leg", label="a", namespace="x")
    with pytest.raises(ValidationError):
        ServoGroupConfig(id="arm", label="a", namespace="x", state_stale_s=0)
    with pytest.raises(ValidationError):
        ServoGroupConfig(id="arm", label="a", namespace="x", service_timeout_s=0)


def test_derived_names() -> None:
    group = ServoGroupConfig(id="arm", label="Arm", namespace="follower")
    assert group.state_topic == "/follower/torque_state"
    assert group.service("release") == "/follower/servo/release_torque"
    assert group.service("enable") == "/follower/servo/enable_torque"
    assert group.service("record") == "/follower/servo/record_neutral"
    assert group.service("save") == "/follower/servo/save_neutral"
    config = AppConfig(calibration=CalibrationConfig(servo_groups=groups()))
    assert config.servo_state_topics() == {"follower": "/follower/torque_state", "head": "/head/torque_state"}
    assert "/head/servo/release_torque" in config.trigger_services()
    assert "/follower/servo/save_neutral" in config.trigger_services()
    assert "/head/servo/save_neutral" not in config.trigger_services()


def test_get_fresh_stale_missing_and_marker(tmp_path: Path) -> None:
    marker = tmp_path / "stale.json"
    marker.write_text(json.dumps({"stale": ["joint_offsets", "home"]}))
    bridge = FakeBridge({"follower": (STATE, 100.0), "head": (STATE, 90.0)})
    with patch("web_ui.servo_calibration.time.monotonic", return_value=101.0):
        response = make_client(tmp_path, bridge, marker).get("/api/calibration/servos")
    assert response.status_code == 200
    arm, head = response.json()["groups"]
    assert arm == {
        "id": "arm", "label": "Arm", "neutral": True, "state": STATE, "age_s": 1.0, "stale": False,
        "calibration_stale": ["joint_offsets", "home"],
    }  # fmt: skip
    assert head["stale"] is True and head["age_s"] == 11.0 and head["calibration_stale"] is None
    assert head["neutral"] is False


def test_get_missing_sample_and_marker_absent(tmp_path: Path) -> None:
    response = make_client(tmp_path, FakeBridge(), tmp_path / "none.json").get("/api/calibration/servos")
    arm = response.json()["groups"][0]
    assert arm["state"] is None and arm["age_s"] is None and arm["stale"] is True
    assert arm["calibration_stale"] is None


def test_get_without_bridge(tmp_path: Path) -> None:
    arm = make_client(tmp_path, None).get("/api/calibration/servos").json()["groups"][0]
    assert arm["state"] is None


@pytest.mark.parametrize(
    ("action", "service"),
    [
        ("release", "/follower/servo/release_torque"),
        ("enable", "/follower/servo/enable_torque"),
        ("record", "/follower/servo/record_neutral"),
        ("save", "/follower/servo/save_neutral"),
    ],
)
def test_post_calls_derived_service(tmp_path: Path, action: str, service: str) -> None:
    bridge = FakeBridge()
    response = make_client(tmp_path, bridge).post(f"/api/calibration/servos/arm/{action}")
    assert response.status_code == 200
    assert response.json() == {"ok": True, "message": "done"}
    assert bridge.services == [service]


def test_post_refusal_is_409(tmp_path: Path) -> None:
    bridge = FakeBridge(trigger=SimpleNamespace(success=False, message="no"))
    response = make_client(tmp_path, bridge).post("/api/calibration/servos/arm/release")
    assert response.status_code == 409
    assert response.json() == {"ok": False, "message": "no"}


def test_post_404_paths(tmp_path: Path) -> None:
    client = make_client(tmp_path, FakeBridge())
    assert client.post("/api/calibration/servos/base/release").status_code == 404
    assert client.post("/api/calibration/servos/arm/explode").status_code == 404
    assert client.post("/api/calibration/servos/head/record").status_code == 404
    assert client.post("/api/calibration/servos/head/save").status_code == 404


def test_post_503_paths(tmp_path: Path) -> None:
    assert make_client(tmp_path, None).post("/api/calibration/servos/arm/release").status_code == 503
    bridge = FakeBridge(no_service=True)
    assert make_client(tmp_path, bridge).post("/api/calibration/servos/arm/release").status_code == 503


def test_save_blocked_under_cutoff_release_is_not(tmp_path: Path) -> None:
    bridge = FakeBridge()
    client = make_client(tmp_path, bridge, guard=FakeGuard())
    assert client.post("/api/calibration/servos/arm/save").status_code != 200
    assert bridge.services == []
    assert client.post("/api/calibration/servos/arm/release").status_code == 200
    assert client.post("/api/calibration/servos/arm/enable").status_code == 200


def test_config_endpoint_servos_flag(tmp_path: Path) -> None:
    from web_ui.server import build_app

    def flag(servo_groups: list[ServoGroupConfig]) -> Any:
        config = AppConfig(calibration=CalibrationConfig(servo_groups=servo_groups))
        app = build_app(config=config, urdf_dir=tmp_path, static_dir=tmp_path / "none", bridge_node=FakeBridge())
        return TestClient(app).get("/api/calibration/config").json().get("servos")

    assert flag(groups()) is True
    assert flag([]) is None


def test_bridge_subscribes_and_caches_servo_state() -> None:
    from web_ui.bridge import BridgeNode, String

    created: list[tuple[type, str, Any]] = []
    with (
        patch("web_ui.bridge.Node.__init__", return_value=None),
        patch.object(
            BridgeNode, "create_subscription", create=True, side_effect=lambda c, t, cb, q: created.append((c, t, q))
        ),
        patch.object(BridgeNode, "create_timer", create=True),
        patch.object(BridgeNode, "create_publisher", create=True),
        patch.object(BridgeNode, "get_logger", create=True),
        patch.object(BridgeNode, "create_client", create=True),
    ):
        node = BridgeNode(
            topics=[], allowed_publish_topics=set(), servo_state_topics={"follower": "/follower/torque_state"}
        )
    assert any(c is String and t == "/follower/torque_state" for c, t, _ in created)
    assert node.servo_torque_sample("follower") is None
    node.on_servo_torque_state("follower", SimpleNamespace(data="not json"))
    assert node.servo_torque_sample("follower") is None
    node.on_servo_torque_state("follower", SimpleNamespace(data=json.dumps(STATE)))
    sample = node.servo_torque_sample("follower")
    assert sample is not None and sample[0] == STATE
