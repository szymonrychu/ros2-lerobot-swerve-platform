"""Tests for the IMU calibration routes: status from the bridge cache, save / reset through Trigger services."""

from __future__ import annotations

import json
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import patch

from fastapi.testclient import TestClient

from web_ui.config import AppConfig, CalibrationConfig

SAVE = "/imu/calibration/save"
RESET = "/imu/calibration/reset"
SAMPLE = {"sys": 3, "gyro": 3, "accel": 2, "mag": 3, "restored": False}


class FakeBridge:
    """Stand-in bridge: canned IMU sample and Trigger answer."""

    def __init__(
        self,
        sample: tuple[dict[str, Any], float] | None = None,
        trigger: SimpleNamespace | None = None,
        no_service: bool = False,
        never: bool = False,
    ) -> None:
        self.sample = sample
        self.trigger = trigger or SimpleNamespace(success=True, message="saved")
        self.no_service = no_service
        self.never = never
        self.services: list[str] = []

    def imu_calibration_sample(self) -> tuple[dict[str, Any], float] | None:
        return self.sample

    def trigger_async(self, service: str) -> Future | None:
        self.services.append(service)
        if self.no_service:
            return None
        future: Future = Future()
        if not self.never:
            future.set_result(self.trigger)
        return future


def make_client(tmp_path: Path, bridge: Any, calibration: bool = True) -> TestClient:
    from web_ui.server import build_app

    config = AppConfig(
        calibration=CalibrationConfig(depthd_url="http://depthd:8090", imu_service_timeout_s=0.05, imu_stale_s=3.0)
        if calibration
        else None
    )
    return TestClient(build_app(config=config, urdf_dir=tmp_path, static_dir=tmp_path / "none", bridge_node=bridge))


def test_status_fresh(tmp_path: Path) -> None:
    with patch("web_ui.imu_calibration.time.monotonic", return_value=101.0):
        response = make_client(tmp_path, FakeBridge(sample=(SAMPLE, 100.0))).get("/api/calibration/imu")
    assert response.status_code == 200
    assert response.json() == {**SAMPLE, "age_s": 1.0}


def test_status_missing(tmp_path: Path) -> None:
    response = make_client(tmp_path, FakeBridge()).get("/api/calibration/imu")
    assert response.status_code == 503
    assert response.json() == {"ok": False, "message": "no IMU calibration sample yet"}


def test_status_no_bridge(tmp_path: Path) -> None:
    assert make_client(tmp_path, None).get("/api/calibration/imu").status_code == 503


def test_status_stale(tmp_path: Path) -> None:
    with patch("web_ui.imu_calibration.time.monotonic", return_value=110.0):
        response = make_client(tmp_path, FakeBridge(sample=(SAMPLE, 100.0))).get("/api/calibration/imu")
    assert response.status_code == 503
    assert response.json() == {"ok": False, "message": "IMU calibration status stale (10 s)"}


def test_malformed_payload_not_cached() -> None:
    from web_ui.bridge import BridgeNode

    node = BridgeNode.__new__(BridgeNode)
    node._imu_status = None
    for bad in ("not json", "[1]", json.dumps({"sys": 3}), json.dumps({**SAMPLE, "mag": "x"})):
        node.on_imu_calibration_status(SimpleNamespace(data=bad))
        assert node.imu_calibration_sample() is None
    node.on_imu_calibration_status(SimpleNamespace(data=json.dumps({**SAMPLE, "extra": 1})))
    sample = node.imu_calibration_sample()
    assert sample is not None and sample[0] == SAMPLE


def test_save_success(tmp_path: Path) -> None:
    bridge = FakeBridge()
    response = make_client(tmp_path, bridge).post("/api/calibration/imu/save")
    assert response.status_code == 200
    assert response.json() == {"ok": True, "message": "saved"}
    assert bridge.services == [SAVE]


def test_reset_success(tmp_path: Path) -> None:
    bridge = FakeBridge(trigger=SimpleNamespace(success=True, message="restarted"))
    response = make_client(tmp_path, bridge).post("/api/calibration/imu/reset")
    assert response.status_code == 200
    assert response.json() == {"ok": True, "message": "restarted"}
    assert bridge.services == [RESET]


def test_refusal_is_409(tmp_path: Path) -> None:
    bridge = FakeBridge(trigger=SimpleNamespace(success=False, message="mag is 2, need 3"))
    response = make_client(tmp_path, bridge).post("/api/calibration/imu/save")
    assert response.status_code == 409
    assert response.json() == {"ok": False, "message": "mag is 2, need 3"}


def test_unavailable_is_503(tmp_path: Path) -> None:
    assert make_client(tmp_path, FakeBridge(no_service=True)).post("/api/calibration/imu/reset").status_code == 503


def test_timeout_is_504(tmp_path: Path) -> None:
    assert make_client(tmp_path, FakeBridge(never=True)).post("/api/calibration/imu/save").status_code == 504


def test_not_configured_is_404(tmp_path: Path) -> None:
    client = make_client(tmp_path, FakeBridge(), calibration=False)
    assert client.get("/api/calibration/imu").status_code == 404
    assert client.post("/api/calibration/imu/save").status_code == 404


def test_trigger_services_follow_calibration() -> None:
    assert {SAVE, RESET} <= set(AppConfig(calibration=CalibrationConfig(depthd_url="http://d:1")).trigger_services())
    assert AppConfig().trigger_services() == []


def _subscriptions(imu_status_topic: str | None) -> list[tuple[type, str]]:
    from web_ui.bridge import BridgeNode

    created: list[tuple[type, str]] = []
    with (
        patch("web_ui.bridge.Node.__init__", return_value=None),
        patch.object(
            BridgeNode, "create_subscription", create=True, side_effect=lambda c, t, cb, q: created.append((c, t))
        ),
        patch.object(BridgeNode, "create_timer", create=True),
        patch.object(BridgeNode, "create_publisher", create=True),
        patch.object(BridgeNode, "get_logger", create=True),
        patch.object(BridgeNode, "create_client", create=True),
    ):
        BridgeNode(topics=[], allowed_publish_topics=set(), imu_status_topic=imu_status_topic)
    return created


def test_subscription_only_with_calibration() -> None:
    from web_ui.bridge import String

    assert (String, "/imu/calibration") in _subscriptions("/imu/calibration")
    assert "/imu/calibration" not in {t for _, t in _subscriptions(None)}


def test_calibration_config_route_is_imu_only(tmp_path: Path) -> None:
    response = make_client(tmp_path, FakeBridge()).get("/api/calibration/config")
    assert response.status_code == 200
    assert response.json() == {"imu": True, "arm": "deferred"}


def test_calibration_config_route_404_when_not_configured(tmp_path: Path) -> None:
    response = make_client(tmp_path, FakeBridge(), calibration=False).get("/api/calibration/config")
    assert response.status_code == 404
    assert response.json() == {"ok": False, "message": "calibration page not configured"}


def test_stereo_calibration_routes_are_gone(tmp_path: Path) -> None:
    client = make_client(tmp_path, FakeBridge())
    assert client.get("/api/calibration/stereo/preview").status_code == 404
