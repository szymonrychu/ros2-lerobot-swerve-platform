"""Tests for the Calibrate page General section: calibration.general config and POST /api/calibration/general/{action}."""

from __future__ import annotations

from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import patch

import pytest
import structlog
from fastapi.testclient import TestClient
from pydantic import ValidationError

from web_ui.config import (
    DEFAULT_ARM_SERVICE_TIMEOUT_S,
    GENERAL_ACTION_GROUPS,
    GENERAL_ACTIONS,
    AppConfig,
    CalibrationConfig,
    CalibrationGeneralConfig,
    ServoGroupConfig,
)

SERVICES = {
    "home_arm": "/arm/home",
    "open_gripper": "/arm/gripper/open",
    "close_gripper": "/arm/gripper/close",
    "home_head": "/head/home",
    "home_steering": "/base/steer_home",
}
RELEASED = {"released": True}
ENABLED = {"released": False}
NAMESPACES = {"arm": "follower", "head": "head", "base": "swerve_drive"}


class FakeBridge:
    """Stand-in bridge: canned torque samples and Trigger answer, records the called services and timeouts."""

    def __init__(
        self,
        samples: dict[str, tuple[dict[str, Any], float]] | None = None,
        trigger: Any = None,
        no_service: bool = False,
    ) -> None:
        self.samples = samples or {}
        self.trigger = trigger if trigger is not None else SimpleNamespace(success=True, message="converged: done")
        self.no_service = no_service
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


def servo_groups() -> list[ServoGroupConfig]:
    return [ServoGroupConfig(id=gid, label=gid.capitalize(), namespace=ns) for gid, ns in NAMESPACES.items()]


def make_client(
    tmp_path: Path,
    bridge: Any,
    general: CalibrationGeneralConfig | None = None,
    guard: Any = None,
    calibration: bool = True,
) -> TestClient:
    from web_ui.server import build_app

    cal = CalibrationConfig(servo_groups=servo_groups(), general=general) if calibration else None
    config = AppConfig(calibration=cal)
    return TestClient(
        build_app(
            config=config, urdf_dir=tmp_path, static_dir=tmp_path / "none", bridge_node=bridge, battery_guard=guard
        )
    )


# --- config ----------------------------------------------------------------------------------------------------------


def test_defaults() -> None:
    general = CalibrationGeneralConfig()
    assert {a: general.service(a) for a in GENERAL_ACTIONS} == SERVICES
    assert general.timeout_s == DEFAULT_ARM_SERVICE_TIMEOUT_S
    assert general.offered() == list(SERVICES)
    assert CalibrationConfig().general is None


def test_actions_order_and_groups() -> None:
    assert GENERAL_ACTIONS == ("home_arm", "open_gripper", "close_gripper", "home_head", "home_steering")
    assert GENERAL_ACTION_GROUPS == {
        "home_arm": "arm",
        "open_gripper": "arm",
        "close_gripper": "arm",
        "home_head": "head",
        "home_steering": "base",
    }


def test_none_service_is_not_offered() -> None:
    general = CalibrationGeneralConfig(open_gripper_service=None, home_head_service=None)
    assert general.offered() == ["home_arm", "close_gripper", "home_steering"]
    assert general.service("open_gripper") is None
    assert general.service("unknown") is None


def test_parses_from_yaml_shape() -> None:
    cal = CalibrationConfig.model_validate({"general": {"home_steering_service": None, "timeout_s": 60}})
    assert cal.general is not None
    assert cal.general.timeout_s == 60
    assert "home_steering" not in cal.general.offered()


def test_timeout_must_be_positive() -> None:
    with pytest.raises(ValidationError):
        CalibrationGeneralConfig(timeout_s=0)


def test_unknown_field_is_rejected() -> None:
    with pytest.raises(ValidationError):
        CalibrationGeneralConfig.model_validate({"home_arm": "/arm/home"})


def test_trigger_services_include_the_offered_general_services() -> None:
    services = AppConfig(calibration=CalibrationConfig(general=CalibrationGeneralConfig())).trigger_services()
    assert set(SERVICES.values()) <= set(services)
    partial = CalibrationGeneralConfig(home_head_service=None)
    services = AppConfig(calibration=CalibrationConfig(general=partial)).trigger_services()
    assert "/head/home" not in services
    assert "/head/home" not in AppConfig(calibration=CalibrationConfig()).trigger_services()


# --- GET /api/calibration/config -------------------------------------------------------------------------------------


def test_config_lists_offered_actions_in_order(tmp_path: Path) -> None:
    general = CalibrationGeneralConfig(close_gripper_service=None)
    body = make_client(tmp_path, FakeBridge(), general).get("/api/calibration/config").json()
    assert body["general"] == ["home_arm", "open_gripper", "home_head", "home_steering"]


def test_config_general_empty_without_the_block(tmp_path: Path) -> None:
    body = make_client(tmp_path, FakeBridge()).get("/api/calibration/config").json()
    assert body["general"] == []
    assert body["imu"] is True and body["servos"] is True


# --- POST /api/calibration/general/{action} --------------------------------------------------------------------------


@pytest.mark.parametrize("action", GENERAL_ACTIONS)
def test_each_action_calls_its_service(tmp_path: Path, action: str) -> None:
    bridge = FakeBridge()
    response = make_client(tmp_path, bridge, CalibrationGeneralConfig()).post(f"/api/calibration/general/{action}")
    assert response.status_code == 200
    assert response.json() == {"ok": True, "message": "converged: done"}
    assert bridge.services == [SERVICES[action]]


def test_uses_the_configured_timeout(tmp_path: Path) -> None:
    seen: list[float] = []

    async def fake_call(action: str, future: Any, service: str, timeout_s: float) -> tuple[Any, None]:
        seen.append(timeout_s)
        return SimpleNamespace(success=True, message="ok"), None

    with patch("web_ui.server.call_ros_service", fake_call):
        client = make_client(tmp_path, FakeBridge(), CalibrationGeneralConfig(timeout_s=42.0))
        assert client.post("/api/calibration/general/home_head").status_code == 200
    assert seen == [42.0]


def test_unknown_action_is_404(tmp_path: Path) -> None:
    bridge = FakeBridge()
    response = make_client(tmp_path, bridge, CalibrationGeneralConfig()).post("/api/calibration/general/dance")
    assert response.status_code == 404
    assert response.json()["ok"] is False
    assert bridge.services == []


def test_not_offered_action_is_404(tmp_path: Path) -> None:
    bridge = FakeBridge()
    general = CalibrationGeneralConfig(home_steering_service=None)
    response = make_client(tmp_path, bridge, general).post("/api/calibration/general/home_steering")
    assert response.status_code == 404
    assert bridge.services == []


def test_not_configured_is_404(tmp_path: Path) -> None:
    assert make_client(tmp_path, FakeBridge()).post("/api/calibration/general/home_arm").status_code == 404
    client = make_client(tmp_path, FakeBridge(), calibration=False)
    assert client.post("/api/calibration/general/home_arm").status_code == 404


def test_battery_cutoff_rejects(tmp_path: Path) -> None:
    bridge = FakeBridge()
    client = make_client(tmp_path, bridge, CalibrationGeneralConfig(), guard=FakeGuard())
    response = client.post("/api/calibration/general/open_gripper")
    assert response.status_code == 503
    assert response.json()["message"] == "battery cut-off"
    assert bridge.services == []


@pytest.mark.parametrize("action", GENERAL_ACTIONS)
def test_released_group_is_409(tmp_path: Path, action: str) -> None:
    group = GENERAL_ACTION_GROUPS[action]
    samples = {ns: (ENABLED, 100.0) for ns in NAMESPACES.values()}
    samples[NAMESPACES[group]] = (RELEASED, 100.0)
    bridge = FakeBridge(samples)
    with patch("web_ui.servo_calibration.time.monotonic", return_value=100.5):
        response = make_client(tmp_path, bridge, CalibrationGeneralConfig()).post(f"/api/calibration/general/{action}")
    assert response.status_code == 409
    assert response.json() == {"ok": False, "message": f"{group.capitalize()} servo torque is off (Calibrate page)"}
    assert bridge.services == []


@pytest.mark.parametrize("age", [None, 50.0])
def test_stale_or_missing_torque_state_does_not_refuse(tmp_path: Path, age: float | None) -> None:
    samples = {} if age is None else {"follower": (RELEASED, 100.0 - age)}
    bridge = FakeBridge(samples)
    with patch("web_ui.servo_calibration.time.monotonic", return_value=100.0):
        response = make_client(tmp_path, bridge, CalibrationGeneralConfig()).post("/api/calibration/general/home_arm")
    assert response.status_code == 200
    assert bridge.services == ["/arm/home"]


def test_released_other_group_does_not_refuse(tmp_path: Path) -> None:
    bridge = FakeBridge({"head": (RELEASED, 100.0)})
    with patch("web_ui.servo_calibration.time.monotonic", return_value=100.5):
        response = make_client(tmp_path, bridge, CalibrationGeneralConfig()).post(
            "/api/calibration/general/open_gripper"
        )
    assert response.status_code == 200


def test_no_bridge_is_503(tmp_path: Path) -> None:
    response = make_client(tmp_path, None, CalibrationGeneralConfig()).post("/api/calibration/general/home_arm")
    assert response.status_code == 503
    assert response.json()["ok"] is False


def test_service_refusal_is_409_with_its_message(tmp_path: Path) -> None:
    text = "open gripper refused: arm control is held - stop the running calibration (Abort) or release control first"
    bridge = FakeBridge(trigger=SimpleNamespace(success=False, message=text))
    response = make_client(tmp_path, bridge, CalibrationGeneralConfig()).post("/api/calibration/general/open_gripper")
    assert response.status_code == 409
    assert response.json() == {"ok": False, "message": text}


def test_service_failure_without_message_names_the_service(tmp_path: Path) -> None:
    bridge = FakeBridge(trigger=SimpleNamespace(success=False, message=""))
    response = make_client(tmp_path, bridge, CalibrationGeneralConfig()).post("/api/calibration/general/home_head")
    assert response.status_code == 409
    assert response.json()["message"] == "/head/home refused"


def test_missing_service_answers_like_call_ros_service(tmp_path: Path) -> None:
    bridge = FakeBridge(no_service=True)
    response = make_client(tmp_path, bridge, CalibrationGeneralConfig()).post("/api/calibration/general/home_steering")
    assert response.status_code == 503
    assert response.json() == {"ok": False, "message": "service /base/steer_home unavailable (is it running?)"}


# --- logs ------------------------------------------------------------------------------------------------------------


def general_logs(logs: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [e for e in logs if e["event"] == "calib_general"]


def test_logs_start_and_ok(tmp_path: Path) -> None:
    client = make_client(tmp_path, FakeBridge(), CalibrationGeneralConfig())
    with structlog.testing.capture_logs() as logs:
        client.post("/api/calibration/general/home_head")
    start, end = general_logs(logs)
    assert start["outcome"] == "start" and end["outcome"] == "ok"
    assert start["run_id"] == end["run_id"] and len(start["run_id"]) == 8
    int(start["run_id"], 16)
    assert start["action"] == end["action"] == "home_head"
    assert start["service"] == end["service"] == "/head/home"
    assert end["reason"] is None and end["duration_s"] >= 0.0
    assert start["log_level"] == "info" and end["log_level"] == "info"


def test_logs_refusals(tmp_path: Path) -> None:
    client = make_client(tmp_path, FakeBridge(), CalibrationGeneralConfig(), guard=FakeGuard())
    with structlog.testing.capture_logs() as logs:
        client.post("/api/calibration/general/close_gripper")
    start, end = general_logs(logs)
    assert (start["outcome"], end["outcome"]) == ("start", "refused")
    assert end["reason"] == "battery cut-off"
    assert end["log_level"] == "warning"


def test_logs_service_refusal_and_failure(tmp_path: Path) -> None:
    refused = FakeBridge(
        trigger=SimpleNamespace(success=False, message="steering home refused: the base lease is held")
    )
    failed = FakeBridge(trigger=SimpleNamespace(success=False, message="timeout: head target ..."))
    with structlog.testing.capture_logs() as logs:
        make_client(tmp_path, refused, CalibrationGeneralConfig()).post("/api/calibration/general/home_steering")
        make_client(tmp_path, failed, CalibrationGeneralConfig()).post("/api/calibration/general/home_head")
        make_client(tmp_path, FakeBridge(no_service=True), CalibrationGeneralConfig()).post(
            "/api/calibration/general/home_arm"
        )
    ends = [e for e in general_logs(logs) if e["outcome"] != "start"]
    assert [(e["action"], e["outcome"], e["log_level"]) for e in ends] == [
        ("home_steering", "refused", "warning"),
        ("home_head", "failed", "error"),
        ("home_arm", "failed", "error"),
    ]
    assert ends[1]["reason"] == "timeout: head target ..."
    assert "unavailable" in ends[2]["reason"]


def test_no_log_for_unknown_action(tmp_path: Path) -> None:
    client = make_client(tmp_path, FakeBridge(), CalibrationGeneralConfig())
    with structlog.testing.capture_logs() as logs:
        client.post("/api/calibration/general/dance")
    assert general_logs(logs) == []
