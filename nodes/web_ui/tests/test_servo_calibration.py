"""Tests for the servo torque / neutral routes: config, state cache, GET groups and POST actions."""

from __future__ import annotations

import json
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import patch

import pytest
from fastapi.responses import JSONResponse
from fastapi.testclient import TestClient
from pydantic import ValidationError

from web_ui.config import AppConfig, CalibrationConfig, ServoGroupConfig

ZERO_RESTORE_ACTIONS = (
    "zero_restore_dry_run",
    "zero_restore_apply",
    "zero_restore_apply_confirmed",
    "save_rest_reference",
)
STATE = {"namespace": "follower", "released": True, "joints": {"a": 0}, "stamp_ns": 1}
PLAN_RUN_ID = "1a2b3c4d"
PLAN_STATE = {**STATE, "zero_restore": {"plan": {"run_id": PLAN_RUN_ID, "needs_confirm": False, "joints": {}}}}
APPLY_ACTIONS = ("zero_restore_apply", "zero_restore_apply_confirmed")


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
            zero_restore=True,
            move_to_neutral_service="/arm/move_to_neutral",
            set_home_service="/arm/set_home",
            home_pose_hint="pointing straight forward",
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
    assert group.service("save_confirmed") == "/follower/servo/save_neutral_confirmed"
    config = AppConfig(calibration=CalibrationConfig(servo_groups=groups()))
    assert config.servo_state_topics() == {"follower": "/follower/torque_state", "head": "/head/torque_state"}
    assert "/head/servo/release_torque" in config.trigger_services()
    assert "/follower/servo/save_neutral" in config.trigger_services()
    assert "/head/servo/save_neutral" not in config.trigger_services()
    assert "/follower/servo/save_neutral_confirmed" in config.trigger_services()
    assert "/head/servo/save_neutral_confirmed" not in config.trigger_services()
    for action in ZERO_RESTORE_ACTIONS:
        assert f"/follower/servo/{action}" in config.trigger_services()
        assert f"/head/servo/{action}" not in config.trigger_services()
    assert "/arm/move_to_neutral" in config.trigger_services()
    assert "/arm/set_home" in config.trigger_services()


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
        "calibration_stale": ["joint_offsets", "home"], "zero_restore": True, "servo_calibration": False, "move_to_neutral": True,
        "set_home": True, "home_pose_hint": "pointing straight forward",
    }  # fmt: skip
    assert head["stale"] is True and head["age_s"] == 11.0 and head["calibration_stale"] is None
    assert head["neutral"] is False
    assert head["zero_restore"] is False and head["move_to_neutral"] is False and head["set_home"] is False
    assert head["home_pose_hint"] is None


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
        ("save_confirmed", "/follower/servo/save_neutral_confirmed"),
        ("zero_restore_dry_run", "/follower/servo/zero_restore_dry_run"),
        ("zero_restore_apply", "/follower/servo/zero_restore_apply"),
        ("zero_restore_apply_confirmed", "/follower/servo/zero_restore_apply_confirmed"),
        ("save_rest_reference", "/follower/servo/save_rest_reference"),
        ("move_to_neutral", "/arm/move_to_neutral"),
        ("set_home", "/arm/set_home"),
    ],
)
def test_post_calls_derived_service(tmp_path: Path, action: str, service: str) -> None:
    # Apply actions are bound to the shown dry run: a fresh cached plan and its run id (see test_apply_plan_binding).
    bridge = FakeBridge({"follower": (PLAN_STATE, 100.0)} if action in APPLY_ACTIONS else None)
    query = f"?plan_run_id={PLAN_RUN_ID}" if action in APPLY_ACTIONS else ""
    with patch("web_ui.servo_calibration.time.monotonic", return_value=100.5):
        response = make_client(tmp_path, bridge).post(f"/api/calibration/servos/arm/{action}{query}")
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
    assert client.post("/api/calibration/servos/head/save_confirmed").status_code == 404
    for action in ZERO_RESTORE_ACTIONS:
        assert client.post(f"/api/calibration/servos/head/{action}").status_code == 404
    assert client.post("/api/calibration/servos/head/move_to_neutral").status_code == 404
    assert client.post("/api/calibration/servos/head/set_home").status_code == 404


def test_post_503_paths(tmp_path: Path) -> None:
    assert make_client(tmp_path, None).post("/api/calibration/servos/arm/release").status_code == 503
    bridge = FakeBridge(no_service=True)
    assert make_client(tmp_path, bridge).post("/api/calibration/servos/arm/release").status_code == 503


def test_save_blocked_under_cutoff_release_is_not(tmp_path: Path) -> None:
    bridge = FakeBridge()
    client = make_client(tmp_path, bridge, guard=FakeGuard())
    assert client.post("/api/calibration/servos/arm/save").status_code != 200
    assert client.post("/api/calibration/servos/arm/save_confirmed").status_code != 200
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


def test_zero_restore_and_move_timeouts() -> None:
    group = ServoGroupConfig(id="arm", label="Arm", namespace="follower")
    assert group.zero_restore is False and group.move_to_neutral_service is None
    assert group.zero_restore_timeout_s == 30.0
    assert group.move_to_neutral_timeout_s == 30.0
    assert group.timeout("zero_restore_apply") == 30.0
    assert group.timeout("save_rest_reference") == 30.0
    assert group.timeout("move_to_neutral") == 30.0
    assert group.timeout("save") == group.service_timeout_s == 15.0
    with pytest.raises(ValidationError):
        ServoGroupConfig(id="arm", label="a", namespace="x", zero_restore_timeout_s=0)
    with pytest.raises(ValidationError):
        ServoGroupConfig(id="arm", label="a", namespace="x", move_to_neutral_timeout_s=0)


def test_zero_restore_actions_wait_zero_restore_timeout(tmp_path: Path) -> None:
    seen: list[tuple[str, float]] = []

    async def fake_call(action: str, future: Any, service: str, timeout_s: float) -> tuple[Any, None]:
        seen.append((service, timeout_s))
        return SimpleNamespace(success=True, message="ok"), None

    with (
        patch("web_ui.server.call_ros_service", fake_call),
        patch("web_ui.servo_calibration.time.monotonic", return_value=100.5),
    ):
        # torque on in the cached state, so move_to_neutral is not refused as released
        client = make_client(tmp_path, FakeBridge({"follower": ({**PLAN_STATE, "released": False}, 100.0)}))
        client.post(f"/api/calibration/servos/arm/zero_restore_apply?plan_run_id={PLAN_RUN_ID}")
        client.post("/api/calibration/servos/arm/release")
        client.post("/api/calibration/servos/arm/move_to_neutral")
    assert seen == [
        ("/follower/servo/zero_restore_apply", 30.0),
        ("/follower/servo/release_torque", 15.0),
        ("/arm/move_to_neutral", 30.0),
    ]


def test_apply_and_move_blocked_under_cutoff_dry_run_is_not(tmp_path: Path) -> None:
    bridge = FakeBridge()
    client = make_client(tmp_path, bridge, guard=FakeGuard())
    for action in ("zero_restore_apply", "zero_restore_apply_confirmed", "move_to_neutral"):
        assert client.post(f"/api/calibration/servos/arm/{action}").status_code != 200
    assert bridge.services == []
    assert client.post("/api/calibration/servos/arm/zero_restore_dry_run").status_code == 200
    assert client.post("/api/calibration/servos/arm/save_rest_reference").status_code == 200


def test_move_to_neutral_refused_while_released(tmp_path: Path) -> None:
    bridge = FakeBridge({"follower": (STATE, 100.0)})
    with patch("web_ui.servo_calibration.time.monotonic", return_value=100.5):
        response = make_client(tmp_path, bridge).post("/api/calibration/servos/arm/move_to_neutral")
    assert response.status_code == 409
    assert response.json() == {"ok": False, "message": "Arm servo torque is off (Calibrate page)"}
    assert bridge.services == []


def test_set_home_is_allowed_with_torque_off_and_waits_the_service_timeout(tmp_path: Path) -> None:
    """Set home moves nothing: the user poses the arm by hand with torque off, then records it (mcp_server /arm/set_home)."""
    seen: list[tuple[str, float]] = []

    async def fake_call(action: str, future: Any, service: str, timeout_s: float) -> tuple[Any, None]:
        seen.append((service, timeout_s))
        return SimpleNamespace(success=True, message="home stored"), None

    with (
        patch("web_ui.server.call_ros_service", fake_call),
        patch("web_ui.servo_calibration.time.monotonic", return_value=100.5),
    ):
        response = make_client(tmp_path, FakeBridge({"follower": (STATE, 100.0)})).post(
            "/api/calibration/servos/arm/set_home"
        )
    assert STATE["released"] is True
    assert (response.status_code, response.json()) == (200, {"ok": True, "message": "home stored"})
    assert seen == [("/arm/set_home", 15.0)]


def test_set_home_blocked_under_cutoff(tmp_path: Path) -> None:
    bridge = FakeBridge()
    response = make_client(tmp_path, bridge, guard=FakeGuard()).post("/api/calibration/servos/arm/set_home")
    assert response.status_code != 200
    assert bridge.services == []


def test_set_home_refusal_is_409_with_the_mcp_message(tmp_path: Path) -> None:
    bridge = FakeBridge(trigger=SimpleNamespace(success=False, message="no fresh arm joint state"))
    response = make_client(tmp_path, bridge).post("/api/calibration/servos/arm/set_home")
    assert (response.status_code, response.json()) == (409, {"ok": False, "message": "no fresh arm joint state"})


STALE_SET_HOME_REFUSAL = (
    "set home refused: the arm zero is stale (joint_offsets); run the zero restore first "
    "(arm re-zero at 2026-10-20T10:00:00+00:00)"
)


def test_set_home_stale_zero_refusal_passes_through_as_409(tmp_path: Path) -> None:
    """mcp_server refuses /arm/set_home while joint_offsets is stale: the route returns its message as the error."""
    bridge = FakeBridge(
        {"follower": (STATE, 100.0)}, trigger=SimpleNamespace(success=False, message=STALE_SET_HOME_REFUSAL)
    )
    response = make_client(tmp_path, bridge).post("/api/calibration/servos/arm/set_home")
    assert (response.status_code, response.json()) == (409, {"ok": False, "message": STALE_SET_HOME_REFUSAL})
    assert bridge.services == ["/arm/set_home"]


def test_move_to_neutral_404_without_service(tmp_path: Path) -> None:
    from web_ui.server import build_app

    group = ServoGroupConfig(id="arm", label="Arm", namespace="follower", neutral=True)
    config = AppConfig(calibration=CalibrationConfig(servo_groups=[group]))
    app = build_app(config=config, urdf_dir=tmp_path, static_dir=tmp_path / "none", bridge_node=FakeBridge())
    client = TestClient(app)
    assert client.post("/api/calibration/servos/arm/move_to_neutral").status_code == 404
    assert client.post("/api/calibration/servos/arm/set_home").status_code == 404
    assert client.post("/api/calibration/servos/arm/zero_restore_dry_run").status_code == 404


PLAN_CHANGED = "the dry run changed since it was shown; review the new plan"


@pytest.mark.parametrize("action", APPLY_ACTIONS)
@pytest.mark.parametrize(
    ("samples", "query", "now"),
    [
        ({"follower": (PLAN_STATE, 100.0)}, "", 100.5),  # no run id sent
        ({"follower": (PLAN_STATE, 100.0)}, "?plan_run_id=deadbeef", 100.5),  # another plan is current
        ({"follower": (STATE, 100.0)}, f"?plan_run_id={PLAN_RUN_ID}", 100.5),  # no plan cached
        ({"follower": (PLAN_STATE, 100.0)}, f"?plan_run_id={PLAN_RUN_ID}", 200.0),  # cached state is stale
        ({}, f"?plan_run_id={PLAN_RUN_ID}", 100.5),  # no state at all
    ],
)
def test_apply_plan_binding_refuses_a_plan_other_than_the_shown_one(
    tmp_path: Path, action: str, samples: dict, query: str, now: float
) -> None:
    bridge = FakeBridge(samples)
    with patch("web_ui.servo_calibration.time.monotonic", return_value=now):
        response = make_client(tmp_path, bridge).post(f"/api/calibration/servos/arm/{action}{query}")
    assert response.status_code == 409
    assert response.json() == {"ok": False, "message": PLAN_CHANGED}
    assert bridge.services == []


def test_plan_run_id_is_ignored_by_other_actions(tmp_path: Path) -> None:
    bridge = FakeBridge()
    response = make_client(tmp_path, bridge).post("/api/calibration/servos/arm/zero_restore_dry_run?plan_run_id=x")
    assert response.status_code == 200
    assert bridge.services == ["/follower/servo/zero_restore_dry_run"]


CAL_ACTIONS = (
    "calibration_record_home",
    "calibration_capture_start",
    "calibration_capture_stop",
    "calibration_plan",
    "calibration_store",
    "calibration_store_confirmed",
)
CAL_STORES = ("calibration_store", "calibration_store_confirmed")
CAL_RUN_ID = "c0ffee11"
CAL_STATE = {
    **STATE,
    "released": False,
    "servo_calibration": {"plan": {"run_id": CAL_RUN_ID, "needs_confirm": False, "ok": True, "joints": {}}},
}
SET_HOME_FROM_CAL = "/arm/set_home_from_calibration"


def cal_groups() -> list[ServoGroupConfig]:
    return [
        ServoGroupConfig(
            id="arm",
            label="Arm",
            namespace="follower",
            servo_calibration=True,
            set_home_from_calibration_service=SET_HOME_FROM_CAL,
        ),
        ServoGroupConfig(id="head", label="Head", namespace="head", servo_calibration=True),
        ServoGroupConfig(id="base", label="Base", namespace="base"),
    ]


def cal_client(tmp_path: Path, bridge: Any, guard: Any = None) -> TestClient:
    from web_ui.server import build_app

    config = AppConfig(calibration=CalibrationConfig(servo_groups=cal_groups()))
    return TestClient(
        build_app(
            config=config, urdf_dir=tmp_path, static_dir=tmp_path / "none", bridge_node=bridge, battery_guard=guard
        )
    )


def cal_samples() -> dict[str, tuple[dict[str, Any], float]]:
    return {"follower": (CAL_STATE, 100.0), "head": (CAL_STATE, 100.0)}


def cal_url(group: str, action: str) -> str:
    query = f"?plan_run_id={CAL_RUN_ID}" if action in CAL_STORES else ""
    return f"/api/calibration/servos/{group}/{action}{query}"


@pytest.mark.parametrize("action", CAL_ACTIONS)
def test_calibration_actions_map_to_bridge_services(tmp_path: Path, action: str) -> None:
    bridge = FakeBridge(cal_samples())
    with patch("web_ui.servo_calibration.time.monotonic", return_value=100.5):
        response = cal_client(tmp_path, bridge).post(cal_url("head", action))
    assert response.status_code == 200
    assert bridge.services == [f"/head/servo/{action}"]
    config = AppConfig(calibration=CalibrationConfig(servo_groups=cal_groups()))
    assert "/head/servo/calibration_plan" in config.trigger_services()
    assert SET_HOME_FROM_CAL in config.trigger_services()


@pytest.mark.parametrize("action", CAL_ACTIONS)
def test_calibration_actions_404_without_flag(tmp_path: Path, action: str) -> None:
    bridge = FakeBridge(cal_samples())
    assert cal_client(tmp_path, bridge).post(cal_url("base", action)).status_code == 404
    assert bridge.services == []


@pytest.mark.parametrize("action", CAL_STORES)
def test_calibration_store_battery_blocked(tmp_path: Path, action: str) -> None:
    bridge = FakeBridge(cal_samples())
    response = cal_client(tmp_path, bridge, guard=FakeGuard()).post(cal_url("head", action))
    assert response.status_code == 503
    assert bridge.services == []
    with patch("web_ui.servo_calibration.time.monotonic", return_value=100.5):
        assert (
            cal_client(tmp_path, bridge, guard=FakeGuard()).post(cal_url("head", "calibration_plan")).status_code == 200
        )


@pytest.mark.parametrize("action", CAL_STORES)
@pytest.mark.parametrize(
    ("samples", "query", "now"),
    [
        ("cal", "", 100.5),
        ("cal", "?plan_run_id=deadbeef", 100.5),
        ("zero", f"?plan_run_id={CAL_RUN_ID}", 100.5),  # only a zero_restore plan cached
        ("cal", f"?plan_run_id={CAL_RUN_ID}", 200.0),  # stale
        ("none", f"?plan_run_id={CAL_RUN_ID}", 100.5),
    ],
)
def test_calibration_store_plan_bound(tmp_path: Path, action: str, samples: str, query: str, now: float) -> None:
    cached = {
        "cal": {"head": (CAL_STATE, 100.0)},
        "zero": {"head": ({**STATE, "zero_restore": {"plan": {"run_id": CAL_RUN_ID}}}, 100.0)},
        "none": {},
    }[samples]
    bridge = FakeBridge(cached)
    with patch("web_ui.servo_calibration.time.monotonic", return_value=now):
        response = cal_client(tmp_path, bridge).post(f"/api/calibration/servos/head/{action}{query}")
    assert response.status_code == 409
    assert response.json() == {"ok": False, "message": PLAN_CHANGED}
    assert bridge.services == []


def test_calibration_store_uses_long_timeout(tmp_path: Path) -> None:
    seen: list[tuple[str, float]] = []

    async def fake_call(action: str, future: Any, service: str, timeout_s: float) -> tuple[Any, None]:
        seen.append((service, timeout_s))
        return SimpleNamespace(success=True, message="ok"), None

    with (
        patch("web_ui.server.call_ros_service", fake_call),
        patch("web_ui.servo_calibration.time.monotonic", return_value=100.5),
    ):
        client = cal_client(tmp_path, FakeBridge(cal_samples()))
        client.post(cal_url("head", "calibration_store"))
        client.post(cal_url("head", "calibration_capture_start"))
        client.post("/api/calibration/servos/head/release")
    assert seen == [
        ("/head/servo/calibration_store", 60.0),
        ("/head/servo/calibration_capture_start", 60.0),
        ("/head/servo/release_torque", 15.0),
    ]


@pytest.mark.parametrize("action", CAL_STORES)
def test_arm_store_chains_set_home_from_calibration(tmp_path: Path, action: str) -> None:
    bridge = FakeBridge(cal_samples(), trigger=SimpleNamespace(success=True, message="stored"))
    with patch("web_ui.servo_calibration.time.monotonic", return_value=100.5):
        response = cal_client(tmp_path, bridge).post(cal_url("arm", action))
    assert response.status_code == 200
    body = response.json()
    assert body["ok"] is True and body["home_set"] is True
    assert "stored" in body["message"] and "set home from calibration" in body["message"]
    assert bridge.services == [f"/follower/servo/{action}", SET_HOME_FROM_CAL]


def test_arm_store_reports_home_not_set_on_failure(tmp_path: Path) -> None:
    async def fake_call(action: str, future: Any, service: str, timeout_s: float) -> tuple[Any, None]:
        ok = service != SET_HOME_FROM_CAL
        return SimpleNamespace(success=ok, message="stored" if ok else "no record"), None

    with (
        patch("web_ui.server.call_ros_service", fake_call),
        patch("web_ui.servo_calibration.time.monotonic", return_value=100.5),
    ):
        response = cal_client(tmp_path, FakeBridge(cal_samples())).post(cal_url("arm", "calibration_store"))
    body = response.json()
    assert response.status_code == 200 and body["ok"] is True and body["home_set"] is False
    assert "stored" in body["message"] and "no record" in body["message"]


def test_arm_store_chain_call_failure_reports_reason(tmp_path: Path) -> None:
    async def fake_call(action: str, future: Any, service: str, timeout_s: float) -> tuple[Any, Any]:
        if service == SET_HOME_FROM_CAL:
            return None, JSONResponse({"ok": False, "message": f"{service} timed out after 5 s"}, status_code=504)
        return SimpleNamespace(success=True, message="stored"), None

    with (
        patch("web_ui.server.call_ros_service", fake_call),
        patch("web_ui.servo_calibration.time.monotonic", return_value=100.5),
    ):
        response = cal_client(tmp_path, FakeBridge(cal_samples())).post(cal_url("arm", "calibration_store"))
    body = response.json()
    assert response.status_code == 200 and body["ok"] is True and body["home_set"] is False
    assert "stored" in body["message"] and "timed out after 5 s" in body["message"]


def test_arm_store_chain_success_with_empty_message_says_succeeded(tmp_path: Path) -> None:
    async def fake_call(action: str, future: Any, service: str, timeout_s: float) -> tuple[Any, None]:
        return SimpleNamespace(success=True, message="stored" if service != SET_HOME_FROM_CAL else ""), None

    with (
        patch("web_ui.server.call_ros_service", fake_call),
        patch("web_ui.servo_calibration.time.monotonic", return_value=100.5),
    ):
        response = cal_client(tmp_path, FakeBridge(cal_samples())).post(cal_url("arm", "calibration_store"))
    body = response.json()
    assert body["home_set"] is True
    assert f"{SET_HOME_FROM_CAL} succeeded" in body["message"] and "failed" not in body["message"]


def test_arm_failed_store_does_not_chain(tmp_path: Path) -> None:
    bridge = FakeBridge(cal_samples(), trigger=SimpleNamespace(success=False, message="refused"))
    with patch("web_ui.servo_calibration.time.monotonic", return_value=100.5):
        response = cal_client(tmp_path, bridge).post(cal_url("arm", "calibration_store"))
    assert response.status_code == 409
    assert bridge.services == ["/follower/servo/calibration_store"]


def test_head_store_does_not_chain(tmp_path: Path) -> None:
    bridge = FakeBridge(cal_samples())
    with patch("web_ui.servo_calibration.time.monotonic", return_value=100.5):
        response = cal_client(tmp_path, bridge).post(cal_url("head", "calibration_store"))
    assert response.json()["home_set"] is None
    assert bridge.services == ["/head/servo/calibration_store"]


def test_status_lists_servo_calibration_flag(tmp_path: Path) -> None:
    groups_json = cal_client(tmp_path, FakeBridge()).get("/api/calibration/servos").json()["groups"]
    assert {g["id"]: g["servo_calibration"] for g in groups_json} == {"arm": True, "head": True, "base": False}
