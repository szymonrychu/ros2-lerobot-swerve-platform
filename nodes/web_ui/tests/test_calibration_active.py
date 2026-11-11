"""GET /api/calibration/active: rules, sources, servo groups, in-flight registry, route and consumer fixture."""

from __future__ import annotations

import asyncio
import json
import time
from collections.abc import Callable
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import patch

import httpx
import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from web_ui.calibration_active import (
    SOURCE_TIMEOUT_S,
    CalibrationActive,
    InFlightRegistry,
    SourceError,
    csi_cameras,
    csi_capture_reasons,
    evaluate,
    fetch_json,
    floor_reasons,
    ground_board_reasons,
    head_arm_reasons,
    register_calibration_active_routes,
    rgbd_state_reasons,
    servo_group_reasons,
)
from web_ui.config import AppConfig, CalibrationConfig, CalibrationGeneralConfig, ServoGroupConfig
from web_ui.server import build_app

RGBD_URL = "http://calib.test:18110"
FIXTURE = Path(__file__).parent / "fixtures" / "api" / "calibration_active.json"
IDLE_BODIES: dict[str, dict[str, Any]] = {
    "/api/state": {"capture": {"running": False}, "compute": {"running": False}},
    "/api/head-arm/state": {"sequence": {"running": False, "phase": "idle", "board_record": None}},
    "/api/floor/state": {"sweep": {"running": False}},
    "/api/ground-board/state": {
        "sweep": {"running": False},
        "verify": {"running": False},
        "compute": {"running": False},
    },
    "/api/csi": {"cameras": ["front"]},
    "/api/csi/front/state": {"capture": {"running": False}},
}
RGBD_SOURCES = ["rgbd:state", "rgbd:head-arm", "rgbd:floor", "rgbd:ground-board", "rgbd:csi"]


class FakeBridge:
    """Bridge stand-in serving servo_torque_sample from a dict namespace -> (state, age_s)."""

    def __init__(self, samples: dict[str, tuple[dict[str, Any], float]] | None = None) -> None:
        self.samples = samples or {}

    def servo_torque_sample(self, namespace: str) -> tuple[dict[str, Any], float] | None:
        entry = self.samples.get(namespace)
        return None if entry is None else (entry[0], time.monotonic() - entry[1])


def servo_state(**over: Any) -> dict[str, Any]:
    state: dict[str, Any] = {
        "released": False,
        "neutral_capture": None,
        "servo_calibration": {"home": None, "capture": None, "plan": None},
        "zero_restore": None,
    }
    state.update(over)
    return state


def group(gid: str = "arm", stale_s: float = 3.0) -> ServoGroupConfig:
    return ServoGroupConfig(id=gid, label=gid.title(), namespace=f"ns_{gid}", state_stale_s=stale_s)


def transport_for(
    bodies: dict[str, Any], overrides: dict[str, Callable[[], httpx.Response]] | None = None
) -> httpx.MockTransport:
    def handler(request: httpx.Request) -> httpx.Response:
        path = request.url.path
        if overrides and path in overrides:
            return overrides[path]()
        if path in bodies:
            return httpx.Response(200, json=bodies[path])
        return httpx.Response(404)

    return httpx.MockTransport(handler)


def make_app(
    transport: httpx.AsyncBaseTransport | None,
    bridge: Any = None,
    groups: list[ServoGroupConfig] | None = None,
    rgbd_url: str | None = RGBD_URL,
    registry: InFlightRegistry | None = None,
    calibration: bool = True,
) -> TestClient:
    app = FastAPI()
    config = AppConfig(
        calibration=CalibrationConfig(rgbd_url=rgbd_url, servo_groups=groups or []) if calibration else None
    )
    register_calibration_active_routes(app, config, bridge, registry or InFlightRegistry(), transport=transport)
    return TestClient(app)


def get(client: TestClient) -> dict[str, Any]:
    response = client.get("/api/calibration/active")
    assert response.status_code == 200
    body = response.json()
    CalibrationActive.model_validate(body)
    return body


def by_name(entries: list[dict[str, str]]) -> dict[str, str]:
    return {e["name"]: e["reason"] for e in entries}


# --- pure rules -----------------------------------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("rule", "idle", "active"),
    [
        (
            rgbd_state_reasons,
            IDLE_BODIES["/api/state"],
            [
                ({"capture": {"running": True}, "compute": {"running": False}}, ["capture.running"]),
                ({"capture": {"running": False}, "compute": {"running": True}}, ["compute.running"]),
            ],
        ),
        (
            floor_reasons,
            IDLE_BODIES["/api/floor/state"],
            [({"sweep": {"running": True}}, ["sweep.running"])],
        ),
        (
            ground_board_reasons,
            IDLE_BODIES["/api/ground-board/state"],
            [
                (
                    {"sweep": {"running": True}, "verify": {"running": False}, "compute": {"running": False}},
                    ["sweep.running"],
                ),
                (
                    {"sweep": {"running": False}, "verify": {"running": True}, "compute": {"running": False}},
                    ["verify.running"],
                ),
                (
                    {"sweep": {"running": False}, "verify": {"running": False}, "compute": {"running": True}},
                    ["compute.running"],
                ),
            ],
        ),
        (
            csi_capture_reasons,
            IDLE_BODIES["/api/csi/front/state"],
            [({"capture": {"running": True}}, ["capture.running"])],
        ),
    ],
)
def test_flag_rules(rule: Callable[[Any], list[str]], idle: Any, active: list[tuple[Any, list[str]]]) -> None:
    assert rule(idle) == []
    for body, reasons in active:
        assert rule(body) == reasons


def test_head_arm_rules() -> None:
    idle = IDLE_BODIES["/api/head-arm/state"]
    assert head_arm_reasons(idle) == []
    assert head_arm_reasons({"sequence": {"running": True, "phase": "idle", "board_record": None}}) == [
        "sequence.running"
    ]
    assert head_arm_reasons({"sequence": {"running": False, "phase": "collect", "board_record": None}}) == [
        "sequence.phase=collect"
    ]
    assert head_arm_reasons({"sequence": {"running": False, "phase": "idle", "board_record": {"id": 1}}}) == [
        "sequence.board_record"
    ]
    assert head_arm_reasons({"sequence": {"running": True, "phase": "collect", "board_record": {}}}) == [
        "sequence.running",
        "sequence.phase=collect",
        "sequence.board_record",
    ]


@pytest.mark.parametrize(
    "body",
    [
        [],
        "x",
        None,
        {},
        {"sequence": []},
        {"sequence": {"running": "yes", "phase": "idle", "board_record": None}},
        {"sequence": {"running": False, "phase": None, "board_record": None}},
        {"sequence": {"running": False, "board_record": None}},
        {"sequence": {"running": False, "phase": "idle"}},
    ],
)
def test_head_arm_rejects_malformed(body: Any) -> None:
    with pytest.raises(SourceError):
        head_arm_reasons(body)


@pytest.mark.parametrize("rule", [rgbd_state_reasons, floor_reasons, ground_board_reasons, csi_capture_reasons])
@pytest.mark.parametrize("body", [[], {}, {"capture": {}}, {"sweep": {"running": 1}}, {"capture": {"running": "yes"}}])
def test_flag_rules_reject_malformed(rule: Callable[[Any], list[str]], body: Any) -> None:
    with pytest.raises(SourceError):
        rule(body)


def test_csi_cameras() -> None:
    assert csi_cameras({"cameras": ["front", "rear"]}) == ["front", "rear"]
    for body in ([], {}, {"cameras": "front"}, {"cameras": [1]}):
        with pytest.raises(SourceError):
            csi_cameras(body)


# --- servo rules ----------------------------------------------------------------------------------------------------


def status(state: Any, stale: bool = False) -> dict[str, Any]:
    return {"state": state, "stale": stale}


def test_servo_idle_variants() -> None:
    assert servo_group_reasons(status(servo_state())) == []
    assert servo_group_reasons(status(servo_state(servo_calibration={"home": None, "capture": {}, "plan": None}))) == []
    # The bridge leaves blocks out for groups without them.
    assert servo_group_reasons(status({"released": False, "neutral_capture": None})) == []


@pytest.mark.parametrize(
    ("over", "reason"),
    [
        ({"released": True}, "released"),
        ({"neutral_capture": {"shoulder": {"steps": 1}}}, "neutral_capture"),
        (
            {"servo_calibration": {"home": None, "capture": {"running": True}, "plan": None}},
            "servo_calibration.capture.running",
        ),
        ({"servo_calibration": {"home": {"x": 1}, "capture": None, "plan": None}}, "servo_calibration.home"),
        ({"servo_calibration": {"home": None, "capture": None, "plan": {"run_id": "a"}}}, "servo_calibration.plan"),
        ({"zero_restore": {"plan": {"run_id": "a"}}}, "zero_restore.plan"),
    ],
)
def test_servo_rules_active(over: dict[str, Any], reason: str) -> None:
    assert servo_group_reasons(status(servo_state(**over))) == [reason]


def test_servo_capture_not_running_and_zero_restore_without_plan_idle() -> None:
    state = servo_state(
        servo_calibration={"home": None, "capture": {"running": False}, "plan": None}, zero_restore={"plan": None}
    )
    assert servo_group_reasons(status(state)) == []


def test_servo_rules_unknown() -> None:
    with pytest.raises(SourceError, match="stale"):
        servo_group_reasons(status(servo_state(), stale=True))
    for state in (None, [], {"neutral_capture": None}, {"released": "no"}):
        with pytest.raises(SourceError):
            servo_group_reasons(status(state))
    with pytest.raises(SourceError):
        servo_group_reasons(status(servo_state(servo_calibration={"capture": {"running": "yes"}})))
    with pytest.raises(SourceError):
        servo_group_reasons(status(servo_state(servo_calibration=[])))


# --- fetch_json -----------------------------------------------------------------------------------------------------


def run_fetch(handler: Callable[[httpx.Request], httpx.Response]) -> Any:
    async def go() -> Any:
        async with httpx.AsyncClient(transport=httpx.MockTransport(handler)) as client:
            return await fetch_json(client, RGBD_URL + "/x")

    return asyncio.run(go())


def test_fetch_json_outcomes() -> None:
    assert run_fetch(lambda r: httpx.Response(200, json={"a": 1})) == {"a": 1}
    with pytest.raises(SourceError, match="^HTTP 503$"):
        run_fetch(lambda r: httpx.Response(503))
    with pytest.raises(SourceError, match="^bad JSON$"):
        run_fetch(lambda r: httpx.Response(200, content=b"{nope"))

    def refuse(request: httpx.Request) -> httpx.Response:
        raise httpx.ConnectError("down")

    with pytest.raises(SourceError, match="^unreachable: ConnectError$"):
        run_fetch(refuse)


# --- in-flight registry ---------------------------------------------------------------------------------------------


def test_registry_nested_and_dropped_at_zero() -> None:
    registry = InFlightRegistry()
    assert registry.snapshot() == {}
    with registry.track("general:home_arm"):
        with registry.track("general:home_arm"):
            with registry.track("general:home_head"):
                assert registry.snapshot() == {"general:home_arm": 2, "general:home_head": 1}
        assert registry.snapshot() == {"general:home_arm": 1}
    assert registry.snapshot() == {}


def test_registry_decrements_after_exception() -> None:
    registry = InFlightRegistry()
    with pytest.raises(RuntimeError), registry.track("k"):
        raise RuntimeError("boom")
    assert registry.snapshot() == {}


def test_registry_concurrent_same_key() -> None:
    registry = InFlightRegistry()

    async def go() -> list[dict[str, int]]:
        seen: list[dict[str, int]] = []
        gate = asyncio.Event()

        async def worker() -> None:
            with registry.track("k"):
                seen.append(registry.snapshot())
                await gate.wait()

        tasks = [asyncio.create_task(worker()) for _ in range(3)]
        await asyncio.sleep(0.01)
        seen.append(registry.snapshot())
        gate.set()
        await asyncio.gather(*tasks)
        return seen

    assert asyncio.run(go())[-1] == {"k": 3}
    assert registry.snapshot() == {}


def test_in_flight_flows_in_response() -> None:
    registry = InFlightRegistry()
    client = make_app(None, rgbd_url=None, registry=registry)
    with registry.track("general:home_head"), registry.track("general:home_arm"), registry.track("general:home_arm"):
        body = get(client)
    assert body["active"] is True
    assert body["flows"] == [
        {"name": "general:home_arm", "reason": "2 request(s) in flight"},
        {"name": "general:home_head", "reason": "1 request(s) in flight"},
    ]
    assert get(client) == {"active": False, "flows": [], "unknown": []}


# --- route ----------------------------------------------------------------------------------------------------------


def test_all_idle() -> None:
    client = make_app(transport_for(IDLE_BODIES), FakeBridge({"ns_arm": (servo_state(), 0.1)}), [group()])
    assert get(client) == {"active": False, "flows": [], "unknown": []}


def test_each_rgbd_source_active() -> None:
    bodies = {
        **IDLE_BODIES,
        "/api/state": {"capture": {"running": True}, "compute": {"running": True}},
        "/api/head-arm/state": {"sequence": {"running": False, "phase": "collect", "board_record": {"a": 1}}},
        "/api/floor/state": {"sweep": {"running": True}},
        "/api/ground-board/state": {
            "sweep": {"running": True},
            "verify": {"running": True},
            "compute": {"running": False},
        },
        "/api/csi/front/state": {"capture": {"running": True}},
    }
    body = get(make_app(transport_for(bodies)))
    assert body["active"] is True
    assert body["unknown"] == []
    assert [f["name"] for f in body["flows"]] == RGBD_SOURCES[:4] + ["rgbd:csi/front"]
    reasons = by_name(body["flows"])
    assert reasons["rgbd:state"] == "capture.running; compute.running"
    assert reasons["rgbd:head-arm"] == "sequence.phase=collect; sequence.board_record"
    assert reasons["rgbd:floor"] == "sweep.running"
    assert reasons["rgbd:ground-board"] == "sweep.running; verify.running"
    assert reasons["rgbd:csi/front"] == "capture.running"


@pytest.mark.parametrize(
    ("path", "name"),
    [
        ("/api/state", "rgbd:state"),
        ("/api/head-arm/state", "rgbd:head-arm"),
        ("/api/floor/state", "rgbd:floor"),
        ("/api/ground-board/state", "rgbd:ground-board"),
        ("/api/csi", "rgbd:csi"),
        ("/api/csi/front/state", "rgbd:csi/front"),
    ],
)
@pytest.mark.parametrize(
    ("replacement", "reason"),
    [
        (lambda: httpx.Response(503), "HTTP 503"),
        (lambda: httpx.Response(200, content=b"<html>"), "bad JSON"),
        (lambda: httpx.Response(200, json=[1, 2]), None),
        (lambda: httpx.Response(200, json={"unrelated": 1}), None),
        (lambda: httpx.Response(200, json={"capture": {"running": "yes"}, "sweep": "x", "cameras": 7}), None),
    ],
)
def test_source_failure_goes_to_unknown(
    path: str, name: str, replacement: Callable[[], httpx.Response], reason: str | None
) -> None:
    body = get(make_app(transport_for(IDLE_BODIES, {path: replacement})))
    assert body["active"] is False
    assert body["flows"] == []
    unknown = by_name(body["unknown"])
    assert name in unknown
    if reason is not None:
        assert unknown[name] == reason


def test_unreachable_source() -> None:
    def refuse(request: httpx.Request) -> httpx.Response:
        raise httpx.ConnectError("down")

    body = get(make_app(httpx.MockTransport(refuse)))
    assert body["active"] is False
    assert by_name(body["unknown"]) == {name: "unreachable: ConnectError" for name in RGBD_SOURCES}


def test_phase_null_is_unknown_not_500() -> None:
    bodies = {
        **IDLE_BODIES,
        "/api/head-arm/state": {"sequence": {"running": False, "phase": None, "board_record": None}},
    }
    body = get(make_app(transport_for(bodies)))
    assert body["active"] is False
    assert list(by_name(body["unknown"])) == ["rgbd:head-arm"]


def test_hanging_upstream_times_out_under_one_second() -> None:
    async def hang(request: httpx.Request) -> httpx.Response:
        await asyncio.sleep(5)
        return httpx.Response(200, json={})

    client = make_app(httpx.MockTransport(hang))
    started = time.monotonic()
    body = get(client)
    assert time.monotonic() - started < 1.0
    assert body["active"] is False
    assert by_name(body["unknown"]) == {name: f"timeout after {SOURCE_TIMEOUT_S:g} s" for name in RGBD_SOURCES}
    assert SOURCE_TIMEOUT_S == 0.8


def test_one_hanging_source_does_not_hide_others() -> None:
    async def handler(request: httpx.Request) -> httpx.Response:
        if request.url.path == "/api/floor/state":
            await asyncio.sleep(5)
        path = request.url.path
        if path == "/api/head-arm/state":
            return httpx.Response(200, json={"sequence": {"running": True, "phase": "idle", "board_record": None}})
        return httpx.Response(200, json=IDLE_BODIES[path])

    body = get(make_app(httpx.MockTransport(handler)))
    assert by_name(body["unknown"]) == {"rgbd:floor": "timeout after 0.8 s"}
    assert by_name(body["flows"]) == {"rgbd:head-arm": "sequence.running"}
    assert body["active"] is True


def test_csi_list_failure() -> None:
    body = get(make_app(transport_for(IDLE_BODIES, {"/api/csi": lambda: httpx.Response(500)})))
    assert by_name(body["unknown"]) == {"rgbd:csi": "HTTP 500"}
    assert body["flows"] == []


def test_csi_one_camera_fails_others_evaluated() -> None:
    bodies = {
        **IDLE_BODIES,
        "/api/csi": {"cameras": ["front", "rear"]},
        "/api/csi/front/state": {"capture": {"running": True}},
    }
    body = get(make_app(transport_for(bodies, {"/api/csi/rear/state": lambda: httpx.Response(503)})))
    assert by_name(body["unknown"]) == {"rgbd:csi/rear": "HTTP 503"}
    assert by_name(body["flows"]) == {"rgbd:csi/front": "capture.running"}
    assert body["active"] is True


def test_csi_one_camera_hangs_others_evaluated() -> None:
    async def handler(request: httpx.Request) -> httpx.Response:
        path = request.url.path
        if path == "/api/csi/rear/state":
            await asyncio.sleep(5)
        if path == "/api/csi":
            return httpx.Response(200, json={"cameras": ["front", "rear"]})
        if path == "/api/csi/front/state":
            return httpx.Response(200, json={"capture": {"running": True}})
        return httpx.Response(200, json=IDLE_BODIES[path])

    started = time.monotonic()
    body = get(make_app(httpx.MockTransport(handler)))
    assert time.monotonic() - started < 1.0
    assert by_name(body["flows"]) == {"rgbd:csi/front": "capture.running"}
    assert by_name(body["unknown"]) == {"rgbd:csi/rear": f"timeout after {SOURCE_TIMEOUT_S:g} s"}
    assert body["active"] is True


def test_csi_without_cameras_is_idle() -> None:
    body = get(make_app(transport_for({**IDLE_BODIES, "/api/csi": {"cameras": []}})))
    assert body == {"active": False, "flows": [], "unknown": []}


def test_servo_sources_in_response() -> None:
    bridge = FakeBridge(
        {
            "ns_arm": (servo_state(released=True, neutral_capture={"j": {}}), 0.1),
            "ns_head": (servo_state(), 10.0),
            "ns_base": ({"neutral_capture": None}, 0.1),
        }
    )
    client = make_app(None, bridge, [group("arm"), group("head"), group("base")], rgbd_url=None)
    body = get(client)
    assert by_name(body["flows"]) == {"servo:arm": "released; neutral_capture"}
    unknown = by_name(body["unknown"])
    assert "stale" in unknown["servo:head"]
    assert unknown["servo:base"] == "missing state.released"
    assert body["active"] is True


def test_servo_no_sample_and_no_bridge_unknown() -> None:
    body = get(make_app(None, FakeBridge(), [group("arm")], rgbd_url=None))
    assert "stale" in by_name(body["unknown"])["servo:arm"]
    body = get(make_app(None, None, [group("arm")], rgbd_url=None))
    assert "stale" in by_name(body["unknown"])["servo:arm"]


def test_no_rgbd_url_only_local_sources() -> None:
    registry = InFlightRegistry()
    bridge = FakeBridge({"ns_arm": (servo_state(), 0.1)})
    client = make_app(None, bridge, [group()], rgbd_url=None, registry=registry)
    assert get(client) == {"active": False, "flows": [], "unknown": []}
    with registry.track("general:home_arm"):
        assert list(by_name(get(client)["flows"])) == ["general:home_arm"]


def test_no_calibration_block() -> None:
    assert get(make_app(None, calibration=False)) == {"active": False, "flows": [], "unknown": []}


def test_model_active_derived_from_flows() -> None:
    assert CalibrationActive.build([], []).active is False
    flow = {"name": "a", "reason": "b"}
    assert CalibrationActive.model_validate({"active": True, "flows": [flow], "unknown": []}).flows[0].name == "a"


# --- consumer fixture -----------------------------------------------------------------------------------------------


def test_fixture_matches_built_response() -> None:
    registry = InFlightRegistry()
    bodies = {
        **IDLE_BODIES,
        "/api/head-arm/state": {"sequence": {"running": False, "phase": "collect", "board_record": None}},
    }
    bridge = FakeBridge({"ns_arm": (servo_state(), 30.0)})
    client = make_app(transport_for(bodies), bridge, [group("arm")], registry=registry)
    with registry.track("general:home_arm"):
        body = get(client)
    fixture = json.loads(FIXTURE.read_text())
    CalibrationActive.model_validate(fixture)
    assert body == fixture


# --- in-flight hooks in the general and servo calibration routes ---------------------------------------------------


class BlockingBridge:
    """Bridge stand-in whose trigger futures stay pending until the test completes them (or raises on demand)."""

    def __init__(self, exc: Exception | None = None) -> None:
        self.exc = exc
        self.futures: list[Future] = []

    def servo_torque_sample(self, namespace: str) -> None:
        return None

    def trigger_async(self, service: str) -> Future:
        future: Future = Future()
        if self.exc is not None:
            future.set_exception(self.exc)
        self.futures.append(future)
        return future


def build_real_app(tmp_path: Path, bridge: Any) -> FastAPI:
    cal = CalibrationConfig(
        servo_groups=[ServoGroupConfig(id="arm", label="Arm", namespace="follower", set_home_service="/arm/set_home")],
        general=CalibrationGeneralConfig(),
    )
    return build_app(
        config=AppConfig(calibration=cal), urdf_dir=tmp_path, static_dir=tmp_path / "none", bridge_node=bridge
    )


async def flow_names(client: httpx.AsyncClient) -> list[str]:
    body = (await client.get("/api/calibration/active")).json()
    return [f["name"] for f in body["flows"] if f["name"].startswith("general:")]


async def wait_for_future(bridge: BlockingBridge) -> Future:
    for _ in range(200):
        if bridge.futures:
            return bridge.futures[0]
        await asyncio.sleep(0.01)
    raise AssertionError("trigger never called")


@pytest.mark.parametrize(
    ("url", "flow"),
    [
        ("/api/calibration/general/home_arm", "general:home_arm"),
        ("/api/calibration/servos/arm/set_home", "general:servo/arm/set_home"),
    ],
)
async def test_route_is_in_flight_while_pending(tmp_path: Path, url: str, flow: str) -> None:
    bridge = BlockingBridge()
    transport = httpx.ASGITransport(app=build_real_app(tmp_path, bridge))
    async with httpx.AsyncClient(transport=transport, base_url="http://t") as client:
        post = asyncio.create_task(client.post(url))
        future = await wait_for_future(bridge)
        body = (await client.get("/api/calibration/active")).json()
        assert body["active"] is True
        assert flow in [f["name"] for f in body["flows"]]
        future.set_result(SimpleNamespace(success=True, message="ok"))
        assert (await post).status_code == 200
        assert await flow_names(client) == []


@pytest.mark.parametrize("url", ["/api/calibration/general/home_arm", "/api/calibration/servos/arm/set_home"])
async def test_route_whose_ros_call_raises_leaves_registry_empty(tmp_path: Path, url: str) -> None:
    bridge = BlockingBridge(exc=RuntimeError("boom"))
    transport = httpx.ASGITransport(app=build_real_app(tmp_path, bridge))
    async with httpx.AsyncClient(transport=transport, base_url="http://t") as client:
        assert (await client.post(url)).status_code == 500
        assert await flow_names(client) == []


@pytest.mark.parametrize("url", ["/api/calibration/general/nope", "/api/calibration/servos/arm/nope"])
async def test_unknown_action_is_never_tracked(tmp_path: Path, url: str) -> None:
    transport = httpx.ASGITransport(app=build_real_app(tmp_path, BlockingBridge()))
    with patch.object(InFlightRegistry, "track", side_effect=AssertionError("tracked")):
        async with httpx.AsyncClient(transport=transport, base_url="http://t") as client:
            assert (await client.post(url)).status_code == 404


# --- never 500: unexpected exceptions and malformed capture --------------------------------------------------------


@pytest.mark.parametrize("capture", [False, 0, "", [], "yes"])
def test_servo_capture_falsy_malformed_is_unknown(capture: Any) -> None:
    state = servo_state(servo_calibration={"home": None, "capture": capture, "plan": None})
    with pytest.raises(SourceError):
        servo_group_reasons(status(state))


def test_evaluate_unexpected_exception_is_unknown() -> None:
    async def boom() -> list[str]:
        raise RecursionError("deep")

    assert asyncio.run(evaluate("rgbd:x", boom(), 1.0)) == ("rgbd:x", [], "error: RecursionError")


def test_route_unexpected_exception_in_rgbd_source_is_unknown_200() -> None:
    with patch("web_ui.calibration_active.csi_capture_reasons", side_effect=RuntimeError("x")):
        body = get(make_app(transport_for(IDLE_BODIES)))
    assert body["active"] is False
    assert by_name(body["unknown"]) == {"rgbd:csi/front": "error: RuntimeError"}


def test_csi_list_unexpected_exception_is_unknown_200() -> None:
    with patch("web_ui.calibration_active.csi_cameras", side_effect=RuntimeError("x")):
        body = get(make_app(transport_for(IDLE_BODIES)))
    assert by_name(body["unknown"]) == {"rgbd:csi": "error: RuntimeError"}


def test_servo_bridge_raising_is_unknown_200() -> None:
    class Boom(FakeBridge):
        def servo_torque_sample(self, namespace: str) -> Any:
            raise RuntimeError("bridge")

    body = get(make_app(None, Boom(), [group("arm")], rgbd_url=None))
    assert by_name(body["unknown"]) == {"servo:arm": "error: RuntimeError"}
    assert body["active"] is False


def test_csi_camera_name_is_quoted_in_path() -> None:
    seen: list[bytes] = []

    def handler(request: httpx.Request) -> httpx.Response:
        seen.append(request.url.raw_path)
        if request.url.path == "/api/csi":
            return httpx.Response(200, json={"cameras": ["a b/c"]})
        return httpx.Response(200, json=IDLE_BODIES.get(request.url.path, {"capture": {"running": False}}))

    body = get(make_app(httpx.MockTransport(handler)))
    assert b"/api/csi/a%20b%2Fc/state" in seen
    assert "rgbd:csi/a b/c" not in by_name(body["unknown"])
