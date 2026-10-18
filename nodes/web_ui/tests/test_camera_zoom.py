"""Tests for the Head tab zoom backend: config, BridgeNode.zoom_request_async and POST /api/camera/zoom."""

from __future__ import annotations

import base64
import json
import threading
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest
from fastapi.testclient import TestClient
from pydantic import ValidationError

from web_ui.config import AppConfig, TabConfig

CMD = "/head_camera/zoom/command"
RES = "/head_camera/zoom/result"
IMG = "/head_camera/zoom/image"
JPEG = b"\xff\xd8fakejpeg\xff\xd9"
STAMP_NS = 1_700_000_123_456_789_012


def zoom_tab(**extra: Any) -> TabConfig:
    return TabConfig(
        id="head",
        type="head",
        label="Head",
        zoom_command_topic=CMD,
        zoom_result_topic=RES,
        zoom_image_topic=IMG,
        **extra,
    )


def result_json(request_id: str, status: str = "ok", stamp_ns: int = STAMP_NS) -> SimpleNamespace:
    body = {"request_id": request_id, "status": status, "message": "m", "stamp_ns": stamp_ns, "stamp_s": 1.7e9}
    return SimpleNamespace(data=json.dumps(body))


def image_msg(stamp_ns: int = STAMP_NS, data: bytes = JPEG) -> SimpleNamespace:
    stamp = SimpleNamespace(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)
    return SimpleNamespace(header=SimpleNamespace(stamp=stamp), data=list(data), format="jpeg")


def make_bridge(subscribers: int = 1) -> Any:
    """Build a BridgeNode without rclpy initialisation, with a mock zoom publisher for tab 'head'."""
    from web_ui.bridge import BridgeNode

    node = BridgeNode.__new__(BridgeNode)
    node._lock = threading.Lock()
    node._zoom_pubs = {"head": MagicMock()}
    node._zoom_pubs["head"].get_subscription_count.return_value = subscribers
    node._zoom_pending = {}
    node._zoom_images = {}
    return node


def sent_command(node: Any) -> dict[str, Any]:
    return json.loads(node._zoom_pubs["head"].publish.call_args[0][0].data)


# --- config -----------------------------------------------------------------------------------------------------


def test_zoom_topics_all_set_and_defaults() -> None:
    tab = zoom_tab()
    assert (tab.zoom_command_topic, tab.zoom_result_topic, tab.zoom_image_topic) == (CMD, RES, IMG)
    assert tab.zoom_timeout_s == 8.0
    assert AppConfig(tabs=[tab]).zoom_topics() == {"head": (CMD, RES, IMG)}
    assert AppConfig(tabs=[TabConfig(id="head", type="head", label="Head")]).zoom_topics() == {}


@pytest.mark.parametrize("missing", ["zoom_command_topic", "zoom_result_topic", "zoom_image_topic"])
def test_zoom_topics_all_or_none(missing: str) -> None:
    topics = {"zoom_command_topic": CMD, "zoom_result_topic": RES, "zoom_image_topic": IMG}
    del topics[missing]
    with pytest.raises(ValidationError, match="zoom"):
        TabConfig(id="head", type="head", label="Head", **topics)


def test_zoom_timeout_must_be_positive() -> None:
    with pytest.raises(ValidationError):
        zoom_tab(zoom_timeout_s=0)


def test_zoom_topics_only_on_head_tab() -> None:
    with pytest.raises(ValidationError, match="head"):
        TabConfig(id="a", type="arm", label="Arm", zoom_command_topic=CMD, zoom_result_topic=RES, zoom_image_topic=IMG)


def test_default_and_deployed_head_tab_have_zoom_topics() -> None:
    import yaml

    from web_ui.config import load_config

    default = load_config(Path(__file__).parent.parent / "config" / "default.yaml")
    assert default.zoom_topics() == {"head": (CMD, RES, IMG)}
    client = yaml.safe_load((Path(__file__).parents[3] / "ansible" / "group_vars" / "client.yml").read_text())
    node = next(n for n in client["ros2_nodes"] if n["name"] == "web_ui")
    shipped = AppConfig.model_validate(yaml.safe_load(node["config"]))
    assert shipped.zoom_topics() == {"head": (CMD, RES, IMG)}


# --- bridge -----------------------------------------------------------------------------------------------------


def test_zoom_request_publishes_command_json() -> None:
    node = make_bridge()
    future = node.zoom_request_async("head", 0.62, 0.41, 4)
    assert future is not None
    cmd = sent_command(node)
    assert cmd == {"request_id": cmd["request_id"], "u": 0.62, "v": 0.41, "zoom": 4, "max_px": 1024, "af": True}
    assert cmd["request_id"]
    node2 = make_bridge()
    node2.zoom_request_async("head", 0.1, 0.2, 2)
    assert sent_command(node2)["request_id"] != cmd["request_id"]


def test_zoom_request_none_without_zoom_or_subscriber() -> None:
    assert make_bridge(subscribers=0).zoom_request_async("head", 0.5, 0.5, 2) is None
    assert make_bridge().zoom_request_async("other", 0.5, 0.5, 2) is None


def test_zoom_result_then_image_resolves() -> None:
    node = make_bridge()
    future = node.zoom_request_async("head", 0.5, 0.5, 2)
    rid = sent_command(node)["request_id"]
    node.on_zoom_result("head", result_json(rid))
    assert not future.done()
    node.on_zoom_image("head", image_msg())
    out = future.result(timeout=0)
    assert out["result"]["status"] == "ok" and out["jpeg"] == JPEG
    assert node._zoom_pending == {}


def test_zoom_image_then_result_resolves() -> None:
    node = make_bridge()
    future = node.zoom_request_async("head", 0.5, 0.5, 2)
    rid = sent_command(node)["request_id"]
    node.on_zoom_image("head", image_msg(STAMP_NS + 5, b"other"))
    node.on_zoom_image("head", image_msg())
    node.on_zoom_result("head", result_json(rid))
    assert future.result(timeout=0)["jpeg"] == JPEG


def test_zoom_af_failed_returns_image() -> None:
    node = make_bridge()
    future = node.zoom_request_async("head", 0.5, 0.5, 8)
    rid = sent_command(node)["request_id"]
    node.on_zoom_image("head", image_msg())
    node.on_zoom_result("head", result_json(rid, "af_failed"))
    out = future.result(timeout=0)
    assert out["result"]["status"] == "af_failed" and out["jpeg"] == JPEG


@pytest.mark.parametrize("status", ["busy", "timeout", "camera_error", "invalid"])
def test_zoom_no_image_statuses_resolve_at_once(status: str) -> None:
    node = make_bridge()
    future = node.zoom_request_async("head", 0.5, 0.5, 2)
    rid = sent_command(node)["request_id"]
    node.on_zoom_result("head", result_json(rid, status, 0))
    out = future.result(timeout=0)
    assert out == {"result": out["result"], "jpeg": None} and out["result"]["status"] == status


def test_zoom_keeps_last_four_images_only() -> None:
    node = make_bridge()
    for i in range(5):
        node.on_zoom_image("head", image_msg(100 + i, bytes([i])))
    assert [stamp for stamp, _ in node._zoom_images["head"]] == [101, 102, 103, 104]


def test_zoom_unknown_and_malformed_results_ignored() -> None:
    node = make_bridge()
    node.on_zoom_result("head", result_json("zzz"))
    node.on_zoom_result("head", SimpleNamespace(data="{bad"))
    node.on_zoom_result("head", SimpleNamespace(data=json.dumps({"status": "ok"})))
    assert node._zoom_pending == {}


def test_zoom_cancelled_request_is_forgotten() -> None:
    node = make_bridge()
    future = node.zoom_request_async("head", 0.5, 0.5, 2)
    future.cancel()
    assert node._zoom_pending == {}


# --- endpoint ---------------------------------------------------------------------------------------------------


class FakeBridge:
    """Stand-in bridge returning a canned zoom outcome (or None, or a future that never resolves)."""

    def __init__(self, outcome: dict[str, Any] | None = None, never: bool = False, none: bool = False) -> None:
        self.outcome = outcome
        self.never = never
        self.none = none
        self.calls: list[tuple[str, float, float, int]] = []

    def zoom_request_async(self, tab_id: str, u: float, v: float, zoom: int) -> Future | None:
        self.calls.append((tab_id, u, v, zoom))
        if self.none:
            return None
        future: Future = Future()
        if not self.never:
            future.set_result(self.outcome)
        return future


def outcome(status: str = "ok", jpeg: bytes | None = JPEG) -> dict[str, Any]:
    return {"result": {"request_id": "r", "status": status, "message": "m", "stamp_ns": STAMP_NS}, "jpeg": jpeg}


def make_client(tmp_path: Path, bridge: Any, battery_guard: Any = None, timeout: float = 8.0) -> TestClient:
    from web_ui.server import build_app

    config = AppConfig(
        tabs=[
            zoom_tab(zoom_timeout_s=timeout),
            TabConfig(id="nozoom", type="head", label="Head 2"),
            TabConfig(id="map", type="map_nav", label="Map"),
        ]
    )
    app = build_app(
        config=config, urdf_dir=tmp_path, static_dir=tmp_path / "none", bridge_node=bridge, battery_guard=battery_guard
    )
    return TestClient(app)


BODY = {"u": 0.62, "v": 0.41, "zoom": 4}


@pytest.mark.parametrize("status", ["ok", "af_failed"])
def test_api_zoom_returns_result_and_jpeg(tmp_path: Path, status: str) -> None:
    bridge = FakeBridge(outcome(status))
    resp = make_client(tmp_path, bridge).post("/api/camera/zoom?tab=head", json=BODY)
    assert resp.status_code == 200
    body = resp.json()
    assert body["status"] == status and body["stamp_ns"] == STAMP_NS
    assert base64.b64decode(body["jpeg_b64"]) == JPEG
    assert bridge.calls == [("head", 0.62, 0.41, 4)]


def test_api_zoom_unknown_tab_and_tab_without_zoom_404(tmp_path: Path) -> None:
    client = make_client(tmp_path, FakeBridge(outcome()))
    for tab in ("nope", "nozoom", "map"):
        assert client.post(f"/api/camera/zoom?tab={tab}", json=BODY).status_code == 404


@pytest.mark.parametrize(
    "body",
    [{}, {"u": 1.5, "v": 0.5, "zoom": 2}, {"u": 0.5, "v": -0.1, "zoom": 2}, {"u": 0.5, "v": 0.5, "zoom": 3}, "x"],
)
def test_api_zoom_bad_body_422(tmp_path: Path, body: Any) -> None:
    bridge = FakeBridge(outcome())
    assert make_client(tmp_path, bridge).post("/api/camera/zoom?tab=head", json=body).status_code == 422
    assert bridge.calls == []


def test_api_zoom_non_json_body_422(tmp_path: Path) -> None:
    resp = make_client(tmp_path, FakeBridge(outcome())).post("/api/camera/zoom?tab=head", content=b"{oops")
    assert resp.status_code == 422


def test_api_zoom_503_without_bridge_or_camera_node(tmp_path: Path) -> None:
    assert make_client(tmp_path, None).post("/api/camera/zoom?tab=head", json=BODY).status_code == 503
    assert make_client(tmp_path, FakeBridge(none=True)).post("/api/camera/zoom?tab=head", json=BODY).status_code == 503


def test_api_zoom_busy_409(tmp_path: Path) -> None:
    resp = make_client(tmp_path, FakeBridge(outcome("busy", None))).post("/api/camera/zoom?tab=head", json=BODY)
    assert resp.status_code == 409 and resp.json()["status"] == "busy"


@pytest.mark.parametrize("status", ["timeout", "camera_error", "invalid"])
def test_api_zoom_failures_502_with_result(tmp_path: Path, status: str) -> None:
    resp = make_client(tmp_path, FakeBridge(outcome(status, None))).post("/api/camera/zoom?tab=head", json=BODY)
    assert resp.status_code == 502 and resp.json()["status"] == status


def test_api_zoom_504_after_zoom_timeout(tmp_path: Path) -> None:
    resp = make_client(tmp_path, FakeBridge(never=True), timeout=0.05).post("/api/camera/zoom?tab=head", json=BODY)
    assert resp.status_code == 504


def test_api_zoom_not_blocked_by_battery_and_writes_nothing(tmp_path: Path) -> None:
    guard = MagicMock()
    guard.is_cutoff.return_value = True
    before = set(tmp_path.rglob("*"))
    resp = make_client(tmp_path, FakeBridge(outcome()), battery_guard=guard).post(
        "/api/camera/zoom?tab=head", json=BODY
    )
    assert resp.status_code == 200
    assert set(tmp_path.rglob("*")) == before
