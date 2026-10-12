"""Tests for the GPS status feature: JSON parsing, base poll fetch/poller and the bridge callback."""

from __future__ import annotations

import json
import threading
from types import SimpleNamespace
from typing import Any
from unittest.mock import patch

import httpx
import pytest

from web_ui.gps_status import (
    GPS_BASE_STATUS_KEY,
    BasePoller,
    fetch_base_status,
    parse_status_json,
)

URL = "http://server.ros2.lan:5016/status"
BASE = {
    "role": "base",
    "quality": 4,
    "fix": "RTK Fixed",
    "num_satellites": 18,
    "hdop": 0.7,
    "ntrip_clients": 1,
    "survey_state": "fixed_position",
    "gga_seq": 7,
}


def client_for(handler: Any) -> httpx.Client:
    return httpx.Client(transport=httpx.MockTransport(handler))


def test_base_status_key() -> None:
    assert GPS_BASE_STATUS_KEY == "/web_ui/gps_base_status"


def test_parse_status_json_valid() -> None:
    assert parse_status_json('{"quality":4}') == {"quality": 4}


@pytest.mark.parametrize("raw", ["not json", "[1,2]", "null", "", '"str"'])
def test_parse_status_json_invalid(raw: str) -> None:
    assert parse_status_json(raw) is None


def test_fetch_success_parses_caster_status_json() -> None:
    client = client_for(lambda req: httpx.Response(200, json=BASE))
    out = fetch_base_status(client, URL, 2.0)
    assert out["reachable"] is True
    assert out["has_status"] is True
    assert out["fix"] == "RTK Fixed"
    assert out["num_satellites"] == 18
    assert out["survey_state"] == "fixed_position"
    assert out["sample_seq"] == 7
    assert isinstance(out["received_at"], float)


def test_fetch_requests_the_configured_url() -> None:
    seen: list[str] = []

    def handler(req: httpx.Request) -> httpx.Response:
        seen.append(str(req.url))
        return httpx.Response(200, json=BASE)

    fetch_base_status(client_for(handler), URL, 2.0)
    assert seen == [URL]


def test_fetch_reachable_base_without_status_yet() -> None:
    client = client_for(lambda req: httpx.Response(503, json={"error": "no status yet"}))
    out = fetch_base_status(client, URL, 2.0)
    assert out == {"reachable": True, "has_status": False, "reason": "no status yet"}


def test_fetch_other_http_status_is_reachable_without_status() -> None:
    client = client_for(lambda req: httpx.Response(404, text="Not Found"))
    out = fetch_base_status(client, URL, 2.0)
    assert out == {"reachable": True, "has_status": False, "reason": "HTTP 404"}


def test_fetch_timeout() -> None:
    def handler(req: httpx.Request) -> httpx.Response:
        raise httpx.ReadTimeout("slow", request=req)

    assert fetch_base_status(client_for(handler), URL, 2.0) == {"reachable": False, "error": "timeout"}


def test_fetch_connection_error() -> None:
    def handler(req: httpx.Request) -> httpx.Response:
        raise httpx.ConnectError("refused", request=req)

    out = fetch_base_status(client_for(handler), URL, 2.0)
    assert out == {"reachable": False, "error": "connection error"}


@pytest.mark.parametrize("body", ["garbage", "[1, 2]", "null"])
def test_fetch_bad_payload_has_no_fabricated_values(body: str) -> None:
    client = client_for(lambda req: httpx.Response(200, text=body))
    out = fetch_base_status(client, URL, 2.0)
    assert out == {"reachable": True, "has_status": False, "reason": "invalid payload"}


def test_poller_stores_each_poll_and_flags_stale_when_seq_stalls() -> None:
    stored: list[dict[str, Any]] = []
    clock = iter([0.0, 1.0, 10.0])
    client = client_for(lambda req: httpx.Response(200, json=BASE))
    poller = BasePoller(URL, 1.0, 2.0, 5.0, lambda data: stored.append(data), client=client, clock=lambda: next(clock))
    for _ in range(3):
        poller.poll_once()
    assert [s["stale"] for s in stored] == [False, False, True]
    assert all(s["reachable"] for s in stored)


def test_poller_not_stale_while_gga_seq_advances() -> None:
    stored: list[dict[str, Any]] = []
    clock = iter([0.0, 10.0, 20.0])
    seqs = iter([1, 2, 3])
    client = client_for(lambda req: httpx.Response(200, json={**BASE, "gga_seq": next(seqs)}))
    poller = BasePoller(URL, 1.0, 2.0, 5.0, stored.append, client=client, clock=lambda: next(clock))
    for _ in range(3):
        poller.poll_once()
    assert [s["stale"] for s in stored] == [False, False, False]


def test_poller_keeps_base_position_in_stored_payload() -> None:
    stored: list[dict[str, Any]] = []
    status = {**BASE, "latitude": 52.1, "longitude": 21.0, "altitude": 110.5}
    client = client_for(lambda req: httpx.Response(200, json=status))
    BasePoller(URL, 1.0, 2.0, 5.0, stored.append, client=client).poll_once()
    assert (stored[0]["latitude"], stored[0]["longitude"], stored[0]["altitude"]) == (52.1, 21.0, 110.5)


def test_poller_unreachable_has_no_stale_or_fix_fields() -> None:
    stored: list[dict[str, Any]] = []

    def handler(req: httpx.Request) -> httpx.Response:
        raise httpx.ConnectError("refused", request=req)

    BasePoller(URL, 1.0, 2.0, 5.0, stored.append, client=client_for(handler)).poll_once()
    assert stored == [{"reachable": False, "error": "connection error"}]


def test_poller_no_status_yet_has_no_stale_or_fix_fields() -> None:
    stored: list[dict[str, Any]] = []
    client = client_for(lambda req: httpx.Response(503, json={"error": "no status yet"}))
    BasePoller(URL, 1.0, 2.0, 5.0, stored.append, client=client).poll_once()
    assert stored == [{"reachable": True, "has_status": False, "reason": "no status yet"}]


def test_poller_run_stops_on_event() -> None:
    stored: list[dict[str, Any]] = []
    stop = threading.Event()

    def on_status(data: dict[str, Any]) -> None:
        stored.append(data)
        stop.set()

    client = client_for(lambda req: httpx.Response(200, json=BASE))
    BasePoller(URL, 50.0, 2.0, 5.0, on_status, client=client).run(stop)
    assert len(stored) == 1


# ----------------------------------------------------------------- bridge


def make_bridge() -> Any:
    with patch("web_ui.bridge.rclpy"), patch("web_ui.bridge.Node.__init__", return_value=None):
        from web_ui.bridge import BridgeNode

        node = BridgeNode.__new__(BridgeNode)
        node._latest = {}
        node._dirty = set()
        node._cleared = set()
        node._lock = threading.Lock()
        node._topic_last_rx = {}
        return node


def test_bridge_gps_status_stores_parsed_json() -> None:
    node = make_bridge()
    node.on_gps_status("/client/gps/status", SimpleNamespace(data=json.dumps({"role": "rover", "quality": 4})))
    env = node.flush_dirty()
    assert env == [{"topic": "/client/gps/status", "data": {"role": "rover", "quality": 4}}]


def test_bridge_gps_status_drops_invalid_json() -> None:
    node = make_bridge()
    node.on_gps_status("/client/gps/status", SimpleNamespace(data="{oops"))
    assert node.flush_dirty() == []


def test_bridge_store_base_status_under_synthetic_key() -> None:
    node = make_bridge()
    node.store(GPS_BASE_STATUS_KEY, {"reachable": False, "error": "timeout"})
    assert node.flush_dirty() == [{"topic": GPS_BASE_STATUS_KEY, "data": {"reachable": False, "error": "timeout"}}]
