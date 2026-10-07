"""Tests for mcp_server.poi_client: request/response matching by request_id, timeouts, missing store."""

import json
import threading

import pytest

from mcp_server.models import RobotError
from mcp_server.poi_client import PoiRequests, parse_poi_list

TIMEOUT = 0.3


def make(published: list[dict], store_up: bool = True, responder=None) -> PoiRequests:
    holder: dict[str, PoiRequests] = {}

    def publish(payload: str) -> None:
        msg = json.loads(payload)
        published.append(msg)
        if responder is not None:
            responder(holder["client"], msg)

    holder["client"] = PoiRequests(publish, lambda: store_up, TIMEOUT)
    return holder["client"]


def ok_result(client: PoiRequests, msg: dict, **extra) -> None:
    client.on_result(
        json.dumps({"request_id": msg["request_id"], "ok": True, "message": "added", "poi": msg["poi"]} | extra)
    )


def test_request_publishes_op_poi_and_request_id_and_returns_matching_result() -> None:
    published: list[dict] = []
    client = make(published, responder=ok_result)
    result = client.request("add", {"name": "dock", "kind": "point"})
    assert result["ok"] is True and result["poi"]["name"] == "dock"
    (msg,) = published
    assert msg["op"] == "add" and msg["poi"] == {"name": "dock", "kind": "point"} and msg["request_id"]


def test_each_request_gets_a_fresh_request_id() -> None:
    published: list[dict] = []
    client = make(published, responder=ok_result)
    client.request("add", {})
    client.request("add", {})
    assert published[0]["request_id"] != published[1]["request_id"]


def test_results_for_other_requests_are_ignored() -> None:
    published: list[dict] = []

    def noisy(client: PoiRequests, msg: dict) -> None:
        client.on_result(json.dumps({"request_id": "someone-else", "ok": False, "message": "x", "poi": None}))
        ok_result(client, msg)

    client = make(published, responder=noisy)
    assert client.request("delete", {"id": "a"})["ok"] is True


def test_answer_arriving_from_another_thread_is_matched() -> None:
    published: list[dict] = []

    def later(client: PoiRequests, msg: dict) -> None:
        threading.Timer(0.05, lambda: ok_result(client, msg)).start()

    assert make(published, responder=later).request("add", {})["ok"] is True


def test_timeout_raises_and_forgets_the_request() -> None:
    client = make([], responder=None)
    with pytest.raises(RobotError, match="did not answer"):
        client.request("add", {})
    assert client.pending_count() == 0


def test_store_not_running_is_an_error_without_publishing() -> None:
    published: list[dict] = []
    client = make(published, store_up=False)
    with pytest.raises(RobotError, match="poi_store is not running"):
        client.request("add", {})
    assert published == []


def test_store_rejection_is_raised_with_its_message() -> None:
    def reject(client: PoiRequests, msg: dict) -> None:
        client.on_result(
            json.dumps({"request_id": msg["request_id"], "ok": False, "message": "unknown poi id", "poi": None})
        )

    with pytest.raises(RobotError, match="unknown poi id"):
        make([], responder=reject).request("update", {"id": "zz"})


def test_malformed_result_is_dropped() -> None:
    client = make([])
    client.on_result("{not json")
    client.on_result(json.dumps({"ok": True}))
    assert client.pending_count() == 0


def test_parse_poi_list() -> None:
    parsed = parse_poi_list(json.dumps({"pois": [{"id": "a"}], "revision": 3}))
    assert parsed == ([{"id": "a"}], 3)
    with pytest.raises(ValueError):
        parse_poi_list("[]")
