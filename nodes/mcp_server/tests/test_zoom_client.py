"""Tests for mcp_server.zoom_client: command JSON, result/image pairing by stamp, errors, concurrency."""

import json
import threading
import time

import pytest

from mcp_server.models import RobotError
from mcp_server.zoom_client import ZoomFailed, ZoomNodeDown, ZoomReply, ZoomRequests, ZoomTimeout

TIMEOUT = 0.4
GRACE = 0.2
STAMP_NS = 1_700_000_000_123_456_789
JPEG = b"\xff\xd8jpeg\xff\xd9"


def result_json(msg: dict, status: str = "ok", stamp_ns: int = STAMP_NS, **extra) -> str:
    has_image = status in ("ok", "af_failed")
    body = {
        "request_id": msg["request_id"],
        "status": status,
        "message": "m",
        "af_state": "focused",
        "lens_position": 4.5,
        "crop": [100, 200, 1152, 648] if has_image else None,
        "full": [4608, 2592],
        "out": [1024, 576] if has_image else None,
        "stamp_ns": stamp_ns if has_image else 0,
        "stamp_s": stamp_ns / 1e9 if has_image else 0.0,
        "phase_ms": {"af": 1.0, "switch": 2.0, "capture": 3.0, "restore": 4.0},
    }
    return json.dumps(body | extra)


def make(published: list[dict], up: bool = True, responder=None) -> ZoomRequests:
    holder: dict[str, ZoomRequests] = {}

    def publish(payload: str) -> None:
        msg = json.loads(payload)
        published.append(msg)
        if responder is not None:
            responder(holder["client"], msg)

    holder["client"] = ZoomRequests(publish, lambda: up, TIMEOUT, GRACE)
    return holder["client"]


def result_then_image(client: ZoomRequests, msg: dict) -> None:
    client.on_result(result_json(msg))
    client.on_image(STAMP_NS, JPEG)


def image_then_result(client: ZoomRequests, msg: dict) -> None:
    client.on_image(STAMP_NS, JPEG)
    client.on_result(result_json(msg))


def test_published_json_has_exactly_the_request_keys() -> None:
    published: list[dict] = []
    client = make(published, responder=result_then_image)
    client.request(0.62, 0.41, 4, 1024)
    (msg,) = published
    assert set(msg) == {"request_id", "u", "v", "zoom", "max_px", "af"}
    assert (msg["u"], msg["v"], msg["zoom"], msg["max_px"], msg["af"]) == (0.62, 0.41, 4, 1024, True)
    assert msg["request_id"]


def test_af_false_is_published() -> None:
    published: list[dict] = []
    make(published, responder=result_then_image).request(0.5, 0.5, 2, 512, af=False)
    assert published[0]["af"] is False


@pytest.mark.parametrize("responder", [result_then_image, image_then_result])
def test_result_and_image_in_either_order_resolve(responder) -> None:
    client = make([], responder=responder)
    reply = client.request(0.5, 0.5, 4, 1024)
    assert isinstance(reply, ZoomReply)
    assert reply.jpeg == JPEG and reply.status == "ok"
    assert reply.crop == [100, 200, 1152, 648] and reply.full == [4608, 2592] and reply.out == [1024, 576]
    assert reply.stamp_ns == STAMP_NS and reply.af_state == "focused" and reply.lens_position == 4.5
    assert reply.phase_ms["capture"] == 3.0
    assert client.pending_count() == 0


def test_af_failed_returns_the_image() -> None:
    def responder(client: ZoomRequests, msg: dict) -> None:
        client.on_image(STAMP_NS, JPEG)
        client.on_result(result_json(msg, "af_failed", af_state="failed"))

    reply = make([], responder=responder).request(0.5, 0.5, 4, 1024)
    assert reply.status == "af_failed" and reply.jpeg == JPEG


def test_image_arriving_after_the_result_within_grace_resolves() -> None:
    def responder(client: ZoomRequests, msg: dict) -> None:
        client.on_result(result_json(msg))
        threading.Timer(0.05, client.on_image, args=(STAMP_NS, JPEG)).start()

    assert make([], responder=responder).request(0.5, 0.5, 4, 1024).jpeg == JPEG


@pytest.mark.parametrize("status", ["busy", "timeout", "camera_error", "invalid"])
def test_failure_statuses_raise_zoom_failed(status: str) -> None:
    def responder(client: ZoomRequests, msg: dict) -> None:
        client.on_result(result_json(msg, status, message=f"because {status}"))

    with pytest.raises(ZoomFailed) as exc:
        make([], responder=responder).request(0.5, 0.5, 4, 1024)
    assert exc.value.status == status and f"because {status}" in str(exc.value)
    assert isinstance(exc.value, RobotError)


def test_no_result_times_out() -> None:
    client = make([])
    started = time.monotonic()
    with pytest.raises(ZoomTimeout):
        client.request(0.5, 0.5, 4, 1024)
    assert time.monotonic() - started >= TIMEOUT
    assert client.pending_count() == 0


def test_ok_result_without_image_times_out_after_grace() -> None:
    def responder(client: ZoomRequests, msg: dict) -> None:
        client.on_result(result_json(msg))

    started = time.monotonic()
    with pytest.raises(ZoomTimeout, match="image"):
        make([], responder=responder).request(0.5, 0.5, 4, 1024)
    assert time.monotonic() - started < TIMEOUT + GRACE + 0.5


def test_image_with_other_stamp_does_not_match() -> None:
    def responder(client: ZoomRequests, msg: dict) -> None:
        client.on_image(STAMP_NS + 1, JPEG)
        client.on_result(result_json(msg))

    with pytest.raises(ZoomTimeout):
        make([], responder=responder).request(0.5, 0.5, 4, 1024)


def test_no_subscriber_raises_node_down_without_publishing() -> None:
    published: list[dict] = []
    with pytest.raises(ZoomNodeDown):
        make(published, up=False).request(0.5, 0.5, 4, 1024)
    assert published == []


def test_unknown_ids_and_malformed_payloads_are_ignored() -> None:
    def responder(client: ZoomRequests, msg: dict) -> None:
        client.on_result("not json")
        client.on_result("[1, 2]")
        client.on_result(json.dumps({"status": "ok"}))
        client.on_result(json.dumps({"request_id": "other", "status": "busy", "message": ""}))
        client.on_result(json.dumps({"request_id": msg["request_id"], "status": "ok"}))  # missing fields
        result_then_image(client, msg)

    assert make([], responder=responder).request(0.5, 0.5, 4, 1024).status == "ok"


def test_concurrent_requests_resolve_independently() -> None:
    ids = iter(["a", "b"])
    holder: dict[str, ZoomRequests] = {}
    published: list[dict] = []
    both = threading.Barrier(2, timeout=2)
    replies: dict[str, object] = {}

    def publish(payload: str) -> None:
        published.append(json.loads(payload))

    client = ZoomRequests(publish, lambda: True, 2.0, GRACE, new_id=lambda: next(ids))
    holder["c"] = client

    def run(name: str, u: float) -> None:
        both.wait()
        try:
            replies[name] = client.request(u, 0.5, 4, 1024)
        except RobotError as exc:
            replies[name] = exc

    threads = [threading.Thread(target=run, args=(n, u)) for n, u in (("t1", 0.1), ("t2", 0.9))]
    for t in threads:
        t.start()
    deadline = time.monotonic() + 2
    while len(published) < 2 and time.monotonic() < deadline:
        time.sleep(0.01)
    by_u = {m["u"]: m for m in published}
    client.on_result(result_json(by_u[0.9], "busy"))
    client.on_image(STAMP_NS + 5, b"\xff\xd8two")
    client.on_result(result_json(by_u[0.1], stamp_ns=STAMP_NS + 5))
    for t in threads:
        t.join(3)
    assert isinstance(replies["t2"], ZoomFailed) and replies["t2"].status == "busy"
    assert replies["t1"].jpeg == b"\xff\xd8two"


def test_only_the_last_four_images_are_kept() -> None:
    client = make([])
    for i in range(6):
        client.on_image(i + 1, b"x")
    assert len(client.images) == 4 and 1 not in client.images and 6 in client.images
