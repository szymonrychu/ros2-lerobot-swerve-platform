"""Tests for lazy serialization in BridgeNode and CompressedImage JPEG pass-through."""

from __future__ import annotations

import base64
import threading
from typing import Any
from unittest.mock import MagicMock, patch

import cv2
import numpy as np

from web_ui.bridge import BridgeNode, subscription_spec
from web_ui.msg_serializer import RGBD_COLOR_TOPICS, msg_to_dict

TOPIC = "/lazy/topic"
RGBD_TOPIC = next(iter(RGBD_COLOR_TOPICS))


def make_node() -> BridgeNode:
    """BridgeNode with just the state callbacks, store and flush need."""
    with patch("web_ui.bridge.rclpy"), patch("web_ui.bridge.Node.__init__", return_value=None):
        node = BridgeNode.__new__(BridgeNode)
    node._latest = {}
    node._pending = {}
    node._color_small = False
    node._dirty = set()
    node._cleared = set()
    node._lock = threading.Lock()
    node._topic_last_rx = {}
    node._robot_pose_frames = None
    node._tf_buffer = None
    return node


class CountingSerializer:
    """Serializer spy that records every message it is asked to serialize."""

    def __init__(self) -> None:
        self.messages: list[Any] = []

    def __call__(self, msg: Any) -> dict[str, Any]:
        self.messages.append(msg)
        return {"value": msg}


def make_jpeg_msg(width: int = 64, height: int = 48) -> Any:
    """CompressedImage-like message carrying a real JPEG."""
    img = np.random.default_rng(0).integers(0, 255, (height, width, 3), dtype=np.uint8)
    ok, buf = cv2.imencode(".jpg", img)
    assert ok
    msg = MagicMock()
    msg.__class__.__name__ = "CompressedImage"
    msg.format = "jpeg"
    msg.data = buf.tobytes()
    return msg


def test_callback_only_stores_raw_message() -> None:
    node = make_node()
    serializer = CountingSerializer()
    cb = node._make_callback(TOPIC, serializer, None)
    cb(1)
    cb(2)
    assert serializer.messages == []
    assert node._latest == {}
    assert TOPIC in node._dirty


def test_flush_serializes_once_with_latest_of_n_messages() -> None:
    node = make_node()
    serializer = CountingSerializer()
    cb = node._make_callback(TOPIC, serializer, None)
    for value in range(10):
        cb(value)
    envelopes = node.flush_dirty()
    assert serializer.messages == [9]
    assert envelopes == [{"topic": TOPIC, "data": {"value": 9}}]
    assert node.flush_dirty() == []
    assert serializer.messages == [9]


def test_no_client_connected_serializes_nothing() -> None:
    node = make_node()
    serializer = CountingSerializer()
    for value in range(5):
        node._make_callback(TOPIC, serializer, None)(value)
    assert serializer.messages == []
    assert node.flush_dirty() == [{"topic": TOPIC, "data": {"value": 4}}]
    assert serializer.messages == [4]


def test_snapshot_serializes_pending_once_and_flush_reuses_it() -> None:
    node = make_node()
    serializer = CountingSerializer()
    node._make_callback(TOPIC, serializer, None)(7)
    snapshot = node.latest_envelopes()
    assert snapshot == [{"topic": TOPIC, "data": {"value": 7}}]
    assert node.latest_envelopes() == snapshot
    assert node.flush_dirty() == snapshot
    assert serializer.messages == [7]


def test_only_dirty_topics_are_serialized() -> None:
    node = make_node()
    first, second = CountingSerializer(), CountingSerializer()
    node._make_callback("/a", first, None)(1)
    node._make_callback("/b", second, None)(2)
    node.flush_dirty()
    node._make_callback("/a", first, None)(3)
    node.flush_dirty()
    assert first.messages == [1, 3]
    assert second.messages == [2]


def test_serialize_error_is_dropped_and_keeps_previous_envelope() -> None:
    node = make_node()
    calls: list[int] = []

    def serializer(msg: int) -> dict[str, Any]:
        calls.append(msg)
        if msg == 2:
            raise ValueError("boom")
        return {"value": msg}

    cb = node._make_callback(TOPIC, serializer, None)
    cb(1)
    node.flush_dirty()
    cb(2)
    assert node.flush_dirty() == []
    assert node.latest_envelopes() == [{"topic": TOPIC, "data": {"value": 1}}]


def test_compressed_jpeg_is_passed_through_byte_equal() -> None:
    msg = make_jpeg_msg()
    node = make_node()
    _, _, serializer = subscription_spec("/h/rgb", "head_rgb")
    node._make_callback("/h/rgb", serializer, "head_rgb")(msg)
    with patch("web_ui.msg_serializer.cv2.imdecode") as dec, patch("web_ui.msg_serializer.cv2.imencode") as enc:
        envelopes = node.flush_dirty()
    dec.assert_not_called()
    enc.assert_not_called()
    assert base64.b64decode(envelopes[0]["data"]["jpeg_b64"]) == msg.data


def test_compressed_non_jpeg_is_still_decoded_and_reencoded() -> None:
    img = np.zeros((8, 8, 3), dtype=np.uint8)
    ok, buf = cv2.imencode(".png", img)
    assert ok
    msg = MagicMock()
    msg.__class__.__name__ = "CompressedImage"
    msg.format = "png"
    msg.data = buf.tobytes()
    result = msg_to_dict(msg, topic="/h/rgb")
    assert base64.b64decode(result["jpeg_b64"])[:2] == b"\xff\xd8"


def test_color_small_only_decodes_when_requested() -> None:
    msg = make_jpeg_msg()
    with patch("web_ui.msg_serializer.cv2.imdecode", wraps=cv2.imdecode) as dec:
        plain = msg_to_dict(msg, topic=RGBD_TOPIC, color_small=False)
        assert dec.call_count == 0
        full = msg_to_dict(msg, topic=RGBD_TOPIC)
        assert dec.call_count == 1
    assert "color_small_b64" not in plain
    assert base64.b64decode(plain["jpeg_b64"]) == msg.data
    assert full["color_small_b64"] is not None
    assert base64.b64decode(full["jpeg_b64"]) == msg.data


def test_color_small_not_produced_for_other_topics() -> None:
    result = msg_to_dict(make_jpeg_msg(), topic="/other/image")
    assert "color_small_b64" not in result


def test_rgbd_frame_is_not_decoded_before_a_flush() -> None:
    node = make_node()
    _, _, serializer = subscription_spec(RGBD_TOPIC, None)
    msg = make_jpeg_msg()
    msg.__class__.__name__ = "Image"
    with patch("web_ui.msg_serializer.cv2.imencode") as enc, patch("web_ui.msg_serializer.cv2.imdecode") as dec:
        node._make_callback(RGBD_TOPIC, serializer, None)(msg)
    enc.assert_not_called()
    dec.assert_not_called()


# --- per-client subscription gating ------------------------------------------------------------------------------


def make_conn(topics: Any = None, color_small: bool = False) -> Any:
    """ClientConnection stand-in with a subscription."""
    from web_ui.ws_clients import ClientConnection

    conn = ClientConnection(MagicMock(), on_dead=lambda _c: None)
    conn.topics = None if topics is None else frozenset(topics)
    conn.color_small = color_small
    return conn


def run_broadcaster(clients: dict[str, Any], bridge: Any) -> None:
    """Run the real broadcast loop for a few ticks."""
    import asyncio

    from web_ui.server import _make_start_broadcaster

    async def main() -> None:
        start = _make_start_broadcaster(MagicMock(), clients, bridge, 0.01, MagicMock(), {})
        await start()
        await asyncio.sleep(0.1)
        for task in asyncio.all_tasks() - {asyncio.current_task()}:
            task.cancel()

    asyncio.run(main())


def test_broadcaster_with_no_client_never_flushes_or_serializes() -> None:
    bridge = MagicMock()
    run_broadcaster({}, bridge)
    bridge.flush_dirty.assert_not_called()
    bridge.latest_envelopes.assert_not_called()


def test_broadcaster_flushes_only_the_union_of_client_subscriptions() -> None:
    bridge = MagicMock()
    bridge.flush_dirty.return_value = []
    clients = {"a": make_conn({"/x", "/y"}), "b": make_conn({"/y", "/z"})}
    run_broadcaster(clients, bridge)
    assert bridge.flush_dirty.call_args.args[0] == {"/x", "/y", "/z"}
    bridge.set_color_small.assert_called_with(False)


def test_broadcaster_color_small_follows_any_client_head_tab() -> None:
    bridge = MagicMock()
    bridge.flush_dirty.return_value = []
    run_broadcaster({"a": make_conn({"/x"}), "b": make_conn({"/x"}, color_small=True)}, bridge)
    bridge.set_color_small.assert_called_with(True)


def test_client_without_subscribe_message_gets_every_topic() -> None:
    bridge = MagicMock()
    bridge.flush_dirty.return_value = []
    run_broadcaster({"a": make_conn(None), "b": make_conn({"/x"})}, bridge)
    assert bridge.flush_dirty.call_args.args[0] is None


def test_unsubscribed_topic_is_not_serialized_but_stays_pending_for_a_later_subscriber() -> None:
    node = make_node()
    wanted, unwanted = CountingSerializer(), CountingSerializer()
    node._make_callback("/a", wanted, None)(1)
    node._make_callback("/b", unwanted, None)(2)
    assert node.flush_dirty({"/a"}) == [{"topic": "/a", "data": {"value": 1}}]
    assert unwanted.messages == []
    assert node.flush_dirty({"/a"}) == []
    assert node.latest_envelopes({"/b"}) == [{"topic": "/b", "data": {"value": 2}}]
    assert unwanted.messages == [2]


def test_color_small_skipped_unless_a_head_tab_needs_it() -> None:
    node = make_node()
    _, _, serializer = subscription_spec(RGBD_TOPIC, None)
    msg = make_jpeg_msg()
    node._make_callback(RGBD_TOPIC, serializer, None)(msg)
    with patch("web_ui.msg_serializer.cv2.imdecode", wraps=cv2.imdecode) as dec:
        plain = node.flush_dirty()[0]["data"]
    dec.assert_not_called()
    assert "color_small_b64" not in plain
    node.set_color_small(True)
    node._make_callback(RGBD_TOPIC, serializer, None)(msg)
    assert node.flush_dirty()[0]["data"]["color_small_b64"] is not None


def test_broadcast_offers_each_envelope_only_to_subscribed_clients() -> None:
    import asyncio

    from web_ui.server import broadcast_envelopes

    a, b = make_conn({"/x"}), make_conn({"/y"})
    envelopes = [{"topic": "/x", "data": {"v": 1}}, {"topic": "/y", "data": {"v": 2}}]
    asyncio.run(broadcast_envelopes(envelopes, {"a": a, "b": b}, {}, MagicMock()))
    assert set(a.mailbox) == {"/x"}
    assert set(b.mailbox) == {"/y"}


def test_subscribe_message_sets_subscription_and_sends_snapshot_of_new_topics(tmp_path: Any, urdf_dir: Any) -> None:
    from fastapi.testclient import TestClient

    from web_ui.config import AppConfig, TabConfig
    from web_ui.server import build_app

    bridge = MagicMock()
    bridge.latest_envelopes.side_effect = lambda topics=None: (
        [{"topic": "/x", "data": {"v": 1}}] if topics == {"/x"} else []
    )
    bridge.tilt_state.return_value = None
    config = AppConfig(tabs=[TabConfig(id="map", type="map_nav", label="Map", goal_topic="/goal")])
    app = build_app(config=config, urdf_dir=urdf_dir, static_dir=tmp_path / "none", bridge_node=bridge)
    with patch("web_ui.server.WS_RECONNECT_MIN_INTERVAL_S", 0.0), TestClient(app).websocket_connect("/ws") as ws:
        ws.send_json({"type": "subscribe", "topics": ["/x"], "color_small": True})
        assert ws.receive_json() == {"topic": "/x", "data": {"v": 1}}
