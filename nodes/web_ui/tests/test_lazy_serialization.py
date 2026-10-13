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
