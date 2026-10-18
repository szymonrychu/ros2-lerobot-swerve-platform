"""Tests for the map_guard reset subscription: /map_guard/map_reset drops the GPS anchor and the cached map."""

from __future__ import annotations

import threading
from typing import Any
from unittest.mock import MagicMock, patch


def make_bridge(**attrs: Any) -> Any:
    """Construct a BridgeNode without running rclpy initialisation."""
    from web_ui.bridge import BridgeNode

    node = BridgeNode.__new__(BridgeNode)
    node._latest = {}
    node._dirty = set()
    node._pending = {}
    node._cleared = set()
    node._lock = threading.Lock()
    node._gps_anchor = None
    node._map_topics = []
    for key, value in attrs.items():
        setattr(node, key, value)
    return node


def test_bridge_subscribes_to_the_map_guard_reset_topic() -> None:
    from web_ui.bridge import MAP_GUARD_RESET_TOPIC, BridgeNode, Empty

    created: list[tuple[Any, str, Any]] = []
    with (
        patch("web_ui.bridge.Node.__init__", return_value=None),
        patch.object(
            BridgeNode,
            "create_subscription",
            create=True,
            side_effect=lambda c, t, cb, q: created.append((c, t, cb)),
        ),
        patch.object(BridgeNode, "create_timer", create=True),
        patch.object(BridgeNode, "create_publisher", create=True),
        patch.object(BridgeNode, "get_logger", create=True),
        patch.object(BridgeNode, "create_client", create=True),
    ):
        node = BridgeNode(topics=["/map"], allowed_publish_topics=set(), topic_roles={"/map": "map"})
    matches = [(c, cb) for c, t, cb in created if t == MAP_GUARD_RESET_TOPIC]
    assert MAP_GUARD_RESET_TOPIC == "/map_guard/map_reset"
    assert len(matches) == 1
    assert matches[0][0] is Empty
    assert matches[0][1] == node.on_map_guard_reset
    assert node._map_topics == ["/map"]


def test_map_guard_reset_drops_the_gps_anchor_and_the_map() -> None:
    from web_ui.bridge import GPS_ANCHOR_TOPIC

    anchor = MagicMock()
    node = make_bridge(_gps_anchor=anchor, _map_topics=["/map"])
    node._latest["/map"] = {"topic": "/map", "data": {"png_b64": "x"}}
    node._latest[GPS_ANCHOR_TOPIC] = {"topic": GPS_ANCHOR_TOPIC, "data": {"lat": 1.0}}
    node.on_map_guard_reset(object())
    anchor.reset.assert_called_once()
    envelopes = {env["topic"]: env["data"] for env in node.flush_dirty()}
    assert envelopes == {"/map": None, GPS_ANCHOR_TOPIC: None}
    assert node.latest_envelopes() == []


def test_map_guard_reset_without_anchor_still_clears() -> None:
    from web_ui.bridge import GPS_ANCHOR_TOPIC

    node = make_bridge()
    node.on_map_guard_reset(object())
    assert {env["topic"] for env in node.flush_dirty()} == {GPS_ANCHOR_TOPIC}
