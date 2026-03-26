"""Tests for BridgeNode dirty-flag store and publish allowlist."""

from __future__ import annotations

import threading
from unittest.mock import patch


def _make_bridge_node(topics=None, publish_topics=None):
    """Construct a BridgeNode with rclpy mocked out."""
    topics = topics or []
    publish_topics = publish_topics or []
    with patch("web_ui.bridge.rclpy"), patch("web_ui.bridge.Node.__init__", return_value=None):
        from web_ui.bridge import BridgeNode

        node = BridgeNode.__new__(BridgeNode)
        node._latest = {}
        node._dirty = set()
        node._lock = threading.Lock()
        node.publishers_ = {}
        node._allowed_publish_topics = set(publish_topics)
        return node


def test_flush_dirty_returns_dirty_topics() -> None:
    node = _make_bridge_node()
    node._latest["/foo"] = {"topic": "/foo", "data": {}}
    node._dirty.add("/foo")
    results = node.flush_dirty()
    assert len(results) == 1
    assert results[0]["topic"] == "/foo"


def test_flush_dirty_clears_after_flush() -> None:
    node = _make_bridge_node()
    node._latest["/foo"] = {"topic": "/foo", "data": {}}
    node._dirty.add("/foo")
    node.flush_dirty()
    assert node.flush_dirty() == []


def test_flush_dirty_only_returns_dirty() -> None:
    node = _make_bridge_node()
    node._latest["/foo"] = {"topic": "/foo", "data": {}}
    node._latest["/bar"] = {"topic": "/bar", "data": {}}
    node._dirty.add("/foo")
    results = node.flush_dirty()
    topics = [r["topic"] for r in results]
    assert "/foo" in topics
    assert "/bar" not in topics


def test_publish_dict_rejects_non_allowlisted() -> None:
    node = _make_bridge_node(publish_topics=["/allowed_topic"])
    with patch.object(node, "_log_warning") as mock_warn:
        node.publish_dict("/evil_topic", {})
    mock_warn.assert_called_once()
    assert "/evil_topic" not in node.publishers_
