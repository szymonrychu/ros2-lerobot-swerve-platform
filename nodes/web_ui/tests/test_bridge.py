"""Tests for BridgeNode dirty-flag store and publish allowlist."""

from __future__ import annotations

import threading
from unittest.mock import MagicMock, patch


def _make_bridge_node(topics=None, publish_topics=None):
    """Construct a BridgeNode with rclpy mocked out."""
    topics = topics or []
    publish_topics = publish_topics or []
    with patch("web_ui.bridge.rclpy"), patch("web_ui.bridge.Node.__init__", return_value=None):
        from web_ui.bridge import BridgeNode

        node = BridgeNode.__new__(BridgeNode)
        node._latest = {}
        node._dirty = set()
        node._pending = {}
        node._color_small = False
        node._cleared = set()
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


def test_overview_camera_topic_is_a_compressed_image_subscription() -> None:
    """The Overview camera tab topic resolves to a CompressedImage subscription without a role."""
    from sensor_msgs.msg import CompressedImage

    from web_ui.bridge import subscription_spec

    assert subscription_spec("/overview_camera/image_raw/compressed", None)[0] is CompressedImage


def test_stereo_topics_have_type_hints() -> None:
    """The stereo RGBD tab topics resolve to Image/CameraInfo subscriptions without a role."""
    from sensor_msgs.msg import CameraInfo, Image

    from web_ui.bridge import subscription_spec

    assert subscription_spec("/stereo/left/image_rect", None)[0] is Image
    assert subscription_spec("/stereo/depth/image_rect", None)[0] is Image
    assert subscription_spec("/stereo/depth/camera_info", None)[0] is CameraInfo


def test_odom_topic_is_an_odometry_subscription() -> None:
    """The client /odom topic (status bar Vel overlay) resolves to an Odometry subscription without a role."""
    from nav_msgs.msg import Odometry

    from web_ui.bridge import subscription_spec

    assert subscription_spec("/odom", None)[0] is Odometry


def test_head_roles_pick_message_types_for_arbitrary_topic_names() -> None:
    """head roles subscribe RGB as CompressedImage and ToF depth and amplitude as Image on any topic name."""
    from sensor_msgs.msg import CompressedImage, Image

    from web_ui.bridge import subscription_spec

    assert subscription_spec("/any/rgb", "head_rgb")[0] is CompressedImage
    assert subscription_spec("/any/depth", "head_depth", 2.0)[0] is Image
    assert subscription_spec("/any/amp", "head_amplitude")[0] is Image


def test_bridge_subscribes_head_topics() -> None:
    """The bridge creates one subscription per head topic with the right message type."""
    from sensor_msgs.msg import CompressedImage, Image

    created: list[tuple[type, str]] = []

    from web_ui.bridge import BridgeNode

    with (
        patch("web_ui.bridge.Node.__init__", return_value=None),
        patch.object(
            BridgeNode, "create_subscription", create=True, side_effect=lambda c, t, cb, q: created.append((c, t))
        ),
        patch.object(BridgeNode, "create_timer", create=True),
        patch.object(BridgeNode, "create_publisher", create=True),
        patch.object(BridgeNode, "get_logger", create=True),
        patch.object(BridgeNode, "create_client", create=True),
    ):
        BridgeNode(
            topics=["/h/rgb", "/h/depth", "/h/amp"],
            allowed_publish_topics=set(),
            topic_roles={"/h/rgb": "head_rgb", "/h/depth": "head_depth", "/h/amp": "head_amplitude"},
            tof_ranges={"/h/depth": 2.5},
        )
    assert (CompressedImage, "/h/rgb") in created
    assert (Image, "/h/depth") in created
    assert (Image, "/h/amp") in created


def _make_preview_node():
    """BridgeNode with just the state _make_callback and store need."""
    node = _make_bridge_node()
    node._topic_last_rx = {}
    node._robot_pose_frames = None
    node._tf_buffer = None
    return node


def _amplitude_msg():
    """Return a mono8 Image message."""
    import numpy as np
    from sensor_msgs.msg import Image

    msg = Image()
    msg.encoding = "mono8"
    msg.width = 8
    msg.height = 4
    msg.step = 8
    msg.data = np.full((4, 8), 100, dtype=np.uint8).tobytes()
    return msg


def test_head_previews_are_rate_limited_per_topic() -> None:
    """A second head frame inside the preview interval is not serialised; one after it is."""
    from web_ui import bridge

    node = _make_preview_node()
    calls: list[int] = []

    def serializer(msg):
        calls.append(1)
        return {"jpeg_b64": "x"}

    cb = node._make_callback("/h/amp", serializer, "head_amplitude")
    with patch("web_ui.bridge.time.monotonic") as clock:
        clock.return_value = 100.0
        cb(_amplitude_msg())
        clock.return_value = 100.0 + bridge.HEAD_PREVIEW_MIN_INTERVAL_S / 2
        cb(_amplitude_msg())
        node.flush_dirty()
        assert len(calls) == 1
        clock.return_value = 100.0 + bridge.HEAD_PREVIEW_MIN_INTERVAL_S * 1.1
        cb(_amplitude_msg())
        node.flush_dirty()
    assert len(calls) == 2


def test_head_preview_with_decode_error_is_dropped() -> None:
    """A 32FC1 amplitude frame is logged and not stored (no placeholder frame)."""
    from web_ui.bridge import subscription_spec

    node = _make_preview_node()
    _, _, serializer = subscription_spec("/h/amp", "head_amplitude")
    msg = MagicMock()
    msg.__class__.__name__ = "Image"
    msg.encoding = "32FC1"
    msg.width = 2
    msg.height = 2
    msg.step = 8
    msg.data = bytes(16)
    node._make_callback("/h/amp", serializer, "head_amplitude")(msg)
    assert node.flush_dirty() == []
