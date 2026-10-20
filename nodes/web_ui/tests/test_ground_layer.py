"""Tests for the ground layer backend: tab config keys, roles, labelled cloud serializer and ground state staleness."""

from __future__ import annotations

import base64
import json
import threading
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import patch

import numpy as np
import pytest
import yaml
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

from web_ui import cloud_serializer
from web_ui.bridge import MAP_SUB_QOS, POSE_FRAME_ROLES, ROLE_SPECS, SENSOR_SUB_QOS, subscription_spec
from web_ui.cloud_serializer import serialize_ground_cloud
from web_ui.config import (
    GROUND_CLOUD_ROLE,
    GROUND_STALE_S_DEFAULT,
    GROUND_STATE_ROLE,
    MAP_NAV_TOPIC_ROLES,
    AppConfig,
    TabConfig,
    load_config,
)
from web_ui.msg_serializer import serialize_json_object

NODES = Path(__file__).resolve().parents[2]
DEFAULT_YAML = Path(__file__).resolve().parents[1] / "config" / "default.yaml"
CLIENT_YML = Path(__file__).resolve().parents[3] / "ansible" / "group_vars" / "client.yml"
GROUND_YAML = NODES / "ground_estimator_cpp" / "config" / "ground_estimator.yaml"
STATE_EXAMPLE = NODES / "ground_estimator_cpp" / "test" / "data" / "state_example.json"
CLOUD_TOPIC = "/ground/labelled_points"
STATE_TOPIC = "/ground/state"
FLOAT32 = 7
UINT32 = 6
UINT8 = 2


def make_labelled_cloud(
    points: list[tuple[float, float, float]], labels: list[int] | None, frame: str = "base_link"
) -> Any:
    """PointCloud2-like message: x y z float32, rgb uint32 and (optionally) label uint8, point_step 17."""
    arr = np.zeros(
        len(points),
        dtype=np.dtype(
            {
                "names": ["x", "y", "z", "rgb", "label"],
                "formats": ["<f4", "<f4", "<f4", "<u4", "u1"],
                "offsets": [0, 4, 8, 12, 16],
                "itemsize": 17,
            }
        ),
    )
    for i, (x, y, z) in enumerate(points):
        arr["x"][i], arr["y"][i], arr["z"][i] = x, y, z
        arr["rgb"][i] = (10 << 16) | (20 << 8) | i
        if labels is not None:
            arr["label"][i] = labels[i]
    fields = [SimpleNamespace(name=n, offset=o, datatype=FLOAT32, count=1) for n, o in (("x", 0), ("y", 4), ("z", 8))]
    fields.append(SimpleNamespace(name="rgb", offset=12, datatype=UINT32, count=1))
    if labels is not None:
        fields.append(SimpleNamespace(name="label", offset=16, datatype=UINT8, count=1))
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame, stamp=SimpleNamespace(sec=5, nanosec=0)),
        height=1,
        width=len(points),
        fields=fields,
        is_bigendian=False,
        point_step=17,
        row_step=17 * len(points),
        data=arr.tobytes(),
        is_dense=False,
    )


def decode_labels(payload: dict[str, Any]) -> np.ndarray:
    return np.frombuffer(base64.b64decode(payload["labels_b64"]), dtype=np.uint8)


def decode_points(payload: dict[str, Any]) -> np.ndarray:
    return np.frombuffer(base64.b64decode(payload["points_b64"]), dtype="<i2").reshape(-1, 3)


# --- config ---------------------------------------------------------------------------------------------------------


def test_config_keys_optional_and_roles_wired() -> None:
    tab = TabConfig(id="m", type="map_nav", label="M")
    assert tab.ground_cloud_topic is None
    assert tab.ground_state_topic is None
    assert tab.ground_state_stale_s == GROUND_STALE_S_DEFAULT == 2.0
    with pytest.raises(ValueError):
        TabConfig(id="m", type="map_nav", label="M", ground_state_stale_s=0)
    assert ("ground_cloud_topic", GROUND_CLOUD_ROLE) in MAP_NAV_TOPIC_ROLES
    assert ("ground_state_topic", GROUND_STATE_ROLE) in MAP_NAV_TOPIC_ROLES
    cfg = AppConfig(tabs=[TabConfig(id="m", type="map_nav", label="M")])
    assert not [t for t in cfg.all_subscribed_topics() if "ground" in t]
    assert cfg.ground_state_stale_s() == {}


def test_shipped_configs_name_the_ground_estimator_topics() -> None:
    ns = yaml.safe_load(GROUND_YAML.read_text())["output_namespace"]
    assert f"{ns}/labelled_points" == CLOUD_TOPIC
    assert f"{ns}/state" == STATE_TOPIC
    node = next(n for n in yaml.safe_load(CLIENT_YML.read_text())["ros2_nodes"] if n["name"] == "web_ui")
    for cfg in (load_config(DEFAULT_YAML), AppConfig.model_validate(yaml.safe_load(node["config"]))):
        nav = next(t for t in cfg.tabs if t.type == "map_nav")
        assert nav.ground_cloud_topic == CLOUD_TOPIC
        assert nav.ground_state_topic == STATE_TOPIC
        roles = cfg.topic_roles()
        assert roles[CLOUD_TOPIC] == GROUND_CLOUD_ROLE
        assert roles[STATE_TOPIC] == GROUND_STATE_ROLE
        assert cfg.ground_state_stale_s() == {STATE_TOPIC: nav.ground_state_stale_s}


# --- roles ----------------------------------------------------------------------------------------------------------


def test_roles_derive_message_types_and_qos() -> None:
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String

    cls, qos, _ = subscription_spec(CLOUD_TOPIC, GROUND_CLOUD_ROLE)
    assert cls is PointCloud2
    assert qos is SENSOR_SUB_QOS
    cls, qos, ser = subscription_spec(STATE_TOPIC, GROUND_STATE_ROLE)
    assert cls is String
    assert qos.reliability is ReliabilityPolicy.RELIABLE
    assert qos.durability is DurabilityPolicy.TRANSIENT_LOCAL
    assert ser is serialize_json_object
    assert GROUND_CLOUD_ROLE in POSE_FRAME_ROLES
    assert ROLE_SPECS[GROUND_STATE_ROLE][1] is MAP_SUB_QOS


# --- serializer -----------------------------------------------------------------------------------------------------


def test_labels_align_with_kept_points_after_voxel_downsampling() -> None:
    # Points 0 and 1 share a 5 cm voxel (the first, label 1, wins); point 2 is non-finite and dropped.
    pts = [(1.0, 1.0, 0.0), (1.001, 1.001, 0.001), (float("nan"), 0.0, 0.0), (3.0, 0.0, 0.2), (-2.0, 1.0, 0.0)]
    msg = make_labelled_cloud(pts, [1, 2, 3, 4, 5])
    out = serialize_ground_cloud(msg)
    assert out["frame"] == "base_link"
    assert out["count"] == 3
    assert decode_points(out).shape[0] == 3
    assert decode_labels(out).tolist() == [1, 4, 5]


def test_labels_follow_voxel_growth_under_the_point_cap() -> None:
    pts = [(i * 0.03, 0.0, 0.0) for i in range(100)]
    msg = make_labelled_cloud(pts, list(range(100)))
    with patch.object(cloud_serializer, "GROUND_CLOUD_MAX_POINTS", 10, create=True):
        out = serialize_ground_cloud(msg)
    assert out["count"] <= 10
    labels = decode_labels(out)
    assert len(labels) == out["count"]
    xs = decode_points(out)[:, 0] / 100.0
    assert [round(x / 0.03) for x in xs] == labels.tolist()


def test_missing_label_field_has_no_labels_and_logs_once() -> None:
    cloud_serializer.GROUND_LABEL_WARNED.clear()
    msg = make_labelled_cloud([(1.0, 0.0, 0.0)], None)
    a = serialize_ground_cloud(msg)
    b = serialize_ground_cloud(msg)
    assert "labels_b64" not in a and "labels_b64" not in b
    assert a["count"] == 1
    assert a["frame"] == "base_link"
    assert len(cloud_serializer.GROUND_LABEL_WARNED) == 1


# --- state passthrough and staleness --------------------------------------------------------------------------------


def test_state_example_passes_through_unchanged() -> None:
    raw = STATE_EXAMPLE.read_text()
    assert serialize_json_object(SimpleNamespace(data=raw)) == json.loads(raw)


def make_bridge() -> Any:
    with patch("web_ui.bridge.rclpy"), patch("web_ui.bridge.Node.__init__", return_value=None):
        from web_ui.bridge import BridgeNode

        node = BridgeNode.__new__(BridgeNode)
        node._latest = {}
        node._dirty = set()
        node._pending = {}
        node._cleared = set()
        node._lock = threading.Lock()
        node._topic_last_rx = {STATE_TOPIC: 0.0}
        node._ground_stale_s = {STATE_TOPIC: 2.0}
        return node


def test_mark_ground_stale_after_limit_and_cleared_by_next_message() -> None:
    node = make_bridge()
    state = json.loads(STATE_EXAMPLE.read_text())
    with patch("web_ui.bridge.time.monotonic", return_value=100.0):
        node.store(STATE_TOPIC, state)
        node.flush_dirty()
    with patch("web_ui.bridge.time.monotonic", return_value=101.5):
        node.mark_ground_stale()
        assert node.flush_dirty() == []
    with patch("web_ui.bridge.time.monotonic", return_value=102.5):
        node.mark_ground_stale()
        assert node.flush_dirty() == [{"topic": STATE_TOPIC, "data": {**state, "stale": True}}]
        node.mark_ground_stale()
        assert node.flush_dirty() == []
        node.store(STATE_TOPIC, state)
        assert node.flush_dirty() == [{"topic": STATE_TOPIC, "data": state}]
        node.mark_ground_stale()
        assert node.flush_dirty() == []


def test_mark_ground_stale_is_a_noop_without_a_message_or_subscription() -> None:
    node = make_bridge()
    node.mark_ground_stale()
    assert node.flush_dirty() == []
    node.store(STATE_TOPIC, {"a": 1})
    node._topic_last_rx.pop(STATE_TOPIC)  # unsubscribed
    node.flush_dirty()
    with patch("web_ui.bridge.time.monotonic", return_value=1e9):
        node.mark_ground_stale()
    assert node.flush_dirty() == []
