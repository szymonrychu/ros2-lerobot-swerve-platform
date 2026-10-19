"""Tests for the map3d cloud backend: PointCloud2 serializer, map_nav config fields and the bridge roles."""

from __future__ import annotations

import base64
from collections.abc import Iterator
from contextlib import ExitStack
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import patch

import numpy as np
import pytest
import yaml
from pydantic import ValidationError

from web_ui.bridge import ROLE_SPECS, SENSOR_SUB_QOS, BridgeNode, PointCloud2, ReliabilityPolicy
from web_ui.cloud_serializer import serialize_map3d_cloud
from web_ui.config import MAP_NAV_DEFAULTS, MAP_NAV_TOPIC_ROLES, AppConfig, TabConfig
from web_ui.msg_serializer import serialize_point_cloud

FLOAT32 = 7
UINT32 = 6
INT32 = 5
CONTRACT_YAML = Path(__file__).resolve().parents[3] / "nodes" / "rtabmap_mapping" / "config" / "map3d_topics.yaml"
Point = tuple[float, float, float]


def make_cloud(
    points: list[Point],
    colors: list[tuple[int, int, int]] | None = None,
    rgb_name: str = "rgb",
    rgb_float: bool = False,
    frame: str = "rtabmap_map",
    width: int | None = None,
) -> Any:
    """PointCloud2-like message: x y z float32 (+ packed rgb) with point_step 16."""
    rgb_type = np.float32 if rgb_float else np.uint32
    arr = np.zeros(len(points), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", rgb_type)])
    for i, (x, y, z) in enumerate(points):
        arr["x"][i], arr["y"][i], arr["z"][i] = x, y, z
    if colors is not None:
        packed = np.array([(r << 16) | (g << 8) | b for r, g, b in colors], dtype=np.uint32)
        arr["rgb"] = packed.view(np.float32) if rgb_float else packed
    fields = [SimpleNamespace(name=n, offset=o, datatype=FLOAT32, count=1) for n, o in (("x", 0), ("y", 4), ("z", 8))]
    if colors is not None:
        fields.append(SimpleNamespace(name=rgb_name, offset=12, datatype=FLOAT32 if rgb_float else UINT32, count=1))
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame, stamp=SimpleNamespace(sec=5, nanosec=500_000_000)),
        height=1,
        width=len(points) if width is None else width,
        fields=fields,
        is_bigendian=False,
        point_step=16,
        row_step=16 * len(points),
        data=arr.tobytes(),
        is_dense=False,
    )


def decode_points(payload: dict[str, Any]) -> np.ndarray:
    """int16 centimetres (n, 3) from points_b64."""
    return np.frombuffer(base64.b64decode(payload["points_b64"]), dtype="<i2").reshape(-1, 3)


def decode_colors(payload: dict[str, Any]) -> np.ndarray:
    return np.frombuffer(base64.b64decode(payload["colors_b64"]), dtype=np.uint8).reshape(-1, 3)


def test_xyz_rgb_cloud_roundtrip() -> None:
    msg = make_cloud([(1.0, 2.0, 0.5), (-1.0, 0.0, 1.0)], colors=[(255, 0, 10), (1, 2, 3)])
    out = serialize_map3d_cloud(msg, max_points=100, voxel_m=0.05)
    assert out["count"] == out["source_count"] == 2
    assert out["frame_id"] == "rtabmap_map"
    assert out["stamp"] == pytest.approx(5.5)
    assert out["voxel_m"] == 0.05
    assert out["dropped_out_of_range"] == 0
    assert sorted(map(tuple, decode_points(out).tolist())) == [(-100, 0, 100), (100, 200, 50)]
    colors = {tuple(p): tuple(c) for p, c in zip(decode_points(out).tolist(), decode_colors(out).tolist(), strict=True)}
    assert colors[(100, 200, 50)] == (255, 0, 10)
    assert colors[(-100, 0, 100)] == (1, 2, 3)


def test_float32_rgb_and_rgba_names() -> None:
    for name, as_float in (("rgb", True), ("rgba", False)):
        out = serialize_map3d_cloud(
            make_cloud([(0.0, 0.0, 0.0)], colors=[(9, 8, 7)], rgb_name=name, rgb_float=as_float), 10, 0.05
        )
        assert decode_colors(out).tolist() == [[9, 8, 7]]


def test_xyz_only_cloud_omits_colors() -> None:
    out = serialize_map3d_cloud(make_cloud([(1.0, 1.0, 1.0)]), 10, 0.05)
    assert "colors_b64" not in out
    assert out["count"] == 1


def test_non_finite_points_dropped() -> None:
    msg = make_cloud([(float("nan"), 0, 0), (1.0, float("inf"), 0), (0.5, 0.5, 0.5)])
    out = serialize_map3d_cloud(msg, 10, 0.05)
    assert out["count"] == 1
    assert out["source_count"] == 3
    assert decode_points(out).tolist() == [[50, 50, 50]]


def test_centimetre_rounding() -> None:
    out = serialize_map3d_cloud(make_cloud([(1.234, -0.006, 0.0149)]), 10, 0.02)
    assert decode_points(out).tolist() == [[123, -1, 1]]


def test_voxel_keeps_first_point_per_cell() -> None:
    msg = make_cloud(
        [(0.01, 0.01, 0.01), (0.02, 0.02, 0.02), (1.0, 1.0, 1.0)], colors=[(1, 1, 1), (2, 2, 2), (3, 3, 3)]
    )
    out = serialize_map3d_cloud(msg, 10, 0.1)
    assert out["count"] == 2
    assert out["voxel_m"] == 0.1
    assert sorted(decode_colors(out).tolist()) == [[1, 1, 1], [3, 3, 3]]


def test_cap_respected_with_voxel_grown() -> None:
    xs = np.arange(2000) * 0.05
    msg = make_cloud([(float(x), 0.0, 0.0) for x in xs])
    out = serialize_map3d_cloud(msg, max_points=300, voxel_m=0.05)
    assert out["count"] <= 300
    assert out["source_count"] == 2000
    assert out["voxel_m"] > 0.05
    assert out["voxel_m"] == pytest.approx(0.05 * 1.5 ** round(np.log(out["voxel_m"] / 0.05) / np.log(1.5)))
    assert len(decode_points(out)) == out["count"]


def test_out_of_range_counted_and_dropped() -> None:
    msg = make_cloud([(400.0, 0.0, 0.0), (-400.0, 0, 0), (327.67, -327.67, 0.0), (1.0, 1.0, 1.0)])
    out = serialize_map3d_cloud(msg, 10, 0.05)
    assert out["dropped_out_of_range"] == 2
    assert out["count"] == 2
    assert sorted(map(tuple, decode_points(out).tolist())) == [(100, 100, 100), (32767, -32767, 0)]


def test_unsupported_layout_raises_naming_fields() -> None:
    msg = make_cloud([(1.0, 1.0, 1.0)])
    msg.fields = [f for f in msg.fields if f.name != "z"]
    with pytest.raises(ValueError, match="x.*y"):
        serialize_map3d_cloud(msg, 10, 0.05)
    bad = make_cloud([(1.0, 1.0, 1.0)], colors=[(1, 1, 1)])
    bad.fields[-1].datatype = INT32
    with pytest.raises(ValueError, match="rgb"):
        serialize_map3d_cloud(bad, 10, 0.05)


def test_empty_cloud_is_a_real_empty_map() -> None:
    out = serialize_map3d_cloud(make_cloud([]), 10, 0.05)
    assert out["count"] == 0
    assert out["source_count"] == 0
    assert decode_points(out).size == 0


def test_malformed_message_raises_not_placeholder() -> None:
    msg = make_cloud([(1.0, 1.0, 1.0)], width=5)
    with pytest.raises(ValueError, match="data"):
        serialize_map3d_cloud(msg, 10, 0.05)


def test_obstacle_serializer_still_works_on_shared_decoder() -> None:
    out = serialize_point_cloud(make_cloud([(1.2346, 0.0, 0.0)]))
    assert out["points"] == [1.235, 0.0, 0.0]


# --- config ---------------------------------------------------------------------------------------------------------


def test_config_defaults() -> None:
    tab = TabConfig(id="m", type="map_nav", label="Map")
    assert tab.map3d_cloud_topic == "/map3d/cloud"
    assert tab.map3d_status_topic == "/map3d/status"
    assert tab.map3d_result_topic == "/map3d/result"
    assert tab.map3d_command_topic == "/map3d/command"
    assert tab.map3d_frame_id == "rtabmap_map"
    assert tab.map3d_max_points == 60000
    assert tab.map3d_voxel_m == 0.05
    assert tab.lidar_wall_height_m == 1.0
    assert ("map3d_cloud_topic", "map3d_cloud") in MAP_NAV_TOPIC_ROLES
    assert ("map3d_status_topic", "map3d_status") in MAP_NAV_TOPIC_ROLES
    assert ("map3d_result_topic", "map3d_result") in MAP_NAV_TOPIC_ROLES
    roles = AppConfig(tabs=[tab]).topic_roles()
    assert roles["/map3d/cloud"] == "map3d_cloud"
    assert roles["/map3d/status"] == "map3d_status"
    assert roles["/map3d/result"] == "map3d_result"
    assert "/map3d/command" in AppConfig(tabs=[tab]).publish_topics()


@pytest.mark.parametrize(
    ("field", "low", "high"),
    [("map3d_max_points", 1000, 300000), ("map3d_voxel_m", 0.02, 0.5), ("lidar_wall_height_m", 0.1, 3.0)],
)
def test_config_bounds(field: str, low: float, high: float) -> None:
    for ok in (low, high):
        assert getattr(TabConfig(id="m", type="map_nav", label="M", **{field: ok}), field) == ok
    for bad in (low / 2, high * 2):
        with pytest.raises(ValidationError):
            TabConfig(id="m", type="map_nav", label="M", **{field: bad})


def test_config_defaults_match_topic_contract() -> None:
    downlink = {entry["dest"] for entry in yaml.safe_load(CONTRACT_YAML.read_text())["downlink"]}
    frames = yaml.safe_load(CONTRACT_YAML.read_text())["frames"]
    for key in ("map3d_cloud_topic", "map3d_status_topic", "map3d_result_topic"):
        assert MAP_NAV_DEFAULTS[key] in downlink
    assert MAP_NAV_DEFAULTS["map3d_frame_id"] == frames["map3d_frame"]
    uplink = {entry["dest"].removeprefix("/uplink") for entry in yaml.safe_load(CONTRACT_YAML.read_text())["uplink"]}
    assert MAP_NAV_DEFAULTS["map3d_command_topic"] in uplink


def test_map3d_limits_per_topic() -> None:
    tab = TabConfig(id="m", type="map_nav", label="M", map3d_max_points=5000, map3d_voxel_m=0.1)
    assert AppConfig(tabs=[tab]).map3d_limits() == {"/map3d/cloud": (5000, 0.1)}


# --- bridge ---------------------------------------------------------------------------------------------------------


@pytest.fixture
def subs() -> Iterator[list[SimpleNamespace]]:
    created: list[SimpleNamespace] = []

    def create_subscription(msg_cls: Any, topic: str, callback: Any, qos: Any) -> SimpleNamespace:
        sub = SimpleNamespace(msg_cls=msg_cls, topic=topic, callback=callback, qos=qos)
        created.append(sub)
        return sub

    with ExitStack() as stack:
        stack.enter_context(patch("web_ui.bridge.Node.__init__", return_value=None))
        for name, side_effect in (
            ("create_subscription", create_subscription),
            ("destroy_subscription", None),
            ("create_timer", None),
            ("create_publisher", None),
            ("get_logger", None),
            ("create_client", None),
            ("get_clock", None),
        ):
            stack.enter_context(patch.object(BridgeNode, name, create=True, side_effect=side_effect))
        stack.enter_context(patch("web_ui.bridge.Buffer"))
        stack.enter_context(patch("web_ui.bridge.TransformListener"))
        yield created


def make_node(**kwargs: Any) -> BridgeNode:
    roles = {"/map3d/cloud": "map3d_cloud", "/map3d/status": "map3d_status", "/map3d/result": "map3d_result"}
    args: dict[str, Any] = {"topics": list(roles), "allowed_publish_topics": set(), "topic_roles": roles}
    args.update(kwargs)
    return BridgeNode(**args)


def test_role_specs_qos_and_serializers() -> None:
    from std_msgs.msg import String

    cls, qos, _ = ROLE_SPECS["map3d_cloud"]
    assert cls is PointCloud2
    assert qos is SENSOR_SUB_QOS
    assert (qos.depth, qos.reliability) == (1, ReliabilityPolicy.BEST_EFFORT)
    cls, qos, _ = ROLE_SPECS["map3d_status"]
    assert cls is String
    assert (qos.depth, qos.reliability) == (1, ReliabilityPolicy.RELIABLE)
    cls, qos, _ = ROLE_SPECS["map3d_result"]
    assert cls is String
    assert (qos.depth, qos.reliability) == (10, ReliabilityPolicy.RELIABLE)


def test_cloud_is_on_demand_status_and_result_too(subs: list[SimpleNamespace]) -> None:
    node = make_node()
    topics = {"/map3d/cloud", "/map3d/status", "/map3d/result"}
    assert not topics & {s.topic for s in subs}
    node.update_demand(set(), now=0.0)
    assert not topics & {s.topic for s in subs}
    node.update_demand({"/map3d/cloud"}, now=0.0)
    assert topics & {s.topic for s in subs} == {"/map3d/cloud"}
    assert sub_of(subs, "/map3d/cloud").qos is SENSOR_SUB_QOS


def sub_of(subs: list[SimpleNamespace], topic: str) -> SimpleNamespace:
    """The subscription created for topic."""
    return next(s for s in subs if s.topic == topic)


def test_cloud_serializer_uses_tab_limits_and_is_lazy(subs: list[SimpleNamespace]) -> None:
    node = make_node(map3d_limits={"/map3d/cloud": (1000, 0.5)})
    node.update_demand({"/map3d/cloud"}, now=0.0)
    msg = make_cloud([(float(i) * 0.1, 0.0, 0.0) for i in range(100)])
    sub_of(subs, "/map3d/cloud").callback(msg)
    assert "/map3d/cloud" in node._pending
    node.serialize_pending(["/map3d/cloud"])
    data = node._latest["/map3d/cloud"]["data"]
    assert data["voxel_m"] == 0.5
    assert data["count"] <= 1000
    assert node.serialize_pending(["/map3d/cloud"]) == set()


def test_status_json_passed_through(subs: list[SimpleNamespace]) -> None:
    node = make_node()
    node.update_demand({"/map3d/status"}, now=0.0)
    sub_of(subs, "/map3d/status").callback(SimpleNamespace(data='{"state": "mapping", "nodes": 3}'))
    node.serialize_pending(["/map3d/status"])
    assert node._latest["/map3d/status"]["data"] == {"state": "mapping", "nodes": 3}


def test_result_burst_delivers_every_message_and_is_not_replayed(subs: list[SimpleNamespace]) -> None:
    node = make_node()
    node.update_demand({"/map3d/result"}, now=0.0)
    callback = sub_of(subs, "/map3d/result").callback
    callback(SimpleNamespace(data='{"request_id": "a", "ok": true}'))
    callback(SimpleNamespace(data='{"request_id": "b", "ok": false}'))
    envelopes = node.flush_dirty()
    assert [e["data"]["request_id"] for e in envelopes if e["topic"] == "/map3d/result"] == ["a", "b"]
    assert node.flush_dirty() == []
    assert [e for e in node.latest_envelopes() if e["topic"] == "/map3d/result"] == []
