"""Tests for the head RGB-D fusion view backend: tab config keys, roles, subscription specs and serializers."""

from __future__ import annotations

import base64
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import patch

import numpy as np
import pytest
import yaml

from tests.test_bridge import _make_preview_node
from tests.test_map3d_cloud import decode_colors, decode_points, make_cloud
from web_ui import bridge
from web_ui.bridge import HEAD_ROLES, POSE_FRAME_ROLES, ROLE_SPECS, SENSOR_SUB_QOS, subscription_spec
from web_ui.cloud_serializer import serialize_map3d_cloud
from web_ui.config import (
    GENERIC_TOPIC_ATTRS,
    HEAD_RGBD_DEPTH_ROLE,
    HEAD_RGBD_RGB_ROLE,
    MAP_NAV_TOPIC_ROLES,
    RGBD_CLOUD_ROLE,
    AppConfig,
    TabConfig,
    load_config,
)
from web_ui.msg_serializer import msg_to_dict, serialize_registered_depth

RGBD_RGB = "/head_rgbd/rgb/image_raw/compressed"
RGBD_DEPTH = "/head_rgbd/depth/image_raw"
RGBD_CLOUD = "/head_rgbd/points"
DEFAULT_YAML = Path(__file__).resolve().parents[1] / "config" / "default.yaml"
CLIENT_YML = Path(__file__).resolve().parents[3] / "ansible" / "group_vars" / "client.yml"
STAMP = SimpleNamespace(sec=12, nanosec=250_000_000)
STAMP_S = 12.25

CONFIG_YAML = f"""
tabs:
  - id: head
    type: head
    label: Head
    rgbd_rgb_topic: {RGBD_RGB}
    rgbd_depth_topic: {RGBD_DEPTH}
    tof_range_m: 3.0
  - id: map
    type: map_nav
    label: Map
    rgbd_cloud_topic: {RGBD_CLOUD}
overlays: []
"""


def named(cls_name: str, **fields: Any) -> Any:
    """A message stand-in whose class name msg_to_dict dispatches on."""
    return type(cls_name, (), fields)()


def depth_msg(width: int = 288, height: int = 162, fill_mm: int = 1500, step_extra: int = 0) -> Any:
    """16UC1 depth Image with a stamp; step_extra pads each row by that many uint16."""
    arr = np.zeros((height, width + step_extra), dtype=np.uint16)
    arr[:, :width] = fill_mm
    arr[0, 0] = 0
    arr[1, 2] = 4321
    return named(
        "Image",
        header=SimpleNamespace(stamp=STAMP, frame_id="head_camera_optical_frame"),
        encoding="16UC1",
        is_bigendian=False,
        width=width,
        height=height,
        step=(width + step_extra) * 2,
        data=arr.tobytes(),
    )


# --- config ---------------------------------------------------------------------------------------------------------


def test_tab_config_keeps_the_rgbd_keys(tmp_path: Path) -> None:
    """rgbd_rgb_topic, rgbd_depth_topic and rgbd_cloud_topic are model fields, not silently dropped."""
    for key in ("rgbd_rgb_topic", "rgbd_depth_topic", "rgbd_cloud_topic"):
        assert key in TabConfig.model_fields
        assert getattr(TabConfig(id="x", type="head", label="X"), key) is None
    p = tmp_path / "cfg.yaml"
    p.write_text(CONFIG_YAML)
    head, nav = load_config(p).tabs
    assert (head.rgbd_rgb_topic, head.rgbd_depth_topic) == (RGBD_RGB, RGBD_DEPTH)
    assert nav.rgbd_cloud_topic == RGBD_CLOUD


def test_rgbd_topics_get_roles_ranges_and_subscriptions(tmp_path: Path) -> None:
    """The head RGB-D topics use the head roles, the depth topic carries tof_range_m, the cloud its own role."""
    assert {"rgbd_rgb_topic", "rgbd_depth_topic"} <= set(GENERIC_TOPIC_ATTRS)
    assert ("rgbd_cloud_topic", RGBD_CLOUD_ROLE) in MAP_NAV_TOPIC_ROLES
    p = tmp_path / "cfg.yaml"
    p.write_text(CONFIG_YAML)
    cfg = load_config(p)
    expected = {RGBD_RGB: HEAD_RGBD_RGB_ROLE, RGBD_DEPTH: HEAD_RGBD_DEPTH_ROLE, RGBD_CLOUD: RGBD_CLOUD_ROLE}
    roles = cfg.topic_roles()
    assert {t: roles[t] for t in expected} == expected
    assert cfg.tof_ranges() == {RGBD_DEPTH: 3.0}
    assert {RGBD_RGB, RGBD_DEPTH, RGBD_CLOUD} <= set(cfg.all_subscribed_topics())


def test_unset_keys_subscribe_nothing() -> None:
    """Without the keys no RGB-D topic is subscribed."""
    cfg = AppConfig(tabs=[TabConfig(id="h", type="head", label="H"), TabConfig(id="m", type="map_nav", label="M")])
    assert not [t for t in cfg.all_subscribed_topics() if "head_rgbd" in t]


def test_shipped_configs_name_the_rgbd_topics() -> None:
    """default.yaml and the client.yml web_ui block both carry the three keys with the head_rgbd topics."""
    node = next(n for n in yaml.safe_load(CLIENT_YML.read_text())["ros2_nodes"] if n["name"] == "web_ui")
    for cfg in (load_config(DEFAULT_YAML), AppConfig.model_validate(yaml.safe_load(node["config"]))):
        head = next(t for t in cfg.tabs if t.type == "head")
        nav = next(t for t in cfg.tabs if t.type == "map_nav")
        assert head.rgbd_rgb_topic == RGBD_RGB
        assert head.rgbd_depth_topic == RGBD_DEPTH
        assert nav.rgbd_cloud_topic == RGBD_CLOUD


# --- bridge specs ---------------------------------------------------------------------------------------------------


def test_subscription_specs_are_best_effort_depth_one() -> None:
    """RGB is a CompressedImage, depth an Image, the cloud a PointCloud2, all with SENSOR_SUB_QOS."""
    from sensor_msgs.msg import CompressedImage, Image, PointCloud2

    assert HEAD_RGBD_DEPTH_ROLE in HEAD_ROLES  # shares the head preview rate limit
    assert RGBD_CLOUD_ROLE in POSE_FRAME_ROLES  # the sensor pose in map at the stamp is attached
    assert HEAD_RGBD_RGB_ROLE in HEAD_ROLES
    assert subscription_spec(RGBD_RGB, HEAD_RGBD_RGB_ROLE)[0] is CompressedImage
    depth_cls, depth_qos, _ = subscription_spec(RGBD_DEPTH, HEAD_RGBD_DEPTH_ROLE, 3.0)
    assert depth_cls is Image
    assert depth_qos is SENSOR_SUB_QOS
    cloud_cls, cloud_qos, _ = ROLE_SPECS[RGBD_CLOUD_ROLE]
    assert cloud_cls is PointCloud2
    assert cloud_qos is SENSOR_SUB_QOS
    assert subscription_spec(RGBD_CLOUD, RGBD_CLOUD_ROLE)[0] is PointCloud2


# --- serializers ----------------------------------------------------------------------------------------------------


def test_registered_depth_is_full_size_raw_with_stamp() -> None:
    out = serialize_registered_depth(depth_msg())
    assert (out["width"], out["height"]) == (288, 162)
    assert out["stamp"] == pytest.approx(STAMP_S)
    assert out["frame_id"] == "head_camera_optical_frame"
    raw = np.frombuffer(base64.b64decode(out["depth_raw_b64"]), dtype="<u2").reshape(162, 288)
    assert raw[0, 0] == 0
    assert raw[1, 2] == 4321
    assert raw[100, 200] == 1500


def test_registered_depth_drops_row_padding() -> None:
    out = serialize_registered_depth(depth_msg(width=8, height=4, step_extra=3))
    raw = np.frombuffer(base64.b64decode(out["depth_raw_b64"]), dtype="<u2")
    assert raw.size == 8 * 4
    assert raw[1 * 8 + 2] == 4321


def test_registered_depth_rejects_other_encodings_and_truncated_data() -> None:
    bad = depth_msg()
    bad.encoding = "32FC1"
    with pytest.raises(ValueError, match="32FC1"):
        serialize_registered_depth(bad)
    short = depth_msg()
    short.data = short.data[:100]
    with pytest.raises(ValueError, match="truncated"):
        serialize_registered_depth(short)


def test_head_depth_roles_serialise_the_stamp() -> None:
    """The ToF preview payload carries the stamp; the rgbd depth spec serialises the raw registered depth."""
    assert msg_to_dict(depth_msg(), tof_range_m=3.0)["stamp"] == pytest.approx(STAMP_S)
    serializer = subscription_spec(RGBD_DEPTH, HEAD_RGBD_DEPTH_ROLE, 3.0)[2]
    out = serializer(depth_msg())
    assert out["stamp"] == pytest.approx(STAMP_S)
    assert "depth_raw_b64" in out


def test_compressed_rgb_payload_carries_the_same_stamp_as_depth() -> None:
    jpeg = b"\xff\xd8\xff\xe0jpegbytes"
    msg = named("CompressedImage", header=SimpleNamespace(stamp=STAMP, frame_id="f"), format="jpeg", data=jpeg)
    out = subscription_spec(RGBD_RGB, HEAD_RGBD_RGB_ROLE)[2](msg)
    assert base64.b64decode(out["jpeg_b64"]) == jpeg
    assert out["stamp"] == serialize_registered_depth(depth_msg())["stamp"]


def test_rgbd_cloud_keeps_colours_frame_stamp_and_its_own_cap() -> None:
    serializer = ROLE_SPECS[RGBD_CLOUD_ROLE][2]
    msg = make_cloud(
        [(1.0, 2.0, 0.5), (-1.0, 0.0, 1.0)], colors=[(255, 0, 10), (1, 2, 3)], frame="head_camera_optical_frame"
    )
    out = serializer(msg)
    assert out["frame"] == "head_camera_optical_frame"  # what attach_frame_pose looks up
    assert out["stamp"] == pytest.approx(5.5)
    assert out["count"] == 2
    colors = {tuple(p): tuple(c) for p, c in zip(decode_points(out).tolist(), decode_colors(out).tolist(), strict=True)}
    assert colors[(100, 200, 50)] == (255, 0, 10)
    assert bridge.RGBD_CLOUD_MAX_POINTS == 20000
    assert bridge.RGBD_CLOUD_VOXEL_M == 0.02


def test_rgbd_cloud_is_capped_at_20000_points() -> None:
    rng = np.random.default_rng(1)
    pts = [tuple(p) for p in rng.uniform(0.0, 3.0, size=(60000, 3)).tolist()]
    msg = make_cloud(pts, colors=[(1, 2, 3)] * len(pts))
    out = ROLE_SPECS[RGBD_CLOUD_ROLE][2](msg)
    assert out["count"] <= 20000
    assert out["count"] == len(decode_colors(out))
    assert out["voxel_m"] > 0.02
    assert out == {**serialize_map3d_cloud(msg, 20000, 0.02), "frame": "rtabmap_map"}


def test_registered_depth_honours_big_endian() -> None:
    msg = depth_msg(width=4, height=2)
    msg.is_bigendian = True
    msg.data = np.frombuffer(msg.data, dtype="<u2").astype(">u2").tobytes()
    raw = np.frombuffer(base64.b64decode(serialize_registered_depth(msg)["depth_raw_b64"]), dtype="<u2")
    assert raw[1 * 4 + 2] == 4321
    assert raw[7] == 1500


def stamped(sec: float) -> Any:
    whole = int(sec)
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=whole, nanosec=round((sec - whole) * 1e9))))


def forwarded_stamps(node: Any, topic: str, role: str, arrivals: list[tuple[float, float]]) -> list[float]:
    """Feed (arrival time, stamp) pairs to the callback and return the stamps that reached the latest-message store."""
    seen: list[float] = []
    node.store_raw = lambda _t, msg, _s, _r: seen.append(
        round(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, 3)
    )
    cb = node._make_callback(topic, lambda m: {}, role)
    with patch("web_ui.bridge.time.monotonic") as clock:
        for arrival, stamp in arrivals:
            clock.return_value = arrival
            cb(stamped(stamp))
    return seen


def test_rgbd_roles_forward_the_same_stamps_whatever_the_arrival_jitter() -> None:
    """Both RGB-D topics are gated on the header stamp, so arrival jitter cannot make them keep different sets."""
    stamps = [100.0 + 0.05 * i for i in range(40)]  # sets every 0.05 s, faster than the 0.1 s gate
    rgb = [(s + 0.03 * (-1) ** i, s) for i, s in enumerate(stamps)]
    depth = [(s + 0.03 * (-1) ** (i // 2), s) for i, s in enumerate(stamps)]
    got_rgb = forwarded_stamps(_make_preview_node(), RGBD_RGB, HEAD_RGBD_RGB_ROLE, rgb)
    got_depth = forwarded_stamps(_make_preview_node(), RGBD_DEPTH, HEAD_RGBD_DEPTH_ROLE, depth)
    assert got_rgb == got_depth
    assert 15 <= len(got_rgb) <= 21  # one set per 0.1 s


def test_rgbd_gate_passes_a_stamp_that_went_backwards() -> None:
    got = forwarded_stamps(
        _make_preview_node(), RGBD_DEPTH, HEAD_RGBD_DEPTH_ROLE, [(0.0, 100.0), (0.01, 100.01), (0.02, 5.0), (0.03, 5.2)]
    )
    assert got == [100.0, 5.0, 5.2]
