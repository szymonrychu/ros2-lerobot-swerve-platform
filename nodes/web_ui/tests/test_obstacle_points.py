"""Tests for the map_nav "3D obstacles" layer backend: ToF cloud and lidar points serialisers, config and bridge."""

from __future__ import annotations

import math
import threading
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import numpy as np
import pytest

from web_ui.config import MAP_NAV_TOPIC_ROLES, AppConfig, TabConfig
from web_ui.msg_serializer import serialize_point_cloud, serialize_scan_points

FLOAT32 = 7  # sensor_msgs/PointField.FLOAT32


def stamp(sec: int = 5, nanosec: int = 500_000_000) -> SimpleNamespace:
    return SimpleNamespace(sec=sec, nanosec=nanosec)


def make_cloud(points: list[tuple[float, float, float]], frame: str = "tof_link", extra: bool = True) -> Any:
    """PointCloud2-like message: x y z float32 (+ an intensity field) with point_step 16."""
    arr = np.zeros(len(points), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("i", "<f4")])
    for i, (x, y, z) in enumerate(points):
        arr[i] = (x, y, z, 1.0)
    fields = [SimpleNamespace(name=n, offset=o, datatype=FLOAT32, count=1) for n, o in (("x", 0), ("y", 4), ("z", 8))]
    if extra:
        fields.append(SimpleNamespace(name="i", offset=12, datatype=FLOAT32, count=1))
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame, stamp=stamp()),
        height=1,
        width=len(points),
        fields=fields,
        is_bigendian=False,
        point_step=16,
        row_step=16 * len(points),
        data=arr.tobytes(),
        is_dense=False,
    )


def make_scan(ranges: list[float], frame: str = "laser", rmin: float = 0.1, rmax: float = 12.0) -> Any:
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame, stamp=stamp(7, 0)),
        angle_min=0.0,
        angle_max=math.pi / 2,
        angle_increment=math.pi / 2 / max(len(ranges) - 1, 1),
        range_min=rmin,
        range_max=rmax,
        ranges=ranges,
    )


def test_cloud_serialises_flat_xyz_rounded_to_mm() -> None:
    data = serialize_point_cloud(make_cloud([(1.23456, -0.5, 0.0004), (2.0, 3.0, 4.0)]))
    assert data["frame"] == "tof_link"
    assert data["stamp"] == pytest.approx(5.5)
    assert data["points"] == [1.235, -0.5, 0.0, 2.0, 3.0, 4.0]


def test_cloud_drops_non_finite_points() -> None:
    data = serialize_point_cloud(make_cloud([(math.nan, 0, 0), (1, 1, 1), (0, math.inf, 0), (2, 2, -math.inf)]))
    assert data["points"] == [1.0, 1.0, 1.0]


def test_empty_cloud_gives_empty_points() -> None:
    data = serialize_point_cloud(make_cloud([]))
    assert data["points"] == []
    assert data["frame"] == "tof_link"


def test_cloud_downsampled_evenly_to_cap() -> None:
    pts = [(float(i), 0.0, 0.0) for i in range(10_000)]
    data = serialize_point_cloud(make_cloud(pts))
    assert len(data["points"]) == 2000 * 3
    xs = data["points"][0::3]
    assert xs[0] == 0.0 and xs[-1] == 9999.0
    assert xs == sorted(xs)
    assert len(serialize_point_cloud(make_cloud(pts), max_points=50)["points"]) == 150


def test_cloud_under_cap_is_untouched() -> None:
    pts = [(float(i), 1.0, 2.0) for i in range(30)]
    assert len(serialize_point_cloud(make_cloud(pts))["points"]) == 90


def test_scan_points_flat_xy_within_range() -> None:
    data = serialize_scan_points(make_scan([1.0, 2.0, 3.0]))
    assert data["frame"] == "laser"
    assert data["stamp"] == pytest.approx(7.0)
    pts = data["points"]
    assert len(pts) == 6
    assert pts[0:2] == [1.0, 0.0]
    assert pts[2] == pytest.approx(2.0 * math.cos(math.pi / 4), abs=1e-3)
    assert pts[4] == pytest.approx(0.0, abs=1e-3) and pts[5] == pytest.approx(3.0)


def test_scan_points_skip_non_finite_and_out_of_range() -> None:
    data = serialize_scan_points(make_scan([math.inf, 0.05, 1.0, math.nan, 13.0, 2.0]))
    assert len(data["points"]) == 4


def test_scan_points_capped_and_empty() -> None:
    assert len(serialize_scan_points(make_scan([1.0] * 2000))["points"]) == 720 * 2
    assert len(serialize_scan_points(make_scan([1.0] * 2000), max_points=100)["points"]) == 200
    assert serialize_scan_points(make_scan([]))["points"] == []


def test_config_defaults_and_roles() -> None:
    tab = TabConfig(id="m", type="map_nav", label="Map")
    assert tab.obstacle_cloud_topic == "/tof/points_filtered"
    assert tab.scan_points_topic == "/scan_filtered"
    assert ("obstacle_cloud_topic", "cloud") in MAP_NAV_TOPIC_ROLES
    assert ("scan_points_topic", "scan_points") in MAP_NAV_TOPIC_ROLES
    roles = AppConfig(tabs=[tab]).topic_roles()
    assert roles["/tof/points_filtered"] == "cloud"
    assert roles["/scan_filtered"] == "scan_points"


def test_config_overrides_and_not_defaulted_elsewhere() -> None:
    tab = TabConfig(id="m", type="map_nav", label="Map", obstacle_cloud_topic="/c", scan_points_topic="/s")
    roles = AppConfig(tabs=[tab]).topic_roles()
    assert roles["/c"] == "cloud" and roles["/s"] == "scan_points"
    poi = TabConfig(id="p", type="poi", label="P")
    assert poi.obstacle_cloud_topic is None and poi.scan_points_topic is None
    assert "/tof/points_filtered" not in AppConfig(tabs=[poi]).topic_roles()


def make_bridge(tf_buffer: Any) -> Any:
    from web_ui.bridge import BridgeNode

    node = BridgeNode.__new__(BridgeNode)
    node._latest = {}
    node._dirty = set()
    node._pending = {}
    node._frame_pose_pending = {}
    node._frame_pose_lock = threading.Lock()
    node._frame_pose_logged = {}
    node._cleared = set()
    node._topic_last_rx = {}
    node._lock = threading.Lock()
    node._tf_buffer = tf_buffer
    node._robot_pose_frames = ("map", "base_link")
    return node


def tf_to_map() -> SimpleNamespace:
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="map", stamp=stamp()),
        child_frame_id="tof_link",
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=1.0, y=2.0, z=0.5),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )


def test_role_specs_use_serialisers() -> None:
    from web_ui.bridge import ROLE_SPECS

    assert ROLE_SPECS["cloud"][2] is serialize_point_cloud
    assert ROLE_SPECS["scan_points"][2] is serialize_scan_points


def test_bridge_sends_nothing_when_map_tf_unknown() -> None:
    from web_ui.bridge import TransformException

    tf_buffer = MagicMock()
    tf_buffer.lookup_transform.side_effect = TransformException("no tf")
    node = make_bridge(tf_buffer)
    cb = node._make_callback("/tof/points_filtered", serialize_point_cloud, "cloud")
    cb(make_cloud([(1, 1, 1)]))
    assert node.flush_dirty() == []
    assert node._latest == {}


def test_bridge_counts_frame_pose_drops_by_role_and_reason() -> None:
    from web_ui.bridge import TransformException
    from web_ui.metrics import FRAME_POSE_DROPS

    class ExtrapolationException(TransformException):  # stands in for the tf2 subclass
        pass

    tf_buffer = MagicMock()
    tf_buffer.lookup_transform.side_effect = ExtrapolationException("future")
    node = make_bridge(tf_buffer)
    counter = FRAME_POSE_DROPS.labels(role="scan_points", reason="ExtrapolationException")
    before = counter._value.get()
    cb = node._make_callback("/scan_filtered", serialize_scan_points, "scan_points")
    cb(make_scan([1.0, 2.0], frame="tof_link"))
    assert node.flush_dirty() == []
    assert counter._value.get() == before + 1


def test_bridge_attaches_frame_pose_when_tf_known() -> None:
    tf_buffer = MagicMock()
    tf_buffer.lookup_transform.return_value = tf_to_map()
    node = make_bridge(tf_buffer)
    cb = node._make_callback("/scan_filtered", serialize_scan_points, "scan_points")
    cb(make_scan([1.0, 2.0], frame="tof_link"))
    envelope = node.flush_dirty()[0]
    assert envelope["topic"] == "/scan_filtered"
    pose = envelope["data"]["frame_pose"]
    assert pose == {"x": 1.0, "y": 2.0, "z": 0.5, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
    assert envelope["data"]["frame"] == "tof_link"
    assert tf_buffer.lookup_transform.call_args.args[:2] == ("map", "tof_link")


def test_bridge_without_tf_tracking_sends_nothing() -> None:
    node = make_bridge(None)
    node._robot_pose_frames = None
    cb = node._make_callback("/tof/points_filtered", serialize_point_cloud, "cloud")
    cb(make_cloud([(1, 1, 1)]))
    assert node.flush_dirty() == []


def test_cloud_roles_are_on_demand_and_need_tf() -> None:
    from web_ui.bridge import POSE_FRAME_ROLES

    assert POSE_FRAME_ROLES == frozenset({"cloud", "scan_points", "rgbd_cloud"})


def drops(role: str, reason: str) -> float:
    from web_ui.metrics import FRAME_POSE_DROPS

    return FRAME_POSE_DROPS.labels(role=role, reason=reason)._value.get()


def pending_bridge(side_effect: Any) -> tuple[Any, Any, Any]:
    tf_buffer = MagicMock()
    tf_buffer.lookup_transform.side_effect = side_effect
    node = make_bridge(tf_buffer)
    cb = node._make_callback("/scan_filtered", serialize_scan_points, "scan_points")
    return node, tf_buffer, cb


def test_extrapolation_is_held_and_published_on_retry() -> None:
    from tf2_ros import ExtrapolationException

    node, tf_buffer, cb = pending_bridge(ExtrapolationException("future"))
    before = drops("scan_points", "ExtrapolationException")
    cb(make_scan([1.0, 2.0], frame="tof_link"))
    assert node.flush_dirty() == []
    tf_buffer.lookup_transform.side_effect = None
    tf_buffer.lookup_transform.return_value = tf_to_map()
    node.retry_frame_poses()
    envelope = node.flush_dirty()[0]
    assert envelope["topic"] == "/scan_filtered"
    assert envelope["data"]["frame_pose"]["x"] == 1.0
    assert drops("scan_points", "ExtrapolationException") == before
    assert node._frame_pose_pending == {}


def test_newer_message_supersedes_pending() -> None:
    from tf2_ros import ExtrapolationException

    node, tf_buffer, cb = pending_bridge(ExtrapolationException("future"))
    before = drops("scan_points", "superseded")
    cb(make_scan([1.0], frame="tof_link"))
    node.flush_dirty()
    cb(make_scan([5.0, 6.0], frame="tof_link"))
    node.flush_dirty()
    assert drops("scan_points", "superseded") == before + 1
    tf_buffer.lookup_transform.side_effect = None
    tf_buffer.lookup_transform.return_value = tf_to_map()
    node.retry_frame_poses()
    envelopes = node.flush_dirty()
    assert len(envelopes) == 1
    assert len(envelopes[0]["data"]["ranges"]) == 2 if "ranges" in envelopes[0]["data"] else True


def test_pending_older_than_wait_is_dropped_and_counted() -> None:
    from tf2_ros import ExtrapolationException

    from web_ui.bridge import FRAME_POSE_WAIT_S

    node, _, cb = pending_bridge(ExtrapolationException("future"))
    before = drops("scan_points", "ExtrapolationException")
    cb(make_scan([1.0], frame="tof_link"))
    node.flush_dirty()
    first = node._frame_pose_pending["/scan_filtered"].first_attempt
    node.retry_frame_poses(now=first + FRAME_POSE_WAIT_S / 2)
    assert "/scan_filtered" in node._frame_pose_pending
    assert drops("scan_points", "ExtrapolationException") == before
    node.retry_frame_poses(now=first + FRAME_POSE_WAIT_S + 0.01)
    assert node._frame_pose_pending == {}
    assert drops("scan_points", "ExtrapolationException") == before + 1
    assert node.flush_dirty() == []


def test_lookup_exception_still_drops_immediately() -> None:
    from tf2_ros import LookupException

    node, _, cb = pending_bridge(LookupException("no frame"))
    before = drops("scan_points", "LookupException")
    cb(make_scan([1.0], frame="tof_link"))
    assert node.flush_dirty() == []
    assert node._frame_pose_pending == {}
    assert drops("scan_points", "LookupException") == before + 1


def test_retry_runs_from_apply_demand_timer() -> None:
    from tf2_ros import ExtrapolationException

    node, tf_buffer, cb = pending_bridge(ExtrapolationException("future"))
    cb(make_scan([1.0], frame="tof_link"))
    node.flush_dirty()
    tf_buffer.lookup_transform.side_effect = None
    tf_buffer.lookup_transform.return_value = tf_to_map()
    assert node.flush_dirty() == []  # the WS thread path never retries
    node.run_publisher_tasks = MagicMock()
    node._demand_lock = threading.Lock()
    node._wanted = frozenset()
    node._demand = {}
    node._active = {}
    node._linger_until = {}
    node.reconcile_tf = MagicMock()
    node.apply_demand()
    assert len(node.flush_dirty()) == 1
