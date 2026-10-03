"""Tests for the map_nav tab: config, map/path/goal serialization, robot pose, goal publishing, save map."""

from __future__ import annotations

import array
import base64
import math
import threading
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock, patch

import cv2
import numpy as np
import pytest
from fastapi.testclient import TestClient
from pydantic import ValidationError

from web_ui.config import AppConfig, TabConfig, load_config

MAP_NAV_YAML = """
tabs:
  - id: map
    type: map_nav
    label: Map
"""


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def yaw_quat(yaw: float) -> SimpleNamespace:
    """Return a planar quaternion namespace for a yaw angle."""
    return SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def stamp(sec: int = 0, nanosec: int = 0) -> SimpleNamespace:
    """Return a builtin_interfaces/Time-like namespace."""
    return SimpleNamespace(sec=sec, nanosec=nanosec)


def make_grid(width: int, height: int, data: list[int], yaw: float = 0.0) -> SimpleNamespace:
    """Return an OccupancyGrid-like namespace."""
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="map", stamp=stamp(12, 500_000_000)),
        info=SimpleNamespace(
            width=width,
            height=height,
            resolution=0.05,
            origin=SimpleNamespace(position=SimpleNamespace(x=-1.5, y=2.0, z=0.0), orientation=yaw_quat(yaw)),
        ),
        data=array.array("b", data),
    )


def make_path(n: int, frame_id: str = "map") -> SimpleNamespace:
    """Return a nav_msgs/Path-like namespace with n poses along x."""
    poses = [
        SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=float(i), y=float(i) * 0.5, z=0.0)))
        for i in range(n)
    ]
    return SimpleNamespace(header=SimpleNamespace(frame_id=frame_id, stamp=stamp()), poses=poses)


def make_transform(x: float, y: float, yaw: float, frame_id: str = "map") -> SimpleNamespace:
    """Return a geometry_msgs/TransformStamped-like namespace."""
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame_id, stamp=stamp(3, 250_000_000)),
        child_frame_id="base_link",
        transform=SimpleNamespace(translation=SimpleNamespace(x=x, y=y, z=0.0), rotation=yaw_quat(yaw)),
    )


def fake_clock(now_s: float) -> SimpleNamespace:
    """Return an rclpy Clock-like namespace whose now() is now_s seconds."""
    return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(now_s * 1e9)))


def make_bridge(**attrs: Any) -> Any:
    """Construct a BridgeNode without running rclpy initialisation."""
    from web_ui.bridge import BridgeNode

    node = BridgeNode.__new__(BridgeNode)
    node._latest = {}
    node._dirty = set()
    node._lock = threading.Lock()
    node.publishers_ = {}
    node._allowed_publish_topics = set()
    node._topic_last_rx = {}
    node._tf_buffer = None
    node._robot_pose_frames = None
    node._frame_id_defaults = {}
    node._serialize_map_client = None
    node._reset_map_clients = {}
    node._cancel_goal_clients = {}
    node._cleared = set()
    node.get_clock = lambda: fake_clock(3.5)
    for key, value in attrs.items():
        setattr(node, key, value)
    return node


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------


def test_map_nav_is_valid_tab_type() -> None:
    tab = TabConfig(id="m", type="map_nav", label="Map")
    assert tab.type == "map_nav"


def test_unknown_tab_type_still_rejected() -> None:
    with pytest.raises(ValidationError):
        TabConfig(id="m", type="map_navx", label="Map")


def test_map_nav_defaults(tmp_path: Path) -> None:
    p = tmp_path / "cfg.yaml"
    p.write_text(MAP_NAV_YAML)
    tab = load_config(p).tabs[0]
    assert tab.map_topic == "/map"
    assert tab.global_plan_topic == "/plan"
    assert tab.local_plan_topic == "/optimal_trajectory"
    assert tab.goal_topic == "/goal_pose"
    assert tab.map_frame == "map"
    assert tab.base_frame == "base_link"
    assert tab.map_save_path == "/var/lib/ros2/maps/slam_map"
    assert tab.map_reset_service == "/slam_toolbox/reset"
    assert tab.navigate_action == "/navigate_to_pose"


def test_map_nav_reset_and_navigate_overrides_kept() -> None:
    tab = TabConfig(id="m", type="map_nav", label="Map", map_reset_service="/slam/reset", navigate_action="/nav")
    assert tab.map_reset_service == "/slam/reset"
    assert tab.navigate_action == "/nav"


def test_reset_and_navigate_not_defaulted_for_other_tabs() -> None:
    tab = TabConfig(id="n", type="camera", label="Nav")
    assert tab.map_reset_service is None
    assert tab.navigate_action is None


def test_app_config_lists_reset_services_and_navigate_actions() -> None:
    cfg = AppConfig(
        tabs=[
            TabConfig(id="a", type="map_nav", label="A"),
            TabConfig(id="b", type="map_nav", label="B", map_reset_service="/r2", navigate_action="/n2"),
            TabConfig(id="c", type="camera", label="C"),
        ]
    )
    assert cfg.map_reset_services() == ["/r2", "/slam_toolbox/reset"]
    assert cfg.navigate_actions() == ["/n2", "/navigate_to_pose"]


def test_map_nav_overrides_kept() -> None:
    tab = TabConfig(id="m", type="map_nav", label="Map", map_topic="/slam/map", goal_topic="/g", map_frame="world")
    assert tab.map_topic == "/slam/map"
    assert tab.goal_topic == "/g"
    assert tab.map_frame == "world"
    assert tab.global_plan_topic == "/plan"


def test_map_nav_defaults_not_applied_to_other_tabs() -> None:
    tab = TabConfig(id="n", type="camera", label="Nav")
    assert tab.map_topic is None
    assert tab.goal_topic is None
    assert tab.map_save_path is None


def test_map_nav_empty_frame_rejected() -> None:
    with pytest.raises(ValidationError):
        TabConfig(id="m", type="map_nav", label="Map", map_frame="")


def test_all_subscribed_topics_include_map_nav_topics() -> None:
    cfg = AppConfig(tabs=[TabConfig(id="m", type="map_nav", label="Map")])
    topics = cfg.all_subscribed_topics()
    for t in ("/map", "/plan", "/optimal_trajectory", "/goal_pose"):
        assert t in topics


def test_publish_topics_include_map_nav_goal() -> None:
    cfg = AppConfig(tabs=[TabConfig(id="m", type="map_nav", label="Map", goal_topic="/nav/goal")])
    assert "/nav/goal" in cfg.publish_topics()


def test_topic_roles_from_map_nav_tabs() -> None:
    cfg = AppConfig(tabs=[TabConfig(id="m", type="map_nav", label="Map", map_topic="/m", local_plan_topic="/lp")])
    assert cfg.topic_roles() == {
        "/m": "map",
        "/plan": "path",
        "/lp": "path",
        "/goal_pose": "goal",
        "/local_costmap/published_footprint": "footprint",
        "/local_costmap/costmap": "costmap",
        "/client/gps/fix": "gps",
    }


def test_robot_pose_frames_and_frame_defaults() -> None:
    cfg = AppConfig(tabs=[TabConfig(id="m", type="map_nav", label="Map", map_frame="world", base_frame="base")])
    assert cfg.robot_pose_frames() == ("world", "base")
    assert cfg.frame_id_defaults() == {"/goal_pose": "world"}


def test_robot_pose_frames_none_without_map_nav() -> None:
    assert AppConfig(tabs=[]).robot_pose_frames() is None
    assert AppConfig(tabs=[]).topic_roles() == {}


# ---------------------------------------------------------------------------
# Serialization
# ---------------------------------------------------------------------------


def decode_png(b64: str) -> np.ndarray:
    """Decode a base64 PNG into a grayscale numpy array."""
    buf = np.frombuffer(base64.b64decode(b64), dtype=np.uint8)
    return cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)


def test_occupancy_grid_grey_levels_and_orientation() -> None:
    from web_ui.msg_serializer import (
        MAP_GREY_FREE,
        MAP_GREY_OCCUPIED,
        MAP_GREY_UNKNOWN,
        serialize_occupancy_grid,
    )

    # Row 0 of an OccupancyGrid is the bottom row (lowest y); image row 0 must be the top row.
    grid = make_grid(3, 2, [100, 0, -1, 0, 0, 0])
    out = serialize_occupancy_grid(grid)
    img = decode_png(out["png_b64"])
    assert img.shape == (2, 3)
    assert list(img[0]) == [MAP_GREY_FREE] * 3
    assert list(img[1]) == [MAP_GREY_OCCUPIED, MAP_GREY_FREE, MAP_GREY_UNKNOWN]
    assert len({MAP_GREY_FREE, MAP_GREY_OCCUPIED, MAP_GREY_UNKNOWN}) == 3


def test_occupancy_grid_thresholds() -> None:
    from web_ui.msg_serializer import (
        MAP_GREY_FREE,
        MAP_GREY_OCCUPIED,
        MAP_GREY_UNKNOWN,
        serialize_occupancy_grid,
    )

    img = decode_png(serialize_occupancy_grid(make_grid(4, 1, [10, 50, 70, -1]))["png_b64"])
    assert list(img[0]) == [MAP_GREY_FREE, MAP_GREY_UNKNOWN, MAP_GREY_OCCUPIED, MAP_GREY_UNKNOWN]


def test_occupancy_grid_metadata() -> None:
    from web_ui.msg_serializer import serialize_occupancy_grid

    out = serialize_occupancy_grid(make_grid(3, 2, [0] * 6, yaw=0.5))
    assert out["width"] == 3
    assert out["height"] == 2
    assert out["resolution"] == pytest.approx(0.05)
    assert out["origin"]["x"] == pytest.approx(-1.5)
    assert out["origin"]["y"] == pytest.approx(2.0)
    assert out["origin"]["yaw"] == pytest.approx(0.5)
    assert out["frame_id"] == "map"
    assert out["stamp"] == pytest.approx(12.5)
    assert "data" not in out


def test_occupancy_grid_size_mismatch_raises() -> None:
    from web_ui.msg_serializer import serialize_occupancy_grid

    with pytest.raises(ValueError):
        serialize_occupancy_grid(make_grid(3, 2, [0] * 5))


def test_path_serialization_small() -> None:
    from web_ui.msg_serializer import serialize_path

    out = serialize_path(make_path(3, frame_id="map"))
    assert out == {"frame_id": "map", "points": [[0.0, 0.0], [1.0, 0.5], [2.0, 1.0]]}


def test_path_downsampled_to_max_points_keeps_ends() -> None:
    from web_ui.msg_serializer import PATH_MAX_POINTS, serialize_path

    out = serialize_path(make_path(2000))
    assert PATH_MAX_POINTS == 500
    assert len(out["points"]) == PATH_MAX_POINTS
    assert out["points"][0] == [0.0, 0.0]
    assert out["points"][-1] == [1999.0, 999.5]
    xs = [p[0] for p in out["points"]]
    assert xs == sorted(xs)


def test_path_empty() -> None:
    from web_ui.msg_serializer import serialize_path

    assert serialize_path(make_path(0)) == {"frame_id": "map", "points": []}


def test_goal_pose_serialization() -> None:
    from web_ui.msg_serializer import serialize_goal_pose

    msg = SimpleNamespace(
        header=SimpleNamespace(frame_id="map", stamp=stamp()),
        pose=SimpleNamespace(position=SimpleNamespace(x=1.25, y=-2.0, z=0.0), orientation=yaw_quat(1.0)),
    )
    out = serialize_goal_pose(msg)
    assert out["frame_id"] == "map"
    assert out["x"] == pytest.approx(1.25)
    assert out["y"] == pytest.approx(-2.0)
    assert out["yaw"] == pytest.approx(1.0)


def test_quaternion_to_yaw() -> None:
    from web_ui.msg_serializer import quaternion_to_yaw

    q = yaw_quat(-2.5)
    assert quaternion_to_yaw(q.x, q.y, q.z, q.w) == pytest.approx(-2.5)


def test_transform_to_pose_dict() -> None:
    from web_ui.msg_serializer import transform_to_pose_dict

    out = transform_to_pose_dict(make_transform(1.0, 2.0, 0.75))
    assert out == {"x": 1.0, "y": 2.0, "yaw": pytest.approx(0.75), "frame_id": "map", "stamp": pytest.approx(3.25)}


def test_transform_points_2d() -> None:
    from web_ui.msg_serializer import transform_points_2d

    out = transform_points_2d([[1.0, 0.0]], 10.0, 5.0, math.pi / 2)
    assert out[0][0] == pytest.approx(10.0)
    assert out[0][1] == pytest.approx(6.0)


# ---------------------------------------------------------------------------
# Bridge subscriptions
# ---------------------------------------------------------------------------


def test_subscription_spec_map_uses_latched_qos() -> None:
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

    from web_ui.bridge import OccupancyGrid, subscription_spec
    from web_ui.msg_serializer import serialize_occupancy_grid

    msg_cls, qos, serializer = subscription_spec("/any_map_name", "map")
    assert msg_cls is OccupancyGrid
    assert qos.reliability is ReliabilityPolicy.RELIABLE
    assert qos.durability is DurabilityPolicy.TRANSIENT_LOCAL
    assert qos.history is HistoryPolicy.KEEP_LAST
    assert qos.depth == 1
    assert serializer is serialize_occupancy_grid


def test_subscription_spec_path_and_goal() -> None:
    from web_ui.bridge import Path as PathMsg
    from web_ui.bridge import PoseStamped, subscription_spec
    from web_ui.msg_serializer import serialize_goal_pose, serialize_path

    msg_cls, _, serializer = subscription_spec("/custom_plan", "path")
    assert msg_cls is PathMsg
    assert serializer is serialize_path
    msg_cls, _, serializer = subscription_spec("/custom_goal", "goal")
    assert msg_cls is PoseStamped
    assert serializer is serialize_goal_pose


def test_subscription_spec_unknown_topic_without_role_is_none() -> None:
    from web_ui.bridge import subscription_spec

    assert subscription_spec("/nope", None) is None


def test_bridge_init_subscribes_map_nav_topics_by_role() -> None:
    from web_ui.bridge import BridgeNode, OccupancyGrid

    created: list[tuple[Any, str]] = []
    with (
        patch("web_ui.bridge.Node.__init__", return_value=None),
        patch.object(
            BridgeNode, "create_subscription", create=True, side_effect=lambda c, t, cb, q: created.append((c, t))
        ),
        patch.object(BridgeNode, "create_timer", create=True),
        patch.object(BridgeNode, "get_logger", create=True),
        patch.object(BridgeNode, "create_client", create=True),
        patch("web_ui.bridge.Buffer"),
        patch("web_ui.bridge.TransformListener"),
    ):
        BridgeNode(
            topics=["/map", "/plan", "/goal_pose"],
            allowed_publish_topics={"/goal_pose"},
            topic_roles={"/map": "map", "/plan": "path", "/goal_pose": "goal"},
            robot_pose_frames=("map", "base_link"),
        )
    topics = {t for _, t in created}
    assert topics == {"/map", "/plan", "/goal_pose"}
    assert (OccupancyGrid, "/map") in created


def test_bridge_map_callback_stores_serialized_map() -> None:
    node = make_bridge()
    from web_ui.msg_serializer import serialize_occupancy_grid

    cb = node._make_callback("/map", serialize_occupancy_grid, "map")
    cb(make_grid(2, 1, [0, 100]))
    env = node.flush_dirty()
    assert env[0]["topic"] == "/map"
    assert "png_b64" in env[0]["data"]


def test_bridge_path_in_other_frame_transformed_to_map() -> None:
    from web_ui.msg_serializer import serialize_path

    tf_buffer = MagicMock()
    tf_buffer.lookup_transform.return_value = make_transform(10.0, 0.0, 0.0)
    node = make_bridge(_tf_buffer=tf_buffer, _robot_pose_frames=("map", "base_link"))
    cb = node._make_callback("/optimal_trajectory", serialize_path, "path")
    cb(make_path(2, frame_id="odom"))
    data = node.flush_dirty()[0]["data"]
    assert data["frame_id"] == "map"
    assert data["points"] == [[10.0, 0.0], [11.0, 0.5]]


def test_bridge_path_in_other_frame_dropped_without_tf() -> None:
    from tf2_ros import TransformException

    from web_ui.msg_serializer import serialize_path

    tf_buffer = MagicMock()
    tf_buffer.lookup_transform.side_effect = TransformException("no tf")
    node = make_bridge(_tf_buffer=tf_buffer, _robot_pose_frames=("map", "base_link"))
    node._make_callback("/optimal_trajectory", serialize_path, "path")(make_path(2, frame_id="odom"))
    assert node.flush_dirty() == []


def test_bridge_goal_in_other_frame_transformed_to_map() -> None:
    from web_ui.msg_serializer import serialize_goal_pose

    tf_buffer = MagicMock()
    tf_buffer.lookup_transform.return_value = make_transform(10.0, 5.0, math.pi / 2)
    node = make_bridge(_tf_buffer=tf_buffer, _robot_pose_frames=("map", "base_link"))
    goal = SimpleNamespace(
        header=SimpleNamespace(frame_id="odom", stamp=stamp()),
        pose=SimpleNamespace(position=SimpleNamespace(x=1.0, y=0.0, z=0.0), orientation=yaw_quat(0.25)),
    )
    node._make_callback("/goal_pose", serialize_goal_pose, "goal")(goal)
    data = node.flush_dirty()[0]["data"]
    assert data["frame_id"] == "map"
    assert data["x"] == pytest.approx(10.0)
    assert data["y"] == pytest.approx(6.0)
    assert data["yaw"] == pytest.approx(0.25 + math.pi / 2)


@pytest.mark.parametrize(
    "data",
    [
        {"frame_id": "odom", "x": 1.0, "y": 2.0, "yaw": 0.5},
        {"frame_id": "odom", "points": [[1.0, 2.0]]},
    ],
)
def test_non_map_nav_topic_with_pose_like_keys_passes_through(data: dict[str, Any]) -> None:
    tf_buffer = MagicMock()
    tf_buffer.lookup_transform.return_value = make_transform(10.0, 0.0, 0.0)
    node = make_bridge(_tf_buffer=tf_buffer, _robot_pose_frames=("map", "base_link"))
    node._make_callback("/controller/odom", lambda _msg: dict(data), None)(object())
    assert node.flush_dirty()[0]["data"] == data
    tf_buffer.lookup_transform.assert_not_called()


def test_non_map_nav_topic_not_dropped_without_tf() -> None:
    from tf2_ros import TransformException

    tf_buffer = MagicMock()
    tf_buffer.lookup_transform.side_effect = TransformException("no tf")
    node = make_bridge(_tf_buffer=tf_buffer, _robot_pose_frames=("map", "base_link"))
    data = {"frame_id": "odom", "x": 1.0, "y": 2.0, "yaw": 0.5}
    node._make_callback("/controller/odom", lambda _msg: dict(data), None)(object())
    assert node.flush_dirty()[0]["data"] == data


def test_map_role_not_transformed() -> None:
    tf_buffer = MagicMock()
    node = make_bridge(_tf_buffer=tf_buffer, _robot_pose_frames=("map", "base_link"))
    data = {"frame_id": "odom", "png_b64": "x", "yaw": 0.1}
    node._make_callback("/map", lambda _msg: dict(data), "map")(object())
    assert node.flush_dirty()[0]["data"] == data
    tf_buffer.lookup_transform.assert_not_called()


def test_bridge_init_passes_role_to_callbacks() -> None:
    from web_ui.bridge import BridgeNode

    roles_seen: dict[str, str | None] = {}

    def fake_make_callback(self: Any, topic: str, serializer: Any, role: str | None) -> Any:
        roles_seen[topic] = role
        return lambda msg: None

    with (
        patch("web_ui.bridge.Node.__init__", return_value=None),
        patch.object(BridgeNode, "create_subscription", create=True),
        patch.object(BridgeNode, "create_timer", create=True),
        patch.object(BridgeNode, "get_logger", create=True),
        patch.object(BridgeNode, "create_client", create=True),
        patch.object(BridgeNode, "_make_callback", fake_make_callback),
        patch("web_ui.bridge.Buffer"),
        patch("web_ui.bridge.TransformListener"),
    ):
        BridgeNode(
            topics=["/plan", "/goal_pose", "/controller/odom"],
            allowed_publish_topics=set(),
            topic_roles={"/plan": "path", "/goal_pose": "goal"},
            robot_pose_frames=("map", "base_link"),
        )
    assert roles_seen == {"/plan": "path", "/goal_pose": "goal", "/controller/odom": None}


def test_latest_envelopes_returns_all_cached() -> None:
    node = make_bridge()
    node._latest["/map"] = {"topic": "/map", "data": {"png_b64": "x"}}
    node.flush_dirty()
    assert node.latest_envelopes() == [{"topic": "/map", "data": {"png_b64": "x"}}]


# ---------------------------------------------------------------------------
# Robot pose
# ---------------------------------------------------------------------------


def test_robot_pose_published_when_tf_available() -> None:
    from web_ui.bridge import ROBOT_POSE_TOPIC

    tf_buffer = MagicMock()
    tf_buffer.lookup_transform.return_value = make_transform(1.0, -1.0, 0.3)
    node = make_bridge(_tf_buffer=tf_buffer, _robot_pose_frames=("map", "base_link"))
    assert node.update_robot_pose() is True
    env = node.flush_dirty()
    assert ROBOT_POSE_TOPIC == "/web_ui/robot_pose"
    assert env[0]["topic"] == ROBOT_POSE_TOPIC
    assert env[0]["data"]["x"] == 1.0
    assert env[0]["data"]["yaw"] == pytest.approx(0.3)
    assert tf_buffer.lookup_transform.call_args[0][:2] == ("map", "base_link")


def test_robot_pose_omitted_when_tf_unavailable() -> None:
    from tf2_ros import TransformException

    tf_buffer = MagicMock()
    tf_buffer.lookup_transform.side_effect = TransformException("map frame does not exist")
    node = make_bridge(_tf_buffer=tf_buffer, _robot_pose_frames=("map", "base_link"))
    assert node.update_robot_pose() is False
    assert node.flush_dirty() == []
    assert node.latest_envelopes() == []


def test_robot_pose_unchanged_same_stamp_not_marked_dirty_again() -> None:
    tf_buffer = MagicMock()
    tf_buffer.lookup_transform.return_value = make_transform(1.0, -1.0, 0.3)
    node = make_bridge(_tf_buffer=tf_buffer, _robot_pose_frames=("map", "base_link"))
    assert node.update_robot_pose() is True
    assert len(node.flush_dirty()) == 1
    assert node.update_robot_pose() is False
    assert node.flush_dirty() == []
    assert len(node.latest_envelopes()) == 1


def test_robot_pose_advanced_stamp_marked_dirty() -> None:
    from web_ui.bridge import ROBOT_POSE_TOPIC

    tf_buffer = MagicMock()
    tf = make_transform(1.0, -1.0, 0.3)
    tf_buffer.lookup_transform.return_value = tf
    node = make_bridge(_tf_buffer=tf_buffer, _robot_pose_frames=("map", "base_link"))
    node.update_robot_pose()
    node.flush_dirty()
    tf.header.stamp = stamp(3, 300_000_000)
    assert node.update_robot_pose() is True
    env = node.flush_dirty()
    assert env[0]["topic"] == ROBOT_POSE_TOPIC
    assert env[0]["data"]["stamp"] == pytest.approx(3.3)


def test_robot_pose_changed_same_stamp_marked_dirty() -> None:
    tf_buffer = MagicMock()
    tf = make_transform(1.0, -1.0, 0.3)
    tf_buffer.lookup_transform.return_value = tf
    node = make_bridge(_tf_buffer=tf_buffer, _robot_pose_frames=("map", "base_link"))
    node.update_robot_pose()
    node.flush_dirty()
    tf.transform.translation.x = 1.5
    assert node.update_robot_pose() is True
    assert node.flush_dirty()[0]["data"]["x"] == 1.5


def test_stale_robot_pose_not_cached_or_in_snapshot() -> None:
    from web_ui.bridge import ROBOT_POSE_STALE_S

    tf_buffer = MagicMock()
    tf_buffer.lookup_transform.return_value = make_transform(1.0, -1.0, 0.3)  # stamp 3.25 s
    node = make_bridge(_tf_buffer=tf_buffer, _robot_pose_frames=("map", "base_link"))
    node.get_clock = lambda: fake_clock(3.25 + ROBOT_POSE_STALE_S + 0.1)
    assert node.update_robot_pose() is False
    assert node.flush_dirty() == []
    assert node.latest_envelopes() == []


def test_cached_robot_pose_cleared_when_tf_goes_stale() -> None:
    from web_ui.bridge import ROBOT_POSE_STALE_S

    tf_buffer = MagicMock()
    tf_buffer.lookup_transform.return_value = make_transform(1.0, -1.0, 0.3)  # stamp 3.25 s
    node = make_bridge(_tf_buffer=tf_buffer, _robot_pose_frames=("map", "base_link"))
    node._latest["/map"] = {"topic": "/map", "data": {"png_b64": "x"}}
    assert node.update_robot_pose() is True
    node.get_clock = lambda: fake_clock(3.25 + ROBOT_POSE_STALE_S + 0.1)
    assert node.update_robot_pose() is False
    assert node.flush_dirty() == []  # the pending (now stale) pose is not broadcast either
    assert node.latest_envelopes() == [{"topic": "/map", "data": {"png_b64": "x"}}]


def test_cached_robot_pose_cleared_when_tf_lost() -> None:
    from tf2_ros import TransformException

    tf_buffer = MagicMock()
    tf_buffer.lookup_transform.return_value = make_transform(1.0, -1.0, 0.3)
    node = make_bridge(_tf_buffer=tf_buffer, _robot_pose_frames=("map", "base_link"))
    assert node.update_robot_pose() is True
    tf_buffer.lookup_transform.side_effect = TransformException("map frame gone")
    assert node.update_robot_pose() is False
    assert node.flush_dirty() == []
    assert node.latest_envelopes() == []


def test_robot_pose_noop_without_map_nav() -> None:
    node = make_bridge()
    assert node.update_robot_pose() is False


# ---------------------------------------------------------------------------
# Goal publishing / _dict_to_ros_msg
# ---------------------------------------------------------------------------


class Typed:
    """Minimal rclpy-like message: typed setters reject wrong Python types like generated rclpy code."""

    __slots__: tuple[str, ...] = ()
    _types: dict[str, type] = {}

    def __setattr__(self, name: str, value: Any) -> None:
        expected = self._types.get(name)
        if expected is not None and expected is not object:
            assert isinstance(value, expected) and not (
                expected is int and isinstance(value, bool)
            ), f"{name} expects {expected.__name__}, got {type(value).__name__}"
        object.__setattr__(self, name, value)


class TimeMsg(Typed):
    __slots__ = ("_sec", "_nanosec", "sec", "nanosec")
    _types = {"sec": int, "nanosec": int}

    def __init__(self) -> None:
        self.sec = 0
        self.nanosec = 0


class HeaderMsg(Typed):
    __slots__ = ("_stamp", "_frame_id", "stamp", "frame_id")
    _types = {"frame_id": str, "stamp": object}

    def __init__(self) -> None:
        self.stamp = TimeMsg()
        self.frame_id = ""


class PointMsg(Typed):
    __slots__ = ("_x", "_y", "_z", "x", "y", "z")
    _types = {"x": float, "y": float, "z": float}

    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class QuatMsg(Typed):
    __slots__ = ("_x", "_y", "_z", "_w", "x", "y", "z", "w")
    _types = {"x": float, "y": float, "z": float, "w": float}

    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 1.0


class PoseMsg(Typed):
    __slots__ = ("_position", "_orientation", "position", "orientation")
    _types = {"position": object, "orientation": object}

    def __init__(self) -> None:
        self.position = PointMsg()
        self.orientation = QuatMsg()


class PoseStampedMsg(Typed):
    __slots__ = ("_header", "_pose", "header", "pose")
    _types = {"header": object, "pose": object}

    def __init__(self) -> None:
        self.header = HeaderMsg()
        self.pose = PoseMsg()


class JointStateMsg(Typed):
    __slots__ = ("_name", "_position", "name", "position")
    _types = {"name": object, "position": object}

    def __init__(self) -> None:
        self.name: list[str] = []
        self.position = array.array("d")


def test_dict_to_ros_msg_int_and_float_fields() -> None:
    from web_ui.bridge import _dict_to_ros_msg

    msg = _dict_to_ros_msg(
        PoseStampedMsg(),
        {
            "header": {"frame_id": "map", "stamp": {"sec": 5, "nanosec": 7}},
            "pose": {"position": {"x": 1, "y": 2.5, "z": 0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
        },
    )
    assert msg.header.stamp.sec == 5 and isinstance(msg.header.stamp.sec, int)
    assert msg.header.stamp.nanosec == 7 and isinstance(msg.header.stamp.nanosec, int)
    assert msg.pose.position.x == 1.0 and isinstance(msg.pose.position.x, float)
    assert isinstance(msg.pose.orientation.w, float)
    assert msg.header.frame_id == "map"


def test_dict_to_ros_msg_integral_float_into_int_field() -> None:
    from web_ui.bridge import _dict_to_ros_msg

    msg = _dict_to_ros_msg(TimeMsg(), {"sec": 3.0})
    assert msg.sec == 3 and isinstance(msg.sec, int)


def test_dict_to_ros_msg_fractional_float_into_int_field_raises() -> None:
    from web_ui.bridge import _dict_to_ros_msg

    with pytest.raises(TypeError):
        _dict_to_ros_msg(TimeMsg(), {"sec": 3.5})


def test_dict_to_ros_msg_wrong_type_raises() -> None:
    from web_ui.bridge import _dict_to_ros_msg

    with pytest.raises(TypeError):
        _dict_to_ros_msg(PointMsg(), {"x": "abc"})


def test_dict_to_ros_msg_float_array_from_ints() -> None:
    from web_ui.bridge import _dict_to_ros_msg

    msg = _dict_to_ros_msg(JointStateMsg(), {"name": ["a", "b"], "position": [0, 1.5]})
    assert list(msg.position) == [0.0, 1.5]
    assert all(isinstance(v, float) for v in msg.position)
    assert msg.name == ["a", "b"]


def test_publish_dict_fills_stamp_and_frame() -> None:
    clock_stamp = TimeMsg()
    clock_stamp.sec = 100
    clock_stamp.nanosec = 42
    pub = MagicMock()
    node = make_bridge(
        _allowed_publish_topics={"/goal_pose"},
        _frame_id_defaults={"/goal_pose": "map"},
    )
    node.publishers_["/goal_pose"] = (pub, PoseStampedMsg)
    node.get_clock = MagicMock()
    node.get_clock.return_value.now.return_value.to_msg.return_value = clock_stamp
    node.publish_dict("/goal_pose", {"header": {"stamp": {"sec": 0, "nanosec": 0}}, "pose": {"position": {"x": 1}}})
    sent = pub.publish.call_args[0][0]
    assert sent.header.stamp.sec == 100
    assert sent.header.stamp.nanosec == 42
    assert sent.header.frame_id == "map"
    assert sent.pose.position.x == 1.0


def test_publish_dict_keeps_client_stamp_and_frame() -> None:
    pub = MagicMock()
    node = make_bridge(_allowed_publish_topics={"/goal_pose"}, _frame_id_defaults={"/goal_pose": "map"})
    node.publishers_["/goal_pose"] = (pub, PoseStampedMsg)
    node.get_clock = MagicMock()
    node.publish_dict("/goal_pose", {"header": {"frame_id": "odom", "stamp": {"sec": 9, "nanosec": 1}}})
    sent = pub.publish.call_args[0][0]
    assert sent.header.stamp.sec == 9
    assert sent.header.frame_id == "odom"
    node.get_clock.assert_not_called()


def test_publish_dict_bad_payload_not_published() -> None:
    pub = MagicMock()
    node = make_bridge(_allowed_publish_topics={"/goal_pose"})
    node.publishers_["/goal_pose"] = (pub, PoseStampedMsg)
    node.get_clock = MagicMock()
    node.publish_dict("/goal_pose", {"header": {"stamp": {"sec": 1.5}}})
    pub.publish.assert_not_called()


# ---------------------------------------------------------------------------
# Save map
# ---------------------------------------------------------------------------


def test_serialize_map_async_unavailable_returns_none() -> None:
    client = MagicMock()
    client.service_is_ready.return_value = False
    node = make_bridge(_serialize_map_client=client)
    assert node.serialize_map_async("/tmp/m") is None
    client.call_async.assert_not_called()


def test_serialize_map_async_sends_filename() -> None:
    client = MagicMock()
    client.service_is_ready.return_value = True
    node = make_bridge(_serialize_map_client=client)
    fut = node.serialize_map_async("/var/lib/ros2/maps/slam_map")
    assert fut is client.call_async.return_value
    request = client.call_async.call_args[0][0]
    assert request.filename == "/var/lib/ros2/maps/slam_map"


class FakeFuture:
    """rclpy Future stand-in: resolves immediately (result given) or never (timeout)."""

    def __init__(self, result: Any = None, resolve: bool = True) -> None:
        self._result = result
        self._resolve = resolve
        self.cancelled = False

    def add_done_callback(self, cb: Any) -> None:
        if self._resolve:
            cb(self)

    def result(self) -> Any:
        return self._result

    def exception(self) -> Any:
        return None

    def cancel(self) -> None:
        self.cancelled = True


@pytest.fixture
def map_app(tmp_path: Path, urdf_dir: Path) -> Any:
    """Return (app factory) building an app with a map_nav tab and a given bridge."""
    from web_ui.server import build_app

    config = AppConfig(tabs=[TabConfig(id="map", type="map_nav", label="Map", map_save_path="/maps/test")])

    def factory(bridge: Any) -> TestClient:
        return TestClient(build_app(config=config, urdf_dir=urdf_dir, static_dir=tmp_path / "none", bridge_node=bridge))

    return factory


def test_save_map_success(map_app: Any) -> None:
    bridge = MagicMock()
    bridge.serialize_map_async.return_value = FakeFuture(SimpleNamespace(result=0))
    resp = map_app(bridge).post("/api/map/save?tab=map")
    assert resp.status_code == 200
    body = resp.json()
    assert body["ok"] is True
    assert "/maps/test" in body["message"]
    bridge.serialize_map_async.assert_called_once_with("/maps/test")


def test_save_map_slam_failure_result(map_app: Any) -> None:
    bridge = MagicMock()
    bridge.serialize_map_async.return_value = FakeFuture(SimpleNamespace(result=255))
    resp = map_app(bridge).post("/api/map/save?tab=map")
    assert resp.status_code == 500
    assert resp.json()["ok"] is False


def test_save_map_service_unavailable(map_app: Any) -> None:
    bridge = MagicMock()
    bridge.serialize_map_async.return_value = None
    resp = map_app(bridge).post("/api/map/save?tab=map")
    assert resp.status_code == 503
    body = resp.json()
    assert body["ok"] is False
    assert "unavailable" in body["message"]


def test_save_map_timeout(map_app: Any) -> None:
    bridge = MagicMock()
    fut = FakeFuture(resolve=False)
    bridge.serialize_map_async.return_value = fut
    with patch("web_ui.server.SAVE_MAP_TIMEOUT_S", 0.05):
        resp = map_app(bridge).post("/api/map/save?tab=map")
    assert resp.status_code == 504
    assert resp.json()["ok"] is False
    assert "timed out" in resp.json()["message"]
    assert fut.cancelled


def test_save_map_unknown_tab(map_app: Any) -> None:
    resp = map_app(MagicMock()).post("/api/map/save?tab=nope")
    assert resp.status_code == 404
    assert resp.json()["ok"] is False


def test_save_map_no_bridge(map_app: Any) -> None:
    resp = map_app(None).post("/api/map/save?tab=map")
    assert resp.status_code == 503
    assert resp.json()["ok"] is False


# ---------------------------------------------------------------------------
# Reset map / stop navigation
# ---------------------------------------------------------------------------


def ready_client(ready: bool = True) -> MagicMock:
    """Return a service client mock whose service_is_ready() returns ready."""
    client = MagicMock()
    client.service_is_ready.return_value = ready
    return client


def test_bridge_init_creates_reset_and_cancel_clients() -> None:
    created: list[tuple[Any, str]] = []
    node = make_bridge()
    node.create_client = lambda srv, name: created.append((srv, name)) or ready_client()
    node.create_service_clients(["/slam_toolbox/reset"], ["/navigate_to_pose"])
    names = [name for _, name in created]
    assert names == ["/slam_toolbox/reset", "/navigate_to_pose/_action/cancel_goal"]
    assert set(node._reset_map_clients) == {"/slam_toolbox/reset"}
    assert set(node._cancel_goal_clients) == {"/navigate_to_pose"}


def test_reset_map_async_unavailable_returns_none() -> None:
    client = ready_client(False)
    node = make_bridge(_reset_map_clients={"/slam_toolbox/reset": client})
    assert node.reset_map_async("/slam_toolbox/reset") is None
    assert node.reset_map_async("/unknown/reset") is None
    client.call_async.assert_not_called()


def test_reset_map_async_does_not_pause_measurements() -> None:
    client = ready_client()
    node = make_bridge(_reset_map_clients={"/slam_toolbox/reset": client})
    fut = node.reset_map_async("/slam_toolbox/reset")
    assert fut is client.call_async.return_value
    request = client.call_async.call_args[0][0]
    assert request.pause_new_measurements is False


def test_cancel_all_goals_async_unavailable_returns_none() -> None:
    client = ready_client(False)
    node = make_bridge(_cancel_goal_clients={"/navigate_to_pose": client})
    assert node.cancel_all_goals_async("/navigate_to_pose") is None
    assert node.cancel_all_goals_async("/other") is None
    client.call_async.assert_not_called()


def test_cancel_all_goals_request_is_zero_uuid_and_zero_stamp() -> None:
    client = ready_client()
    node = make_bridge(_cancel_goal_clients={"/navigate_to_pose": client})
    fut = node.cancel_all_goals_async("/navigate_to_pose")
    assert fut is client.call_async.return_value
    info = client.call_async.call_args[0][0].goal_info
    assert list(info.goal_id.uuid) == [0] * 16
    assert info.stamp.sec == 0
    assert info.stamp.nanosec == 0


def test_clear_and_notify_drops_cache_and_broadcasts_null_once() -> None:
    node = make_bridge()
    node.store("/map", {"png_b64": "abc"})
    node.clear_and_notify("/map")
    assert node.latest_envelopes() == []
    assert node.flush_dirty() == [{"topic": "/map", "data": None}]
    assert node.flush_dirty() == []


def test_new_data_after_clear_replaces_pending_clear() -> None:
    node = make_bridge()
    node.clear_and_notify("/map")
    node.store("/map", {"png_b64": "new"})
    assert node.flush_dirty() == [{"topic": "/map", "data": {"png_b64": "new"}}]


@pytest.fixture
def nav_app(tmp_path: Path, urdf_dir: Path) -> Any:
    """Return an app factory with a map_nav tab using custom reset service and navigate action."""
    from web_ui.server import build_app

    config = AppConfig(
        tabs=[
            TabConfig(
                id="map",
                type="map_nav",
                label="Map",
                map_topic="/slam/map",
                goal_topic="/goal",
                map_reset_service="/slam/reset",
                navigate_action="/nav",
            )
        ]
    )

    def factory(bridge: Any) -> TestClient:
        return TestClient(build_app(config=config, urdf_dir=urdf_dir, static_dir=tmp_path / "none", bridge_node=bridge))

    return factory


def test_reset_map_success_clears_cached_map(nav_app: Any) -> None:
    bridge = MagicMock()
    bridge.reset_map_async.return_value = FakeFuture(SimpleNamespace(result=0))
    resp = nav_app(bridge).post("/api/map/reset?tab=map")
    assert resp.status_code == 200
    assert resp.json()["ok"] is True
    bridge.reset_map_async.assert_called_once_with("/slam/reset")
    bridge.clear_and_notify.assert_called_once_with("/slam/map")


def test_reset_map_slam_failure_keeps_map(nav_app: Any) -> None:
    bridge = MagicMock()
    bridge.reset_map_async.return_value = FakeFuture(SimpleNamespace(result=255))
    resp = nav_app(bridge).post("/api/map/reset?tab=map")
    assert resp.status_code == 500
    assert resp.json()["ok"] is False
    assert "255" in resp.json()["message"]
    bridge.clear_and_notify.assert_not_called()


def test_reset_map_service_unavailable(nav_app: Any) -> None:
    bridge = MagicMock()
    bridge.reset_map_async.return_value = None
    resp = nav_app(bridge).post("/api/map/reset?tab=map")
    assert resp.status_code == 503
    assert resp.json()["ok"] is False
    assert "/slam/reset unavailable" in resp.json()["message"]
    bridge.clear_and_notify.assert_not_called()


def test_reset_map_timeout(nav_app: Any) -> None:
    bridge = MagicMock()
    fut = FakeFuture(resolve=False)
    bridge.reset_map_async.return_value = fut
    with patch("web_ui.server.RESET_MAP_TIMEOUT_S", 0.05):
        resp = nav_app(bridge).post("/api/map/reset?tab=map")
    assert resp.status_code == 504
    assert "timed out" in resp.json()["message"]
    assert fut.cancelled
    bridge.clear_and_notify.assert_not_called()


def test_reset_map_unknown_tab(nav_app: Any) -> None:
    resp = nav_app(MagicMock()).post("/api/map/reset?tab=nope")
    assert resp.status_code == 404
    assert resp.json()["ok"] is False


def test_reset_map_no_bridge(nav_app: Any) -> None:
    resp = nav_app(None).post("/api/map/reset?tab=map")
    assert resp.status_code == 503
    assert resp.json()["ok"] is False


def test_reset_map_does_not_touch_saved_posegraph(map_app: Any, tmp_path: Path) -> None:
    saved = tmp_path / "slam_map.posegraph"
    saved.write_text("graph")
    bridge = MagicMock()
    bridge.reset_map_async.return_value = FakeFuture(SimpleNamespace(result=0))
    resp = map_app(bridge).post("/api/map/reset?tab=map")
    assert resp.status_code == 200
    assert saved.read_text() == "graph"
    bridge.serialize_map_async.assert_not_called()
    bridge.reset_map_async.assert_called_once_with("/slam_toolbox/reset")


def test_stop_nav_success_clears_cached_goal(nav_app: Any) -> None:
    bridge = MagicMock()
    bridge.cancel_all_goals_async.return_value = FakeFuture(SimpleNamespace(return_code=0, goals_canceling=[1, 2]))
    resp = nav_app(bridge).post("/api/nav/stop?tab=map")
    assert resp.status_code == 200
    body = resp.json()
    assert body["ok"] is True
    assert "2 goal" in body["message"]
    bridge.cancel_all_goals_async.assert_called_once_with("/nav")
    bridge.clear_and_notify.assert_called_once_with("/goal")


def test_stop_nav_no_active_goal_still_ok(nav_app: Any) -> None:
    bridge = MagicMock()
    bridge.cancel_all_goals_async.return_value = FakeFuture(SimpleNamespace(return_code=0, goals_canceling=[]))
    resp = nav_app(bridge).post("/api/nav/stop?tab=map")
    assert resp.status_code == 200
    assert "no active goal" in resp.json()["message"]
    bridge.clear_and_notify.assert_called_once_with("/goal")


def test_stop_nav_rejected(nav_app: Any) -> None:
    bridge = MagicMock()
    bridge.cancel_all_goals_async.return_value = FakeFuture(SimpleNamespace(return_code=1, goals_canceling=[]))
    resp = nav_app(bridge).post("/api/nav/stop?tab=map")
    assert resp.status_code == 500
    assert resp.json()["ok"] is False
    assert "rejected" in resp.json()["message"]
    bridge.clear_and_notify.assert_not_called()


def test_stop_nav_unavailable(nav_app: Any) -> None:
    bridge = MagicMock()
    bridge.cancel_all_goals_async.return_value = None
    resp = nav_app(bridge).post("/api/nav/stop?tab=map")
    assert resp.status_code == 503
    assert "/nav/_action/cancel_goal unavailable" in resp.json()["message"]
    bridge.clear_and_notify.assert_not_called()


def test_stop_nav_timeout(nav_app: Any) -> None:
    bridge = MagicMock()
    fut = FakeFuture(resolve=False)
    bridge.cancel_all_goals_async.return_value = fut
    with patch("web_ui.server.NAV_STOP_TIMEOUT_S", 0.05):
        resp = nav_app(bridge).post("/api/nav/stop?tab=map")
    assert resp.status_code == 504
    assert "timed out" in resp.json()["message"]
    assert fut.cancelled
    bridge.clear_and_notify.assert_not_called()


def test_stop_nav_unknown_tab_and_no_bridge(nav_app: Any) -> None:
    assert nav_app(MagicMock()).post("/api/nav/stop?tab=nope").status_code == 404
    resp = nav_app(None).post("/api/nav/stop?tab=map")
    assert resp.status_code == 503
    assert resp.json()["ok"] is False


def test_ws_connect_sends_cached_snapshot(map_app: Any) -> None:
    bridge = MagicMock()
    bridge.latest_envelopes.return_value = [{"topic": "/map", "data": {"png_b64": "abc"}}]
    bridge.flush_dirty.return_value = []
    with map_app(bridge).websocket_connect("/ws") as ws:
        assert ws.receive_json() == {"topic": "/map", "data": {"png_b64": "abc"}}


def test_default_config_has_map_nav_tab() -> None:
    cfg = load_config(Path(__file__).parent.parent / "config" / "default.yaml")
    tab = cfg.map_nav_tabs()[0]
    assert tab.map_topic == "/map"
    assert "/goal_pose" in cfg.publish_topics()
    assert cfg.robot_pose_frames() == ("map", "base_link")
    assert tab.map_reset_service == "/slam_toolbox/reset"
    assert tab.navigate_action == "/navigate_to_pose"


def test_default_yaml_map_tab_uses_nav2_local_plan_topic() -> None:
    """The shipped default config shows MPPI's local plan (/optimal_trajectory), which Nav2 actually publishes."""
    cfg = load_config(Path(__file__).resolve().parents[1] / "config" / "default.yaml")
    map_tabs = [t for t in cfg.tabs if t.type == "map_nav"]
    assert map_tabs
    assert all(t.local_plan_topic == "/optimal_trajectory" for t in map_tabs)


# --- robot footprint (Nav2 published footprint drawn instead of the robot arrow) ---


def make_polygon(points: list[tuple[float, float]], frame_id: str = "odom") -> MagicMock:
    msg = MagicMock()
    msg.header.frame_id = frame_id
    msg.polygon.points = [MagicMock(x=x, y=y, z=0.0) for x, y in points]
    return msg


def test_map_nav_footprint_topic_default_and_role() -> None:
    tab = TabConfig(id="m", type="map_nav", label="Map")
    assert tab.footprint_topic == "/local_costmap/published_footprint"
    cfg = AppConfig(tabs=[tab])
    assert "/local_costmap/published_footprint" in cfg.all_subscribed_topics()
    assert cfg.topic_roles()["/local_costmap/published_footprint"] == "footprint"


def test_footprint_not_defaulted_for_other_tabs() -> None:
    assert TabConfig(id="n", type="camera", label="Nav").footprint_topic is None


def test_serialize_polygon_points() -> None:
    from web_ui.msg_serializer import serialize_polygon

    data = serialize_polygon(make_polygon([(0.235, 0.193), (0.235, -0.193), (-0.235, -0.193), (-0.235, 0.193)]))
    assert data == {
        "frame_id": "odom",
        "points": [[0.235, 0.193], [0.235, -0.193], [-0.235, -0.193], [-0.235, 0.193]],
    }


def test_footprint_role_uses_polygon_stamped_and_map_frame_transform() -> None:
    from web_ui.bridge import ROLE_SPECS
    from web_ui.msg_serializer import serialize_polygon

    msg_type, _qos, serializer = ROLE_SPECS["footprint"]
    assert msg_type.__name__ == "PolygonStamped" and serializer is serialize_polygon
    tf_buffer = MagicMock()
    tf_buffer.lookup_transform.return_value = make_transform(10.0, 0.0, 0.0)
    node = make_bridge(_tf_buffer=tf_buffer, _robot_pose_frames=("map", "base_link"))
    cb = node._make_callback("/local_costmap/published_footprint", serialize_polygon, "footprint")
    cb(make_polygon([(1.0, 0.0), (0.0, 1.0)]))
    data = node.flush_dirty()[0]["data"]
    assert data == {"frame_id": "map", "points": [[11.0, 0.0], [10.0, 1.0]]}
