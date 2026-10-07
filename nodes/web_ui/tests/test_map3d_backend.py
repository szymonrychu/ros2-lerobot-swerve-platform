"""Tests for the merged 3D map tab backend: costmap layer, GPS + anchor fit, tile proxy, arm home, cache headers."""

from __future__ import annotations

import array
import math
import threading
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock, patch

import cv2
import httpx
import numpy as np
import pytest
from fastapi.testclient import TestClient
from pydantic import ValidationError

from web_ui.config import AppConfig, TabConfig, load_config

DEFAULT_YAML = Path(__file__).resolve().parents[1] / "config" / "default.yaml"
REMOVED_TAB_TYPES = ("effector_graph", "nav_local", "nav_gps", "scene3d", "robot_status")
ANCHOR_LAT = 52.2297
ANCHOR_LON = 21.0122
PNG_BYTES = b"\x89PNG\r\n\x1a\nfake-tile"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def yaw_quat(yaw: float) -> SimpleNamespace:
    """Return a planar quaternion namespace for a yaw angle."""
    return SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def stamp(sec: int = 0, nanosec: int = 0) -> SimpleNamespace:
    """Return a builtin_interfaces/Time-like namespace."""
    return SimpleNamespace(sec=sec, nanosec=nanosec)


def make_costmap(
    width: int, height: int, data: list[int], frame_id: str = "odom", x: float = 1.0, y: float = 0.0
) -> SimpleNamespace:
    """Return a Nav2 costmap OccupancyGrid-like namespace."""
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame_id, stamp=stamp(7)),
        info=SimpleNamespace(
            width=width,
            height=height,
            resolution=0.05,
            origin=SimpleNamespace(position=SimpleNamespace(x=x, y=y, z=0.0), orientation=yaw_quat(0.0)),
        ),
        data=array.array("b", data),
    )


def make_fix(lat: float, lon: float, status: int = 0, sec: int = 100, nanosec: int = 0) -> SimpleNamespace:
    """Return a sensor_msgs/NavSatFix-like namespace."""
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="gps_link", stamp=stamp(sec, nanosec)),
        status=SimpleNamespace(status=status, service=1),
        latitude=lat,
        longitude=lon,
        altitude=110.0,
        position_covariance=[0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.04],
        position_covariance_type=2,
    )


def make_transform(x: float, y: float, yaw: float, frame_id: str = "map", sec: int = 100) -> SimpleNamespace:
    """Return a geometry_msgs/TransformStamped-like namespace."""
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame_id, stamp=stamp(sec)),
        child_frame_id="base_link",
        transform=SimpleNamespace(translation=SimpleNamespace(x=x, y=y, z=0.0), rotation=yaw_quat(yaw)),
    )


def make_bridge(**attrs: Any) -> Any:
    """Construct a BridgeNode without running rclpy initialisation."""
    from web_ui.bridge import BridgeNode

    node = BridgeNode.__new__(BridgeNode)
    node._latest = {}
    node._dirty = set()
    node._cleared = set()
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
    node._trigger_clients = {}
    node._gps_anchor = None
    for key, value in attrs.items():
        setattr(node, key, value)
    return node


def decode_rgba(png_b64: str) -> np.ndarray:
    """Decode a base64 PNG into an RGBA array (height, width, 4)."""
    import base64

    raw = np.frombuffer(base64.b64decode(png_b64), dtype=np.uint8)
    bgra = cv2.imdecode(raw, cv2.IMREAD_UNCHANGED)
    assert bgra is not None and bgra.shape[2] == 4
    return cv2.cvtColor(bgra, cv2.COLOR_BGRA2RGBA)


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


# ---------------------------------------------------------------------------
# Config: new map_nav fields, removed tab types, default.yaml tab set
# ---------------------------------------------------------------------------


def test_map_nav_new_field_defaults() -> None:
    tab = TabConfig(id="m", type="map_nav", label="Map")
    assert tab.local_costmap_topic == "/local_costmap/costmap"
    assert tab.gps_fix_topic == "/client/gps/fix"
    assert "{z}" in tab.tile_url and "{x}" in tab.tile_url and "{y}" in tab.tile_url
    assert tab.tile_subdomains == "abcd"
    assert tab.tile_cache_dir == "/var/cache/web_ui/tiles"
    assert tab.tile_cache_max_mb > 0
    assert tab.arm_home_service == "/arm/home"
    assert tab.arm_set_home_service == "/arm/set_home"
    assert tab.base_urdf == "robot.urdf"
    assert tab.arm_urdf == "so101_arm.urdf"
    assert tab.base_joint_states_topic == "/swerve_drive/joint_states"
    assert tab.arm_joint_states_topic == "/follower/joint_states"
    assert tab.arm_command_topic == "/filter/web_ui_joint_commands"
    assert tab.gps_anchor_min_points >= 3
    assert tab.gps_anchor_min_spread_m > 0
    assert tab.gps_anchor_max_residual_m > 0


def test_map_nav_legacy_model_fields_removed() -> None:
    tab = TabConfig(id="m", type="map_nav", label="Map")
    for legacy in ("urdf_file", "arm_urdf_file", "arm_joint_topic"):
        assert not hasattr(tab, legacy)
    assert "urdf_file" not in tab.model_dump()


def test_map_nav_model_fields_serialized_for_frontend() -> None:
    dumped = TabConfig(id="m", type="map_nav", label="Map").model_dump()
    assert dumped["base_urdf"] == "robot.urdf"
    assert dumped["arm_urdf"] == "so101_arm.urdf"
    assert dumped["base_joint_states_topic"] == "/swerve_drive/joint_states"
    assert dumped["arm_joint_states_topic"] == "/follower/joint_states"


def test_map_nav_new_fields_not_defaulted_for_other_tabs() -> None:
    tab = TabConfig(id="c", type="camera", label="Cam")
    assert tab.base_urdf is None
    assert tab.arm_joint_states_topic is None
    assert tab.local_costmap_topic is None
    assert tab.gps_fix_topic is None
    assert tab.arm_home_service is None
    assert tab.arm_command_topic is None


def test_map_nav_overrides_new_fields() -> None:
    tab = TabConfig(id="m", type="map_nav", label="Map", local_costmap_topic="/lc", gps_fix_topic="/fix")
    assert tab.local_costmap_topic == "/lc"
    assert tab.gps_fix_topic == "/fix"


def test_topic_roles_include_costmap_and_gps() -> None:
    cfg = AppConfig(tabs=[TabConfig(id="m", type="map_nav", label="Map")])
    roles = cfg.topic_roles()
    assert roles["/local_costmap/costmap"] == "costmap"
    assert roles["/client/gps/fix"] == "gps"
    topics = cfg.all_subscribed_topics()
    assert "/local_costmap/costmap" in topics
    assert "/client/gps/fix" in topics
    assert "/follower/joint_states" in topics
    assert "/swerve_drive/joint_states" in topics


def test_trigger_services_listed() -> None:
    cfg = AppConfig(tabs=[TabConfig(id="m", type="map_nav", label="Map")])
    assert cfg.trigger_services() == ["/arm/home", "/arm/set_home"]


def test_gps_anchor_estimator_from_config() -> None:
    cfg = AppConfig(
        tabs=[
            TabConfig(
                id="m",
                type="map_nav",
                label="Map",
                gps_anchor_min_points=7,
                gps_anchor_min_spread_m=3.0,
                gps_anchor_max_residual_m=0.5,
            )
        ]
    )
    est = cfg.gps_anchor_estimator()
    assert est is not None
    assert (est.min_points, est.min_spread_m, est.max_residual_m) == (7, 3.0, 0.5)
    assert AppConfig(tabs=[]).gps_anchor_estimator() is None


@pytest.mark.parametrize("tab_type", REMOVED_TAB_TYPES)
def test_removed_tab_types_rejected(tab_type: str) -> None:
    with pytest.raises(ValidationError):
        TabConfig(id="x", type=tab_type, label="X")


@pytest.mark.parametrize("tab_type", ["camera", "sensor_graph", "imu_orientation", "rgbd_camera", "map_nav"])
def test_kept_tab_types_valid(tab_type: str) -> None:
    assert TabConfig(id="x", type=tab_type, label="X").type == tab_type


def test_default_yaml_tab_set_map_first() -> None:
    cfg = load_config(DEFAULT_YAML)
    types = [t.type for t in cfg.tabs]
    assert types[0] == "map_nav"
    assert types == ["map_nav", "camera", "camera", "imu_orientation"]
    for removed in REMOVED_TAB_TYPES:
        assert removed not in types
    tab = cfg.tabs[0]
    assert tab.local_costmap_topic == "/local_costmap/costmap"
    assert tab.gps_fix_topic == "/client/gps/fix"
    assert tab.arm_command_topic == "/filter/web_ui_joint_commands"
    assert tab.base_urdf == "robot.urdf"
    assert tab.arm_urdf == "so101_arm.urdf"
    assert tab.base_joint_states_topic == "/swerve_drive/joint_states"
    assert tab.arm_joint_states_topic == "/follower/joint_states"
    assert tab.arm_home_service == "/arm/home"
    assert tab.arm_set_home_service == "/arm/set_home"
    assert tab.topic is None


def test_default_yaml_has_no_legacy_map_nav_keys() -> None:
    import yaml

    raw = yaml.safe_load(DEFAULT_YAML.read_text())
    map_tab = raw["tabs"][0]
    for legacy in ("urdf_file", "arm_urdf_file", "arm_joint_topic", "topic"):
        assert legacy not in map_tab
    assert "/filter/arm_home" not in DEFAULT_YAML.read_text()


def test_default_yaml_topics_all_have_known_types() -> None:
    from web_ui.bridge import subscription_spec

    cfg = load_config(DEFAULT_YAML)
    roles = cfg.topic_roles()
    unknown = [t for t in cfg.all_subscribed_topics() if subscription_spec(t, roles.get(t)) is None]
    assert unknown == []


# ---------------------------------------------------------------------------
# Costmap serializer + bridge role
# ---------------------------------------------------------------------------


def test_costmap_png_transparency_gradient_lethal_and_orientation() -> None:
    from web_ui.msg_serializer import COSTMAP_INSCRIBED_RGBA, COSTMAP_LETHAL_RGBA, serialize_costmap

    # grid row 0 (bottom): -1, 0, 50, 100 ; grid row 1 (top): 1, 98, 99, 100
    data = serialize_costmap(make_costmap(4, 2, [-1, 0, 50, 100, 1, 98, 99, 100]))
    img = decode_rgba(data["png_b64"])
    assert img.shape == (2, 4, 4)
    top, bottom = img[0], img[1]  # image row 0 is the top = grid row 1
    assert bottom[0][3] == 0  # unknown transparent
    assert bottom[1][3] == 0  # free transparent
    assert tuple(bottom[3]) == COSTMAP_LETHAL_RGBA
    assert tuple(top[3]) == COSTMAP_LETHAL_RGBA
    assert tuple(top[2]) == COSTMAP_INSCRIBED_RGBA
    low, mid, high = top[0], bottom[2], top[1]
    assert 0 < low[3] <= mid[3] <= high[3]
    assert low[0] < mid[0] < high[0]  # red rises with cost
    assert tuple(high) != COSTMAP_LETHAL_RGBA and tuple(high) != COSTMAP_INSCRIBED_RGBA


def test_costmap_metadata() -> None:
    from web_ui.msg_serializer import serialize_costmap

    data = serialize_costmap(make_costmap(2, 1, [0, 100], x=1.5, y=-2.0))
    assert data["width"] == 2 and data["height"] == 1
    assert data["resolution"] == pytest.approx(0.05)
    assert data["origin"] == {"x": 1.5, "y": -2.0, "yaw": 0.0}
    assert data["frame_id"] == "odom"
    assert data["stamp"] == pytest.approx(7.0)


def test_costmap_size_mismatch_raises() -> None:
    from web_ui.msg_serializer import serialize_costmap

    with pytest.raises(ValueError):
        serialize_costmap(make_costmap(3, 3, [0, 1]))


def test_subscription_spec_costmap_reliable_transient_local() -> None:
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

    from web_ui.bridge import OccupancyGrid, subscription_spec
    from web_ui.msg_serializer import serialize_costmap

    msg_cls, qos, serializer = subscription_spec("/local_costmap/costmap", "costmap")
    assert msg_cls is OccupancyGrid
    assert qos.reliability is ReliabilityPolicy.RELIABLE
    assert qos.durability is DurabilityPolicy.TRANSIENT_LOCAL
    assert serializer is serialize_costmap


def test_costmap_origin_transformed_into_map_frame() -> None:
    from web_ui.msg_serializer import serialize_costmap

    tf_buffer = MagicMock()
    tf_buffer.lookup_transform.return_value = make_transform(10.0, 0.0, math.pi / 2, frame_id="map")
    node = make_bridge(_tf_buffer=tf_buffer, _robot_pose_frames=("map", "base_link"))
    cb = node._make_callback("/local_costmap/costmap", serialize_costmap, "costmap")
    cb(make_costmap(2, 1, [0, 100], frame_id="odom", x=1.0, y=0.0))
    data = node.flush_dirty()[0]["data"]
    assert data["frame_id"] == "map"
    assert data["origin"]["x"] == pytest.approx(10.0)
    assert data["origin"]["y"] == pytest.approx(1.0)
    assert data["origin"]["yaw"] == pytest.approx(math.pi / 2)
    assert tf_buffer.lookup_transform.call_args[0][:2] == ("map", "odom")


def test_costmap_dropped_without_tf() -> None:
    from tf2_ros import TransformException

    from web_ui.msg_serializer import serialize_costmap

    tf_buffer = MagicMock()
    tf_buffer.lookup_transform.side_effect = TransformException("no tf")
    node = make_bridge(_tf_buffer=tf_buffer, _robot_pose_frames=("map", "base_link"))
    cb = node._make_callback("/local_costmap/costmap", serialize_costmap, "costmap")
    cb(make_costmap(2, 1, [0, 100]))
    assert node.flush_dirty() == []
    assert node.latest_envelopes() == []


def test_costmap_serialized_once_per_message_not_per_broadcast() -> None:
    from web_ui import msg_serializer

    node = make_bridge()
    with patch.object(msg_serializer, "serialize_costmap", wraps=msg_serializer.serialize_costmap) as spy:
        cb = node._make_callback("/local_costmap/costmap", spy, "costmap")
        cb(make_costmap(2, 1, [0, 100], frame_id="map"))
        for _ in range(5):
            node.flush_dirty()
            node.latest_envelopes()
    assert spy.call_count == 1


# ---------------------------------------------------------------------------
# GPS fix role
# ---------------------------------------------------------------------------


def test_subscription_spec_gps_navsatfix() -> None:
    from web_ui.bridge import NavSatFix, subscription_spec
    from web_ui.msg_serializer import serialize_navsatfix

    msg_cls, _, serializer = subscription_spec("/client/gps/fix", "gps")
    assert msg_cls is NavSatFix
    assert serializer is serialize_navsatfix


def test_serialize_navsatfix_payload() -> None:
    from web_ui.msg_serializer import serialize_navsatfix

    data = serialize_navsatfix(make_fix(ANCHOR_LAT, ANCHOR_LON, status=2, sec=5, nanosec=500_000_000))
    assert data == {
        "latitude": ANCHOR_LAT,
        "longitude": ANCHOR_LON,
        "altitude": 110.0,
        "status": 2,
        "horizontal_accuracy_m": pytest.approx(0.1),
        "frame_id": "gps_link",
        "stamp": pytest.approx(5.5),
    }


def test_gps_fix_stored_for_ws() -> None:
    from web_ui.msg_serializer import serialize_navsatfix

    node = make_bridge()
    node._make_callback("/client/gps/fix", serialize_navsatfix, "gps")(make_fix(ANCHOR_LAT, ANCHOR_LON))
    env = node.flush_dirty()
    assert env[0]["topic"] == "/client/gps/fix"
    assert env[0]["data"]["latitude"] == ANCHOR_LAT


def test_gps_no_fix_ignored() -> None:
    from web_ui.gps_anchor import GpsAnchorEstimator
    from web_ui.msg_serializer import serialize_navsatfix

    est = MagicMock(spec=GpsAnchorEstimator)
    node = make_bridge(_gps_anchor=est)
    node._make_callback("/client/gps/fix", serialize_navsatfix, "gps")(make_fix(0.0, 0.0, status=-1))
    assert node.flush_dirty() == []
    est.add_sample.assert_not_called()


# ---------------------------------------------------------------------------
# GPS anchor fit (pure numpy)
# ---------------------------------------------------------------------------


def l_track(n: int = 40, length_m: float = 20.0) -> np.ndarray:
    """Return an L-shaped map-frame track (n points)."""
    half = n // 2
    leg1 = np.column_stack([np.linspace(0.0, length_m, half), np.zeros(half)])
    leg2 = np.column_stack([np.full(n - half, length_m), np.linspace(0.0, length_m / 2, n - half + 1)[1:]])
    return np.vstack([leg1, leg2]) + np.array([3.0, -2.0])


def track_fixes(track: np.ndarray, heading: float, noise_m: float = 0.0, seed: int = 0) -> list[tuple[float, float]]:
    """Map-frame track -> (lat, lon) fixes with the map origin at ANCHOR_LAT/LON and map +x at heading from East."""
    from web_ui.gps_anchor import enu_to_latlon

    rng = np.random.default_rng(seed)
    c, s = math.cos(heading), math.sin(heading)
    rot = np.array([[c, -s], [s, c]])
    enu = track @ rot.T + rng.normal(0.0, noise_m, track.shape) if noise_m else track @ rot.T
    return [enu_to_latlon(e, n, ANCHOR_LAT, ANCHOR_LON) for e, n in enu]


def feed(est: Any, track: np.ndarray, fixes: list[tuple[float, float]]) -> Any:
    result = None
    for (x, y), (lat, lon) in zip(track, fixes, strict=True):
        result = est.add_sample(lat, lon, float(x), float(y))
    return result


def test_latlon_enu_roundtrip() -> None:
    from web_ui.gps_anchor import enu_to_latlon, latlon_to_enu

    e, n = latlon_to_enu(ANCHOR_LAT + 0.0001, ANCHOR_LON + 0.0002, ANCHOR_LAT, ANCHOR_LON)
    assert n == pytest.approx(11.13, abs=0.05)
    assert e == pytest.approx(13.6, abs=0.1)
    lat, lon = enu_to_latlon(e, n, ANCHOR_LAT, ANCHOR_LON)
    assert lat == pytest.approx(ANCHOR_LAT + 0.0001, abs=1e-9)
    assert lon == pytest.approx(ANCHOR_LON + 0.0002, abs=1e-9)


def test_fit_rigid_2d_recovers_rotation_translation() -> None:
    from web_ui.gps_anchor import fit_rigid_2d

    src = l_track()
    theta = -1.2
    rot = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
    dst = src @ rot.T + np.array([5.0, -7.0])
    fit_theta, t, residual = fit_rigid_2d(src, dst)
    assert fit_theta == pytest.approx(theta, abs=1e-9)
    assert t == pytest.approx([5.0, -7.0], abs=1e-9)
    assert residual == pytest.approx(0.0, abs=1e-9)


def test_anchor_recovers_heading_and_origin() -> None:
    from web_ui.gps_anchor import GpsAnchorEstimator

    track = l_track()
    est = GpsAnchorEstimator(min_points=10, min_spread_m=5.0, max_residual_m=1.0)
    anchor = feed(est, track, track_fixes(track, heading=0.7))
    assert anchor is not None
    assert anchor["heading_rad"] == pytest.approx(0.7, abs=1e-6)
    assert anchor["lat"] == pytest.approx(ANCHOR_LAT, abs=1e-7)
    assert anchor["lon"] == pytest.approx(ANCHOR_LON, abs=1e-7)
    assert anchor["residual_m"] < 0.01
    assert anchor["n_points"] == len(track)


def test_anchor_tolerates_small_noise() -> None:
    from web_ui.gps_anchor import GpsAnchorEstimator

    track = l_track(n=80)
    est = GpsAnchorEstimator(min_points=10, min_spread_m=5.0, max_residual_m=0.5)
    anchor = feed(est, track, track_fixes(track, heading=-2.0, noise_m=0.05))
    assert anchor is not None
    assert anchor["heading_rad"] == pytest.approx(-2.0, abs=0.02)


def test_anchor_rejected_too_few_points() -> None:
    from web_ui.gps_anchor import GpsAnchorEstimator

    track = l_track()[:5]
    est = GpsAnchorEstimator(min_points=10, min_spread_m=1.0, max_residual_m=1.0, min_sample_spacing_m=0.0)
    assert feed(est, track, track_fixes(track, heading=0.3)) is None


def test_anchor_rejected_track_too_short() -> None:
    from web_ui.gps_anchor import GpsAnchorEstimator

    track = l_track(n=40, length_m=2.0)
    est = GpsAnchorEstimator(min_points=10, min_spread_m=10.0, max_residual_m=1.0, min_sample_spacing_m=0.0)
    assert feed(est, track, track_fixes(track, heading=0.3)) is None


def test_anchor_rejected_when_noisy() -> None:
    from web_ui.gps_anchor import GpsAnchorEstimator

    track = l_track(n=60)
    est = GpsAnchorEstimator(min_points=10, min_spread_m=5.0, max_residual_m=0.5, min_sample_spacing_m=0.0)
    assert feed(est, track, track_fixes(track, heading=0.3, noise_m=3.0)) is None


def test_anchor_refits_as_data_accumulates() -> None:
    from web_ui.gps_anchor import GpsAnchorEstimator

    track = l_track(n=40)
    fixes = track_fixes(track, heading=1.0)
    est = GpsAnchorEstimator(min_points=10, min_spread_m=5.0, max_residual_m=1.0)
    first = feed(est, track[:25], fixes[:25])
    second = feed(est, track[25:], fixes[25:])
    assert first is not None and second is not None
    assert second["n_points"] > first["n_points"]


def test_anchor_skips_samples_closer_than_spacing() -> None:
    from web_ui.gps_anchor import GpsAnchorEstimator

    est = GpsAnchorEstimator(min_points=3, min_spread_m=0.0, max_residual_m=1.0, min_sample_spacing_m=0.5)
    for _ in range(10):
        est.add_sample(ANCHOR_LAT, ANCHOR_LON, 0.0, 0.0)
    assert est.n_points == 1


def test_anchor_reset_clears_samples() -> None:
    from web_ui.gps_anchor import GpsAnchorEstimator

    track = l_track()
    est = GpsAnchorEstimator(min_points=10, min_spread_m=5.0, max_residual_m=1.0)
    assert feed(est, track, track_fixes(track, heading=0.7)) is not None
    est.reset()
    assert est.n_points == 0
    assert est.add_sample(ANCHOR_LAT, ANCHOR_LON, 0.0, 0.0) is None


def test_anchor_sample_buffer_bounded() -> None:
    from web_ui.gps_anchor import GpsAnchorEstimator

    track = l_track(n=50)
    est = GpsAnchorEstimator(min_points=10, min_spread_m=5.0, max_residual_m=1.0, max_samples=20)
    anchor = feed(est, track, track_fixes(track, heading=0.4))
    assert est.n_points == 20
    assert anchor is not None and anchor["n_points"] == 20


# ---------------------------------------------------------------------------
# GPS anchor in the bridge
# ---------------------------------------------------------------------------


def test_bridge_publishes_gps_anchor_from_fixes_and_tf() -> None:
    from web_ui.bridge import GPS_ANCHOR_TOPIC
    from web_ui.gps_anchor import GpsAnchorEstimator
    from web_ui.msg_serializer import serialize_navsatfix

    track = l_track(n=12)
    fixes = track_fixes(track, heading=0.5)
    tf_buffer = MagicMock()
    est = GpsAnchorEstimator(min_points=10, min_spread_m=5.0, max_residual_m=1.0)
    node = make_bridge(_tf_buffer=tf_buffer, _robot_pose_frames=("map", "base_link"), _gps_anchor=est)
    cb = node._make_callback("/client/gps/fix", serialize_navsatfix, "gps")
    for (x, y), (lat, lon) in zip(track, fixes, strict=True):
        tf_buffer.lookup_transform.return_value = make_transform(float(x), float(y), 0.0, sec=100)
        cb(make_fix(lat, lon, sec=100))
    anchors = [e for e in node.flush_dirty() if e["topic"] == GPS_ANCHOR_TOPIC]
    assert len(anchors) == 1
    data = anchors[0]["data"]
    assert data["heading_rad"] == pytest.approx(0.5, abs=1e-6)
    assert data["lat"] == pytest.approx(ANCHOR_LAT, abs=1e-7)
    assert set(data) == {"lat", "lon", "heading_rad", "residual_m", "n_points"}


def test_bridge_gps_anchor_skips_fix_when_tf_skewed_or_missing() -> None:
    from tf2_ros import TransformException

    from web_ui.gps_anchor import GpsAnchorEstimator
    from web_ui.msg_serializer import serialize_navsatfix

    est = MagicMock(spec=GpsAnchorEstimator)
    tf_buffer = MagicMock()
    tf_buffer.lookup_transform.return_value = make_transform(1.0, 1.0, 0.0, sec=90)
    node = make_bridge(_tf_buffer=tf_buffer, _robot_pose_frames=("map", "base_link"), _gps_anchor=est)
    cb = node._make_callback("/client/gps/fix", serialize_navsatfix, "gps")
    cb(make_fix(ANCHOR_LAT, ANCHOR_LON, sec=100))
    tf_buffer.lookup_transform.side_effect = TransformException("none")
    cb(make_fix(ANCHOR_LAT, ANCHOR_LON, sec=100))
    est.add_sample.assert_not_called()


def test_bridge_gps_anchor_cleared_when_fit_degrades() -> None:
    from web_ui.bridge import GPS_ANCHOR_TOPIC

    est = MagicMock()
    est.add_sample.side_effect = [{"lat": 1.0, "lon": 2.0, "heading_rad": 0.0, "residual_m": 0.1, "n_points": 10}, None]
    tf_buffer = MagicMock()
    tf_buffer.lookup_transform.return_value = make_transform(0.0, 0.0, 0.0, sec=100)
    node = make_bridge(_tf_buffer=tf_buffer, _robot_pose_frames=("map", "base_link"), _gps_anchor=est)
    node.feed_gps_anchor({"latitude": 1.0, "longitude": 2.0, "stamp": 100.0})
    assert any(e["topic"] == GPS_ANCHOR_TOPIC and e["data"] for e in node.flush_dirty())
    node.feed_gps_anchor({"latitude": 1.0, "longitude": 2.0, "stamp": 100.0})
    assert {"topic": GPS_ANCHOR_TOPIC, "data": None} in node.flush_dirty()


def test_bridge_reset_gps_anchor() -> None:
    from web_ui.bridge import GPS_ANCHOR_TOPIC

    est = MagicMock()
    node = make_bridge(_gps_anchor=est)
    node.store(GPS_ANCHOR_TOPIC, {"lat": 1.0})
    node.flush_dirty()
    node.reset_gps_anchor()
    est.reset.assert_called_once()
    assert node.flush_dirty() == [{"topic": GPS_ANCHOR_TOPIC, "data": None}]
    assert node.latest_envelopes() == []


def test_map_reset_endpoint_resets_gps_anchor(tmp_path: Path, urdf_dir: Path) -> None:
    from web_ui.server import build_app

    config = AppConfig(tabs=[TabConfig(id="map", type="map_nav", label="Map")])
    bridge = MagicMock()
    bridge.reset_map_async.return_value = FakeFuture(SimpleNamespace(result=0))
    client = TestClient(build_app(config=config, urdf_dir=urdf_dir, static_dir=tmp_path / "x", bridge_node=bridge))
    assert client.post("/api/map/reset?tab=map").status_code == 200
    bridge.reset_gps_anchor.assert_called_once()


def test_bridge_init_wires_costmap_gps_and_trigger_clients() -> None:
    from web_ui.bridge import BridgeNode, NavSatFix, OccupancyGrid, Trigger

    created: list[tuple[Any, str]] = []
    clients: list[tuple[Any, str]] = []
    with (
        patch("web_ui.bridge.Node.__init__", return_value=None),
        patch.object(
            BridgeNode, "create_subscription", create=True, side_effect=lambda c, t, cb, q: created.append((c, t))
        ),
        patch.object(BridgeNode, "create_timer", create=True),
        patch.object(BridgeNode, "get_logger", create=True),
        patch.object(BridgeNode, "create_client", create=True, side_effect=lambda c, s: clients.append((c, s))),
        patch("web_ui.bridge.Buffer"),
        patch("web_ui.bridge.TransformListener"),
    ):
        node = BridgeNode(
            topics=["/local_costmap/costmap", "/client/gps/fix"],
            allowed_publish_topics=set(),
            topic_roles={"/local_costmap/costmap": "costmap", "/client/gps/fix": "gps"},
            robot_pose_frames=("map", "base_link"),
            trigger_services=["/arm/home", "/arm/set_home"],
            gps_anchor=MagicMock(),
        )
    assert (OccupancyGrid, "/local_costmap/costmap") in created
    assert (NavSatFix, "/client/gps/fix") in created
    assert (Trigger, "/arm/home") in clients
    assert (Trigger, "/arm/set_home") in clients
    assert node._gps_anchor is not None


# ---------------------------------------------------------------------------
# Arm home endpoints
# ---------------------------------------------------------------------------


def ready_client(ready: bool = True) -> MagicMock:
    client = MagicMock()
    client.service_is_ready.return_value = ready
    return client


def test_trigger_async_unavailable_returns_none() -> None:
    node = make_bridge(_trigger_clients={"/arm/home": ready_client(False)})
    assert node.trigger_async("/arm/home") is None
    assert node.trigger_async("/unknown") is None


def test_trigger_async_calls_service() -> None:
    from web_ui.bridge import Trigger

    client = ready_client()
    node = make_bridge(_trigger_clients={"/arm/home": client})
    assert node.trigger_async("/arm/home") is client.call_async.return_value
    assert isinstance(client.call_async.call_args[0][0], Trigger.Request)


@pytest.fixture
def arm_app(tmp_path: Path, urdf_dir: Path) -> Any:
    """Return an app factory with a map_nav tab using custom arm services."""
    from web_ui.server import build_app

    def factory(bridge: Any, **tab_kwargs: Any) -> TestClient:
        config = AppConfig(
            tabs=[
                TabConfig(
                    id="map",
                    type="map_nav",
                    label="Map",
                    arm_home_service="/arm/home",
                    arm_set_home_service="/arm/set",
                    **tab_kwargs,
                )
            ]
        )
        return TestClient(build_app(config=config, urdf_dir=urdf_dir, static_dir=tmp_path / "none", bridge_node=bridge))

    return factory


@pytest.mark.parametrize(("endpoint", "service"), [("home", "/arm/home"), ("set_home", "/arm/set")])
def test_arm_endpoint_success(arm_app: Any, endpoint: str, service: str) -> None:
    bridge = MagicMock()
    bridge.trigger_async.return_value = FakeFuture(SimpleNamespace(success=True, message="done"))
    resp = arm_app(bridge).post(f"/api/arm/{endpoint}?tab=map")
    assert resp.status_code == 200
    assert resp.json() == {"ok": True, "message": "done"}
    bridge.trigger_async.assert_called_once_with(service)


def test_arm_endpoint_service_reports_failure(arm_app: Any) -> None:
    bridge = MagicMock()
    bridge.trigger_async.return_value = FakeFuture(SimpleNamespace(success=False, message="no home pose saved"))
    resp = arm_app(bridge).post("/api/arm/home?tab=map")
    assert resp.status_code == 500
    assert resp.json() == {"ok": False, "message": "no home pose saved"}


def test_arm_endpoint_unavailable(arm_app: Any) -> None:
    bridge = MagicMock()
    bridge.trigger_async.return_value = None
    resp = arm_app(bridge).post("/api/arm/set_home?tab=map")
    assert resp.status_code == 503
    assert resp.json()["ok"] is False
    assert "unavailable" in resp.json()["message"]


def test_arm_endpoint_timeout(arm_app: Any) -> None:
    bridge = MagicMock()
    fut = FakeFuture(resolve=False)
    bridge.trigger_async.return_value = fut
    resp = arm_app(bridge, arm_service_timeout_s=0.05).post("/api/arm/home?tab=map")
    assert resp.status_code == 504
    assert "timed out" in resp.json()["message"]
    assert fut.cancelled


@pytest.mark.parametrize("endpoint", ["home", "set_home"])
def test_arm_endpoint_unknown_tab_and_no_bridge(arm_app: Any, endpoint: str) -> None:
    resp = arm_app(MagicMock()).post(f"/api/arm/{endpoint}?tab=nope")
    assert resp.status_code == 404
    assert resp.json()["ok"] is False
    resp = arm_app(None).post(f"/api/arm/{endpoint}?tab=map")
    assert resp.status_code == 503


# ---------------------------------------------------------------------------
# URDF cache headers + CSP
# ---------------------------------------------------------------------------


@pytest.fixture
def plain_app(tmp_path: Path, urdf_dir: Path) -> TestClient:
    from web_ui.server import build_app

    return TestClient(build_app(config=AppConfig(), urdf_dir=urdf_dir, static_dir=tmp_path / "none"))


def test_urdf_files_have_cache_control(plain_app: TestClient, urdf_dir: Path) -> None:
    (urdf_dir / "meshes").mkdir()
    (urdf_dir / "meshes" / "wheel.stl").write_bytes(b"solid x")
    for path in ("robot.urdf", "meshes/wheel.stl"):
        resp = plain_app.get(f"/api/urdf/{path}")
        assert resp.status_code == 200
        assert "max-age=86400" in resp.headers["cache-control"]


def test_urdf_status_not_cached(plain_app: TestClient) -> None:
    assert "cache-control" not in plain_app.get("/api/urdf/status").headers


def test_csp_img_src_self_only_for_tiles(plain_app: TestClient) -> None:
    csp = plain_app.get("/api/config").headers["content-security-policy"]
    img_src = next(d for d in csp.split(";") if d.strip().startswith("img-src"))
    assert "'self'" in img_src
    assert "cartocdn" not in csp
    assert "openstreetmap" not in csp


# ---------------------------------------------------------------------------
# Tile proxy
# ---------------------------------------------------------------------------


def test_validate_tile() -> None:
    from web_ui.tiles import validate_tile

    assert validate_tile(0, 0, 0)
    assert validate_tile(18, 2**18 - 1, 5)
    assert not validate_tile(18, 2**18, 5)
    assert not validate_tile(3, 1, 8)
    assert not validate_tile(-1, 0, 0)
    assert not validate_tile(23, 0, 0)
    assert not validate_tile(3, -1, 0)


def test_build_tile_url_subdomain_rotation_and_retina_placeholder() -> None:
    from web_ui.tiles import build_tile_url

    template = "https://{s}.example.com/{z}/{x}/{y}{r}.png"
    urls = {build_tile_url(template, "abcd", 5, x, 3) for x in range(4)}
    assert urls == {f"https://{s}.example.com/5/{x}/3.png" for x, s in zip(range(4), "dabc", strict=True)}
    assert build_tile_url("https://t.example.com/{z}/{x}/{y}.png", "", 1, 0, 1) == "https://t.example.com/1/0/1.png"


def test_tile_cache_put_get_and_eviction(tmp_path: Path) -> None:
    import os

    from web_ui.tiles import TileCache

    cache = TileCache(tmp_path / "tiles", max_bytes=25)
    cache.put(1, 0, 0, b"a" * 10)
    os.utime(cache.path(1, 0, 0), (1, 1))
    cache.put(1, 0, 1, b"b" * 10)
    assert cache.get(1, 0, 0) == b"a" * 10
    cache.put(1, 1, 0, b"c" * 10)
    assert cache.get(1, 0, 0) is None  # oldest evicted
    assert cache.get(1, 0, 1) == b"b" * 10
    assert cache.get(1, 1, 0) == b"c" * 10
    assert cache.total_bytes() <= 25


def test_tile_cache_unwritable_dir_does_not_raise(tmp_path: Path) -> None:
    from web_ui.tiles import TileCache

    blocker = tmp_path / "file"
    blocker.write_text("x")
    cache = TileCache(blocker / "tiles", max_bytes=100)
    cache.put(1, 0, 0, b"x")
    assert cache.get(1, 0, 0) is None


class Upstream:
    """httpx MockTransport handler recording requests; behaviour selectable per test."""

    def __init__(self, mode: str = "ok") -> None:
        self.mode = mode
        self.requests: list[httpx.Request] = []

    def __call__(self, request: httpx.Request) -> httpx.Response:
        self.requests.append(request)
        if self.mode == "timeout":
            raise httpx.ReadTimeout("slow", request=request)
        if self.mode == "offline":
            raise httpx.ConnectError("no route", request=request)
        if self.mode == "missing":
            return httpx.Response(404, request=request)
        if self.mode == "html":
            return httpx.Response(200, content=b"<html/>", headers={"content-type": "text/html"}, request=request)
        return httpx.Response(200, content=PNG_BYTES, headers={"content-type": "image/png"}, request=request)


def tile_client(tmp_path: Path, urdf_dir: Path, upstream: Upstream, tabs: list[TabConfig] | None = None) -> TestClient:
    from web_ui.server import build_app

    if tabs is None:
        tabs = [
            TabConfig(
                id="map",
                type="map_nav",
                label="Map",
                tile_url="https://{s}.tiles.test/{z}/{x}/{y}.png",
                tile_subdomains="ab",
                tile_cache_dir=str(tmp_path / "tilecache"),
            )
        ]
    config = AppConfig(tabs=tabs)
    app = build_app(
        config=config,
        urdf_dir=urdf_dir,
        static_dir=tmp_path / "none",
        tile_transport=httpx.MockTransport(upstream),
    )
    return TestClient(app)


def test_tile_cache_miss_fetches_then_hit_serves_from_disk(tmp_path: Path, urdf_dir: Path) -> None:
    upstream = Upstream()
    client = tile_client(tmp_path, urdf_dir, upstream)
    resp = client.get("/api/tiles/3/5/2.png")
    assert resp.status_code == 200
    assert resp.content == PNG_BYTES
    assert resp.headers["content-type"] == "image/png"
    assert "max-age" in resp.headers["cache-control"]
    assert str(upstream.requests[0].url) == "https://b.tiles.test/3/5/2.png"
    assert "web_ui" in upstream.requests[0].headers["user-agent"]
    assert (tmp_path / "tilecache" / "3" / "5" / "2.png").read_bytes() == PNG_BYTES
    resp = client.get("/api/tiles/3/5/2.png")
    assert resp.status_code == 200
    assert len(upstream.requests) == 1


@pytest.mark.parametrize("path", ["/api/tiles/25/0/0.png", "/api/tiles/3/8/0.png", "/api/tiles/3/0/8.png"])
def test_tile_out_of_range_rejected(tmp_path: Path, urdf_dir: Path, path: str) -> None:
    upstream = Upstream()
    resp = tile_client(tmp_path, urdf_dir, upstream).get(path)
    assert resp.status_code == 400
    assert upstream.requests == []


def test_tile_non_integer_rejected(tmp_path: Path, urdf_dir: Path) -> None:
    upstream = Upstream()
    resp = tile_client(tmp_path, urdf_dir, upstream).get("/api/tiles/3/a/0.png")
    assert resp.status_code in (404, 422)
    assert upstream.requests == []


@pytest.mark.parametrize(("mode", "status"), [("missing", 404), ("html", 404), ("timeout", 504), ("offline", 504)])
def test_tile_upstream_failure(tmp_path: Path, urdf_dir: Path, mode: str, status: int) -> None:
    resp = tile_client(tmp_path, urdf_dir, Upstream(mode)).get("/api/tiles/2/1/1.png")
    assert resp.status_code == status
    assert not (tmp_path / "tilecache" / "2" / "1" / "1.png").exists()


def test_tile_served_from_cache_when_offline(tmp_path: Path, urdf_dir: Path) -> None:
    assert tile_client(tmp_path, urdf_dir, Upstream()).get("/api/tiles/4/3/2.png").status_code == 200
    offline = Upstream("offline")
    resp = tile_client(tmp_path, urdf_dir, offline).get("/api/tiles/4/3/2.png")
    assert resp.status_code == 200
    assert resp.content == PNG_BYTES
    assert offline.requests == []


def test_tile_without_map_nav_tab_is_404(tmp_path: Path, urdf_dir: Path) -> None:
    upstream = Upstream()
    resp = tile_client(tmp_path, urdf_dir, upstream, tabs=[]).get("/api/tiles/1/0/0.png")
    assert resp.status_code == 404
    assert upstream.requests == []


# ---------------------------------------------------------------------------
# F3: arm timeout config, GPS anchor thread safety/caching, tile eviction low-water mark
# ---------------------------------------------------------------------------


def test_arm_service_timeout_default_is_30s() -> None:
    assert TabConfig(id="map", type="map_nav", label="Map").arm_service_timeout_s == 30.0


@pytest.mark.parametrize("endpoint", ["home", "set_home"])
@pytest.mark.parametrize(("tab_kwargs", "expected"), [({}, 30.0), ({"arm_service_timeout_s": 45.5}, 45.5)])
def test_arm_endpoints_pass_timeout_to_service_call(
    arm_app: Any, monkeypatch: pytest.MonkeyPatch, endpoint: str, tab_kwargs: dict[str, Any], expected: float
) -> None:
    import web_ui.server as server

    seen: list[float] = []

    async def fake_call(action: str, future: Any, service: str, timeout_s: float) -> tuple[Any, None]:
        seen.append(timeout_s)
        return SimpleNamespace(success=True, message="ok"), None

    monkeypatch.setattr(server, "call_ros_service", fake_call)
    bridge = MagicMock()
    assert arm_app(bridge, **tab_kwargs).post(f"/api/arm/{endpoint}?tab=map").status_code == 200
    assert seen == [expected]


@pytest.mark.parametrize("endpoint", ["home", "set_home"])
def test_arm_endpoints_time_out_at_configured_value(arm_app: Any, endpoint: str) -> None:
    import time

    bridge = MagicMock()
    bridge.trigger_async.return_value = FakeFuture(resolve=False)
    start = time.monotonic()
    resp = arm_app(bridge, arm_service_timeout_s=0.2).post(f"/api/arm/{endpoint}?tab=map")
    assert time.monotonic() - start < 5.0
    assert resp.status_code == 504
    assert "timed out after 0.2 s" in resp.json()["message"]


def test_anchor_concurrent_reset_and_add_sample_do_not_raise() -> None:
    import threading

    from web_ui.gps_anchor import GpsAnchorEstimator

    est = GpsAnchorEstimator(min_points=3, min_spread_m=0.0, max_residual_m=1e9, min_sample_spacing_m=0.0)
    errors: list[BaseException] = []
    stop = threading.Event()

    def writer() -> None:
        i = 0
        try:
            while not stop.is_set():
                i += 1
                est.add_sample(ANCHOR_LAT + i * 1e-7, ANCHOR_LON, float(i % 50), float(i % 7))
        except BaseException as exc:  # noqa: BLE001
            errors.append(exc)

    def resetter() -> None:
        try:
            for _ in range(3000):
                est.reset()
                est.fit()
        except BaseException as exc:  # noqa: BLE001
            errors.append(exc)

    threads = [threading.Thread(target=writer) for _ in range(2)]
    for t in threads:
        t.start()
    r = threading.Thread(target=resetter)
    r.start()
    r.join()
    stop.set()
    for t in threads:
        t.join()
    assert errors == []


def test_anchor_rejected_sample_returns_cached_result_without_refit() -> None:
    from unittest.mock import patch as _patch

    from web_ui.gps_anchor import GpsAnchorEstimator

    track = l_track()
    est = GpsAnchorEstimator(min_points=10, min_spread_m=5.0, max_residual_m=1.0, min_sample_spacing_m=0.5)
    first = feed(est, track, track_fixes(track, heading=0.7))
    assert first is not None
    last_x, last_y = track[-1]
    with _patch("web_ui.gps_anchor.fit_rigid_2d") as fit_mock:
        again = est.add_sample(ANCHOR_LAT, ANCHOR_LON, float(last_x), float(last_y))
    fit_mock.assert_not_called()
    assert again == first


def test_bridge_gps_anchor_not_rebroadcast_when_unchanged() -> None:
    from web_ui.bridge import GPS_ANCHOR_TOPIC

    anchor = {"lat": 1.0, "lon": 2.0, "heading_rad": 0.0, "residual_m": 0.1, "n_points": 10}
    est = MagicMock()
    est.add_sample.return_value = dict(anchor)
    tf_buffer = MagicMock()
    tf_buffer.lookup_transform.return_value = make_transform(0.0, 0.0, 0.0, sec=100)
    node = make_bridge(_tf_buffer=tf_buffer, _robot_pose_frames=("map", "base_link"), _gps_anchor=est)
    node.feed_gps_anchor({"latitude": 1.0, "longitude": 2.0, "stamp": 100.0})
    assert any(e["topic"] == GPS_ANCHOR_TOPIC for e in node.flush_dirty())
    node.feed_gps_anchor({"latitude": 1.0, "longitude": 2.0, "stamp": 100.0})
    assert not any(e["topic"] == GPS_ANCHOR_TOPIC for e in node.flush_dirty())


def test_tile_cache_evicts_to_low_water_mark(tmp_path: Path) -> None:
    from web_ui.tiles import TileCache

    cache = TileCache(tmp_path / "tiles", max_bytes=100)
    for i in range(11):
        cache.put(1, 0, i, b"x" * 10)
    # 110 > 100 triggers eviction down to 90% of the cap (90), not to exactly 100
    assert cache.total_bytes() <= 90


def test_tile_cache_does_not_rescan_on_every_write_under_the_mark(tmp_path: Path) -> None:
    from unittest.mock import patch as _patch

    from web_ui.tiles import TileCache

    cache = TileCache(tmp_path / "tiles", max_bytes=100)
    for i in range(11):
        cache.put(1, 0, i, b"x" * 10)
    with _patch.object(TileCache, "files", wraps=cache.files) as files_mock:
        for i in range(11, 12):  # 90 -> 100 bytes, not over the cap
            cache.put(1, 0, i, b"x" * 10)
    files_mock.assert_not_called()


@pytest.mark.asyncio
async def test_tile_proxy_writes_cache_off_the_event_loop(tmp_path: Path) -> None:
    import threading

    from web_ui.tiles import TileCache, TileProxy

    cache = TileCache(tmp_path / "tiles", max_bytes=1000)
    put_threads: list[int] = []
    real_put = cache.put

    def spy(*args: Any) -> None:
        put_threads.append(threading.get_ident())
        real_put(*args)

    cache.put = spy  # type: ignore[method-assign]
    transport = httpx.MockTransport(
        lambda _: httpx.Response(200, content=b"png", headers={"content-type": "image/png"})
    )
    proxy = TileProxy("https://t.example.com/{z}/{x}/{y}.png", "", cache, transport=transport)
    status, _ = await proxy.get(1, 0, 0)
    await proxy.aclose()
    assert status == 200
    assert put_threads and put_threads[0] != threading.get_ident()
