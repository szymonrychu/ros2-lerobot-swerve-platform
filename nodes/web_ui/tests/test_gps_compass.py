"""Tests for the compass GPS anchor: pure math, IMU heading conversion and the bridge wiring."""

from __future__ import annotations

import json
import math
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from web_ui.bridge import GPS_ANCHOR_TOPIC
from web_ui.gps_anchor import (
    CompassSettings,
    GpsAnchorEstimator,
    calibration_trusted,
    compass_anchor,
    enu_to_latlon,
    enu_yaw_from_imu,
    normalize_angle,
    valid_imu_orientation,
)

from .test_map3d_backend import (
    ANCHOR_LAT,
    ANCHOR_LON,
    l_track,
    make_bridge,
    make_fix,
    make_transform,
    stamp,
    track_fixes,
)

IDENTITY = (0.0, 0.0, 0.0, 1.0)


def yaw_xyzw(yaw: float) -> tuple[float, float, float, float]:
    """Return the quaternion (x, y, z, w) of a pure yaw rotation."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def angle_close(a: float, b: float, tol: float = 1e-9) -> bool:
    """Return True if two angles are equal modulo 2 pi."""
    return abs(normalize_angle(a - b)) < tol


# ---------------------------------------------------------------------------
# compass_anchor
# ---------------------------------------------------------------------------


def test_normalize_angle_wraps_into_half_open_pi_range() -> None:
    assert normalize_angle(3 * math.pi) == pytest.approx(math.pi)
    assert normalize_angle(-3 * math.pi / 2) == pytest.approx(math.pi / 2)
    assert normalize_angle(0.25) == pytest.approx(0.25)


def test_compass_anchor_zero_pose_places_origin_at_the_fix() -> None:
    anchor = compass_anchor(ANCHOR_LAT, ANCHOR_LON, 0.0, 0.0, 0.0, 0.0)
    assert anchor["lat"] == pytest.approx(ANCHOR_LAT, abs=1e-12)
    assert anchor["lon"] == pytest.approx(ANCHOR_LON, abs=1e-12)
    assert anchor["heading_rad"] == pytest.approx(0.0)
    assert anchor["source"] == "compass"
    assert anchor["n_points"] == 1
    assert anchor["residual_m"] is None
    assert set(anchor) == {"lat", "lon", "heading_rad", "residual_m", "n_points", "source"}


@pytest.mark.parametrize(
    ("x", "y", "yaw", "heading"), [(3.0, -2.0, 0.4, 0.7), (-10.0, 5.0, -2.5, 2.0), (0.0, 4.0, math.pi, -1.2)]
)
def test_compass_anchor_reproduces_known_anchor(x: float, y: float, yaw: float, heading: float) -> None:
    """A robot at map pose (x, y, yaw) on a known anchor gives that anchor back (fit semantics)."""
    c, s = math.cos(heading), math.sin(heading)
    fix_lat, fix_lon = enu_to_latlon(c * x - s * y, s * x + c * y, ANCHOR_LAT, ANCHOR_LON)
    anchor = compass_anchor(fix_lat, fix_lon, x, y, yaw, heading + yaw)
    assert anchor["lat"] == pytest.approx(ANCHOR_LAT, abs=1e-8)
    assert anchor["lon"] == pytest.approx(ANCHOR_LON, abs=1e-8)
    assert angle_close(anchor["heading_rad"], heading)


def test_compass_anchor_heading_wraps_around() -> None:
    anchor = compass_anchor(ANCHOR_LAT, ANCHOR_LON, 0.0, 0.0, 3.0, -3.0)
    assert -math.pi < anchor["heading_rad"] <= math.pi
    assert angle_close(anchor["heading_rad"], -6.0)


# ---------------------------------------------------------------------------
# enu_yaw_from_imu
# ---------------------------------------------------------------------------


def test_imu_facing_magnetic_north_is_enu_yaw_pi_over_2() -> None:
    assert enu_yaw_from_imu(IDENTITY, None, 0.0, 0.0) == pytest.approx(math.pi / 2)


def test_imu_counter_clockwise_yaw_increases_enu_yaw() -> None:
    """Chip yaw +90 deg (facing magnetic west) is ENU yaw 180 deg."""
    assert angle_close(enu_yaw_from_imu(yaw_xyzw(math.pi / 2), None, 0.0, 0.0), math.pi)


def test_east_declination_turns_true_heading_clockwise() -> None:
    """Facing magnetic north with +6.6 deg east declination: true bearing 6.6 deg, ENU yaw 90 - 6.6 deg."""
    yaw = enu_yaw_from_imu(IDENTITY, None, 0.0, 6.6)
    assert yaw == pytest.approx(math.radians(90.0 - 6.6))


def test_mount_transform_is_removed_from_heading() -> None:
    """imu_link rotated +90 deg in base_link: IMU facing north means base_link faces east (ENU yaw 0)."""
    assert angle_close(enu_yaw_from_imu(IDENTITY, yaw_xyzw(math.pi / 2), 0.0, 0.0), 0.0)


def test_yaw_offset_used_when_no_mount_transform() -> None:
    assert angle_close(enu_yaw_from_imu(IDENTITY, None, math.pi / 2, 0.0), 0.0)


def test_mount_transform_wins_over_yaw_offset() -> None:
    assert angle_close(enu_yaw_from_imu(IDENTITY, IDENTITY, math.pi, 0.0), math.pi / 2)


def test_tilted_imu_yaw_is_extracted_from_full_orientation() -> None:
    """A 20 deg pitched IMU facing magnetic west still reports base yaw 90 deg CCW of north when mounted pitched."""
    pitch = math.radians(20.0)
    q_pitch = (0.0, math.sin(pitch / 2), 0.0, math.cos(pitch / 2))
    yaw = math.pi / 2
    qz = yaw_xyzw(yaw)
    # imu orientation = Rz(yaw) * Ry(pitch); mount (imu in base) = Ry(pitch)
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = q_pitch
    q = (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )
    assert angle_close(enu_yaw_from_imu(q, q_pitch, 0.0, 0.0), math.pi)


# ---------------------------------------------------------------------------
# validity gates
# ---------------------------------------------------------------------------


def test_valid_imu_orientation_accepts_known_unit_quaternion() -> None:
    assert valid_imu_orientation(IDENTITY, [0.01] + [0.0] * 8)


def test_valid_imu_orientation_rejects_unknown_covariance_and_zero_quaternion() -> None:
    assert not valid_imu_orientation(IDENTITY, [-1.0] + [0.0] * 8)
    assert not valid_imu_orientation((0.0, 0.0, 0.0, 0.0), [0.01] + [0.0] * 8)
    assert not valid_imu_orientation((math.nan, 0.0, 0.0, 1.0), [0.01] + [0.0] * 8)


def test_calibration_trusted_gates_on_mag_only() -> None:
    assert calibration_trusted({"sys": 1, "gyro": 3, "accel": 3, "mag": 2})
    assert not calibration_trusted({"sys": 1, "gyro": 3, "accel": 3, "mag": 1})
    # Observed on the robot: sys stays 0 in NDOF while mag is fully calibrated.
    assert calibration_trusted({"sys": 0, "gyro": 3, "accel": 3, "mag": 3})
    assert not calibration_trusted(None)
    assert not calibration_trusted({"sys": 3})


def test_calibration_trusted_when_saved_profile_restored() -> None:
    """After a restart the chip reports mag 0 until it re-checks, but the restored offsets are already valid."""
    assert calibration_trusted({"sys": 0, "gyro": 0, "accel": 0, "mag": 0, "restored": True})
    assert not calibration_trusted({"sys": 0, "gyro": 0, "accel": 0, "mag": 0, "restored": False})


# ---------------------------------------------------------------------------
# drive fit tags its anchor
# ---------------------------------------------------------------------------


def test_fit_anchor_is_tagged_with_source_fit() -> None:
    track = l_track(n=12)
    fixes = track_fixes(track, heading=0.5)
    est = GpsAnchorEstimator(min_points=10, min_spread_m=5.0, max_residual_m=1.0)
    anchor = None
    for (x, y), (lat, lon) in zip(track, fixes, strict=True):
        anchor = est.add_sample(lat, lon, float(x), float(y))
    assert anchor is not None and anchor["source"] == "fit"


# ---------------------------------------------------------------------------
# bridge wiring
# ---------------------------------------------------------------------------

SETTINGS = CompassSettings(imu_topic="/imu/data")
FIX = {"latitude": ANCHOR_LAT, "longitude": ANCHOR_LON, "stamp": 100.0}


def imu_msg(yaw: float = 0.0, sec: int = 100, cov0: float = 0.01, frame_id: str = "imu_link") -> SimpleNamespace:
    """Return a sensor_msgs/Imu-like namespace facing chip yaw."""
    x, y, z, w = yaw_xyzw(yaw)
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame_id, stamp=stamp(sec)),
        orientation=SimpleNamespace(x=x, y=y, z=z, w=w),
        orientation_covariance=[cov0] + [0.0] * 8,
    )


def compass_bridge(settings: CompassSettings | None = SETTINGS, est: object | None = None, **attrs: object) -> object:
    """Return a bridge with a TF buffer, a (non-passing) estimator and compass settings."""
    tf_buffer = MagicMock()
    tf_buffer.lookup_transform.side_effect = lambda target, _source, _t: (
        make_transform(0.0, 0.0, 0.0, sec=100) if target == "map" else make_transform(0.0, 0.0, 0.0, sec=0)
    )
    est = est if est is not None else GpsAnchorEstimator()
    return make_bridge(
        _tf_buffer=tf_buffer, _robot_pose_frames=("map", "base_link"), _gps_anchor=est, _compass=settings, **attrs
    )


def published(node: object) -> list[dict]:
    return [e for e in node.flush_dirty() if e["topic"] == GPS_ANCHOR_TOPIC]


def test_bridge_publishes_compass_anchor_while_parked() -> None:
    node = compass_bridge()
    node.on_anchor_imu(imu_msg(yaw=0.0))
    node.feed_gps_anchor(FIX)
    (entry,) = published(node)
    data = entry["data"]
    assert data["source"] == "compass" and data["n_points"] == 1 and data["residual_m"] is None
    # robot at map origin facing map +x, IMU faces magnetic north (declination 0): heading = pi/2 - 0
    assert data["heading_rad"] == pytest.approx(math.pi / 2)
    assert data["lat"] == pytest.approx(ANCHOR_LAT) and data["lon"] == pytest.approx(ANCHOR_LON)


def test_bridge_applies_declination_and_map_yaw() -> None:
    node = compass_bridge(CompassSettings(imu_topic="/imu/data", declination_deg=6.6))
    node.on_anchor_imu(imu_msg(yaw=0.0))
    node._tf_buffer.lookup_transform.side_effect = lambda target, _source, _t: (
        make_transform(0.0, 0.0, 0.3, sec=100) if target == "map" else make_transform(0.0, 0.0, 0.0, sec=0)
    )
    node.feed_gps_anchor(FIX)
    (entry,) = published(node)
    assert entry["data"]["heading_rad"] == pytest.approx(math.radians(90.0 - 6.6) - 0.3)


def test_bridge_uses_base_to_imu_tf_for_mounting() -> None:
    node = compass_bridge()
    node.on_anchor_imu(imu_msg(yaw=0.0))
    node._tf_buffer.lookup_transform.side_effect = lambda target, _source, _t: (
        make_transform(0.0, 0.0, 0.0, sec=100) if target == "map" else make_transform(0.0, 0.0, math.pi / 2, sec=0)
    )
    node.feed_gps_anchor(FIX)
    (entry,) = published(node)
    assert entry["data"]["heading_rad"] == pytest.approx(0.0, abs=1e-9)


def test_bridge_falls_back_to_yaw_offset_without_mount_tf() -> None:
    from tf2_ros import TransformException

    node = compass_bridge(CompassSettings(imu_topic="/imu/data", imu_yaw_offset_deg=90.0))
    node.on_anchor_imu(imu_msg(yaw=0.0))
    pose_tf = make_transform(0.0, 0.0, 0.0, sec=100)

    def lookup(target: str, _source: str, _t: object) -> object:
        if target == "map":
            return pose_tf
        raise TransformException("no imu tf")

    node._tf_buffer.lookup_transform.side_effect = lookup
    node.feed_gps_anchor(FIX)
    (entry,) = published(node)
    assert entry["data"]["heading_rad"] == pytest.approx(0.0, abs=1e-9)


def test_bridge_publishes_nothing_without_imu_or_with_stale_or_invalid_imu() -> None:
    node = compass_bridge()
    node.feed_gps_anchor(FIX)
    assert published(node) == []
    node.on_anchor_imu(imu_msg(sec=90))  # 10 s older than the fix
    node.feed_gps_anchor(FIX)
    assert published(node) == []
    node.on_anchor_imu(imu_msg(sec=100, cov0=-1.0))  # unknown orientation must not replace anything
    node.feed_gps_anchor(FIX)
    assert published(node) == []


def test_bridge_publishes_nothing_when_compass_disabled() -> None:
    node = compass_bridge(settings=None)
    node.feed_gps_anchor(FIX)
    assert published(node) == []


def test_bridge_calibration_gate_blocks_until_trusted() -> None:
    settings = CompassSettings(imu_topic="/imu/data", calibration_topic="/imu/calibration")
    node = compass_bridge(settings)
    node.on_anchor_imu(imu_msg())
    node.feed_gps_anchor(FIX)
    assert published(node) == []  # no calibration message yet
    node.on_anchor_calibration(SimpleNamespace(data=json.dumps({"sys": 1, "gyro": 3, "accel": 3, "mag": 1})))
    node.feed_gps_anchor(FIX)
    assert published(node) == []
    node.on_anchor_calibration(SimpleNamespace(data=json.dumps({"sys": 3, "gyro": 3, "accel": 3, "mag": 3})))
    node.feed_gps_anchor(FIX)
    assert len(published(node)) == 1
    node.on_anchor_calibration(SimpleNamespace(data="not json"))  # garbage is ignored, last good value kept


def test_bridge_stale_calibration_is_not_trusted() -> None:
    settings = CompassSettings(imu_topic="/imu/data", calibration_topic="/imu/calibration")
    node = compass_bridge(settings)
    node.on_anchor_imu(imu_msg())
    node._imu_calibration = ({"sys": 3, "gyro": 3, "accel": 3, "mag": 3}, time.monotonic() - 1000.0)
    node.feed_gps_anchor(FIX)
    assert published(node) == []


def test_bridge_compass_anchor_not_republished_for_tiny_changes() -> None:
    node = compass_bridge()
    node.on_anchor_imu(imu_msg(yaw=0.0))
    node.feed_gps_anchor(FIX)
    assert len(published(node)) == 1
    node.on_anchor_imu(imu_msg(yaw=math.radians(0.05)))
    node.feed_gps_anchor(FIX)
    assert published(node) == []
    node.on_anchor_imu(imu_msg(yaw=math.radians(1.0)))
    node.feed_gps_anchor(FIX)
    assert len(published(node)) == 1


def test_bridge_fit_wins_and_is_not_overwritten_by_compass() -> None:
    est = MagicMock()
    fit = {"lat": 1.0, "lon": 2.0, "heading_rad": 0.1, "residual_m": 0.2, "n_points": 12, "source": "fit"}
    est.add_sample.return_value = fit
    node = compass_bridge(est=est)
    node.on_anchor_imu(imu_msg())
    node.feed_gps_anchor(FIX)
    assert [e["data"] for e in published(node)] == [fit]
    est.add_sample.return_value = fit
    node.feed_gps_anchor(FIX)
    assert published(node) == []


def test_bridge_falls_back_to_compass_when_fit_degrades() -> None:
    est = MagicMock()
    fit = {"lat": 1.0, "lon": 2.0, "heading_rad": 0.1, "residual_m": 0.2, "n_points": 12, "source": "fit"}
    est.add_sample.side_effect = [fit, None]
    node = compass_bridge(est=est)
    node.on_anchor_imu(imu_msg())
    node.feed_gps_anchor(FIX)
    node.flush_dirty()
    node.feed_gps_anchor(FIX)
    (entry,) = published(node)
    assert entry["data"]["source"] == "compass"


def test_bridge_reset_clears_compass_anchor() -> None:
    node = compass_bridge()
    node.on_anchor_imu(imu_msg())
    node.feed_gps_anchor(FIX)
    node.flush_dirty()
    node.reset_gps_anchor()
    assert published(node) == [{"topic": GPS_ANCHOR_TOPIC, "data": None}]
    assert node.latest_envelopes() == []


def test_bridge_compass_anchor_via_fix_callback() -> None:
    from web_ui.msg_serializer import serialize_navsatfix

    node = compass_bridge()
    node.on_anchor_imu(imu_msg())
    cb = node._make_callback("/client/gps/fix", serialize_navsatfix, "gps")
    cb(make_fix(ANCHOR_LAT, ANCHOR_LON, sec=100))
    assert published(node)[0]["data"]["source"] == "compass"
