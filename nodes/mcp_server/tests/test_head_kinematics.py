"""Tests for mcp_server.head_kinematics: head URDF forward kinematics and look-at aiming of the RGB optical axis."""

import math

import numpy as np
import pytest

from mcp_server.config import REPO_ROOT
from mcp_server.head_kinematics import (
    CAMERA_FRAME,
    DEFAULT_HEAD_URDF,
    TOF_FRAME,
    HeadAimError,
    HeadKinematics,
)

HEAD_URDF = REPO_ROOT / "nodes/web_ui/urdf/head.urdf"
# URDF origins at pan = tilt = 0: mount (0.15, 0.10, 0.31) + tilt axis (0, 0, 0.04) + camera / ToF (0.02, -+0.025, 0).
CAMERA_HOME_XYZ = (0.17, 0.075, 0.35)
TOF_HOME_XYZ = (0.17, 0.125, 0.35)
# Optical frame columns at zero angles in base_link: x right (-y), y down (-z), z forward (+x).
OPTICAL_HOME_ROTATION = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
RESIDUAL_LIMIT_DEG = 0.2
# Reachable points in base_link (m): left, right, near, far, raised, and close to the lowest view (tilt limit 30 deg).
REACHABLE_POINTS = (
    (1.0, 0.5, 0.0),  # floor, left
    (1.0, -0.5, 0.0),  # floor, right
    (0.9, 0.3, 0.0),  # floor, near
    (3.0, 0.0, 0.0),  # floor, far
    (0.85, 0.1, 0.0),  # floor, close to the lowest view (tilt near 30 deg)
    (0.3, 0.9, 0.3),  # raised, far left (pan about 80 deg)
    (0.4, -0.7, 0.2),  # raised, far right
    (2.0, 1.0, 0.5),  # raised above the camera, left
    (1.5, -0.8, 0.8),  # raised above the camera, right (tilt up)
)


@pytest.fixture(scope="module")
def head() -> HeadKinematics:
    return HeadKinematics(HEAD_URDF)


def optical_axis(head: HeadKinematics, pan: float, tilt: float) -> tuple[np.ndarray, np.ndarray]:
    pose = head.link_pose(pan, tilt, CAMERA_FRAME)
    return pose[:3, 3], pose[:3, 2]


def distance_from_axis(head: HeadKinematics, pan: float, tilt: float, point: np.ndarray) -> tuple[float, float]:
    """(perpendicular distance of point from the optical axis ray, distance along it) in m."""
    origin, axis = optical_axis(head, pan, tilt)
    ray = point - origin
    along = float(ray @ axis)
    return float(np.linalg.norm(ray - along * axis)), along


def test_default_urdf_is_the_web_ui_head_urdf() -> None:
    assert DEFAULT_HEAD_URDF == HEAD_URDF
    assert DEFAULT_HEAD_URDF.is_file()


def test_limits_come_from_the_urdf(head: HeadKinematics) -> None:
    assert head.limits == {
        "head_pan": pytest.approx((-1.5708, 1.5708)),
        "head_tilt": pytest.approx((-0.5236, 0.5236)),
    }


def test_optical_axis_is_base_link_x_at_zero(head: HeadKinematics) -> None:
    _origin, axis = optical_axis(head, 0.0, 0.0)
    np.testing.assert_allclose(axis, [1.0, 0.0, 0.0], atol=1e-4)


def test_link_pose_of_camera_and_tof_match_urdf_origins(head: HeadKinematics) -> None:
    camera = head.link_pose(0.0, 0.0, CAMERA_FRAME)
    tof = head.link_pose(0.0, 0.0, TOF_FRAME)
    np.testing.assert_allclose(camera[:3, 3], CAMERA_HOME_XYZ, atol=1e-9)
    np.testing.assert_allclose(tof[:3, 3], TOF_HOME_XYZ, atol=1e-9)
    np.testing.assert_allclose(camera[:3, :3], OPTICAL_HOME_ROTATION, atol=1e-4)
    np.testing.assert_allclose(tof[:3, :3], OPTICAL_HOME_ROTATION, atol=1e-4)
    np.testing.assert_allclose(camera[3], [0.0, 0.0, 0.0, 1.0])


def test_link_pose_follows_pan_and_tilt_signs(head: HeadKinematics) -> None:
    # pan +90 deg looks left: the camera offset (0.02, -0.025) rotates to (0.025, 0.02) about the pan axis.
    left = head.link_pose(math.pi / 2, 0.0, "head_camera_link")
    np.testing.assert_allclose(left[:3, 3], [0.175, 0.12, 0.35], atol=1e-9)
    np.testing.assert_allclose(left[:3, 0], [0.0, 1.0, 0.0], atol=1e-9)
    # tilt +30 deg looks down: the optical axis points forward and below horizontal.
    _origin, axis = optical_axis(head, 0.0, math.radians(30.0))
    np.testing.assert_allclose(axis, [math.cos(math.radians(30.0)), 0.0, -math.sin(math.radians(30.0))], atol=1e-4)
    # the tilt axis sits 0.04 m above the pan base, so tilting does not move head_link.
    np.testing.assert_allclose(head.link_pose(0.3, 0.5, "head_link")[:3, 3], [0.15, 0.10, 0.35], atol=1e-9)
    np.testing.assert_allclose(head.link_pose(0.3, 0.5, "base_link"), np.eye(4), atol=1e-12)


def test_link_pose_rejects_unknown_link(head: HeadKinematics) -> None:
    with pytest.raises(ValueError, match="no_such_link"):
        head.link_pose(0.0, 0.0, "no_such_link")


@pytest.mark.parametrize("point", REACHABLE_POINTS)
def test_look_at_aims_the_optical_axis_through_reachable_points(head: HeadKinematics, point: tuple) -> None:
    target = np.array(point)
    pan, tilt = head.look_at(target)
    (lo_pan, hi_pan), (lo_tilt, hi_tilt) = head.limits["head_pan"], head.limits["head_tilt"]
    assert lo_pan <= pan <= hi_pan
    assert lo_tilt <= tilt <= hi_tilt
    assert math.degrees(head.aim_error(pan, tilt, target)) < RESIDUAL_LIMIT_DEG
    off_axis, along = distance_from_axis(head, pan, tilt, target)
    assert along > 0.0
    # within 2 mm at 2 m, i.e. 1 mrad of the ray
    assert off_axis <= 0.002 * along / 2.0


def test_look_at_hits_a_point_two_metres_away_within_two_millimetres(head: HeadKinematics) -> None:
    camera, _axis = optical_axis(head, 0.0, 0.0)
    target = camera + 2.0 * np.array([math.cos(0.4) * math.cos(0.3), math.sin(0.4) * math.cos(0.3), -math.sin(0.3)])
    pan, tilt = head.look_at(target)
    off_axis, along = distance_from_axis(head, pan, tilt, target)
    assert along == pytest.approx(2.0, abs=0.05)
    assert off_axis < 0.002


def test_look_at_accounts_for_camera_offsets(head: HeadKinematics) -> None:
    # The offset-free atan2 from the pan axis would miss: the camera sits 0.025 m right and 0.02 m forward.
    target = np.array([0.5, 0.6, 0.0])
    pan, tilt = head.look_at(target)
    naive_pan = math.atan2(0.6 - 0.10, 0.5 - 0.15)
    naive_tilt = math.atan2(0.35, math.hypot(0.6 - 0.10, 0.5 - 0.15))
    assert math.degrees(head.aim_error(naive_pan, naive_tilt, target)) > 1.0
    assert math.degrees(head.aim_error(pan, tilt, target)) < RESIDUAL_LIMIT_DEG


def test_home_view_floor_point_gives_tilt_near_30_deg(head: HeadKinematics) -> None:
    # straight ahead of the camera, on the floor, 0.6 m ahead
    camera, _axis = optical_axis(head, 0.0, 0.0)
    target = np.array([camera[0] + 0.6, camera[1], 0.0])
    pan, tilt = head.look_at(target)
    assert pan == pytest.approx(0.0, abs=1e-3)
    assert math.degrees(tilt) == pytest.approx(30.0, abs=2.0)


def test_point_straight_behind_is_refused_with_the_angles(head: HeadKinematics) -> None:
    with pytest.raises(HeadAimError) as info:
        head.look_at(np.array([-1.0, 0.1, 0.3]))
    err = info.value
    assert isinstance(err, ValueError)
    assert abs(err.pan) > math.radians(90.0)
    assert err.limits == head.limits
    assert "pan" in str(err) and "tilt" in str(err)
    assert f"{math.degrees(err.pan):.1f}" in str(err)


def test_point_needing_tilt_past_30_deg_is_refused_with_the_angles(head: HeadKinematics) -> None:
    with pytest.raises(HeadAimError) as info:
        head.look_at(np.array([0.3, 0.075, 0.0]))
    err = info.value
    assert err.tilt > math.radians(30.0)
    assert abs(err.pan) < math.radians(90.0)
    assert err.limits == head.limits
    assert f"{math.degrees(err.tilt):.1f}" in str(err)
    # the needed angles really aim at the point: refused, not clamped
    assert math.degrees(head.aim_error(err.pan, err.tilt, np.array([0.3, 0.075, 0.0]))) < RESIDUAL_LIMIT_DEG


def test_point_too_close_to_the_camera_is_refused(head: HeadKinematics) -> None:
    camera, axis = optical_axis(head, 0.0, 0.0)
    with pytest.raises(HeadAimError, match="0.15"):
        head.look_at(camera + 0.1 * axis)


def test_point_off_the_reachable_cone_is_refused_not_clamped(head: HeadKinematics) -> None:
    # a point the camera cannot aim at within limits: look_at raises instead of returning clamped angles
    target = np.array([0.0, -1.5, 0.0])
    with pytest.raises(HeadAimError) as info:
        head.look_at(target)
    assert info.value.pan < -math.radians(90.0)
