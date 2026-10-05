"""Unit tests for the pose-difference body twist and covariance of the rf2o relay."""

import math

import pytest

from rf2o_odom_relay.twist import PoseSample, body_twist, twist_covariance

DT = 0.1


def test_straight_ahead_when_yaw_is_zero() -> None:
    twist = body_twist(PoseSample(0.0, 0.0, 0.0, 1.0), PoseSample(0.02, 0.0, 0.0, 1.0 + DT))
    assert twist == pytest.approx((0.2, 0.0, 0.0))


def test_world_motion_is_rotated_into_the_body_frame() -> None:
    # Robot heading +90 deg moves along world +y: that is body +x (forward), not body vy.
    yaw = math.pi / 2
    twist = body_twist(PoseSample(1.0, 1.0, yaw, 5.0), PoseSample(1.0, 1.03, yaw, 5.0 + DT))
    assert twist == pytest.approx((0.3, 0.0, 0.0), abs=1e-9)


def test_sideways_motion_is_reported_as_vy() -> None:
    # rf2o's own twist always has vy = 0; the relay recovers the holonomic lateral speed.
    twist = body_twist(PoseSample(0.0, 0.0, 0.0, 1.0), PoseSample(0.0, 0.01, 0.0, 1.0 + DT))
    assert twist == pytest.approx((0.0, 0.1, 0.0))


def test_yaw_rate_wraps_across_pi() -> None:
    twist = body_twist(PoseSample(0.0, 0.0, math.pi - 0.01, 1.0), PoseSample(0.0, 0.0, -math.pi + 0.01, 1.0 + DT))
    assert twist[2] == pytest.approx(0.2)


def test_translation_during_a_turn_uses_the_mid_heading() -> None:
    # Rotating 0.2 rad while advancing: mid heading 0.1 rad, so a world step of (cos 0.1, sin 0.1) * 0.05 is pure vx.
    step = 0.05
    twist = body_twist(
        PoseSample(0.0, 0.0, 0.0, 1.0),
        PoseSample(step * math.cos(0.1), step * math.sin(0.1), 0.2, 1.0 + DT),
    )
    assert twist == pytest.approx((0.5, 0.0, 2.0), abs=1e-9)


@pytest.mark.parametrize("dt", [0.0, -0.1, 1.5])
def test_no_twist_for_unusable_time_steps(dt: float) -> None:
    assert body_twist(PoseSample(0, 0, 0, 1.0), PoseSample(0.1, 0, 0, 1.0 + dt), max_dt_s=1.0) is None


def test_twist_covariance_diagonal_only() -> None:
    cov = twist_covariance(0.02, 0.05)
    assert len(cov) == 36
    assert cov[0] == cov[7] == 0.02
    assert cov[35] == 0.05
    assert sum(1 for v in cov if v != 0.0) == 3
