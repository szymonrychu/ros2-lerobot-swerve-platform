"""Unit tests for swerve drive kinematics (IK, FK, safeguard)."""

import math

import pytest

from swerve_drive_controller.kinematics import (
    compute_wheel_commands,
    desaturate_wheel_speeds,
    fold_to_steer_range,
    forward_kinematics,
    integrate_odometry,
    inverse_kinematics,
    normalize_angle,
    should_zero_drive,
    steer_angle_difference,
    wheel_positions,
    wheel_states,
)


def test_wheel_positions() -> None:
    lx, ly = 0.2, 0.15
    pos = wheel_positions(lx, ly)
    assert pos.shape == (4, 2)
    assert pos[0, 0] == lx and pos[0, 1] == ly  # fl
    assert pos[1, 0] == lx and pos[1, 1] == -ly  # fr
    assert pos[2, 0] == -lx and pos[2, 1] == ly  # rl
    assert pos[3, 0] == -lx and pos[3, 1] == -ly  # rr


def test_inverse_kinematics_straight_forward() -> None:
    steer, drive = inverse_kinematics(1.0, 0.0, 0.0, 0.2, 0.15, 0.15)
    assert len(steer) == 4 and len(drive) == 4
    for i in range(4):
        assert abs(steer[i]) < 1e-9
    for i in range(4):
        assert abs(drive[i] - 1.0 / 0.15) < 1e-6


def test_inverse_kinematics_straight_sideways() -> None:
    steer, drive = inverse_kinematics(0.0, 1.0, 0.0, 0.2, 0.15, 0.15)
    for i in range(4):
        assert abs(steer[i] - math.pi / 2) < 1e-6
    for i in range(4):
        assert abs(drive[i] - 1.0 / 0.15) < 1e-6


def test_inverse_kinematics_zero() -> None:
    steer, drive = inverse_kinematics(0.0, 0.0, 0.0, 0.2, 0.15, 0.15)
    for i in range(4):
        assert steer[i] == 0.0 and drive[i] == 0.0


def test_forward_kinematics_roundtrip() -> None:
    vx, vy, omega = 0.5, -0.2, 0.3
    steer, drive = inverse_kinematics(vx, vy, omega, 0.2, 0.15, 0.15)
    vx_fk, vy_fk, omega_fk = forward_kinematics(steer, drive, 0.2, 0.15, 0.15)
    assert abs(vx_fk - vx) < 1e-6 and abs(vy_fk - vy) < 1e-6 and abs(omega_fk - omega) < 1e-6


def test_steer_angle_difference() -> None:
    assert abs(steer_angle_difference(0.0, 0.0)) < 1e-9
    assert abs(steer_angle_difference(0.0, math.pi / 2) - math.pi / 2) < 1e-9
    # -pi and +pi are the same angle (difference 0)
    assert abs(steer_angle_difference(math.pi, -math.pi)) < 1e-9


def test_should_zero_drive() -> None:
    assert should_zero_drive(0.0, 0.0, 0.35) is False
    assert should_zero_drive(0.0, 0.5, 0.35) is True
    assert should_zero_drive(0.0, 0.2, 0.35) is False


def test_normalize_angle_already_normalized() -> None:
    assert abs(normalize_angle(0.5) - 0.5) < 1e-9


def test_normalize_angle_wraps_positive() -> None:
    assert abs(normalize_angle(math.pi + 0.1) - (-math.pi + 0.1)) < 1e-6


def test_normalize_angle_wraps_negative() -> None:
    assert abs(normalize_angle(-math.pi - 0.1) - (math.pi - 0.1)) < 1e-6


# Real platform geometry (Platform dimensions.md): 305 x 266.6 mm yaw-axis rectangle, 60 mm wheels.
LX = 0.1525
LY = 0.1333
R = 0.06
HALF_PI = math.pi / 2


def test_fold_keeps_angle_inside_limit() -> None:
    assert fold_to_steer_range(0.3, 2.0, current_steer=0.0, limit=HALF_PI) == pytest.approx((0.3, 2.0))


def test_fold_flips_backward_angle_and_reverses_drive() -> None:
    steer, drive = fold_to_steer_range(math.pi - 0.2, 2.0, current_steer=0.0, limit=HALF_PI)
    assert steer == pytest.approx(-0.2)
    assert drive == pytest.approx(-2.0)


def test_fold_straight_backward_is_zero_steer_negative_drive() -> None:
    steer, drive = fold_to_steer_range(math.pi, 1.0, current_steer=0.0, limit=HALF_PI)
    assert steer == pytest.approx(0.0, abs=1e-9)
    assert drive == pytest.approx(-1.0)


def test_fold_boundary_picks_side_closer_to_current() -> None:
    steer, drive = fold_to_steer_range(HALF_PI, 1.0, current_steer=-1.4, limit=HALF_PI)
    assert steer == pytest.approx(-HALF_PI)
    assert drive == pytest.approx(-1.0)
    steer, drive = fold_to_steer_range(-HALF_PI, 1.0, current_steer=1.4, limit=HALF_PI)
    assert steer == pytest.approx(HALF_PI)
    assert drive == pytest.approx(-1.0)


def test_desaturate_scales_all_wheels_proportionally() -> None:
    assert desaturate_wheel_speeds([2.0, -8.0, 4.0, 1.0], 4.0) == pytest.approx([1.0, -4.0, 2.0, 0.5])


def test_desaturate_noop_within_limit() -> None:
    assert desaturate_wheel_speeds([1.0, -2.0, 0.0, 3.9], 4.0) == pytest.approx([1.0, -2.0, 0.0, 3.9])


def test_wheel_commands_forward() -> None:
    steer, drive = compute_wheel_commands(0.1, 0.0, 0.0, [0.0] * 4, LX, LY, R, HALF_PI, 10.0)
    assert steer == pytest.approx([0.0] * 4)
    assert drive == pytest.approx([0.1 / R] * 4)


def test_wheel_commands_backward_keeps_wheels_straight() -> None:
    steer, drive = compute_wheel_commands(-0.1, 0.0, 0.0, [0.0] * 4, LX, LY, R, HALF_PI, 10.0)
    assert steer == pytest.approx([0.0] * 4, abs=1e-9)
    assert drive == pytest.approx([-0.1 / R] * 4)


def test_wheel_commands_strafe_left_within_limits() -> None:
    steer, drive = compute_wheel_commands(0.0, 0.1, 0.0, [0.0] * 4, LX, LY, R, HALF_PI, 10.0)
    for s, d in zip(steer, drive):
        assert abs(s) == pytest.approx(HALF_PI)
        assert math.copysign(1.0, s) * d * R == pytest.approx(0.1)


def test_wheel_commands_rotate_in_place() -> None:
    """Pure yaw: wheels tangent to the circle through the yaw axes, all at the same speed."""
    omega = 0.5
    steer, drive = compute_wheel_commands(0.0, 0.0, omega, [0.0] * 4, LX, LY, R, HALF_PI, 10.0)
    alpha = math.atan2(LX, LY)
    assert steer == pytest.approx([-alpha, alpha, alpha, -alpha])
    speed = omega * math.hypot(LX, LY) / R
    assert [abs(d) for d in drive] == pytest.approx([speed] * 4)
    vx, vy, w = forward_kinematics(steer, drive, LX, LY, R)
    assert (vx, vy, w) == pytest.approx((0.0, 0.0, omega), abs=1e-9)


def test_wheel_commands_stopped_holds_current_steer() -> None:
    current = [0.4, -0.2, 0.1, 0.0]
    steer, drive = compute_wheel_commands(0.0, 0.0, 0.0, current, LX, LY, R, HALF_PI, 10.0)
    assert steer == pytest.approx(current)
    assert drive == pytest.approx([0.0] * 4)


def test_wheel_commands_desaturated_preserves_direction() -> None:
    steer, drive = compute_wheel_commands(1.0, 0.0, 1.0, [0.0] * 4, LX, LY, R, HALF_PI, 4.0)
    assert max(abs(d) for d in drive) == pytest.approx(4.0)
    vx, vy, w = forward_kinematics(steer, drive, LX, LY, R)
    assert w / vx == pytest.approx(1.0)


def test_wheel_commands_roundtrip_random_twists() -> None:
    for vx, vy, omega in [(0.1, 0.05, 0.2), (-0.08, 0.02, -0.4), (0.0, -0.1, 0.3), (0.05, 0.0, -1.0)]:
        steer, drive = compute_wheel_commands(vx, vy, omega, [0.0] * 4, LX, LY, R, HALF_PI, 100.0)
        assert all(abs(s) <= HALF_PI + 1e-9 for s in steer)
        assert forward_kinematics(steer, drive, LX, LY, R) == pytest.approx((vx, vy, omega), abs=1e-9)


def test_wheel_states_requires_all_joints() -> None:
    positions = {"fl_steer": 0.1, "fr_steer": 0.2, "rl_steer": 0.3}
    velocities = {"fl_drive": 1.0, "fr_drive": 1.0, "rl_drive": 1.0, "rr_drive": 1.0}
    steer_joints = ["fl_steer", "fr_steer", "rl_steer", "rr_steer"]
    drive_joints = ["fl_drive", "fr_drive", "rl_drive", "rr_drive"]
    assert wheel_states(positions, velocities, steer_joints, drive_joints) is None
    positions["rr_steer"] = 0.4
    assert wheel_states(positions, velocities, steer_joints, drive_joints) == ([0.1, 0.2, 0.3, 0.4], [1.0] * 4)


def test_integrate_odometry_straight_then_rotated() -> None:
    assert integrate_odometry((0.0, 0.0, 0.0), (0.1, 0.0, 0.0), 1.0) == pytest.approx((0.1, 0.0, 0.0))
    x, y, theta = integrate_odometry((0.0, 0.0, HALF_PI), (0.1, 0.0, 0.0), 1.0)
    assert (x, y, theta) == pytest.approx((0.0, 0.1, HALF_PI), abs=1e-9)


def test_integrate_odometry_arc_uses_midpoint_heading() -> None:
    x, y, theta = integrate_odometry((0.0, 0.0, 0.0), (0.1, 0.0, 0.2), 1.0)
    assert theta == pytest.approx(0.2)
    assert x == pytest.approx(0.1 * math.cos(0.1))
    assert y == pytest.approx(0.1 * math.sin(0.1))


def test_fold_hysteresis_keeps_side_just_past_limit() -> None:
    """Heading slightly past +90 deg with the wheel at the + side: stay at +90 instead of swinging to -90."""
    steer, drive = fold_to_steer_range(HALF_PI + 0.05, 1.0, current_steer=HALF_PI - 0.01, limit=HALF_PI)
    assert steer == pytest.approx(HALF_PI)
    assert drive == pytest.approx(1.0)
    steer, drive = fold_to_steer_range(-HALF_PI - 0.05, 1.0, current_steer=-HALF_PI + 0.01, limit=HALF_PI)
    assert steer == pytest.approx(-HALF_PI)
    assert drive == pytest.approx(1.0)


def test_fold_hysteresis_switches_side_when_far_past_limit() -> None:
    steer, drive = fold_to_steer_range(HALF_PI + 0.5, 1.0, current_steer=HALF_PI, limit=HALF_PI)
    assert steer == pytest.approx(-HALF_PI + 0.5)
    assert drive == pytest.approx(-1.0)
