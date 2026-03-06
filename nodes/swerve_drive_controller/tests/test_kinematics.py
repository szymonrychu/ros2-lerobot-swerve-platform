"""Unit tests for swerve drive kinematics (IK, FK, safeguard)."""

import math

from swerve_drive_controller.kinematics import (
    forward_kinematics,
    inverse_kinematics,
    should_zero_drive,
    steer_angle_difference,
    wheel_positions,
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
