"""Forward and inverse kinematics for symmetric 4-wheel swerve drive."""

import math

import numpy as np

# Wheel positions in body frame (x forward, y left): fl, fr, rl, rr.
# Order matches joint_names: fl_drive, fl_steer, fr_drive, fr_steer, rl_drive, rl_steer, rr_drive, rr_steer.
WHEEL_ORDER = ("fl", "fr", "rl", "rr")


def wheel_positions(lx: float, ly: float) -> np.ndarray:
    """Return 4x2 array of (x, y) wheel positions in body frame for fl, fr, rl, rr.

    Args:
        lx: Half-length (center to front/rear axle), meters.
        ly: Half-width (center to left/right), meters.

    Returns:
        np.ndarray: Shape (4, 2), rows are [x, y] for each wheel (fl, fr, rl, rr).
    """
    return np.array(
        [
            [lx, ly],  # fl
            [lx, -ly],  # fr
            [-lx, ly],  # rl
            [-lx, -ly],  # rr
        ],
        dtype=float,
    )


def inverse_kinematics(
    vx: float,
    vy: float,
    omega: float,
    lx: float,
    ly: float,
    wheel_radius: float,
) -> tuple[list[float], list[float]]:
    """Compute steering angles (rad) and drive angular velocities (rad/s) for each wheel.

    Body frame: x forward, y left, omega positive = counterclockwise (yaw).
    Wheel order: fl, fr, rl, rr.

    Args:
        vx: Forward velocity in body frame, m/s.
        vy: Leftward velocity in body frame, m/s.
        omega: Angular velocity about vertical, rad/s.
        lx: Half-length, m.
        ly: Half-width, m.
        wheel_radius: Wheel radius, m.

    Returns:
        tuple: (steer_angles, drive_angular_velocities), each list of 4 floats (rad, rad/s).
    """
    positions = wheel_positions(lx, ly)
    steer_angles: list[float] = []
    drive_angular: list[float] = []
    for i in range(4):
        x_i, y_i = positions[i, 0], positions[i, 1]
        vx_i = vx - omega * y_i
        vy_i = vy + omega * x_i
        speed = math.sqrt(vx_i * vx_i + vy_i * vy_i)
        if speed < 1e-9:
            steer_angles.append(0.0)
            drive_angular.append(0.0)
            continue
        alpha = math.atan2(vy_i, vx_i)
        steer_angles.append(alpha)
        drive_angular.append(speed / wheel_radius)
    return steer_angles, drive_angular


def forward_kinematics(
    steer_angles: list[float],
    drive_angular_velocities: list[float],
    lx: float,
    ly: float,
    wheel_radius: float,
) -> tuple[float, float, float]:
    """Compute body twist (vx, vy, omega) from wheel states.

    Uses least-squares: wheel velocities must be consistent with a single body twist.

    Args:
        steer_angles: Steering angle per wheel (rad), order fl, fr, rl, rr.
        drive_angular_velocities: Drive angular velocity per wheel (rad/s), order fl, fr, rl, rr.
        lx: Half-length, m.
        ly: Half-width, m.
        wheel_radius: Wheel radius, m.

    Returns:
        tuple: (vx, vy, omega) in body frame (m/s, m/s, rad/s).
    """
    positions = wheel_positions(lx, ly)
    # Each wheel i: vx_i = s_i*cos(alpha_i), vy_i = s_i*sin(alpha_i) with s_i = drive_angular_i * R.
    # Body: vx_i = vx - omega*y_i, vy_i = vy + omega*x_i.
    # So we have 8 equations: for i in 0..3, [vx_i, vy_i] = [vx - omega*y_i, vy + omega*x_i].
    # Stack as A @ [vx, vy, omega] = b, where b = [vx_0, vy_0, vx_1, vy_1, ...].
    A = np.zeros((8, 3))
    b = np.zeros(8)
    for i in range(4):
        x_i, y_i = positions[i, 0], positions[i, 1]
        alpha = steer_angles[i] if i < len(steer_angles) else 0.0
        drive = drive_angular_velocities[i] if i < len(drive_angular_velocities) else 0.0
        s_i = drive * wheel_radius
        vx_i = s_i * math.cos(alpha)
        vy_i = s_i * math.sin(alpha)
        row_vx = 2 * i
        row_vy = 2 * i + 1
        A[row_vx, :] = [1, 0, -y_i]
        A[row_vy, :] = [0, 1, x_i]
        b[row_vx] = vx_i
        b[row_vy] = vy_i
    x, _residuals, _rank, _s = np.linalg.lstsq(A, b, rcond=None)
    return (float(x[0]), float(x[1]), float(x[2]))


def steer_angle_difference(current: float, desired: float) -> float:
    """Smallest difference (rad) from current to desired steer angle in [-pi, pi].

    Args:
        current: Current steering angle, rad.
        desired: Desired steering angle, rad.

    Returns:
        float: Difference in rad, in [-pi, pi].
    """
    diff = desired - current
    while diff > math.pi:
        diff -= 2.0 * math.pi
    while diff < -math.pi:
        diff += 2.0 * math.pi
    return diff


def should_zero_drive(
    current_steer: float,
    desired_steer: float,
    threshold_rad: float,
) -> bool:
    """Return True if drive should be zeroed to avoid strain (steer error above threshold).

    Args:
        current_steer: Current steering angle, rad.
        desired_steer: Desired steering angle, rad.
        threshold_rad: Max allowed error before zeroing drive, rad.

    Returns:
        bool: True if |error| > threshold.
    """
    return abs(steer_angle_difference(current_steer, desired_steer)) > threshold_rad
