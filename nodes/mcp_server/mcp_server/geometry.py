"""Planar pose helpers (yaw/quaternion, relative goals) and twist clamping."""

import math


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Yaw (rotation about z) of a quaternion.

    Args:
        x (float): Quaternion x.
        y (float): Quaternion y.
        z (float): Quaternion z.
        w (float): Quaternion w.

    Returns:
        float: Yaw in rad, in (-pi, pi].
    """
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    """Quaternion (x, y, z, w) of a pure yaw rotation.

    Args:
        yaw (float): Yaw in rad.

    Returns:
        tuple[float, float, float, float]: Quaternion.
    """
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def normalize_angle(angle: float) -> float:
    """Wrap an angle into [-pi, pi].

    Args:
        angle (float): Angle in rad.

    Returns:
        float: Wrapped angle.
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def compose_relative(x: float, y: float, yaw: float, dx: float, dy: float, dyaw: float) -> tuple[float, float, float]:
    """Apply a displacement expressed in the robot frame to a pose in a fixed frame.

    Args:
        x (float): Current x in the fixed frame.
        y (float): Current y in the fixed frame.
        yaw (float): Current yaw in the fixed frame.
        dx (float): Forward displacement (robot frame).
        dy (float): Leftward displacement (robot frame).
        dyaw (float): Yaw change.

    Returns:
        tuple[float, float, float]: Goal (x, y, yaw) in the fixed frame, yaw wrapped.
    """
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * dx - s * dy, y + s * dx + c * dy, normalize_angle(yaw + dyaw)


def clamp_twist(vx: float, vy: float, wz: float, max_linear: float, max_angular: float) -> tuple[float, float, float]:
    """Clamp each velocity component to its limit.

    Args:
        vx (float): Forward velocity (m/s).
        vy (float): Lateral velocity (m/s).
        wz (float): Yaw rate (rad/s).
        max_linear (float): Linear limit per axis (m/s).
        max_angular (float): Angular limit (rad/s).

    Returns:
        tuple[float, float, float]: Clamped (vx, vy, wz).
    """
    if not all(math.isfinite(v) for v in (vx, vy, wz)):
        raise ValueError("velocities must be finite")

    def clip(v: float, limit: float) -> float:
        return min(max(v, -limit), limit)

    return clip(vx, max_linear), clip(vy, max_linear), clip(wz, max_angular)


def integrate_twist(vx: float, vy: float, wz: float, duration_s: float) -> tuple[float, float, float]:
    """Displacement in the starting robot frame of a constant body twist held for duration_s.

    Args:
        vx (float): Forward velocity (m/s).
        vy (float): Leftward velocity (m/s).
        wz (float): Yaw rate (rad/s).
        duration_s (float): Duration (s).

    Returns:
        tuple[float, float, float]: (dx, dy, dyaw) in the starting frame (exact arc for wz != 0).
    """
    dyaw = wz * duration_s
    if abs(wz) < 1e-9:
        return vx * duration_s, vy * duration_s, 0.0
    sin, cos = math.sin(dyaw), math.cos(dyaw)
    return (vx * sin - vy * (1.0 - cos)) / wz, (vx * (1.0 - cos) + vy * sin) / wz, dyaw


def relative_pose(x0: float, y0: float, yaw0: float, x1: float, y1: float, yaw1: float) -> tuple[float, float, float]:
    """Pose 1 expressed in the frame of pose 0 (inverse of compose_relative).

    Args:
        x0 (float): Start x in the fixed frame.
        y0 (float): Start y.
        yaw0 (float): Start yaw.
        x1 (float): End x.
        y1 (float): End y.
        yaw1 (float): End yaw.

    Returns:
        tuple[float, float, float]: (dx, dy, dyaw) in the start robot frame, yaw wrapped.
    """
    c, s = math.cos(yaw0), math.sin(yaw0)
    ex, ey = x1 - x0, y1 - y0
    return c * ex + s * ey, -s * ex + c * ey, normalize_angle(yaw1 - yaw0)
