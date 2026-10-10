"""Body-frame twist from consecutive rf2o poses, and the diagonal twist covariance the EKF needs."""

import math
from dataclasses import dataclass

from .metrics import DT_REJECTED, MESSAGES

# Default upper bound on the time between two poses that still yields a twist, s.
DEFAULT_MAX_DT_S = 1.0
# nav_msgs/Odometry twist covariance is a row-major 6x6: x, y, z, roll, pitch, yaw.
COVARIANCE_SIZE = 36
VX_INDEX = 0
VY_INDEX = 7
VYAW_INDEX = 35


@dataclass(frozen=True)
class PoseSample:
    """Planar pose of base_link in the odometry frame.

    Attributes:
        x: Position x, m.
        y: Position y, m.
        yaw: Heading, rad.
        time_s: Message time, s.
    """

    x: float
    y: float
    yaw: float
    time_s: float


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi].

    Args:
        angle: Angle, rad.

    Returns:
        float: Equivalent angle in [-pi, pi].
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def body_twist(
    previous: PoseSample, current: PoseSample, max_dt_s: float = DEFAULT_MAX_DT_S
) -> tuple[float, float, float] | None:
    """Body-frame twist (vx, vy, yaw rate) between two poses.

    rf2o's own twist is a laser-frame x speed with vy fixed at 0 (and the lidar is mounted rotated 180 deg), so
    the twist is rebuilt from the pose difference, rotated into the body frame at the mid heading.

    Args:
        previous: Earlier pose.
        current: Later pose.
        max_dt_s: Largest usable time step, s.

    Returns:
        tuple[float, float, float] | None: (vx m/s, vy m/s, omega rad/s), or None when the time step is not in
        (0, max_dt_s].
    """
    dt = current.time_s - previous.time_s
    if dt <= 0.0 or dt > max_dt_s:
        return None
    dyaw = wrap_angle(current.yaw - previous.yaw)
    mid_yaw = previous.yaw + dyaw / 2.0
    dx = current.x - previous.x
    dy = current.y - previous.y
    cos_yaw = math.cos(mid_yaw)
    sin_yaw = math.sin(mid_yaw)
    return ((cos_yaw * dx + sin_yaw * dy) / dt, (-sin_yaw * dx + cos_yaw * dy) / dt, dyaw / dt)


def relay_twist(previous: PoseSample, current: PoseSample, max_dt_s: float) -> tuple[float, float, float] | None:
    """body_twist plus the relay counters: published messages and time-step rejections.

    Args:
        previous: Earlier pose.
        current: Later pose.
        max_dt_s: Largest usable time step, s.

    Returns:
        tuple[float, float, float] | None: Same as body_twist.
    """
    twist = body_twist(previous, current, max_dt_s)
    if twist is None:
        DT_REJECTED.inc()
    else:
        MESSAGES.inc()
    return twist


def twist_covariance(var_vx_vy: float, var_vyaw: float) -> list[float]:
    """Row-major 6x6 twist covariance with only vx, vy and yaw-rate variances set.

    Args:
        var_vx_vy: Variance of vx and vy, (m/s)^2.
        var_vyaw: Variance of the yaw rate, (rad/s)^2.

    Returns:
        list[float]: 36 values, zero except the three diagonal entries.
    """
    covariance = [0.0] * COVARIANCE_SIZE
    covariance[VX_INDEX] = var_vx_vy
    covariance[VY_INDEX] = var_vx_vy
    covariance[VYAW_INDEX] = var_vyaw
    return covariance
