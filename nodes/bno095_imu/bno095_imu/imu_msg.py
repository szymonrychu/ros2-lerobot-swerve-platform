"""Build sensor_msgs/Imu from BNO08x raw readings and config (covariances, units)."""

from __future__ import annotations

from typing import Any


def quaternion_ijkr_to_xyzw(i: float, j: float, k: float, r: float) -> tuple[float, float, float, float]:
    """Convert BNO08x quaternion (i, j, k, real) to ROS (x, y, z, w).

    Args:
        i, j, k, r: BNO08x quaternion components.

    Returns:
        tuple[float, float, float, float]: (x, y, z, w) for geometry_msgs/Quaternion.
    """
    return (float(i), float(j), float(k), float(r))


def build_imu_message(
    stamp_sec: float,
    stamp_nanosec: int,
    frame_id: str,
    quat_xyzw: tuple[float, float, float, float],
    angular_vel_xyz: tuple[float, float, float],
    linear_accel_xyz: tuple[float, float, float],
    orientation_covariance: list[float],
    angular_velocity_covariance: list[float],
    linear_acceleration_covariance: list[float],
) -> Any:
    """Build a sensor_msgs/Imu message from components (ROS units: rad/s, m/s²).

    Orientation and covariances are filled; linear_acceleration in m/s², angular_velocity in rad/s.

    Args:
        stamp_sec: Header stamp seconds.
        stamp_nanosec: Header stamp nanoseconds.
        frame_id: Header frame_id.
        quat_xyzw: Orientation quaternion (x, y, z, w).
        angular_vel_xyz: Angular velocity (x, y, z) in rad/s.
        linear_accel_xyz: Linear acceleration (x, y, z) in m/s².
        orientation_covariance: 9-element row-major (use -1 in [0] for unknown).
        angular_velocity_covariance: 9-element row-major.
        linear_acceleration_covariance: 9-element row-major.

    Returns:
        Imu message (sensor_msgs/Imu).
    """
    from builtin_interfaces.msg import Time
    from sensor_msgs.msg import Imu

    msg = Imu()
    msg.header.stamp = Time(sec=int(stamp_sec), nanosec=stamp_nanosec)
    msg.header.frame_id = frame_id
    msg.orientation.x = float(quat_xyzw[0])
    msg.orientation.y = float(quat_xyzw[1])
    msg.orientation.z = float(quat_xyzw[2])
    msg.orientation.w = float(quat_xyzw[3])
    msg.orientation_covariance = list(orientation_covariance)
    msg.angular_velocity.x = float(angular_vel_xyz[0])
    msg.angular_velocity.y = float(angular_vel_xyz[1])
    msg.angular_velocity.z = float(angular_vel_xyz[2])
    msg.angular_velocity_covariance = list(angular_velocity_covariance)
    msg.linear_acceleration.x = float(linear_accel_xyz[0])
    msg.linear_acceleration.y = float(linear_accel_xyz[1])
    msg.linear_acceleration.z = float(linear_accel_xyz[2])
    msg.linear_acceleration_covariance = list(linear_acceleration_covariance)
    return msg
