"""Unit tests for IMU message mapping (quaternion, units, covariance)."""

import pytest

from bno095_imu.imu_msg import quaternion_ijkr_to_xyzw


def test_quaternion_ijkr_to_xyzw_identity() -> None:
    """BNO (i,j,k,r) maps to ROS (x,y,z,w) in order."""
    x, y, z, w = quaternion_ijkr_to_xyzw(0.0, 0.0, 0.0, 1.0)
    assert (x, y, z, w) == (0.0, 0.0, 0.0, 1.0)


def test_quaternion_ijkr_to_xyzw_values() -> None:
    """Arbitrary (i,j,k,r) preserved as (x,y,z,w)."""
    x, y, z, w = quaternion_ijkr_to_xyzw(0.1, 0.2, 0.3, 0.9)
    assert (x, y, z, w) == (0.1, 0.2, 0.3, 0.9)


def test_build_imu_message_units_and_covariance() -> None:
    """build_imu_message produces Imu with correct units and covariance arrays."""
    pytest.importorskip("sensor_msgs.msg", reason="sensor_msgs not available (no ROS)")
    pytest.importorskip("builtin_interfaces.msg", reason="builtin_interfaces not available")
    from bno095_imu.imu_msg import build_imu_message

    stamp_sec = 100
    stamp_nanosec = 500000000
    frame_id = "imu_link"
    quat_xyzw = (0.0, 0.0, 0.0, 1.0)
    angular_vel_xyz = (0.1, 0.2, 0.3)  # rad/s
    linear_accel_xyz = (0.0, 0.0, 9.81)  # m/s²
    ori_cov = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
    ang_cov = [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.02]
    acc_cov = [0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04]

    msg = build_imu_message(
        stamp_sec=stamp_sec,
        stamp_nanosec=stamp_nanosec,
        frame_id=frame_id,
        quat_xyzw=quat_xyzw,
        angular_vel_xyz=angular_vel_xyz,
        linear_accel_xyz=linear_accel_xyz,
        orientation_covariance=ori_cov,
        angular_velocity_covariance=ang_cov,
        linear_acceleration_covariance=acc_cov,
    )

    assert msg.header.stamp.sec == 100
    assert msg.header.stamp.nanosec == 500000000
    assert msg.header.frame_id == "imu_link"
    assert msg.orientation.x == 0.0 and msg.orientation.w == 1.0
    assert msg.angular_velocity.x == 0.1 and msg.angular_velocity.z == 0.3
    assert msg.linear_acceleration.z == pytest.approx(9.81)
    assert list(msg.orientation_covariance) == ori_cov
    assert list(msg.angular_velocity_covariance) == ang_cov
    assert list(msg.linear_acceleration_covariance) == acc_cov


def test_build_imu_message_unknown_orientation_covariance() -> None:
    """Orientation covariance can use -1 in first element for 'unknown'."""
    pytest.importorskip("sensor_msgs.msg", reason="sensor_msgs not available (no ROS)")
    pytest.importorskip("builtin_interfaces.msg", reason="builtin_interfaces not available")
    from bno095_imu.imu_msg import build_imu_message

    ori_cov = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    msg = build_imu_message(
        stamp_sec=0,
        stamp_nanosec=0,
        frame_id="",
        quat_xyzw=(0, 0, 0, 1),
        angular_vel_xyz=(0, 0, 0),
        linear_accel_xyz=(0, 0, 0),
        orientation_covariance=ori_cov,
        angular_velocity_covariance=[0.0] * 9,
        linear_acceleration_covariance=[0.0] * 9,
    )
    assert msg.orientation_covariance[0] == -1.0
