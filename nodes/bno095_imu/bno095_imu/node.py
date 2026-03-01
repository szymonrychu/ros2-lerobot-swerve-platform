"""ROS2 BNO095 IMU node: read sensor over I2C, publish sensor_msgs/Imu with covariance."""

import time
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

from .config import ImuNodeConfig
from .imu_msg import build_imu_message, quaternion_ijkr_to_xyzw

IMU_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# Report interval in microseconds for BNO08x (max report rate ~100 Hz for rotation vector).
BNO_REPORT_INTERVAL_US = 10000


def _create_i2c(i2c_bus: int) -> Any:
    """Create I2C bus object for Blinka-based drivers."""
    try:
        from adafruit_extended_bus import ExtendedI2C as I2C

        return I2C(i2c_bus)
    except ImportError:
        import board
        import busio

        return busio.I2C(board.SCL, board.SDA)


def _create_bno08x(i2c_bus: int, i2c_address: int) -> tuple[Any, int]:
    """Create BNO08x I2C driver with configured address and fallback to alternate BNO address."""
    from adafruit_bno08x import BNO_REPORT_GYROSCOPE, BNO_REPORT_LINEAR_ACCELERATION, BNO_REPORT_ROTATION_VECTOR
    from adafruit_bno08x.i2c import BNO08X_I2C

    tried: list[tuple[int, Exception]] = []
    addresses: list[int] = [i2c_address]
    if i2c_address != 0x4A:
        addresses.append(0x4A)
    if i2c_address != 0x4B:
        addresses.append(0x4B)
    for addr in addresses:
        try:
            i2c = _create_i2c(i2c_bus)
            bno = BNO08X_I2C(i2c, address=addr)
            bno.enable_feature(BNO_REPORT_ROTATION_VECTOR, BNO_REPORT_INTERVAL_US)
            bno.enable_feature(BNO_REPORT_GYROSCOPE, BNO_REPORT_INTERVAL_US)
            bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION, BNO_REPORT_INTERVAL_US)
            return bno, addr
        except Exception as exc:  # noqa: BLE001
            tried.append((addr, exc))
    tried_desc = ", ".join([f"0x{addr:02x} ({type(exc).__name__})" for addr, exc in tried])
    raise RuntimeError(f"Unable to initialize BNO08x on i2c-{i2c_bus}; tried {tried_desc}")


def run_imu_node(config: ImuNodeConfig) -> None:
    """Run the IMU node: read BNO095, publish Imu at config.publish_hz."""
    rclpy.init()
    node = Node("bno095_imu")
    pub = node.create_publisher(Imu, config.topic, IMU_QOS)
    clock = node.get_clock()
    period_s = 1.0 / max(1.0, config.publish_hz)

    try:
        bno, used_addr = _create_bno08x(config.i2c_bus, config.i2c_address)
    except Exception as e:
        node.get_logger().error("BNO095 init failed: %s" % e)
        node.destroy_node()
        rclpy.shutdown()
        raise

    node.get_logger().info(
        "BNO095 IMU: publishing %s at %.1f Hz (frame_id=%s, i2c=%d, address=0x%02x)"
        % (config.topic, config.publish_hz, config.frame_id, config.i2c_bus, used_addr)
    )

    while rclpy.ok():
        try:
            quat = bno.quaternion
            gyro = bno.gyro
            accel = bno.linear_acceleration
        except (RuntimeError, OSError) as e:
            node.get_logger().warn("BNO095 read error: %s" % e, throttle_duration_sec=5.0)
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(period_s)
            continue
        if quat is None or gyro is None or accel is None:
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(period_s)
            continue
        stamp = clock.now().to_msg()
        quat_xyzw = quaternion_ijkr_to_xyzw(quat[0], quat[1], quat[2], quat[3])
        msg = build_imu_message(
            stamp_sec=stamp.sec,
            stamp_nanosec=stamp.nanosec,
            frame_id=config.frame_id,
            quat_xyzw=quat_xyzw,
            angular_vel_xyz=(gyro[0], gyro[1], gyro[2]),
            linear_accel_xyz=(accel[0], accel[1], accel[2]),
            orientation_covariance=config.orientation_covariance,
            angular_velocity_covariance=config.angular_velocity_covariance,
            linear_acceleration_covariance=config.linear_acceleration_covariance,
        )
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.001)
        time.sleep(period_s)

    node.destroy_node()
    rclpy.shutdown()
