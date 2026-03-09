"""ROS2 BNO055 IMU node: read sensor over I2C, publish sensor_msgs/Imu with covariance."""

import time
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

from .config import ImuNodeConfig
from .imu_msg import build_imu_message, quaternion_wxyz_to_xyzw

IMU_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def _create_i2c(i2c_bus: int) -> Any:
    """Create I2C bus object for Blinka-based drivers."""
    try:
        from adafruit_extended_bus import ExtendedI2C as I2C

        return I2C(i2c_bus)
    except ImportError:
        import board
        import busio

        return busio.I2C(board.SCL, board.SDA)


def _create_bno055(i2c_bus: int, i2c_address: int) -> tuple[Any, int]:
    """Create BNO055 I2C driver with configured address and fallback to alternate address."""
    from adafruit_bno055 import BNO055_I2C, ACCGYRO_MODE

    tried: list[tuple[int, Exception]] = []
    addresses: list[int] = [i2c_address]
    if i2c_address != 0x28:
        addresses.append(0x28)
    if i2c_address != 0x29:
        addresses.append(0x29)
    for addr in addresses:
        try:
            i2c = _create_i2c(i2c_bus)
            bno = BNO055_I2C(i2c, address=addr)
            bno.use_external_crystal = True
            # ACCGYRO: raw accel+gyro only — no fusion, always returns real data (incl. gravity ~9.8)
            bno.mode = ACCGYRO_MODE
            time.sleep(0.1)
            return bno, addr
        except Exception as exc:  # noqa: BLE001
            tried.append((addr, exc))
    tried_desc = ", ".join([f"0x{addr:02x} ({type(exc).__name__})" for addr, exc in tried])
    raise RuntimeError(f"Unable to initialize BNO055 on i2c-{i2c_bus}; tried {tried_desc}")


def run_imu_node(config: ImuNodeConfig) -> None:
    """Run the IMU node: read BNO055, publish Imu at config.publish_hz."""
    rclpy.init()
    node = Node("bno055_imu")
    pub = node.create_publisher(Imu, config.topic, IMU_QOS)
    clock = node.get_clock()
    period_s = 1.0 / max(1.0, config.publish_hz)

    try:
        bno, used_addr = _create_bno055(config.i2c_bus, config.i2c_address)
    except Exception as e:
        node.get_logger().error("BNO055 init failed: %s" % e)
        node.destroy_node()
        rclpy.shutdown()
        raise

    node.get_logger().info(
        "BNO055 IMU: publishing %s at %.1f Hz (frame_id=%s, i2c=%d, address=0x%02x)"
        % (config.topic, config.publish_hz, config.frame_id, config.i2c_bus, used_addr)
    )
    try:
        sys_c, gyro_c, accel_c, mag_c = bno.calibration_status
        node.get_logger().info(
            "BNO055 calibration (sys, gyro, accel, mag): %d, %d, %d, %d (0=uncal, 3=full)"
            % (sys_c, gyro_c, accel_c, mag_c)
        )
    except Exception:  # noqa: S110
        pass

    def _coerce(v: Any, default: float = 0.0) -> float:
        return default if v is None else float(v)

    ORIENTATION_UNKNOWN_COV = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    IDENTITY_QUAT_WXYZ = (1.0, 0.0, 0.0, 0.0)

    def _all_zero(seq: tuple[float, ...], tol: float = 1e-6) -> bool:
        return seq and all(abs(x) <= tol for x in seq)

    def _has_valid_data(acc: tuple | None, gyro: tuple | None) -> bool:
        if not acc or not gyro:
            return False
        av = tuple(_coerce(acc[i]) for i in range(min(3, len(acc))))
        gv = tuple(_coerce(gyro[i]) for i in range(min(3, len(gyro))))
        if _all_zero(av) and _all_zero(gv):
            return False
        return True

    while rclpy.ok():
        raw_acc, raw_gyro = None, None
        for _ in range(5):
            try:
                raw_acc = bno.acceleration
                raw_gyro = bno.gyro
            except (RuntimeError, OSError) as e:
                node.get_logger().warn("BNO055 read error: %s" % e, throttle_duration_sec=5.0)
                break
            if _has_valid_data(raw_acc, raw_gyro):
                break
            time.sleep(0.01)
        else:
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(period_s)
            continue

        gyro_vals = tuple(_coerce(raw_gyro[i]) for i in range(min(3, len(raw_gyro or [])))) if raw_gyro else (0.0, 0.0, 0.0)
        accel_vals = tuple(_coerce(raw_acc[i]) for i in range(min(3, len(raw_acc or [])))) if raw_acc else (0.0, 0.0, 0.0)
        while len(gyro_vals) < 3:
            gyro_vals = gyro_vals + (0.0,)
        while len(accel_vals) < 3:
            accel_vals = accel_vals + (0.0,)

        stamp = clock.now().to_msg()
        quat_xyzw = quaternion_wxyz_to_xyzw(
            IDENTITY_QUAT_WXYZ[0], IDENTITY_QUAT_WXYZ[1], IDENTITY_QUAT_WXYZ[2], IDENTITY_QUAT_WXYZ[3]
        )
        msg = build_imu_message(
            stamp_sec=stamp.sec,
            stamp_nanosec=stamp.nanosec,
            frame_id=config.frame_id,
            quat_xyzw=quat_xyzw,
            angular_vel_xyz=gyro_vals,
            linear_accel_xyz=accel_vals,
            orientation_covariance=ORIENTATION_UNKNOWN_COV,
            angular_velocity_covariance=config.angular_velocity_covariance,
            linear_acceleration_covariance=config.linear_acceleration_covariance,
        )
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.001)
        time.sleep(period_s)

    node.destroy_node()
    rclpy.shutdown()
