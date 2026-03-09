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
    from adafruit_bno055 import BNO055_I2C, IMUPLUS_MODE

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
            # Adafruit breakout has external 32kHz crystal — required for stable sensor output
            bno.use_external_crystal = True
            # IMUPLUS: accel+gyro fusion — quaternion, gyro, linear_acceleration (no magnetometer)
            bno.mode = IMUPLUS_MODE
            time.sleep(0.5)  # allow fusion to stabilize
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

    def _all_zero(seq: tuple[float, ...], tol: float = 1e-9) -> bool:
        return all(abs(x) <= tol for x in seq)

    ORIENTATION_UNKNOWN_COV = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    while rclpy.ok():
        try:
            quat = bno.quaternion
            gyro = bno.gyro
            accel = bno.linear_acceleration
        except (RuntimeError, OSError) as e:
            node.get_logger().warn("BNO055 read error: %s" % e, throttle_duration_sec=5.0)
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(period_s)
            continue

        quat_vals = tuple(_coerce(quat[i]) for i in range(min(4, len(quat or [])))) if quat else (0.0, 0.0, 0.0, 0.0)
        gyro_vals = tuple(_coerce(gyro[i]) for i in range(min(3, len(gyro or [])))) if gyro else (0.0, 0.0, 0.0)
        accel_vals = tuple(_coerce(accel[i]) for i in range(min(3, len(accel or [])))) if accel else (0.0, 0.0, 0.0)
        while len(quat_vals) < 4:
            quat_vals = quat_vals + (0.0,)
        while len(gyro_vals) < 3:
            gyro_vals = gyro_vals + (0.0,)
        while len(accel_vals) < 3:
            accel_vals = accel_vals + (0.0,)

        # If fusion returns all zeros (before calibration), fall back to raw so data changes when moving
        use_raw = _all_zero(quat_vals) and _all_zero(gyro_vals) and _all_zero(accel_vals)
        if use_raw:
            try:
                raw_acc = bno.acceleration
                raw_gyro = bno.gyro
            except (RuntimeError, OSError):
                raw_acc = raw_gyro = None
            if raw_acc and raw_gyro and not (
                _all_zero(tuple(_coerce(raw_acc[i]) for i in range(min(3, len(raw_acc or [])))))
                and _all_zero(tuple(_coerce(raw_gyro[i]) for i in range(min(3, len(raw_gyro or [])))))
            ):
                gyro_vals = tuple(_coerce(raw_gyro[i]) for i in range(min(3, len(raw_gyro or []))))
                accel_vals = tuple(_coerce(raw_acc[i]) for i in range(min(3, len(raw_acc or []))))
                while len(gyro_vals) < 3:
                    gyro_vals = gyro_vals + (0.0,)
                while len(accel_vals) < 3:
                    accel_vals = accel_vals + (0.0,)
            quat_vals = (1.0, 0.0, 0.0, 0.0)

        stamp = clock.now().to_msg()
        quat_xyzw = quaternion_wxyz_to_xyzw(quat_vals[0], quat_vals[1], quat_vals[2], quat_vals[3])
        orient_cov = ORIENTATION_UNKNOWN_COV if use_raw else config.orientation_covariance
        msg = build_imu_message(
            stamp_sec=stamp.sec,
            stamp_nanosec=stamp.nanosec,
            frame_id=config.frame_id,
            quat_xyzw=quat_xyzw,
            angular_vel_xyz=gyro_vals,
            linear_accel_xyz=accel_vals,
            orientation_covariance=orient_cov,
            angular_velocity_covariance=config.angular_velocity_covariance,
            linear_acceleration_covariance=config.linear_acceleration_covariance,
        )
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.001)
        time.sleep(period_s)

    node.destroy_node()
    rclpy.shutdown()
