"""ROS2 BNO055 IMU node: read sensor over I2C, publish sensor_msgs/Imu with covariance."""

import time
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

from .config import ImuNodeConfig
from .imu_msg import build_imu_message, quaternion_wxyz_to_xyzw

I2C_RECONNECT_THRESHOLD = 10
WARMUP_TIMEOUT_S = 10.0
CRYSTAL_STABILIZE_S = 1.0  # BNO055 needs time after crystal switch before mode writes take effect
MODE_SWITCH_DELAY_S = 1.5  # Fusion needs time after mode switch to produce valid data
MODE_VERIFY_RETRIES = 3
IMUPLUS_MODE_VALUE = 0x08  # adafruit_bno055.IMUPLUS_MODE — kept here for testability without hardware libs

IMU_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

ORIENTATION_UNKNOWN_COV = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
IDENTITY_QUAT_WXYZ = (1.0, 0.0, 0.0, 0.0)


def coerce(v: Any, default: float = 0.0) -> float:
    """Return default if v is None, else float(v)."""
    return default if v is None else float(v)


def all_zero(seq: tuple[float, ...], tol: float = 1e-6) -> bool:
    """Return True if seq is non-empty and all elements are within tol of zero."""
    return seq and all(abs(x) <= tol for x in seq)


def has_valid_tuple(tup: tuple | None, min_len: int, allow_zeros: bool = False) -> bool:
    """Return True if tup is a tuple of at least min_len non-None elements (not all-zero unless allow_zeros)."""
    if not tup or len(tup) < min_len:
        return False
    if any(tup[i] is None for i in range(min(min_len, len(tup)))):
        return False
    vals = tuple(coerce(tup[i]) for i in range(min(min_len, len(tup))))
    if not allow_zeros and all_zero(vals):
        return False
    return True


def valid_quat(quat: tuple | None) -> bool:
    """Return True if quat is a valid unit quaternion (non-None, non-zero, norm in [0.9, 1.1])."""
    if not quat or len(quat) < 4 or any(quat[i] is None for i in range(4)):
        return False
    w, x, y, z = coerce(quat[0]), coerce(quat[1]), coerce(quat[2]), coerce(quat[3])
    if all_zero((w, x, y, z)):
        return False
    n2 = w * w + x * x + y * y + z * z
    return 0.9 <= n2 <= 1.1  # quaternion should be unit length


def warmup_check(bno: Any) -> bool:
    """Return True if bno sensor has valid gyro and acceleration readings."""
    try:
        g = bno.gyro
        a = bno.linear_acceleration
    except (RuntimeError, OSError):
        return False
    if not g or len(g) < 3 or any(g[i] is None for i in range(3)):
        return False
    if not a or len(a) < 3 or any(a[i] is None for i in range(3)):
        try:
            a = bno.acceleration
        except (RuntimeError, OSError):
            return False
    return bool(a and len(a) >= 3 and a[0] is not None and a[1] is not None and a[2] is not None)


def _spin_once_safe(node: Node, timeout_sec: float = 0.01) -> None:
    """Call rclpy.spin_once, silently ignoring errors from invalid RCL context."""
    try:
        rclpy.spin_once(node, timeout_sec=timeout_sec)
    except Exception:  # noqa: BLE001
        pass  # context may be invalid during shutdown or reconnect


def _warmup(bno: Any, node: Node, timeout_s: float = WARMUP_TIMEOUT_S) -> bool:
    """Poll sensor until valid gyro+accel or timeout.

    Args:
        bno: BNO055 driver instance.
        node: ROS2 node (for logging and rclpy.ok check).
        timeout_s: Maximum seconds to wait.

    Returns:
        bool: True if sensor produced valid data within timeout_s.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and rclpy.ok():
        if warmup_check(bno):
            node.get_logger().info("BNO055 warm-up complete, sensor ready")
            return True
        time.sleep(0.2)
        _spin_once_safe(node, timeout_sec=0.01)
    node.get_logger().warn(
        "BNO055 warm-up timed out (%.0f s); will retry each cycle. "
        "Check I2C, calibration, and keep sensor still for a few seconds." % timeout_s
    )
    return False


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
            bno.use_external_crystal = True
            # At low I2C speeds (e.g. 10 kHz), the BNO055 needs ~1 s after a crystal
            # switch before mode writes take effect. The Adafruit library only sleeps
            # 10 ms, so its internal mode restore silently fails, locking the sensor in
            # CONFIG mode (0x00). Every property read then returns (None, None, None).
            time.sleep(CRYSTAL_STABILIZE_S)
            # IMUPLUS: accel+gyro fusion — quaternion, gyro, linear_acceleration (no magnetometer)
            bno.mode = IMUPLUS_MODE
            # Fusion needs 1–2 s after mode switch to produce valid data.
            time.sleep(MODE_SWITCH_DELAY_S)
            # Verify mode actually set; at low I2C speeds mode writes can silently fail.
            for attempt in range(MODE_VERIFY_RETRIES):
                if bno.mode == IMUPLUS_MODE_VALUE:
                    break
                time.sleep(0.5 * (attempt + 1))
                bno.mode = IMUPLUS_MODE
                time.sleep(0.5)
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

    init_attempt = 0
    bno, used_addr = None, None
    while rclpy.ok() and bno is None:
        try:
            bno, used_addr = _create_bno055(config.i2c_bus, config.i2c_address)
        except Exception as e:  # noqa: BLE001
            init_attempt += 1
            backoff = min(30.0, 2 ** min(init_attempt, 5))
            node.get_logger().error(
                "BNO055 init failed (attempt %d): %s — retrying in %.0fs" % (init_attempt, e, backoff)
            )
            time.sleep(backoff)
            _spin_once_safe(node, timeout_sec=0.01)
    if bno is None:
        node.destroy_node()
        rclpy.shutdown()
        return

    actual_mode = bno.mode
    node.get_logger().info(
        "BNO055 IMU: mode=0x%02x, publishing %s at %.1f Hz (frame_id=%s, i2c=%d, address=0x%02x)"
        % (actual_mode, config.topic, config.publish_hz, config.frame_id, config.i2c_bus, used_addr)
    )
    if actual_mode != IMUPLUS_MODE_VALUE:
        node.get_logger().warn(
            "BNO055 mode mismatch after init: expected 0x08 (IMUPLUS), got 0x%02x"
            " — reads will return None" % actual_mode
        )
    try:
        sys_c, gyro_c, accel_c, mag_c = bno.calibration_status
        node.get_logger().info(
            "BNO055 calibration (sys, gyro, accel, mag): %d, %d, %d, %d (0=uncal, 3=full)"
            % (sys_c, gyro_c, accel_c, mag_c)
        )
    except Exception as e:  # noqa: BLE001
        node.get_logger().debug("BNO055 calibration status unavailable: %s" % e)

    _warmup(bno, node)

    consecutive_failures: int = 0
    hard_failures: int = 0  # OSError/RuntimeError — tracks true I2C bus errors
    reconnect_count: int = 0  # increments on each full reconnect; reset only on successful publish
    while rclpy.ok():
        if consecutive_failures >= I2C_RECONNECT_THRESHOLD:
            # First try a soft mode restore: just re-write the mode register without
            # touching use_external_crystal.  At slow I2C speeds (e.g. 10 kHz), the
            # crystal-switch sequence in _create_bno055 can corrupt the bus; a simple
            # mode write is much safer for transient mode-register read failures.
            soft_ok = False
            if hard_failures < I2C_RECONNECT_THRESHOLD:
                try:
                    from adafruit_bno055 import IMUPLUS_MODE

                    bno.mode = IMUPLUS_MODE
                    time.sleep(MODE_SWITCH_DELAY_S)
                    if bno.mode == IMUPLUS_MODE_VALUE:
                        consecutive_failures = 0
                        hard_failures = 0
                        soft_ok = True
                        node.get_logger().info("BNO055 soft mode restore succeeded")
                        _warmup(bno, node)
                except Exception:  # noqa: BLE001
                    pass
            if soft_ok:
                _spin_once_safe(node, timeout_sec=0.01)
                time.sleep(period_s)
                continue

            node.get_logger().warn("BNO055: %d consecutive failures — attempting I2C reconnect" % consecutive_failures)
            # Backoff grows with reconnect_count to slow down repeated reconnect loops.
            backoff = min(30.0, 2 ** min(reconnect_count, 5))
            time.sleep(backoff)
            try:
                bno, used_addr = _create_bno055(config.i2c_bus, config.i2c_address)
                consecutive_failures = 0
                hard_failures = 0
                reconnect_count += 1
                reconnect_mode = bno.mode
                node.get_logger().info(
                    "BNO055 reconnected on 0x%02x, mode=0x%02x (reconnect #%d)"
                    % (used_addr, reconnect_mode, reconnect_count)
                )
                if reconnect_mode != 0x08:  # IMUPLUS_MODE
                    node.get_logger().warn(
                        "BNO055 mode mismatch after reconnect: expected 0x08, got 0x%02x" % reconnect_mode
                    )
                _warmup(bno, node)
            except Exception as e:  # noqa: BLE001
                node.get_logger().error("BNO055 reconnect failed: %s" % e)
                # Don't reset counter — will retry again after threshold
            _spin_once_safe(node, timeout_sec=0.01)
            time.sleep(period_s)
            continue

        quat, gyro, accel = None, None, None
        for _ in range(5):
            try:
                quat = bno.quaternion
                gyro = bno.gyro
                accel = bno.linear_acceleration
            except (RuntimeError, OSError) as e:
                node.get_logger().warn("BNO055 read error: %s" % e, throttle_duration_sec=5.0)
                quat = gyro = accel = None
                hard_failures += 1
                break
            has_gyro = has_valid_tuple(gyro, 3, allow_zeros=True)
            has_accel = has_valid_tuple(accel, 3, allow_zeros=True)
            if has_gyro and has_accel:
                break
            time.sleep(0.01)
        else:
            consecutive_failures += 1
            node.get_logger().warn(
                "BNO055 no valid gyro/accel after 5 retries (gyro=%r, accel=%r); skipping publish" % (gyro, accel),
                throttle_duration_sec=5.0,
            )
            _spin_once_safe(node, timeout_sec=0.01)
            time.sleep(period_s)
            continue

        if not has_valid_tuple(accel, 3, allow_zeros=True):
            try:
                accel = bno.acceleration
            except (RuntimeError, OSError):
                pass
        if not has_valid_tuple(gyro, 3, allow_zeros=True) or not has_valid_tuple(accel, 3, allow_zeros=True):
            consecutive_failures += 1
            node.get_logger().warn(
                "BNO055 accel fallback still invalid (gyro_ok=%s, accel_ok=%s); skipping publish"
                % (has_valid_tuple(gyro, 3, allow_zeros=True), has_valid_tuple(accel, 3, allow_zeros=True)),
                throttle_duration_sec=5.0,
            )
            _spin_once_safe(node, timeout_sec=0.01)
            time.sleep(period_s)
            continue

        consecutive_failures = 0
        hard_failures = 0
        reconnect_count = 0  # successful publish — backoff resets
        gyro_vals = tuple(coerce(gyro[i]) for i in range(min(3, len(gyro or [])))) if gyro else (0.0, 0.0, 0.0)
        accel_vals = tuple(coerce(accel[i]) for i in range(min(3, len(accel or [])))) if accel else (0.0, 0.0, 0.0)
        while len(gyro_vals) < 3:
            gyro_vals = gyro_vals + (0.0,)
        while len(accel_vals) < 3:
            accel_vals = accel_vals + (0.0,)

        if valid_quat(quat):
            quat_vals = tuple(coerce(quat[i]) for i in range(4)) if quat else IDENTITY_QUAT_WXYZ
            orient_cov = config.orientation_covariance
        else:
            quat_vals = IDENTITY_QUAT_WXYZ
            orient_cov = ORIENTATION_UNKNOWN_COV

        stamp = clock.now().to_msg()
        quat_xyzw = quaternion_wxyz_to_xyzw(quat_vals[0], quat_vals[1], quat_vals[2], quat_vals[3])
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
        _spin_once_safe(node, timeout_sec=0.001)
        time.sleep(period_s)

    node.destroy_node()
    rclpy.shutdown()
