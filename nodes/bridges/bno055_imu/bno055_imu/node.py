"""ROS2 BNO055 IMU node: read sensor over I2C, publish sensor_msgs/Imu with covariance."""

import json
import time
from pathlib import Path
from typing import Any

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from ros2_metrics import resolve_metrics_port, start_metrics_server
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from . import metrics
from .calibration import CalibrationProfile, apply_profile, load_profile, save_if_changed, should_save
from .config import ImuNodeConfig
from .covariance import CovarianceEstimator
from .imu_msg import build_imu_message, quaternion_wxyz_to_xyzw
from .recovery import Action, RecoveryPolicy

I2C_RECONNECT_THRESHOLD = 10
WARMUP_TIMEOUT_S = 10.0
CRYSTAL_STABILIZE_S = 1.0  # BNO055 needs time after crystal switch before mode writes take effect
MODE_SWITCH_DELAY_S = 1.5  # Fusion needs time after mode switch to produce valid data
MODE_VERIFY_RETRIES = 3
SYS_TRIGGER_REGISTER = 0x3F
SYS_TRIGGER_RST_SYS = 0x20  # reset the whole chip; it reboots into CONFIG mode
CHIP_RESET_WAIT_S = 0.65  # BNO055 boot time after RST_SYS
IMUPLUS_MODE_VALUE = 0x08  # adafruit_bno055.IMUPLUS_MODE — kept here for testability without hardware libs
NDOF_FMC_OFF_MODE_VALUE = 0x0B
NDOF_MODE_VALUE = 0x0C
OPERATION_MODE_VALUES = {
    "IMUPLUS": IMUPLUS_MODE_VALUE,
    "NDOF_FMC_OFF": NDOF_FMC_OFF_MODE_VALUE,
    "NDOF": NDOF_MODE_VALUE,
}
CALIBRATION_KEYS = ("sys", "gyro", "accel", "mag")
CALIBRATION_PUBLISH_PERIOD_S = 1.0

NODE_NAME = "bno055_imu"

IMU_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

ORIENTATION_UNKNOWN_COV = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
IDENTITY_QUAT_WXYZ = (1.0, 0.0, 0.0, 0.0)

DEG_TO_RAD = 0.017453292519943295  # math.pi / 180


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


def mode_value(name: str) -> int:
    """Return the BNO055 OPR_MODE register value for a supported operation mode name.

    Args:
        name (str): One of IMUPLUS, NDOF, NDOF_FMC_OFF.

    Returns:
        int: Register value.

    Raises:
        ValueError: If name is not a supported mode.
    """
    if name not in OPERATION_MODE_VALUES:
        raise ValueError(f"unsupported BNO055 operation mode: {name!r}")
    return OPERATION_MODE_VALUES[name]


def calibration_payload(status: tuple | None, restored: bool = False) -> str | None:
    """Encode the BNO055 calibration status as JSON.

    Args:
        status (tuple | None): (sys, gyro, accel, mag), each 0 (uncalibrated) to 3 (fully calibrated).
        restored (bool): True when the saved calibration profile was written to the chip at init; the chip then
            reports 0 until it re-checks itself, although the restored offsets are already in use.

    Returns:
        str | None: JSON {"sys", "gyro", "accel", "mag", "restored"}, or None when the status is unavailable or
            incomplete.
    """
    if not status or len(status) < len(CALIBRATION_KEYS) or any(v is None for v in status[:4]):
        return None
    payload: dict[str, int | bool] = {key: int(status[i]) for i, key in enumerate(CALIBRATION_KEYS)}
    payload["restored"] = restored
    return json.dumps(payload)


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


def _spin_once_safe(executor: SingleThreadedExecutor, timeout_sec: float = 0.01) -> None:
    """Call executor.spin_once, silently ignoring errors from invalid RCL context."""
    try:
        executor.spin_once(timeout_sec=timeout_sec)
    except Exception:  # noqa: BLE001
        pass  # context may be invalid during shutdown or reconnect


def _warmup(bno: Any, node: Node, executor: SingleThreadedExecutor, timeout_s: float = WARMUP_TIMEOUT_S) -> bool:
    """Poll sensor until valid gyro+accel or timeout.

    Args:
        bno: BNO055 driver instance.
        node: ROS2 node (for logging and rclpy.ok check).
        executor: Persistent executor for spinning callbacks during warmup.
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
        _spin_once_safe(executor, timeout_sec=0.01)
    node.get_logger().warn(
        f"BNO055 warm-up timed out ({timeout_s:.0f} s); will retry each cycle. "
        "Check I2C, calibration, and keep sensor still for a few seconds."
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


def load_saved_profile(config: ImuNodeConfig, node: Node) -> CalibrationProfile | None:
    """Load the persisted calibration profile; a missing, corrupt or invalid file means uncalibrated.

    Args:
        config: Node config (calibration_file; None disables persistence).
        node: ROS2 node for logging.

    Returns:
        CalibrationProfile | None: The valid profile, or None.
    """
    if not config.calibration_file:
        return None
    try:
        return load_profile(Path(config.calibration_file))
    except ValueError as exc:
        node.get_logger().warn(f"ignoring calibration profile {config.calibration_file}: {exc}; starting uncalibrated")
        return None


def _create_bno055(
    i2c_bus: int, i2c_address: int, operation_mode: str = "IMUPLUS", profile: CalibrationProfile | None = None
) -> tuple[Any, int]:
    """Create BNO055 I2C driver with configured address and fallback to alternate address.

    Args:
        i2c_bus (int): I2C bus number.
        i2c_address (int): Preferred I2C address.
        operation_mode (str): IMUPLUS, NDOF or NDOF_FMC_OFF.
        profile (CalibrationProfile | None): Calibration offsets written in CONFIG mode before the operation mode.

    Returns:
        tuple[Any, int]: (driver, address used).
    """
    from adafruit_bno055 import BNO055_I2C

    target_mode = mode_value(operation_mode)

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
            if profile is not None:
                apply_profile(bno, profile)
            # IMUPLUS: accel+gyro fusion (no magnetometer); NDOF adds the magnetometer for absolute heading.
            bno.mode = target_mode
            # Fusion needs 1–2 s after mode switch to produce valid data.
            time.sleep(MODE_SWITCH_DELAY_S)
            # Verify mode actually set; at low I2C speeds mode writes can silently fail.
            for attempt in range(MODE_VERIFY_RETRIES):
                if bno.mode == target_mode:
                    break
                time.sleep(0.5 * (attempt + 1))
                bno.mode = target_mode
                time.sleep(0.5)
            return bno, addr
        except Exception as exc:  # noqa: BLE001
            tried.append((addr, exc))
    tried_desc = ", ".join([f"0x{addr:02x} ({type(exc).__name__})" for addr, exc in tried])
    raise RuntimeError(f"Unable to initialize BNO055 on i2c-{i2c_bus}; tried {tried_desc}")


def chip_reset(bno: Any) -> bool:
    """Reset the BNO055 via SYS_TRIGGER RST_SYS and wait for it to reboot; never raises.

    Args:
        bno (Any): Driver instance exposing _write_register (adafruit_bno055.BNO055_I2C).

    Returns:
        bool: True if the reset was issued, False if the driver lacks _write_register or the write failed.
    """
    try:
        bno._write_register(SYS_TRIGGER_REGISTER, SYS_TRIGGER_RST_SYS)
    except Exception:  # noqa: BLE001
        return False
    time.sleep(CHIP_RESET_WAIT_S)
    return True


def release_driver(bno: Any) -> None:
    """Release the I2C bus object of a driver being replaced; never raises."""
    try:
        bno.i2c_device.i2c.deinit()
    except Exception:  # noqa: BLE001
        pass


def make_estimators(config: ImuNodeConfig) -> tuple[CovarianceEstimator | None, ...]:
    """Build fresh (orientation, gyro, accel) covariance estimators, or Nones when estimation is disabled."""
    if not config.compute_covariance:
        return (None, None, None)
    return tuple(CovarianceEstimator(config.covariance_window, config.covariance_min_samples) for _ in range(3))


def create_sensor(config: ImuNodeConfig, node: Node) -> tuple[Any, int, CalibrationProfile | None]:
    """Create the I2C bus and driver (crystal, saved calibration, operation mode set and verified).

    Args:
        config (ImuNodeConfig): Node config.
        node (Node): ROS2 node for logging.

    Returns:
        tuple[Any, int, CalibrationProfile | None]: (driver, address used, restored profile or None).

    Raises:
        RuntimeError: If no address answers.
    """
    profile = load_saved_profile(config, node)
    bno, used_addr = _create_bno055(config.i2c_bus, config.i2c_address, config.operation_mode, profile)
    if profile is not None:
        node.get_logger().info(f"restored calibration profile from {config.calibration_file}")
    return bno, used_addr, profile


def run_imu_node(config: ImuNodeConfig) -> None:
    """Run the IMU node: read BNO055, publish Imu at config.publish_hz."""
    rclpy.init()
    node = Node(NODE_NAME)
    start_metrics_server(resolve_metrics_port(config.metrics_port), NODE_NAME)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    target_mode = mode_value(config.operation_mode)
    pub = node.create_publisher(Imu, config.topic, IMU_QOS)
    calibration_pub = (
        node.create_publisher(String, config.calibration_topic, IMU_QOS) if config.calibration_topic else None
    )
    last_calibration_pub_s = 0.0
    clock = node.get_clock()
    period_s = 1.0 / max(1.0, config.publish_hz)

    calibration_path = Path(config.calibration_file) if config.calibration_file else None
    saved_profile: CalibrationProfile | None = None
    last_save_s = time.monotonic()

    init_attempt = 0
    bno, used_addr = None, None
    while rclpy.ok() and bno is None:
        metrics.record_init_attempt()
        try:
            bno, used_addr, profile = create_sensor(config, node)
            if profile is not None:
                saved_profile = profile
        except Exception as e:  # noqa: BLE001
            init_attempt += 1
            backoff = min(30.0, 2 ** min(init_attempt, 5))
            node.get_logger().error(f"BNO055 init failed (attempt {init_attempt}): {e} — retrying in {backoff:.0f}s")
            time.sleep(backoff)
            _spin_once_safe(executor, timeout_sec=0.01)
    if bno is None:
        node.destroy_node()
        rclpy.shutdown()
        return

    actual_mode = bno.mode
    metrics.record_mode(actual_mode)
    node.get_logger().info(
        f"BNO055 IMU: mode=0x{actual_mode:02x}, publishing {config.topic} at {config.publish_hz:.1f} Hz "
        f"(frame_id={config.frame_id}, i2c={config.i2c_bus}, address=0x{used_addr:02x})"
    )
    if actual_mode != target_mode:
        node.get_logger().warn(
            f"BNO055 mode mismatch after init: expected 0x{target_mode:02x} ({config.operation_mode}), got 0x{actual_mode:02x} — reads will return None"
        )
    try:
        sys_c, gyro_c, accel_c, mag_c = bno.calibration_status
        metrics.record_calibration((sys_c, gyro_c, accel_c, mag_c))
        node.get_logger().info(
            f"BNO055 calibration (sys, gyro, accel, mag): {sys_c}, {gyro_c}, {accel_c}, {mag_c} (0=uncal, 3=full)"
        )
    except Exception as e:  # noqa: BLE001
        node.get_logger().debug(f"BNO055 calibration status unavailable: {e}")

    _warmup(bno, node, executor)

    # Optional rolling-window covariance estimation (disabled by default).
    orient_est, gyro_est, accel_est = make_estimators(config)
    if config.compute_covariance:
        node.get_logger().info(
            f"BNO055: real covariance estimation enabled "
            f"(window={config.covariance_window}, min_samples={config.covariance_min_samples})"
        )

    policy = RecoveryPolicy(config.max_soft_restores, config.reinit_after_s)
    consecutive_failures: int = 0
    hard_failures: int = 0  # OSError/RuntimeError - tracks true I2C bus errors
    total_reinits: int = 0
    while rclpy.ok():
        decision = policy.decide(
            recovery_needed=consecutive_failures >= I2C_RECONNECT_THRESHOLD,
            soft_possible=hard_failures < I2C_RECONNECT_THRESHOLD,
        )
        metrics.record_decision(decision)
        if decision.action is Action.SOFT_RESTORE:
            # Re-write the mode register only (no crystal switch: at slow I2C speeds that sequence can corrupt the
            # bus). Do not read the mode back, the read can fail with the same corruption. This does not clear the
            # policy's escalation state; only a published sample does.
            try:
                bno.mode = target_mode
                time.sleep(MODE_SWITCH_DELAY_S)
                node.get_logger().info(
                    f"BNO055 soft mode restore {policy.soft_restores}/{policy.max_soft_restores} (no bus reinit)"
                )
                consecutive_failures = 0
                hard_failures = 0
                _warmup(bno, node, executor)
            except Exception as e:  # noqa: BLE001
                hard_failures += 1
                node.get_logger().warn(f"BNO055 soft mode restore failed: {e}")
            _spin_once_safe(executor, timeout_sec=period_s)
            continue
        if decision.action is Action.FULL_REINIT:
            total_reinits += 1
            backoff = policy.backoff_s()
            node.get_logger().warn(
                f"BNO055 full re-init (reinit #{total_reinits}): {decision.reason}; backing off {backoff:.0f}s"
            )
            time.sleep(backoff)
            reinit_ok = False
            try:
                chip_reset(bno)
                release_driver(bno)
                metrics.record_init_attempt()
                bno, used_addr, profile = create_sensor(config, node)
                if profile is not None:
                    saved_profile = profile
                consecutive_failures = 0
                hard_failures = 0
                reinit_ok = True
                reinit_mode = bno.mode
                metrics.record_mode(reinit_mode)
                node.get_logger().info(
                    f"BNO055 re-initialised on 0x{used_addr:02x}, mode=0x{reinit_mode:02x} (reinit #{total_reinits})"
                )
                if reinit_mode != target_mode:
                    node.get_logger().warn(
                        f"BNO055 mode mismatch after re-init: expected 0x{target_mode:02x}, got 0x{reinit_mode:02x}"
                    )
                # Reset estimators so stale pre-reinit samples don't pollute covariance.
                orient_est, gyro_est, accel_est = make_estimators(config)
                _warmup(bno, node, executor)
            except Exception as e:  # noqa: BLE001
                node.get_logger().error(f"BNO055 re-init failed: {e}")
            policy.on_reinit(success=reinit_ok)
            _spin_once_safe(executor, timeout_sec=period_s)
            continue

        quat, gyro, accel = None, None, None
        for _ in range(5):
            try:
                quat = bno.quaternion
                gyro = bno.gyro
                accel = bno.linear_acceleration
            except (RuntimeError, OSError) as e:
                metrics.record_read_error()
                node.get_logger().warn(f"BNO055 read error: {e}", throttle_duration_sec=5.0)
                quat = gyro = accel = None
                hard_failures += 1
                break
            has_gyro = has_valid_tuple(gyro, 3, allow_zeros=True)
            has_accel = has_valid_tuple(accel, 3, allow_zeros=True)
            if has_gyro and has_accel:
                break
            time.sleep(0.01)
        else:
            node.get_logger().warn(
                f"BNO055 no valid gyro/accel after 5 retries (gyro={gyro!r}, accel={accel!r}); attempting mode restore",
                throttle_duration_sec=5.0,
            )
            # All-None returns indicate CONFIG mode (0x00), not a transient I2C glitch: ask the recovery policy
            # (soft restore or escalation) on the next iteration instead of waiting for 10 failing cycles.
            consecutive_failures = I2C_RECONNECT_THRESHOLD
            _spin_once_safe(executor, timeout_sec=period_s)
            continue

        if not has_valid_tuple(accel, 3, allow_zeros=True):
            try:
                accel = bno.acceleration
            except (RuntimeError, OSError):
                pass
        if not has_valid_tuple(gyro, 3, allow_zeros=True) or not has_valid_tuple(accel, 3, allow_zeros=True):
            consecutive_failures += 1
            node.get_logger().warn(
                f"BNO055 accel fallback still invalid (gyro_ok={has_valid_tuple(gyro, 3, allow_zeros=True)}, accel_ok={has_valid_tuple(accel, 3, allow_zeros=True)}); skipping publish",
                throttle_duration_sec=5.0,
            )
            _spin_once_safe(executor, timeout_sec=period_s)
            continue

        consecutive_failures = 0
        hard_failures = 0
        gyro_vals = tuple(coerce(gyro[i]) for i in range(min(3, len(gyro or [])))) if gyro else (0.0, 0.0, 0.0)
        accel_vals = tuple(coerce(accel[i]) for i in range(min(3, len(accel or [])))) if accel else (0.0, 0.0, 0.0)
        while len(gyro_vals) < 3:
            gyro_vals = gyro_vals + (0.0,)
        while len(accel_vals) < 3:
            accel_vals = accel_vals + (0.0,)

        # Feed estimators (only when compute_covariance is enabled).
        if gyro_est is not None:
            gyro_est.add(gyro_vals[0], gyro_vals[1], gyro_vals[2])
        if accel_est is not None:
            accel_est.add(accel_vals[0], accel_vals[1], accel_vals[2])
        if orient_est is not None:
            try:
                euler = bno.euler  # (heading_deg, roll_deg, pitch_deg) or None
                if euler and len(euler) >= 3 and all(euler[i] is not None for i in range(3)):
                    orient_est.add(
                        coerce(euler[0]) * DEG_TO_RAD,
                        coerce(euler[1]) * DEG_TO_RAD,
                        coerce(euler[2]) * DEG_TO_RAD,
                    )
            except (RuntimeError, OSError):
                pass

        if valid_quat(quat):
            quat_vals = tuple(coerce(quat[i]) for i in range(4)) if quat else IDENTITY_QUAT_WXYZ
            _orient_est_cov = orient_est.covariance() if orient_est is not None else None
            # Reject zero-diagonal estimated covariance (e.g. Euler heading stuck at 0 in
            # IMUPLUS mode — no magnetometer — making the estimator useless for orientation).
            if _orient_est_cov is not None and not any(_orient_est_cov[i] > 1e-15 for i in (0, 4, 8)):
                _orient_est_cov = None
            orient_cov = _orient_est_cov or config.orientation_covariance
        else:
            quat_vals = IDENTITY_QUAT_WXYZ
            orient_cov = ORIENTATION_UNKNOWN_COV

        angular_vel_cov = (
            gyro_est.covariance() if gyro_est is not None else None
        ) or config.angular_velocity_covariance
        linear_accel_cov = (
            accel_est.covariance() if accel_est is not None else None
        ) or config.linear_acceleration_covariance

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
            angular_velocity_covariance=angular_vel_cov,
            linear_acceleration_covariance=linear_accel_cov,
        )
        pub.publish(msg)
        metrics.record_publish()
        policy.on_publish()  # a restore only counts as successful once a valid sample is published
        if calibration_pub is not None and time.monotonic() - last_calibration_pub_s >= CALIBRATION_PUBLISH_PERIOD_S:
            last_calibration_pub_s = time.monotonic()
            try:
                calibration_status = bno.calibration_status
                metrics.record_calibration(calibration_status)
                payload = calibration_payload(calibration_status, restored=saved_profile is not None)
            except (RuntimeError, OSError):
                payload = None
            if payload is not None:
                calibration_pub.publish(String(data=payload))
                node.get_logger().debug(f"BNO055 calibration {payload}")
        if calibration_path is not None:
            now_s = time.monotonic()
            if now_s - last_save_s >= config.calibration_save_interval_s:
                last_save_s = now_s
                try:
                    if should_save(bno.calibration_status, 0.0, now_s, 0.0, None, saved_profile):
                        new_profile = save_if_changed(bno, calibration_path, target_mode, saved_profile)
                        if new_profile != saved_profile:
                            saved_profile = new_profile
                            node.get_logger().info(f"saved calibration profile to {calibration_path}")
                except (RuntimeError, OSError) as e:
                    node.get_logger().warn(f"BNO055 calibration save failed: {e}")
        _spin_once_safe(executor, timeout_sec=period_s)

    node.destroy_node()
    rclpy.shutdown()
