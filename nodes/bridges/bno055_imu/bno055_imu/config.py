"""Configuration loading for BNO055 IMU node (topic, frame_id, publish rate, bus, covariances)."""

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

DEFAULT_CONFIG_PATH = Path("/etc/ros2/bno055_imu/config.yaml")
ENV_CONFIG_PATH_KEY = "BNO055_IMU_CONFIG"
SUPPORTED_OPERATION_MODES = ("IMUPLUS", "NDOF", "NDOF_FMC_OFF")
DEFAULT_OPERATION_MODE = "IMUPLUS"
DEFAULT_CALIBRATION_TOPIC = "/imu/calibration"
DEFAULT_CALIBRATION_FILE = "/var/lib/ros2/bno055_imu/calibration.json"
DEFAULT_CALIBRATION_SAVE_INTERVAL_S = 60.0
DEFAULT_MAX_SOFT_RESTORES = 3
DEFAULT_REINIT_AFTER_S = 10.0
DEFAULT_ODOM_TOPIC = "/odom"
DEFAULT_ODOM_TIMEOUT_S = 1.0
DEFAULT_STATIONARY_LINEAR_MPS = 0.01
DEFAULT_STATIONARY_ANGULAR_RAD_S = 0.01
DEFAULT_GYRO_BIAS_STILLNESS_S = 2.0
DEFAULT_GYRO_BIAS_TAU_S = 60.0
DEFAULT_GYRO_BIAS_MAX_RAD_S = 0.02
DEFAULT_GYRO_BIAS_SANITY_RAD_S = 0.05
DEFAULT_GYRO_BIAS_FILE = "/var/lib/ros2/bno055_imu/gyro_bias.json"
DEFAULT_GYRO_BIAS_SAVE_INTERVAL_S = 60.0
DEFAULT_GYRO_BIAS_MEAN_S = 30.0
DEFAULT_GYRO_BIAS_TOPIC = "/imu/gyro_bias"
DEFAULT_SAVE_CALIBRATION_SERVICE = "/imu/calibration/save"
DEFAULT_RESET_CALIBRATION_SERVICE = "/imu/calibration/reset"


def _diagonal_covariance(var: float) -> list[float]:
    """Build row-major 3x3 diagonal covariance from single variance (same for x,y,z).

    Args:
        var: Variance for each axis (same for all).

    Returns:
        list[float]: 9-element row-major covariance (diagonal = var, rest 0).
    """
    return [float(var), 0.0, 0.0, 0.0, float(var), 0.0, 0.0, 0.0, float(var)]


def parse_float(raw: Any, default: float, minimum: float) -> float:
    """Parse a float config value, falling back to default when invalid and clamping to minimum.

    Args:
        raw: Config value.
        default: Value used when raw is not a number.
        minimum: Lower bound of the result.

    Returns:
        float: max(minimum, float(raw)), or default when raw is invalid.
    """
    try:
        return max(minimum, float(raw))
    except (TypeError, ValueError):
        return default


def parse_name(data: dict[str, Any], key: str, default: str) -> str | None:
    """Parse an optional topic, service or file name: stripped, and an empty or null value disables it.

    Args:
        data: Parsed YAML mapping.
        key: Config key.
        default: Value used when the key is absent.

    Returns:
        str | None: The stripped name, or None when disabled.
    """
    return str(data.get(key, default) or "").strip() or None


def _parse_covariance(raw: Any, default_var: float) -> list[float]:
    """Parse covariance from config: single float -> diagonal; list of 9 -> as-is.

    Args:
        raw: Config value (number or list of 9 floats).
        default_var: Default diagonal variance if raw invalid.

    Returns:
        list[float]: 9-element row-major covariance.
    """
    if isinstance(raw, (int, float)):
        return _diagonal_covariance(float(raw))
    if isinstance(raw, list) and len(raw) == 9:
        return [float(x) for x in raw]
    return _diagonal_covariance(default_var)


@dataclass
class ImuNodeConfig:
    """BNO055 IMU node config: topic, frame_id, rate, I2C bus, covariances.

    Attributes:
        topic: ROS2 topic for sensor_msgs/Imu (e.g. /imu/data).
        frame_id: Header frame_id for Imu messages (e.g. imu_link).
        publish_hz: Publish rate in Hz.
        i2c_bus: I2C bus number (e.g. 1 for /dev/i2c-1). Used only when bus selection is supported.
        i2c_address: I2C device address for BNO055 (typically 0x28 or 0x29).
        orientation_covariance: 9-element row-major orientation covariance; -1 in [0] means unknown.
        angular_velocity_covariance: 9-element row-major angular velocity covariance.
        linear_acceleration_covariance: 9-element row-major linear acceleration covariance.
        compute_covariance: When True, compute covariance from a rolling window of readings instead of
            using the fixed config values. Config values are used as fallback until enough samples accumulate.
        covariance_window: Rolling window size (number of samples) for covariance estimation.
        covariance_min_samples: Minimum samples required before estimated covariance is published.
        operation_mode: BNO055 fusion mode: IMUPLUS (gyro+accel, relative heading) or NDOF / NDOF_FMC_OFF
            (adds the magnetometer: heading absolute, referenced to magnetic north).
        calibration_topic: Topic for std_msgs/String JSON {sys, gyro, accel, mag} (0-3 each); None disables it.
        calibration_file: JSON file the sensor offsets are saved to and restored from at init; None disables
            persistence.
        calibration_save_interval_s: Minimum seconds between calibration saves (once gyro, accel, mag are all 3).
        max_soft_restores: Consecutive soft mode restores without a published sample before a full re-init (min 0).
        reinit_after_s: Seconds without a published sample before a full re-init regardless of failure path (min 1).
        odom_topic: nav_msgs/Odometry topic (wheel odometry) that tells when the robot is stationary; None disables
            gyro bias learning (raw angular velocity is published).
        odom_timeout_s: Odometry older than this counts as not stationary (min 0.1).
        stationary_linear_mps: Odometry |vx|, |vy| below this counts as still.
        stationary_angular_rad_s: Odometry |wz| below this counts as still.
        gyro_bias_stillness_s: Seconds of continuous stillness before the gyro bias is learned.
        gyro_bias_tau_s: Time constant of the bias moving average in seconds (min 1).
        gyro_bias_max_rad_s: Clamp of the bias magnitude per axis in rad/s.
        gyro_bias_sanity_rad_s: A sample whose gyro norm minus the current bias exceeds this is not learned.
        gyro_bias_file: JSON file the settled gyro bias is saved to and loaded from at start (next to the calibration
            file); None disables persistence.
        gyro_bias_save_interval_s: Minimum seconds between bias saves (min 1); only a bias that changed is written.
        gyro_bias_mean_s: Seconds of learning during which the bias is a running mean instead of an EMA (min 0); a
            bias is saved only after this window completed.
        gyro_bias_topic: std_msgs/String JSON diagnostic at 1 Hz (raw gyro, bias, learning, settled, rejected
            samples); None disables it.
        save_calibration_service: std_srvs/Trigger that reads the chip profile and writes calibration_file now
            (needs mag 3 and a stationary robot); None disables it.
        reset_calibration_service: std_srvs/Trigger that deletes calibration_file, resets the chip and re-initialises
            it uncalibrated (refused while moving); None disables it.
        metrics_port: Port of the Prometheus /metrics endpoint on 127.0.0.1; None falls back to env METRICS_PORT.
    """

    topic: str
    frame_id: str
    publish_hz: float
    i2c_bus: int
    i2c_address: int
    orientation_covariance: list[float]
    angular_velocity_covariance: list[float]
    linear_acceleration_covariance: list[float]
    compute_covariance: bool
    covariance_window: int
    covariance_min_samples: int
    operation_mode: str = DEFAULT_OPERATION_MODE
    calibration_topic: str | None = DEFAULT_CALIBRATION_TOPIC
    calibration_file: str | None = DEFAULT_CALIBRATION_FILE
    calibration_save_interval_s: float = DEFAULT_CALIBRATION_SAVE_INTERVAL_S
    max_soft_restores: int = DEFAULT_MAX_SOFT_RESTORES
    reinit_after_s: float = DEFAULT_REINIT_AFTER_S
    odom_topic: str | None = DEFAULT_ODOM_TOPIC
    odom_timeout_s: float = DEFAULT_ODOM_TIMEOUT_S
    stationary_linear_mps: float = DEFAULT_STATIONARY_LINEAR_MPS
    stationary_angular_rad_s: float = DEFAULT_STATIONARY_ANGULAR_RAD_S
    gyro_bias_stillness_s: float = DEFAULT_GYRO_BIAS_STILLNESS_S
    gyro_bias_tau_s: float = DEFAULT_GYRO_BIAS_TAU_S
    gyro_bias_max_rad_s: float = DEFAULT_GYRO_BIAS_MAX_RAD_S
    gyro_bias_sanity_rad_s: float = DEFAULT_GYRO_BIAS_SANITY_RAD_S
    gyro_bias_file: str | None = DEFAULT_GYRO_BIAS_FILE
    gyro_bias_save_interval_s: float = DEFAULT_GYRO_BIAS_SAVE_INTERVAL_S
    gyro_bias_mean_s: float = DEFAULT_GYRO_BIAS_MEAN_S
    gyro_bias_topic: str | None = DEFAULT_GYRO_BIAS_TOPIC
    save_calibration_service: str | None = DEFAULT_SAVE_CALIBRATION_SERVICE
    reset_calibration_service: str | None = DEFAULT_RESET_CALIBRATION_SERVICE
    metrics_port: int | None = None


# Default covariance values: diagonal, low/moderate uncertainty for Nav2.
DEFAULT_ORIENTATION_COVARIANCE = 0.01
DEFAULT_ANGULAR_VELOCITY_COVARIANCE = 0.01
DEFAULT_LINEAR_ACCELERATION_COVARIANCE = 0.04

DEFAULT_COMPUTE_COVARIANCE = False
DEFAULT_COVARIANCE_WINDOW = 100
DEFAULT_COVARIANCE_MIN_SAMPLES = 20


def load_config(path: Path | None = None) -> ImuNodeConfig | None:
    """Load BNO055 IMU config from YAML file.

    Args:
        path: Path to YAML file. If None, uses DEFAULT_CONFIG_PATH.

    Returns:
        ImuNodeConfig | None: Parsed config, or None if file missing/invalid.
    """
    if path is None:
        path = DEFAULT_CONFIG_PATH
    if not path.exists():
        return None
    data = yaml.safe_load(path.read_text())
    if data is None or not isinstance(data, dict):
        return None
    topic = (data.get("topic") or "/imu/data").strip()
    frame_id = (data.get("frame_id") or "imu_link").strip()
    raw_hz = data.get("publish_hz", 100.0)
    try:
        publish_hz = max(1.0, min(1000.0, float(raw_hz)))
    except (TypeError, ValueError):
        publish_hz = 100.0
    raw_bus = data.get("i2c_bus", 1)
    try:
        i2c_bus = max(0, int(raw_bus))
    except (TypeError, ValueError):
        i2c_bus = 1
    raw_addr = data.get("i2c_address", "0x28")
    try:
        i2c_address = int(str(raw_addr), 0)
        if not (0x03 <= i2c_address <= 0x77):
            i2c_address = 0x28
    except (TypeError, ValueError):
        i2c_address = 0x28
    orientation_cov = _parse_covariance(data.get("orientation_covariance"), DEFAULT_ORIENTATION_COVARIANCE)
    angular_vel_cov = _parse_covariance(data.get("angular_velocity_covariance"), DEFAULT_ANGULAR_VELOCITY_COVARIANCE)
    linear_accel_cov = _parse_covariance(
        data.get("linear_acceleration_covariance"), DEFAULT_LINEAR_ACCELERATION_COVARIANCE
    )
    compute_covariance = bool(data.get("compute_covariance", DEFAULT_COMPUTE_COVARIANCE))
    raw_window = data.get("covariance_window", DEFAULT_COVARIANCE_WINDOW)
    try:
        covariance_window = max(2, int(raw_window))
    except (TypeError, ValueError):
        covariance_window = DEFAULT_COVARIANCE_WINDOW
    raw_min = data.get("covariance_min_samples", DEFAULT_COVARIANCE_MIN_SAMPLES)
    try:
        covariance_min_samples = max(2, int(raw_min))
    except (TypeError, ValueError):
        covariance_min_samples = DEFAULT_COVARIANCE_MIN_SAMPLES
    operation_mode = str(data.get("operation_mode", DEFAULT_OPERATION_MODE)).strip().upper()
    if operation_mode not in SUPPORTED_OPERATION_MODES:
        operation_mode = DEFAULT_OPERATION_MODE
    calibration_topic = parse_name(data, "calibration_topic", DEFAULT_CALIBRATION_TOPIC)
    calibration_file = parse_name(data, "calibration_file", DEFAULT_CALIBRATION_FILE)
    raw_interval = data.get("calibration_save_interval_s", DEFAULT_CALIBRATION_SAVE_INTERVAL_S)
    try:
        calibration_save_interval_s = max(1.0, float(raw_interval))
    except (TypeError, ValueError):
        calibration_save_interval_s = DEFAULT_CALIBRATION_SAVE_INTERVAL_S
    raw_soft = data.get("max_soft_restores", DEFAULT_MAX_SOFT_RESTORES)
    try:
        max_soft_restores = max(0, int(raw_soft))
    except (TypeError, ValueError):
        max_soft_restores = DEFAULT_MAX_SOFT_RESTORES
    raw_reinit = data.get("reinit_after_s", DEFAULT_REINIT_AFTER_S)
    try:
        reinit_after_s = max(1.0, float(raw_reinit))
    except (TypeError, ValueError):
        reinit_after_s = DEFAULT_REINIT_AFTER_S
    odom_topic = parse_name(data, "odom_topic", DEFAULT_ODOM_TOPIC)
    raw_metrics_port = data.get("metrics_port")
    try:
        metrics_port = int(raw_metrics_port) if raw_metrics_port is not None else None
    except (TypeError, ValueError):
        metrics_port = None
    return ImuNodeConfig(
        topic=topic,
        frame_id=frame_id,
        publish_hz=publish_hz,
        i2c_bus=i2c_bus,
        i2c_address=i2c_address,
        orientation_covariance=orientation_cov,
        angular_velocity_covariance=angular_vel_cov,
        linear_acceleration_covariance=linear_accel_cov,
        compute_covariance=compute_covariance,
        covariance_window=covariance_window,
        covariance_min_samples=covariance_min_samples,
        operation_mode=operation_mode,
        calibration_topic=calibration_topic,
        calibration_file=calibration_file,
        calibration_save_interval_s=calibration_save_interval_s,
        max_soft_restores=max_soft_restores,
        reinit_after_s=reinit_after_s,
        odom_topic=odom_topic,
        odom_timeout_s=parse_float(data.get("odom_timeout_s", DEFAULT_ODOM_TIMEOUT_S), DEFAULT_ODOM_TIMEOUT_S, 0.1),
        stationary_linear_mps=parse_float(
            data.get("stationary_linear_mps", DEFAULT_STATIONARY_LINEAR_MPS), DEFAULT_STATIONARY_LINEAR_MPS, 0.0
        ),
        stationary_angular_rad_s=parse_float(
            data.get("stationary_angular_rad_s", DEFAULT_STATIONARY_ANGULAR_RAD_S),
            DEFAULT_STATIONARY_ANGULAR_RAD_S,
            0.0,
        ),
        gyro_bias_stillness_s=parse_float(
            data.get("gyro_bias_stillness_s", DEFAULT_GYRO_BIAS_STILLNESS_S), DEFAULT_GYRO_BIAS_STILLNESS_S, 0.0
        ),
        gyro_bias_tau_s=parse_float(data.get("gyro_bias_tau_s", DEFAULT_GYRO_BIAS_TAU_S), DEFAULT_GYRO_BIAS_TAU_S, 1.0),
        gyro_bias_max_rad_s=parse_float(
            data.get("gyro_bias_max_rad_s", DEFAULT_GYRO_BIAS_MAX_RAD_S), DEFAULT_GYRO_BIAS_MAX_RAD_S, 0.0
        ),
        gyro_bias_sanity_rad_s=parse_float(
            data.get("gyro_bias_sanity_rad_s", DEFAULT_GYRO_BIAS_SANITY_RAD_S), DEFAULT_GYRO_BIAS_SANITY_RAD_S, 0.0
        ),
        gyro_bias_file=parse_name(data, "gyro_bias_file", DEFAULT_GYRO_BIAS_FILE),
        gyro_bias_save_interval_s=parse_float(
            data.get("gyro_bias_save_interval_s", DEFAULT_GYRO_BIAS_SAVE_INTERVAL_S),
            DEFAULT_GYRO_BIAS_SAVE_INTERVAL_S,
            1.0,
        ),
        gyro_bias_mean_s=parse_float(
            data.get("gyro_bias_mean_s", DEFAULT_GYRO_BIAS_MEAN_S), DEFAULT_GYRO_BIAS_MEAN_S, 0.0
        ),
        gyro_bias_topic=parse_name(data, "gyro_bias_topic", DEFAULT_GYRO_BIAS_TOPIC),
        save_calibration_service=parse_name(data, "save_calibration_service", DEFAULT_SAVE_CALIBRATION_SERVICE),
        reset_calibration_service=parse_name(data, "reset_calibration_service", DEFAULT_RESET_CALIBRATION_SERVICE),
        metrics_port=metrics_port,
    )


def load_config_from_env() -> ImuNodeConfig | None:
    """Load config from path in BNO055_IMU_CONFIG env, or default path.

    Returns:
        ImuNodeConfig | None: Result of load_config(path).
    """
    path_str = os.environ.get(ENV_CONFIG_PATH_KEY, "").strip()
    path = Path(path_str) if path_str else DEFAULT_CONFIG_PATH
    return load_config(path)
