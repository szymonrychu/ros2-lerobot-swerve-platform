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


def _diagonal_covariance(var: float) -> list[float]:
    """Build row-major 3x3 diagonal covariance from single variance (same for x,y,z).

    Args:
        var: Variance for each axis (same for all).

    Returns:
        list[float]: 9-element row-major covariance (diagonal = var, rest 0).
    """
    return [float(var), 0.0, 0.0, 0.0, float(var), 0.0, 0.0, 0.0, float(var)]


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
    calibration_topic = str(data.get("calibration_topic", DEFAULT_CALIBRATION_TOPIC) or "").strip() or None
    calibration_file = str(data.get("calibration_file", DEFAULT_CALIBRATION_FILE) or "").strip() or None
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
