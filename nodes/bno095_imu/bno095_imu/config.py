"""Configuration loading for BNO095 IMU node (topic, frame_id, publish rate, bus, covariances)."""

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

DEFAULT_CONFIG_PATH = Path("/etc/ros2/bno095_imu/config.yaml")
ENV_CONFIG_PATH_KEY = "BNO095_IMU_CONFIG"


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
    """BNO095 IMU node config: topic, frame_id, rate, I2C bus, covariances.

    Attributes:
        topic: ROS2 topic for sensor_msgs/Imu (e.g. /imu/data).
        frame_id: Header frame_id for Imu messages (e.g. imu_link).
        publish_hz: Publish rate in Hz.
        i2c_bus: I2C bus number (e.g. 1 for /dev/i2c-1). Used only when bus selection is supported.
        orientation_covariance: 9-element row-major orientation covariance; -1 in [0] means unknown.
        angular_velocity_covariance: 9-element row-major angular velocity covariance.
        linear_acceleration_covariance: 9-element row-major linear acceleration covariance.
    """

    topic: str
    frame_id: str
    publish_hz: float
    i2c_bus: int
    orientation_covariance: list[float]
    angular_velocity_covariance: list[float]
    linear_acceleration_covariance: list[float]


# Default covariance values: diagonal, low/moderate uncertainty for Nav2.
DEFAULT_ORIENTATION_COVARIANCE = 0.01
DEFAULT_ANGULAR_VELOCITY_COVARIANCE = 0.01
DEFAULT_LINEAR_ACCELERATION_COVARIANCE = 0.04


def load_config(path: Path | None = None) -> ImuNodeConfig | None:
    """Load BNO095 IMU config from YAML file.

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
    orientation_cov = _parse_covariance(data.get("orientation_covariance"), DEFAULT_ORIENTATION_COVARIANCE)
    angular_vel_cov = _parse_covariance(data.get("angular_velocity_covariance"), DEFAULT_ANGULAR_VELOCITY_COVARIANCE)
    linear_accel_cov = _parse_covariance(
        data.get("linear_acceleration_covariance"), DEFAULT_LINEAR_ACCELERATION_COVARIANCE
    )
    return ImuNodeConfig(
        topic=topic,
        frame_id=frame_id,
        publish_hz=publish_hz,
        i2c_bus=i2c_bus,
        orientation_covariance=orientation_cov,
        angular_velocity_covariance=angular_vel_cov,
        linear_acceleration_covariance=linear_accel_cov,
    )


def load_config_from_env() -> ImuNodeConfig | None:
    """Load config from path in BNO095_IMU_CONFIG env, or default path.

    Returns:
        ImuNodeConfig | None: Result of load_config(path).
    """
    path_str = os.environ.get(ENV_CONFIG_PATH_KEY, "").strip()
    path = Path(path_str) if path_str else DEFAULT_CONFIG_PATH
    return load_config(path)
