"""Configuration for the rf2o odometry relay."""

import os
from dataclasses import dataclass
from pathlib import Path

import yaml

from .twist import DEFAULT_MAX_DT_S

DEFAULT_CONFIG_PATH = Path("/etc/ros2/rf2o_odom_relay/config.yaml")
ENV_CONFIG_PATH_KEY = "RF2O_ODOM_RELAY_CONFIG"
DEFAULT_INPUT_TOPIC = "/odom_rf2o"
DEFAULT_OUTPUT_TOPIC = "/odom_rf2o_twist"
# rf2o gives no quality figure. Pose differences over ~0.1 s scans carry a few mm of noise, i.e. about 0.05-0.1 m/s,
# so 0.02 (m/s)^2 is conservative: larger than the wheel odometry's 0.01 floor, so wheels win while they are
# consistent and the lidar takes over when the wheel covariance grows with the slip residual.
DEFAULT_VAR_VX_VY = 0.02
# Yaw rate variance (rad/s)^2; large against the gyro (0.0004) so it can never outvote it.
DEFAULT_VAR_VYAW = 0.05


@dataclass(frozen=True)
class RelayConfig:
    """Relay settings.

    Attributes:
        input_topic: rf2o nav_msgs/Odometry topic.
        output_topic: Topic published for the EKF.
        var_vx_vy: Variance of vx and vy, (m/s)^2.
        var_vyaw: Variance of the yaw rate, (rad/s)^2.
        max_dt_s: Largest time between two rf2o poses that still yields a twist, s.
    """

    input_topic: str
    output_topic: str
    var_vx_vy: float
    var_vyaw: float
    max_dt_s: float


def load_config(path: Path | None = None) -> RelayConfig | None:
    """Load the relay config from YAML.

    Args:
        path: YAML file; DEFAULT_CONFIG_PATH when None.

    Returns:
        RelayConfig | None: Parsed config, or None when the file is missing or not a mapping.
    """
    path = path or DEFAULT_CONFIG_PATH
    if not path.exists():
        return None
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict):
        return None
    return RelayConfig(
        input_topic=str(data.get("input_topic") or DEFAULT_INPUT_TOPIC).strip(),
        output_topic=str(data.get("output_topic") or DEFAULT_OUTPUT_TOPIC).strip(),
        var_vx_vy=max(1e-6, float(data.get("var_vx_vy", DEFAULT_VAR_VX_VY))),
        var_vyaw=max(1e-6, float(data.get("var_vyaw", DEFAULT_VAR_VYAW))),
        max_dt_s=max(0.01, float(data.get("max_dt_s", DEFAULT_MAX_DT_S))),
    )


def load_config_from_env() -> RelayConfig | None:
    """Load the config from RF2O_ODOM_RELAY_CONFIG or the default path.

    Returns:
        RelayConfig | None: Result of load_config.
    """
    value = os.environ.get(ENV_CONFIG_PATH_KEY, "").strip()
    return load_config(Path(value) if value else None)
