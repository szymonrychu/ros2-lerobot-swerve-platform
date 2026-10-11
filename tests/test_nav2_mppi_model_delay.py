"""MPPI model_delay_vx/vy/wz must follow from the swerve steering kinematics (static YAML checks, no ROS)."""

import math
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
NAV2_PARAMS = REPO_ROOT / "nodes" / "nav2_bringup" / "config" / "nav2_params.yaml"
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"

RESTEER_STEPS = 3


def follow_path() -> dict[str, Any]:
    """Load the controller_server FollowPath (MPPI) namespace.

    Returns:
        dict[str, Any]: FollowPath parameters from nav2_params.yaml.
    """
    params = yaml.safe_load(NAV2_PARAMS.read_text())
    return params["controller_server"]["ros__parameters"]["FollowPath"]


def swerve_config() -> dict[str, Any]:
    """Load the swerve_controller config block deployed on the client.

    Returns:
        dict[str, Any]: Parsed config.yaml of the swerve_controller ros2_nodes entry.
    """
    nodes = yaml.safe_load(CLIENT_VARS.read_text())["ros2_nodes"]
    entry = next(n for n in nodes if n["name"] == "swerve_controller")
    return yaml.safe_load(entry["config"])


def resteer_steps(angle_rad: float, rate: float, model_dt: float) -> int:
    """Number of MPPI steps a steering move takes (the offset MPPI computes from a delay).

    Args:
        angle_rad (float): Steering angle to travel, rad.
        rate (float): Steering rate, rad/s.
        model_dt (float): MPPI model step, s.

    Returns:
        int: round(angle / rate / model_dt).
    """
    return round(angle_rad / rate / model_dt)


def test_model_delay_vy_matches_the_ninety_degree_resteer() -> None:
    fp = follow_path()
    cfg = swerve_config()
    rate = cfg["max_steer_angular_velocity_rad_s"]
    threshold = cfg["steer_error_threshold_rad"]
    dt = fp["model_dt"]
    assert resteer_steps(math.pi / 2, rate, dt) == RESTEER_STEPS
    assert resteer_steps(math.pi / 2 - threshold, rate, dt) == RESTEER_STEPS
    assert round(fp["model_delay_vy"] / dt) == RESTEER_STEPS


def test_model_delay_vx_and_wz_are_zero() -> None:
    fp = follow_path()
    assert fp["model_delay_vx"] == 0.0
    assert fp["model_delay_wz"] == 0.0


def test_inplace_resteer_rounds_to_a_noop_step() -> None:
    cfg = swerve_config()
    dt = follow_path()["model_dt"]
    rate = cfg["max_steer_angular_velocity_rad_s"]
    # Drive resumes once the steer error is under the threshold, as for the vy derivation.
    assert resteer_steps(math.radians(48.8) - cfg["steer_error_threshold_rad"], rate, dt) == 1


def test_keys_sit_in_the_mppi_namespace_only() -> None:
    params = yaml.safe_load(NAV2_PARAMS.read_text())["controller_server"]["ros__parameters"]
    assert {"model_delay_vx", "model_delay_vy", "model_delay_wz"} <= set(params["FollowPath"])
    assert not {"model_delay_vx", "model_delay_vy", "model_delay_wz"} & set(params)
