"""arm_home under the measured servo delay (2026-10-11 12:19:55 aborted_tracking) and the delay-aware tracking limit."""

from pathlib import Path

import pytest

from mcp_server.arm import ArmController, setpoint_travel, tracking_limit
from mcp_server.config import McpServerConfig
from mcp_server.home_store import save_home
from mcp_server.ik import ArmKinematics, load_joint_limits

from .fakes import JOINTS, DelayedFollower, FakeArmBackend

CONFIG = McpServerConfig()
KIN = ArmKinematics(CONFIG.arm.urdf_path, margin=CONFIG.limits.arm_limit_margin_rad)

# Measured pose right before the aborted arm_home (mcp_server arm_settle 12:19:50, follower joint lines).
START = {
    "shoulder_pan": -0.0862,
    "shoulder_lift": -0.2008,
    "elbow_flex": 0.5904,
    "wrist_flex": 0.7001,
    "wrist_roll": -1.6,
    "gripper": 0.325,
}
# Stored home pose (target of the converging move_arm_joints at 12:20:11); the gripper value is not logged.
HOME = {
    "shoulder_pan": -0.003,
    "shoulder_lift": -1.35,
    "elbow_flex": 1.578,
    "wrist_flex": -0.1,
    "wrist_roll": -1.637,
    "gripper": 0.1,
}
# Command-to-motion delay per joint fitted to the follower's measured lines during the aborted home (s).
MEASURED_DELAYS_S = {"shoulder_lift": 0.53, "elbow_flex": 0.65, "wrist_flex": 0.63}
DEFAULT_DELAY_S = 0.6


def make(tmp_path: Path, config: McpServerConfig = CONFIG) -> tuple[ArmController, FakeArmBackend]:
    """Controller on a fake follower at the measured start pose, with the measured home pose stored.

    Args:
        tmp_path (Path): Test directory (home file).
        config (McpServerConfig): Configuration to copy.

    Returns:
        tuple[ArmController, FakeArmBackend]: Controller and backend.
    """
    be = FakeArmBackend(dict(START))
    cfg = config.model_copy(deep=True)
    cfg.arm.home_file = tmp_path / "arm" / "home.yaml"
    save_home(cfg.arm.home_file, HOME)
    return ArmController(be, KIN, load_joint_limits(cfg.arm.urdf_path), cfg), be


def test_home_with_the_measured_servo_delay_converges(tmp_path: Path) -> None:
    """The 2026-10-11 abort: full-speed home lagged by the measured delay converges instead of aborted_tracking."""
    arm, be = make(tmp_path)
    DelayedFollower(be, MEASURED_DELAYS_S, DEFAULT_DELAY_S)
    res = arm.home()
    assert res.status == "converged", res.message
    assert be.positions["shoulder_lift"] == pytest.approx(HOME["shoulder_lift"], abs=0.05)


def test_home_with_the_measured_delay_aborted_with_the_instantaneous_limit(tmp_path: Path) -> None:
    """Without the delay window (arm_tracking_delay_s 0) the same home aborts like on the robot (0.504 rad over 0.500).

    At nominal 40 ms ticks the fake trips on elbow_flex (longest delay) just before shoulder_lift; on the robot (48.5 ms
    ticks, plus its gravity sag) shoulder_lift led.
    """
    cfg = CONFIG.model_copy(deep=True)
    cfg.limits.arm_tracking_delay_s = 0.0
    arm, be = make(tmp_path, cfg)
    DelayedFollower(be, MEASURED_DELAYS_S, DEFAULT_DELAY_S)
    res = arm.home()
    assert res.status == "aborted_tracking"
    assert "exceeds 0.500" in res.message
    assert res.message.endswith(("(shoulder_lift)", "(elbow_flex)"))


def test_a_jammed_joint_still_aborts_a_full_speed_home(tmp_path: Path) -> None:
    """A joint that does not move at all (jam, collision) still aborts, the limit stays bounded."""
    arm, be = make(tmp_path)
    arm.acquire()
    be.follow = False
    res = arm.home()
    assert res.status == "aborted_tracking"
    lim = CONFIG.limits
    vmax = lim.arm_max_joint_velocity_rps
    worst = lim.arm_tracking_error_rad + max(lim.arm_tracking_lag_s * vmax, lim.arm_tracking_delay_s * vmax)
    assert abs(be.positions["shoulder_lift"] - START["shoulder_lift"]) < 1e-9
    assert float(res.message.split("tracking error ")[1].split(" ")[0]) <= worst + 0.05


def test_tracking_limit_takes_the_larger_of_the_speed_term_and_the_recent_travel() -> None:
    assert tracking_limit(0.25, 0.25, 1.0, 0.6) == pytest.approx(0.85)
    assert tracking_limit(0.25, 0.25, 1.0, 0.1) == pytest.approx(0.5)
    assert tracking_limit(0.25, 0.25, 1.0) == pytest.approx(0.5)


def test_setpoint_travel_spans_the_window() -> None:
    history = [
        (0.0, {"shoulder_lift": 0.0, "elbow_flex": 0.0}),
        (0.5, {"shoulder_lift": -0.3, "elbow_flex": 0.1}),
        (1.0, {"shoulder_lift": -0.5, "elbow_flex": 0.1}),
    ]
    assert setpoint_travel(history, 1.0, 0.6) == pytest.approx({"shoulder_lift": 0.5, "elbow_flex": 0.1})
    assert setpoint_travel(history, 1.0, 0.4) == pytest.approx({"shoulder_lift": 0.2, "elbow_flex": 0.0})
    assert setpoint_travel(history, 1.0, 0.0) == pytest.approx({"shoulder_lift": 0.0, "elbow_flex": 0.0})
    assert setpoint_travel([], 1.0, 0.6) == {}


def test_joints_constant_is_complete() -> None:
    assert set(START) == set(HOME) == set(JOINTS)
