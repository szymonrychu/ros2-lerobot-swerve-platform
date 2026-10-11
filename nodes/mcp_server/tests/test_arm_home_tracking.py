"""arm_home under the measured servo lag and the tracking check against the setpoint of arm_tracking_delay_s ago."""

import re
from pathlib import Path

import pytest

from mcp_server.arm import ArmController, tracking_limit
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
# Clean command-to-motion lag of the follower (fit of the measured joint lines against the actual setpoint times,
# 2026-10-11 with P=16: 0.20-0.24 s on every joint).
SERVO_LAG_S = 0.22
# A follower stalled far longer than the clean lag (the old register-dump stall the 0.65 s window had covered).
STALLED_LAG_S = 1.2


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


def test_home_with_the_measured_servo_lag_converges(tmp_path: Path) -> None:
    """Full-speed home on a follower lagging the measured 0.22 s converges with arm_tracking_delay_s 0.35."""
    assert CONFIG.limits.arm_tracking_delay_s == pytest.approx(0.35)
    arm, be = make(tmp_path)
    DelayedFollower(be, {}, SERVO_LAG_S)
    res = arm.home()
    assert res.status == "converged", res.message
    assert be.positions["shoulder_lift"] == pytest.approx(HOME["shoulder_lift"], abs=0.05)


def test_home_aborts_on_a_follower_stalled_far_beyond_the_delay(tmp_path: Path) -> None:
    """The tracking check compares against the setpoint of arm_tracking_delay_s ago: a 1.2 s stall is a fault."""
    arm, be = make(tmp_path)
    DelayedFollower(be, {}, STALLED_LAG_S)
    res = arm.home()
    assert res.status == "aborted_tracking", res.message


def test_a_jammed_joint_still_aborts_a_full_speed_home(tmp_path: Path) -> None:
    """A joint that does not move at all (jam, collision) still aborts, the limit stays the speed-term limit."""
    arm, be = make(tmp_path)
    arm.acquire()
    be.follow = False
    res = arm.home()
    assert res.status == "aborted_tracking"
    lim = CONFIG.limits
    worst = tracking_limit(lim.arm_tracking_error_rad, lim.arm_tracking_lag_s, lim.arm_max_joint_velocity_rps)
    assert abs(be.positions["shoulder_lift"] - START["shoulder_lift"]) < 1e-9
    match = re.search(r"tracking error ([0-9.]+) rad exceeds ([0-9.]+)", res.message)
    assert match is not None, res.message
    error, limit = float(match.group(1)), float(match.group(2))
    assert limit == pytest.approx(worst, abs=1e-3)
    assert error <= worst + 0.05


def test_tracking_limit_grows_with_the_speed() -> None:
    assert tracking_limit(0.25, 0.25, 1.0) == pytest.approx(0.5)
    assert tracking_limit(0.25, 0.25, 0.2) == pytest.approx(0.3)


def test_joints_constant_is_complete() -> None:
    assert set(START) == set(HOME) == set(JOINTS)
