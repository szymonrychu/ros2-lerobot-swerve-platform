"""Arm settle policy (final vs trajectory_end), per-joint convergence tolerances, gripper exclusion, timing split."""

from pathlib import Path

import pytest
from pydantic import ValidationError

from mcp_server.arm import ArmController, tracking_limit
from mcp_server.config import LimitSettings, McpServerConfig
from mcp_server.ik import ArmKinematics, load_joint_limits

from .fakes import FakeArmBackend

CONFIG = McpServerConfig()
KIN = ArmKinematics(CONFIG.arm.urdf_path, margin=CONFIG.limits.arm_limit_margin_rad)
PERIOD = 1.0 / CONFIG.limits.arm_rate_hz


def make(tmp_path: Path) -> tuple[ArmController, FakeArmBackend]:
    be = FakeArmBackend()
    cfg = CONFIG.model_copy(deep=True)
    cfg.arm.home_file = tmp_path / "arm" / "home.yaml"
    return ArmController(be, KIN, load_joint_limits(cfg.arm.urdf_path), cfg), be


def lag_behind(be: FakeArmBackend, joint: str, lag: float) -> None:
    """The follower trails the setpoint of `joint` by a constant `lag` (rad) and follows the other joints exactly."""
    be.follow = False

    def hook(b: FakeArmBackend) -> None:
        last = b.commands[-1]
        for j, v in last.items():
            b.positions[j] = v - lag if j == joint else v

    be.on_sleep = hook


# --- trajectory_end -------------------------------------------------------------------------------------------


def test_trajectory_end_returns_when_the_trajectory_finishes_and_reports_settling(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    arm.acquire()
    lag_behind(be, "elbow_flex", 0.1)
    t0 = be.t
    res = arm.move_joints({"elbow_flex": 0.3}, speed_scale=0.5, settle="trajectory_end")
    assert res.status == "converged"
    assert res.settle == "trajectory_end" and res.settling is True
    assert res.tracking_error_rad == pytest.approx(0.1, abs=0.01)
    assert "settling" in res.message
    assert res.settle_s == 0.0 and res.trajectory_s == res.duration_s
    # No convergence wait: only the streamed trajectory elapsed (0.3 rad at 0.5 rad/s plus the quintic stretch).
    assert be.t - t0 < 2.0 and be.t - t0 < CONFIG.timeouts.arm_converge_timeout_s
    assert be.commands[-1]["elbow_flex"] == pytest.approx(0.3)


def test_trajectory_end_within_tolerance_is_not_settling(tmp_path: Path) -> None:
    arm, _ = make(tmp_path)
    res = arm.move_joints({"elbow_flex": 0.3}, speed_scale=0.5, settle="trajectory_end")
    assert res.status == "converged" and res.settling is False
    assert res.tracking_error_rad == pytest.approx(0.0, abs=1e-6)
    assert res.message == "reached target"


def test_final_policy_keeps_waiting_for_convergence(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    arm.acquire()
    lag_behind(be, "wrist_flex", 0.1)  # outside the wrist's settle band
    res = arm.move_joints({"wrist_flex": 0.3}, speed_scale=0.5)  # default policy of the controller is final
    assert res.status == "timeout" and res.settle == "final" and res.settling is False
    assert res.settle_s >= CONFIG.timeouts.arm_converge_timeout_s - PERIOD
    assert res.trajectory_s + res.settle_s == pytest.approx(res.duration_s, abs=0.002)


def test_trajectory_end_keeps_the_goal_commanded_and_releases_the_motion_lock(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    arm.acquire()
    lag_behind(be, "elbow_flex", 0.1)
    arm.move_joints({"elbow_flex": 0.3}, speed_scale=0.5, settle="trajectory_end")
    arm.keepalive_tick()
    assert be.commands[-1]["elbow_flex"] == pytest.approx(0.3)
    assert arm.control_held
    arm.move_joints({"shoulder_lift": 0.1}, speed_scale=0.5, settle="trajectory_end")  # no ArmBusyError


def test_next_move_starts_named_joints_from_the_measured_state_and_keeps_other_intents(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    arm.acquire()
    lag_behind(be, "elbow_flex", 0.1)
    arm.move_joints({"elbow_flex": 0.3}, speed_scale=0.5, settle="trajectory_end")
    measured_elbow = be.positions["elbow_flex"]
    n = len(be.commands)
    arm.move_joints({"elbow_flex": 0.5, "wrist_flex": 0.2}, speed_scale=0.5, settle="trajectory_end")
    assert be.commands[n]["elbow_flex"] == pytest.approx(measured_elbow, abs=0.02)  # no jump back or ahead
    # A move that does not name the settling joint keeps its intended target.
    arm.move_joints({"elbow_flex": 0.3}, speed_scale=0.5, settle="trajectory_end")
    n = len(be.commands)
    arm.move_joints({"shoulder_pan": 0.1}, speed_scale=0.5, settle="trajectory_end")
    assert be.commands[n]["elbow_flex"] == pytest.approx(0.3)


def test_settling_marker_is_cleared_by_release(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    arm.acquire()
    lag_behind(be, "elbow_flex", 0.1)
    arm.move_joints({"elbow_flex": 0.3}, speed_scale=0.5, settle="trajectory_end")
    arm.release()
    assert arm.settling == []


def test_trajectory_end_relaxes_a_stalled_joint_after_the_hold_window(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    arm.acquire()
    lag_behind(be, "elbow_flex", 0.1)
    arm.move_joints({"elbow_flex": 0.3}, speed_scale=0.5, settle="trajectory_end")
    be.on_sleep = None
    be.t += CONFIG.limits.arm_settle_hold_s + 0.1
    arm.keepalive_tick()
    assert be.commands[-1]["elbow_flex"] == pytest.approx(be.positions["elbow_flex"])  # not pushed forever


def test_trajectory_end_still_aborts_on_stale_feedback(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    arm.acquire()
    be.stale_after = be.t + 0.2
    res = arm.move_joints({"elbow_flex": 0.3}, speed_scale=0.5, settle="trajectory_end")
    assert res.status == "aborted_stale"


def test_trajectory_end_still_aborts_on_tracking_error(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    lim = CONFIG.limits
    be.sag = {"shoulder_lift": -(tracking_limit(lim.arm_tracking_error_rad, lim.arm_tracking_lag_s, 1.0) + 0.05)}
    res = arm.move_joints({"shoulder_lift": 0.5}, speed_scale=0.5, settle="trajectory_end")
    assert res.status == "aborted_tracking"
    assert be.commands[-1] == res.positions


def test_cartesian_accepts_the_settle_policy(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    res = arm.move_cartesian(0.2, 0.0, 0.15, None, 0.5, settle="trajectory_end")
    assert res.settle == "trajectory_end"


# --- convergence timeouts: gripper exclusion and per-joint tolerances ------------------------------------------


def test_stuck_gripper_does_not_time_out_an_arm_move(tmp_path: Path) -> None:
    """The measured 54 timeouts: a gripper holding an object (or any stuck jaw) never reaches its target."""
    arm, be = make(tmp_path)
    arm.acquire()
    be.follow = False

    def jaw_stuck(b: FakeArmBackend) -> None:
        for j, v in b.commands[-1].items():
            if j != "gripper":
                b.positions[j] = v

    be.on_sleep = jaw_stuck
    res = arm.move_joints({"elbow_flex": 0.3, "gripper": 1.0}, speed_scale=0.5)
    assert res.status == "converged", res.message
    assert "gripper" in res.message


def test_gripper_only_move_still_judges_the_gripper(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    arm.acquire()
    be.follow = False
    be.on_sleep = lambda b: None  # the jaw never moves
    res = arm.move_joints({"gripper": 1.0}, speed_scale=0.5)
    assert res.status == "timeout"


def test_default_tolerances_are_looser_for_the_gravity_loaded_joints() -> None:
    lim = LimitSettings()
    for joint in ("shoulder_lift", "elbow_flex"):
        assert lim.converge_tolerance_for(joint) > lim.arm_converge_tolerance_rad
        assert lim.arm_converge_tolerance_rad < lim.settle_tolerance_for(joint) < lim.arm_tracking_error_rad
    assert lim.converge_tolerance_for("wrist_flex") == lim.arm_converge_tolerance_rad
    assert lim.settle_tolerance_for("wrist_flex") == lim.arm_settle_tolerance_rad


def test_loaded_joint_within_its_own_tolerance_converges_cleanly(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.sag = {"elbow_flex": 0.04}  # above the global 0.03, inside the elbow tolerance
    res = arm.move_joints({"elbow_flex": 0.3}, speed_scale=0.5)
    assert res.status == "converged" and res.message == "reached target" and res.residual_error == {}


def test_unloaded_joint_keeps_the_global_tolerance(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.sag = {"wrist_flex": 0.04}
    res = arm.move_joints({"wrist_flex": 0.3}, speed_scale=0.5)
    assert res.status == "converged" and "settled with residual error" in res.message


def test_override_must_stay_between_converge_and_settle_bands() -> None:
    with pytest.raises(ValidationError):
        LimitSettings(arm_settle_tolerance_overrides={"elbow_flex": 0.04})  # below its converge override (0.05)
    with pytest.raises(ValidationError):
        LimitSettings(arm_converge_tolerance_overrides={"elbow_flex": 0.5})


def test_default_settle_is_trajectory_end_and_configurable() -> None:
    assert LimitSettings().arm_default_settle == "trajectory_end"
    with pytest.raises(ValidationError):
        LimitSettings(arm_default_settle="never")
