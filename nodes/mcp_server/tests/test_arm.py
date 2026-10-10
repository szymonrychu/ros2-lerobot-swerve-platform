"""Tests for mcp_server.arm.ArmController: autonomy lease, streamed motion, aborts, gripper, home pose, keepalive."""

import math
from pathlib import Path

import pytest

from mcp_server.arm import ArmController, ArmError, tracking_limit
from mcp_server.config import McpServerConfig
from mcp_server.floor_guard import FloorOverride, Tilt, TiltOverrideDeg, TiltSample
from mcp_server.ik import ArmKinematics, grasp_offset, load_joint_limits
from mcp_server.models import ArmMotionResult

from .fakes import JOINTS, FakeArmBackend

CONFIG = McpServerConfig()
KIN = ArmKinematics(CONFIG.arm.urdf_path, margin=CONFIG.limits.arm_limit_margin_rad)


def make(tmp_path: Path, backend: FakeArmBackend | None = None) -> tuple[ArmController, FakeArmBackend]:
    be = backend or FakeArmBackend()
    cfg = CONFIG.model_copy(deep=True)
    cfg.arm.home_file = tmp_path / "arm" / "home.yaml"
    # These tests exercise the global tolerances; the per-joint defaults are covered by test_arm_settle.py.
    cfg.limits.arm_converge_tolerance_overrides = {}
    cfg.limits.arm_settle_tolerance_overrides = {}
    return ArmController(be, KIN, load_joint_limits(cfg.arm.urdf_path), cfg), be


def test_acquire_publishes_measured_pose(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.positions["elbow_flex"] = 0.4
    res = arm.acquire()
    assert res.control_held
    assert be.commands == [be.positions]
    assert arm.control_held


def test_acquire_refuses_without_fresh_joint_states(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.no_samples = True
    with pytest.raises(ArmError):
        arm.acquire()
    assert be.commands == []


def test_acquire_refuses_stale_joint_states(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.stale_after = be.t - 1.0
    with pytest.raises(ArmError):
        arm.acquire()


def test_release_publishes_release(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    arm.acquire()
    res = arm.release()
    assert be.releases == 1
    assert not res.control_held
    assert not arm.control_held


def test_move_joints_streams_and_converges_with_implicit_acquire(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    res = arm.move_joints({"elbow_flex": 0.5}, speed_scale=0.5)
    assert res.status == "converged", res.message
    assert be.commands[0] == {j: 0.0 for j in JOINTS}  # implicit acquire at measured pose
    assert be.commands[-1]["elbow_flex"] == pytest.approx(0.5)
    assert len(be.commands) > 10
    rate = CONFIG.limits.arm_rate_hz
    vmax = CONFIG.limits.arm_max_joint_velocity_rps
    for a, b in zip(be.commands, be.commands[1:], strict=False):
        assert abs(b["elbow_flex"] - a["elbow_flex"]) * rate <= vmax * 1.05
    assert arm.control_held


def test_lower_speed_scale_is_slower(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    arm.move_joints({"elbow_flex": 0.5}, speed_scale=0.5)
    fast = len(be.commands)
    arm2, be2 = make(tmp_path)
    arm2.move_joints({"elbow_flex": 0.5}, speed_scale=0.25)
    assert len(be2.commands) > fast * 1.7


def test_move_joints_clamps_to_limits_minus_margin(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    res = arm.move_joints({"wrist_flex": 3.0}, speed_scale=0.5)
    hi = load_joint_limits(CONFIG.arm.urdf_path)["wrist_flex"][1] - CONFIG.limits.arm_limit_margin_rad
    assert be.commands[-1]["wrist_flex"] == pytest.approx(hi)
    assert res.clamped == ["wrist_flex"]


def test_move_joints_rejects_bad_arguments(tmp_path: Path) -> None:
    arm, _ = make(tmp_path)
    with pytest.raises(ArmError):
        arm.move_joints({"elbow_flex": 0.1}, speed_scale=0.9)
    with pytest.raises(ArmError):
        arm.move_joints({"elbow_flex": 0.1}, speed_scale=0.0)
    with pytest.raises(ArmError):
        arm.move_joints({"knee": 0.1}, speed_scale=0.5)
    with pytest.raises(ArmError):
        arm.move_joints({"elbow_flex": float("nan")}, speed_scale=0.5)
    with pytest.raises(ArmError):
        arm.move_joints({}, speed_scale=0.5)


def test_stale_follower_aborts_and_holds(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.stale_after = be.t + 0.4
    res = arm.move_joints({"elbow_flex": 1.0}, speed_scale=0.5)
    assert res.status == "aborted_stale"
    # Hold = the last measured pose, republished.
    assert be.commands[-1] == be.positions


def test_tracking_error_aborts_and_holds(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    arm.acquire()
    be.follow = False
    res = arm.move_joints({"shoulder_lift": 1.0}, speed_scale=0.5)
    assert res.status == "aborted_tracking"
    assert be.commands[-1] == be.positions
    assert be.positions["shoulder_lift"] == 0.0


def test_stop_request_preempts_motion(tmp_path: Path) -> None:
    arm, be = make(tmp_path)

    def stop_soon(b: FakeArmBackend) -> None:
        if len(b.commands) == 5:
            arm.request_stop()

    be.on_sleep = stop_soon
    res = arm.move_joints({"elbow_flex": 1.0}, speed_scale=0.5)
    assert res.status == "stopped"
    assert be.commands[-1] == be.positions
    assert be.positions["elbow_flex"] < 0.5


def test_timeout_when_follower_never_reaches_goal(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    arm.acquire()

    def lagging(b: FakeArmBackend) -> None:
        # Follows the setpoint but stops 0.1 rad short (within tracking threshold, outside convergence tolerance).
        last = b.commands[-1]["elbow_flex"]
        b.positions["elbow_flex"] = min(last, 0.2)

    be.follow = False
    be.on_sleep = lagging
    res = arm.move_joints({"elbow_flex": 0.3}, speed_scale=0.5)
    assert res.status == "timeout"


def test_move_cartesian_unreachable_does_not_move(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    res = arm.move_cartesian(2.0, 0.0, 0.2, None)
    assert res.status == "unreachable"
    assert be.commands == []


def test_move_cartesian_reaches_target(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    q = {"shoulder_pan": 0.2, "shoulder_lift": -0.3, "elbow_flex": 0.5, "wrist_flex": 0.6, "wrist_roll": 0.0}
    target = KIN.forward(q)
    res = arm.move_cartesian(target.x, target.y, target.z, target.pitch)
    assert res.status == "converged", res.message
    reached = KIN.forward({j: be.positions[j] for j in KIN.joint_names})
    assert abs(reached.x - target.x) < 0.005 and abs(reached.z - target.z) < 0.005


def test_set_gripper_open_fraction(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    res = arm.set_gripper(open_fraction=1.0)
    assert res.status == "converged"
    assert be.commands[-1]["gripper"] == pytest.approx(CONFIG.arm.gripper_open_rad)
    arm.set_gripper(open_fraction=0.0)
    assert be.commands[-1]["gripper"] == pytest.approx(CONFIG.arm.gripper_closed_rad)


def test_set_gripper_close_until_effort_stops_on_contact(tmp_path: Path) -> None:
    be = FakeArmBackend()
    be.positions["gripper"] = CONFIG.arm.gripper_open_rad
    arm, be = make(tmp_path, be)

    def squeeze(b: FakeArmBackend) -> None:
        b.efforts["gripper"] = 500.0 if b.positions["gripper"] < 0.8 else 0.0

    be.on_sleep = squeeze
    res = arm.set_gripper(close_until_effort=True, effort_threshold=300.0)
    assert res.status == "grasped"
    assert 0.6 < be.commands[-1]["gripper"] < 0.85
    assert be.commands[-1] == be.positions


def test_set_gripper_close_without_contact(tmp_path: Path) -> None:
    be = FakeArmBackend()
    be.positions["gripper"] = 0.5
    arm, be = make(tmp_path, be)
    res = arm.set_gripper(close_until_effort=True, effort_threshold=300.0)
    assert res.status == "closed_no_contact"
    assert be.commands[-1]["gripper"] == pytest.approx(CONFIG.arm.gripper_closed_rad)


def test_set_gripper_requires_exactly_one_mode(tmp_path: Path) -> None:
    arm, _ = make(tmp_path)
    with pytest.raises(ArmError):
        arm.set_gripper()
    with pytest.raises(ArmError):
        arm.set_gripper(open_fraction=0.5, close_until_effort=True)
    with pytest.raises(ArmError):
        arm.set_gripper(open_fraction=1.5)


def test_home_requires_stored_pose(tmp_path: Path) -> None:
    arm, _ = make(tmp_path)
    with pytest.raises(ArmError):
        arm.home()


def test_set_home_then_home(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.positions.update({"elbow_flex": 0.3, "gripper": 0.2})
    stored = arm.set_home()
    assert stored["elbow_flex"] == pytest.approx(0.3)
    assert (tmp_path / "arm" / "home.yaml").is_file()
    be.positions.update({"elbow_flex": -0.2, "gripper": 0.6})
    res = arm.home()
    assert res.status == "converged"
    assert be.positions["elbow_flex"] == pytest.approx(0.3)
    assert be.positions["gripper"] == pytest.approx(0.2)


def test_set_home_refuses_stale(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.no_samples = True
    with pytest.raises(ArmError):
        arm.set_home()


def test_keepalive_republishes_last_setpoint_while_held(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    arm.move_joints({"elbow_flex": 0.2}, speed_scale=0.5)
    n = len(be.commands)
    arm.keepalive_tick()
    assert len(be.commands) == n + 1
    assert be.commands[-1]["elbow_flex"] == pytest.approx(0.2)


def test_keepalive_idle_without_lease(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    arm.keepalive_tick()
    assert be.commands == []


def test_lease_lost_when_filter_switches_source(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    arm.acquire()
    be.source = "leader"
    arm.keepalive_tick()
    # Within the grace period filter_node may not have switched to autonomy yet: keep the lease and keep alive.
    assert arm.control_held
    assert len(be.commands) == 2
    be.t += 1.0
    arm.keepalive_tick()
    assert not arm.control_held
    assert len(be.commands) == 2
    arm.keepalive_tick()
    assert len(be.commands) == 2


def test_motion_aborts_when_filter_switches_source(tmp_path: Path) -> None:
    arm, be = make(tmp_path)

    switched_at: list[int] = []

    def leader_takes_over(b: FakeArmBackend) -> None:
        if b.t > 100.8 and not switched_at:
            b.source = "leader"
            switched_at.append(len(b.commands))

    be.on_sleep = leader_takes_over
    res = arm.move_joints({"elbow_flex": 1.0}, speed_scale=0.5)
    assert res.status == "stopped"
    # No hold after losing the lease: the server must not fight the new source.
    assert len(be.commands) == switched_at[0]
    assert "leader" in res.message
    assert not arm.control_held


def test_hold_without_fresh_sample_publishes_nothing(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.no_samples = True
    assert arm.hold() is False
    assert be.commands == []


def test_hold_publishes_measured(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.positions["gripper"] = 0.7
    assert arm.hold() is True
    assert be.commands[-1]["gripper"] == 0.7


def test_state_reports_positions_efforts_and_lease(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.efforts["gripper"] = 42.0
    st = arm.state()
    assert st.positions == be.positions
    assert st.efforts["gripper"] == 42.0
    assert st.gripper_effort == 42.0
    assert st.joint_states_age_s == pytest.approx(0.0)
    assert st.active_source == "autonomy"
    assert st.control_held is False
    assert st.home_stored is False


def test_state_omits_stale_joint_data(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.stale_after = be.t - 5.0
    st = arm.state()
    assert st.positions is None and st.efforts is None
    assert st.joint_states_age_s == pytest.approx(5.0)


def test_state_reports_floor_z_below_the_arm_base(tmp_path: Path) -> None:
    arm, _ = make(tmp_path)
    assert arm.state().floor_z_m == pytest.approx(-0.104)


def test_state_floor_z_follows_configured_base_height(tmp_path: Path) -> None:
    cfg = CONFIG.model_copy(deep=True)
    cfg.arm.arm_base_height_m = 0.2
    cfg.arm.home_file = tmp_path / "home.yaml"
    arm = ArmController(FakeArmBackend(), KIN, load_joint_limits(cfg.arm.urdf_path), cfg)
    assert arm.state().floor_z_m == pytest.approx(-0.2)


# --- sag ratchet: unnamed joints keep the commanded target; small steady-state errors settle, not time out ---

SAG = -0.05  # shoulder_lift ends 0.05 rad short of its command under gravity load (on-robot: 0.05-0.06)


def test_gripper_closed_default_is_a_follower_joint_position(tmp_path: Path) -> None:
    """Gripper targets are follower joint radians (as in /follower/joint_states): closed is near the URDF lower
    limit (measured closed: -0.172 rad), not 0.0 (which is ~10 deg open on the follower)."""
    arm, be = make(tmp_path)
    lo = load_joint_limits(CONFIG.arm.urdf_path)["gripper"][0] + CONFIG.limits.margin_for("gripper")
    res = arm.set_gripper(open_fraction=0.0)
    assert be.commands[-1]["gripper"] == pytest.approx(CONFIG.arm.gripper_closed_rad)
    assert res.clamped == []
    assert lo <= CONFIG.arm.gripper_closed_rad < -0.15


@pytest.mark.parametrize("close_until_effort", [True, False])
def test_gripper_closes_fully_to_the_measured_closed_position(tmp_path: Path, close_until_effort: bool) -> None:
    """Physical closed is -0.172 rad: the commanded closed target (-0.165) must not be clamped to -0.1245."""
    be = FakeArmBackend()
    be.positions["gripper"] = 0.5
    arm, be = make(tmp_path, be)
    if close_until_effort:
        arm.set_gripper(close_until_effort=True, effort_threshold=300.0)
    else:
        arm.set_gripper(open_fraction=0.0)
    assert be.commands[-1]["gripper"] == pytest.approx(-0.165)


def test_limit_margin_override_applies_per_joint(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    arm.cfg.limits.arm_limit_margin_overrides = {"wrist_flex": 0.0}
    hi = load_joint_limits(CONFIG.arm.urdf_path)["wrist_flex"][1]
    arm.move_joints({"wrist_flex": 3.0}, speed_scale=0.5)
    assert be.commands[-1]["wrist_flex"] == pytest.approx(hi)


def test_small_steady_state_error_settles_as_converged_with_residual(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.sag = {"shoulder_lift": SAG}
    res = arm.move_joints({"shoulder_lift": 0.5}, speed_scale=0.5)
    assert res.status == "converged", res.message
    assert "settled with residual error" in res.message
    assert res.residual_error == {"shoulder_lift": pytest.approx(-SAG)}
    # The target stays commanded (no hold at the sagged measured pose) and the keepalive keeps republishing it.
    assert be.commands[-1]["shoulder_lift"] == pytest.approx(0.5)
    arm.keepalive_tick()
    assert be.commands[-1]["shoulder_lift"] == pytest.approx(0.5)


def test_settled_motion_returns_before_the_converge_timeout(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.sag = {"shoulder_lift": SAG}
    res = arm.move_joints({"shoulder_lift": 0.5}, speed_scale=0.5)
    assert res.status == "converged"
    assert res.duration_s < 1.0 + CONFIG.timeouts.arm_converge_timeout_s / 2


def test_exact_convergence_reports_no_residual(tmp_path: Path) -> None:
    arm, _ = make(tmp_path)
    res = arm.move_joints({"elbow_flex": 0.3}, speed_scale=0.5)
    assert res.status == "converged"
    assert res.residual_error == {}
    assert res.message == "reached target"


def test_unnamed_joints_keep_last_commanded_target_not_sagged_measured(tmp_path: Path) -> None:
    """Regression (sag ratchet): a second motion must not lock the sagged measured shoulder_lift in."""
    arm, be = make(tmp_path)
    be.sag = {"shoulder_lift": SAG}
    arm.move_joints({"shoulder_lift": 0.5}, speed_scale=0.5)
    n = len(be.commands)
    res = arm.move_joints({"elbow_flex": 0.3}, speed_scale=0.5)
    assert res.status == "converged", res.message
    assert all(c["shoulder_lift"] == pytest.approx(0.5) for c in be.commands[n:])
    assert be.commands[-1]["elbow_flex"] == pytest.approx(0.3)


def test_repeated_motions_do_not_accumulate_sag(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.sag = {"shoulder_lift": SAG, "elbow_flex": SAG}
    arm.move_joints({"shoulder_lift": 0.5, "elbow_flex": 0.4}, speed_scale=0.5)
    for i in range(5):
        arm.move_joints({"wrist_flex": 0.1 * (i % 2)}, speed_scale=0.5)
    assert be.commands[-1]["shoulder_lift"] == pytest.approx(0.5)
    assert be.commands[-1]["elbow_flex"] == pytest.approx(0.4)
    assert be.positions["shoulder_lift"] == pytest.approx(0.5 + SAG)


def test_named_joint_trajectory_starts_at_commanded_not_measured(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.sag = {"shoulder_lift": SAG}
    arm.move_joints({"shoulder_lift": 0.5}, speed_scale=0.5)
    n = len(be.commands)
    arm.move_joints({"shoulder_lift": 0.6}, speed_scale=0.5)
    assert all(c["shoulder_lift"] >= 0.5 - 1e-9 for c in be.commands[n:])


def test_cartesian_keeps_commanded_gripper_and_wrist_roll(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.sag = {"gripper": -0.03, "wrist_roll": -0.03}
    arm.move_joints({"gripper": 0.5, "wrist_roll": 0.2}, speed_scale=0.5)
    n = len(be.commands)
    q = {"shoulder_pan": 0.1, "shoulder_lift": -0.2, "elbow_flex": 0.4, "wrist_flex": 0.5, "wrist_roll": 0.2}
    target = KIN.forward(q)
    res = arm.move_cartesian(target.x, target.y, target.z, target.pitch)
    assert res.status == "converged", res.message
    assert all(c["gripper"] == pytest.approx(0.5) for c in be.commands[n:])
    assert all(c["wrist_roll"] == pytest.approx(0.2) for c in be.commands[n:])


def test_new_lease_falls_back_to_measured_pose(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.sag = {"shoulder_lift": SAG}
    arm.move_joints({"shoulder_lift": 0.5}, speed_scale=0.5)
    arm.release()
    n = len(be.commands)
    arm.move_joints({"elbow_flex": 0.2}, speed_scale=0.5)
    # Nothing commanded in the new lease yet: unnamed joints start from the measured pose.
    assert be.commands[n]["shoulder_lift"] == pytest.approx(0.5 + SAG)


def test_large_steady_state_error_still_times_out_and_holds_measured(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.sag = {"shoulder_lift": -(CONFIG.limits.settle_tolerance_for("shoulder_lift") + 0.02)}
    res = arm.move_joints({"shoulder_lift": 0.5}, speed_scale=0.5)
    assert res.status == "timeout"
    assert be.commands[-1] == res.positions  # held at the measured pose (the fake then sags further)
    assert res.residual_error == {}


def test_moving_joint_within_settle_band_is_not_settled(tmp_path: Path) -> None:
    """A joint still moving (here oscillating) inside the settle band is not a steady-state error."""
    arm, be = make(tmp_path)
    arm.acquire()
    be.follow = False
    ticks = [0]

    def wobble(b: FakeArmBackend) -> None:
        ticks[0] += 1
        last = b.commands[-1]["wrist_flex"]
        b.positions["wrist_flex"] = last - 0.055 + (0.015 if ticks[0] % 2 else -0.015)  # error 0.04 / 0.07

    be.on_sleep = wobble
    res = arm.move_joints({"wrist_flex": 0.3}, speed_scale=0.5)
    assert res.status == "timeout"


def test_tracking_abort_still_holds_measured_with_settle_enabled(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    lim = CONFIG.limits
    be.sag = {"shoulder_lift": -(tracking_limit(lim.arm_tracking_error_rad, lim.arm_tracking_lag_s, 1.0) + 0.05)}
    res = arm.move_joints({"shoulder_lift": 0.5}, speed_scale=0.5)
    assert res.status == "aborted_tracking"
    assert be.commands[-1] == res.positions


def test_home_settles_with_residual_and_keeps_target(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.positions.update({"shoulder_lift": 0.4})
    arm.set_home()
    be.positions.update({"shoulder_lift": 0.0})
    be.sag = {"shoulder_lift": -0.047}
    arm.acquire()
    res = arm.home()
    assert res.status == "converged", res.message
    assert res.residual_error == {"shoulder_lift": pytest.approx(0.047)}


# --- bounded residual hold: a stalled joint is not pushed at its target forever ---

HOLD_S = CONFIG.limits.arm_settle_hold_s


def settle_shoulder(tmp_path: Path) -> tuple[ArmController, FakeArmBackend]:
    """Move shoulder_lift to 0.5 with 0.05 rad sag: settles with a residual error."""
    arm, be = make(tmp_path)
    be.sag = {"shoulder_lift": SAG}
    res = arm.move_joints({"shoulder_lift": 0.5}, speed_scale=0.5)
    assert res.residual_error, res.message
    return arm, be


def test_residual_hold_keeps_target_within_hold_window(tmp_path: Path) -> None:
    arm, be = settle_shoulder(tmp_path)
    be.t += HOLD_S * 0.5
    arm.keepalive_tick()
    assert be.commands[-1]["shoulder_lift"] == pytest.approx(0.5)


def test_residual_hold_relaxes_to_measured_after_hold_window(tmp_path: Path) -> None:
    arm, be = settle_shoulder(tmp_path)
    measured = be.positions["shoulder_lift"]
    be.t += HOLD_S + 0.1
    arm.keepalive_tick()
    assert be.commands[-1]["shoulder_lift"] == pytest.approx(measured)
    assert be.commands[-1]["elbow_flex"] == pytest.approx(0.0)  # joints without residual keep their target
    arm.keepalive_tick()
    assert be.commands[-1]["shoulder_lift"] == pytest.approx(measured)  # relaxed once, not ratcheting further


def test_residual_hold_not_relaxed_when_joint_reached_target_meanwhile(tmp_path: Path) -> None:
    arm, be = settle_shoulder(tmp_path)
    be.sag = {}
    be.positions["shoulder_lift"] = 0.5
    be.t += HOLD_S + 0.1
    arm.keepalive_tick()
    assert be.commands[-1]["shoulder_lift"] == pytest.approx(0.5)


def test_next_motion_starts_from_intended_target_after_relax(tmp_path: Path) -> None:
    arm, be = settle_shoulder(tmp_path)
    be.t += HOLD_S + 0.1
    arm.keepalive_tick()
    n = len(be.commands)
    arm.move_joints({"elbow_flex": 0.3}, speed_scale=0.5)
    assert all(c["shoulder_lift"] == pytest.approx(0.5) for c in be.commands[n:])


@pytest.mark.parametrize("end", ["release", "drop_lease", "lease_lost"])
def test_intent_and_pending_relax_cleared_when_lease_ends(tmp_path: Path, end: str) -> None:
    arm, be = settle_shoulder(tmp_path)
    if end == "release":
        arm.release()
    elif end == "drop_lease":
        arm.drop_lease()
    else:
        be.source = "leader"
        be.t += 1.0
        arm.keepalive_tick()
        assert not arm.control_held
        be.source = "autonomy"
    assert arm._last_target is None and arm._relax_hold is None
    measured = be.positions["shoulder_lift"]
    n = len(be.commands)
    arm.move_joints({"elbow_flex": 0.2}, speed_scale=0.5)
    assert be.commands[n]["shoulder_lift"] == pytest.approx(measured)


# --- gripper stall on an object counts as a grasp ---

JAW_STALL = -0.12  # jaw stops on an object 0.045 rad before the closed target (within the settle tolerance)


def stall_jaw_at(stop: float) -> FakeArmBackend:
    """Follower whose jaw cannot close beyond `stop` (an object between the fingers, no load reported)."""
    be = FakeArmBackend()
    be.positions["gripper"] = 0.5

    def blocked(b: FakeArmBackend) -> None:
        b.positions["gripper"] = max(b.positions["gripper"], stop)

    be.on_sleep = blocked
    return be


@pytest.mark.parametrize("close_until_effort", [True, False])
def test_gripper_stall_before_closed_is_reported_as_grasp(tmp_path: Path, close_until_effort: bool) -> None:
    arm, be = make(tmp_path, stall_jaw_at(JAW_STALL))
    if close_until_effort:
        res = arm.set_gripper(close_until_effort=True, effort_threshold=300.0)
    else:
        res = arm.set_gripper(open_fraction=0.0)
    assert res.status == "grasped", res.message
    assert "stalled" in res.message
    squeeze = CONFIG.limits.gripper_grasp_squeeze_rad
    # Hold at the stall position plus a small squeeze toward closed, not at the full closed target.
    assert be.commands[-1]["gripper"] == pytest.approx(JAW_STALL - squeeze)
    be.t += HOLD_S + 1.0
    arm.keepalive_tick()
    assert be.commands[-1]["gripper"] == pytest.approx(JAW_STALL - squeeze)


def test_gripper_grasp_squeeze_never_passes_closed(tmp_path: Path) -> None:
    be = stall_jaw_at(-0.13)
    arm, be = make(tmp_path, be)
    arm.cfg.limits.gripper_grasp_squeeze_rad = 0.1
    res = arm.set_gripper(open_fraction=0.0)
    assert res.status == "grasped", res.message
    assert be.commands[-1]["gripper"] == pytest.approx(CONFIG.arm.gripper_closed_rad)


def test_partial_open_fraction_stall_is_not_a_grasp(tmp_path: Path) -> None:
    arm, be = make(tmp_path, stall_jaw_at(0.6))
    be.positions["gripper"] = 1.0
    res = arm.set_gripper(open_fraction=0.3)
    assert res.status != "grasped"


# --- gripper motions keep holding the arm's intended targets ---


def test_gripper_motion_restores_a_relaxed_arm_hold_and_rearms_the_relax(tmp_path: Path) -> None:
    arm, be = settle_shoulder(tmp_path)
    be.t += HOLD_S + 0.1
    arm.keepalive_tick()  # relaxed to the sagged pose before the gripper tool is called
    n = len(be.commands)
    arm.set_gripper(open_fraction=0.5)
    assert all(c["shoulder_lift"] == pytest.approx(0.5) for c in be.commands[n:])
    arm.keepalive_tick()
    assert be.commands[-1]["shoulder_lift"] == pytest.approx(0.5)  # still held right after the gripper finished
    be.t += HOLD_S * 0.5
    arm.keepalive_tick()
    assert be.commands[-1]["shoulder_lift"] == pytest.approx(0.5)
    measured = be.positions["shoulder_lift"]
    be.t += HOLD_S * 0.6
    arm.keepalive_tick()  # arm_settle_hold_s after the gripper motion finished
    assert be.commands[-1]["shoulder_lift"] == pytest.approx(measured)


def test_gripper_motion_does_not_arm_a_relax_for_a_converged_arm(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    arm.move_joints({"shoulder_lift": 0.5}, speed_scale=0.5)
    arm.set_gripper(open_fraction=0.5)
    be.t += HOLD_S + 0.1
    arm.keepalive_tick()
    assert arm._relax_hold is None
    assert be.commands[-1]["shoulder_lift"] == pytest.approx(0.5)


def test_grasp_on_effort_holds_the_arm_intent_not_the_sagged_pose(tmp_path: Path) -> None:
    arm, be = settle_shoulder(tmp_path)
    be.positions["gripper"] = CONFIG.arm.gripper_open_rad
    be.on_sleep = lambda b: b.efforts.update(gripper=500.0 if b.positions["gripper"] < 0.8 else 0.0)
    res = arm.set_gripper(close_until_effort=True, effort_threshold=300.0)
    assert res.status == "grasped"
    assert be.commands[-1]["shoulder_lift"] == pytest.approx(0.5)  # not the sagged 0.45
    assert be.commands[-1]["gripper"] == pytest.approx(be.positions["gripper"])


# --- close_until_effort ignores the motor-start load spike ---


def test_effort_spike_at_motor_start_is_not_contact(tmp_path: Path) -> None:
    be = FakeArmBackend()
    be.positions["gripper"] = CONFIG.arm.gripper_open_rad
    arm, be = make(tmp_path, be)
    t0 = be.t
    be.on_sleep = lambda b: b.efforts.update(gripper=500.0 if b.t < t0 + 0.2 else 0.0)
    res = arm.set_gripper(close_until_effort=True, effort_threshold=300.0)
    assert res.status == "closed_no_contact", res.message
    assert be.commands[-1]["gripper"] == pytest.approx(CONFIG.arm.gripper_closed_rad)


def test_effort_without_jaw_travel_or_stall_is_not_contact(tmp_path: Path) -> None:
    """Past the ignore window but the jaw moved < 0.03 rad and is still creeping: not contact yet."""
    arm, _ = make(tmp_path)
    history = [(100.0 + 0.1 * i, 1.5 - 0.004 * i, 1.0) for i in range(8)]  # 0.028 rad over 0.7 s, still moving
    assert arm.contact_allowed(history, started=99.0) is False
    assert arm.contact_allowed(history, started=100.6) is False  # inside the ignore window


def test_effort_counts_after_jaw_travel_or_stall(tmp_path: Path) -> None:
    arm, _ = make(tmp_path)
    moved = [(100.0 + 0.1 * i, 1.5 - 0.02 * i, 1.2) for i in range(8)]
    assert arm.contact_allowed(moved, started=99.0) is True
    stalled = [(100.0 + 0.1 * i, 0.7, 0.5) for i in range(8)]  # commanded 0.2 rad further closed
    assert arm.contact_allowed(stalled, started=99.0) is True
    short = [(100.0 + 0.1 * i, 0.7, 0.5) for i in range(3)]  # stall window (0.5 s) not covered yet
    assert arm.contact_allowed(short, started=99.0) is False


def test_still_jaw_is_not_stalled_until_the_command_leads_it(tmp_path: Path) -> None:
    """A jaw that has not broken away yet while the close command ramps up is not stalled."""
    arm, _ = make(tmp_path)
    lead = arm.cfg.limits.gripper_stall_lead_rad
    ramping = [(100.0 + 0.1 * i, 0.66, 0.66 - 0.02 * i) for i in range(10)]  # lead 0.0 .. 0.18 rad
    assert arm.contact_allowed(ramping, started=99.0) is False  # window starts below the lead
    led = [(100.0 + 0.1 * i, 0.66, 0.66 - lead - 0.01) for i in range(8)]
    assert arm.contact_allowed(led, started=99.0) is True


# --- joint zero offsets: kinematics in URDF space (urdf = measured + offset), joint commands stay measured ---

OFFSETS = {"shoulder_pan": 0.05, "shoulder_lift": -0.12, "elbow_flex": 0.09, "wrist_flex": 0.2, "wrist_roll": -0.07}
KIN_OFF = ArmKinematics(CONFIG.arm.urdf_path, margin=CONFIG.limits.arm_limit_margin_rad, joint_offsets=OFFSETS)


def make_offset(tmp_path: Path, backend: FakeArmBackend | None = None) -> tuple[ArmController, FakeArmBackend]:
    be = backend or FakeArmBackend()
    cfg = CONFIG.model_copy(deep=True)
    cfg.arm.home_file = tmp_path / "arm" / "home.yaml"
    return ArmController(be, KIN_OFF, load_joint_limits(cfg.arm.urdf_path), cfg), be


def test_state_reports_measured_positions_and_offset_corrected_tool_pose(tmp_path: Path) -> None:
    be = FakeArmBackend()
    be.positions.update({"shoulder_lift": 0.3, "elbow_flex": 0.5, "wrist_flex": 0.4})
    arm, _ = make_offset(tmp_path, be)
    state = arm.state()
    assert state.positions == be.positions
    plain = ArmController(be, KIN, load_joint_limits(CONFIG.arm.urdf_path), CONFIG).state().tool_pose
    pose = KIN.forward({j: be.positions[j] + OFFSETS[j] for j in OFFSETS})
    assert state.tool_pose == pytest.approx({"x": pose.x, "y": pose.y, "z": pose.z, "pitch": pose.pitch})
    assert state.tool_pose["pitch"] != pytest.approx(plain["pitch"], abs=0.05)


def test_move_joints_clamps_in_urdf_space_and_commands_measured_space(tmp_path: Path) -> None:
    arm, be = make_offset(tmp_path)
    res = arm.move_joints({"wrist_flex": 3.0, "elbow_flex": 0.4}, speed_scale=0.5)
    hi = load_joint_limits(CONFIG.arm.urdf_path)["wrist_flex"][1] - CONFIG.limits.arm_limit_margin_rad
    assert be.commands[-1]["wrist_flex"] == pytest.approx(hi - OFFSETS["wrist_flex"])
    assert be.commands[-1]["elbow_flex"] == pytest.approx(0.4)  # in range: the agent's measured-space target as is
    assert res.clamped == ["wrist_flex"]


def test_move_cartesian_reaches_the_target_with_offsets(tmp_path: Path) -> None:
    arm, be = make_offset(tmp_path)
    q = {"shoulder_pan": 0.2, "shoulder_lift": 0.2, "elbow_flex": 0.3, "wrist_flex": 0.4, "wrist_roll": 0.0}
    target = KIN_OFF.forward(q)
    res = arm.move_cartesian(target.x, target.y, target.z, target.pitch, speed_scale=0.5)
    assert res.status == "converged", res.message
    assert res.achieved_tool_pose == pytest.approx(
        {"x": target.x, "y": target.y, "z": target.z, "pitch": target.pitch}, abs=0.004
    )
    final = KIN_OFF.forward({j: be.commands[-1][j] for j in q})
    assert (final.x, final.y, final.z) == pytest.approx((target.x, target.y, target.z), abs=0.003)


# --- stricter grasp detection: the jaw must have closed onto something ---


def stall_from(start: float, stop: float, effort: float = 0.0) -> FakeArmBackend:
    """Follower whose jaw starts at `start` and cannot close beyond `stop` (optionally reporting `effort` there)."""
    be = FakeArmBackend()
    be.positions["gripper"] = start

    def blocked(b: FakeArmBackend) -> None:
        b.positions["gripper"] = max(b.positions["gripper"], stop)
        b.efforts["gripper"] = effort if b.positions["gripper"] <= stop + 1e-9 else 0.0

    be.on_sleep = blocked
    return be


def close_on_load(arm: ArmController) -> ArmMotionResult:
    return arm.set_gripper(close_until_effort=True, effort_threshold=300.0)


def test_tiny_jaw_travel_under_load_is_blocked_not_grasped(tmp_path: Path) -> None:
    arm, be = make(tmp_path, stall_from(1.5, 1.49, effort=500.0))
    res = close_on_load(arm)
    assert res.status == "blocked", res.message
    pos = be.positions["gripper"]
    assert res.message == (
        f"jaw stopped at {pos:.3f} rad after {1.5 - pos:.3f} rad travel "
        "- likely pressing on an object rather than holding it"
    )
    assert be.commands[-1]["gripper"] == pytest.approx(pos)  # holds the measured jaw, no squeeze


def test_barely_moved_jaw_stall_is_blocked_without_squeeze(tmp_path: Path) -> None:
    """Stall path (no load reported): the jaw started near closed and moved 0.07 rad only."""
    be = stall_from(-0.05, -0.12)
    arm, be = make(tmp_path, be)
    res = arm.set_gripper(open_fraction=0.0)
    assert res.status == "blocked", res.message
    assert "after 0.070 rad travel" in res.message
    assert be.commands[-1]["gripper"] == pytest.approx(-0.12)


def test_stall_far_into_the_travel_is_grasped(tmp_path: Path) -> None:
    arm, be = make(tmp_path, stall_from(1.5, 0.4, effort=500.0))
    res = close_on_load(arm)
    assert res.status == "grasped", res.message
    assert be.commands[-1]["gripper"] == pytest.approx(0.4)


def test_stall_near_fully_open_is_blocked_even_with_travel(tmp_path: Path) -> None:
    arm, _ = make(tmp_path, stall_from(1.5, 1.3, effort=500.0))
    res = close_on_load(arm)
    assert res.status == "blocked", res.message
    assert "jaw stopped at 1.300 rad after 0.200 rad travel" in res.message


def test_grasp_thresholds_come_from_config(tmp_path: Path) -> None:
    arm, _ = make(tmp_path, stall_from(1.5, 1.0, effort=500.0))
    assert close_on_load(arm).status == "grasped"
    arm, _ = make(tmp_path, stall_from(1.5, 1.0, effort=500.0))
    arm.cfg.limits.gripper_grasp_min_travel_rad = 0.6
    assert close_on_load(arm).status == "blocked"
    arm, _ = make(tmp_path, stall_from(1.5, 1.0, effort=500.0))
    arm.cfg.limits.gripper_grasp_max_open_rad = 0.9
    assert close_on_load(arm).status == "blocked"
    assert McpServerConfig().limits.gripper_grasp_min_travel_rad == 0.15
    assert McpServerConfig().limits.gripper_grasp_max_open_rad == 1.2


# --- wrist roll guard: no big roll with a wide open gripper ---

WIDE_GRIPPER = 1.2  # rad, above limits.roll_max_gripper_open_rad (0.8)
HALF_GRIPPER = 0.67  # rad, open_fraction 0.5


def test_roll_is_refused_with_a_wide_open_gripper_and_nothing_moves(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.positions["gripper"] = WIDE_GRIPPER
    with pytest.raises(ArmError, match=r"open_fraction about 0\.5") as exc:
        arm.move_joints({"wrist_roll": 1.0})
    assert "lift the arm clear" in str(exc.value)
    assert be.commands == [] and not arm.control_held


def test_roll_is_refused_when_the_same_call_opens_the_gripper_wide(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    with pytest.raises(ArmError, match="gripper"):
        arm.move_joints({"wrist_roll": 1.0, "gripper": 1.5})
    assert be.commands == []


def test_roll_guard_uses_the_larger_of_measured_and_targeted_gripper(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.positions["gripper"] = WIDE_GRIPPER
    with pytest.raises(ArmError):
        arm.move_joints({"wrist_roll": 1.0, "gripper": 0.0})
    assert be.commands == []


def test_roll_is_allowed_with_a_half_open_gripper(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.positions["gripper"] = HALF_GRIPPER
    res = arm.move_joints({"wrist_roll": 1.2})
    assert res.status == "converged", res.message
    assert be.commands[-1]["wrist_roll"] == pytest.approx(1.2)


def test_small_roll_change_is_allowed_with_a_wide_open_gripper(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.positions["gripper"] = WIDE_GRIPPER
    assert arm.move_joints({"wrist_roll": 0.08}).status == "converged"
    assert arm.move_joints({"wrist_roll": 0.0, "elbow_flex": 0.3}).status == "converged"


def test_other_joints_move_with_a_wide_open_gripper(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.positions["gripper"] = WIDE_GRIPPER
    assert arm.move_joints({"elbow_flex": 0.5}).status == "converged"


def test_roll_guard_thresholds_come_from_config(tmp_path: Path) -> None:
    be = FakeArmBackend()
    be.positions["gripper"] = 0.5
    cfg = CONFIG.model_copy(deep=True)
    cfg.limits.roll_max_gripper_open_rad = 0.4
    cfg.limits.roll_guard_min_change_rad = 0.5
    arm = ArmController(be, KIN, load_joint_limits(cfg.arm.urdf_path), cfg)
    assert arm.move_joints({"wrist_roll": 0.4}).status == "converged"  # change below the guard minimum
    with pytest.raises(ArmError):
        arm.move_joints({"wrist_roll": 1.5})


# --- move_cartesian: wrist_roll and object_width_m ---

CARTESIAN_Q = {"shoulder_pan": 0.2, "shoulder_lift": -0.3, "elbow_flex": 0.5, "wrist_flex": 0.6, "wrist_roll": 0.0}


def test_move_cartesian_wrist_roll_replaces_the_current_roll(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    target = KIN.forward(CARTESIAN_Q)
    res = arm.move_cartesian(target.x, target.y, target.z, target.pitch, wrist_roll=-1.57)
    assert res.status == "converged", res.message
    assert be.commands[-1]["wrist_roll"] == pytest.approx(-1.57)
    assert be.positions["wrist_roll"] == pytest.approx(-1.57)
    assert res.clamped == []


def test_move_cartesian_wrist_roll_is_clamped_to_the_limits(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    target = KIN.forward(CARTESIAN_Q)
    res = arm.move_cartesian(target.x, target.y, target.z, target.pitch, wrist_roll=-3.5)
    lo = load_joint_limits(CONFIG.arm.urdf_path)["wrist_roll"][0] + CONFIG.limits.arm_limit_margin_rad
    assert be.commands[-1]["wrist_roll"] == pytest.approx(lo)
    assert "wrist_roll" in res.clamped


def test_move_cartesian_wrist_roll_is_refused_with_a_wide_open_gripper(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.positions["gripper"] = WIDE_GRIPPER
    target = KIN.forward(CARTESIAN_Q)
    with pytest.raises(ArmError, match="half open|open_fraction"):
        arm.move_cartesian(target.x, target.y, target.z, target.pitch, wrist_roll=-1.57)
    assert be.commands == []


def test_move_cartesian_without_wrist_roll_keeps_the_commanded_roll(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    arm.move_joints({"wrist_roll": 0.9})
    target = KIN.forward(CARTESIAN_Q | {"wrist_roll": 0.9})
    arm.move_cartesian(target.x, target.y, target.z, target.pitch)
    assert be.commands[-1]["wrist_roll"] == pytest.approx(0.9)


def test_move_cartesian_object_width_targets_the_object_centre(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    width = 0.04
    target = KIN.forward(CARTESIAN_Q)
    res = arm.move_cartesian(target.x, target.y, target.z, target.pitch, wrist_roll=-1.57, object_width_m=width)
    assert res.status == "converged", res.message
    final = {j: be.commands[-1][j] for j in KIN.joint_names}
    shift = grasp_offset(width, tuple(CONFIG.arm.jaw_open_axis))
    centre = KIN.forward(final, extra_offset=shift)
    assert (centre.x, centre.y, centre.z) == pytest.approx((target.x, target.y, target.z), abs=0.003)
    tool = KIN.forward(final)
    assert math.dist((tool.x, tool.y, tool.z), (target.x, target.y, target.z)) == pytest.approx(width / 2, abs=0.003)
    assert res.grasp_shift is not None
    assert res.grasp_shift["object_width_m"] == width
    assert res.grasp_shift["shift_m"] == pytest.approx(width / 2)
    assert res.grasp_shift["tool_point"] == pytest.approx({"x": tool.x, "y": tool.y, "z": tool.z}, abs=0.003)
    assert res.expected_tool_pose == pytest.approx({"x": target.x, "y": target.y, "z": target.z, "pitch": target.pitch})


def test_move_cartesian_without_object_width_reports_no_grasp_shift(tmp_path: Path) -> None:
    arm, _ = make(tmp_path)
    target = KIN.forward(CARTESIAN_Q)
    assert arm.move_cartesian(target.x, target.y, target.z, target.pitch).grasp_shift is None


@pytest.mark.parametrize("width", [0.0, -0.01, 0.09, float("nan")])
def test_move_cartesian_rejects_bad_object_widths(tmp_path: Path, width: float) -> None:
    arm, be = make(tmp_path)
    with pytest.raises(ArmError, match="object_width_m"):
        arm.move_cartesian(0.2, 0.0, 0.0, None, object_width_m=width)
    assert be.commands == []


# --- joint limit overrides ---


def test_limit_overrides_apply_to_the_clamp_of_move_joints(tmp_path: Path) -> None:
    be = FakeArmBackend()
    cfg = CONFIG.model_copy(deep=True)
    cfg.arm.joint_limit_overrides_rad = {"shoulder_lift": (-1.74533, 2.6)}
    arm = ArmController(be, KIN, load_joint_limits(cfg.arm.urdf_path), cfg)
    res = arm.move_joints({"shoulder_lift": 2.4})
    assert be.commands[-1]["shoulder_lift"] == pytest.approx(2.4) and res.clamped == []
    res = arm.move_joints({"shoulder_lift": 3.0})
    assert be.commands[-1]["shoulder_lift"] == pytest.approx(2.6 - cfg.limits.arm_limit_margin_rad)
    assert res.clamped == ["shoulder_lift"]
    plain, plain_be = make(tmp_path)
    plain.move_joints({"shoulder_lift": 2.4})
    assert plain_be.commands[-1]["shoulder_lift"] == pytest.approx(1.74533 - CONFIG.limits.arm_limit_margin_rad)


def test_move_cartesian_below_the_floor_with_wider_limits(tmp_path: Path) -> None:
    be = FakeArmBackend()
    cfg = CONFIG.model_copy(deep=True)
    cfg.arm.joint_limit_overrides_rad = {"shoulder_lift": (-1.74533, 2.6)}
    kin = ArmKinematics(
        cfg.arm.urdf_path,
        margin=cfg.limits.arm_limit_margin_rad,
        limit_overrides=cfg.arm.joint_limit_overrides_rad,
    )
    arm = ArmController(be, kin, load_joint_limits(cfg.arm.urdf_path), cfg)
    res = arm.move_cartesian(0.2, 0.0, -0.25, None)
    assert res.status == "converged", res.message
    reached = kin.forward({j: be.positions[j] for j in kin.joint_names})
    assert (reached.x, reached.z) == pytest.approx((0.2, -0.25), abs=0.005)
    narrow, narrow_be = make(tmp_path)
    assert narrow.move_cartesian(0.2, 0.0, -0.25, None).status == "unreachable"


def test_tracking_limit_grows_with_the_commanded_speed() -> None:
    """Servos lag more at speed: the abort threshold is base + lag_s * velocity."""
    assert tracking_limit(0.25, 0.25, 1.0) == pytest.approx(0.5)
    assert tracking_limit(0.25, 0.25, 0.5) == pytest.approx(0.375)
    assert tracking_limit(0.25, 0.0, 1.0) == pytest.approx(0.25)


def test_fast_motion_tolerates_lag_within_its_speed_scaled_limit(tmp_path: Path) -> None:
    """A 0.4 rad lag at full speed (1.0 rad/s, limit 0.5 rad) is not a tracking abort (2026-10-08: 0.35-0.37 rad
    lag aborted fast moves)."""
    arm, be = make(tmp_path)
    be.sag = {"shoulder_lift": -0.4}
    res = arm.move_joints({"shoulder_lift": 0.8}, speed_scale=0.5)
    assert res.status != "aborted_tracking"


def test_slow_motion_still_aborts_on_the_same_lag(tmp_path: Path) -> None:
    """At 0.2 rad/s the limit is 0.3 rad: the same 0.4 rad lag aborts (a jam or collision)."""
    arm, be = make(tmp_path)
    be.sag = {"shoulder_lift": -0.4}
    res = arm.move_joints({"shoulder_lift": 0.8}, speed_scale=0.1)
    assert res.status == "aborted_tracking"


# --- below-surface slow zone (floor_guard) -------------------------------------------------------------------------

LOW_SEED = {"shoulder_pan": 0.0, "shoulder_lift": 1.0, "elbow_flex": 0.5, "wrist_flex": 0.0, "wrist_roll": 0.0}


def low_pose(z_above_floor: float = 0.005) -> dict[str, float]:
    """Arm joints with the tool point (pointing down) z_above_floor above the robot plane, 20 cm in front."""
    return KIN.inverse(0.2, 0.0, CONFIG.arm.floor_z_m + z_above_floor, math.pi / 2, LOW_SEED)


def make_guarded(
    tmp_path: Path,
    enabled: bool = True,
    tilt: TiltSample | None = None,
    backend: FakeArmBackend | None = None,
) -> tuple[ArmController, FakeArmBackend]:
    be = backend or FakeArmBackend()
    cfg = CONFIG.model_copy(deep=True)
    cfg.arm.home_file = tmp_path / "arm" / "home.yaml"
    cfg.floor_guard.enabled = enabled
    arm = ArmController(be, KIN, load_joint_limits(cfg.arm.urdf_path), cfg, tilt_source=lambda: tilt)
    return arm, be


def test_motion_into_the_slow_zone_is_slowed_and_reported(tmp_path: Path) -> None:
    target = low_pose()
    plain, be_plain = make_guarded(tmp_path, enabled=False)
    assert plain.move_joints(target, 0.5).slow_zone is None
    arm, be = make_guarded(tmp_path)
    res = arm.move_joints(target, 0.5)
    assert res.status == "converged", res.message
    assert len(be.commands) > len(be_plain.commands) + 5
    assert res.slow_zone is not None and res.slow_zone["slowed_samples"] > 0
    assert res.slow_zone["speed_scale"] == 0.2 and res.slow_zone["tilt_source"] == "none"
    # Slowed steps move at most slow_speed_scale of the velocity cap per tick (25 Hz streaming kept).
    rate, vmax = CONFIG.limits.arm_rate_hz, CONFIG.limits.arm_max_joint_velocity_rps
    tail = be.commands[-6:]
    for a, b in zip(tail, tail[1:], strict=False):
        assert max(abs(b[j] - a[j]) for j in KIN.joint_names) * rate <= vmax * 0.2 * 1.05


def test_motion_above_the_slow_zone_runs_at_normal_speed(tmp_path: Path) -> None:
    target = low_pose(0.10)
    plain, be_plain = make_guarded(tmp_path, enabled=False)
    plain.move_joints(target, 0.5)
    arm, be = make_guarded(tmp_path)
    res = arm.move_joints(target, 0.5)
    assert res.slow_zone is None
    assert len(be.commands) == len(be_plain.commands)


def test_surface_override_allows_normal_speed_down_to_a_lower_surface(tmp_path: Path) -> None:
    target = low_pose(-0.03)  # 3 cm below the robot plane (a step down)
    plain, be_plain = make_guarded(tmp_path, enabled=False)
    plain.move_joints(target, 0.5)
    arm, be = make_guarded(tmp_path)
    res = arm.move_joints(target, 0.5, floor=FloorOverride(surface_z_m=-0.18))
    assert res.slow_zone is None
    assert len(be.commands) == len(be_plain.commands)


def test_imu_tilt_and_tilt_override_reach_the_guard(tmp_path: Path) -> None:
    target = low_pose(0.06)  # 6 cm above the robot plane: normal speed on a flat robot
    nose_down = TiltSample(tilt=Tilt(roll_rad=0.0, pitch_rad=0.3), stamp=100.0)
    arm, _ = make_guarded(tmp_path, tilt=nose_down)
    res = arm.move_joints(target, 0.5)
    assert res.slow_zone is not None and res.slow_zone["tilt_source"] == "imu"
    arm2, _ = make_guarded(tmp_path, tilt=nose_down)
    flat = FloorOverride(tilt_override_deg=TiltOverrideDeg(roll=0.0, pitch=0.0))
    assert arm2.move_joints(target, 0.5, floor=flat).slow_zone is None
    stale = TiltSample(tilt=Tilt(roll_rad=0.0, pitch_rad=0.3), stamp=90.0)
    arm3, _ = make_guarded(tmp_path, tilt=stale)
    assert arm3.move_joints(target, 0.5).slow_zone is None


def test_move_cartesian_passes_the_floor_override(tmp_path: Path) -> None:
    arm, _ = make_guarded(tmp_path)
    z = CONFIG.arm.floor_z_m - 0.02
    assert arm.move_cartesian(0.2, 0.0, z, math.pi / 2).slow_zone is not None
    arm2, _ = make_guarded(tmp_path)
    assert arm2.move_cartesian(0.2, 0.0, z, math.pi / 2, floor=FloorOverride(surface_z_m=-0.2)).slow_zone is None


def test_gripper_motion_near_the_floor_is_slowed(tmp_path: Path) -> None:
    start = low_pose(0.0) | {"gripper": 0.0}
    arm, be = make_guarded(tmp_path, backend=FakeArmBackend(start))
    res = arm.set_gripper(open_fraction=1.0)
    assert res.slow_zone is not None
    arm2, be2 = make_guarded(tmp_path, backend=FakeArmBackend(dict(start)))
    arm2.set_gripper(open_fraction=1.0, floor=FloorOverride(surface_z_m=-0.2))
    assert len(be.commands) > len(be2.commands)


def test_home_motion_uses_the_slow_zone(tmp_path: Path) -> None:
    arm, be = make_guarded(tmp_path, backend=FakeArmBackend(low_pose(0.0) | {"gripper": 0.0}))
    arm.set_home()
    be.positions.update({j: 0.0 for j in KIN.joint_names})
    res = arm.home(keep_prior_control=True)
    assert res.slow_zone is not None


def test_move_path_streams_through_the_samples_and_converges_at_the_end(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    path = [{"elbow_flex": 0.1 * i, "wrist_flex": 0.05 * i} for i in range(1, 6)]
    res = arm.move_path(path, speed_scale=0.25)
    assert res.status == "converged", res.message
    assert be.commands[-1]["elbow_flex"] == pytest.approx(0.5) and be.commands[-1]["wrist_flex"] == pytest.approx(0.25)
    vmax = CONFIG.limits.arm_max_joint_velocity_rps * 0.25 / CONFIG.limits.arm_max_speed_scale
    for a, b in zip(be.commands[1:], be.commands[2:], strict=False):
        assert abs(b["elbow_flex"] - a["elbow_flex"]) * CONFIG.limits.arm_rate_hz <= vmax * 1.05
    assert arm.control_held


def test_move_path_rejects_empty_or_unknown_joints_and_respects_the_roll_guard(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    with pytest.raises(ArmError):
        arm.move_path([], None)
    with pytest.raises(ArmError):
        arm.move_path([{"elbow": 0.1}], None)
    be.positions["gripper"] = 1.5
    with pytest.raises(ArmError, match="wrist_roll"):
        arm.move_path([{"wrist_roll": 0.5}, {"wrist_roll": 1.0}], None)
    assert be.commands == []


def test_move_path_near_the_floor_is_slowed(tmp_path: Path) -> None:
    low = low_pose(0.0)
    high = low_pose(0.04)
    start = high | {"gripper": 0.0}
    arm, be = make_guarded(tmp_path, backend=FakeArmBackend(dict(start)))
    res = arm.move_path([high, low], 0.5)
    assert res.slow_zone is not None and res.slow_zone["slowed_samples"] > 0


# --- close_until_effort with a jaw that starts late (live 2026-10-10: blocked after 0.000 rad travel) ---

START_LAG_S = 0.9  # live: the real jaw first moved 0.65-0.9 s after a close from rest started


def late_jaw(start_load: float) -> FakeArmBackend:
    """Follower whose jaw, once at rest, ignores the command for START_LAG_S after it is commanded away, reporting
    `start_load` while it strains to break away, then follows freely with no load (nothing between the jaws).
    A pause between motions (no sleep for more than a few control periods) puts the jaw at rest again."""
    be = FakeArmBackend()
    be.follow = False
    be.positions["gripper"] = -0.15
    state: dict[str, float | None] = {"moving_since": None, "last_sleep": None}
    period = 1.0 / CONFIG.limits.arm_rate_hz

    def lag(b: FakeArmBackend) -> None:
        last, state["last_sleep"] = state["last_sleep"], b.t
        if last is not None and b.t - last > 3 * period:
            state["moving_since"] = None  # the jaw rested between motions
        if not b.commands:
            return
        cmd = b.commands[-1]
        b.positions.update({j: v for j, v in cmd.items() if j != "gripper"})
        apart = abs(cmd["gripper"] - b.positions["gripper"]) > 1e-6
        if not apart:
            state["moving_since"] = None
            b.efforts["gripper"] = 0.0
            return
        since = state["moving_since"]
        if since is None:
            state["moving_since"] = since = b.t
        if b.t - since < START_LAG_S:
            b.efforts["gripper"] = start_load
            return
        b.positions["gripper"] = cmd["gripper"]
        b.efforts["gripper"] = 0.0

    be.on_sleep = lag
    return be


def test_close_until_effort_waits_for_a_late_starting_jaw_instead_of_reporting_blocked(tmp_path: Path) -> None:
    arm, be = make(tmp_path, late_jaw(start_load=500.0))
    opened = arm.set_gripper(open_fraction=0.5)
    assert opened.status == "converged", opened.message
    be.t += 1.4  # live: the close was called 1.4 s after the open returned
    arm.keepalive_tick()
    res = arm.set_gripper(close_until_effort=True, effort_threshold=300.0)
    assert res.status == "closed_no_contact", res.message
    assert be.positions["gripper"] == pytest.approx(CONFIG.arm.gripper_closed_rad)


def test_close_until_effort_with_nothing_between_the_jaws_and_no_load_travels_to_closed(tmp_path: Path) -> None:
    arm, be = make(tmp_path, late_jaw(start_load=0.0))
    arm.set_gripper(open_fraction=0.5)
    be.t += 1.4
    arm.keepalive_tick()
    res = arm.set_gripper(close_until_effort=True, effort_threshold=300.0)
    assert res.status == "closed_no_contact", res.message
    assert be.commands[-1]["gripper"] == pytest.approx(CONFIG.arm.gripper_closed_rad)
    assert be.positions["gripper"] == pytest.approx(CONFIG.arm.gripper_closed_rad)
