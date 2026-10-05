"""Tests for mcp_server.arm.ArmController: autonomy lease, streamed motion, aborts, gripper, home pose, keepalive."""

from pathlib import Path

import pytest

from mcp_server.arm import ArmController, ArmError
from mcp_server.config import McpServerConfig
from mcp_server.ik import ArmKinematics, load_joint_limits

from .fakes import JOINTS, FakeArmBackend

CONFIG = McpServerConfig()
KIN = ArmKinematics(CONFIG.arm.urdf_path, margin=CONFIG.limits.arm_limit_margin_rad)


def make(tmp_path: Path, backend: FakeArmBackend | None = None) -> tuple[ArmController, FakeArmBackend]:
    be = backend or FakeArmBackend()
    cfg = CONFIG.model_copy(deep=True)
    cfg.arm.home_file = tmp_path / "arm" / "home.yaml"
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
    assert arm.state().floor_z_m == pytest.approx(-0.165)


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
    lo = load_joint_limits(CONFIG.arm.urdf_path)["gripper"][0] + CONFIG.limits.arm_limit_margin_rad
    arm.set_gripper(open_fraction=0.0)
    assert be.commands[-1]["gripper"] == pytest.approx(CONFIG.arm.gripper_closed_rad)
    assert lo <= CONFIG.arm.gripper_closed_rad < -0.1


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
    be.sag = {"shoulder_lift": -(CONFIG.limits.arm_settle_tolerance_rad + 0.02)}
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
        last = b.commands[-1]["elbow_flex"]
        b.positions["elbow_flex"] = last - 0.055 + (0.015 if ticks[0] % 2 else -0.015)  # error 0.04 / 0.07

    be.on_sleep = wobble
    res = arm.move_joints({"elbow_flex": 0.3}, speed_scale=0.5)
    assert res.status == "timeout"


def test_tracking_abort_still_holds_measured_with_settle_enabled(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.sag = {"shoulder_lift": -(CONFIG.limits.arm_tracking_error_rad + 0.05)}
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
