"""Unit tests for the per-cycle control step and combined command builder."""

import math
from dataclasses import replace
from pathlib import Path

import pytest

from swerve_drive_controller.config import SwerveControllerConfig, load_config
from swerve_drive_controller.control import (
    CmdVelSample,
    ControlOutput,
    ControlState,
    JointSample,
    build_joint_command,
    control_step,
    gate_cmd_vel,
)

NAMES = ["fl_drive", "fl_steer", "fr_drive", "fr_steer", "rl_drive", "rl_steer", "rr_drive", "rr_steer"]
STEER = [NAMES[1], NAMES[3], NAMES[5], NAMES[7]]
DRIVE = [NAMES[0], NAMES[2], NAMES[4], NAMES[6]]


def make_config(tmp_path: Path) -> SwerveControllerConfig:
    path = tmp_path / "config.yaml"
    path.write_text("{}\n")
    cfg = load_config(path)
    assert cfg is not None
    return cfg


def make_joints(time_s: float, steer: float = 0.0, drive: float = 0.0) -> JointSample:
    positions = {j: steer for j in STEER}
    velocities = {j: drive for j in DRIVE}
    return JointSample(positions=positions, velocities=velocities, time_s=time_s)


def test_build_joint_command_layout() -> None:
    names, positions, velocities = build_joint_command(NAMES, [0.1, 0.2, 0.3, 0.4], [1.0, 2.0, 3.0, 4.0])
    assert names == NAMES
    assert positions[1] == 0.1 and positions[3] == 0.2 and positions[5] == 0.3 and positions[7] == 0.4
    assert velocities[0] == 1.0 and velocities[2] == 2.0 and velocities[4] == 3.0 and velocities[6] == 4.0
    for i in (0, 2, 4, 6):
        assert math.isnan(positions[i])
    for i in (1, 3, 5, 7):
        assert math.isnan(velocities[i])
    assert len(positions) == len(velocities) == 8


def test_stale_joint_states_no_command(tmp_path: Path) -> None:
    cfg = make_config(tmp_path)
    out, state = control_step(cfg, ControlState(), CmdVelSample((0.1, 0.0, 0.0), 10.0), make_joints(1.0), 10.0)
    assert out is None
    assert state.last_odom_time is None


def test_missing_joint_states_no_command(tmp_path: Path) -> None:
    cfg = make_config(tmp_path)
    joints = JointSample(positions={}, velocities={}, time_s=10.0)
    out, _ = control_step(cfg, ControlState(), CmdVelSample((0.1, 0.0, 0.0), 10.0), joints, 10.0)
    assert out is None


def test_incomplete_joint_states_no_command(tmp_path: Path) -> None:
    cfg = make_config(tmp_path)
    joints = make_joints(10.0)
    del joints.velocities[DRIVE[0]]
    out, _ = control_step(cfg, ControlState(), CmdVelSample((0.1, 0.0, 0.0), 10.0), joints, 10.0)
    assert out is None


def test_forward_command(tmp_path: Path) -> None:
    cfg = make_config(tmp_path)
    out, state = control_step(cfg, ControlState(), CmdVelSample((0.1, 0.0, 0.0), 10.0), make_joints(10.0), 10.0)
    assert out is not None
    assert out.names == NAMES
    for i in (0, 2, 4, 6):
        assert math.isclose(out.velocities[i], 0.1 / cfg.wheel_radius_m, rel_tol=1e-6)
    assert state.steer_targets is not None
    assert out.moving is True


def test_cmd_vel_timeout_zero_twist(tmp_path: Path) -> None:
    cfg = make_config(tmp_path)
    cmd = CmdVelSample((0.1, 0.0, 0.0), 10.0 - cfg.cmd_vel_timeout_s - 0.1)
    out, _ = control_step(cfg, ControlState(), cmd, make_joints(10.0), 10.0)
    assert out is not None
    for i in (0, 2, 4, 6):
        assert out.velocities[i] == 0.0
    assert out.moving is False


def test_deadband_zeroes_tiny_command(tmp_path: Path) -> None:
    cfg = make_config(tmp_path)
    out, _ = control_step(cfg, ControlState(), CmdVelSample((0.004, 0.004, 0.004), 10.0), make_joints(10.0), 10.0)
    assert out is not None
    for i in (0, 2, 4, 6):
        assert out.velocities[i] == 0.0
    assert out.moving is False


def test_steer_targets_held_when_stopped(tmp_path: Path) -> None:
    cfg = make_config(tmp_path)
    state = ControlState()
    out, state = control_step(cfg, state, CmdVelSample((0.0, 0.1, 0.0), 10.0), make_joints(10.0), 10.0)
    assert out is not None
    sideways = [out.positions[i] for i in (1, 3, 5, 7)]
    assert all(abs(s) > 1.0 for s in sideways)
    out2, state = control_step(cfg, state, CmdVelSample((0.0, 0.0, 0.0), 10.02), make_joints(10.02), 10.02)
    assert out2 is not None
    assert [out2.positions[i] for i in (1, 3, 5, 7)] == sideways


def test_no_propulsion_safeguard(tmp_path: Path) -> None:
    cfg = make_config(tmp_path)
    # Wheels measured straight, command sideways: steering error large -> drive zeroed.
    out, _ = control_step(cfg, ControlState(), CmdVelSample((0.0, 0.1, 0.0), 10.0), make_joints(10.0), 10.0)
    assert out is not None
    for i in (0, 2, 4, 6):
        assert out.velocities[i] == 0.0


def test_odometry_integrates_and_reports_twist(tmp_path: Path) -> None:
    cfg = make_config(tmp_path)
    speed = 0.1 / cfg.wheel_radius_m
    state = ControlState()
    out, state = control_step(cfg, state, CmdVelSample((0.1, 0.0, 0.0), 10.0), make_joints(10.0, 0.0, speed), 10.0)
    assert out is not None and state.pose == (0.0, 0.0, 0.0)
    out, state = control_step(cfg, state, CmdVelSample((0.1, 0.0, 0.0), 10.0), make_joints(11.0, 0.0, speed), 11.0)
    assert out is not None
    assert math.isclose(out.odom_twist[0], 0.1, rel_tol=1e-6)
    assert math.isclose(state.pose[0], 0.1, rel_tol=1e-6)


def steer_of(out: ControlOutput) -> list[float]:
    return [out.positions[i] for i in (1, 3, 5, 7)]


def drive_of(out: ControlOutput) -> list[float]:
    return [out.velocities[i] for i in (0, 2, 4, 6)]


def test_group_flip_carried_across_cycles(tmp_path: Path) -> None:
    """Sweeping the travel direction through 90 deg while turning: the group side lives in ControlState and
    the wheels never split across +-90 deg nor swing one at a time."""
    cfg = make_config(tmp_path)
    state = ControlState()
    assert state.steer_flip is None
    measured = [0.0] * 4
    previous: list[float] | None = None
    group_flips = 0
    t = 10.0
    for deg in range(60, 121):
        theta = math.radians(deg)
        joints = JointSample(positions=dict(zip(STEER, measured)), velocities={j: 0.0 for j in DRIVE}, time_s=t)
        cmd = CmdVelSample((0.1 * math.cos(theta), 0.1 * math.sin(theta), 0.02), t)
        out, state = control_step(cfg, state, cmd, joints, t)
        assert out is not None
        assert state.steer_flip is not None
        steer = steer_of(out)
        assert len({math.copysign(1.0, s) for s in steer}) == 1
        jumps = [previous is not None and abs(s - p) > 1.0 for s, p in zip(steer, previous or steer)]
        assert not any(jumps) or all(jumps)
        if all(jumps):
            group_flips += 1
            # The whole set swings together; the safeguard holds every drive until steering catches up.
            assert all(d == 0.0 for d in drive_of(out))
        elif previous is not None:
            assert all(d != 0.0 for d in drive_of(out))
        previous = steer
        measured = steer  # steering tracks the command
        t += 0.02
    assert group_flips == 1


def test_group_flip_kept_when_joint_states_stale(tmp_path: Path) -> None:
    cfg = make_config(tmp_path)
    state = ControlState(steer_targets=[1.5] * 4, steer_flip=True)
    out, new_state = control_step(cfg, state, CmdVelSample((0.0, 0.1, 0.0), 10.0), make_joints(1.0), 10.0)
    assert out is None
    assert new_state.steer_flip is True
    assert new_state.steer_targets == [1.5] * 4


def _sideways_then_stop(cfg, stop_times: list[float]) -> list[list[float]]:
    """Drive sideways at t=10.0, then send zero twist at each of stop_times; return steer targets per stop cycle."""
    state = ControlState()
    _, state = control_step(cfg, state, CmdVelSample((0.0, 0.1, 0.0), 10.0), make_joints(10.0), 10.0)
    steers = []
    for t in stop_times:
        out, state = control_step(cfg, state, CmdVelSample((0.0, 0.0, 0.0), t), make_joints(t), t)
        assert out is not None
        steers.append([out.positions[i] for i in (1, 3, 5, 7)])
    return steers


def test_steering_recenters_after_idle_timeout(tmp_path: Path) -> None:
    cfg = make_config(tmp_path)
    assert cfg.idle_recenter_s == pytest.approx(3.0)
    held, recentered = _sideways_then_stop(cfg, [10.02, 12.9, 13.1])[1:]
    assert all(abs(s) > 1.0 for s in held), "heading still held before the timeout"
    assert recentered == [0.0, 0.0, 0.0, 0.0]


def test_motion_resets_idle_timer(tmp_path: Path) -> None:
    cfg = make_config(tmp_path)
    state = ControlState()
    for t in (10.0, 11.0, 12.0):  # stop for 2 s ...
        _, state = control_step(
            cfg, state, CmdVelSample((0.0, 0.1, 0.0) if t == 10.0 else (0.0, 0.0, 0.0), t), make_joints(t), t
        )
    _, state = control_step(cfg, state, CmdVelSample((0.0, 0.1, 0.0), 12.5), make_joints(12.5), 12.5)  # ... move
    out, state = control_step(cfg, state, CmdVelSample((0.0, 0.0, 0.0), 14.0), make_joints(14.0), 14.0)
    assert out is not None
    assert all(abs(out.positions[i]) > 1.0 for i in (1, 3, 5, 7)), "only 1.5 s idle since the last motion"


def test_idle_recenter_disabled_with_zero(tmp_path: Path) -> None:
    cfg = replace(make_config(tmp_path), idle_recenter_s=0.0)
    late = _sideways_then_stop(cfg, [10.02, 60.0])[-1]
    assert all(abs(s) > 1.0 for s in late)


def test_control_step_reports_fk_residual_and_drops_slipping_wheel(tmp_path: Path) -> None:
    cfg = make_config(tmp_path)
    speed = 0.1 / cfg.wheel_radius_m
    joints = make_joints(10.0, 0.0, speed)
    joints.velocities[DRIVE[0]] = speed + 0.2 / cfg.wheel_radius_m  # front-left wheel spins on a lego
    out, _ = control_step(cfg, ControlState(), CmdVelSample((0.1, 0.0, 0.0), 10.0), joints, 10.0)
    assert out is not None
    assert out.odom_twist == pytest.approx((0.1, 0.0, 0.0), abs=1e-6)
    assert out.odom_residual_mps == pytest.approx(0.0, abs=1e-6)


def test_control_step_residual_is_zero_for_consistent_wheels(tmp_path: Path) -> None:
    cfg = make_config(tmp_path)
    out, _ = control_step(cfg, ControlState(), CmdVelSample((0.1, 0.0, 0.0), 10.0), make_joints(10.0, 0.0, 1.0), 10.0)
    assert out is not None
    assert out.odom_residual_mps == pytest.approx(0.0, abs=1e-9)


def test_control_step_flags_parked_from_measured_wheel_speeds(tmp_path: Path) -> None:
    cfg = make_config(tmp_path)
    stop = CmdVelSample((0.0, 0.0, 0.0), 10.0)
    still, _ = control_step(cfg, ControlState(), stop, make_joints(10.0, 0.0, 0.0), 10.0)
    assert still is not None and still.parked
    rolling, _ = control_step(cfg, ControlState(), stop, make_joints(10.0, 0.0, 1.0), 10.0)
    assert rolling is not None and not rolling.parked
    go = CmdVelSample((0.1, 0.0, 0.0), 10.0)
    commanded, _ = control_step(cfg, ControlState(), go, make_joints(10.0, 0.0, 0.0), 10.0)
    assert commanded is not None and not commanded.parked, "an active command uses the normal variances"
    stale = CmdVelSample((0.1, 0.0, 0.0), 1.0)
    timed_out, _ = control_step(cfg, ControlState(), stale, make_joints(10.0, 0.0, 0.0), 10.0)
    assert timed_out is not None and timed_out.parked, "an expired command is no command"
    deadband = CmdVelSample((0.0, 0.0, 1e-4), 10.0)
    tiny, _ = control_step(cfg, ControlState(), deadband, make_joints(10.0, 0.0, 0.0), 10.0)
    assert tiny is not None and tiny.parked, "a command inside the deadband is no command"


def test_inhibited_ignores_drive_command_and_brakes(tmp_path: Path) -> None:
    """Tilt error: a fresh drive command is ignored; every wheel is commanded to zero speed (torque stays on)."""
    cfg = make_config(tmp_path)
    out, state = control_step(
        cfg, ControlState(), CmdVelSample((0.2, 0.0, 0.0), 10.0), make_joints(10.0), 10.0, inhibited=True
    )
    assert out is not None
    assert [out.velocities[i] for i in (0, 2, 4, 6)] == [0.0, 0.0, 0.0, 0.0]
    assert out.moving is False


def test_inhibited_holds_steering_without_recentering(tmp_path: Path) -> None:
    """Steering is held at its last target while inhibited, also past idle_recenter_s."""
    cfg = make_config(tmp_path)
    state = ControlState()
    _, state = control_step(cfg, state, CmdVelSample((0.0, 0.1, 0.0), 10.0), make_joints(10.0), 10.0)
    held = list(state.steer_targets or [])
    assert all(abs(s) > 1.0 for s in held)
    for t in (10.5, 14.0, 30.0):
        out, state = control_step(
            cfg, state, CmdVelSample((0.3, 0.0, 0.5), t), make_joints(t, steer=1.5), t, inhibited=True
        )
        assert out is not None
        assert [out.positions[i] for i in (1, 3, 5, 7)] == held
        assert [out.velocities[i] for i in (0, 2, 4, 6)] == [0.0] * 4


def test_not_inhibited_passes_command_through(tmp_path: Path) -> None:
    cfg = make_config(tmp_path)
    out, _ = control_step(
        cfg, ControlState(), CmdVelSample((0.1, 0.0, 0.0), 10.0), make_joints(10.0), 10.0, inhibited=False
    )
    assert out is not None and out.moving
    assert all(out.velocities[i] > 0.0 for i in (0, 2, 4, 6))


def test_gate_cmd_vel_drops_and_counts_commands_while_inhibited() -> None:
    from prometheus_client import REGISTRY

    before = REGISTRY.get_sample_value("tilt_guard_inhibited_commands_total") or 0.0
    sample = CmdVelSample((0.2, 0.0, 0.1), 5.0)
    assert gate_cmd_vel(sample, inhibited=False) is sample
    assert REGISTRY.get_sample_value("tilt_guard_inhibited_commands_total") == before
    assert gate_cmd_vel(sample, inhibited=True) == CmdVelSample()
    assert REGISTRY.get_sample_value("tilt_guard_inhibited_commands_total") == before + 1


def speed_scale_after(
    tmp_path: Path, twist: tuple[float, float, float], inhibited: bool = False, age: float = 0.0
) -> float:
    from prometheus_client import REGISTRY

    cfg = make_config(tmp_path)
    out, _ = control_step(
        cfg, ControlState(), CmdVelSample(twist, 10.0 - age), make_joints(10.0), 10.0, inhibited=inhibited
    )
    assert out is not None
    value = REGISTRY.get_sample_value("swerve_cmd_speed_scale")
    assert value is not None
    return value


def test_speed_scale_gauge_reports_corner_clamp(tmp_path: Path) -> None:
    assert speed_scale_after(tmp_path, (0.25, 0.25, 0.0)) == pytest.approx(0.799, abs=1e-3)


def test_speed_scale_gauge_is_one_within_limit(tmp_path: Path) -> None:
    assert speed_scale_after(tmp_path, (0.1, 0.0, 0.0)) == 1.0


def test_speed_scale_gauge_is_one_when_stopped_or_inhibited(tmp_path: Path) -> None:
    speed_scale_after(tmp_path, (0.25, 0.25, 0.0))
    assert speed_scale_after(tmp_path, (0.0, 0.0, 0.0)) == 1.0
    speed_scale_after(tmp_path, (0.25, 0.25, 0.0))
    assert speed_scale_after(tmp_path, (0.25, 0.25, 0.0), inhibited=True) == 1.0


def test_released_publishes_no_steering_zero_drive_and_ignores_cmd(tmp_path: Path) -> None:
    """Released steering: cmd_vel ignored, drives 0 rad/s, no finite steering position, no idle recentring."""
    cfg = make_config(tmp_path)
    state = ControlState()
    for t in (10.0, 14.0, 30.0):
        out, state = control_step(
            cfg, state, CmdVelSample((0.2, 0.1, 0.3), t), make_joints(t, steer=1.2), t, released=True
        )
        assert out is not None
        assert all(math.isnan(out.positions[i]) for i in range(8))
        assert [out.velocities[i] for i in (0, 2, 4, 6)] == [0.0] * 4
        assert out.moving is False


def test_released_still_reports_odometry_from_measured_joints(tmp_path: Path) -> None:
    cfg = make_config(tmp_path)
    out, _ = control_step(cfg, ControlState(), CmdVelSample(), make_joints(10.0, 0.0, 2.0), 10.0, released=True)
    assert out is not None
    assert out.odom_twist[0] == pytest.approx(2.0 * cfg.wheel_radius_m)


def test_reenable_seeds_steering_from_measured(tmp_path: Path) -> None:
    """First cycle after release ends: steering targets equal the measured positions, not the old targets."""
    cfg = make_config(tmp_path)
    state = ControlState()
    _, state = control_step(cfg, state, CmdVelSample((0.0, 0.1, 0.0), 10.0), make_joints(10.0), 10.0)
    _, state = control_step(cfg, state, CmdVelSample(), make_joints(11.0, steer=0.3), 11.0, released=True)
    out, state = control_step(cfg, state, CmdVelSample(), make_joints(11.02, steer=0.3), 11.02)
    assert out is not None
    assert [out.positions[i] for i in (1, 3, 5, 7)] == pytest.approx([0.3] * 4)


def test_reenable_recentres_only_after_idle_recenter_s(tmp_path: Path) -> None:
    cfg = make_config(tmp_path)
    state = ControlState()
    _, state = control_step(cfg, state, CmdVelSample(), make_joints(10.0, steer=0.3), 10.0)
    _, state = control_step(cfg, state, CmdVelSample(), make_joints(20.0, steer=0.3), 20.0, released=True)
    out, state = control_step(cfg, state, CmdVelSample(), make_joints(100.0, steer=0.3), 100.0)
    assert out is not None and [out.positions[i] for i in (1, 3, 5, 7)] == pytest.approx([0.3] * 4)
    out, state = control_step(cfg, state, CmdVelSample(), make_joints(102.9, steer=0.3), 102.9)
    assert out is not None and [out.positions[i] for i in (1, 3, 5, 7)] == pytest.approx([0.3] * 4)
    out, state = control_step(cfg, state, CmdVelSample(), make_joints(103.1, steer=0.3), 103.1)
    assert out is not None and [out.positions[i] for i in (1, 3, 5, 7)] == [0.0] * 4


def test_released_false_default_and_inhibit_unchanged(tmp_path: Path) -> None:
    cfg = make_config(tmp_path)
    out, _ = control_step(
        cfg, ControlState(), CmdVelSample((0.2, 0.0, 0.0), 10.0), make_joints(10.0, steer=0.4), 10.0, inhibited=True
    )
    assert out is not None
    assert [out.positions[i] for i in (1, 3, 5, 7)] == pytest.approx([0.4] * 4)
    assert [out.velocities[i] for i in (0, 2, 4, 6)] == [0.0] * 4


def test_run_cycle_passes_released(tmp_path: Path) -> None:
    from swerve_drive_controller.control import run_cycle

    cfg = make_config(tmp_path)
    seen: list[ControlOutput] = []
    run_cycle(
        cfg,
        ControlState(),
        CmdVelSample((0.1, 0, 0), 10.0),
        make_joints(10.0),
        10.0,
        lambda o, s: seen.append(o),
        released=True,
    )
    assert seen and all(math.isnan(p) for p in seen[0].positions)
