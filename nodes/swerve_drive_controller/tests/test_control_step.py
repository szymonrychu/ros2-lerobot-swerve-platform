"""Unit tests for the per-cycle control step and combined command builder."""

import math
from pathlib import Path

from swerve_drive_controller.config import SwerveControllerConfig, load_config
from swerve_drive_controller.control import CmdVelSample, ControlState, JointSample, build_joint_command, control_step

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
