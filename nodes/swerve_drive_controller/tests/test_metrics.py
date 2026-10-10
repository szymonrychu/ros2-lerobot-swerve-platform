"""Tests for the swerve controller metrics: metrics_port config and updates at each code point."""

from pathlib import Path

import pytest
from prometheus_client import REGISTRY

from swerve_drive_controller.config import SwerveControllerConfig, load_config
from swerve_drive_controller.control import (
    CmdVelSample,
    ControlOutput,
    ControlState,
    JointSample,
    run_cycle,
)

NAMES = ["fl_drive", "fl_steer", "fr_drive", "fr_steer", "rl_drive", "rl_steer", "rr_drive", "rr_steer"]
STEER = [NAMES[1], NAMES[3], NAMES[5], NAMES[7]]
DRIVE = [NAMES[0], NAMES[2], NAMES[4], NAMES[6]]


def sample(name: str) -> float | None:
    return REGISTRY.get_sample_value(name)


def make_config(tmp_path: Path, extra: str = "{}") -> SwerveControllerConfig:
    path = tmp_path / "config.yaml"
    path.write_text(extra + "\n")
    cfg = load_config(path)
    assert cfg is not None
    return cfg


def make_joints(time_s: float, drive: float = 0.0) -> JointSample:
    return JointSample({j: 0.0 for j in STEER}, {j: drive for j in DRIVE}, time_s)


def test_metrics_port_default_and_parsed(tmp_path: Path) -> None:
    assert make_config(tmp_path).metrics_port is None
    assert make_config(tmp_path, "metrics_port: 19107").metrics_port == 19107


def test_fresh_cycle_publishes_and_sets_gauges(tmp_path: Path) -> None:
    cfg = make_config(tmp_path)
    published: list[ControlOutput] = []
    before_odom = sample("swerve_odom_published_total") or 0.0
    before_loop = sample("swerve_loop_duration_seconds_count") or 0.0
    state, was_published = run_cycle(
        cfg,
        ControlState(),
        CmdVelSample((0.1, 0.0, 0.0), 9.75),
        make_joints(10.0),
        10.0,
        lambda output, new_state: published.append(output),
    )
    assert was_published and len(published) == 1
    assert state.last_odom_time == 10.0
    assert sample("swerve_odom_published_total") == before_odom + 1
    assert sample("swerve_loop_duration_seconds_count") == before_loop + 1
    assert sample("swerve_cmd_vel_age_seconds") == pytest.approx(0.25)
    assert sample("swerve_slip_residual") is not None


def test_stale_joint_states_count_and_skip_publish(tmp_path: Path) -> None:
    cfg = make_config(tmp_path)
    published: list[ControlOutput] = []
    before = sample("swerve_joint_states_stale_total") or 0.0
    before_odom = sample("swerve_odom_published_total") or 0.0
    before_loop = sample("swerve_loop_duration_seconds_count") or 0.0
    _, was_published = run_cycle(
        cfg,
        ControlState(),
        CmdVelSample((0.1, 0.0, 0.0), 10.0),
        make_joints(1.0),
        10.0,
        lambda output, new_state: published.append(output),
    )
    assert not was_published and published == []
    assert sample("swerve_joint_states_stale_total") == before + 1
    assert sample("swerve_odom_published_total") == before_odom
    # duration is observed for every cycle, published or not
    assert sample("swerve_loop_duration_seconds_count") == before_loop + 1


def test_cmd_vel_age_unset_before_first_message(tmp_path: Path) -> None:
    cfg = make_config(tmp_path)
    before = sample("swerve_cmd_vel_age_seconds")
    run_cycle(cfg, ControlState(), CmdVelSample(), make_joints(10.0), 10.0, lambda output, new_state: None)
    assert sample("swerve_cmd_vel_age_seconds") == before
