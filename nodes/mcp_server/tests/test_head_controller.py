"""Tests for mcp_server.head: HeadController over FakeHeadBackend (measured, check, move, home)."""

import math

import pytest

from mcp_server.config import HeadSettings, McpServerConfig
from mcp_server.head import HEAD_FRAME_ID, HeadController, HeadError
from mcp_server.head_kinematics import DEFAULT_HEAD_URDF, HeadKinematics
from tests.fakes import FakeHeadBackend

KINEMATICS = HeadKinematics(DEFAULT_HEAD_URDF)


def make(backend: FakeHeadBackend, **overrides: float) -> HeadController:
    return HeadController(backend, KINEMATICS, HeadSettings(**overrides))


def test_defaults() -> None:
    s = McpServerConfig().head
    assert s.urdf_path == DEFAULT_HEAD_URDF
    assert s.joint_names == ("head_pan", "head_tilt")
    assert (s.command_topic, s.state_topic) == ("/head/joint_commands", "/head/joint_states")
    assert (s.home_pan_rad, s.home_tilt_rad) == (0.0, 0.5236)
    assert (s.tolerance_rad, s.move_timeout_s, s.state_max_age_s) == (0.03, 4.0, 0.5)


def test_home_outside_limits_refused() -> None:
    with pytest.raises(ValueError, match="head_tilt"):
        HeadSettings(home_tilt_rad=1.2)
    with pytest.raises(ValueError, match="head_pan"):
        HeadSettings(home_pan_rad=-2.0)


def test_measured_none_when_missing_or_stale() -> None:
    backend = FakeHeadBackend()
    head = make(backend)
    assert head.measured() == {"head_pan": 0.0, "head_tilt": 0.0}
    backend.no_samples = True
    assert head.measured() is None
    backend.no_samples = False
    backend.stale_after = backend.t - 1.0
    assert head.measured() is None


def test_out_of_range_refused_naming_limits() -> None:
    backend = FakeHeadBackend()
    head = make(backend)
    with pytest.raises(HeadError, match=r"head_pan.*-90\.0.*90\.0"):
        head.check(2.0, 0.0)
    with pytest.raises(HeadError, match="head_tilt"):
        head.move(0.0, 1.5)
    assert backend.commands == []


def test_move_converges_with_following_fake() -> None:
    backend = FakeHeadBackend(speed=1.0)
    result = make(backend).move(0.6, 0.3)
    assert result.status == "converged"
    assert result.measured == pytest.approx({"head_pan": 0.6, "head_tilt": 0.3}, abs=0.03)
    assert len(backend.commands) == 1


def test_move_times_out_with_stuck_follower() -> None:
    backend = FakeHeadBackend(speed=0.0)
    result = make(backend).move(0.6, 0.3)
    assert result.status == "timeout"
    assert result.measured == {"head_pan": 0.0, "head_tilt": 0.0}
    assert backend.t - 100.0 >= 4.0


def test_move_no_feedback_without_samples() -> None:
    backend = FakeHeadBackend()
    backend.no_samples = True
    result = make(backend).move(0.2, 0.2)
    assert result.status == "no_feedback"
    assert result.measured is None
    assert len(backend.commands) == 1


def test_command_carries_frame_and_both_joints() -> None:
    backend = FakeHeadBackend()
    make(backend).move(0.1, 0.2)
    assert HEAD_FRAME_ID == "autonomy"
    assert backend.commands == [{"head_pan": 0.1, "head_tilt": 0.2}]


def test_home_moves_to_configured_home() -> None:
    backend = FakeHeadBackend(speed=2.0)
    result = make(backend).home()
    assert result.status == "converged"
    assert backend.commands[-1] == {"head_pan": 0.0, "head_tilt": 0.5236}
    assert math.isclose(result.measured["head_tilt"], 0.5236, abs_tol=0.03)
