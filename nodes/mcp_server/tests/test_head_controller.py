"""Tests for mcp_server.head: HeadController over FakeHeadBackend (measured, check, move, home)."""

import json
import logging
import math
import os
from pathlib import Path

import numpy as np
import pytest
from ros2_common.head_rgbd_calibration.urdf_text import Origin, head_chain_json

from mcp_server.config import HeadSettings, McpServerConfig
from mcp_server.head import HEAD_FRAME_ID, AppliedHeadKinematics, HeadController, HeadError
from mcp_server.head_kinematics import DEFAULT_HEAD_URDF, HeadKinematics
from tests.fakes import FakeHeadBackend

KINEMATICS = HeadKinematics(DEFAULT_HEAD_URDF)


def make(backend: FakeHeadBackend, **overrides: float) -> HeadController:
    return HeadController(backend, AppliedHeadKinematics(DEFAULT_HEAD_URDF, None), HeadSettings(**overrides))


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


FLOOR_POINT = np.array([2.0, 0.3, 0.0])


def write_applied(applied: Path, dz: float = 0.05, session: str = "s1") -> None:
    applied.mkdir(exist_ok=True)
    origins = {"head_mount_joint": Origin((0.15, 0.10, 0.31 + dz), (0.0, 0.0, 0.0))}
    (applied / "head_chain.json").write_text(head_chain_json(origins, "test", session, "ok"))
    (applied / "applied.json").write_text(json.dumps({"session": session}))


def test_no_applied_dir_uses_repo_urdf(tmp_path: Path) -> None:
    for applied in (None, tmp_path / "missing"):
        provider = AppliedHeadKinematics(DEFAULT_HEAD_URDF, applied)
        assert provider.current().look_at(FLOOR_POINT) == KINEMATICS.look_at(FLOOR_POINT)


def test_look_at_uses_applied_head_chain(tmp_path: Path) -> None:
    write_applied(tmp_path)
    provider = AppliedHeadKinematics(DEFAULT_HEAD_URDF, tmp_path)
    assert provider.current().look_at(FLOOR_POINT)[1] != pytest.approx(KINEMATICS.look_at(FLOOR_POINT)[1], abs=1e-3)
    head = HeadController(FakeHeadBackend(), provider, HeadSettings())
    assert head.kinematics.look_at(FLOOR_POINT) == provider.current().look_at(FLOOR_POINT)


def test_applied_marker_change_reloads_kinematics(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    write_applied(tmp_path, dz=0.05, session="s1")
    provider = AppliedHeadKinematics(DEFAULT_HEAD_URDF, tmp_path)
    first = provider.current()
    assert provider.current() is first
    write_applied(tmp_path, dz=0.10, session="s2")
    marker = tmp_path / "applied.json"
    os.utime(marker, ns=(marker.stat().st_atime_ns, marker.stat().st_mtime_ns + 1_000_000_000))
    with caplog.at_level(logging.INFO, logger="mcp_server.head"):
        second = provider.current()
    assert second is not first
    assert second.look_at(FLOOR_POINT) != first.look_at(FLOOR_POINT)
    assert any("s2" in r.getMessage() for r in caplog.records)
    marker.unlink()
    (tmp_path / "head_chain.json").unlink()
    assert provider.current().look_at(FLOOR_POINT) == pytest.approx(KINEMATICS.look_at(FLOOR_POINT))


def test_broken_head_chain_keeps_urdf_chain_with_warning(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    (tmp_path / "head_chain.json").write_text("{not json")
    (tmp_path / "applied.json").write_text("{}")
    with caplog.at_level(logging.WARNING, logger="mcp_server.head"):
        provider = AppliedHeadKinematics(DEFAULT_HEAD_URDF, tmp_path)
        assert provider.current().look_at(FLOOR_POINT) == pytest.approx(KINEMATICS.look_at(FLOOR_POINT))
    assert any("head_chain.json" in r.getMessage() and r.levelno == logging.WARNING for r in caplog.records)


def test_broken_override_keeps_previous_kinematics(
    tmp_path: Path, caplog: pytest.LogCaptureFixture, monkeypatch: pytest.MonkeyPatch
) -> None:
    write_applied(tmp_path)
    provider = AppliedHeadKinematics(DEFAULT_HEAD_URDF, tmp_path)
    first = provider.current()

    def boom(text: str, source: str) -> HeadKinematics:
        raise ValueError("bad urdf")

    monkeypatch.setattr(HeadKinematics, "from_text", staticmethod(boom))
    marker = tmp_path / "applied.json"
    os.utime(marker, ns=(marker.stat().st_atime_ns, marker.stat().st_mtime_ns + 1_000_000_000))
    with caplog.at_level(logging.ERROR, logger="mcp_server.head"):
        assert provider.current() is first
    assert any("bad urdf" in r.getMessage() for r in caplog.records)
