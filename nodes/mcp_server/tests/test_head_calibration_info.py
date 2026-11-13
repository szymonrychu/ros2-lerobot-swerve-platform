"""AppliedHeadKinematics.calibration(): which applied head chain is in effect and its arm-frame residual (the accepted
arm to camera calibration's held-out tool error when it was fitted against the applied chain, else the head-from-arm
fit's held-out board error), and the grasp gate on it."""

import json
import os
from pathlib import Path

from ros2_common.head_rgbd_calibration.urdf_text import ChainResidual, Origin, head_chain_json

from mcp_server.camera_point import calibration_block, gate_refusal, gate_state
from mcp_server.head import AppliedHeadKinematics
from mcp_server.head_kinematics import DEFAULT_HEAD_URDF

from .depth_helpers import FAKE_ARM_RESULT, FAKE_ARM_SESSION, write_arm_calibration

ORIGINS = {"head_tilt": Origin((0.0, 0.0, 0.0412), (0.0, 0.0123, 0.0))}
RESIDUAL = ChainResidual(kind="head_arm", holdout_board_mm=2.9, holdout_rms_px=2.5, rms_px=2.7)
GATE_MM = 5.0
OTHER_CHAIN = ("fake-other-ground-1", "fake-other-r-1")


def apply(applied: Path, residual: ChainResidual | None, source: str = "head_arm", result: str = "r-1") -> None:
    """Write head_chain.json and its applied.json record the way the calibration node's apply does (marker last)."""
    applied.mkdir(parents=True, exist_ok=True)
    (applied / "head_chain.json").write_text(head_chain_json(ORIGINS, "fit", "calib-1", result, residual))
    marker = {"head_chain": {"session": "calib-1", "result": result, "applied_at": "2026-10-29T10:00:00+00:00"}}
    marker["head_chain"]["source"] = source
    (applied / "applied.json").write_text(json.dumps(marker))


def test_nothing_applied() -> None:
    info = AppliedHeadKinematics(DEFAULT_HEAD_URDF, None).calibration()
    assert not info.applied and info.residual is None and info.residual_mm is None
    assert "repo head.urdf" in info.describe()


def test_empty_applied_dir_is_nothing_applied(tmp_path: Path) -> None:
    assert not AppliedHeadKinematics(DEFAULT_HEAD_URDF, tmp_path).calibration().applied


def test_head_arm_chain_reports_its_residual(tmp_path: Path) -> None:
    apply(tmp_path, RESIDUAL)
    info = AppliedHeadKinematics(DEFAULT_HEAD_URDF, tmp_path).calibration()
    assert info.applied and info.source == "head_arm"
    assert (info.session, info.result, info.applied_at) == ("calib-1", "r-1", "2026-10-29T10:00:00+00:00")
    assert info.residual == RESIDUAL and info.residual_mm == 2.9


def test_floor_chain_has_no_arm_frame_residual(tmp_path: Path) -> None:
    apply(tmp_path, None, source="floor")
    info = AppliedHeadKinematics(DEFAULT_HEAD_URDF, tmp_path).calibration()
    assert info.applied and info.source == "floor" and info.residual is None and info.residual_mm is None
    assert "no arm-frame residual" in info.describe()


def test_unreadable_chain_is_not_applied(tmp_path: Path) -> None:
    (tmp_path / "head_chain.json").write_text("{not json")
    assert not AppliedHeadKinematics(DEFAULT_HEAD_URDF, tmp_path).calibration().applied


def test_a_new_apply_changes_the_reported_calibration(tmp_path: Path) -> None:
    apply(tmp_path, RESIDUAL)
    provider = AppliedHeadKinematics(DEFAULT_HEAD_URDF, tmp_path)
    assert provider.calibration().residual_mm == 2.9
    apply(tmp_path, ChainResidual("head_arm", 4.1, 3.0, 2.0), result="r-2")
    marker = tmp_path / "applied.json"
    stat = marker.stat()
    os.utime(marker, ns=(stat.st_atime_ns, stat.st_mtime_ns + 1_000_000_000))  # a distinct mtime on coarse clocks
    info = provider.calibration()
    assert (info.result, info.residual_mm) == ("r-2", 4.1)


def test_describe_names_the_residual_and_the_chain(tmp_path: Path) -> None:
    apply(tmp_path, RESIDUAL)
    text = AppliedHeadKinematics(DEFAULT_HEAD_URDF, tmp_path).calibration().describe()
    assert "head_arm" in text and "calib-1" in text and "2.9 mm" in text


def bump_marker(applied: Path) -> None:
    """Give applied.json a distinct mtime so the provider reloads on coarse clocks."""
    marker = applied / "applied.json"
    stat = marker.stat()
    os.utime(marker, ns=(stat.st_atime_ns, stat.st_mtime_ns + 1_000_000_000))


def test_a_matching_arm_calibration_is_the_residual_and_passes_the_gate(tmp_path: Path) -> None:
    """A ground-board chain has no residual of its own: the arm calibration fitted against it gives one."""
    apply(tmp_path, None, source="ground_board")
    write_arm_calibration(tmp_path, 2.5)
    info = AppliedHeadKinematics(DEFAULT_HEAD_URDF, tmp_path).calibration()
    assert info.residual_source == "arm_calibration" and info.residual_mm == 2.5
    assert gate_state(info, GATE_MM) is True and gate_refusal(info, GATE_MM) is None
    text = info.describe()
    assert "accepted arm to camera calibration" in text and FAKE_ARM_SESSION in text and "2.5 mm" in text
    block = calibration_block(info, GATE_MM)
    assert block["residual_source"] == "arm_calibration" and block["residual_mm"] == 2.5
    assert block["within_gate"] is True and block["source"] == "ground_board"
    assert block["holdout_rms_px"] is None
    assert block["arm_calibration"] == {
        "session": FAKE_ARM_SESSION,
        "result": FAKE_ARM_RESULT,
        "heldout_tool_mm": 2.5,
        "head_chain_session": "calib-1",
        "head_chain_result": "r-1",
        "current": True,
    }


def test_a_matching_arm_calibration_wins_over_the_head_from_arm_residual(tmp_path: Path) -> None:
    apply(tmp_path, ChainResidual("head_arm", 6.0, 3.0, 3.0))
    write_arm_calibration(tmp_path, 1.5)
    info = AppliedHeadKinematics(DEFAULT_HEAD_URDF, tmp_path).calibration()
    assert info.residual_source == "arm_calibration" and info.residual_mm == 1.5
    assert gate_refusal(info, GATE_MM) is None


def test_a_matching_arm_calibration_above_the_gate_refuses(tmp_path: Path) -> None:
    apply(tmp_path, None, source="ground_board")
    write_arm_calibration(tmp_path, 6.5)
    info = AppliedHeadKinematics(DEFAULT_HEAD_URDF, tmp_path).calibration()
    assert gate_state(info, GATE_MM) is False
    refusal = gate_refusal(info, GATE_MM)
    assert refusal is not None
    assert "accepted arm to camera calibration" in refusal and "residual 6.5 mm is above the 5 mm gate" in refusal
    assert "re-run the arm to camera calibration on the web UI Calibrate page" in refusal
    assert "head-from-arm" not in refusal


def test_an_arm_calibration_against_another_head_chain_is_ignored(tmp_path: Path) -> None:
    """Stale: the head chain changed after the arm fit, so its tool error does not describe the applied chain."""
    apply(tmp_path, RESIDUAL)
    write_arm_calibration(tmp_path, 1.0, OTHER_CHAIN)
    info = AppliedHeadKinematics(DEFAULT_HEAD_URDF, tmp_path).calibration()
    assert info.residual_source == "head_from_arm" and info.residual_mm == 2.9
    assert gate_refusal(info, GATE_MM) is None
    assert "stale" in info.describe() and OTHER_CHAIN[0] in info.describe()
    block = calibration_block(info, GATE_MM)
    assert block["residual_source"] == "head_from_arm" and block["arm_calibration"]["current"] is False


def test_a_stale_arm_calibration_and_no_head_from_arm_residual_refuses_as_stale(tmp_path: Path) -> None:
    apply(tmp_path, None, source="ground_board")
    write_arm_calibration(tmp_path, 1.0, OTHER_CHAIN)
    info = AppliedHeadKinematics(DEFAULT_HEAD_URDF, tmp_path).calibration()
    assert info.residual_source is None and gate_state(info, GATE_MM) is None
    refusal = gate_refusal(info, GATE_MM)
    assert refusal is not None and "no arm-frame residual to check against the 5 mm gate" in refusal
    assert "arm to camera calibration is stale for the applied head chain" in refusal
    assert "re-run the arm to camera calibration on the web UI Calibrate page" in refusal


def test_an_arm_calibration_without_a_head_chain_is_ignored(tmp_path: Path) -> None:
    apply(tmp_path, None, source="ground_board")
    write_arm_calibration(tmp_path, 1.0, None)
    info = AppliedHeadKinematics(DEFAULT_HEAD_URDF, tmp_path).calibration()
    assert info.residual_source is None
    assert "stale for the applied head chain" in (gate_refusal(info, GATE_MM) or "")


def test_a_matching_arm_calibration_without_a_tool_error_falls_back(tmp_path: Path) -> None:
    apply(tmp_path, RESIDUAL)
    write_arm_calibration(tmp_path, None)
    info = AppliedHeadKinematics(DEFAULT_HEAD_URDF, tmp_path).calibration()
    assert info.residual_source == "head_from_arm" and info.residual_mm == 2.9
    assert "no held-out tool error" in info.describe()


def test_head_from_arm_residual_without_an_arm_calibration(tmp_path: Path) -> None:
    apply(tmp_path, RESIDUAL)
    info = AppliedHeadKinematics(DEFAULT_HEAD_URDF, tmp_path).calibration()
    assert info.residual_source == "head_from_arm" and info.arm_fit is None
    assert "head-from-arm" in info.describe()
    block = calibration_block(info, GATE_MM)
    assert block["residual_source"] == "head_from_arm" and block["residual_mm"] == 2.9
    assert block["holdout_rms_px"] == 2.5 and block["arm_calibration"] is None
    above = AppliedHeadKinematics(DEFAULT_HEAD_URDF, tmp_path).calibration()
    assert "improve the head-from-arm calibration" in (gate_refusal(above, 2.0) or "")


def test_nothing_applied_refuses_naming_both_calibrations(tmp_path: Path) -> None:
    for info in (
        AppliedHeadKinematics(DEFAULT_HEAD_URDF, None).calibration(),
        AppliedHeadKinematics(DEFAULT_HEAD_URDF, tmp_path).calibration(),
    ):
        assert info.residual_source is None
        refusal = gate_refusal(info, GATE_MM)
        assert refusal is not None and "no arm-frame residual to check against the 5 mm gate" in refusal
        assert "run the arm to camera calibration" in refusal and "head-from-arm calibration" in refusal
        assert calibration_block(info, GATE_MM)["residual_source"] is None


def test_an_unreadable_arm_calibration_is_ignored_and_named(tmp_path: Path) -> None:
    apply(tmp_path, None, source="ground_board")
    write_arm_calibration(tmp_path, 1.0)
    (tmp_path / "arm_calibration.json").write_text("{not json")
    info = AppliedHeadKinematics(DEFAULT_HEAD_URDF, tmp_path).calibration()
    assert info.residual_source is None and info.arm_fit is None
    assert "unreadable" in info.describe()


def test_accepting_an_arm_calibration_reloads_the_residual(tmp_path: Path) -> None:
    apply(tmp_path, None, source="ground_board")
    provider = AppliedHeadKinematics(DEFAULT_HEAD_URDF, tmp_path)
    assert provider.calibration().residual_source is None
    write_arm_calibration(tmp_path, 2.67)
    bump_marker(tmp_path)
    info = provider.calibration()
    assert (info.residual_source, info.residual_mm) == ("arm_calibration", 2.67)
