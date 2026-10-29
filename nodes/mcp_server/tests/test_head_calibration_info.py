"""AppliedHeadKinematics.calibration(): which applied head chain is in effect and its arm-frame residual."""

import json
import os
from pathlib import Path

from ros2_common.head_rgbd_calibration.urdf_text import ChainResidual, Origin, head_chain_json

from mcp_server.head import AppliedHeadKinematics
from mcp_server.head_kinematics import DEFAULT_HEAD_URDF

ORIGINS = {"head_tilt": Origin((0.0, 0.0, 0.0412), (0.0, 0.0123, 0.0))}
RESIDUAL = ChainResidual(kind="head_arm", holdout_board_mm=2.9, holdout_rms_px=2.5, rms_px=2.7)


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
