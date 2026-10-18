"""Tests for scripts/head_rgbd_calibration/urdf_handoff.py on a copy of the head URDF."""

import difflib
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from scripts.head_rgbd_calibration.extrinsics import ExtrinsicResult
from scripts.head_rgbd_calibration.urdf_handoff import (
    check_drift,
    read_chain,
    rewrite_tof_joint,
    tof_joint_origin,
)

HEAD_URDF = Path(__file__).resolve().parents[1] / "nodes" / "web_ui" / "urdf" / "head.urdf"
INLINE_URDF = """<?xml version="1.0"?>
<!-- header comment -->
<robot name="head">
  <link name="head_link"/>
  <link name="head_camera_link"/>
  <joint name="head_camera_joint" type="fixed">
    <parent link="head_link"/>
    <child link="head_camera_link"/>
    <origin xyz="0.02 -0.025 0" rpy="0 0 0"/>
  </joint>
  <link name="head_camera_optical_frame"/>
  <joint name="head_camera_optical_joint" type="fixed">
    <parent link="head_camera_link"/>
    <child link="head_camera_optical_frame"/>
    <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
  </joint>

  <link name="tof_link"/>
  <joint name="tof_joint" type="fixed">
    <parent link="head_link"/>
    <child link="tof_link"/>
    <origin xyz="0.02 0.025 0" rpy="0 0 0"/>
  </joint>
  <link name="tof_optical_frame"/>
  <joint name="tof_optical_joint" type="fixed">
    <parent link="tof_link"/>
    <child link="tof_optical_frame"/>
    <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
  </joint>
</robot>
"""
NOTE = "2026-10-18, stereo RMS 0.31 px"


@pytest.fixture
def urdf_text() -> str:
    """Copy of the head URDF text (inline fixture when the file is missing)."""
    return HEAD_URDF.read_text() if HEAD_URDF.exists() else INLINE_URDF


def make_extrinsic(t: tuple[float, float, float], rotvec: tuple[float, float, float]) -> ExtrinsicResult:
    """Extrinsic with a known pose."""
    return ExtrinsicResult(r=Rotation.from_rotvec(rotvec).as_matrix(), t=np.array(t), stereo_rms=0.3, pair_count=20)


EXTRINSIC = make_extrinsic((-0.0512, -0.0013, 0.0021), (0.004, -0.007, 0.012))


def test_read_chain_parses_joints(urdf_text: str) -> None:
    """The four fixed joints, their parents and children are read."""
    chain = read_chain(urdf_text)
    assert chain.camera.parent == "head_link"
    assert chain.tof.child == "tof_link"
    assert chain.camera_optical.rpy == pytest.approx((-1.5708, 0, -1.5708))
    assert chain.tof.origin.xyz[1] == pytest.approx(0.025)


def test_missing_joint_raises(urdf_text: str) -> None:
    """A URDF without tof_optical_joint raises naming it."""
    broken = urdf_text.replace("tof_optical_joint", "renamed_joint")
    with pytest.raises(ValueError, match="tof_optical_joint"):
        read_chain(broken)


def test_rewrite_reproduces_extrinsic(urdf_text: str) -> None:
    """After rewriting, the URDF transform camopt <- tofopt equals the extrinsic."""
    origin = tof_joint_origin(read_chain(urdf_text), EXTRINSIC)
    out = rewrite_tof_joint(urdf_text, origin, NOTE)
    chain = read_chain(out)
    assert chain.tof.origin.xyz == pytest.approx(origin.xyz, abs=1e-6)
    drift = check_drift(chain, EXTRINSIC)
    assert drift.translation_mm < 1e-3
    assert drift.rotation_deg < np.degrees(1e-6)


def test_diff_touches_only_origin_and_comment(urdf_text: str) -> None:
    """Only the tof_joint origin line and the calibration comment change."""
    origin = tof_joint_origin(read_chain(urdf_text), EXTRINSIC)
    out = rewrite_tof_joint(urdf_text, origin, NOTE)
    changed = [
        line
        for line in difflib.unified_diff(urdf_text.splitlines(), out.splitlines(), lineterm="", n=0)
        if line[:1] in "+-" and line[:3] not in ("---", "+++")
    ]
    assert len(changed) == 3
    assert sum("<origin" in line for line in changed) == 2
    assert sum(NOTE in line for line in changed) == 1
    assert all(line.startswith("+") for line in changed if NOTE in line)


def test_second_rewrite_replaces_comment(urdf_text: str) -> None:
    """A second rewrite leaves one calibration comment with the new note."""
    origin = tof_joint_origin(read_chain(urdf_text), EXTRINSIC)
    once = rewrite_tof_joint(urdf_text, origin, NOTE)
    twice = rewrite_tof_joint(once, origin, "2026-11-01, stereo RMS 0.20 px")
    assert twice.count("tof_joint calibrated") == 1
    assert "2026-11-01" in twice and "2026-10-18" not in twice
    assert twice.count("<!--") == once.count("<!--")


def test_drift_equals_injected_offset(urdf_text: str) -> None:
    """Before a rewrite the drift equals the offset between the URDF and the calibration."""
    chain = read_chain(urdf_text)
    origin = tof_joint_origin(chain, EXTRINSIC)
    calibrated = read_chain(rewrite_tof_joint(urdf_text, origin, NOTE))
    shifted = make_extrinsic(tuple(EXTRINSIC.t + np.array([0.003, 0.0, 0.0])), (0.004, -0.007, 0.012))
    drift = check_drift(calibrated, shifted)
    assert drift.translation_mm == pytest.approx(3.0, abs=1e-3)
    assert drift.rotation_deg < 1e-4
    rotated = ExtrinsicResult(
        r=Rotation.from_rotvec([0, 0, np.radians(2.0)]).as_matrix() @ EXTRINSIC.r,
        t=EXTRINSIC.t,
        stereo_rms=0.3,
        pair_count=20,
    )
    assert check_drift(calibrated, rotated).rotation_deg == pytest.approx(2.0, abs=1e-3)
