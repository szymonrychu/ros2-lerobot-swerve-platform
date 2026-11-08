"""Tests for scripts/markers/jaw_fit.py: the URDF-posed SO-101 gripper meshes, jaw gap, contact angle, penetration."""

import math
from pathlib import Path

import numpy as np
import pytest
import trimesh

from scripts.markers.jaw_fit import (
    MAX_FINGER_DEPTH_MM,
    OPEN_AXIS,
    TOOL_AXIS,
    WIDTH_AXIS,
    Gripper,
    clearance_mm,
    contact_angle_rad,
    fixed_jaw_face_mm,
    jaw_gap_mm,
    load_gripper,
    moving_jaw,
    moving_tip_mm,
    penetration_mm,
)
from scripts.markers.mesh import box, to_trimesh

REPO_ROOT = Path(__file__).resolve().parents[1]
ARM_URDF = REPO_ROOT / "nodes" / "web_ui" / "urdf" / "so101_arm.urdf"
SLAB_DEPTH_MM = 30.0
SLAB_HALF_WIDTH_MM = 15.0

GRIPPER = load_gripper(ARM_URDF)


def slab_between_jaws(gripper: Gripper, thickness_mm: float) -> trimesh.Trimesh:
    """Box `thickness_mm` thick along OPEN_AXIS from the fixed jaw face, spanning 30 mm of depth behind the tips.

    Args:
        gripper (Gripper): Loaded gripper.
        thickness_mm (float): Slab thickness along OPEN_AXIS (mm).

    Returns:
        trimesh.Trimesh: The slab in gripper_link (mm).
    """
    face = fixed_jaw_face_mm(gripper, (0.0, SLAB_DEPTH_MM))
    tip = moving_tip_mm(gripper, 0.0)
    tip_z = float(tip @ np.asarray(TOOL_AXIS))
    centre_y = float(tip @ np.asarray(WIDTH_AXIS))
    # TOOL_AXIS is gripper_link -z: the tips are at z = -tip_z, the depth runs toward +z
    return to_trimesh(
        box(
            face,
            centre_y - SLAB_HALF_WIDTH_MM,
            -tip_z,
            face + thickness_mm,
            centre_y + SLAB_HALF_WIDTH_MM,
            -tip_z + SLAB_DEPTH_MM,
        )
    )


def test_load_gripper_places_the_tool_frame() -> None:
    assert GRIPPER.tool[:3, 3] == pytest.approx([-7.9, -0.218121, -98.1274], abs=1e-3)


def test_moving_jaw_turns_about_the_gripper_joint() -> None:
    a, b = moving_jaw(GRIPPER, 0.0), moving_jaw(GRIPPER, 0.3)
    pivot = GRIPPER.joint[:3, 3]
    assert np.linalg.norm(a.vertices - pivot, axis=1) == pytest.approx(np.linalg.norm(b.vertices - pivot, axis=1))


def test_jaw_gap_matches_the_repo_meshes() -> None:
    """Pin the gap to the repo URDF-posed meshes (the source of truth).

    The earlier research figures (18 mm tip, 23.5 mm at 20 mm in, at 0 deg) used a different sampling
    definition and are inconsistent with the URDF pivot, so they are not used here.
    """
    assert jaw_gap_mm(GRIPPER, 0.0, 0.0) == pytest.approx(15.8, abs=0.5)
    assert jaw_gap_mm(GRIPPER, 0.0, 20.0) == pytest.approx(17.8, abs=0.5)
    assert jaw_gap_mm(GRIPPER, math.radians(-10.0), 0.0) == pytest.approx(2.0, abs=0.5)
    gaps = [jaw_gap_mm(GRIPPER, math.radians(d), 0.0) for d in range(0, 11)]
    assert gaps == sorted(gaps) and 14.0 <= gaps[0] and gaps[-1] <= 31.0


def test_contact_angle_grows_with_thickness() -> None:
    thin, thick = (slab_between_jaws(GRIPPER, t) for t in (16.0, 22.0))
    assert contact_angle_rad(GRIPPER, thin) < contact_angle_rad(GRIPPER, thick)


def test_contact_angle_closes_the_tip_on_a_thin_slab() -> None:
    angle = contact_angle_rad(GRIPPER, slab_between_jaws(GRIPPER, 10.0))
    assert jaw_gap_mm(GRIPPER, angle, 0.0) == pytest.approx(10.0, abs=0.2)


def test_contact_angle_refuses_an_oversized_slab() -> None:
    with pytest.raises(ValueError, match="penetrat"):
        contact_angle_rad(GRIPPER, slab_between_jaws(GRIPPER, 120.0))


def test_contact_angle_refuses_a_slab_thinner_than_the_closed_gap() -> None:
    with pytest.raises(ValueError, match="thinner than the closed jaw gap"):
        contact_angle_rad(GRIPPER, slab_between_jaws(GRIPPER, 0.5))


def test_fixed_jaw_face_refuses_depths_past_the_finger() -> None:
    assert fixed_jaw_face_mm(GRIPPER, (0.0, MAX_FINGER_DEPTH_MM)) == pytest.approx(-7.9, abs=0.1)
    with pytest.raises(ValueError, match="past the finger"):
        fixed_jaw_face_mm(GRIPPER, (0.0, 70.0))
    with pytest.raises(ValueError, match="past the finger"):
        jaw_gap_mm(GRIPPER, 0.0, 70.0)


def test_load_gripper_names_a_missing_urdf_element(tmp_path: Path) -> None:
    urdf = tmp_path / "bad.urdf"
    urdf.write_text(
        "<robot name='r'><link name='gripper_link'/>"
        "<joint name='gripper' type='revolute'><parent link='gripper_link'/></joint>"
        "<joint name='gripper_frame_joint' type='fixed'><parent link='gripper_link'/></joint></robot>"
    )
    with pytest.raises(ValueError, match="child"):
        load_gripper(urdf)


def test_penetration_and_clearance_of_boxes() -> None:
    a = to_trimesh(box(0, 0, 0, 10, 10, 10))
    assert penetration_mm(a, to_trimesh(box(8, 0, 0, 18, 10, 10))) == pytest.approx(2.0, abs=0.05)
    assert clearance_mm(a, to_trimesh(box(13, 0, 0, 20, 10, 10))) == pytest.approx(3.0, abs=0.05)


def test_penetration_is_zero_for_disjoint_meshes() -> None:
    a = to_trimesh(box(0, 0, 0, 10, 10, 10))
    assert penetration_mm(a, to_trimesh(box(13, 0, 0, 20, 10, 10))) == 0.0


def test_fixed_jaw_face_is_the_tool_point_plane_at_the_tip() -> None:
    # gripper_frame_link sits on the fixed jaw inner face, so the tip band face matches the tool point x
    assert fixed_jaw_face_mm(GRIPPER, (0.0, 5.0)) == pytest.approx(float(GRIPPER.tool[0, 3]), abs=0.1)


def test_moving_tip_is_the_finger_extreme_on_the_mid_plane() -> None:
    tip = moving_tip_mm(GRIPPER, 0.0)
    depths = moving_jaw(GRIPPER, 0.0).vertices @ np.asarray(TOOL_AXIS)
    assert float(tip @ np.asarray(TOOL_AXIS)) == pytest.approx(float(depths.max()))
    assert abs(float(tip @ np.asarray(WIDTH_AXIS))) < 1.0  # the finger is centred on the gripper_link xz plane


def test_moving_jaw_opens_along_the_open_axis() -> None:
    closed, opened = moving_tip_mm(GRIPPER, 0.0), moving_tip_mm(GRIPPER, 0.2)
    assert float((opened - closed) @ np.asarray(OPEN_AXIS)) > 0.0
