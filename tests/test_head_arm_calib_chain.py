"""Tests for the URDF forward-kinematics chain in shared/ros2_common/head_rgbd_calibration/chain.py."""

from pathlib import Path

import numpy as np
import pytest
from ros2_common.head_rgbd_calibration.chain import (
    UrdfChain,
    arm_in_robot_base,
    pose_from_xyz_rpy,
    xyz_rpy_from_pose,
)

URDF_DIR = Path(__file__).resolve().parents[1] / "nodes" / "web_ui" / "urdf"
OPTICAL_RPY = (-1.5707963, 0.0, -1.5707963)
CAMERA = "d435_color_optical_frame"
ZERO_HEAD = {"head_pan": 0.0, "head_tilt": 0.0}
DEFAULT_GAUGE = {"x": 0.0592, "y": -0.05, "z": 0.100, "yaw": 0.0}


@pytest.fixture(scope="module")
def head() -> UrdfChain:
    return UrdfChain.from_text((URDF_DIR / "head.urdf").read_text())


@pytest.fixture(scope="module")
def arm() -> UrdfChain:
    return UrdfChain.from_text((URDF_DIR / "so101_arm.urdf").read_text())


def translate(x: float, y: float, z: float) -> np.ndarray:
    return pose_from_xyz_rpy((x, y, z), (0.0, 0.0, 0.0))


def test_d435_colour_pose_at_zero_matches_urdf_origins(head: UrdfChain) -> None:
    expected = (
        translate(0.15, 0.10, 0.31)
        @ translate(0, 0, 0.04)
        @ translate(0.02, 0, 0)
        @ translate(0, -0.015, 0)
        @ pose_from_xyz_rpy((0, 0, 0), OPTICAL_RPY)
    )
    np.testing.assert_allclose(head.transform(CAMERA, ZERO_HEAD), expected, atol=1e-12)
    assert head.joint_names(CAMERA) == [
        "head_mount_joint",
        "head_pan",
        "head_tilt",
        "d435_joint",
        "d435_color_joint",
        "d435_color_optical_joint",
    ]


def test_pan_left_and_tilt_down_signs(head: UrdfChain) -> None:
    base = head.transform(CAMERA, ZERO_HEAD)
    panned = head.transform(CAMERA, {"head_pan": 0.5, "head_tilt": 0.0})
    forward0, forward1 = base[:3, 2], panned[:3, 2]
    assert np.arctan2(forward1[1], forward1[0]) - np.arctan2(forward0[1], forward0[0]) == pytest.approx(0.5)
    tilted = head.transform(CAMERA, {"head_pan": 0.0, "head_tilt": 0.3})
    assert tilted[2, 2] < -0.2
    assert base[2, 2] == pytest.approx(0.0, abs=1e-4)


def test_origin_override_and_offsets(head: UrdfChain) -> None:
    override = {"head_mount_joint": ((0.15, 0.10, 0.25), (0.0, 0.0, 0.0))}
    moved = head.transform(CAMERA, ZERO_HEAD, origins=override)
    base = head.transform(CAMERA, ZERO_HEAD)
    np.testing.assert_allclose(moved[:3, :3], base[:3, :3], atol=1e-12)
    np.testing.assert_allclose(moved[:3, 3] - base[:3, 3], [0, 0, -0.06], atol=1e-12)
    assert head.origin("head_mount_joint") == ((0.15, 0.10, 0.31), (0.0, 0.0, 0.0))
    a = head.transform(CAMERA, {"head_pan": 0.2, "head_tilt": 0.1}, offsets={"head_tilt": 0.05})
    b = head.transform(CAMERA, {"head_pan": 0.2, "head_tilt": 0.15})
    np.testing.assert_allclose(a, b, atol=1e-12)


def test_offsets_add_to_measured(head: UrdfChain) -> None:
    a = head.transform(CAMERA, {"head_pan": 0.2, "head_tilt": 0.1}, offsets={"head_tilt": 0.05})
    b = head.transform(CAMERA, {"head_pan": 0.2, "head_tilt": 0.15})
    np.testing.assert_allclose(a, b, atol=1e-12)


def test_missing_joint_raises(head: UrdfChain) -> None:
    with pytest.raises(KeyError, match="head_tilt"):
        head.transform(CAMERA, {"head_pan": 0.0})
    with pytest.raises(KeyError, match="nope"):
        head.transform("nope", ZERO_HEAD)


def test_limits(head: UrdfChain) -> None:
    assert head.limits("head_pan") == pytest.approx((-1.5708, 1.5708))
    assert head.limits("head_mount_joint") is None


def test_unsupported_joint_type_names_the_joint() -> None:
    text = (
        '<robot name="r"><link name="a"/><link name="b"/>'
        '<joint name="slider" type="prismatic"><parent link="a"/><child link="b"/></joint></robot>'
    )
    with pytest.raises(ValueError, match="slider"):
        UrdfChain.from_text(text)


def test_continuous_joint_has_no_limits() -> None:
    text = (
        '<robot name="r"><link name="a"/><link name="b"/>'
        '<joint name="spin" type="continuous"><parent link="a"/><child link="b"/><axis xyz="1 0 0"/></joint></robot>'
    )
    chain = UrdfChain.from_text(text)
    assert chain.limits("spin") is None
    assert chain.transform("b", {"spin": np.pi / 2})[1, 2] == pytest.approx(-1.0)


def test_so101_gripper_link_at_zero_is_the_product_of_origins(arm: UrdfChain) -> None:
    zero = {n: 0.0 for n in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")}
    names = arm.joint_names("gripper_link")
    # Literals copied from so101_arm.urdf, independent of the parser under test.
    literal_origins = [
        ((0.0388353, -8.97657e-09, 0.0624), (3.14159, 4.18253e-17, -3.14159)),
        ((-0.0303992, -0.0182778, -0.0542), (-1.5708, -1.5708, 0.0)),
        ((-0.11257, -0.028, 1.73763e-16), (-3.63608e-16, 8.74301e-16, 1.5708)),
        ((-0.1349, 0.0052, 3.62355e-17), (4.02456e-15, 8.67362e-16, -1.5708)),
        ((5.55112e-17, -0.0611, 0.0181), (1.5708, 0.0486795, 3.14159)),
    ]
    expected = np.eye(4)
    for xyz, rpy in literal_origins:
        expected = expected @ pose_from_xyz_rpy(xyz, rpy)
    np.testing.assert_allclose(arm.transform("gripper_link", zero), expected, atol=1e-12)
    assert names == ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


def test_xyz_rpy_round_trip() -> None:
    xyz, rpy = (0.1, -0.2, 0.3), (0.4, -0.5, 0.6)
    got_xyz, got_rpy = xyz_rpy_from_pose(pose_from_xyz_rpy(xyz, rpy))
    np.testing.assert_allclose(got_xyz, xyz, atol=1e-12)
    np.testing.assert_allclose(got_rpy, rpy, atol=1e-12)


def test_arm_in_robot_base_default_gauge() -> None:
    pose = arm_in_robot_base(DEFAULT_GAUGE)
    np.testing.assert_allclose(pose @ [0, 0, 0, 1], [0.0592, -0.05, 0.100, 1.0], atol=1e-12)
    yawed = arm_in_robot_base({**DEFAULT_GAUGE, "yaw": np.pi / 2})
    np.testing.assert_allclose(yawed[:3, 0], [0, 1, 0], atol=1e-12)
