"""Runtime arm calibration swap: ArmKinematics.set_offsets and SelfCollisionChecker.set_mount give the same answers as
objects built from the new values (mcp_server swaps an accepted arm to camera calibration into the running objects)."""

import numpy as np
import pytest

from arm_collision.body_model import BodyModel
from arm_collision.head import HeadModel, HeadPose
from arm_collision.ik import ArmKinematics
from arm_collision.jaw import JawModel
from arm_collision.self_collision import SelfCollisionChecker
from arm_collision.settings import REPO_ROOT, ArmBaseOffset, BodyCheckSettings, load_robot_body

URDF = REPO_ROOT / "nodes" / "web_ui" / "urdf" / "so101_arm.urdf"
MARGIN_RAD = 0.05
OLD_OFFSETS = {"shoulder_pan": 0.0182, "shoulder_lift": -0.0345, "elbow_flex": -0.1070, "wrist_flex": 0.0988}
NEW_OFFSETS = {
    "shoulder_pan": 0.03,
    "shoulder_lift": -0.02,
    "elbow_flex": -0.09,
    "wrist_flex": 0.08,
    "wrist_roll": 0.01,
}
OLD_MOUNT = ArmBaseOffset(x=0.0592, y=-0.05, z=0.100, yaw=0.0)
NEW_MOUNT = ArmBaseOffset(x=0.0610, y=-0.048, z=0.102, yaw=0.01)
POSE = {"shoulder_pan": 0.3, "shoulder_lift": -0.4, "elbow_flex": 0.9, "wrist_flex": 0.5, "wrist_roll": 0.2}
POSES = [POSE, {**POSE, "shoulder_lift": 0.6, "elbow_flex": -0.2, "gripper": 0.4}]


def kinematics(offsets: dict[str, float]) -> ArmKinematics:
    """Kinematics on the arm URDF with the given offsets."""
    return ArmKinematics(URDF, margin=MARGIN_RAD, joint_offsets=offsets)


def checker(kin: ArmKinematics, mount: ArmBaseOffset) -> SelfCollisionChecker:
    """Body checker of the packaged body on a mount."""
    jaw = JawModel(kin, (-1.0, 0.0, 0.0), -0.165)
    return SelfCollisionChecker(kin, BodyModel.from_settings(load_robot_body()), BodyCheckSettings(), mount, jaw)


def tool(kin: ArmKinematics, joints: dict[str, float]) -> tuple[float, float, float, float]:
    """Tool point and pitch of a configuration."""
    pose = kin.forward(joints)
    return pose.x, pose.y, pose.z, pose.pitch


def test_set_offsets_matches_kinematics_built_with_them() -> None:
    kin = kinematics(OLD_OFFSETS)
    x, y, z, pitch = tool(kin, POSE)
    kin.inverse(x, y, z, pitch, POSE)  # fill the solution caches with an answer of the old offsets
    kin.core_model()
    kin.set_offsets(NEW_OFFSETS)
    fresh = kinematics(NEW_OFFSETS)
    assert kin.offsets == fresh.offsets
    assert tool(kin, POSE) == tool(fresh, POSE)
    assert not kin.solutions
    assert kin.inverse(x, y, z, pitch, POSE) == pytest.approx(fresh.inverse(x, y, z, pitch, POSE), abs=1e-9)


def test_set_offsets_keeps_the_offsets_dict_object() -> None:
    kin = kinematics(OLD_OFFSETS)
    shared = kin.offsets
    kin.set_offsets(NEW_OFFSETS)
    assert shared is kin.offsets and shared["wrist_roll"] == NEW_OFFSETS["wrist_roll"]


def test_set_offsets_defaults_missing_joints_to_zero() -> None:
    kin = kinematics(NEW_OFFSETS)
    kin.set_offsets({"shoulder_pan": 0.1})
    assert kin.offsets == {
        "shoulder_pan": 0.1,
        "shoulder_lift": 0.0,
        "elbow_flex": 0.0,
        "wrist_flex": 0.0,
        "wrist_roll": 0.0,
    }


def test_set_offsets_rejects_a_non_chain_joint_and_keeps_the_old_offsets() -> None:
    kin = kinematics(OLD_OFFSETS)
    with pytest.raises(ValueError, match="gripper"):
        kin.set_offsets({**NEW_OFFSETS, "gripper": 0.1})
    assert kin.offsets["shoulder_pan"] == OLD_OFFSETS["shoulder_pan"]


def test_set_mount_matches_a_checker_built_on_the_new_mount() -> None:
    kin = kinematics(OLD_OFFSETS)
    swapped = checker(kin, OLD_MOUNT)
    before = swapped.closest(POSES)
    swapped.set_mount(NEW_MOUNT)
    fresh = checker(kin, NEW_MOUNT)
    assert swapped.mount == NEW_MOUNT
    assert np.array_equal(swapped.rotation, fresh.rotation) and np.array_equal(swapped.offset, fresh.offset)
    assert swapped.closest(POSES) == fresh.closest(POSES) != before
    assert swapped.pose(POSE) == fresh.pose(POSE)
    assert np.allclose(swapped.to_base_link(np.zeros(3)), [NEW_MOUNT.x, NEW_MOUNT.y, NEW_MOUNT.z])


def test_set_mount_forgets_the_posed_checker() -> None:
    swapped = checker(kinematics(OLD_OFFSETS), OLD_MOUNT)
    swapped.last_posed = (HeadPose(0.0, 0.5), swapped)
    swapped.set_mount(NEW_MOUNT)
    assert swapped.last_posed is None


def test_a_posed_checker_built_on_the_old_mount_is_not_reused() -> None:
    """A read-only planner may store a posed checker of the old mount while set_mount runs (outside the motion lock):
    the next posed() for the same head pose must not hand it out."""
    swapped = checker(kinematics(OLD_OFFSETS), OLD_MOUNT)
    swapped.head = HeadModel.from_urdf()
    head = HeadPose(0.0, 0.5)
    stale = swapped.posed(head)
    assert stale is not swapped and stale.mount == OLD_MOUNT
    swapped.set_mount(NEW_MOUNT)
    swapped.last_posed = (head, stale)  # the late store of a concurrent posed() call
    assert swapped.posed(head).mount == NEW_MOUNT
