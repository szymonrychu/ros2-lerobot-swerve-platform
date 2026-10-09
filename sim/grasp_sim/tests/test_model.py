"""The vendored SO-101 model loads and agrees with the repo URDF (so101_new_calib)."""

import mujoco
import numpy as np
import pytest
from urdf_fk import link_pose, load_urdf

from grasp_sim.scene import ARM_XML, JOINT_NAMES, build_model

FK_TOLERANCE_M = 0.002
ROT_TOLERANCE = 1e-3
BODY_TO_LINK = {
    "base": "base_link",
    "shoulder": "shoulder_link",
    "upper_arm": "upper_arm_link",
    "lower_arm": "lower_arm_link",
    "wrist": "wrist_link",
    "gripper": "gripper_link",
    "moving_jaw_so101_v1": "moving_jaw_so101_v1_link",
}
# URDF gripper_frame_link origin expressed in the gripper_link frame (the fixed jaw inner face tip).
URDF_GRIPPER_FRAME = ("gripper_link", "gripper_frame_link")
CONFIGS = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.4, 0.5, -1.0, 0.5, 0.0, 1.0],
    [-0.7, -0.3, 1.2, -0.8, 1.5, 0.3],
    [1.2, 1.0, -1.5, 1.2, -1.57, 1.7],
    [0.1, 1.7, -0.4, -1.4, 2.5, 0.0],
    [-1.5, -1.5, 1.6, 1.5, -2.5, 0.9],
]


def test_arm_xml_vendored_with_license() -> None:
    assert ARM_XML.exists()
    assert "Apache License" in (ARM_XML.parent / "LICENSE").read_text()
    assert "0059d4335f8156206f63a35662313385f7ad6d74" in (ARM_XML.parent / "README.md").read_text()


def test_model_loads_with_six_position_actuators(floor_scene) -> None:
    model = build_model(floor_scene)
    assert [model.actuator(i).name for i in range(model.nu)] == list(JOINT_NAMES)
    assert [model.joint(model.actuator_trnid[i, 0]).name for i in range(model.nu)] == list(JOINT_NAMES)


@pytest.mark.parametrize("q", CONFIGS)
def test_mujoco_body_frames_match_urdf_fk(floor_scene, urdf_path, q) -> None:
    model = build_model(floor_scene)
    data = mujoco.MjData(model)
    joint_q = dict(zip(JOINT_NAMES, q, strict=True))
    for name, value in joint_q.items():
        data.qpos[model.joint(name).qposadr[0]] = value
    mujoco.mj_kinematics(model, data)
    urdf = load_urdf(urdf_path)
    for body, link in BODY_TO_LINK.items():
        expected = link_pose(urdf, link, joint_q)
        pos = data.body(body).xpos
        rot = data.body(body).xmat.reshape(3, 3)
        assert np.linalg.norm(pos - expected[:3, 3]) < FK_TOLERANCE_M, f"{body} position at {q}"
        assert np.abs(rot - expected[:3, :3]).max() < ROT_TOLERANCE, f"{body} orientation at {q}"


@pytest.mark.parametrize("q", CONFIGS)
def test_gripper_frame_matches_urdf_gripper_frame_link(floor_scene, urdf_path, q) -> None:
    """The URDF gripper_frame_link origin, carried by the MuJoCo gripper body, lands on the URDF FK within 2 mm."""
    model = build_model(floor_scene)
    data = mujoco.MjData(model)
    joint_q = dict(zip(JOINT_NAMES, q, strict=True))
    for name, value in joint_q.items():
        data.qpos[model.joint(name).qposadr[0]] = value
    mujoco.mj_kinematics(model, data)
    urdf = load_urdf(urdf_path)
    local = urdf[URDF_GRIPPER_FRAME[1]].origin[:3, 3]
    expected = link_pose(urdf, URDF_GRIPPER_FRAME[1], joint_q)[:3, 3]
    body = data.body("gripper")
    actual = body.xpos + body.xmat.reshape(3, 3) @ local
    assert np.linalg.norm(actual - expected) < FK_TOLERANCE_M


def test_menagerie_gripperframe_site_is_offset_along_jaw_axis(floor_scene, urdf_path) -> None:
    """The Menagerie gripperframe site sits about 2 cm from the URDF gripper_frame_link along the jaw axis (documented)."""
    model = build_model(floor_scene)
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    urdf = load_urdf(urdf_path)
    expected = link_pose(urdf, "gripper_frame_link", {})[:3, 3]
    offset = np.linalg.norm(data.site("gripperframe").xpos - expected)
    assert 0.015 < offset < 0.025


def test_joint_ranges_match_urdf_limits_except_documented_override(floor_scene) -> None:
    model = build_model(floor_scene)
    lift = model.joint("shoulder_lift").range
    assert lift[0] == pytest.approx(-1.745, abs=1e-3)
    assert lift[1] == pytest.approx(1.9, abs=1e-6)
    assert model.joint("elbow_flex").range[1] == pytest.approx(1.69, abs=1e-3)
    assert model.actuator("shoulder_lift").ctrlrange[1] == pytest.approx(1.9, abs=1e-6)
