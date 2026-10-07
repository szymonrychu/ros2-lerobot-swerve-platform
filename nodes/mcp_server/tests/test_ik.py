"""Tests for mcp_server.ik (ikpy wrapper on the SO101 URDF): limits, FK, IK round trip, unreachable targets."""

import math

import numpy as np
import pytest

from mcp_server.config import McpServerConfig
from mcp_server.ik import ARM_CHAIN_JOINTS, ArmKinematics, UnreachableError, load_joint_limits

URDF = McpServerConfig().arm.urdf_path
MARGIN = 0.05


@pytest.fixture(scope="module")
def kin() -> ArmKinematics:
    return ArmKinematics(URDF, margin=MARGIN)


def test_load_joint_limits_reads_all_six_joints() -> None:
    limits = load_joint_limits(URDF)
    assert limits["shoulder_pan"] == pytest.approx((-1.91986, 1.91986))
    assert limits["shoulder_lift"] == pytest.approx((-1.74533, 1.74533))
    assert limits["elbow_flex"] == pytest.approx((-1.69, 1.69))
    assert limits["wrist_flex"] == pytest.approx((-1.65806, 1.65806))
    assert limits["wrist_roll"] == pytest.approx((-2.74385, 2.84121))
    assert limits["gripper"] == pytest.approx((-0.174533, 1.74533))


def test_chain_is_the_five_arm_joints(kin: ArmKinematics) -> None:
    assert ARM_CHAIN_JOINTS == ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
    assert kin.joint_names == ARM_CHAIN_JOINTS


def test_forward_at_zero_is_arm_stretched_forward(kin: ArmKinematics) -> None:
    pose = kin.forward({j: 0.0 for j in ARM_CHAIN_JOINTS})
    assert pose.x == pytest.approx(0.391, abs=0.005)
    assert abs(pose.y) < 0.01
    assert pose.z == pytest.approx(0.226, abs=0.005)
    assert pose.pitch == pytest.approx(0.0, abs=0.06)


def test_forward_pitch_positive_when_pointing_down(kin: ArmKinematics) -> None:
    zero = {j: 0.0 for j in ARM_CHAIN_JOINTS}
    down = kin.forward(zero | {"wrist_flex": 1.0})
    up = kin.forward(zero | {"wrist_flex": -1.0})
    assert down.pitch == pytest.approx(1.0, abs=0.05)
    assert up.pitch == pytest.approx(-1.0, abs=0.05)
    assert down.z < up.z


def random_joints(rng: np.random.Generator, kin: ArmKinematics) -> dict[str, float]:
    out = {}
    for j in ARM_CHAIN_JOINTS:
        lo, hi = kin.limits[j]
        out[j] = float(rng.uniform(lo + 0.2, hi - 0.2) * 0.6)
    return out


def test_ik_round_trip_with_pitch(kin: ArmKinematics) -> None:
    rng = np.random.default_rng(42)
    for _ in range(15):
        q = random_joints(rng, kin)
        target = kin.forward(q)
        seed = {j: 0.0 for j in ARM_CHAIN_JOINTS} | {"wrist_roll": q["wrist_roll"]}
        sol = kin.inverse(target.x, target.y, target.z, target.pitch, seed=seed)
        got = kin.forward(sol)
        err = math.dist((got.x, got.y, got.z), (target.x, target.y, target.z))
        assert err < 0.003, (q, sol, err)
        assert abs(got.pitch - target.pitch) < math.radians(2.0)


def test_ik_position_only(kin: ArmKinematics) -> None:
    q = {"shoulder_pan": 0.3, "shoulder_lift": -0.4, "elbow_flex": 0.6, "wrist_flex": 0.5, "wrist_roll": 0.0}
    t = kin.forward(q)
    sol = kin.inverse(t.x, t.y, t.z, None, seed={j: 0.0 for j in ARM_CHAIN_JOINTS})
    got = kin.forward(sol)
    assert math.dist((got.x, got.y, got.z), (t.x, t.y, t.z)) < 0.003


def test_ik_keeps_wrist_roll_from_seed(kin: ArmKinematics) -> None:
    q = {"shoulder_pan": 0.2, "shoulder_lift": 0.2, "elbow_flex": 0.3, "wrist_flex": 0.4, "wrist_roll": 0.7}
    t = kin.forward(q)
    sol = kin.inverse(t.x, t.y, t.z, t.pitch, seed=q | {"shoulder_lift": 0.0})
    assert sol["wrist_roll"] == pytest.approx(0.7)


def test_ik_solution_respects_limits_minus_margin(kin: ArmKinematics) -> None:
    rng = np.random.default_rng(7)
    for _ in range(5):
        q = random_joints(rng, kin)
        t = kin.forward(q)
        sol = kin.inverse(t.x, t.y, t.z, t.pitch, seed={j: 0.0 for j in ARM_CHAIN_JOINTS})
        for j, v in sol.items():
            lo, hi = kin.limits[j]
            assert lo + MARGIN - 1e-9 <= v <= hi - MARGIN + 1e-9


def test_unreachable_far_target_raises(kin: ArmKinematics) -> None:
    with pytest.raises(UnreachableError):
        kin.inverse(2.0, 0.0, 0.2, None, seed={j: 0.0 for j in ARM_CHAIN_JOINTS})


def test_unreachable_pitch_raises(kin: ArmKinematics) -> None:
    # Fully stretched forward the gripper cannot point straight up.
    with pytest.raises(UnreachableError):
        kin.inverse(0.385, 0.0, 0.226, -math.pi / 2, seed={j: 0.0 for j in ARM_CHAIN_JOINTS})


def test_inverse_rejects_non_finite(kin: ArmKinematics) -> None:
    with pytest.raises(UnreachableError):
        kin.inverse(float("nan"), 0.0, 0.2, None, seed={j: 0.0 for j in ARM_CHAIN_JOINTS})


def test_link_frame_base_and_end_match_chain(kin: ArmKinematics) -> None:
    joints = {"shoulder_pan": 0.3, "shoulder_lift": -0.4, "elbow_flex": 0.8, "wrist_flex": 0.2, "wrist_roll": 0.5}
    assert np.allclose(kin.link_frame(joints, "base_link"), np.eye(4))
    end = kin.link_frame(joints, "gripper_frame_link")
    pose = kin.forward(joints)
    assert end[:3, 3] == pytest.approx([pose.x, pose.y, pose.z])


def test_link_frame_gripper_link_is_the_wrist_roll_child(kin: ArmKinematics) -> None:
    joints = {j: 0.0 for j in ARM_CHAIN_JOINTS}
    gripper = kin.link_frame(joints, "gripper_link")
    end = kin.link_frame(joints, "gripper_frame_link")
    # gripper_frame_joint is a fixed offset of (-0.0079, -0.0002, -0.0981) m in gripper_link.
    assert np.linalg.norm(gripper[:3, 3] - end[:3, 3]) == pytest.approx(math.hypot(0.0079, 0.0981), abs=0.002)
    panned = kin.link_frame({**joints, "shoulder_pan": 0.7}, "gripper_link")
    assert panned[1, 3] != pytest.approx(gripper[1, 3], abs=0.05)


def test_link_frame_rejects_unknown_and_off_chain_links(kin: ArmKinematics) -> None:
    with pytest.raises(ValueError, match="moving_jaw_so101_v1_link"):
        kin.link_frame({}, "moving_jaw_so101_v1_link")
    with pytest.raises(ValueError, match="nope"):
        kin.link_frame({}, "nope")
