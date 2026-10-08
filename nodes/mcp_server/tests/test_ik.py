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


# --- joint zero offsets: urdf_angle = measured_angle + offset ---

OFFSETS = {"shoulder_pan": 0.05, "shoulder_lift": -0.12, "elbow_flex": 0.09, "wrist_flex": 0.2, "wrist_roll": -0.07}


@pytest.fixture(scope="module")
def kin_off() -> ArmKinematics:
    return ArmKinematics(URDF, margin=MARGIN, joint_offsets=OFFSETS)


def test_forward_applies_offsets_to_measured_angles(kin: ArmKinematics, kin_off: ArmKinematics) -> None:
    measured = {"shoulder_pan": 0.3, "shoulder_lift": -0.4, "elbow_flex": 0.6, "wrist_flex": 0.5, "wrist_roll": 0.1}
    urdf = {j: measured[j] + OFFSETS[j] for j in measured}
    got, want = kin_off.forward(measured), kin.forward(urdf)
    assert (got.x, got.y, got.z, got.pitch) == pytest.approx((want.x, want.y, want.z, want.pitch))
    assert abs(got.pitch - kin.forward(measured).pitch) > 0.05


def test_link_frame_applies_offsets(kin: ArmKinematics, kin_off: ArmKinematics) -> None:
    measured = {"shoulder_pan": 0.3, "shoulder_lift": -0.4, "elbow_flex": 0.6, "wrist_flex": 0.5, "wrist_roll": 0.1}
    urdf = {j: measured[j] + OFFSETS[j] for j in measured}
    np.testing.assert_allclose(kin_off.link_frame(measured, "gripper_link"), kin.link_frame(urdf, "gripper_link"))


def test_ik_round_trip_with_offsets_returns_measured_space(kin_off: ArmKinematics) -> None:
    rng = np.random.default_rng(3)
    for _ in range(8):
        q = random_joints(rng, kin_off)
        target = kin_off.forward(q)
        seed = {j: 0.0 for j in ARM_CHAIN_JOINTS} | {"wrist_roll": q["wrist_roll"]}
        sol = kin_off.inverse(target.x, target.y, target.z, target.pitch, seed=seed)
        got = kin_off.forward(sol)
        assert math.dist((got.x, got.y, got.z), (target.x, target.y, target.z)) < 0.003
        assert sol["wrist_roll"] == pytest.approx(q["wrist_roll"])


def test_ik_limits_are_checked_in_urdf_space(kin_off: ArmKinematics) -> None:
    rng = np.random.default_rng(5)
    for _ in range(5):
        q = random_joints(rng, kin_off)
        t = kin_off.forward(q)
        sol = kin_off.inverse(t.x, t.y, t.z, t.pitch, seed={j: 0.0 for j in ARM_CHAIN_JOINTS})
        for j, v in sol.items():
            lo, hi = kin_off.limits[j]
            assert lo + MARGIN - 1e-9 <= v + OFFSETS[j] <= hi - MARGIN + 1e-9


def test_to_urdf_and_to_measured_are_inverse_and_skip_the_gripper(kin_off: ArmKinematics) -> None:
    measured = {"shoulder_lift": 0.1, "wrist_flex": -0.3, "gripper": 0.7}
    urdf = kin_off.to_urdf(measured)
    assert urdf == pytest.approx({"shoulder_lift": -0.02, "wrist_flex": -0.1, "gripper": 0.7})
    assert kin_off.to_measured(urdf) == pytest.approx(measured)


def test_unknown_offset_joint_is_rejected() -> None:
    with pytest.raises(ValueError, match="gripper"):
        ArmKinematics(URDF, margin=MARGIN, joint_offsets={"gripper": 0.1})


# --- tool centre point: offset of the jaw closing point in the gripper_frame_link frame ---

TOOL_OFFSET = (0.0104, -0.0282, -0.0017)


@pytest.fixture(scope="module")
def kin_tcp() -> ArmKinematics:
    return ArmKinematics(URDF, margin=MARGIN, joint_offsets=OFFSETS, tool_offset=TOOL_OFFSET)


def test_forward_reports_the_offset_tool_point(kin_off: ArmKinematics, kin_tcp: ArmKinematics) -> None:
    measured = {"shoulder_pan": 0.3, "shoulder_lift": -0.4, "elbow_flex": 0.6, "wrist_flex": 0.5, "wrist_roll": 0.1}
    flange = kin_off.link_frame(measured, "gripper_frame_link")
    want = flange @ np.array([*TOOL_OFFSET, 1.0])
    got, plain = kin_tcp.forward(measured), kin_off.forward(measured)
    assert (got.x, got.y, got.z) == pytest.approx(tuple(want[:3]))
    assert got.pitch == pytest.approx(plain.pitch)
    assert math.dist((got.x, got.y, got.z), (plain.x, plain.y, plain.z)) == pytest.approx(
        math.hypot(0.0104, 0.0282, 0.0017)
    )


def test_zero_tool_offset_is_the_old_behaviour(kin_off: ArmKinematics) -> None:
    zero = ArmKinematics(URDF, margin=MARGIN, joint_offsets=OFFSETS, tool_offset=(0.0, 0.0, 0.0))
    measured = {"shoulder_pan": 0.3, "shoulder_lift": -0.4, "elbow_flex": 0.6, "wrist_flex": 0.5, "wrist_roll": 0.1}
    assert zero.forward(measured) == kin_off.forward(measured)
    seed = {j: 0.0 for j in ARM_CHAIN_JOINTS}
    assert zero.inverse(0.2, 0.05, 0.05, 0.8, seed=seed) == kin_off.inverse(0.2, 0.05, 0.05, 0.8, seed=seed)


@pytest.mark.parametrize("pan", [-0.6, 0.0, 0.5])
@pytest.mark.parametrize("seed_no", [11, 12, 13])
def test_ik_lands_the_offset_tool_point_within_1mm(kin_tcp: ArmKinematics, pan: float, seed_no: int) -> None:
    q = random_joints(np.random.default_rng(seed_no), kin_tcp) | {"shoulder_pan": pan}
    base = kin_tcp.forward(q)
    target = (base.x, base.y, base.z)
    seed = {j: 0.0 for j in ARM_CHAIN_JOINTS} | {"wrist_roll": q["wrist_roll"]}
    sol = kin_tcp.inverse(*target, base.pitch, seed=seed)
    got = kin_tcp.forward(sol)
    assert math.dist((got.x, got.y, got.z), target) < 0.001
    assert abs(got.pitch - base.pitch) <= math.radians(3.0)


def test_ik_offset_tool_point_for_free_targets(kin_tcp: ArmKinematics) -> None:
    seed = {j: 0.0 for j in ARM_CHAIN_JOINTS}
    for x, y, z, pitch in [
        (0.22, 0.0, 0.03, 1.2),
        (0.2, 0.05, -0.05, 1.2),
        (0.2, -0.1, 0.02, 1.4),
        (0.25, 0.0, -0.05, 1.2),
    ]:
        sol = kin_tcp.inverse(x, y, z, pitch, seed=seed)
        got = kin_tcp.forward(sol)
        assert math.dist((got.x, got.y, got.z), (x, y, z)) < 0.001
