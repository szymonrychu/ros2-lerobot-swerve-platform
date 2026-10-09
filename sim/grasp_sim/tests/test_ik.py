"""Harness-internal numeric IK (position plus approach pitch) on the MuJoCo model."""

import numpy as np
import pytest

from grasp_sim.config import SceneConfig
from grasp_sim.ik import ArmIK, IKError

LOCAL_POINT = (-0.0081, 0.0, -0.07)
POSITION_TOLERANCE_M = 0.001


@pytest.fixture
def ik() -> ArmIK:
    return ArmIK(SceneConfig())


@pytest.mark.parametrize(
    ("target", "pitch_down", "roll"),
    [((0.20, 0.0, -0.10), 1.1, 0.0), ((0.18, 0.08, -0.12), 1.1, -1.57), ((0.17, -0.08, -0.10), 1.2, -1.57)],
)
def test_solution_reaches_target_and_pitch(ik: ArmIK, target, pitch_down, roll) -> None:
    q = ik.solve(np.array(target), LOCAL_POINT, pitch_down, roll)
    pos, pitch = ik.forward(q, LOCAL_POINT)
    assert np.linalg.norm(pos - np.array(target)) < POSITION_TOLERANCE_M
    assert pitch == pytest.approx(pitch_down, abs=0.01)
    assert q["wrist_roll"] == roll


def test_unreachable_target_raises(ik: ArmIK) -> None:
    with pytest.raises(IKError):
        ik.solve(np.array([0.9, 0.0, 0.3]), LOCAL_POINT, 0.0, 0.0)


def test_seed_keeps_solution_continuous(ik: ArmIK) -> None:
    first = ik.solve(np.array([0.20, 0.0, -0.10]), LOCAL_POINT, 1.1, 0.0)
    second = ik.solve(np.array([0.205, 0.0, -0.10]), LOCAL_POINT, 1.1, 0.0, seed=first)
    assert max(abs(first[j] - second[j]) for j in first) < 0.1


def test_joint_limits_are_respected(ik: ArmIK) -> None:
    q = ik.solve(np.array([0.20, 0.0, -0.10]), LOCAL_POINT, 1.1, 0.0)
    assert abs(q["elbow_flex"]) <= 1.69
    assert -1.745 <= q["shoulder_lift"] <= 1.9
