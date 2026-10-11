"""Unmeasured gripper/joint efforts (review L25): feetech_servos publishes NaN effort for a joint without an effort
reading (non-effort joints, a missing load). mcp_server must treat a non-finite or missing effort as unmeasured,
never as a load of 0 and never as a number in a comparison (NaN compares false and slips through `<` checks).

Covers: the ros_iface joint sample and ArmState omit non-finite efforts; motion and grasp results report a NaN
gripper effort as null; grasp verification with a NaN or missing load reports it unmeasured instead of claiming a
hold; the hold report gives no holding_load / crush_risk and no re-tighten from a NaN load; the gripper_holding
precondition and the carrying signal are not holding with an unmeasured load; the wait_for_event digest and the live
precondition state carry null for a NaN effort.
"""

import math
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest

from mcp_server.arm import ArmController, finite_effort
from mcp_server.config import McpServerConfig
from mcp_server.grasp import grasp_params
from mcp_server.grasp_tools import GraspExecutor
from mcp_server.grip import resolve_grip_profile
from mcp_server.ik import ArmKinematics, load_joint_limits
from mcp_server.models import ArmMotionResult
from mcp_server.motion_queue import LiveState, Precondition, evaluate_precondition, read_live_state, state_digest

from .fakes import FakeArmBackend
from .test_control import manager
from .test_tools import FakeRobot

NAN = math.nan
HOLD_JAW = 0.29  # an object between the jaws: well short of closed, outside the empty band


def make(tmp_path: Path) -> tuple[ArmController, FakeArmBackend, McpServerConfig]:
    be = FakeArmBackend()
    cfg = McpServerConfig()
    cfg.arm.home_file = tmp_path / "home.yaml"
    kin = ArmKinematics(cfg.arm.urdf_path, margin=cfg.limits.arm_limit_margin_rad)
    return ArmController(be, kin, load_joint_limits(cfg.arm.urdf_path), cfg), be, cfg


@pytest.mark.parametrize(
    ("efforts", "joint", "expected"),
    [({"gripper": 3.0}, "gripper", 3.0), ({"gripper": NAN}, "gripper", None), ({}, "gripper", None)],
)
def test_finite_effort(efforts: dict[str, float], joint: str, expected: float | None) -> None:
    assert finite_effort(efforts, joint) == expected
    assert finite_effort({"gripper": math.inf}, "gripper") is None


def test_nan_effort_for_a_non_effort_joint_is_absent_from_arm_state(tmp_path: Path) -> None:
    arm, be, _ = make(tmp_path)
    be.efforts["shoulder_pan"] = NAN
    be.efforts["gripper"] = 42.0
    st = arm.state()
    assert st.efforts is not None
    assert "shoulder_pan" not in st.efforts
    assert st.efforts["gripper"] == 42.0
    be.efforts["gripper"] = NAN
    st = arm.state()
    assert st.efforts is not None and "gripper" not in st.efforts
    assert st.gripper_effort is None


def test_motion_result_reports_a_nan_gripper_effort_as_null(tmp_path: Path) -> None:
    arm, be, _ = make(tmp_path)
    be.efforts["gripper"] = NAN
    res = arm.set_gripper(open_fraction=0.5)
    assert res.gripper_effort is None


def grasped_close(effort: float | None) -> ArmMotionResult:
    return ArmMotionResult(status="grasped", message="gripper effort reached 100", gripper_effort=effort)


@pytest.mark.parametrize(("close_effort", "sample_effort"), [(NAN, NAN), (None, None), (None, NAN)])
def test_grasp_verification_with_unmeasured_load_does_not_claim_holding(
    tmp_path: Path, close_effort: float | None, sample_effort: float | None
) -> None:
    arm, be, cfg = make(tmp_path)
    be.positions["gripper"] = HOLD_JAW
    if sample_effort is None:
        del be.efforts["gripper"]
    else:
        be.efforts["gripper"] = sample_effort
    executor = GraspExecutor(arm, cfg)
    reason = executor.verify(grasped_close(close_effort), grasp_params(cfg.grasp, None, None))
    assert reason is not None and "unmeasured" in reason


def test_grasp_verification_uses_the_measured_load_when_one_is_finite(tmp_path: Path) -> None:
    arm, be, cfg = make(tmp_path)
    be.positions["gripper"] = HOLD_JAW
    be.efforts["gripper"] = NAN
    params = grasp_params(cfg.grasp, None, None)
    assert GraspExecutor(arm, cfg).verify(grasped_close(params.hold_effort_min + 50.0), params) is None


def test_grasp_result_reports_a_nan_gripper_effort_as_null(tmp_path: Path) -> None:
    arm, be, cfg = make(tmp_path)
    be.efforts["gripper"] = NAN
    result = GraspExecutor(arm, cfg).finish("aborted", ["test"], None, [])
    assert result.gripper_effort is None


def test_hold_report_and_retighten_with_a_nan_load(tmp_path: Path) -> None:
    arm, be, cfg = make(tmp_path)
    be.positions["gripper"] = HOLD_JAW
    be.efforts["gripper"] = NAN
    grip = resolve_grip_profile(cfg.grip_profiles, cfg.limits, "soft")
    assert grip.profile.retighten
    report = arm.hold_report(grip)
    assert report["holding_load"] is None and report["crush_risk"] is None
    sent = len(be.commands)
    assert arm.retighten_hold(HOLD_JAW, {"holding_load": NAN}, grip) is None
    assert len(be.commands) == sent


@pytest.mark.parametrize("effort", [NAN, None])
def test_gripper_holding_precondition_with_unmeasured_load(effort: float | None) -> None:
    cfg = McpServerConfig()
    state = LiveState(joints={"gripper": HOLD_JAW}, gripper_effort=effort, battery_ok=True)
    holding, reason = evaluate_precondition(Precondition(type="gripper_holding"), state, cfg)
    assert holding is False
    assert "unmeasured" in reason


def test_live_state_and_digest_carry_null_for_a_nan_effort(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    robot.arm_backend.efforts["gripper"] = NAN
    assert read_live_state(robot, None, need_twist=False).gripper_effort is None
    digest = state_digest(robot, None)
    assert digest["gripper"] is not None and digest["gripper"]["effort"] is None


def test_carrying_signal_is_not_a_holding_load_when_unmeasured(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    robot.arm_backend.positions["gripper"] = HOLD_JAW
    robot.arm_backend.efforts["gripper"] = NAN
    assert manager(robot).carrying_state().carrying is False


def test_ros_joint_sample_omits_non_finite_efforts(ros_iface: ModuleType) -> None:
    robot = ros_iface.RosRobot.__new__(ros_iface.RosRobot)
    robot.lock = ros_iface.threading.Lock()
    msg = SimpleNamespace(name=["shoulder_pan", "gripper"], position=[0.1, 0.3], effort=[NAN, 120.0])
    robot.latest = {"joint_states": ros_iface.Stamped(value=msg, stamp=1.0)}
    sample = robot.joint_sample()
    assert sample.positions == {"shoulder_pan": 0.1, "gripper": 0.3}
    assert sample.efforts == {"gripper": 120.0}
