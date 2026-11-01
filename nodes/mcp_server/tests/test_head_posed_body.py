"""Arm motions and grasp plans check the head where it is: posed from the fresh head joint state (head_source), the
full head envelope when the head state is missing or stale (fail closed)."""

from pathlib import Path

import pytest
from arm_collision.head import HeadPose
from arm_collision.ik import ArmKinematics, load_joint_limits
from arm_collision.self_collision import joint_line

from mcp_server.arm import ArmController, SelfCollisionError
from mcp_server.config import McpServerConfig
from mcp_server.floor_guard import FloorGuard, JawModel, SurfaceModel
from mcp_server.grasp import GraspPlanner, ObjectSpec, grasp_params

from .fakes import FakeArmBackend

CONFIG = McpServerConfig.model_validate({"arm": {"joint_limit_overrides_rad": {"shoulder_lift": [-1.745, 1.9]}}})
KIN = ArmKinematics(
    CONFIG.arm.urdf_path, margin=CONFIG.limits.arm_limit_margin_rad, limit_overrides={"shoulder_lift": (-1.745, 1.9)}
)
MOUNT = CONFIG.arm.base_in_base_link
JAW = JawModel(KIN, CONFIG.arm.jaw_open_axis, CONFIG.arm.gripper_closed_rad)
GUARD = FloorGuard(KIN, CONFIG.floor_guard, MOUNT, JAW)
FLAT = SurfaceModel(surface_z_m=0.0, tilt=None, tilt_source="none", mount=MOUNT)
SEED = {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow_flex": 1.2, "wrist_flex": 0.3, "wrist_roll": 0.0}
# The head at its home (config.head home_pan_rad / home_tilt_rad): straight ahead, 30 deg down.
HEAD_HOME = HeadPose(pan=CONFIG.head.home_pan_rad, tilt=CONFIG.head.home_tilt_rad)
# Floor object front-left of the arm (test_grasp.py LEG): the top-down grasp lifts the upper arm into the full head
# envelope, not into the head at home.
FLOOR_OBJECT = ObjectSpec(
    frame="arm", x=0.234, y=0.124, support_z=CONFIG.arm.floor_z_m, width_m=0.025, depth_m=0.03, height_m=0.02
)
# Arm swung far left and raised beside the head (default arm calibration): inside the full head envelope, clear of the
# head at home.
BESIDE_THE_HEAD = {
    "shoulder_pan": 1.2,
    "shoulder_lift": -1.5,
    "elbow_flex": -0.6,
    "wrist_flex": 0.8,
    "wrist_roll": 0.0,
    "gripper": 0.0,
}
# The same heading with the arm lowered: the motion raises it into the full envelope.
BESIDE_START = BESIDE_THE_HEAD | {"shoulder_lift": -0.6, "elbow_flex": 0.0}


def controller(tmp_path: Path, head: HeadPose | None) -> ArmController:
    """Arm controller whose head source reports a fixed head pose (None = no fresh head state).

    Args:
        tmp_path (Path): Test dir (home file).
        head (HeadPose | None): Head pose the head source returns.

    Returns:
        ArmController: The controller.
    """
    cfg = CONFIG.model_copy(deep=True)
    cfg.arm.home_file = tmp_path / "home.yaml"
    return ArmController(FakeArmBackend(), KIN, load_joint_limits(cfg.arm.urdf_path), cfg, head_source=lambda: head)


def test_without_a_head_state_a_pose_beside_the_head_is_refused(tmp_path: Path) -> None:
    """No fresh head state: the full envelope holds every pan/tilt, so the motion beside the head is refused."""
    arm = controller(tmp_path, None)
    assert arm.body_now() is arm.body_check
    with pytest.raises(SelfCollisionError, match="head"):
        arm.check_body(joint_line(BESIDE_START, BESIDE_THE_HEAD))


def test_with_the_head_at_home_the_same_pose_clears_the_head(tmp_path: Path) -> None:
    """The head measured at home: the posed head parts replace the envelope and the same motion is allowed."""
    arm = controller(tmp_path, HEAD_HOME)
    assert "d435_link" in [o.name for o in arm.body_now().body.obstacles]
    report = arm.check_body(joint_line(BESIDE_START, BESIDE_THE_HEAD))
    assert not report.rejected


@pytest.mark.parametrize(("head", "feasible"), [(None, False), (HEAD_HOME, True)], ids=["no_head_state", "head_home"])
def test_floor_grasp_plan_checks_the_head_where_it_is(head: HeadPose | None, feasible: bool) -> None:
    """The top-down floor grasp in front of the robot: refused against the full envelope, planned with the head at home."""
    planner = GraspPlanner(KIN, CONFIG, GUARD, JAW, head_source=lambda: head)
    plan = planner.plan(FLOOR_OBJECT, "top_down", grasp_params(CONFIG.grasp, None), FLAT, SEED)
    assert plan.feasible == feasible, plan.reasons
    if not feasible:
        assert any("body_collision:head" in reason for reason in plan.reasons)
