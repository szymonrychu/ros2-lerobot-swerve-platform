"""View types of the head-from-arm fit: one head camera frame of the board on the gripper with the measured joints."""

from dataclasses import dataclass

from ros2_common.head_rgbd_calibration.charuco import Detection
from ros2_common.head_rgbd_calibration.config import ARM_JOINT_NAMES

HEAD_JOINTS = ("head_pan", "head_tilt")
ARM_JOINTS = ARM_JOINT_NAMES


@dataclass(frozen=True)
class ArmBoardView:
    """One still view of the gripper board: `joints` are the measured rad of all seven joints, never defaults."""

    view_id: str
    arm_pose: int
    tier: int
    joints: dict[str, float]
    detection: Detection


def require_joints(view: ArmBoardView) -> None:
    """Check that the view carries all seven joints.

    Args:
        view: The view to check.

    Raises:
        ValueError: Naming the view id and every missing joint.
    """
    missing = [name for name in (*HEAD_JOINTS, *ARM_JOINTS) if name not in view.joints]
    if missing:
        raise ValueError(f"view {view.view_id} is missing joints: {', '.join(missing)}")
