"""The ARM board on its edge handle faces the gripper and the head camera (user m540).

The gripper holds the handle from the pattern side, so the pattern's outward normal n (board -z in gripper_link)
points back along -TOOL_AXIS, toward the wrist. At the arm's base pose, with the head aimed at the board centre
(`aim_head` on the shipped head URDF), the pattern faces the head D435 colour camera: board z seen from the camera
points away from it by at least `MIN_FACING_COS` (tests/head_arm_synthetic.py, the synthetic scenes' rule).
"""

import importlib.util
import sys
from pathlib import Path
from types import ModuleType

import numpy as np
from ros2_common.head_rgbd_calibration.arm_pose_check import PoseChecker
from ros2_common.head_rgbd_calibration.chain import UrdfChain, pose_from_xyz_rpy
from ros2_common.head_rgbd_calibration.head_arm_fit import aim_head, board_centre, camera_pose

from head_rgbd_calibration.config import CAD_BOARD_RPY, CAD_BOARD_XYZ, load_config

NODE_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = NODE_DIR.parents[1]
SHIPPED_YAML = NODE_DIR / "config" / "head_rgbd_calibration.yaml"
SYNTHETIC_FILE = REPO_ROOT / "tests" / "head_arm_synthetic.py"
URDF_DIR = REPO_ROOT / "nodes" / "web_ui" / "urdf"
TOOL_AXIS = np.array([0.0, 0.0, -1.0])  # gripper_link: from the wrist toward the finger tips
FACING_DOT_MAX = -0.99  # n . t of the pattern normal and the tool approach axis
PAN, TILT = "head_pan", "head_tilt"
SYNTHETIC_MODULE = "head_arm_synthetic_facing"


def load_synthetic() -> ModuleType:
    """Import tests/head_arm_synthetic.py of the repo root (the node has its own tests package).

    Returns:
        ModuleType: The loaded module.
    """
    if SYNTHETIC_MODULE in sys.modules:
        return sys.modules[SYNTHETIC_MODULE]
    spec = importlib.util.spec_from_file_location(SYNTHETIC_MODULE, SYNTHETIC_FILE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[SYNTHETIC_MODULE] = module
    spec.loader.exec_module(module)
    return module


def test_cad_mount_pattern_faces_back_toward_the_wrist() -> None:
    mount = pose_from_xyz_rpy(CAD_BOARD_XYZ, CAD_BOARD_RPY)
    assert float(-mount[:3, 2] @ TOOL_AXIS) < FACING_DOT_MAX


def test_pattern_faces_the_aimed_head_camera_at_the_base_pose() -> None:
    cfg = load_config(SHIPPED_YAML)
    fit, seq = cfg.fit, cfg.arm_sequence
    head_urdf = (URDF_DIR / "head.urdf").read_text()
    checker = PoseChecker((URDF_DIR / "so101_arm.urdf").read_text(), head_urdf, fit, seq.pose_check)
    mount = pose_from_xyz_rpy(fit.board_mount.xyz_init, fit.board_mount.rpy_init)
    board = checker.link_poses(seq.base_pose)["gripper_link"] @ mount
    centre = (board @ np.append(board_centre(fit.board), 1.0))[:3]
    chain = UrdfChain.from_text(head_urdf)
    pan, tilt = aim_head(chain, centre)
    in_camera = np.linalg.solve(camera_pose(chain, {PAN: pan, TILT: tilt}), board)
    assert in_camera[2, 2] > load_synthetic().MIN_FACING_COS
