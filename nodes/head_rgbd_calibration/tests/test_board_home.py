"""The board Home with the edge handle board (design D63): the shipped home path margins still hold for the 211.7 g board.

The home path check adds `park_board_sag_margin_m` to the board floor, body and head clearances because the board sags
below FK under its weight: the live run measured about 4 mm with the 214 g stick-mounted board (session
calib-20261023-074418, docs/runbooks/head-arm-calibration.md), its centre about 0.12 m off the tool axis as on the edge
handle. The edge handle board is lighter, so the 10 mm margin stays conservative, and the path from every pose of the
shipped set to the stored arm home passes the node's home path check with the plate and the handle boxes, at the CAD seed
mount (margin_m + provisional_margin_m) and at a fitted mount (margin_m). The home is mcp_server's home.yaml, so the
check runs for the footprint's home (scripts/gen_nav_footprint.yaml parked_arm_joints) and for the home the servo
calibration stored on 2026-11-11.
"""

import math
from pathlib import Path

import pytest
import yaml
from ros2_common.head_rgbd_calibration.arm_pose_check import PoseChecker
from ros2_common.head_rgbd_calibration.config import ARM_JOINT_NAMES

from head_rgbd_calibration.config import load_config

NODE_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = NODE_DIR.parents[1]
SHIPPED_YAML = NODE_DIR / "config" / "head_rgbd_calibration.yaml"
URDF_DIR = REPO_ROOT / "nodes" / "web_ui" / "urdf"
MEASURED_SAG_BOARD_MASS_G = 214.0  # the board the home sag was measured with
MEASURED_SAG_M = 0.004  # its sag below FK at the home
FOOTPRINT_YAML = REPO_ROOT / "scripts" / "gen_nav_footprint.yaml"
FOOTPRINT_HOME: dict[str, float] = {
    name: float(value)
    for name, value in yaml.safe_load(FOOTPRINT_YAML.read_text())["parked_arm_joints"].items()
    if name in ARM_JOINT_NAMES
}
# The home stored by the servo calibration on 2026-11-11 (client /var/lib/ros2/arm/home.yaml, gripper dropped).
CALIBRATED_HOME: dict[str, float] = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -math.pi / 2,
    "elbow_flex": math.pi / 2,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
}

CFG = load_config(SHIPPED_YAML)
SEQ = CFG.arm_sequence


def test_board_is_no_heavier_than_the_board_the_home_sag_was_measured_with() -> None:
    assert SEQ.board_mass_g <= MEASURED_SAG_BOARD_MASS_G
    assert SEQ.park_board_sag_margin_m >= MEASURED_SAG_M


@pytest.mark.parametrize("home", [FOOTPRINT_HOME, CALIBRATED_HOME], ids=["footprint-home", "calibrated-home"])
@pytest.mark.parametrize("nominal_mount", [True, False], ids=["cad-seed", "fitted"])
def test_home_path_with_the_board_is_clear_from_every_pose(nominal_mount: bool, home: dict[str, float]) -> None:
    """As Sequencer.check_board_home_path: board margin + sag, min_board_height_m + sag, grip held."""
    sag = SEQ.park_board_sag_margin_m
    pose_check = SEQ.pose_check.model_copy(update={"min_board_height_m": SEQ.pose_check.min_board_height_m + sag})
    checker = PoseChecker(
        (URDF_DIR / "so101_arm.urdf").read_text(),
        (URDF_DIR / "head.urdf").read_text(),
        CFG.fit,
        pose_check,
        SEQ.board_margin_m(nominal_mount) + sag,
    )
    assert len(checker.board_local) > len(CFG.fit.board_mount.handle_boxes_mm)  # the plate and the handle boxes
    starts = {"base": SEQ.base_pose} | {
        f"tier {tier} pose {index}": pose
        for tier in (0, 1, 2)
        for index, pose in enumerate(getattr(SEQ, f"tier{tier}_poses"))
    }
    failures = [
        f"{name}: {entry}"
        for name, pose in starts.items()
        for entry in checker.check_path(dict(pose), dict(home), SEQ.grip_rad)
    ]
    assert failures == [], "\n".join(failures[:20])
