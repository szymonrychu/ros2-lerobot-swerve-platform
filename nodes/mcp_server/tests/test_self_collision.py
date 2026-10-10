"""Tests for mcp_server.self_collision: link clearances of arm poses to the robot body and the path soft boundary.

Every test pose is FK-checked (the tool point is where the test says) before its clearance is asserted.
"""

import json
import math

import numpy as np
import pytest
import yaml

from mcp_server.config import REPO_ROOT, BodyCheckSettings, McpServerConfig
from mcp_server.floor_guard import JawModel, arm_to_base_link, base_link_to_arm
from mcp_server.ik import ArmKinematics
from mcp_server.self_collision import CHECKED_PARTS, SelfCollisionChecker, combine_scales, joint_line

CONFIG = McpServerConfig()
KIN = ArmKinematics(CONFIG.arm.urdf_path, margin=CONFIG.limits.arm_limit_margin_rad)
JAW = JawModel(KIN, CONFIG.arm.jaw_open_axis, CONFIG.arm.gripper_closed_rad)
CHECKER = SelfCollisionChecker.from_config(KIN, CONFIG, JAW)
MOUNT = CONFIG.arm.base_in_base_link
SEED = {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow_flex": 1.2, "wrist_flex": 0.3, "wrist_roll": 0.0}
# Home pose of the 2026-10-10 calibration (docs/calibration/2026-10-10/session/scripts/fullfit.py, tip 243 mm up).
HOME = {
    "shoulder_pan": -0.0138,
    "shoulder_lift": -1.3622,
    "elbow_flex": 1.5769,
    "wrist_flex": -0.0752,
    "wrist_roll": -1.6245,
}
TOUCHES = json.loads((REPO_ROOT / "docs/calibration/2026-10-10/session/touches.json").read_text())["touches"]
LIDAR = CHECKER.body.by_name("lidar")
WHEEL_FR = CHECKER.body.by_name("wheel_fr")
# Touch-downs the measured lidar dome rejects (test_measured_ground_touches_in_front_are_clear).
LIDAR_REJECTED_TOUCHES = {"t2", "t4"}


def deployed_config() -> McpServerConfig:
    doc = yaml.safe_load((REPO_ROOT / "ansible/group_vars/client.yml").read_text())
    node = next(n for n in doc["ros2_nodes"] if n["name"] == "mcp_server")
    return McpServerConfig.model_validate(yaml.safe_load(node["config"]))


def tool_base_link(kin: ArmKinematics, joints: dict[str, float]) -> np.ndarray:
    pose = kin.forward(joints)
    return arm_to_base_link((pose.x, pose.y, pose.z), MOUNT)


def solve_base_link(x: float, y: float, z: float, pitch: float) -> dict[str, float]:
    """IK for a base_link tool point, FK-checked to 1 mm."""
    target = base_link_to_arm((x, y, z), MOUNT)
    joints = KIN.inverse(*target, pitch, seed=SEED)
    assert np.linalg.norm(tool_base_link(KIN, joints) - np.array([x, y, z])) < 1e-3
    return joints


def clear_pose() -> dict[str, float]:
    """Front-right of the arm, 3.5 cm from the right side frame and further from the lidar dome (at (0.30, -0.05) the
    wrist box stands 2.5 cm from the measured dome)."""
    return solve_base_link(0.33, -0.10, 0.12, 0.8)


def lidar_pose() -> dict[str, float]:
    """Gripper pointing down into the lidar column (front-left of the arm mount), tool point 4 cm under the dome top."""
    return solve_base_link(LIDAR.center_xy[0], LIDAR.center_xy[1], LIDAR.z_max - 0.04, math.pi / 2)


def test_checked_parts_cover_links_hull_and_jaws() -> None:
    assert CHECKED_PARTS == (
        "upper arm",
        "forearm",
        "wrist link",
        "gripper body",
        "jaw_tip",
        "tool_point",
        "moving_jaw_tip",
    )


def test_clear_pose_has_full_clearance() -> None:
    check = CHECKER.pose(clear_pose())
    assert check.min_clearance > CONFIG.body_check.soft_margin_m
    assert set(check.per_obstacle) == {o.name for o in CHECKER.body.obstacles}


def test_home_pose_is_clear_of_the_hard_margin_and_in_the_lidar_slow_band() -> None:
    """The folded home is never rejected, but the wrist link (its sim collision box) stands 1.5 cm from the measured
    lidar dome (radius 74 mm, top 210 mm): inside the soft margin, so moves near home run slowed."""
    tip = tool_base_link(KIN, HOME)
    assert tip[2] > 0.15  # FK check: the folded home keeps the tool well above the body
    check = CHECKER.pose(HOME)
    assert CONFIG.body_check.hard_margin_m < check.min_clearance < CONFIG.body_check.soft_margin_m
    assert (check.obstacle, check.part) == ("lidar", "wrist link")


def test_lidar_pose_penetrates_the_lidar() -> None:
    check = CHECKER.pose(lidar_pose())
    assert check.obstacle == "lidar"
    assert check.min_clearance < 0.0
    assert check.part in CHECKED_PARTS


def test_low_over_front_wheel_hits_the_wheel() -> None:
    """Tool point pointing down at the front-right wheel's front edge, 5 cm above the floor (inside the envelope)."""
    joints = solve_base_link(WHEEL_FR.center_xy[0] + 0.04, WHEEL_FR.center_xy[1], 0.05, math.pi / 2)
    check = CHECKER.pose(joints)
    assert check.per_obstacle["wheel_fr"][0] < 0.0
    assert check.min_clearance < 0.0


@pytest.mark.parametrize("touch", [t["name"] for t in TOUCHES])
def test_measured_ground_touches_in_front_are_clear(touch: str) -> None:
    """The 2026-10-10 tip touch-downs (deployed offsets) are in front of the robot on the floor: never rejected, except
    t2 and t4 (front-left): with the measured lidar dome the forearm (its sim collision box) comes within 4 mm of the
    dome at t2 and dips 3.5 mm into it at t4. The arm reached both on the robot, so the dome model (a full-radius
    cylinder up to the top face, at the assumed laser_frame position) or the FK is that much conservative there."""
    cfg = deployed_config()
    kin = ArmKinematics(
        cfg.arm.urdf_path,
        cfg.limits.arm_limit_margin_rad,
        cfg.arm.joint_offsets_rad.model_dump(),
        (cfg.arm.tool_offset_m.x, cfg.arm.tool_offset_m.y, cfg.arm.tool_offset_m.z),
        cfg.arm.joint_limit_overrides_rad,
    )
    checker = SelfCollisionChecker.from_config(
        kin, cfg, JawModel(kin, cfg.arm.jaw_open_axis, cfg.arm.gripper_closed_rad)
    )
    entry = next(t for t in TOUCHES if t["name"] == touch)
    joints = entry["last_free"]["joints"] if "last_free" in entry else entry["hover"]["joints"]
    tip = tool_base_link(kin, joints)
    assert tip[0] > 0.2 and abs(tip[2]) < 0.03  # FK check: in front of the robot, at the floor
    check = checker.pose(joints)
    if touch in LIDAR_REJECTED_TOUCHES:
        assert check.obstacle == "lidar" and check.part in ("upper arm", "forearm")
        assert -0.01 < check.min_clearance < cfg.body_check.hard_margin_m
    else:
        assert check.min_clearance > cfg.body_check.hard_margin_m


def test_path_into_the_lidar_is_rejected() -> None:
    samples = joint_line(clear_pose(), lidar_pose(), 0.02)
    report = CHECKER.path(samples)
    assert report.rejected
    assert report.violation is not None and report.violation > 0
    reason = report.rejection()
    assert reason["obstacle"] == "lidar"
    assert reason["link"] in CHECKED_PARTS
    assert reason["clearance_m"] < CONFIG.body_check.hard_margin_m
    assert reason["hard_margin_m"] == CONFIG.body_check.hard_margin_m
    assert "hint" in reason and reason["hint"]


# Tool beside the dome's front-right edge (base_link (0.258, -0.02), pitch 1.3 rad). Straight over the dome the arm
# cannot reach down far enough, and toward the dome's centre heading the gripper body is already inside its margin.
HOVER_XY = (LIDAR.center_xy[0] + 0.10, LIDAR.center_xy[1] - 0.08)
HOVER_PITCH = 1.3


def beside_lidar(z: float) -> dict[str, float]:
    return solve_base_link(HOVER_XY[0], HOVER_XY[1], z, HOVER_PITCH)


def hover_over_lidar() -> dict[str, float]:
    """Tool beside the dome edge, lowered in 2 mm steps from just above the dome top until the closest link is
    inside the slow band."""
    z = LIDAR.z_max + 0.008
    while z > LIDAR.z_max - 0.06:
        joints = beside_lidar(z)
        clearance = CHECKER.pose(joints).per_obstacle["lidar"][0]
        if clearance < (CONFIG.body_check.soft_margin_m + CONFIG.body_check.hard_margin_m) / 2.0:
            assert clearance > CONFIG.body_check.hard_margin_m
            return joints
        z -= 0.002
    raise AssertionError("no hover pose in the slow band")


def inside_lidar_margin() -> dict[str, float]:
    """Tool pointing down beside the dome's front-right edge (base_link (0.238, 0.0)), lowered in 2 mm steps from 3 cm
    under the dome top until the gripper body is inside the hard margin of the lidar."""
    z = LIDAR.z_max - 0.03
    while z > LIDAR.z_max - 0.08:
        joints = solve_base_link(LIDAR.center_xy[0] + 0.08, LIDAR.center_xy[1] - 0.06, z, math.pi / 2)
        if CHECKER.pose(joints).per_obstacle["lidar"][0] < CONFIG.body_check.hard_margin_m:
            return joints
        z -= 0.002
    raise AssertionError("no pose inside the lidar hard margin")


def test_path_near_the_lidar_is_slowed_not_rejected() -> None:
    """Lowering the tool beside the dome from above its top: the end runs slow, nothing is rejected."""
    above = beside_lidar(LIDAR.z_max + 0.008)
    assert CHECKER.pose(above).min_clearance > CONFIG.body_check.soft_margin_m
    samples = joint_line(above, hover_over_lidar(), 0.01)
    report = CHECKER.path(samples)
    assert not report.rejected
    assert report.scales[0] == 1.0
    assert report.scales[-1] == CONFIG.body_check.slow_speed_scale
    summary = report.summary()
    assert summary is not None
    assert summary["obstacle"] == "lidar"
    assert summary["speed_scale"] == CONFIG.body_check.slow_speed_scale
    assert CONFIG.body_check.hard_margin_m < summary["min_clearance_m"] < CONFIG.body_check.soft_margin_m
    assert summary["slowed_samples"] >= 1


def test_clear_path_is_neither_slowed_nor_rejected() -> None:
    goal = solve_base_link(0.30, -0.05, 0.25, 0.6)  # 10 cm above the clear pose
    assert CHECKER.pose(goal).min_clearance > CONFIG.body_check.soft_margin_m
    report = CHECKER.path(joint_line(clear_pose(), goal, 0.02))
    assert not report.rejected
    assert report.summary() is None
    assert set(report.scales) == {1.0}


def test_path_home_is_slowed_only_near_the_lidar() -> None:
    report = CHECKER.path(joint_line(clear_pose(), HOME, 0.02))
    assert not report.rejected
    assert report.scales[0] == 1.0
    assert report.scales[-1] == CONFIG.body_check.slow_speed_scale
    summary = report.summary()
    assert summary is not None and summary["obstacle"] == "lidar"


def test_escaping_from_a_colliding_start_is_allowed() -> None:
    """A start already inside the hard margin may move out (clearance never gets worse), slowed."""
    start = inside_lidar_margin()
    goal = beside_lidar(LIDAR.z_max + 0.008)
    report = CHECKER.path(joint_line(start, goal, 0.02))
    assert not report.rejected
    assert report.scales[0] == CONFIG.body_check.slow_speed_scale
    assert report.scales[-1] == 1.0


def test_escaping_the_way_it_came_in_from_deep_inside_is_rejected() -> None:
    """Deep in the dome the straight joint line to the clear pose first drives the gripper body deeper (more than the
    escape tolerance): rejected; the hint says to raise the arm first."""
    report = CHECKER.path(joint_line(lidar_pose(), clear_pose(), 0.02))
    assert report.rejected
    assert report.rejection()["obstacle"] == "lidar"


def test_going_deeper_from_a_colliding_start_is_rejected() -> None:
    deeper = solve_base_link(LIDAR.center_xy[0], LIDAR.center_xy[1], LIDAR.z_max - 0.07, math.pi / 2)
    report = CHECKER.path(joint_line(lidar_pose(), deeper, 0.02))
    assert report.rejected


def test_disabled_check_passes_everything() -> None:
    cfg = CONFIG.model_copy(update={"body_check": BodyCheckSettings(enabled=False)})
    checker = SelfCollisionChecker.from_config(KIN, cfg, JAW)
    report = checker.path(joint_line(clear_pose(), lidar_pose(), 0.02))
    assert not report.rejected
    assert set(report.scales) == {1.0}
    assert report.summary() is None


def test_combine_scales_slower_wins() -> None:
    assert combine_scales([1.0, 0.2, 1.0, 0.2], [1.0, 1.0, 0.3, 0.3]) == [1.0, 0.2, 0.3, 0.2]


def test_joint_line_steps_and_ends() -> None:
    a = {"shoulder_pan": 0.0, "elbow_flex": 0.0}
    b = {"shoulder_pan": 0.1, "elbow_flex": -0.05}
    line = joint_line(a, b, 0.02)
    assert line[0] == a and line[-1] == b
    assert len(line) == 6
    assert all(abs(q["shoulder_pan"] - p["shoulder_pan"]) <= 0.02 + 1e-12 for p, q in zip(line, line[1:]))


def test_checked_points_follow_the_arm_into_base_link() -> None:
    """The checked part points are in base_link: the tool point part is the FK tool point converted."""
    joints = clear_pose()
    parts = CHECKER.part_points(joints)
    assert np.allclose(
        parts["tool_point"][0],
        arm_to_base_link(np.array([KIN.forward(joints).x, KIN.forward(joints).y, KIN.forward(joints).z]), MOUNT),
    )
