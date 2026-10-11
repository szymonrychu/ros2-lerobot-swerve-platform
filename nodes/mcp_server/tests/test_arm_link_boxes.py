"""The body check's arm link model covers the arm collision boxes of the grasp_sim MuJoCo model (so101.xml).

The sim replays every plan against the robot body with these boxes; a planner link model thinner than them lets plans
through that the sim (and the real arm) run into the body. The boxes are read from the MJCF here, independently of
mcp_server.arm_geometry, and sampled on their surfaces.
"""

import itertools
import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from mcp_server.arm_geometry import BOX_POINT_SPACING_M, LINK_COLLISION_BOXES, box_surface_points
from mcp_server.config import REPO_ROOT, McpServerConfig
from mcp_server.floor_guard import JawModel
from mcp_server.ik import ArmKinematics
from mcp_server.self_collision import SelfCollisionChecker, joint_line

CONFIG_HARD_MARGIN_M = 0.01
MJCF = REPO_ROOT / "sim/grasp_sim/assets/so101/so101.xml"
# MJCF body -> URDF link (the frames coincide: same origins and axes for the same raw joint angles).
BODY_LINKS = {"upper_arm": "upper_arm_link", "lower_arm": "lower_arm_link", "wrist": "wrist_link"}
# The upper arm boxes are contact-only (arm_geometry.LINK_COLLISION_BOXES says why): the hard check credits them with
# the hard margin, so it rejects only their contact, and they never slow a motion.
CONTACT_ONLY = {"upper_arm_link": True, "lower_arm_link": False, "wrist_link": False}
ALLOWANCE_M = {link: CONFIG_HARD_MARGIN_M if only else 0.0 for link, only in CONTACT_ONLY.items()}
CONFIG = McpServerConfig()
KIN = ArmKinematics(CONFIG.arm.urdf_path, margin=CONFIG.limits.arm_limit_margin_rad)
CHECKER = SelfCollisionChecker.from_config(
    KIN, CONFIG, JawModel(KIN, CONFIG.arm.jaw_open_axis, CONFIG.arm.gripper_closed_rad)
)
SURFACE_STEPS = 6  # samples per box edge (corners included)
POSES = 300
# Raw joints with the shoulder panned 0.5 rad toward the dome and the upper arm raised and tipped back (shoulder_lift
# 1.5): its sim upper arm box edge comes 9 mm from the measured dome (radius 37 mm), inside the hard margin.
UPPER_ARM_IN_DOME = {
    "shoulder_pan": -0.5,
    "shoulder_lift": 1.5,
    "elbow_flex": 0.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
}
# The upper arm box 23 mm from the dome edge (inside the soft margin by its distance, but contact-only): no slowdown.
UPPER_ARM_NEAR_DOME = {
    "shoulder_pan": -0.4,
    "shoulder_lift": -0.6,
    "elbow_flex": 0.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
}
# Home pose of the 2026-10-10 calibration (raw joints here: tests/test_self_collision.py HOME).
HOME = {
    "shoulder_pan": -0.0138,
    "shoulder_lift": -1.3622,
    "elbow_flex": 1.5769,
    "wrist_flex": -0.0752,
    "wrist_roll": -1.6245,
}
SEED = 20261010
BOUND = BOX_POINT_SPACING_M / math.sqrt(2) + 1e-6


def floats(text: str) -> np.ndarray:
    return np.array([float(v) for v in text.split()])


def quat_matrix(quat: np.ndarray) -> np.ndarray:
    """Rotation of a MuJoCo (w, x, y, z) quaternion (normalised first)."""
    w, x, y, z = quat / np.linalg.norm(quat)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def mjcf_boxes() -> list[tuple[str, np.ndarray, np.ndarray, np.ndarray]]:
    """Collision boxes of the arm links in the sim model: (link, pos, quat wxyz, half sizes)."""
    root = ET.parse(MJCF).getroot()
    out = []
    for body in root.iter("body"):
        link = BODY_LINKS.get(body.get("name", ""))
        if link is None:
            continue
        for geom in body.findall("geom"):
            if geom.get("type") == "box" and geom.get("class") == "collision":
                out.append(
                    (
                        link,
                        floats(geom.get("pos", "0 0 0")),
                        floats(geom.get("quat", "1 0 0 0")),
                        floats(geom.get("size")),
                    )
                )
    return out


def box_surface(pos: np.ndarray, quat: np.ndarray, size: np.ndarray, steps: int = SURFACE_STEPS) -> np.ndarray:
    """Points on the box surface in the link frame (a grid on every face)."""
    axes = [np.linspace(-s, s, steps) for s in size]
    local = np.array(
        [p for p in itertools.product(*axes) if any(abs(abs(v) - s) < 1e-12 for v, s in zip(p, size, strict=True))]
    )
    return local @ quat_matrix(quat).T + pos


def test_the_link_boxes_are_the_sim_collision_boxes() -> None:
    boxes = mjcf_boxes()
    assert len(boxes) == 6
    assert len(LINK_COLLISION_BOXES) == len(boxes)
    assert CONFIG.body_check.hard_margin_m == CONFIG_HARD_MARGIN_M
    for (link, pos, quat, size), box in zip(boxes, LINK_COLLISION_BOXES, strict=True):
        assert box.link == link
        assert box.contact_only == CONTACT_ONLY[link]
        assert box.pos == pytest.approx(tuple(pos))
        assert np.array(box.quat) / np.linalg.norm(box.quat) == pytest.approx(quat / np.linalg.norm(quat))
        assert box.half_size == pytest.approx(tuple(size))


@pytest.mark.parametrize("index", range(6))
def test_box_points_cover_every_face_within_the_spacing(index: int) -> None:
    """Every point of the box surface (a fine grid) lies within spacing / sqrt(2) of a checked point."""
    _link, pos, quat, size = mjcf_boxes()[index]
    fine = box_surface(pos, quat, size, steps=31)
    points = box_surface_points(LINK_COLLISION_BOXES[index])
    nearest = np.linalg.norm(fine[:, None, :] - points[None, :, :], axis=2).min(axis=1)
    assert nearest.max() <= BOX_POINT_SPACING_M / math.sqrt(2) + 1e-9


def test_body_clearance_never_exceeds_the_clearance_of_the_sim_link_boxes() -> None:
    """Random poses over the joint limits: per obstacle, the checker's clearance exceeds the smallest signed distance
    of the sampled sim box surface points (plus the box allowance) by at most the point spacing bound (spacing /
    sqrt(2), the SDF being 1-Lipschitz): the sim arm boxes never come closer than the check says minus that bound and
    the allowance, so a check that passes the hard margin keeps every sim box out of contact."""
    rng = np.random.default_rng(SEED)
    limits = {j: KIN.limits[j] for j in KIN.joint_names}
    boxes = [(link, box_surface(pos, quat, size)) for link, pos, quat, size in mjcf_boxes()]
    for _ in range(POSES):
        joints = {j: float(rng.uniform(*limits[j])) for j in KIN.joint_names}
        frames = KIN.link_frames(joints)
        placed = [
            (ALLOWANCE_M[link], CHECKER.to_base_link(pts @ frames[link][:3, :3].T + frames[link][:3, 3]))
            for link, pts in boxes
        ]
        check = CHECKER.pose(joints)
        for obstacle in CHECKER.body.obstacles:
            sim = min(float(obstacle.signed_distance(points).min()) + allowance for allowance, points in placed)
            assert check.per_obstacle[obstacle.name][0] <= sim + BOUND, (obstacle.name, joints)


def test_an_upper_arm_box_in_the_lidar_dome_is_rejected() -> None:
    """A pose whose sim upper arm box edge comes within the hard margin of the dome is rejected."""
    check = CHECKER.pose(UPPER_ARM_IN_DOME)
    assert check.obstacle == "lidar" and check.part == "upper arm"
    assert check.min_clearance < CONFIG.body_check.hard_margin_m


def test_home_stays_clear_of_the_hard_margin_with_the_upper_arm_boxes() -> None:
    """Home: its upper arm box stands 7 mm from the measured dome, clear of the hard check (contact only)."""
    assert CHECKER.pose(HOME).min_clearance > CONFIG.body_check.hard_margin_m
    assert not CHECKER.path([HOME, HOME]).rejected


def test_the_upper_arm_boxes_never_slow_a_motion() -> None:
    """With the upper arm box 23 mm from the dome edge there is no slowdown from it (contact only); the hard check
    still sees it (the effective clearance is the box distance plus the hard margin)."""
    near = UPPER_ARM_NEAR_DOME
    report = CHECKER.path(joint_line(near, near | {"elbow_flex": 0.3}, 0.05))
    assert not report.rejected
    assert report.scales == [1.0] * len(report.scales)
    assert CHECKER.pose(near).part == "upper arm"
    assert CONFIG.body_check.hard_margin_m < CHECKER.pose(near).min_clearance < CONFIG.body_check.soft_margin_m
