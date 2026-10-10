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
from mcp_server.self_collision import SelfCollisionChecker

MJCF = REPO_ROOT / "sim/grasp_sim/assets/so101/so101.xml"
# MJCF body -> URDF link (the frames coincide: same origins and axes for the same raw joint angles). The upper arm is
# not covered (arm_geometry.LINK_COLLISION_BOXES says why); test_the_upper_arm_boxes_are_left_out pins that choice.
BODY_LINKS = {"lower_arm": "lower_arm_link", "wrist": "wrist_link"}
CONFIG = McpServerConfig()
KIN = ArmKinematics(CONFIG.arm.urdf_path, margin=CONFIG.limits.arm_limit_margin_rad)
CHECKER = SelfCollisionChecker.from_config(
    KIN, CONFIG, JawModel(KIN, CONFIG.arm.jaw_open_axis, CONFIG.arm.gripper_closed_rad)
)
SURFACE_STEPS = 6  # samples per box edge (corners included)
POSES = 300
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
    assert len(boxes) == 4
    assert len(LINK_COLLISION_BOXES) == len(boxes)
    for (link, pos, quat, size), box in zip(boxes, LINK_COLLISION_BOXES, strict=True):
        assert box.link == link
        assert box.pos == pytest.approx(tuple(pos))
        assert np.array(box.quat) / np.linalg.norm(box.quat) == pytest.approx(quat / np.linalg.norm(quat))
        assert box.half_size == pytest.approx(tuple(size))


@pytest.mark.parametrize("index", range(4))
def test_box_points_cover_every_face_within_the_spacing(index: int) -> None:
    """Every point of the box surface (a fine grid) lies within spacing / sqrt(2) of a checked point."""
    _link, pos, quat, size = mjcf_boxes()[index]
    fine = box_surface(pos, quat, size, steps=31)
    points = box_surface_points(LINK_COLLISION_BOXES[index])
    nearest = np.linalg.norm(fine[:, None, :] - points[None, :, :], axis=2).min(axis=1)
    assert nearest.max() <= BOX_POINT_SPACING_M / math.sqrt(2) + 1e-9


def test_body_clearance_never_exceeds_the_clearance_of_the_sim_link_boxes() -> None:
    """Random poses over the joint limits: per obstacle, the checker's clearance exceeds the smallest signed distance
    of the sampled sim box surface points by at most the point spacing bound (spacing / sqrt(2), the SDF being
    1-Lipschitz): the sim arm boxes never come closer than the check says minus that bound."""
    rng = np.random.default_rng(SEED)
    limits = {j: KIN.limits[j] for j in KIN.joint_names}
    boxes = [(link, box_surface(pos, quat, size)) for link, pos, quat, size in mjcf_boxes()]
    for _ in range(POSES):
        joints = {j: float(rng.uniform(*limits[j])) for j in KIN.joint_names}
        frames = KIN.link_frames(joints)
        points = np.vstack(
            [CHECKER.to_base_link(pts @ frames[link][:3, :3].T + frames[link][:3, 3]) for link, pts in boxes]
        )
        check = CHECKER.pose(joints)
        for obstacle in CHECKER.body.obstacles:
            sim = float(obstacle.signed_distance(points).min())
            assert check.per_obstacle[obstacle.name][0] <= sim + BOUND, (obstacle.name, joints)
