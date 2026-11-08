"""SO-101 follower gripper meshes posed from the repo URDF: jaw gap, contact angle, penetration and clearance.

All lengths are mm in the gripper_link frame. The fixed jaw is the gripper_link visual set (the wrist roll follower
part carries the fixed finger, plus the gripper servo). The moving jaw turns about the `gripper` joint z axis.
OPEN_AXIS points from the fixed jaw toward the moving jaw, TOOL_AXIS points out of the jaws toward the tips and
WIDTH_AXIS runs across the finger width.
"""

from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree

import numpy as np
import trimesh
from ros2_common.head_rgbd_calibration.chain import pose_from_xyz_rpy
from scipy.spatial.transform import Rotation

GRIPPER_LINK = "gripper_link"
MOVING_JAW_LINK = "moving_jaw_so101_v1_link"
GRIPPER_JOINT = "gripper"
TOOL_JOINT = "gripper_frame_joint"
MM_PER_M = 1000.0
OPEN_AXIS = (1.0, 0.0, 0.0)  # mcp jaw_open_axis (-1, 0, 0) in gripper_frame_link (rpy 0 pi 0) mapped to gripper_link
TOOL_AXIS = (0.0, 0.0, -1.0)
WIDTH_AXIS = (0.0, 1.0, 0.0)
CONTACT_TOL_MM = 0.05
CONTACT_HI_RAD = 1.0
BISECTION_STEPS = 60
TIP_BAND_MM = 1.0  # depth band behind a tip that counts as the tip (finger centre, minimum fixed face band)
RAY_REACH_MM = 1000.0  # ray start distance along -OPEN_AXIS, well outside the gripper
CLEARANCE_SAMPLES = 20000
PENETRATION_SAMPLES = 5000
SURFACE_SEED = 0
FACE_RAYS = 11  # rays spread over TIP_BAND_MM behind the sample depth, so depth 0 still meets the rounded tip
ZERO_VEC = "0 0 0"
UNIT_SCALE = "1 1 1"


@dataclass(frozen=True)
class Gripper:
    """The posed gripper meshes in mm.

    Attributes:
        fixed (trimesh.Trimesh): gripper_link visuals (fixed jaw and servo) in gripper_link.
        moving (trimesh.Trimesh): Moving jaw visual in its own link frame.
        joint (np.ndarray): 4x4 pose of the gripper joint frame (angle 0) in gripper_link.
        tool (np.ndarray): 4x4 pose of gripper_frame_link in gripper_link.
        axis (np.ndarray): Gripper joint axis in the joint frame.
        lower_rad (float): Gripper lower limit (rad).
    """

    fixed: trimesh.Trimesh
    moving: trimesh.Trimesh
    joint: np.ndarray
    tool: np.ndarray
    axis: np.ndarray
    lower_rad: float


def origin_mm(element: ElementTree.Element | None) -> np.ndarray:
    """4x4 pose of a URDF `<origin>` with the translation in mm.

    Args:
        element (ElementTree.Element | None): The origin element; identity when None.

    Returns:
        np.ndarray: 4x4 homogeneous transform (mm).
    """
    if element is None:
        return np.eye(4)
    xyz = [float(v) * MM_PER_M for v in element.get("xyz", ZERO_VEC).split()]
    rpy = [float(v) for v in element.get("rpy", ZERO_VEC).split()]
    return pose_from_xyz_rpy((xyz[0], xyz[1], xyz[2]), (rpy[0], rpy[1], rpy[2]))


def link_visuals_mm(root: ElementTree.Element, link: str, base_dir: Path) -> trimesh.Trimesh:
    """All mesh visuals of a URDF link, scaled to mm and posed in the link frame.

    Args:
        root (ElementTree.Element): URDF root element.
        link (str): Link name.
        base_dir (Path): Directory the mesh filenames are relative to (the URDF directory).

    Returns:
        trimesh.Trimesh: The concatenated visual meshes.
    """
    parts: list[trimesh.Trimesh] = []
    for visual in root.findall(f"./link[@name='{link}']/visual"):
        mesh_el = visual.find("geometry/mesh")
        if mesh_el is None:
            raise ValueError(f"{link} visual has no mesh geometry")
        mesh = trimesh.load(base_dir / mesh_el.get("filename", ""), force="mesh")
        scale = np.array([float(v) for v in mesh_el.get("scale", UNIT_SCALE).split()]) * MM_PER_M
        mesh.apply_transform(np.diag([*scale, 1.0]))
        mesh.apply_transform(origin_mm(visual.find("origin")))
        parts.append(mesh)
    if not parts:
        raise ValueError(f"{link} has no visuals")
    return trimesh.util.concatenate(parts)


def find_joint(root: ElementTree.Element, name: str) -> ElementTree.Element:
    """The URDF `<joint>` element with that name.

    Args:
        root (ElementTree.Element): URDF root element.
        name (str): Joint name.

    Returns:
        ElementTree.Element: The joint element.
    """
    joint = root.find(f"./joint[@name='{name}']")
    if joint is None:
        raise ValueError(f"joint {name} not in the URDF")
    return joint


def load_gripper(urdf_path: Path) -> Gripper:
    """Load the gripper meshes and frames from the arm URDF.

    Args:
        urdf_path (Path): The SO-101 arm URDF; mesh filenames resolve relative to its directory.

    Returns:
        Gripper: The fixed and moving meshes, the gripper joint and the tool frame in gripper_link (mm).
    """
    root = ElementTree.parse(urdf_path).getroot()
    joint = find_joint(root, GRIPPER_JOINT)
    tool = find_joint(root, TOOL_JOINT)
    for element, parent in ((joint, GRIPPER_LINK), (tool, GRIPPER_LINK)):
        if element.find("parent").get("link") != parent:  # type: ignore[union-attr]
            raise ValueError(f"joint {element.get('name')} is not a child of {parent}")
    if joint.find("child").get("link") != MOVING_JAW_LINK:  # type: ignore[union-attr]
        raise ValueError(f"joint {GRIPPER_JOINT} does not move {MOVING_JAW_LINK}")
    axis = np.array([float(v) for v in joint.find("axis").get("xyz", ZERO_VEC).split()])  # type: ignore[union-attr]
    return Gripper(
        fixed=link_visuals_mm(root, GRIPPER_LINK, urdf_path.parent),
        moving=link_visuals_mm(root, MOVING_JAW_LINK, urdf_path.parent),
        joint=origin_mm(joint.find("origin")),
        tool=origin_mm(tool.find("origin")),
        axis=axis / np.linalg.norm(axis),
        lower_rad=float(joint.find("limit").get("lower", "0")),  # type: ignore[union-attr]
    )


def moving_jaw(gripper: Gripper, angle_rad: float) -> trimesh.Trimesh:
    """The moving jaw mesh in gripper_link at a gripper angle.

    Args:
        gripper (Gripper): Loaded gripper.
        angle_rad (float): Gripper joint angle (rad).

    Returns:
        trimesh.Trimesh: A posed copy of the moving jaw (mm).
    """
    turn = np.eye(4)
    turn[:3, :3] = Rotation.from_rotvec(gripper.axis * angle_rad).as_matrix()
    return gripper.moving.copy().apply_transform(gripper.joint @ turn)


def band(mesh: trimesh.Trimesh, depth_mm: tuple[float, float]) -> trimesh.Trimesh:
    """The part of a mesh within a depth band behind its extreme along TOOL_AXIS.

    Args:
        mesh (trimesh.Trimesh): Mesh in gripper_link (mm).
        depth_mm (tuple[float, float]): Near and far depth behind the tip (mm, near < far).

    Returns:
        trimesh.Trimesh: The clipped mesh (cross-section faces are left open).
    """
    tool = np.asarray(TOOL_AXIS)
    tip = float((mesh.vertices @ tool).max())
    near = trimesh.intersections.slice_mesh_plane(mesh, -tool, tool * (tip - depth_mm[0]))
    return trimesh.intersections.slice_mesh_plane(near, tool, tool * (tip - depth_mm[1]))


def fixed_jaw_face_mm(gripper: Gripper, depth_mm: tuple[float, float]) -> float:
    """OPEN_AXIS coordinate of the fixed jaw inner face over a depth band behind the fixed jaw tip.

    The inner face is the fixed jaw material that reaches furthest toward the moving jaw in that band.

    Args:
        gripper (Gripper): Loaded gripper.
        depth_mm (tuple[float, float]): Near and far depth behind the fixed jaw tip along TOOL_AXIS (mm).

    Returns:
        float: The face coordinate along OPEN_AXIS (mm).
    """
    clipped = band(gripper.fixed, depth_mm)
    if len(clipped.vertices) == 0:
        raise ValueError(f"no fixed jaw material in the depth band {depth_mm}")
    return float((clipped.vertices @ np.asarray(OPEN_AXIS)).max())


def moving_tip_mm(gripper: Gripper, angle_rad: float) -> np.ndarray:
    """The moving finger tip: its extreme along TOOL_AXIS, centred on WIDTH_AXIS.

    Args:
        gripper (Gripper): Loaded gripper.
        angle_rad (float): Gripper joint angle (rad).

    Returns:
        np.ndarray: (3,) point in gripper_link (mm); the TOOL_AXIS coordinate is the extreme, the WIDTH_AXIS
        coordinate the centre of the finger within TIP_BAND_MM of it, the OPEN_AXIS coordinate that of the extreme
        vertex.
    """
    vertices = moving_jaw(gripper, angle_rad).vertices
    tool, width, opening = (np.asarray(a) for a in (TOOL_AXIS, WIDTH_AXIS, OPEN_AXIS))
    depths = vertices @ tool
    extreme = vertices[int(np.argmax(depths))]
    near = vertices[depths >= depths.max() - TIP_BAND_MM] @ width
    centre = (near.min() + near.max()) / 2.0
    return opening * float(extreme @ opening) + width * float(centre) + tool * float(depths.max())


def jaw_gap_mm(gripper: Gripper, angle_rad: float, depth_mm: float) -> float:
    """Inner gap along OPEN_AXIS in the moving finger mid-plane, at a depth behind the moving jaw tip.

    The moving face is the first moving jaw surface that rays along OPEN_AXIS meet on the mid-plane, over the lines
    from that depth to TIP_BAND_MM deeper (FACE_RAYS lines, so depth 0 still meets the rounded tip).
    The fixed face is `fixed_jaw_face_mm` from the fixed jaw tip down to the same line (at least TIP_BAND_MM),
    the fixed jaw material an object reaching that deep bears on.

    Args:
        gripper (Gripper): Loaded gripper.
        angle_rad (float): Gripper joint angle (rad).
        depth_mm (float): Depth behind the moving jaw tip along TOOL_AXIS (mm).

    Returns:
        float: Moving face minus fixed face along OPEN_AXIS (mm); negative when the jaws overlap.
    """
    tool, opening = np.asarray(TOOL_AXIS), np.asarray(OPEN_AXIS)
    tip = moving_tip_mm(gripper, angle_rad)
    line = tip - tool * depth_mm
    lines = line - np.outer(np.linspace(0.0, TIP_BAND_MM, FACE_RAYS), tool)
    starts = lines - opening * (float(line @ opening) + RAY_REACH_MM)
    directions = np.tile(opening, (FACE_RAYS, 1))
    hits, _, _ = moving_jaw(gripper, angle_rad).ray.intersects_location(starts, directions)
    if len(hits) == 0:
        raise ValueError(f"no moving jaw material {depth_mm} mm behind the tip at {angle_rad} rad")
    moving_face = float((hits @ opening).min())
    fixed_tip = float((gripper.fixed.vertices @ tool).max())
    fixed_depth = max(fixed_tip - float(line @ tool), TIP_BAND_MM)
    return moving_face - fixed_jaw_face_mm(gripper, (0.0, fixed_depth))


def surface_points(mesh: trimesh.Trimesh, count: int) -> np.ndarray:
    """The mesh vertices plus `count` seeded samples of its surface.

    Args:
        mesh (trimesh.Trimesh): Mesh (mm).
        count (int): Number of surface samples.

    Returns:
        np.ndarray: (N, 3) points (mm).
    """
    samples, _ = trimesh.sample.sample_surface(mesh, count, seed=SURFACE_SEED)
    return np.vstack([mesh.vertices, samples])


def penetration_mm(solid: trimesh.Trimesh, other: trimesh.Trimesh) -> float:
    """Largest depth of `other` vertices and sampled surface points inside `solid`.

    Surface samples (PENETRATION_SAMPLES) catch faces that cross `solid` while their vertices sit on its surface.

    Args:
        solid (trimesh.Trimesh): Watertight mesh (mm).
        other (trimesh.Trimesh): Mesh whose vertices are tested (mm).

    Returns:
        float: Deepest vertex depth inside `solid` (mm); 0.0 when none is inside.
    """
    lo, hi = solid.bounds
    points = surface_points(other, PENETRATION_SAMPLES)
    points = points[np.all((points >= lo) & (points <= hi), axis=1)]
    if len(points) == 0:
        return 0.0
    return max(float(trimesh.proximity.signed_distance(solid, points).max()), 0.0)


def clearance_mm(a: trimesh.Trimesh, b: trimesh.Trimesh) -> float:
    """Smallest distance between two disjoint meshes, from sampled surface points both ways.

    Args:
        a (trimesh.Trimesh): First mesh (mm).
        b (trimesh.Trimesh): Second mesh (mm).

    Returns:
        float: The smallest closest-point distance (mm).
    """
    best = np.inf
    for source, target in ((a, b), (b, a)):
        _, distances, _ = trimesh.proximity.closest_point(target, surface_points(source, CLEARANCE_SAMPLES))
        best = min(best, float(distances.min()))
    return best


def contact_angle_rad(gripper: Gripper, solid: trimesh.Trimesh) -> float:
    """Smallest gripper angle where the moving jaw penetrates `solid` by at most CONTACT_TOL_MM.

    Bisection over BISECTION_STEPS steps between the gripper lower limit and CONTACT_HI_RAD.

    Args:
        gripper (Gripper): Loaded gripper.
        solid (trimesh.Trimesh): Watertight object held between the jaws (mm, gripper_link).

    Returns:
        float: The contact angle (rad).
    """
    lo, hi = gripper.lower_rad, CONTACT_HI_RAD
    for _ in range(BISECTION_STEPS):
        mid = (lo + hi) / 2.0
        if penetration_mm(solid, moving_jaw(gripper, mid)) <= CONTACT_TOL_MM:
            hi = mid
        else:
            lo = mid
    return hi
