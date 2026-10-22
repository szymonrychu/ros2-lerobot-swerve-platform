"""FK safety check of arm poses and joint paths with the calibration board on the gripper: pure numpy, no ROS.

Every pose is evaluated in the robot base_link (x forward, y left, z up from the floor): the arm URDF FK runs with
`urdf = measured + offset` (`HeadArmFitConfig.joint_offsets_rad`) and is placed by `arm_in_robot_base(
fit.base_in_base_link)`. The body is the client.yml mcp_server `body` block (boxes and vertical cylinders, the same
geometry choices as mcp_server `body_model.py` / `self_collision.py`: exact point signed distances, link capsules
between FK link origins, the upper arm exempt from the crossbar it stands on); the head is a vertical cylinder about the
pan axis of the given head URDF that contains the head at every pan/tilt inside that URDF's limits (`head_envelope`).

Distances are signed (negative = penetration). Every obstacle is convex, so the distance along a capsule axis is a
convex function of the axis parameter: its minimum is found by golden-section search, vectorised over every capsule
and obstacle at once. mcp_server keeps its own checks on top of this one.
"""

import math
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from typing import Self

import numpy as np
from pydantic import Field, model_validator

from ros2_common.head_rgbd_calibration.chain import UrdfChain, arm_in_robot_base, parse_vec, pose_from_xyz_rpy
from ros2_common.head_rgbd_calibration.charuco import board_outline
from ros2_common.head_rgbd_calibration.config import ARM_JOINT_NAMES, HeadArmFitConfig, StrictModel

MM_PER_M = 1000.0
GRIPPER_JOINT = "gripper"
SHOULDER_LIFT = "shoulder_lift"
ELBOW_FLEX = "elbow_flex"
WRIST_ROLL = "wrist_roll"
BOARD_LINK = "gripper_link"
TOOL_LINK = "gripper_frame_link"
# Link capsules (part, from link origin, to link origin), as the upper arm, forearm and wrist capsules of mcp_server.
CAPSULES = (
    ("upper arm", "upper_arm_link", "lower_arm_link"),
    ("forearm", "lower_arm_link", "wrist_link"),
    ("wrist", "wrist_link", TOOL_LINK),
)
FK_LINKS = ("upper_arm_link", "lower_arm_link", "wrist_link", BOARD_LINK, TOOL_LINK)
UPPER_ARM = "upper arm"
CROSSBAR = "crossbar"  # the shoulder stands on it: never checked against the upper arm
BOARD_LINK_PARTS = ("upper arm", "forearm")  # the board sits on the wrist end, so only these are checked against it
HEAD_OBSTACLE = "head"
HEAD_MOUNT_JOINT = "head_mount_joint"
HEAD_PAN = "head_pan"
HEAD_TILT = "head_tilt"
HEAD_SAMPLE_STEP_RAD = math.radians(15.0)
RIM_SIDES = 32  # cylinder visuals as circumscribed polygons: their vertices bound the round rim from outside
GOLDEN_RATIO = (math.sqrt(5.0) - 1.0) / 2.0
GOLDEN_ITERATIONS = 48  # shrinks the bracket below 1e-10 of the capsule length
PATH_FRACTION = "at path fraction"
DEFAULT_BOXES = (
    ("frame_left", (0.0, 0.1333, 0.15), (0.425, 0.035, 0.06)),
    ("frame_right", (0.0, -0.1333, 0.15), (0.425, 0.035, 0.06)),
    ("crossbar", (-0.025, 0.0, 0.08), (0.25, 0.2316, 0.04)),
)
DEFAULT_CYLINDERS = (
    ("wheel_fl", 0.1525, 0.1333, 0.0625, 0.0, 0.12),
    ("wheel_fr", 0.1525, -0.1333, 0.0625, 0.0, 0.12),
    ("wheel_rl", -0.1525, 0.1333, 0.0625, 0.0, 0.12),
    ("wheel_rr", -0.1525, -0.1333, 0.0625, 0.0, 0.12),
    ("lidar", 0.150, 0.040, 0.037, 0.10, 0.21),
)

Vec3 = tuple[float, float, float]


class BoxObstacle(StrictModel):
    """Axis-aligned box in base_link: centre and full size (m)."""

    name: str
    center: Vec3
    size: Vec3

    @model_validator(mode="after")
    def check_size(self) -> Self:
        if min(self.size) <= 0:
            raise ValueError(f"box {self.name}: every size must be > 0")
        return self


class CylinderObstacle(StrictModel):
    """Vertical cylinder about (x, y) from z_min to z_max in base_link (m)."""

    name: str
    x: float
    y: float
    radius: float = Field(gt=0)
    z_min: float
    z_max: float

    @model_validator(mode="after")
    def check_height(self) -> Self:
        if self.z_max <= self.z_min:
            raise ValueError(f"cylinder {self.name}: z_max must be above z_min")
        return self


def default_boxes() -> list[BoxObstacle]:
    """The client.yml mcp_server body boxes."""
    return [BoxObstacle(name=n, center=c, size=s) for n, c, s in DEFAULT_BOXES]


def default_cylinders() -> list[CylinderObstacle]:
    """The client.yml mcp_server body cylinders."""
    return [
        CylinderObstacle(name=n, x=x, y=y, radius=r, z_min=lo, z_max=hi) for n, x, y, r, lo, hi in DEFAULT_CYLINDERS
    ]


class BodyConfig(StrictModel):
    """Robot body obstacles, a copy of the client.yml mcp_server `body` block (a test pins them equal)."""

    boxes: list[BoxObstacle] = Field(default_factory=default_boxes)
    cylinders: list[CylinderObstacle] = Field(default_factory=default_cylinders)


class PoseCheckConfig(StrictModel):
    """Pose check settings (m, rad).

    `head_radius_m` and `head_height_m` (above the head mount z) are the head envelope of the shipped head URDF;
    `head_envelope` grows them for a URDF whose head reaches further.
    """

    body: BodyConfig = Field(default_factory=BodyConfig)
    margin_m: float = Field(default=0.03, ge=0)
    min_tool_height_m: float = Field(default=0.10, ge=0)
    min_board_height_m: float = Field(default=0.05, ge=0)
    link_radius_m: float = Field(default=0.025, ge=0)
    head_radius_m: float = Field(default=0.075, ge=0)
    head_height_m: float = Field(default=0.075, ge=0)
    roll_min_tool_height_m: float = Field(default=0.15, ge=0)
    max_roll_gripper_rad: float = Field(default=0.8, gt=0)
    stretched_shoulder_lift_max_rad: float = 1.85
    path_step_rad: float = Field(default=0.05, gt=0)
    board_grid: int = Field(default=5, ge=2)

    @model_validator(mode="after")
    def check_crossbar(self) -> Self:
        if not any(box.name == CROSSBAR for box in self.body.boxes):
            raise ValueError(f"body.boxes must name the {CROSSBAR} (the head envelope stands on it)")
        return self


def limit_samples(lower: float, upper: float, step: float) -> np.ndarray:
    """Angles from lower to upper, both included, no step over `step`.

    Args:
        lower: Lower limit (rad).
        upper: Upper limit (rad).
        step: Largest step (rad, > 0).

    Returns:
        np.ndarray: The samples.
    """
    count = max(1, math.ceil((upper - lower) / step - 1e-9))
    return np.linspace(lower, upper, count + 1)


def visual_hull_points(urdf_text: str) -> dict[str, np.ndarray]:
    """Points per child link whose convex hull contains every visual of that link.

    Boxes give their corners, cylinders the vertices of circumscribed polygons at both ends, spheres the corners of
    their bounding cube; a link without visuals (a camera frame) gives its origin only.

    Args:
        urdf_text: URDF XML.

    Returns:
        dict[str, np.ndarray]: (K, 3) points in the link frame per link that is a joint child.

    Raises:
        ValueError: A visual has a mesh or an unknown geometry (the hull is not known).
    """
    root = ET.fromstring(urdf_text)
    children = {j.find("child").get("link") for j in root.findall("joint")}
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)
    angles = np.arange(RIM_SIDES) * 2.0 * math.pi / RIM_SIDES
    out: dict[str, np.ndarray] = {}
    for link in root.findall("link"):
        name = link.get("name", "")
        if name not in children:
            continue
        points = []
        for visual in link.findall("visual"):
            origin = visual.find("origin")
            xyz = parse_vec(None if origin is None else origin.get("xyz"), (0.0, 0.0, 0.0))
            rpy = parse_vec(None if origin is None else origin.get("rpy"), (0.0, 0.0, 0.0))
            geometry = visual.find("geometry")
            shape = None if geometry is None or len(geometry) == 0 else geometry[0]
            if shape is not None and shape.tag == "box":
                local = corners * np.array(parse_vec(shape.get("size"), (0.0, 0.0, 0.0))) / 2.0
            elif shape is not None and shape.tag == "cylinder":
                rim = float(shape.get("radius", 0.0)) / math.cos(math.pi / RIM_SIDES)
                half = float(shape.get("length", 0.0)) / 2.0
                ring = np.column_stack([rim * np.cos(angles), rim * np.sin(angles)])
                local = np.vstack([np.column_stack([ring, np.full(RIM_SIDES, z)]) for z in (-half, half)])
            elif shape is not None and shape.tag == "sphere":
                local = corners * float(shape.get("radius", 0.0))
            else:
                raise ValueError(f"link {name}: visual geometry {None if shape is None else shape.tag!r} has no hull")
            pose = pose_from_xyz_rpy(xyz, rpy)
            points.append(local @ pose[:3, :3].T + pose[:3, 3])
        out[name] = np.vstack(points) if points else np.zeros((1, 3))
    return out


def head_envelope(head_urdf: str, cfg: PoseCheckConfig) -> CylinderObstacle:
    """Vertical cylinder about the head pan axis that contains the head at every pan/tilt inside the URDF limits.

    The axis is the x, y of `head_mount_joint` in the given URDF (pass the applied chain), the bottom the crossbar
    top, the top the mount z plus `head_height_m`, the radius `head_radius_m`; both grow to contain the hull points of
    every head visual (a link without visuals by its origin) at pan and tilt sampled at the URDF limits and every
    15 deg between. The growth adds the largest arc sag between samples, so the head between them is contained too.

    Args:
        head_urdf: Head URDF XML (with `head_mount_joint`, `head_pan`, `head_tilt`).
        cfg: Pose check settings.

    Returns:
        CylinderObstacle: The envelope, named "head".

    Raises:
        ValueError: A head joint has no limits.
    """
    chain = UrdfChain.from_text(head_urdf)
    (mx, my, mz), _ = chain.origin(HEAD_MOUNT_JOINT)
    grids = []
    for joint in (HEAD_PAN, HEAD_TILT):
        limits = chain.limits(joint)
        if limits is None:
            raise ValueError(f"head URDF joint {joint} has no limits")
        grids.append(limit_samples(*limits, HEAD_SAMPLE_STEP_RAD))
    pans, tilts = grids
    step = max(float(np.max(np.diff(g), initial=0.0)) for g in grids)
    pivot_reach = sum(float(np.linalg.norm(chain.origin(j)[0])) for j in (HEAD_PAN, HEAD_TILT))
    mount = np.array([mx, my, mz])
    hulls = visual_hull_points(head_urdf)
    radial = reach = 0.0
    top = -math.inf
    for pan in pans:
        for tilt in tilts:
            positions = {HEAD_PAN: float(pan), HEAD_TILT: float(tilt)}
            for link, local in hulls.items():
                pose = chain.transform(link, positions)
                placed = local @ pose[:3, :3].T + pose[:3, 3]
                moves = bool({HEAD_PAN, HEAD_TILT} & set(chain.joint_names(link)))
                link_reach = float(np.linalg.norm(placed - mount, axis=1).max()) + pivot_reach if moves else 0.0
                reach = max(reach, link_reach)
                radial = max(radial, float(np.hypot(placed[:, 0] - mx, placed[:, 1] - my).max()))
                top = max(top, float(placed[:, 2].max()))
    # Between grid samples a point leaves the chord by at most reach * (1 - cos(step / 2)) per joint; the radial
    # distance and z are convex along the chord, so twice that bounds the head anywhere inside the limits.
    sag = 2.0 * reach * (1.0 - math.cos(step / 2.0))
    crossbar = next(box for box in cfg.body.boxes if box.name == CROSSBAR)
    return CylinderObstacle(
        name=HEAD_OBSTACLE,
        x=mx,
        y=my,
        radius=max(cfg.head_radius_m, radial + sag),
        z_min=crossbar.center[2] + crossbar.size[2] / 2.0,
        z_max=max(mz + cfg.head_height_m, top + sag),
    )


class ObstacleSet:
    """Boxes and vertical cylinders as arrays, for signed distances of many points to every obstacle at once."""

    def __init__(self, boxes: list[BoxObstacle], cylinders: list[CylinderObstacle]) -> None:
        """Stack the obstacle dimensions.

        Args:
            boxes: Box obstacles (first in `names`).
            cylinders: Cylinder obstacles (after the boxes).
        """
        self.names = [b.name for b in boxes] + [c.name for c in cylinders]
        self.box_centre = np.array([b.center for b in boxes], dtype=float).reshape(-1, 3)
        self.box_half = np.array([b.size for b in boxes], dtype=float).reshape(-1, 3) / 2.0
        self.cyl_xy = np.array([(c.x, c.y) for c in cylinders], dtype=float).reshape(-1, 2)
        self.cyl_radius = np.array([c.radius for c in cylinders], dtype=float)
        self.cyl_mid = np.array([(c.z_min + c.z_max) / 2.0 for c in cylinders], dtype=float)
        self.cyl_half = np.array([(c.z_max - c.z_min) / 2.0 for c in cylinders], dtype=float)

    def distances(self, points: np.ndarray) -> np.ndarray:
        """Exact signed distance of every point to every obstacle (negative inside, the depth to the nearest face).

        Args:
            points: (..., 3) points in base_link (m).

        Returns:
            np.ndarray: (..., n_obstacles) distances (m), obstacles in `names` order.
        """
        p = np.asarray(points, dtype=float)[..., None, :]
        q = np.abs(p - self.box_centre) - self.box_half
        box = np.linalg.norm(np.maximum(q, 0.0), axis=-1) + np.minimum(q.max(axis=-1), 0.0)
        radial = np.linalg.norm(p[..., :2] - self.cyl_xy, axis=-1) - self.cyl_radius
        vertical = np.abs(p[..., 2] - self.cyl_mid) - self.cyl_half
        cyl = np.hypot(np.maximum(radial, 0.0), np.maximum(vertical, 0.0)) + np.minimum(
            np.maximum(radial, vertical), 0.0
        )
        return np.concatenate([box, cyl], axis=-1)

    def segment_distances(self, a: np.ndarray, b: np.ndarray) -> np.ndarray:
        """Smallest signed distance along each segment a -> b to every obstacle (golden-section over the convex
        distance along the segment).

        Args:
            a: (S, 3) segment starts (m).
            b: (S, 3) segment ends (m).

        Returns:
            np.ndarray: (S, n_obstacles) distances (m).
        """
        a = np.asarray(a, dtype=float)[:, None, :]
        d = np.asarray(b, dtype=float)[:, None, :] - a
        count = len(self.names)
        index = np.arange(count)

        def at(t: np.ndarray) -> np.ndarray:
            return self.distances(a + d * t[..., None])[:, index, index]

        lo, hi = np.zeros((a.shape[0], count)), np.ones((a.shape[0], count))
        x1, x2 = hi - GOLDEN_RATIO * (hi - lo), lo + GOLDEN_RATIO * (hi - lo)
        f1, f2 = at(x1), at(x2)
        for _ in range(GOLDEN_ITERATIONS):
            left = f1 < f2  # the minimum lies in [lo, x2]
            hi = np.where(left, x2, hi)
            lo = np.where(left, lo, x1)
            x2_new = np.where(left, x1, lo + GOLDEN_RATIO * (hi - lo))
            x1_new = np.where(left, hi - GOLDEN_RATIO * (hi - lo), x2)
            f_new = at(np.where(left, x1_new, x2_new))
            f1, f2 = np.where(left, f_new, f2), np.where(left, f1, f_new)
            x1, x2 = x1_new, x2_new
        ends = np.minimum(at(np.zeros_like(lo)), at(np.ones_like(lo)))
        return np.minimum(np.minimum(f1, f2), ends)


def point_segment_distances(points: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Euclidean distance of points to the segment a -> b.

    Args:
        points: (N, 3) points (m).
        a: (3,) segment start (m).
        b: (3,) segment end (m).

    Returns:
        np.ndarray: (N,) distances (m).
    """
    d = b - a
    length2 = float(d @ d)
    t = np.zeros(len(points)) if length2 == 0.0 else np.clip((points - a) @ d / length2, 0.0, 1.0)
    return np.linalg.norm(points - (a + t[:, None] * d), axis=1)


def board_points_local(fit: HeadArmFitConfig, grid: int) -> np.ndarray:
    """Board outline corners plus a grid x grid lattice over the outline, in gripper_link.

    Args:
        fit: Fit config (`board`, `board_mount` xyz_init/rpy_init/outline_margin_m).
        grid: Points per board edge.

    Returns:
        np.ndarray: (4 + grid * grid, 3) points (m).
    """
    outline = board_outline(fit.board, fit.board_mount.outline_margin_m)
    xs = np.linspace(outline[:, 0].min(), outline[:, 0].max(), grid)
    ys = np.linspace(outline[:, 1].min(), outline[:, 1].max(), grid)
    lattice = np.array([[x, y, 0.0] for y in ys for x in xs])
    board = np.vstack([outline, lattice])
    mount = pose_from_xyz_rpy(fit.board_mount.xyz_init, fit.board_mount.rpy_init)
    return board @ mount[:3, :3].T + mount[:3, 3]


def mm(value_m: float) -> str:
    """Metres as a mm string with one decimal."""
    return f"{value_m * MM_PER_M:.1f} mm"


class PoseChecker:
    """FK check of arm poses and joint paths with the board on the gripper, against the floor, body and head."""

    def __init__(
        self,
        arm_urdf: str,
        head_urdf: str,
        fit: HeadArmFitConfig,
        cfg: PoseCheckConfig,
        board_margin_m: float | None = None,
    ) -> None:
        """Parse the arm, build the head envelope and the obstacle set.

        Args:
            arm_urdf: Arm URDF XML (so101_arm.urdf).
            head_urdf: Head URDF XML, the applied chain (the head envelope reads its mount and joint limits).
            fit: Head-from-arm fit config: joint offsets, arm base in base_link, board and its mount.
            cfg: Pose check settings.
            board_margin_m: Clearance the board points keep from the obstacles and the arm links (m); None uses
                `cfg.margin_m`. The arm sequence passes `margin_m + provisional_margin_m` for the estimated
                board mount (design D47): the mount uncertainty moves the board, not the arm links.

        Raises:
            ValueError: `board_margin_m` is negative.
        """
        if board_margin_m is not None and board_margin_m < 0.0:
            raise ValueError(f"board_margin_m must be >= 0, got {board_margin_m}")
        self.chain = UrdfChain.from_text(arm_urdf)
        self.fit = fit
        self.cfg = cfg
        self.board_margin_m = cfg.margin_m if board_margin_m is None else board_margin_m
        self.base = arm_in_robot_base(fit.base_in_base_link.model_dump())
        self.envelope = head_envelope(head_urdf, cfg)
        self.obstacles = ObstacleSet(list(cfg.body.boxes), [*cfg.body.cylinders, self.envelope])
        self.board_local = board_points_local(fit, cfg.board_grid)
        self.limits = {name: self.chain.limits(name) for name in (*ARM_JOINT_NAMES, GRIPPER_JOINT)}

    def link_poses(self, joints: Mapping[str, float]) -> dict[str, np.ndarray]:
        """Poses of the checked arm links in base_link.

        Args:
            joints: Measured angle (rad) of every arm joint.

        Returns:
            dict[str, np.ndarray]: 4x4 pose per link of FK_LINKS.
        """
        offsets = self.fit.joint_offsets_rad
        return {link: self.base @ self.chain.transform(link, joints, offsets=offsets) for link in FK_LINKS}

    def tool_point(self, joints: Mapping[str, float]) -> np.ndarray:
        """Tool point (gripper_frame_link origin) in base_link (m)."""
        return self.link_poses(joints)[TOOL_LINK][:3, 3]

    def board_points(self, joints: Mapping[str, float]) -> np.ndarray:
        """Board outline and grid points in base_link (m), (4 + board_grid^2, 3)."""
        gripper = self.link_poses(joints)[BOARD_LINK]
        return self.board_local @ gripper[:3, :3].T + gripper[:3, 3]

    def check_pose(self, joints: dict[str, float], gripper_rad: float, rolling: bool) -> list[str]:
        """Check one arm pose with the board on the gripper.

        Args:
            joints: Measured angle (rad) of every arm joint (other keys are ignored).
            gripper_rad: Gripper angle (rad) held during the move.
            rolling: True when wrist_roll changes on this move.

        Returns:
            list[str]: Violations, each "<rule>: <part> ... <clearance>"; empty when the pose is safe.

        Raises:
            ValueError: An arm joint is missing.
        """
        missing = [name for name in ARM_JOINT_NAMES if name not in joints]
        if missing:
            raise ValueError(f"pose is missing arm joints: {', '.join(missing)}")
        cfg = self.cfg
        entries = self.joint_limit_entries(joints, gripper_rad)
        if joints[SHOULDER_LIFT] > cfg.stretched_shoulder_lift_max_rad and joints[ELBOW_FLEX] < 0.0:
            entries.append(
                f"stretched_shoulder_lift_max_rad: shoulder_lift {joints[SHOULDER_LIFT]:.3f} rad > "
                f"{cfg.stretched_shoulder_lift_max_rad:.3f} rad with elbow_flex {joints[ELBOW_FLEX]:.3f} rad < 0 "
                "(stretched arm)"
            )
        poses = self.link_poses(joints)
        tool_z = float(poses[TOOL_LINK][2, 3])
        if tool_z < cfg.min_tool_height_m:
            entries.append(f"min_tool_height_m: tool point height {mm(tool_z)} < {mm(cfg.min_tool_height_m)}")
        if rolling:
            if gripper_rad > cfg.max_roll_gripper_rad:
                entries.append(
                    f"max_roll_gripper_rad: gripper {gripper_rad:.3f} rad > {cfg.max_roll_gripper_rad:.3f} rad "
                    "while wrist_roll changes"
                )
            if tool_z < cfg.roll_min_tool_height_m:
                entries.append(
                    f"roll_min_tool_height_m: tool point height {mm(tool_z)} < {mm(cfg.roll_min_tool_height_m)} "
                    "while wrist_roll changes"
                )
        gripper = poses[BOARD_LINK]
        board = self.board_local @ gripper[:3, :3].T + gripper[:3, 3]
        board_z = float(board[:, 2].min())
        if board_z < cfg.min_board_height_m:
            entries.append(f"min_board_height_m: board point height {mm(board_z)} < {mm(cfg.min_board_height_m)}")
        margin = f"margin_m {mm(cfg.margin_m)}"
        board_margin = f"board_margin_m {mm(self.board_margin_m)}"
        for name, clearance in zip(self.obstacles.names, self.obstacles.distances(board).min(axis=0), strict=True):
            if clearance < self.board_margin_m:
                entries.append(f"board_clearance: board to {name} clearance {mm(clearance)} < {board_margin}")
        starts = np.array([poses[start][:3, 3] for _, start, _ in CAPSULES])
        ends = np.array([poses[end][:3, 3] for _, _, end in CAPSULES])
        link_clear = self.obstacles.segment_distances(starts, ends) - cfg.link_radius_m
        for (part, _, _), row in zip(CAPSULES, link_clear, strict=True):
            for name, clearance in zip(self.obstacles.names, row, strict=True):
                if part == UPPER_ARM and name == CROSSBAR:
                    continue
                if clearance < cfg.margin_m:
                    entries.append(f"link_clearance: {part} to {name} clearance {mm(clearance)} < {margin}")
        for (part, _, _), start, end in zip(CAPSULES, starts, ends, strict=True):
            if part not in BOARD_LINK_PARTS:
                continue
            clearance = float(point_segment_distances(board, start, end).min()) - cfg.link_radius_m
            if clearance < self.board_margin_m:
                entries.append(f"board_link_clearance: board to {part} clearance {mm(clearance)} < {board_margin}")
        return entries

    def joint_limit_entries(self, joints: Mapping[str, float], gripper_rad: float) -> list[str]:
        """Joints outside the arm URDF limits (`urdf = measured + offset`; the gripper has no offset).

        Args:
            joints: Measured arm joint angles (rad).
            gripper_rad: Gripper angle (rad).

        Returns:
            list[str]: One "joint_limits: ..." entry per joint outside its limits.
        """
        entries = []
        offsets = self.fit.joint_offsets_rad
        angles = {name: joints[name] + offsets.get(name, 0.0) for name in ARM_JOINT_NAMES} | {
            GRIPPER_JOINT: gripper_rad
        }
        for name, angle in angles.items():
            limits = self.limits[name]
            if limits is None:
                continue
            lower, upper = limits
            excess = max(lower - angle, angle - upper)
            if excess > 0.0:
                entries.append(
                    f"joint_limits: {name} urdf {angle:.3f} rad outside [{lower:.3f}, {upper:.3f}] rad "
                    f"by {excess:.3f} rad"
                )
        return entries

    def check_path(self, start: dict[str, float], target: dict[str, float], gripper_rad: float) -> list[str]:
        """Check the joint-linear path start -> target, sampled with no joint step over `path_step_rad`.

        Args:
            start: Measured start angles (rad) of every arm joint (other keys are ignored).
            target: Target angles (rad) of every arm joint (other keys are ignored).
            gripper_rad: Gripper angle (rad) held during the move.

        Returns:
            list[str]: Every violation of every sample, each ending "at path fraction <f>"; empty when safe.

        Raises:
            ValueError: An arm joint is missing from start or target.
        """
        for label, pose in (("start", start), ("target", target)):
            missing = [name for name in ARM_JOINT_NAMES if name not in pose]
            if missing:
                raise ValueError(f"path {label} is missing arm joints: {', '.join(missing)}")
        delta = {name: target[name] - start[name] for name in ARM_JOINT_NAMES}
        rolling = delta[WRIST_ROLL] != 0.0
        count = max(1, math.ceil(max(abs(v) for v in delta.values()) / self.cfg.path_step_rad - 1e-9))
        entries = []
        for i in range(count + 1):
            fraction = i / count
            sample = {name: start[name] + delta[name] * fraction for name in ARM_JOINT_NAMES}
            entries += [f"{e} {PATH_FRACTION} {fraction:.3f}" for e in self.check_pose(sample, gripper_rad, rolling)]
        return entries
