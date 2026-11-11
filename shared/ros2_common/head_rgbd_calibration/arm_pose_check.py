"""FK safety check of arm poses and joint paths with the calibration board on the gripper: pure numpy, no ROS.

Every pose is evaluated in the robot base_link (x forward, y left, z up from the floor): the arm URDF FK runs with
`urdf = measured + offset` (`HeadArmFitConfig.joint_offsets_rad`) and is placed by `arm_in_robot_base(
fit.base_in_base_link)`. The body is the client.yml mcp_server `body` block (boxes and vertical cylinders, the same
geometry choices as mcp_server `body_model.py` / `self_collision.py`: exact point signed distances, link capsules
between FK link origins, the upper arm exempt from the crossbar it stands on). The head has two models:

- the full-range envelope (`head_envelope`, the default): a vertical cylinder about the head mount (`head_mount_joint`
  x, y) of the given head URDF that contains the head at every pan/tilt inside that URDF's limits;
- the real head over a pan/tilt region (`HeadRegion`, `head_region_boxes`): the head URDF's collision (else visual)
  primitives as boxes, posed at pan/tilt samples covering the region and inflated to contain the head between them,
  as oriented boxes named `head <link>`. `PoseChecker.head_band` (a held head) and `PoseChecker.head_span` (a head
  move) build the regions; `PoseChecker.check_head_move` checks the standing arm against the head alone.

Each obstacle name is reported once, with its worst clearance.

Distances are signed (negative = penetration). Every obstacle is convex, so the distance along a capsule axis is a
convex function of the axis parameter: its minimum is found by golden-section search, vectorised over every capsule
and obstacle at once. mcp_server keeps its own checks on top of this one.

`PoseChecker.check_view` checks one view of the board by the head D435 colour camera at a head pan/tilt: every
ChArUco corner keeps `d435_min_z_m` plus `d435_min_z_margin_m` of depth from the camera's optical frame (the D435
depth min-Z, design D2) and projects inside the colour image of the camera_info. The corners lie on the printed face,
the face the camera sees; the board's thickness is behind it, so it does not bring the corners nearer the camera.
The collision checks use a lattice through the plate (`board_points_local`) and over the material beyond it, the edge
handle's post, gusset and tab (`handle_box_points_local`, from `board_mount.handle_boxes_mm`), no gap wider than
`board_spacing_m` <= `margin_m` (`board_lattice`): a point check misses at most half a cell diagonal of clearance, so
the board rules compare against the board margin plus that bound (`lattice_miss_bound`) and keep the full margin.
"""

import math
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Self

import cv2
import numpy as np
import yaml
from pydantic import Field, model_validator

from ros2_common.camera_geometry import CameraIntrinsics
from ros2_common.head_rgbd_calibration.board_lattice import (
    MIN_SPACING_M,
    handle_box_lattice,
    lattice_miss_bound,
    plate_lattice,
)
from ros2_common.head_rgbd_calibration.chain import UrdfChain, arm_in_robot_base, parse_vec, pose_from_xyz_rpy
from ros2_common.head_rgbd_calibration.charuco import board_corners_3d
from ros2_common.head_rgbd_calibration.config import ARM_JOINT_NAMES, HeadArmFitConfig, StrictModel
from ros2_common.head_rgbd_calibration.head_arm_fit import D435_COLOR_FRAME

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
HEAD_REGION_STEP_RAD = math.radians(5.0)  # largest pan/tilt step between the posed real-head boxes of a region
HEAD_CLEARANCE_RULE = "head_clearance"
HEAD_REGION_PREFIX = "head "  # the region boxes of a head link are named f"head {link}"
RIM_SIDES = 32  # cylinder visuals as circumscribed polygons: their vertices bound the round rim from outside
GOLDEN_RATIO = (math.sqrt(5.0) - 1.0) / 2.0
GOLDEN_ITERATIONS = 48  # shrinks the bracket below 1e-10 of the capsule length
PATH_FRACTION = "at path fraction"
MIN_Z_RULE = "inside D435 min-Z"
FOV_RULE = "outside colour FOV"
D435_MIN_Z_M = 0.15  # depth min-Z of the D435 at 640x360 (design D2, datasheet table 4-12)
D435_MIN_Z_MARGIN_M = 0.05
# The one robot body definition (shared/arm_collision, settings.load_robot_body): the BodyConfig defaults load it.
ROBOT_BODY_FILE = Path(__file__).resolve().parents[2] / "arm_collision" / "arm_collision" / "robot_body.yaml"

Vec3 = tuple[float, float, float]
UNIT_CUBE = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)


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
    """The robot body boxes of robot_body.yaml.

    Returns:
        list[BoxObstacle]: Boxes in file order.
    """
    return [BoxObstacle.model_validate(box) for box in yaml.safe_load(ROBOT_BODY_FILE.read_text())["boxes"]]


def default_cylinders() -> list[CylinderObstacle]:
    """The robot body cylinders of robot_body.yaml.

    Returns:
        list[CylinderObstacle]: Cylinders in file order.
    """
    return [CylinderObstacle.model_validate(c) for c in yaml.safe_load(ROBOT_BODY_FILE.read_text())["cylinders"]]


class BodyConfig(StrictModel):
    """Robot body obstacles, by default the full robot body of shared/arm_collision robot_body.yaml (head included)."""

    boxes: list[BoxObstacle] = Field(default_factory=default_boxes)
    cylinders: list[CylinderObstacle] = Field(default_factory=default_cylinders)


class PoseCheckConfig(StrictModel):
    """Pose check settings (m, rad).

    `head_radius_m` and `head_height_m` (above the head mount z) are the head envelope of the shipped head URDF;
    `head_envelope` grows them for a URDF whose head reaches further. A view keeps every board corner at
    `d435_min_z_m + d435_min_z_margin_m` or more of depth from the camera.
    """

    body: BodyConfig = Field(default_factory=BodyConfig)
    margin_m: float = Field(default=0.005, ge=0)  # the 5 mm keep-away of every body part
    min_tool_height_m: float = Field(default=0.10, ge=0)
    min_board_height_m: float = Field(default=0.05, ge=0)
    link_radius_m: float = Field(default=0.025, ge=0)
    head_radius_m: float = Field(default=0.075, ge=0)
    head_height_m: float = Field(default=0.09, ge=0)
    roll_min_tool_height_m: float = Field(default=0.15, ge=0)
    max_roll_gripper_rad: float = Field(default=0.8, gt=0)
    stretched_shoulder_lift_max_rad: float = 1.85
    path_step_rad: float = Field(default=0.05, gt=0)
    board_spacing_m: float = Field(default=0.005, ge=MIN_SPACING_M)  # largest gap of the board lattice, <= margin_m
    d435_min_z_m: float = Field(default=D435_MIN_Z_M, ge=0)
    d435_min_z_margin_m: float = Field(default=D435_MIN_Z_MARGIN_M, ge=0)
    head_band_deg: float = Field(default=15.0, gt=0.0, le=45.0)  # +- band about a held head (`PoseChecker.head_band`)

    @model_validator(mode="after")
    def check_crossbar(self) -> Self:
        if not any(box.name == CROSSBAR for box in self.body.boxes):
            raise ValueError(f"body.boxes must name the {CROSSBAR} (the head envelope stands on it)")
        return self

    @model_validator(mode="after")
    def check_board_spacing(self) -> Self:
        """Refuse a board lattice coarser than the margin (a point check misses up to half a cell diagonal).

        Returns:
            Self: self.

        Raises:
            ValueError: board_spacing_m above margin_m.
        """
        if self.board_spacing_m > self.margin_m:
            raise ValueError(
                f"board_spacing_m {self.board_spacing_m} must not exceed margin_m {self.margin_m}: a point check "
                "misses up to half a lattice cell diagonal"
            )
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
    """Vertical cylinder about the head mount that contains the head at every pan/tilt inside the URDF limits.

    The axis is centred on the x, y of `head_mount_joint` in the given URDF (pass the applied chain), not on the pan
    axis (a pan origin off the mount is covered by the radius growth below), the bottom the crossbar
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


@dataclass(frozen=True)
class OrientedBox:
    """Box of any orientation in base_link.

    Attributes:
        name: Obstacle name.
        pose: 4x4 pose of the box centre and axes in base_link.
        half: (3,) half sizes along the box axes (m).
    """

    name: str
    pose: np.ndarray
    half: np.ndarray


@dataclass(frozen=True)
class HeadShape:
    """One collision (else visual) primitive of a head link as a box.

    Attributes:
        link: Head URDF link the shape belongs to.
        local: 4x4 pose of the box centre and axes in the link frame.
        half: (3,) half sizes (m).
    """

    link: str
    local: np.ndarray
    half: np.ndarray


@dataclass(frozen=True)
class HeadRegion:
    """A head pan/tilt region: closed ranges (lo, hi) in rad, lo <= hi.

    Attributes:
        pan_rad: head_pan range (rad).
        tilt_rad: head_tilt range (rad).
    """

    pan_rad: tuple[float, float]
    tilt_rad: tuple[float, float]

    def __post_init__(self) -> None:
        """Refuse a range with lo above hi.

        Raises:
            ValueError: A range is reversed.
        """
        for label, (lo, hi) in (("pan_rad", self.pan_rad), ("tilt_rad", self.tilt_rad)):
            if lo > hi:
                raise ValueError(f"head region {label} lower {lo} is above upper {hi}")


def head_shapes(head_urdf: str) -> list[HeadShape]:
    """The head primitives as boxes: a link's collision elements if it has any, else its visuals.

    A box is itself, a cylinder the box circumscribing it (2r x 2r x length), a sphere the cube 2r; a link without
    geometry gives nothing.

    Args:
        head_urdf: Head URDF XML.

    Returns:
        list[HeadShape]: Shapes in link and element order.

    Raises:
        ValueError: An element has a mesh or an unknown geometry.
    """
    out: list[HeadShape] = []
    for link in ET.fromstring(head_urdf).findall("link"):
        name = link.get("name", "")
        elements = link.findall("collision") or link.findall("visual")
        for element in elements:
            origin = element.find("origin")
            xyz = parse_vec(None if origin is None else origin.get("xyz"), (0.0, 0.0, 0.0))
            rpy = parse_vec(None if origin is None else origin.get("rpy"), (0.0, 0.0, 0.0))
            geometry = element.find("geometry")
            shape = None if geometry is None or len(geometry) == 0 else geometry[0]
            if shape is not None and shape.tag == "box":
                half = np.array(parse_vec(shape.get("size"), (0.0, 0.0, 0.0))) / 2.0
            elif shape is not None and shape.tag == "cylinder":
                radius = float(shape.get("radius", 0.0))
                half = np.array([radius, radius, float(shape.get("length", 0.0)) / 2.0])
            elif shape is not None and shape.tag == "sphere":
                half = np.full(3, float(shape.get("radius", 0.0)))
            else:
                kind = None if shape is None else shape.tag
                raise ValueError(f"head link {name}: {element.tag} geometry {kind!r} has no box (mesh or unknown)")
            out.append(HeadShape(link=name, local=pose_from_xyz_rpy(xyz, rpy), half=half))
    return out


def region_samples(lower: float, upper: float) -> np.ndarray:
    """Samples of a region range: one for a degenerate range, else `limit_samples` at HEAD_REGION_STEP_RAD.

    Args:
        lower: Range lower end (rad).
        upper: Range upper end (rad).

    Returns:
        np.ndarray: The samples (rad).
    """
    return np.array([lower]) if upper <= lower else limit_samples(lower, upper, HEAD_REGION_STEP_RAD)


def head_region_boxes(head_urdf_chain: UrdfChain, shapes: list[HeadShape], region: HeadRegion) -> list[OrientedBox]:
    """The head shapes posed over a pan/tilt region, inflated to contain them between the samples.

    Each shape is posed at every pan (tilt) sample of the region (both ends included, no step over
    HEAD_REGION_STEP_RAD) of the joints it hangs from; a static shape is posed once. A point of a shape at any
    pan/tilt of the region is within `rho_pan * dpan / 2 + rho_tilt * dtilt / 2` of the same point at the nearest
    sample (a rotation by theta moves a point at distance r from the axis by at most r * theta), so every box grows
    by that on each half size. d is the actual sample spacing; rho_pan bounds the distance of the shape's box corners
    from the head_pan origin over every tilt (the corner distance from the head_tilt origin plus the tilt origin's
    distance from the pan origin), rho_tilt is the corner distance from the head_tilt origin; zero for a joint the
    link does not hang from.

    Args:
        head_urdf_chain: Head URDF chain (the applied one).
        shapes: Head shapes (`head_shapes`).
        region: The pan/tilt region.

    Returns:
        list[OrientedBox]: The boxes, shape by shape, each named f"head {link}".
    """
    pans, tilts = region_samples(*region.pan_rad), region_samples(*region.tilt_rad)
    dpan = float(np.max(np.diff(pans), initial=0.0))
    dtilt = float(np.max(np.diff(tilts), initial=0.0))
    pan_child = head_urdf_chain.joints[HEAD_PAN].child
    tilt_child = head_urdf_chain.joints[HEAD_TILT].child
    zero = {HEAD_PAN: 0.0, HEAD_TILT: 0.0}
    pan_origin = head_urdf_chain.transform(pan_child, zero)[:3, 3]
    tilt_origin = head_urdf_chain.transform(tilt_child, zero)[:3, 3]
    tilt_lever = float(np.linalg.norm(tilt_origin - pan_origin))
    out: list[OrientedBox] = []
    for shape in shapes:
        joints = set(head_urdf_chain.joint_names(shape.link))
        at_zero = head_urdf_chain.transform(shape.link, zero) @ shape.local
        corners = (UNIT_CUBE * shape.half) @ at_zero[:3, :3].T + at_zero[:3, 3]
        rho_tilt = float(np.linalg.norm(corners - tilt_origin, axis=1).max()) if HEAD_TILT in joints else 0.0
        if HEAD_PAN not in joints:
            rho_pan = 0.0
        elif HEAD_TILT in joints:
            rho_pan = rho_tilt + tilt_lever
        else:
            rho_pan = float(np.linalg.norm(corners - pan_origin, axis=1).max())
        shape_pans = pans if HEAD_PAN in joints else pans[:1]
        shape_tilts = tilts if HEAD_TILT in joints else tilts[:1]
        half = shape.half + (rho_pan * dpan + rho_tilt * dtilt) / 2.0
        for pan in shape_pans:
            for tilt in shape_tilts:
                pose = head_urdf_chain.transform(shape.link, {HEAD_PAN: float(pan), HEAD_TILT: float(tilt)})
                out.append(OrientedBox(name=f"{HEAD_REGION_PREFIX}{shape.link}", pose=pose @ shape.local, half=half))
    return out


class ObstacleSet:
    """Boxes, vertical cylinders and oriented boxes as arrays, for signed distances of many points to every obstacle
    at once."""

    def __init__(
        self,
        boxes: list[BoxObstacle],
        cylinders: list[CylinderObstacle],
        oriented: list[OrientedBox] | None = None,
    ) -> None:
        """Stack the obstacle dimensions.

        Args:
            boxes: Box obstacles (first in `names`).
            cylinders: Cylinder obstacles (after the boxes).
            oriented: Oriented boxes (after the cylinders); None for none.
        """
        oriented = oriented or []
        self.names = [b.name for b in boxes] + [c.name for c in cylinders] + [o.name for o in oriented]
        self.unique_names = list(dict.fromkeys(self.names))
        index = {name: i for i, name in enumerate(self.unique_names)}
        group = np.array([index[n] for n in self.names], dtype=int)
        self.name_order = np.argsort(group, kind="stable")
        self.name_starts = np.searchsorted(group[self.name_order], np.arange(len(self.unique_names)))
        self.n_boxes, self.n_cylinders = len(boxes), len(cylinders)
        self.ori_rotation = np.array([o.pose[:3, :3] for o in oriented], dtype=float).reshape(-1, 3, 3)
        self.ori_centre = np.array([o.pose[:3, 3] for o in oriented], dtype=float).reshape(-1, 3)
        self.ori_half = np.array([o.half for o in oriented], dtype=float).reshape(-1, 3)
        self.ori_radius = np.linalg.norm(self.ori_half, axis=1)  # bounding sphere radius about the centre
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
        p = np.asarray(points, dtype=float)
        flat = oriented_distances(p.reshape(-1, 3), self.ori_rotation, self.ori_centre, self.ori_half)
        oriented = flat.reshape(*p.shape[:-1], len(self.ori_half))
        return np.concatenate([self.body_distances(p[..., None, :], p[..., None, :]), oriented], axis=-1)

    def paired_distances(self, points: np.ndarray) -> np.ndarray:
        """Signed distance of the k-th point of the last-but-one axis to the k-th obstacle.

        Args:
            points: (..., n_obstacles, 3) points in base_link (m), one per obstacle in `names` order.

        Returns:
            np.ndarray: (..., n_obstacles) distances (m).
        """
        p = np.asarray(points, dtype=float)
        nb, nc = self.n_boxes, self.n_cylinders
        local = np.einsum("...ni,nij->...nj", p[..., nb + nc :, :] - self.ori_centre, self.ori_rotation)
        oriented = box_distances(np.abs(local) - self.ori_half)
        return np.concatenate([self.body_distances(p[..., :nb, :], p[..., nb : nb + nc, :]), oriented], axis=-1)

    def worst_point_clearances(self, points: np.ndarray, cutoff: float) -> np.ndarray:
        """Smallest signed distance of a point set to each obstacle name, exact below `cutoff`.

        Broad phase for the many oriented boxes: a box whose bounding sphere keeps `cutoff` or more from the bounding
        sphere of the points gets that sphere gap instead of its exact distance. The gap is a lower bound of the
        distance and not below `cutoff`, so every value below `cutoff` is the exact smallest distance of its name.

        Args:
            points: (N, 3) points in base_link (m), N >= 1.
            cutoff: Distance below which the values must be exact (m).

        Returns:
            np.ndarray: (n_unique,) distances in `unique_names` order (m).
        """
        p = np.asarray(points, dtype=float)
        centre = p.mean(axis=0)
        spread = float(np.linalg.norm(p - centre, axis=1).max())
        gap = np.linalg.norm(self.ori_centre - centre, axis=1) - spread - self.ori_radius
        near = gap < cutoff
        oriented = gap.copy()
        oriented[near] = oriented_distances(p, self.ori_rotation[near], self.ori_centre[near], self.ori_half[near]).min(
            axis=0
        )
        body = self.body_distances(p[:, None, :], p[:, None, :]).min(axis=0)
        return self.worst_by_name(np.concatenate([body, oriented]))

    def body_distances(self, p_box: np.ndarray, p_cyl: np.ndarray) -> np.ndarray:
        """Signed distances of points broadcast against the axis-aligned boxes and the cylinders.

        Args:
            p_box: (..., n_boxes or 1, 3) points for the boxes (m).
            p_cyl: (..., n_cylinders or 1, 3) points for the cylinders (m).

        Returns:
            np.ndarray: (..., n_boxes + n_cylinders) distances (m), in `names` order.
        """
        box = box_distances(np.abs(p_box - self.box_centre) - self.box_half)
        radial = np.linalg.norm(p_cyl[..., :2] - self.cyl_xy, axis=-1) - self.cyl_radius
        vertical = np.abs(p_cyl[..., 2] - self.cyl_mid) - self.cyl_half
        cyl = np.hypot(np.maximum(radial, 0.0), np.maximum(vertical, 0.0)) + np.minimum(
            np.maximum(radial, vertical), 0.0
        )
        return np.concatenate([box, cyl], axis=-1)

    def worst_by_name(self, distances: np.ndarray) -> np.ndarray:
        """The smallest distance per obstacle name.

        Args:
            distances: (..., n_obstacles) distances in `names` order (m).

        Returns:
            np.ndarray: (..., n_unique) distances in `unique_names` order (m).
        """
        return np.minimum.reduceat(distances[..., self.name_order], self.name_starts, axis=-1)

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

        def at(t: np.ndarray) -> np.ndarray:
            return self.paired_distances(a + d * t[..., None])

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


def oriented_distances(points: np.ndarray, rotation: np.ndarray, centre: np.ndarray, half: np.ndarray) -> np.ndarray:
    """Exact signed distance of every point to every oriented box (negative inside).

    The local point of box k is `(p - t_k) @ R_k`; each local axis is one matrix product over every point and box,
    and the box formula runs on (N, n) slabs per axis (no reduction over a length-3 axis).

    Args:
        points: (N, 3) points in base_link (m).
        rotation: (n, 3, 3) box axes (columns) in base_link.
        centre: (n, 3) box centres in base_link (m).
        half: (n, 3) box half sizes (m).

    Returns:
        np.ndarray: (N, n) distances (m).
    """
    offset = np.einsum("ni,nij->nj", centre, rotation)  # t_k @ R_k
    q = [np.abs(points @ rotation[:, :, j].T - offset[:, j]) - half[:, j] for j in range(3)]
    outside = np.sqrt(sum(np.square(np.maximum(qj, 0.0)) for qj in q))
    return outside + np.minimum(np.maximum(np.maximum(q[0], q[1]), q[2]), 0.0)


def box_distances(q: np.ndarray) -> np.ndarray:
    """Signed box distance from the per-axis excess q = |local point| - half (exact, negative inside).

    Args:
        q: (..., 3) per-axis excess (m).

    Returns:
        np.ndarray: (...) distances (m).
    """
    return np.linalg.norm(np.maximum(q, 0.0), axis=-1) + np.minimum(q.max(axis=-1), 0.0)


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


def board_points_local(fit: HeadArmFitConfig, max_spacing_m: float) -> np.ndarray:
    """A lattice over the plate (outline incl. margin, printed face to plate back), in gripper_link.

    The printed face is at board z = 0 and the plate's back face `fit.board.thickness_mm` behind it (+z, away from
    the camera), so the points span the real board volume.

    Args:
        fit: Fit config (`board` incl. thickness_mm, `board_mount` xyz_init/rpy_init/outline_margin_m).
        max_spacing_m: Largest gap between neighbouring points per axis (m).

    Returns:
        np.ndarray: (N, 3) points (m), `board_lattice.plate_lattice` mounted.
    """
    board = fit.board
    local = plate_lattice(
        board.squares_x * board.square_mm / MM_PER_M,
        board.squares_y * board.square_mm / MM_PER_M,
        fit.board_mount.outline_margin_m,
        board.thickness_mm / MM_PER_M,
        max_spacing_m,
    )
    return mounted(fit, local)


def handle_box_points_local(fit: HeadArmFitConfig, max_spacing_m: float) -> np.ndarray:
    """A lattice over each box of `fit.board_mount.handle_boxes_mm`, in gripper_link.

    Args:
        fit: Fit config (`board_mount` xyz_init/rpy_init/handle_boxes_mm).
        max_spacing_m: Largest gap between neighbouring points per axis (m).

    Returns:
        np.ndarray: (N, 3) points (m), box by box; (0, 3) without boxes.
    """
    return mounted(fit, handle_box_lattice(fit.board_mount.handle_boxes_mm, max_spacing_m))


def mounted(fit: HeadArmFitConfig, points: np.ndarray) -> np.ndarray:
    """Board-frame points in gripper_link through `fit.board_mount` (xyz_init, rpy_init).

    Args:
        fit: Fit config.
        points: (N, 3) points in the board frame (m).

    Returns:
        np.ndarray: (N, 3) points in gripper_link (m).
    """
    mount = pose_from_xyz_rpy(fit.board_mount.xyz_init, fit.board_mount.rpy_init)
    return points @ mount[:3, :3].T + mount[:3, 3]


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
            head_urdf: Head URDF XML, the applied chain (the head envelope reads its mount and joint limits, the view
                check its camera frames).
            fit: Head-from-arm fit config: joint offsets, arm base in base_link, board and its mount.
            cfg: Pose check settings.
            board_margin_m: Clearance the board keeps from the obstacles and the arm links (m); None uses
                `cfg.margin_m`. The arm sequence passes `margin_m + provisional_margin_m` for the estimated board
                mount (design D47): the mount uncertainty moves the board, not the arm links. The board points are
                compared against it plus `lattice_miss_bound(cfg.board_spacing_m)` (`board_threshold_m`), so the
                margin holds for the true board surface, not only for the lattice points.

        Raises:
            ValueError: `board_margin_m` is negative.
        """
        if board_margin_m is not None and board_margin_m < 0.0:
            raise ValueError(f"board_margin_m must be >= 0, got {board_margin_m}")
        self.chain = UrdfChain.from_text(arm_urdf)
        self.head = UrdfChain.from_text(head_urdf)
        self.fit = fit
        self.cfg = cfg
        self.board_margin_m = cfg.margin_m if board_margin_m is None else board_margin_m
        self.board_threshold_m = self.board_margin_m + lattice_miss_bound(cfg.board_spacing_m)
        self.base = arm_in_robot_base(fit.base_in_base_link.model_dump())
        self.head_urdf = head_urdf
        self.envelope = head_envelope(head_urdf, cfg)
        self.obstacles = ObstacleSet(list(cfg.body.boxes), [*cfg.body.cylinders, self.envelope])
        self.envelope_only = ObstacleSet([], [self.envelope])
        self.shapes: list[HeadShape] | None = None  # the head shapes, parsed on the first region
        self.region_cache: dict[HeadRegion, tuple[ObstacleSet, ObstacleSet]] = {}
        self.board_local = np.vstack(
            [board_points_local(fit, cfg.board_spacing_m), handle_box_points_local(fit, cfg.board_spacing_m)]
        )
        self.corners_local = mounted(fit, board_corners_3d(fit.board))
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
        """Board plate lattice, then the handle box lattices, in base_link (m)."""
        gripper = self.link_poses(joints)[BOARD_LINK]
        return self.board_local @ gripper[:3, :3].T + gripper[:3, 3]

    def region_sets(self, region: HeadRegion) -> tuple[ObstacleSet, ObstacleSet]:
        """The obstacle sets of a head region, cached per region.

        Args:
            region: The head pan/tilt region.

        Returns:
            tuple[ObstacleSet, ObstacleSet]: The body (boxes, cylinders incl. its static head cylinder) plus the
                region's head boxes, and the region's head boxes alone.

        Raises:
            ValueError: A head shape has a mesh or an unknown geometry.
        """
        if region not in self.region_cache:
            if self.shapes is None:
                self.shapes = head_shapes(self.head_urdf)
            boxes = head_region_boxes(self.head, self.shapes, region)
            body = self.cfg.body
            self.region_cache[region] = (
                ObstacleSet(list(body.boxes), list(body.cylinders), boxes),
                ObstacleSet([], [], boxes),
            )
        return self.region_cache[region]

    def region_obstacles(self, region: HeadRegion | None) -> ObstacleSet:
        """The obstacles of check_pose: body and envelope (None) or body and the region's head boxes.

        Args:
            region: The head region, or None for the full-range envelope.

        Returns:
            ObstacleSet: The obstacles (cached per region).
        """
        return self.obstacles if region is None else self.region_sets(region)[0]

    def head_region_range(self, joint: str, lower: float, upper: float) -> tuple[float, float]:
        """A head joint range widened by +-`head_band_deg` and clipped to the head URDF limits.

        Args:
            joint: head_pan or head_tilt.
            lower: Range lower end (rad).
            upper: Range upper end (rad).

        Returns:
            tuple[float, float]: The (lo, hi) range (rad).
        """
        band = math.radians(self.cfg.head_band_deg)
        lo, hi = lower - band, upper + band
        limits = self.head.limits(joint)
        if limits is not None:
            lo, hi = max(lo, limits[0]), min(hi, limits[1])
        return (lo, hi)

    def head_band(self, pan_rad: float, tilt_rad: float) -> HeadRegion:
        """The region of a held head: +-`head_band_deg` about its pan/tilt, clipped to the head URDF limits.

        Args:
            pan_rad: Held head_pan (rad).
            tilt_rad: Held head_tilt (rad).

        Returns:
            HeadRegion: The band.

        Raises:
            ValueError: The head is further outside its limits than the band.
        """
        return HeadRegion(
            pan_rad=self.head_region_range(HEAD_PAN, pan_rad, pan_rad),
            tilt_rad=self.head_region_range(HEAD_TILT, tilt_rad, tilt_rad),
        )

    def head_span(self, start: tuple[float, float], target: tuple[float, float]) -> HeadRegion:
        """The region of a head move: the (pan, tilt) rectangle spanned by start and target, +-`head_band_deg`,
        clipped to the head URDF limits (the joints may move independently).

        Args:
            start: (pan, tilt) before the move (rad).
            target: (pan, tilt) after the move (rad).

        Returns:
            HeadRegion: The span.

        Raises:
            ValueError: A head pose is further outside its limits than the band.
        """
        (p0, t0), (p1, t1) = start, target
        return HeadRegion(
            pan_rad=self.head_region_range(HEAD_PAN, min(p0, p1), max(p0, p1)),
            tilt_rad=self.head_region_range(HEAD_TILT, min(t0, t1), max(t0, t1)),
        )

    def clearance_entries(
        self,
        poses: dict[str, np.ndarray],
        board: np.ndarray,
        obstacles: ObstacleSet,
        board_rule: str,
        link_rule: str,
    ) -> list[str]:
        """Board lattice and link capsule clearances to the obstacles, one entry per part and obstacle name (worst).

        Args:
            poses: Link poses (`link_poses`).
            board: (N, 3) board points in base_link (m).
            obstacles: The obstacles.
            board_rule: Rule of the board entries.
            link_rule: Rule of the capsule entries.

        Returns:
            list[str]: Board entries, then capsule entries (the upper arm never against the crossbar).
        """
        cfg = self.cfg
        entries = []
        margin = f"margin_m {mm(cfg.margin_m)}"
        miss = mm(self.board_threshold_m - self.board_margin_m)
        board_margin = f"board_margin_m {mm(self.board_margin_m)} + lattice miss bound {miss}"
        names = obstacles.unique_names
        board_clear = obstacles.worst_point_clearances(board, self.board_threshold_m)
        for name, clearance in zip(names, board_clear, strict=True):
            if clearance < self.board_threshold_m:
                entries.append(f"{board_rule}: board to {name} clearance {mm(clearance)} < {board_margin}")
        starts = np.array([poses[start][:3, 3] for _, start, _ in CAPSULES])
        ends = np.array([poses[end][:3, 3] for _, _, end in CAPSULES])
        link_clear = obstacles.worst_by_name(obstacles.segment_distances(starts, ends)) - cfg.link_radius_m
        for (part, _, _), row in zip(CAPSULES, link_clear, strict=True):
            for name, clearance in zip(names, row, strict=True):
                if part == UPPER_ARM and name == CROSSBAR:
                    continue
                if clearance < cfg.margin_m:
                    entries.append(f"{link_rule}: {part} to {name} clearance {mm(clearance)} < {margin}")
        return entries

    def check_head_move(self, joints: Mapping[str, float], gripper_rad: float, head: HeadRegion | None) -> list[str]:
        """Check the arm standing at `joints` against the head alone, the head anywhere in a region (a head move).

        Link capsules keep `margin_m`, the board lattice the board threshold, from the region's head boxes (or the
        full-range envelope when `head` is None); the body is not checked here.

        Args:
            joints: Measured angle (rad) of every arm joint (other keys are ignored).
            gripper_rad: Gripper angle (rad) held, as in `check_pose`; the checked links and the board are on the
                gripper_link side and do not move with it.
            head: The head region the head moves through, or None for the full-range envelope.

        Returns:
            list[str]: Violations, each f"{HEAD_CLEARANCE_RULE}: <part or board> to <head obstacle> clearance ...";
                empty when the head can move there.

        Raises:
            ValueError: An arm joint is missing.
        """
        missing = [name for name in ARM_JOINT_NAMES if name not in joints]
        if missing:
            raise ValueError(f"pose is missing arm joints: {', '.join(missing)}")
        obstacles = self.envelope_only if head is None else self.region_sets(head)[1]
        poses = self.link_poses(joints)
        gripper = poses[BOARD_LINK]
        board = self.board_local @ gripper[:3, :3].T + gripper[:3, 3]
        return self.clearance_entries(poses, board, obstacles, HEAD_CLEARANCE_RULE, HEAD_CLEARANCE_RULE)

    def check_pose(
        self, joints: dict[str, float], gripper_rad: float, rolling: bool, head: HeadRegion | None = None
    ) -> list[str]:
        """Check one arm pose with the board on the gripper.

        Args:
            joints: Measured angle (rad) of every arm joint (other keys are ignored).
            gripper_rad: Gripper angle (rad) held during the move.
            rolling: True when wrist_roll changes on this move.
            head: The region the head stays in (the real head shapes over it, no envelope), or None for the
                full-range envelope.

        Returns:
            list[str]: Violations, each "<rule>: <part> ... <clearance>", one per part and obstacle name (the
                worst); empty when the pose is safe.

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
        miss = mm(self.board_threshold_m - self.board_margin_m)
        board_margin = f"board_margin_m {mm(self.board_margin_m)} + lattice miss bound {miss}"
        obstacles = self.region_obstacles(head)
        entries += self.clearance_entries(poses, board, obstacles, "board_clearance", "link_clearance")
        for part, start, end in CAPSULES:
            if part not in BOARD_LINK_PARTS:
                continue
            a, b = poses[start][:3, 3], poses[end][:3, 3]
            clearance = float(point_segment_distances(board, a, b).min()) - cfg.link_radius_m
            if clearance < self.board_threshold_m:
                entries.append(f"board_link_clearance: board to {part} clearance {mm(clearance)} < {board_margin}")
        return entries

    def check_view(
        self,
        joints: Mapping[str, float],
        pan_rad: float,
        tilt_rad: float,
        intrinsics: CameraIntrinsics,
        camera_frame: str = D435_COLOR_FRAME,
    ) -> list[str]:
        """Check the board corners as the camera sees them with the head at one pan/tilt.

        Args:
            joints: Measured angle (rad) of every arm joint.
            pan_rad: head_pan of the view (rad).
            tilt_rad: head_tilt of the view (rad).
            intrinsics: The colour camera_info of the camera (`CameraIntrinsics.from_camera_info`).
            camera_frame: Optical frame of the camera in the head URDF (default the D435 colour frame).

        Returns:
            list[str]: Violations, each "<rule>: <detail>": MIN_Z_RULE when a ChArUco corner is nearer along the
                optical axis than `d435_min_z_m + d435_min_z_margin_m`, FOV_RULE when one lies behind the camera or
                projects outside the image (pixel centres 0..width-1, 0..height-1); empty when the view is usable.
        """
        cfg = self.cfg
        gripper = self.link_poses(joints)[BOARD_LINK]
        camera = self.head.transform(camera_frame, {HEAD_PAN: pan_rad, HEAD_TILT: tilt_rad})
        corners = self.corners_local @ gripper[:3, :3].T + gripper[:3, 3]
        in_camera = (corners - camera[:3, 3]) @ camera[:3, :3]
        entries = []
        nearest = float(in_camera[:, 2].min())
        limit = cfg.d435_min_z_m + cfg.d435_min_z_margin_m
        if nearest < limit:
            entries.append(
                f"{MIN_Z_RULE}: board corner depth {mm(nearest)} < {mm(limit)} "
                f"(min-Z {mm(cfg.d435_min_z_m)} + margin {mm(cfg.d435_min_z_margin_m)})"
            )
        size = f"{intrinsics.width}x{intrinsics.height} colour image"
        behind = int(np.count_nonzero(in_camera[:, 2] <= 0.0))
        if behind:
            entries.append(f"{FOV_RULE}: {behind} of {len(in_camera)} board corners behind the camera")
            return entries
        dist = np.asarray(intrinsics.distortion or np.zeros(5), dtype=float)
        pixels = cv2.projectPoints(in_camera, np.zeros(3), np.zeros(3), intrinsics.matrix(), dist)[0].reshape(-1, 2)
        outside = (
            (pixels[:, 0] < 0.0)
            | (pixels[:, 0] > intrinsics.width - 1)
            | (pixels[:, 1] < 0.0)
            | (pixels[:, 1] > intrinsics.height - 1)
        )
        count = int(np.count_nonzero(outside))
        if count:
            entries.append(f"{FOV_RULE}: {count} of {len(in_camera)} board corners outside the {size}")
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

    def check_path(
        self,
        start: dict[str, float],
        target: dict[str, float],
        gripper_rad: float,
        head: HeadRegion | None = None,
    ) -> list[str]:
        """Check the joint-linear path start -> target, sampled with no joint step over `path_step_rad`.

        Args:
            start: Measured start angles (rad) of every arm joint (other keys are ignored).
            target: Target angles (rad) of every arm joint (other keys are ignored).
            gripper_rad: Gripper angle (rad) held during the move.
            head: The region the head stays in during the move, or None for the full-range envelope (`check_pose`).

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
            entries += [
                f"{e} {PATH_FRACTION} {fraction:.3f}" for e in self.check_pose(sample, gripper_rad, rolling, head)
            ]
        return entries
