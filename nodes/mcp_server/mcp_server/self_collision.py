"""Arm self-collision soft boundary: clearance of the arm links to the robot body (body_model), no ROS.

Checked parts (CHECKED_PARTS), all from one FK pass per joint sample and converted to base_link with the arm mount:
the upper arm (shoulder_lift axis -> elbow), forearm (elbow -> wrist_flex axis) and wrist link (wrist_flex axis ->
gripper_link), each as a capsule of half the STS3215 width and as points on the faces of its grasp_sim collision boxes
(arm_geometry.LINK_COLLISION_BOXES, which sit off the origin line; the upper arm boxes are contact-only), the gripper
body / fixed jaw hull points and the three jaw points
of the floor guard (jaw tip, tool point, moving jaw tip). The arm base and the shoulder link stand on the crossbar and
only turn in place about the pan axis, so they are not checked at all (in particular not against the crossbar).

Path semantics (BodyReport, over the executed joint trajectory samples): a sample whose smallest clearance is below
body_check.soft_margin_m runs at slow_speed_scale; a sample below hard_margin_m (or penetrating) rejects the whole
motion before it starts. A start that is already inside the hard margin of an obstacle may move out: for that obstacle
the limit becomes the start clearance minus ESCAPE_TOLERANCE_M (the clearance must not get worse).

Pruning (bounded_clearances; path, closest, clear_of): the box face and hull points sit in blocks of CLEAR_OF_BLOCK
neighbours on their link (LINK_BLOCKS) with a bounding sphere. Only the sphere centres are placed for every sample;
a block or capsule whose sphere lower bound is beyond every threshold it could matter for (the sample's smallest soft
clearance, bounded from above by the nearest sphere, and the hard limit) is skipped. Results equal the full
computation (clearance_matrix) bit for bit wherever a verdict or a reported value depends on them; points are placed
element by element (rigid) so a point gets the same bits in any batch.

The clearance computations (path, pose, closest, clear_of, clearance_matrix, clearances_of, bounded_clearances) run in
the C++ kernel mcp_core_cpp.collision.BodyChecker (mcp_core_cpp/src/collision/, docs/collision.md); the forward
kinematics (geometry, part_points) and the report stay in Python. The original Python bodies are
mcp_server/reference/collision.py, which writes the goldens tests/test_core_parity_collision.py checks the kernel
against.
"""

import math
from collections import Counter
from dataclasses import asdict, dataclass, field
from typing import Any

import numpy as np
from mcp_core_cpp.collision import BodyChecker

from .arm_geometry import (
    BOX_POINT_SPACING_M,
    FOREARM_CAPSULE,
    GRIPPER_BODY,
    GRIPPER_BODY_POINTS,
    JAW_POINTS,
    LINK_COLLISION_BOXES,
    LINK_ORIGINS,
    UPPER_ARM_CAPSULE,
    WRIST_CAPSULE,
    box_surface_points,
)
from .body_model import BodyModel, BoxObstacle, CylinderObstacle, Obstacle
from .config import ArmBaseOffset, BodyCheckSettings, McpServerConfig
from .floor_guard import GRIPPER_LINK, JawModel, yaw_matrix
from .ik import ArmKinematics
from .jaw_crop import jaw_points

CAPSULES = (UPPER_ARM_CAPSULE, FOREARM_CAPSULE, WRIST_CAPSULE)
CHECKED_PARTS = (*(c[0] for c in CAPSULES), GRIPPER_BODY, *JAW_POINTS)
# Box face points per capsule part, (link, (K, 3) points in the link frame): held to the margins (BOX_POINTS) and
# contact-only (CONTACT_POINTS, stacked under contact_key(part)).
BOX_POINTS = {
    name: [(b.link, box_surface_points(b)) for b in LINK_COLLISION_BOXES if b.part == name and not b.contact_only]
    for name, *_ in CAPSULES
}
CONTACT_POINTS = {
    name: [(b.link, box_surface_points(b)) for b in LINK_COLLISION_BOXES if b.part == name and b.contact_only]
    for name, *_ in CAPSULES
}


def contact_key(part: str) -> str:
    """part_points key of the contact-only box points of a capsule part.

    Args:
        part (str): Capsule part name.

    Returns:
        str: The key.
    """
    return f"{part} (contact only)"


NORMAL_SCALE = 1.0
# Parts that move when only the gripper joint moves: everything else is fixed to gripper_link or an upstream link.
GRIPPER_MOVING_PARTS = ("moving_jaw_tip",)
# A start inside the hard margin may move out as long as no clearance drops more than this below its start value (m).
ESCAPE_TOLERANCE_M = 0.002
# Joint-space step (rad) of a straight joint line sampled for a check (planner free moves, home routes).
JOINT_LINE_STEP_RAD = 0.02
# Neighbouring box face points grouped under one bounding sphere (SelfCollisionChecker.clear_of, bounded_clearances).
CLEAR_OF_BLOCK = 32
# Slack on the bounding-sphere bounds of the pruned checks (m): covers float rounding, far below any margin.
PRUNE_SLACK_M = 1e-9
ROUND_M = 4
# Obstacle kinds of the C++ kernel's obstacle table (obstacle_rows).
BOX_KIND = 0.0
CYLINDER_KIND = 1.0


def link_model_description() -> dict[str, Any]:
    """The arm link model the check covers (capsules, collision boxes and their face point spacing), for cache keys.

    Returns:
        dict[str, Any]: capsules (name, from, to, radius), boxes (LinkBox fields), spacing_m.
    """
    return {
        "capsules": [list(c) for c in CAPSULES],
        "boxes": [asdict(box) for box in LINK_COLLISION_BOXES],
        "spacing_m": BOX_POINT_SPACING_M,
    }


def joint_line(
    start: dict[str, float], goal: dict[str, float], step: float = JOINT_LINE_STEP_RAD
) -> list[dict[str, float]]:
    """Samples of the straight joint-space line start -> goal (the path every quintic joint move follows).

    Args:
        start (dict[str, float]): Start pose (must name every joint of goal).
        goal (dict[str, float]): Goal pose; joints only in start keep their start value.
        step (float): Largest joint change between samples (rad, > 0).

    Returns:
        list[dict[str, float]]: Samples, start and goal included.
    """
    span = max((abs(goal[j] - start[j]) for j in goal), default=0.0)
    n = max(1, math.ceil(span / step - 1e-9))
    return [start | {j: start[j] + (goal[j] - start[j]) * i / n for j in goal} for i in range(n + 1)]


def point_blocks(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Group the points of every sample in blocks of up to CLEAR_OF_BLOCK neighbours under a bounding sphere.

    A block's signed distance to a convex obstacle lies within its radius of the distance of its centre (the exact
    signed distance changes by at most the distance moved), which is what the pruned checks skip blocks by.

    Args:
        points (np.ndarray): (N, K, 3) points per sample (K >= 1); the last point pads the last block.

    Returns:
        tuple[np.ndarray, np.ndarray, np.ndarray]: blocks (N, B, size, 3), centres (N, B, 3), radii (N, B).
    """
    size = min(CLEAR_OF_BLOCK, points.shape[1])
    pad = -points.shape[1] % size
    if pad:
        points = np.concatenate([points, np.repeat(points[:, -1:], pad, axis=1)], axis=1)
    blocks = points.reshape(len(points), -1, size, 3)
    centre = blocks.mean(axis=2)
    return blocks, centre, np.linalg.norm(blocks - centre[:, :, None, :], axis=-1).max(axis=2)


def rigid(rotation: np.ndarray, translation: np.ndarray, points: np.ndarray) -> np.ndarray:
    """rotation @ point + translation for points (..., 3), element by element: a point gets the same bits whatever
    batch it is placed in (a matrix product may sum in a different order for different shapes), so the pruned checks
    place only the points they evaluate and still match the full computation exactly.

    Args:
        rotation (np.ndarray): (..., 3, 3) rotations (broadcast against the points).
        translation (np.ndarray): (..., 3) translations.
        points (np.ndarray): (..., 3) points.

    Returns:
        np.ndarray: Transformed points (broadcast shape, ..., 3).
    """
    p = np.asarray(points, dtype=np.float64)
    turned = p[..., 0:1] * rotation[..., :, 0] + p[..., 1:2] * rotation[..., :, 1] + p[..., 2:3] * rotation[..., :, 2]
    return turned + translation


@dataclass(frozen=True)
class PointBlocks:
    """Part points fixed to one link, in blocks of up to CLEAR_OF_BLOCK neighbours with their bounding spheres (link
    frame; a rigid placement keeps the radii)."""

    part: int  # index into CHECKED_PARTS
    link: str
    contact_only: bool
    points: np.ndarray  # (B, size, 3)
    centres: np.ndarray  # (B, 3)
    radii: np.ndarray  # (B,)
    count: int  # points before the padding of the last block


def link_blocks() -> tuple[PointBlocks, ...]:
    """The box face points (BOX_POINTS, CONTACT_POINTS) and the gripper body hull points as PointBlocks per link.

    Returns:
        tuple[PointBlocks, ...]: Held box points of each capsule part, its contact-only boxes, then the gripper hull.
    """
    entries = [(p, link, False, pts) for p, (name, *_r) in enumerate(CAPSULES) for link, pts in BOX_POINTS[name]]
    entries += [(p, link, True, pts) for p, (name, *_r) in enumerate(CAPSULES) for link, pts in CONTACT_POINTS[name]]
    entries.append((CHECKED_PARTS.index(GRIPPER_BODY), GRIPPER_LINK, False, GRIPPER_BODY_POINTS))
    out = []
    for part, link, contact, points in entries:
        blocks, centres, radii = point_blocks(np.asarray(points, dtype=np.float64)[None])
        out.append(PointBlocks(part, link, contact, blocks[0], centres[0], radii[0], len(points)))
    return tuple(out)


LINK_BLOCKS = link_blocks()
BLOCK_LINKS = tuple(sorted({blocks.link for blocks in LINK_BLOCKS}))


@dataclass(frozen=True)
class SampleGeometry:
    """Stacked geometry of N configurations for the pruned checks (SelfCollisionChecker.geometry)."""

    fixed: dict[str, np.ndarray]  # fixed_points per key, stacked: capsules (N, 2, 3), jaw points (N, 1, 3)
    poses: dict[str, tuple[np.ndarray, np.ndarray]]  # link -> rotations (N, 3, 3), translations (N, 3) in the arm frame

    @property
    def n(self) -> int:
        """Number of configurations.

        Returns:
            int: N.
        """
        return len(next(iter(self.fixed.values())))

    def rows(self, pick: slice) -> "SampleGeometry":
        """The geometry of a slice of the configurations.

        Args:
            pick (slice): Which configurations.

        Returns:
            SampleGeometry: The sliced geometry.
        """
        return SampleGeometry(
            {key: points[pick] for key, points in self.fixed.items()},
            {link: (rotation[pick], translation[pick]) for link, (rotation, translation) in self.poses.items()},
        )

    def arrays(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """The geometry as the C++ kernel takes it (mcp_core_cpp.collision.BodyChecker).

        Returns:
            tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]: capsule axis ends (N, C, 2, 3) in CAPSULES order,
                jaw points (N, J, 3) in JAW_POINTS order, rotations (N, L, 3, 3) and translations (N, L, 3) of the
                BLOCK_LINKS.
        """
        return (
            np.stack([self.fixed[name] for name, *_ in CAPSULES], axis=1),
            np.concatenate([self.fixed[name] for name in JAW_POINTS], axis=1),
            np.stack([self.poses[link][0] for link in BLOCK_LINKS], axis=1),
            np.stack([self.poses[link][1] for link in BLOCK_LINKS], axis=1),
        )


def obstacle_rows(body: BodyModel) -> np.ndarray:
    """The body obstacles as the C++ kernel's obstacle table: kind 0 (box) lo xyz hi xyz, kind 1 (vertical cylinder)
    x y radius z_min z_max 0.

    Args:
        body (BodyModel): Body obstacles.

    Returns:
        np.ndarray: (M, 7) table.

    Raises:
        TypeError: For an obstacle that is neither a BoxObstacle nor a CylinderObstacle.
    """
    rows = []
    for obstacle in body.obstacles:
        if isinstance(obstacle, BoxObstacle):
            rows.append([BOX_KIND, *obstacle.lo, *obstacle.hi])
        elif isinstance(obstacle, CylinderObstacle):
            x, y = obstacle.center_xy
            rows.append([CYLINDER_KIND, x, y, obstacle.radius, obstacle.z_min, obstacle.z_max, 0.0])
        else:
            raise TypeError(f"no C++ kernel for obstacle {obstacle!r}")
    return np.array(rows, dtype=np.float64).reshape(len(rows), 7)


def body_kernel(body: BodyModel, mount_rotation: np.ndarray, mount_offset: np.ndarray) -> BodyChecker:
    """The C++ kernel of a body and arm mount, holding CAPSULES, LINK_BLOCKS and the part layout of CHECKED_PARTS.

    Args:
        body (BodyModel): Body obstacles.
        mount_rotation (np.ndarray): (3, 3) arm mount rotation in base_link.
        mount_offset (np.ndarray): (3,) arm mount position in base_link.

    Returns:
        BodyChecker: The kernel.
    """
    return BodyChecker(
        obstacle_rows(body),
        np.array([radius for *_, radius in CAPSULES], dtype=np.float64),
        [b.part for b in LINK_BLOCKS],
        [BLOCK_LINKS.index(b.link) for b in LINK_BLOCKS],
        [b.contact_only for b in LINK_BLOCKS],
        [b.count for b in LINK_BLOCKS],
        [b.points for b in LINK_BLOCKS],
        [b.centres for b in LINK_BLOCKS],
        [b.radii for b in LINK_BLOCKS],
        np.asarray(mount_rotation, dtype=np.float64),
        np.asarray(mount_offset, dtype=np.float64),
        len(CHECKED_PARTS),
        len(JAW_POINTS),
        len(BLOCK_LINKS),
        PRUNE_SLACK_M,
        ESCAPE_TOLERANCE_M,
    )


def combine_scales(floor: list[float], body: list[float]) -> list[float]:
    """Per-sample speed scale of the floor guard and the body check together: the slower wins.

    Args:
        floor (list[float]): Floor guard scales.
        body (list[float]): Body check scales (same length).

    Returns:
        list[float]: Combined scales.
    """
    return [min(a, b) for a, b in zip(floor, body, strict=True)]


def obstacle_hint(obstacle: Obstacle, part: str, settings: BodyCheckSettings) -> str:
    """How to get a rejected motion past an obstacle.

    Args:
        obstacle (Obstacle): The limiting obstacle.
        part (str): The arm part that comes too close.
        settings (BodyCheckSettings): Margins.

    Returns:
        str: The hint.
    """
    where = {
        "lidar": "the lidar dome stands front-left of the arm base",
        "wheel": "the wheels steer, so the whole circle around each steer axis is blocked",
        "frame": "the side frames run along both sides above the wheels",
        "crossbar": "the crossbar is the robot deck under and behind the arm base",
    }
    note = next((text for key, text in where.items() if obstacle.name.startswith(key)), "")
    return (
        f"keep the {part} more than {settings.soft_margin_m:g} m from {obstacle.name} (top at z "
        f"{obstacle.top_z:.3f} m in base_link{'; ' + note if note else ''}): raise the arm first (shoulder_lift back, "
        f"elbow up) so it passes above z {obstacle.top_z + settings.soft_margin_m:.2f} m, then lower outside the "
        f"obstacle, or approach from another side (turn shoulder_pan away from {obstacle.name}). get_arm_reach lists "
        "the collision-free envelope and the pitches that work for a point."
    )


@dataclass(frozen=True)
class PoseClearance:
    """Clearance of one joint configuration to every obstacle: name -> (clearance m, closest part), as the hard check
    sees it, and name -> soft clearance (m, what the slow band uses: without the contact-only boxes)."""

    per_obstacle: dict[str, tuple[float, str]]
    soft: dict[str, float] = field(default_factory=dict)

    @property
    def min_soft_clearance(self) -> float:
        """Smallest soft clearance (m; inf without obstacles): below body_check.soft_margin_m a motion runs slowed.

        Returns:
            float: Clearance.
        """
        return min(self.soft.values(), default=math.inf)

    @property
    def obstacle(self) -> str:
        """Obstacle with the smallest clearance.

        Returns:
            str: Obstacle name ('' without obstacles).
        """
        return min(self.per_obstacle, key=lambda n: self.per_obstacle[n][0], default="")

    @property
    def min_clearance(self) -> float:
        """Smallest clearance (m; inf without obstacles).

        Returns:
            float: Clearance.
        """
        return self.per_obstacle[self.obstacle][0] if self.per_obstacle else math.inf

    @property
    def part(self) -> str:
        """Arm part closest to the limiting obstacle.

        Returns:
            str: Part name ('' without obstacles).
        """
        return self.per_obstacle[self.obstacle][1] if self.per_obstacle else ""


@dataclass
class BodyReport:
    """Per-sample body check of a trajectory: speed scales, smallest soft clearance (without the contact-only boxes),
    its obstacle and part, and the hard verdict."""

    scales: list[float]
    clearances: list[float]
    obstacles: list[str]
    parts: list[str]
    settings: BodyCheckSettings
    enabled: bool = True
    violation: int | None = None  # first sample breaking the hard limit
    violation_obstacle: str = ""
    violation_part: str = ""
    violation_clearance: float = math.inf
    hints: dict[str, str] = field(default_factory=dict)

    @property
    def rejected(self) -> bool:
        """Whether the motion must not run.

        Returns:
            bool: True when a sample breaks the hard limit.
        """
        return self.violation is not None

    @property
    def min_clearance(self) -> float:
        """Smallest clearance over the samples (m; inf for no samples or a disabled check).

        Returns:
            float: Clearance.
        """
        return min(self.clearances, default=math.inf)

    def summary(self) -> dict[str, Any] | None:
        """Slowdown report for motion results; None when no sample was slowed.

        Returns:
            dict[str, Any] | None: obstacle, link, min_clearance_m, speed_scale, soft_margin_m, slowed_samples, samples
                and every obstacle that slowed a sample.
        """
        slowed = [i for i, s in enumerate(self.scales) if s < NORMAL_SCALE]
        if not slowed:
            return None
        worst = min(slowed, key=lambda i: self.clearances[i])
        return {
            "obstacle": self.obstacles[worst],
            "link": self.parts[worst],
            "min_clearance_m": round(self.clearances[worst], ROUND_M),
            "speed_scale": self.settings.slow_speed_scale,
            "soft_margin_m": self.settings.soft_margin_m,
            "slowed_samples": len(slowed),
            "samples": len(self.scales),
            "obstacles": sorted({self.obstacles[i] for i in slowed}),
        }

    def rejection(self) -> dict[str, Any]:
        """Why the motion was rejected (empty when it was not).

        Returns:
            dict[str, Any]: obstacle, link, clearance_m, hard_margin_m, sample, samples and a hint.
        """
        if self.violation is None:
            return {}
        return {
            "obstacle": self.violation_obstacle,
            "link": self.violation_part,
            "clearance_m": round(self.violation_clearance, ROUND_M),
            "hard_margin_m": self.settings.hard_margin_m,
            "sample": self.violation,
            "samples": len(self.scales),
            "hint": self.hints.get(self.violation_obstacle, ""),
        }

    def message(self) -> str:
        """One-line rejection message for tool errors.

        Returns:
            str: The message ('' when not rejected).
        """
        r = self.rejection()
        if not r:
            return ""
        return (
            f"refused (self-collision): the {r['link']} would come {r['clearance_m']:.3f} m from the robot body "
            f"obstacle {r['obstacle']} (hard margin {r['hard_margin_m']:g} m) at trajectory sample {r['sample']} of "
            f"{r['samples']}. Hint: {r['hint']} Nothing moved."
        )


class SelfCollisionChecker:
    """Body clearance of arm joint configurations (see the module docstring)."""

    def __init__(
        self,
        kin: ArmKinematics,
        body: BodyModel,
        settings: BodyCheckSettings,
        mount: ArmBaseOffset,
        jaw: JawModel,
        gripper_joint: str = "gripper",
    ) -> None:
        """Bind the checker to the arm model and the body.

        Args:
            kin (ArmKinematics): Kinematics on the arm URDF.
            body (BodyModel): Body obstacles (base_link).
            settings (BodyCheckSettings): Margins, slow scale, enabled.
            mount (ArmBaseOffset): Arm base pose in base_link.
            jaw (JawModel): Moving jaw model.
            gripper_joint (str): Gripper joint name in the samples (closed when missing).
        """
        self.kin = kin
        self.body = body
        self.settings = settings
        self.mount = mount
        self.jaw = jaw
        self.gripper = gripper_joint
        self.rotation = yaw_matrix(mount.yaw)
        self.offset = np.array([mount.x, mount.y, mount.z])
        # The C++ kernels (path, pose, closest, clear_of and the clearance matrices run in it).
        self.core = body_kernel(body, self.rotation, self.offset)
        # Exact distances evaluated ('points': part points, 'capsules': capsule axis searches), for the pruning tests.
        self.distance_counts: Counter[str] = Counter()

    @classmethod
    def from_config(cls, kin: ArmKinematics, config: McpServerConfig, jaw: JawModel) -> "SelfCollisionChecker":
        """Checker from the node config (body, body_check, arm mount; no mount = straight above base_link at the mount
        height, as the floor guard assumes).

        Args:
            kin (ArmKinematics): Kinematics.
            config (McpServerConfig): Node configuration.
            jaw (JawModel): Jaw model.

        Returns:
            SelfCollisionChecker: The checker.
        """
        mount = config.arm.base_in_base_link or ArmBaseOffset(z=config.arm.arm_base_height_m)
        body = BodyModel.from_settings(config.body)
        return cls(kin, body, config.body_check, mount, jaw, config.arm.gripper_joint)

    def to_base_link(self, points: np.ndarray) -> np.ndarray:
        """Arm-frame points (..., 3) -> base_link (element by element, see rigid).

        Args:
            points (np.ndarray): Points in the arm base frame (m).

        Returns:
            np.ndarray: Points in base_link (m).
        """
        return rigid(self.rotation, self.offset, points)

    def fixed_points(self, frames: dict[str, np.ndarray], joints: dict[str, float]) -> dict[str, np.ndarray]:
        """Base_link capsule axis ends and jaw points of one configuration (the parts without box points).

        Args:
            frames (dict[str, np.ndarray]): kin.link_frames(joints).
            joints (dict[str, float]): Joint name -> rad (measured space; gripper optional).

        Returns:
            dict[str, np.ndarray]: capsule parts -> (2, 3) axis ends; jaw points -> (1, 3).
        """
        origins = {key: frames[link][:3, 3] for key, link in LINK_ORIGINS.items()}
        out = {name: self.to_base_link(np.vstack([origins[a], origins[b]])) for name, a, b, _ in CAPSULES}
        jaw = jaw_points(self.kin, self.jaw, frames, joints.get(self.gripper, self.jaw.closed_rad))
        for name in JAW_POINTS:
            out[name] = self.to_base_link(jaw[name][None, :])
        return out

    def part_points(self, joints: dict[str, float]) -> dict[str, np.ndarray]:
        """Base_link geometry of every checked part for one joint configuration.

        Args:
            joints (dict[str, float]): Joint name -> rad (measured space; gripper optional).

        Returns:
            dict[str, np.ndarray]: capsule parts -> (2 + K, 3): the capsule axis ends, then the K points on their
                collision box faces (BOX_POINTS); contact_key(part) -> (K, 3) contact-only box points (CONTACT_POINTS,
                parts that have them); 'gripper body' -> (K, 3) hull points; jaw points -> (1, 3).
        """
        frames = self.kin.link_frames(joints)
        fixed = self.fixed_points(frames, joints)

        def placed(boxes: list[tuple[str, np.ndarray]]) -> np.ndarray:
            return self.to_base_link(
                np.vstack([rigid(frames[link][:3, :3], frames[link][:3, 3], points) for link, points in boxes])
            )

        out = {}
        for name, *_ in CAPSULES:
            out[name] = np.vstack([fixed[name], placed(BOX_POINTS[name])]) if BOX_POINTS[name] else fixed[name]
            if CONTACT_POINTS[name]:
                out[contact_key(name)] = placed(CONTACT_POINTS[name])
        out[GRIPPER_BODY] = placed([(GRIPPER_LINK, GRIPPER_BODY_POINTS)])
        for name in JAW_POINTS:
            out[name] = fixed[name]
        return out

    def clearance_matrix(
        self, samples: list[dict[str, float]]
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Clearance of every sample to every obstacle and the closest part, every distance in full (C++ kernel).

        Args:
            samples (list[dict[str, float]]): Joint configurations.

        Returns:
            tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]: clearances_of output.
        """
        n, m = len(samples), len(self.body.obstacles)
        if n == 0 or m == 0:
            empty, parts = np.zeros((n, m)), np.zeros((n, m), dtype=int)
            return empty, parts, empty, parts
        return self.counted(self.core.full(*self.geometry(samples).arrays(), self.settings.hard_margin_m))

    def stacked_points(self, samples: list[dict[str, float]]) -> dict[str, np.ndarray]:
        """part_points of every sample stacked per part.

        Args:
            samples (list[dict[str, float]]): Joint configurations (at least one).

        Returns:
            dict[str, np.ndarray]: Part key -> (N, K, 3) points (capsules: K = 2 axis ends + box points).
        """
        geometry = [self.part_points(s) for s in samples]
        return {key: np.stack([g[key] for g in geometry]) for key in geometry[0]}

    def counted(
        self, result: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, int, int]
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Add a C++ clearance kernel's distance counts to distance_counts and return its clearances.

        Args:
            result (tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, int, int]): values, parts, soft, soft parts,
                point and capsule distances evaluated.

        Returns:
            tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]: values, parts, soft, soft parts.
        """
        values, parts, soft, soft_parts, points, capsules = result
        self.distance_counts["points"] += points
        self.distance_counts["capsules"] += capsules
        return values, parts, soft, soft_parts

    def clearances_of(self, stacked: dict[str, np.ndarray]) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Clearance of stacked part points to every obstacle and the closest part (C++ kernel).

        The hard check's clearance of a contact-only box is its signed distance plus hard_margin_m (it breaks the
        hard margin exactly when it touches); the soft clearance leaves those boxes out (they never slow a motion).

        Args:
            stacked (dict[str, np.ndarray]): stacked_points output.

        Returns:
            tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]: (N, M) clearances (m) and (N, M) indices into
                CHECKED_PARTS of the closest part as the hard check sees them, then the same two without the
                contact-only boxes (the soft clearances the slow band uses).
        """
        n = stacked[CHECKED_PARTS[0]].shape[0]
        none = np.empty((n, 0, 3))
        return self.counted(
            self.core.points_full(
                [stacked[name] for name, *_ in CAPSULES],
                [stacked.get(contact_key(name), none) for name, *_ in CAPSULES],
                [stacked[part] for part in CHECKED_PARTS[len(CAPSULES) :]],
                self.settings.hard_margin_m,
            )
        )

    def geometry(self, samples: list[dict[str, float]]) -> SampleGeometry:
        """What the pruned checks need of each configuration: one FK pass, the capsule axis ends and jaw points in
        base_link (fixed_points) and the pose of every link that carries LINK_BLOCKS.

        Args:
            samples (list[dict[str, float]]): Joint configurations (at least one).

        Returns:
            SampleGeometry: The stacked geometry.
        """
        frames = [self.kin.link_frames(s) for s in samples]
        rows = [self.fixed_points(f, s) for f, s in zip(frames, samples, strict=True)]
        return SampleGeometry(
            {key: np.stack([row[key] for row in rows]) for key in rows[0]},
            {
                link: (np.stack([f[link][:3, :3] for f in frames]), np.stack([f[link][:3, 3] for f in frames]))
                for link in BLOCK_LINKS
            },
        )

    def bounded_clearances(
        self, geometry: SampleGeometry, hard_limit: np.ndarray | None = None, soft: bool = True
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """clearance_matrix with bounding-sphere pruning, exact wherever a verdict or a reported value depends on it.

        Every capsule and every block of LINK_BLOCKS has a lower bound on its distance to an obstacle (bounding sphere:
        centre distance minus radius) and an upper bound (centre distance plus radius, a capsule axis end). Per sample
        the smallest upper bound bounds its smallest soft clearance from above. Only the block centres are placed in
        base_link for every sample; a capsule or block whose lower bound is above every threshold it could matter for
        is neither placed point by point nor evaluated. So, bit for bit as clearance_matrix computes them:

        - the smallest soft clearance of each sample, its obstacle (soft argmin) and its part (soft_parts there);
        - every hard clearance at or below hard_limit[n, k], with its part (an entry above the limit stays above it);
          without hard_limit the smallest hard clearance of each sample with its obstacle and part.

        Other entries are only known to exceed those thresholds (they hold a larger value, possibly inf). With soft
        False nothing is evaluated for the soft band: only the hard guarantee holds (the soft outputs are not exact).

        Args:
            geometry (SampleGeometry): geometry(samples) of at least one sample (the body has obstacles).
            hard_limit (np.ndarray | None): (N, M) hard clearance per sample and obstacle below which the hard values
                must be exact (the path limit); None for each sample's smallest hard clearance.
            soft (bool): Keep the soft band guarantee (the per-sample smallest soft clearance exact).

        Returns:
            tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]: (N, M) hard clearances (m), (N, M) indices into
                CHECKED_PARTS, (N, M) soft clearances, (N, M) soft part indices, exact as described above.
        """
        limit = None if hard_limit is None else np.ascontiguousarray(hard_limit, dtype=np.float64)
        return self.counted(self.core.bounded(*geometry.arrays(), self.settings.hard_margin_m, limit, soft))

    def clear_of(self, geometry: SampleGeometry, margin: float) -> np.ndarray:
        """Whether every checked part of each sample keeps at least margin from every obstacle (the hard verdict).

        Same verdict as clearances_of(stacked_points(samples))[0].min(axis=1) >= margin (bounded_clearances to the
        margin, without the soft band): only capsules and point blocks whose bounding sphere comes within margin of an
        obstacle are evaluated, and only those blocks are placed point by point.

        Args:
            geometry (SampleGeometry): geometry(samples).
            margin (float): Required clearance (m).

        Returns:
            np.ndarray: (N,) bool, True where the sample is clear.
        """
        if not self.body.obstacles:
            return np.ones(geometry.n, dtype=bool)
        limit = np.full((geometry.n, len(self.body.obstacles)), margin)
        return self.bounded_clearances(geometry, limit, soft=False)[0].min(axis=1) >= margin

    def pose(self, joints: dict[str, float]) -> PoseClearance:
        """Clearance of one configuration to every obstacle (computed even when the check is disabled).

        Args:
            joints (dict[str, float]): Joint configuration.

        Returns:
            PoseClearance: Per obstacle (clearance, closest part).
        """
        values, parts, soft, _soft_parts = self.clearance_matrix([joints])
        obstacles = self.body.obstacles
        return PoseClearance(
            {o.name: (float(values[0, k]), CHECKED_PARTS[int(parts[0, k])]) for k, o in enumerate(obstacles)},
            {o.name: float(soft[0, k]) for k, o in enumerate(obstacles)},
        )

    def closest(self, samples: list[dict[str, float]]) -> list[tuple[float, str, str]]:
        """Smallest hard clearance of every configuration with its obstacle and part, pruned (bounded_clearances).

        Per sample the same as pose(): (min_clearance, obstacle, part), computed even when the check is disabled.

        Args:
            samples (list[dict[str, float]]): Joint configurations.

        Returns:
            list[tuple[float, str, str]]: (clearance m, obstacle, part) per sample; (inf, '', '') without obstacles.
        """
        if not samples or not self.body.obstacles:
            return [(math.inf, "", "")] * len(samples)
        values, parts, _soft, _soft_parts = self.bounded_clearances(self.geometry(samples))
        worst = values.argmin(axis=1)
        names = [o.name for o in self.body.obstacles]
        return [(float(values[i, k]), names[int(k)], CHECKED_PARTS[int(parts[i, k])]) for i, k in enumerate(worst)]

    def path(
        self,
        samples: list[dict[str, float]],
        allow_escape: bool = True,
        prune: bool = True,
        moving_parts: tuple[str, ...] | None = None,
    ) -> BodyReport:
        """Speed scale of every sample and the hard verdict of a trajectory (samples[0] = the start pose).

        Runs in the C++ kernel (BodyChecker.path). With prune (bounding-sphere pruning: the start exact below the hard
        margin, then the rest to its limits) the report is the same as with the full clearances, field for field.
        With moving_parts (a gripper-only motion: GRIPPER_MOVING_PARTS) the soft band (scales, clearances, obstacles,
        parts) covers only those parts: the clearance of a link that does not move cannot change during the motion, so
        it never slows it. The hard verdict still covers every part.

        Args:
            samples (list[dict[str, float]]): Joint samples the arm will follow, start first.
            allow_escape (bool): A start inside the hard margin may move out (the clearance must not get worse);
                False holds every sample to the hard margin (the planner's own waypoints).
            prune (bool): Bounding-sphere pruning (False: every distance in full, the reference the tests compare).
            moving_parts (tuple[str, ...] | None): Single-point parts (JAW_POINTS) that move; None = every part moves.

        Returns:
            BodyReport: Scales, clearances, limiting obstacle/part per sample and the verdict.

        Raises:
            ValueError: When moving_parts names a part that is not a single jaw point.
        """
        if moving_parts is not None and not set(moving_parts) <= set(JAW_POINTS):
            raise ValueError(f"moving_parts must be jaw points {JAW_POINTS}, got {moving_parts}")
        cfg = self.settings
        n = len(samples)
        if not cfg.enabled or n == 0:
            return BodyReport([NORMAL_SCALE] * n, [math.inf] * n, [""] * n, [""] * n, cfg, enabled=False)
        if not self.body.obstacles:
            return BodyReport([NORMAL_SCALE] * n, [math.inf] * n, [""] * n, [""] * n, cfg)
        moving = [CHECKED_PARTS.index(part) for part in moving_parts or ()]
        clearances, worst, soft_parts, violation, violation_obstacle, violation_part, violation_clearance, *counts = (
            self.core.path(*self.geometry(samples).arrays(), cfg.hard_margin_m, allow_escape, prune, moving)
        )
        self.distance_counts["points"] += counts[0]
        self.distance_counts["capsules"] += counts[1]
        names = [o.name for o in self.body.obstacles]
        report = BodyReport(
            scales=[cfg.slow_speed_scale if c < cfg.soft_margin_m else NORMAL_SCALE for c in clearances.tolist()],
            clearances=clearances.tolist(),
            obstacles=[names[k] for k in worst.tolist()],
            parts=[CHECKED_PARTS[p] for p in soft_parts.tolist()],
            settings=cfg,
            hints={o.name: obstacle_hint(o, "arm", cfg) for o in self.body.obstacles},
        )
        if violation >= 0:
            part = CHECKED_PARTS[violation_part]
            report.violation = violation
            report.violation_obstacle = names[violation_obstacle]
            report.violation_part = part
            report.violation_clearance = violation_clearance
            report.hints[names[violation_obstacle]] = obstacle_hint(self.body.obstacles[violation_obstacle], part, cfg)
        return report
