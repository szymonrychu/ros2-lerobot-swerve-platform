"""SO101 arm kinematics on the repo URDF via ikpy: 5-DOF chain, position + approach pitch IK, URDF joint limits."""

import math
import warnings
import xml.etree.ElementTree as ET
from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path

import ikpy.chain
import numpy as np

BASE_LINK = "base_link"
ARM_CHAIN_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
# Joints the IK solves; wrist_roll spins the gripper about its approach axis and is kept from the seed.
IK_ACTIVE_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")
# The tool frame (gripper_frame_link) approach direction is its local z axis (forward/horizontal at zero pose).
APPROACH_AXIS = "Z"
POSITION_TOLERANCE_M = 0.002
PITCH_TOLERANCE_RAD = math.radians(3.0)
# Extra IK seeds (shoulder_lift, elbow_flex, wrist_flex) tried after the caller's seed.
HEADING_REFINEMENTS = 6
# Fixed-point iterations / convergence (m) of the IK on the flange target when the tool point is offset.
TOOL_ITERATIONS = 6
SOLUTION_CACHE_SIZE = 8192  # ikpy solves kept (identical queries repeat across seeds, refinements and strategies)
TOOL_CONVERGENCE_M = 0.0005
EXTRA_SEEDS = ((0.0, 0.0, 0.0), (-0.8, 0.8, 0.5), (0.5, -0.5, 1.0), (-1.2, 1.4, 0.0), (0.8, 0.8, -0.5))


class UnreachableError(ValueError):
    """Raised when no joint configuration within limits reaches the requested target."""


@dataclass(frozen=True)
class CartesianPose:
    """Tool point pose in the arm base_link frame; pitch > 0 means the gripper points below horizontal."""

    x: float
    y: float
    z: float
    pitch: float


def load_joint_limits(urdf_path: Path) -> dict[str, tuple[float, float]]:
    """Read revolute joint limits from a URDF.

    Args:
        urdf_path (Path): URDF file.

    Returns:
        dict[str, tuple[float, float]]: Joint name -> (lower, upper) in rad.
    """
    root = ET.parse(urdf_path).getroot()
    limits: dict[str, tuple[float, float]] = {}
    for joint in root.iter("joint"):
        limit = joint.find("limit")
        if joint.get("type") in ("revolute", "prismatic") and limit is not None:
            limits[joint.attrib["name"]] = (float(limit.attrib["lower"]), float(limit.attrib["upper"]))
    return limits


def grasp_offset(object_width_m: float, jaw_open_axis: tuple[float, float, float]) -> tuple[float, float, float]:
    """Extra tool offset that makes the controlled point the centre between the jaws of an object.

    The configured tool point is the fixed jaw's inner face; the object centre lies half its width away in the
    direction the moving jaw opens.

    Args:
        object_width_m (float): Object width across the jaws (m).
        jaw_open_axis (tuple[float, float, float]): Unit opening direction in gripper_frame_link.

    Returns:
        tuple[float, float, float]: Offset (m) in gripper_frame_link, added to the configured tool offset.
    """
    half = object_width_m / 2.0
    return (half * jaw_open_axis[0], half * jaw_open_axis[1], half * jaw_open_axis[2])


def pitch_of(approach: np.ndarray) -> float:
    """Pitch of an approach vector: angle below the horizontal plane.

    Args:
        approach (np.ndarray): 3-vector.

    Returns:
        float: Pitch in rad (positive = pointing down).
    """
    return math.atan2(-float(approach[2]), math.hypot(float(approach[0]), float(approach[1])))


class ArmKinematics:
    """Forward and inverse kinematics of the 5-DOF SO101 chain base_link -> gripper_frame_link.

    Public joint maps (``forward``, ``link_frame``, the ``inverse`` seed and result) are in MEASURED follower space.
    The single place that maps to the URDF model is ``to_urdf``: urdf_angle = measured_angle + offset. Limits, ikpy
    and the FK itself work in URDF space.
    """

    def __init__(
        self,
        urdf_path: Path,
        margin: float,
        joint_offsets: dict[str, float] | None = None,
        tool_offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
        limit_overrides: dict[str, tuple[float, float]] | None = None,
    ) -> None:
        """Build the ikpy chain and shrink the joint bounds by margin.

        Args:
            urdf_path (Path): Arm URDF.
            margin (float): Safety margin kept from each URDF limit (rad).
            joint_offsets (dict[str, float] | None): Joint zero offsets (rad) of the chain joints,
                urdf_angle = measured_angle + offset; missing joints and None mean 0.
            tool_offset (tuple[float, float, float]): Tool centre point (jaw closing point) relative to
                gripper_frame_link, expressed in that frame (m). Forward kinematics reports this point and the
                inverse places it on the target.
            limit_overrides (dict[str, tuple[float, float]] | None): Replacement (lower, upper) URDF-space limits
                (rad) of chain joints; used for the IK bounds, within_limits and clamp_seed.

        Raises:
            ValueError: For an offset on a joint that is not a chain joint (e.g. the gripper), or a limit override
                on a non-chain joint or with lower >= upper.
        """
        all_limits = load_joint_limits(urdf_path)
        self.limits = {j: all_limits[j] for j in ARM_CHAIN_JOINTS}
        for name, (lo, hi) in (limit_overrides or {}).items():
            if name not in self.limits:
                raise ValueError(f"limit overrides only apply to {list(ARM_CHAIN_JOINTS)}, not {name!r}")
            if not lo < hi:
                raise ValueError(f"limit override for {name}: lower {lo} must be below upper {hi}")
            self.limits[name] = (float(lo), float(hi))
        self.margin = margin
        self.tool_offset = np.array(tool_offset, dtype=np.float64)
        unknown = sorted(set(joint_offsets or {}) - set(ARM_CHAIN_JOINTS))
        if unknown:
            raise ValueError(f"joint offsets only apply to {list(ARM_CHAIN_JOINTS)}, not {unknown}")
        self.offsets = {j: float((joint_offsets or {}).get(j, 0.0)) for j in ARM_CHAIN_JOINTS}
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            chain = ikpy.chain.Chain.from_urdf_file(str(urdf_path), base_elements=[BASE_LINK])
        names = [link.name for link in chain.links]
        missing = [j for j in ARM_CHAIN_JOINTS if j not in names]
        if missing:
            raise ValueError(f"URDF chain from {BASE_LINK} lacks joints {missing}")
        self.index = {j: names.index(j) for j in ARM_CHAIN_JOINTS}
        mask = [link.name in IK_ACTIVE_JOINTS for link in chain.links]
        for j in ARM_CHAIN_JOINTS:
            lo, hi = self.limits[j]
            chain.links[self.index[j]].bounds = (lo + margin, hi - margin)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self.chain = ikpy.chain.Chain(chain.links, active_links_mask=mask, name="so101")
        self.joint_names = ARM_CHAIN_JOINTS
        # A tool point can never be farther from the arm base than the sum of the link offsets (triangle inequality).
        self.max_reach_m = float(
            sum(np.linalg.norm(getattr(link, "origin_translation", (0.0, 0.0, 0.0))) for link in chain.links)
        )
        self.solutions: OrderedDict[bytes, dict[str, float]] = OrderedDict()
        pan_origin = self.chain.forward_kinematics(np.zeros(len(self.chain.links)), full_kinematics=True)[
            self.index["shoulder_pan"]
        ]
        self.pan_axis_xy = (float(pan_origin[0, 3]), float(pan_origin[1, 3]))
        # URDF link name -> index of its frame in ikpy's full kinematics (a chain link is named after its joint, its
        # frame is the child link's frame). Links off the chain (the moving jaw) have no entry.
        self.link_index = {BASE_LINK: 0}
        for joint in ET.parse(urdf_path).getroot().iter("joint"):
            child = joint.find("child")
            if child is not None and joint.attrib["name"] in names:
                self.link_index[child.attrib["link"]] = names.index(joint.attrib["name"])

    def to_urdf(self, joints: dict[str, float]) -> dict[str, float]:
        """Measured joint angles -> URDF model angles (urdf = measured + offset); other joints pass through.

        Args:
            joints (dict[str, float]): Joint name -> measured rad.

        Returns:
            dict[str, float]: Joint name -> URDF rad.
        """
        return {j: v + self.offsets.get(j, 0.0) for j, v in joints.items()}

    def to_measured(self, joints: dict[str, float]) -> dict[str, float]:
        """URDF model angles -> measured joint angles (measured = urdf - offset); other joints pass through.

        Args:
            joints (dict[str, float]): Joint name -> URDF rad.

        Returns:
            dict[str, float]: Joint name -> measured rad.
        """
        return {j: v - self.offsets.get(j, 0.0) for j, v in joints.items()}

    def to_vector(self, joints: dict[str, float]) -> np.ndarray:
        """Convert a measured joint map into ikpy's full link vector (URDF space).

        Args:
            joints (dict[str, float]): Joint name -> measured rad (missing chain joints default to 0 measured).

        Returns:
            np.ndarray: Link vector in URDF angles.
        """
        return self.urdf_vector(self.to_urdf({j: joints.get(j, 0.0) for j in ARM_CHAIN_JOINTS}))

    def urdf_vector(self, joints: dict[str, float]) -> np.ndarray:
        """Convert a URDF-angle joint map into ikpy's full link vector.

        Args:
            joints (dict[str, float]): Joint name -> URDF rad (missing chain joints default to 0).

        Returns:
            np.ndarray: Link vector.
        """
        q = np.zeros(len(self.chain.links))
        for j in ARM_CHAIN_JOINTS:
            q[self.index[j]] = joints.get(j, 0.0)
        return q

    def forward(
        self, joints: dict[str, float], extra_offset: tuple[float, float, float] | None = None
    ) -> CartesianPose:
        """Tool pose for a joint configuration.

        Args:
            joints (dict[str, float]): Joint name -> measured rad.
            extra_offset (tuple[float, float, float] | None): Extra tool offset (m, gripper_frame_link axes) added to
                the configured one for this call only (e.g. grasp_offset); None for the configured tool point.

        Returns:
            CartesianPose: Tool position and approach pitch in base_link.
        """
        return self.forward_urdf(self.to_urdf(joints), extra_offset)

    def forward_urdf(
        self, joints: dict[str, float], extra_offset: tuple[float, float, float] | None = None
    ) -> CartesianPose:
        """Tool pose for URDF-space joint angles.

        Args:
            joints (dict[str, float]): Joint name -> URDF rad.
            extra_offset (tuple[float, float, float] | None): Extra tool offset for this call only (see forward).

        Returns:
            CartesianPose: Tool position and approach pitch in base_link.
        """
        frame = self.chain.forward_kinematics(self.urdf_vector(joints))
        offset = self.tool_offset if extra_offset is None else self.tool_offset + np.array(extra_offset)
        point = frame[:3, 3] + frame[:3, :3] @ offset
        return CartesianPose(x=float(point[0]), y=float(point[1]), z=float(point[2]), pitch=pitch_of(frame[:3, 2]))

    def link_frame(self, joints: dict[str, float], link: str) -> np.ndarray:
        """Pose of a URDF link frame in the arm base_link frame for a joint configuration.

        Args:
            joints (dict[str, float]): Joint name -> measured rad (missing chain joints default to 0).
            link (str): URDF link name on the base -> gripper_frame_link chain (e.g. gripper_link).

        Returns:
            np.ndarray: 4x4 transform T_arm_base_link.

        Raises:
            ValueError: When the link is not on the kinematic chain.
        """
        if link not in self.link_index:
            raise ValueError(f"link {link!r} is not on the arm chain; use one of {sorted(self.link_index)}")
        return self.link_frames(joints)[link]

    def link_frames(self, joints: dict[str, float]) -> dict[str, np.ndarray]:
        """Poses of every URDF link frame on the chain from a single forward kinematics pass.

        Args:
            joints (dict[str, float]): Joint name -> measured rad (missing chain joints default to 0).

        Returns:
            dict[str, np.ndarray]: Link name -> 4x4 transform T_arm_base_link.
        """
        frames = self.chain.forward_kinematics(self.to_vector(joints), full_kinematics=True)
        return {link: np.array(frames[i], dtype=np.float64) for link, i in self.link_index.items()}

    def approach_vector(self, heading: float, pitch: float) -> np.ndarray:
        """Approach direction with the given horizontal heading, pitched down by pitch.

        Args:
            heading (float): Horizontal heading of the approach in base_link (rad).
            pitch (float): Approach pitch (rad, positive down).

        Returns:
            np.ndarray: Unit 3-vector.
        """
        return np.array([math.cos(pitch) * math.cos(heading), math.cos(pitch) * math.sin(heading), -math.sin(pitch)])

    def within_limits(self, joints: dict[str, float]) -> bool:
        """Whether every chain joint lies within its limits minus margin.

        Args:
            joints (dict[str, float]): Joint name -> URDF rad (limits are URDF values).

        Returns:
            bool: True when inside.
        """
        eps = 1e-9
        return all(
            self.limits[j][0] + self.margin - eps <= joints[j] <= self.limits[j][1] - self.margin + eps
            for j in ARM_CHAIN_JOINTS
        )

    def inverse(
        self,
        x: float,
        y: float,
        z: float,
        pitch: float | None,
        seed: dict[str, float],
        extra_offset: tuple[float, float, float] | None = None,
    ) -> dict[str, float]:
        """Joint configuration placing the tool point at (x, y, z), optionally with the given approach pitch.

        With a tool offset the gripper_frame_link target is target - R_tool @ offset; R_tool depends on the solution,
        so it is iterated: the flange target is shifted by the tool point residual until it is within
        TOOL_CONVERGENCE_M.

        Args:
            x (float): Target x in base_link (m).
            y (float): Target y in base_link (m).
            z (float): Target z in base_link (m).
            pitch (float | None): Approach pitch (rad, positive down), or None for position only.
            seed (dict[str, float]): Starting configuration in measured space; wrist_roll is kept.
            extra_offset (tuple[float, float, float] | None): Extra tool offset (m, gripper_frame_link axes) added to
                the configured one for this solve only; the shared tool offset is not modified.

        Returns:
            dict[str, float]: Chain joint name -> measured rad.

        Raises:
            UnreachableError: When the target cannot be reached within limits and tolerances.
        """
        tool_reach = float(np.linalg.norm(self.tool_offset + (0.0 if extra_offset is None else np.array(extra_offset))))
        if math.sqrt(x * x + y * y + z * z) > self.max_reach_m + tool_reach + POSITION_TOLERANCE_M:
            raise UnreachableError(
                f"target ({x:.3f}, {y:.3f}, {z:.3f}) is beyond the arm's reach of {self.max_reach_m + tool_reach:.3f} m"
            )
        if not (self.tool_offset.any() or (extra_offset is not None and any(extra_offset))):
            return self.inverse_flange(x, y, z, pitch, seed)
        target = np.array([x, y, z])
        flange = target.copy()
        sol = self.inverse_flange(*flange, pitch, seed)
        for _ in range(TOOL_ITERATIONS):
            reached = self.forward(sol, extra_offset)
            residual = target - np.array([reached.x, reached.y, reached.z])
            if float(np.linalg.norm(residual)) < TOOL_CONVERGENCE_M:
                break
            flange = flange + residual
            sol = self.inverse_flange(*flange, pitch, sol)
        reached = self.forward(sol, extra_offset)
        if math.dist((reached.x, reached.y, reached.z), (x, y, z)) > POSITION_TOLERANCE_M:
            raise UnreachableError(
                f"target ({x:.3f}, {y:.3f}, {z:.3f}) tool point did not converge within joint limits"
            )
        return sol

    def inverse_flange(
        self, x: float, y: float, z: float, pitch: float | None, seed: dict[str, float]
    ) -> dict[str, float]:
        """Joint configuration placing gripper_frame_link at (x, y, z), optionally with the given approach pitch.

        Tries the seed first, then a few fixed seeds, and accepts a solution only when forward kinematics confirms it
        within POSITION_TOLERANCE_M / PITCH_TOLERANCE_RAD and inside the limits; it never returns a best guess.

        Args:
            x (float): Target x in base_link (m).
            y (float): Target y in base_link (m).
            z (float): Target z in base_link (m).
            pitch (float | None): Approach pitch (rad, positive down), or None for position only.
            seed (dict[str, float]): Starting configuration in measured space (typically the measured pose);
                wrist_roll is kept.

        Returns:
            dict[str, float]: Chain joint name -> measured rad (command space: urdf angle - offset).

        Raises:
            UnreachableError: When the target cannot be reached within limits and tolerances.
        """
        values = (x, y, z) if pitch is None else (x, y, z, pitch)
        if not all(math.isfinite(v) for v in values):
            raise UnreachableError("target must be finite")
        seed = self.to_urdf(seed)
        roll = min(
            max(seed.get("wrist_roll", 0.0), self.limits["wrist_roll"][0] + self.margin),
            self.limits["wrist_roll"][1] - self.margin,
        )
        target = np.array([x, y, z])
        heading = math.atan2(y - self.pan_axis_xy[1], x - self.pan_axis_xy[0])
        seeds = [dict(seed)]
        for lift, elbow, wrist in EXTRA_SEEDS:
            seeds.append({"shoulder_pan": -heading, "shoulder_lift": lift, "elbow_flex": elbow, "wrist_flex": wrist})
        best_error = math.inf
        for candidate in seeds:
            sol = self.clamp_seed(candidate | {"wrist_roll": roll})
            # The arm plane heading depends on the solution (link offsets, wrist_roll), so refine it from each solve.
            for _ in range(HEADING_REFINEMENTS if pitch is not None else 1):
                orientation = None if pitch is None else self.approach_vector(heading, pitch)
                sol = self.solve(target, orientation, sol, roll)
                frame = self.chain.forward_kinematics(self.urdf_vector(sol))
                approach = frame[:3, 2]
                if math.hypot(float(approach[0]), float(approach[1])) > 1e-6:
                    heading = math.atan2(float(approach[1]), float(approach[0]))
                error = math.dist(tuple(frame[:3, 3]), (x, y, z))
                reached = self.forward_urdf(sol)
                best_error = min(best_error, error)
                pitch_ok = pitch is None or abs(reached.pitch - pitch) <= PITCH_TOLERANCE_RAD
                if error <= POSITION_TOLERANCE_M and pitch_ok and self.within_limits(sol):
                    return self.to_measured(sol)
        raise UnreachableError(
            f"target ({x:.3f}, {y:.3f}, {z:.3f})"
            + ("" if pitch is None else f" pitch {pitch:.2f} rad")
            + f" is unreachable within joint limits (closest {best_error * 1000:.0f} mm)"
        )

    def solve(
        self, target: np.ndarray, orientation: np.ndarray | None, start: dict[str, float], roll: float
    ) -> dict[str, float]:
        """One ikpy solve from a start configuration.

        Args:
            target (np.ndarray): Target position.
            orientation (np.ndarray | None): Approach vector, or None for position only.
            start (dict[str, float]): Start configuration in URDF angles (inside bounds).
            roll (float): wrist_roll kept fixed.

        Returns:
            dict[str, float]: Solved chain joints (URDF angles).
        """
        initial = self.urdf_vector(self.clamp_seed(start))
        key = b"".join(
            np.asarray(part, dtype=np.float64).tobytes()
            for part in (target, np.zeros(0) if orientation is None else orientation, initial, (roll,))
        ) + (b"p" if orientation is None else b"o")
        cached = self.solutions.get(key)
        if cached is not None:
            self.solutions.move_to_end(key)
            return dict(cached)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            q = self.chain.inverse_kinematics(
                target,
                orientation,
                orientation_mode=None if orientation is None else APPROACH_AXIS,
                initial_position=initial,
            )
        sol = {j: float(q[self.index[j]]) for j in ARM_CHAIN_JOINTS}
        sol["wrist_roll"] = roll
        self.solutions[key] = dict(sol)
        if len(self.solutions) > SOLUTION_CACHE_SIZE:
            self.solutions.popitem(last=False)
        return sol

    def clamp_seed(self, joints: dict[str, float]) -> dict[str, float]:
        """Clamp a seed into the IK bounds (ikpy refuses seeds outside them).

        Args:
            joints (dict[str, float]): Seed configuration.

        Returns:
            dict[str, float]: Clamped seed with every chain joint present.
        """
        return {
            j: min(max(joints.get(j, 0.0), self.limits[j][0] + self.margin), self.limits[j][1] - self.margin)
            for j in ARM_CHAIN_JOINTS
        }
