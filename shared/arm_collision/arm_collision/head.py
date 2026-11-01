"""The pan/tilt head as a posed body part: forward kinematics of head.urdf at the measured pan/tilt, no ROS.

The packaged body (robot_body.yaml) holds the head as its full envelope, the cylinder `head` that holds the head at
every pan/tilt. With a measured head pose (head_pose: both joints known, finite and fresh) the checks use the posed
head instead: the axis-aligned box (base_link) around the pan base, the pan link, the bracket (head_link) and the D435
(d435_link) visuals of head.urdf at that pan/tilt, rounded outwards to 1 mm (HeadModel.box, posed_body). A missing or
stale head pose keeps the full envelope (fail closed).
"""

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .settings import REPO_ROOT, BodyBox, BodySettings

HEAD_URDF = REPO_ROOT / "nodes" / "web_ui" / "urdf" / "head.urdf"
HEAD_PAN = "head_pan"
HEAD_TILT = "head_tilt"
# The head links the posed box holds: the pan base under the pan servo and every link the pan or tilt moves.
HEAD_LINKS = ("head_pan_base", "head_pan_link", "head_link", "d435_link")
# Name of the head obstacle in the body: the full envelope cylinder, or the posed box replacing it.
HEAD_OBSTACLE = "head"
ROOT_LINK = "base_link"
ROUND_OUT_M = 0.001
# Shaves float noise off a bound before rounding it outwards, so an exact millimetre stays that millimetre.
ROUND_EPS_M = 1e-9


@dataclass(frozen=True)
class HeadPose:
    """Measured head joint angles (rad, head.urdf conventions: pan positive left, tilt positive down)."""

    pan: float
    tilt: float


@dataclass(frozen=True)
class UrdfJoint:
    """One head.urdf joint: parent link, origin transform and the axis of a revolute joint (None when fixed)."""

    name: str
    parent: str
    origin: np.ndarray
    axis: np.ndarray | None


@dataclass(frozen=True)
class Visual:
    """One visual primitive in its link frame: a box (size) or a cylinder along the local z (radius, length)."""

    origin: np.ndarray
    box: tuple[float, float, float] | None
    cylinder: tuple[float, float] | None


def head_pose(angles: dict[str, float] | None) -> HeadPose | None:
    """The head pose of measured joint angles, None unless both head joints are present and finite.

    Args:
        angles (dict[str, float] | None): Joint name -> rad (a fresh head joint state), None when missing or stale.

    Returns:
        HeadPose | None: The pose, or None (the checks then use the full head envelope).
    """
    if angles is None or HEAD_PAN not in angles or HEAD_TILT not in angles:
        return None
    pan, tilt = float(angles[HEAD_PAN]), float(angles[HEAD_TILT])
    if not (math.isfinite(pan) and math.isfinite(tilt)):
        return None
    return HeadPose(pan=pan, tilt=tilt)


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF fixed-axis roll/pitch/yaw rotation (Rz(yaw) Ry(pitch) Rx(roll)).

    Args:
        roll (float): About x (rad).
        pitch (float): About y (rad).
        yaw (float): About z (rad).

    Returns:
        np.ndarray: (3, 3) rotation.
    """
    cr, sr, cp, sp, cy, sy = (
        math.cos(roll),
        math.sin(roll),
        math.cos(pitch),
        math.sin(pitch),
        math.cos(yaw),
        math.sin(yaw),
    )
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rotation about a unit axis (Rodrigues).

    Args:
        axis (np.ndarray): (3,) unit axis.
        angle (float): Angle (rad).

    Returns:
        np.ndarray: (3, 3) rotation.
    """
    k = np.array([[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


def origin_transform(element: ET.Element | None) -> np.ndarray:
    """The 4x4 transform of a URDF <origin> element (identity when absent).

    Args:
        element (ET.Element | None): The origin element.

    Returns:
        np.ndarray: (4, 4) transform.
    """
    transform = np.eye(4)
    if element is not None:
        xyz = [float(v) for v in element.get("xyz", "0 0 0").split()]
        rpy = [float(v) for v in element.get("rpy", "0 0 0").split()]
        transform[:3, :3] = rpy_matrix(*rpy)
        transform[:3, 3] = xyz
    return transform


def round_out(lo: np.ndarray, hi: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Bounds rounded outwards to ROUND_OUT_M.

    Args:
        lo (np.ndarray): (3,) lower corner (m).
        hi (np.ndarray): (3,) upper corner (m).

    Returns:
        tuple[np.ndarray, np.ndarray]: (lo, hi) on the ROUND_OUT_M grid, holding the input box.
    """
    return (
        np.floor((lo + ROUND_EPS_M) / ROUND_OUT_M) * ROUND_OUT_M,
        np.ceil((hi - ROUND_EPS_M) / ROUND_OUT_M) * ROUND_OUT_M,
    )


class HeadModel:
    """Head forward kinematics and visuals of head.urdf (the HEAD_LINKS chain from base_link)."""

    def __init__(self, joints: dict[str, UrdfJoint], visuals: dict[str, list[Visual]]) -> None:
        """Hold the parsed chain.

        Args:
            joints (dict[str, UrdfJoint]): Child link -> the joint above it.
            visuals (dict[str, list[Visual]]): Link -> its visual primitives (HEAD_LINKS only).
        """
        self.joints = joints
        self.visuals = visuals

    @classmethod
    def from_urdf(cls, path: Path = HEAD_URDF) -> "HeadModel":
        """Parse the head joints and the HEAD_LINKS visuals of a head URDF.

        Args:
            path (Path): Head URDF (default the repo's head.urdf).

        Returns:
            HeadModel: The model.

        Raises:
            ValueError: When a head link has a visual that is neither a box nor a cylinder.
        """
        root = ET.fromstring(path.read_text())
        joints: dict[str, UrdfJoint] = {}
        for joint in root.iter("joint"):
            axis = joint.find("axis")
            revolute = joint.get("type") in ("revolute", "continuous")
            unit = (
                np.array([float(v) for v in axis.get("xyz", "1 0 0").split()])
                if axis is not None
                else np.array([1.0, 0.0, 0.0])
            )
            joints[joint.find("child").get("link")] = UrdfJoint(
                name=joint.get("name"),
                parent=joint.find("parent").get("link"),
                origin=origin_transform(joint.find("origin")),
                axis=unit / np.linalg.norm(unit) if revolute else None,
            )
        visuals: dict[str, list[Visual]] = {}
        for link in root.iter("link"):
            if link.get("name") not in HEAD_LINKS:
                continue
            parts = []
            for visual in link.iter("visual"):
                geometry = visual.find("geometry")
                box, cylinder = geometry.find("box"), geometry.find("cylinder")
                if box is None and cylinder is None:
                    raise ValueError(f"head link {link.get('name')}: only box and cylinder visuals are modelled")
                parts.append(
                    Visual(
                        origin=origin_transform(visual.find("origin")),
                        box=tuple(float(v) for v in box.get("size").split()) if box is not None else None,
                        cylinder=(float(cylinder.get("radius")), float(cylinder.get("length")))
                        if cylinder is not None
                        else None,
                    )
                )
            visuals[link.get("name")] = parts
        return cls(joints, visuals)

    def link_transform(self, link: str, pose: HeadPose) -> np.ndarray:
        """Pose of a link in base_link at a head pose.

        Args:
            link (str): Link name.
            pose (HeadPose): Pan/tilt.

        Returns:
            np.ndarray: (4, 4) transform base_link <- link.
        """
        angles = {HEAD_PAN: pose.pan, HEAD_TILT: pose.tilt}
        transform = np.eye(4)
        while link != ROOT_LINK:
            joint = self.joints[link]
            step = joint.origin.copy()
            if joint.axis is not None:
                rotation = np.eye(4)
                rotation[:3, :3] = axis_rotation(joint.axis, angles.get(joint.name, 0.0))
                step = step @ rotation
            transform = step @ transform
            link = joint.parent
        return transform

    def box(self, pose: HeadPose) -> BodyBox:
        """The posed head: the base_link axis-aligned box around every HEAD_LINKS visual, rounded outwards to 1 mm.

        Args:
            pose (HeadPose): Measured pan/tilt.

        Returns:
            BodyBox: Box named HEAD_OBSTACLE.
        """
        lo, hi = np.full(3, math.inf), np.full(3, -math.inf)
        for link, parts in self.visuals.items():
            frame = self.link_transform(link, pose)
            for part in parts:
                placed = frame @ part.origin
                rotation, centre = placed[:3, :3], placed[:3, 3]
                if part.box is not None:
                    half = np.abs(rotation) @ (np.array(part.box) / 2.0)
                else:
                    radius, length = part.cylinder
                    axis = rotation[:, 2]
                    half = np.abs(axis) * length / 2.0 + radius * np.sqrt(np.clip(1.0 - axis**2, 0.0, 1.0))
                lo, hi = np.minimum(lo, centre - half), np.maximum(hi, centre + half)
        lo, hi = round_out(lo, hi)
        return BodyBox(
            name=HEAD_OBSTACLE,
            center=tuple(float(v) for v in np.round((lo + hi) / 2.0, 6)),
            size=tuple(float(v) for v in np.round(hi - lo, 6)),
        )


def posed_body(body: BodySettings, head: BodyBox | None) -> BodySettings:
    """The body with its head envelope replaced by a posed head box; None keeps the full envelope.

    Args:
        body (BodySettings): Body holding the head envelope cylinder HEAD_OBSTACLE (load_robot_body).
        head (BodyBox | None): Posed head (HeadModel.box), None without a fresh head pose.

    Returns:
        BodySettings: The body to check against.
    """
    if head is None:
        return body
    return BodySettings(
        boxes=[*(b for b in body.boxes if b.name != HEAD_OBSTACLE), head],
        cylinders=[c for c in body.cylinders if c.name != HEAD_OBSTACLE],
    )
