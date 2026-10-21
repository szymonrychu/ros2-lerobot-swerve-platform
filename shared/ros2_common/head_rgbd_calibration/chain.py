"""Forward kinematics over URDF text: the chain shared by the head fit, the arm fit and the arm pose sequence."""

import xml.etree.ElementTree as ET
from collections.abc import Mapping
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from ros2_common.head_rgbd_calibration.urdf_handoff import RPY_SEQUENCE

FIXED = "fixed"
MOVABLE_TYPES = ("revolute", "continuous")
SUPPORTED_TYPES = (FIXED, *MOVABLE_TYPES)

Vec3 = tuple[float, float, float]
XyzRpy = tuple[Vec3, Vec3]


@dataclass(frozen=True)
class ChainJoint:
    """One URDF joint: its tree position, origin, axis and limits."""

    name: str
    type: str
    parent: str
    child: str
    xyz: Vec3
    rpy: Vec3
    axis: Vec3
    limits: tuple[float, float] | None

    @property
    def movable(self) -> bool:
        """True for revolute and continuous joints."""
        return self.type in MOVABLE_TYPES


def parse_vec(text: str | None, default: Vec3) -> Vec3:
    """Parse a whitespace separated 3-vector attribute; `default` when absent."""
    if text is None:
        return default
    x, y, z = (float(v) for v in text.split())
    return (x, y, z)


def pose_from_xyz_rpy(xyz: Vec3, rpy: Vec3) -> np.ndarray:
    """4x4 transform of a URDF origin (fixed-axis xyz rotation).

    Args:
        xyz: Translation in metres.
        rpy: Roll, pitch, yaw in radians.

    Returns:
        np.ndarray: 4x4 homogeneous transform.
    """
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler(RPY_SEQUENCE, rpy).as_matrix()
    pose[:3, 3] = xyz
    return pose


def xyz_rpy_from_pose(pose: np.ndarray) -> XyzRpy:
    """Inverse of `pose_from_xyz_rpy`.

    Args:
        pose: 4x4 homogeneous transform.

    Returns:
        tuple[xyz, rpy]: Translation (m) and fixed-axis xyz angles (rad).
    """
    x, y, z = (float(v) for v in pose[:3, 3])
    r, p, w = (float(v) for v in Rotation.from_matrix(pose[:3, :3]).as_euler(RPY_SEQUENCE))
    return (x, y, z), (r, p, w)


def arm_in_robot_base(base: Mapping[str, float]) -> np.ndarray:
    """Transform from the arm URDF root to the robot base_link.

    Args:
        base: Mapping with keys `x`, `y`, `z` (m) and `yaw` (rad).

    Returns:
        np.ndarray: 4x4 pose of the arm root in robot base_link.
    """
    return pose_from_xyz_rpy((base["x"], base["y"], base["z"]), (0.0, 0.0, base["yaw"]))


class UrdfChain:
    """Kinematic tree of a URDF, evaluated with numpy."""

    def __init__(self, joints: list[ChainJoint]) -> None:
        """Index the joints by name and by child link.

        Args:
            joints: Parsed joints of one URDF.
        """
        self.joints = {j.name: j for j in joints}
        self.by_child = {j.child: j for j in joints}
        self.links = {j.parent for j in joints} | set(self.by_child)

    @classmethod
    def from_text(cls, urdf_text: str) -> "UrdfChain":
        """Parse URDF text.

        Args:
            urdf_text: The URDF XML.

        Returns:
            UrdfChain: The parsed tree.

        Raises:
            ValueError: A joint has a type other than fixed, revolute or continuous.
        """
        joints = []
        for el in ET.fromstring(urdf_text).findall("joint"):
            name = el.get("name", "")
            kind = el.get("type", "")
            if kind not in SUPPORTED_TYPES:
                raise ValueError(f"joint {name!r} has unsupported type {kind!r}")
            origin = el.find("origin")
            axis = el.find("axis")
            limit = el.find("limit")
            limits = None
            if kind == "revolute" and limit is not None:
                limits = (float(limit.get("lower", 0.0)), float(limit.get("upper", 0.0)))
            joints.append(
                ChainJoint(
                    name=name,
                    type=kind,
                    parent=el.find("parent").get("link"),
                    child=el.find("child").get("link"),
                    xyz=parse_vec(None if origin is None else origin.get("xyz"), (0.0, 0.0, 0.0)),
                    rpy=parse_vec(None if origin is None else origin.get("rpy"), (0.0, 0.0, 0.0)),
                    axis=parse_vec(None if axis is None else axis.get("xyz"), (1.0, 0.0, 0.0)),
                    limits=limits,
                )
            )
        return cls(joints)

    def path(self, link: str) -> list[ChainJoint]:
        """Joints from the root to `link`.

        Raises:
            KeyError: The link is not in the URDF.
        """
        if link not in self.links:
            raise KeyError(f"unknown link {link!r}")
        out: list[ChainJoint] = []
        while link in self.by_child:
            joint = self.by_child[link]
            out.append(joint)
            link = joint.parent
        return out[::-1]

    def joint_names(self, link: str) -> list[str]:
        """Joint names from the root to `link`."""
        return [j.name for j in self.path(link)]

    def origin(self, joint: str) -> XyzRpy:
        """URDF origin (xyz, rpy) of a joint."""
        j = self.joints[joint]
        return j.xyz, j.rpy

    def limits(self, joint: str) -> tuple[float, float] | None:
        """(lower, upper) of a revolute joint; None for fixed and continuous joints."""
        return self.joints[joint].limits

    def transform(
        self,
        link: str,
        positions: Mapping[str, float],
        origins: Mapping[str, XyzRpy] | None = None,
        offsets: Mapping[str, float] | None = None,
    ) -> np.ndarray:
        """Pose of `link` in the URDF root.

        Args:
            link: Target link name.
            positions: Measured angle (rad) of each movable joint on the path.
            origins: Per-joint replacement of the URDF origin (xyz, rpy); nothing else changes.
            offsets: Per-joint offset (rad); urdf_angle = measured + offset.

        Returns:
            np.ndarray: 4x4 pose.

        Raises:
            KeyError: Unknown link, or a movable joint on the path missing from `positions`.
        """
        origins = origins or {}
        offsets = offsets or {}
        pose = np.eye(4)
        for joint in self.path(link):
            xyz, rpy = origins.get(joint.name, (joint.xyz, joint.rpy))
            pose = pose @ pose_from_xyz_rpy(xyz, rpy)
            if joint.movable:
                if joint.name not in positions:
                    raise KeyError(f"no position for joint {joint.name!r}")
                angle = positions[joint.name] + offsets.get(joint.name, 0.0)
                axis = np.asarray(joint.axis, dtype=float)
                spin = np.eye(4)
                spin[:3, :3] = Rotation.from_rotvec(angle * axis / np.linalg.norm(axis)).as_matrix()
                pose = pose @ spin
        return pose
