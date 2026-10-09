"""Tiny numpy URDF forward-kinematics chain parser, used only by the tests as the FK reference."""

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np

FIXED = "fixed"


@dataclass(frozen=True)
class UrdfJoint:
    """One URDF joint: parent/child link names, origin transform and motion axis."""

    name: str
    kind: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF fixed-axis rpy to a 3x3 rotation matrix (R = Rz(yaw) Ry(pitch) Rx(roll)).

    Args:
        roll (float): Rotation about x in rad.
        pitch (float): Rotation about y in rad.
        yaw (float): Rotation about z in rad.

    Returns:
        np.ndarray: 3x3 rotation matrix.
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def axis_angle_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues rotation matrix.

    Args:
        axis (np.ndarray): Rotation axis (any length, normalised here).
        angle (float): Angle in rad.

    Returns:
        np.ndarray: 3x3 rotation matrix.
    """
    a = axis / np.linalg.norm(axis)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * (k @ k)


def load_urdf(path: Path) -> dict[str, UrdfJoint]:
    """Parse the joints of a URDF, keyed by child link name.

    Args:
        path (Path): URDF file.

    Returns:
        dict[str, UrdfJoint]: Child link name -> joint that leads to it.
    """
    joints: dict[str, UrdfJoint] = {}
    for el in ET.parse(path).getroot().iter("joint"):
        parent = el.find("parent")
        child = el.find("child")
        if parent is None or child is None:
            continue
        origin = el.find("origin")
        xyz = [float(v) for v in origin.attrib.get("xyz", "0 0 0").split()] if origin is not None else [0.0] * 3
        rpy = [float(v) for v in origin.attrib.get("rpy", "0 0 0").split()] if origin is not None else [0.0] * 3
        transform = np.eye(4)
        transform[:3, :3] = rpy_to_matrix(*rpy)
        transform[:3, 3] = xyz
        axis_el = el.find("axis")
        axis = np.array([float(v) for v in axis_el.attrib["xyz"].split()]) if axis_el is not None else np.zeros(3)
        joints[child.attrib["link"]] = UrdfJoint(
            el.attrib["name"], el.attrib["type"], parent.attrib["link"], child.attrib["link"], transform, axis
        )
    return joints


def link_pose(joints: dict[str, UrdfJoint], link: str, q: dict[str, float]) -> np.ndarray:
    """Forward kinematics of one link in the root link frame.

    Args:
        joints (dict[str, UrdfJoint]): Output of load_urdf.
        link (str): Target link name.
        q (dict[str, float]): Joint name -> angle in rad (missing joints are 0).

    Returns:
        np.ndarray: 4x4 pose of the link in the root frame.
    """
    chain: list[UrdfJoint] = []
    while link in joints:
        chain.append(joints[link])
        link = joints[link].parent
    pose = np.eye(4)
    for joint in reversed(chain):
        pose = pose @ joint.origin
        if joint.kind != FIXED:
            motion = np.eye(4)
            motion[:3, :3] = axis_angle_matrix(joint.axis, q.get(joint.name, 0.0))
            pose = pose @ motion
    return pose
