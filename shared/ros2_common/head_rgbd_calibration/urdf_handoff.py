"""Hand the calibrated ToF pose to the head URDF: read the chain, compute and rewrite the tof_joint origin."""

import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from ros2_common.head_rgbd_calibration.extrinsics import ExtrinsicResult
from ros2_common.head_rgbd_calibration.urdf_text import TOF_JOINT, Origin, format_vec, rewrite_tof_joint

CAMERA_JOINT = "head_camera_joint"
CAMERA_OPTICAL_JOINT = "head_camera_optical_joint"
TOF_OPTICAL_JOINT = "tof_optical_joint"
CHAIN_JOINTS = (CAMERA_JOINT, CAMERA_OPTICAL_JOINT, TOF_JOINT, TOF_OPTICAL_JOINT)
RPY_SEQUENCE = "xyz"  # lowercase = extrinsic (fixed-axis) rotations, the URDF rpy convention
MM_PER_M = 1000.0


@dataclass(frozen=True)
class JointInfo:
    """A fixed joint of the head chain."""

    name: str
    parent: str
    child: str
    origin: Origin

    @property
    def rpy(self) -> tuple[float, float, float]:
        """The origin rpy."""
        return self.origin.rpy


@dataclass(frozen=True)
class HeadChain:
    """The four fixed joints that place the camera and ToF optical frames under head_link."""

    camera: JointInfo
    camera_optical: JointInfo
    tof: JointInfo
    tof_optical: JointInfo


@dataclass(frozen=True)
class Drift:
    """Difference between the URDF's camera-to-ToF transform and a calibration."""

    translation_mm: float
    rotation_deg: float


def to_matrix(origin: Origin) -> np.ndarray:
    """4x4 homogeneous transform of an origin."""
    m = np.eye(4)
    m[:3, :3] = Rotation.from_euler(RPY_SEQUENCE, origin.rpy).as_matrix()
    m[:3, 3] = origin.xyz
    return m


def from_matrix(m: np.ndarray) -> Origin:
    """Origin of a 4x4 transform."""
    rpy = Rotation.from_matrix(m[:3, :3]).as_euler(RPY_SEQUENCE)
    return Origin(xyz=tuple(float(v) for v in m[:3, 3]), rpy=tuple(float(v) for v in rpy))  # type: ignore[arg-type]


def extrinsic_matrix(extrinsic: ExtrinsicResult) -> np.ndarray:
    """4x4 transform T(head_camera_optical_frame <- tof_optical_frame) of an extrinsic."""
    m = np.eye(4)
    m[:3, :3] = extrinsic.r
    m[:3, 3] = extrinsic.t
    return m


def parse_vec(text: str | None) -> tuple[float, float, float]:
    """Parse 'x y z' (zeros when absent)."""
    values = [float(v) for v in text.split()] if text else [0.0, 0.0, 0.0]
    return values[0], values[1], values[2]


def read_chain(urdf_text: str) -> HeadChain:
    """Parse the camera and ToF joints of a head URDF.

    Args:
        urdf_text: URDF XML text.

    Returns:
        The HeadChain.

    Raises:
        ValueError: a required joint is missing (the message names it).
    """
    root = ET.fromstring(urdf_text)
    found: dict[str, JointInfo] = {}
    for joint in root.findall("joint"):
        name = joint.get("name", "")
        if name not in CHAIN_JOINTS:
            continue
        parent, child = joint.find("parent"), joint.find("child")
        origin = joint.find("origin")
        found[name] = JointInfo(
            name=name,
            parent=parent.get("link", "") if parent is not None else "",
            child=child.get("link", "") if child is not None else "",
            origin=Origin(
                xyz=parse_vec(origin.get("xyz") if origin is not None else None),
                rpy=parse_vec(origin.get("rpy") if origin is not None else None),
            ),
        )
    for name in CHAIN_JOINTS:
        if name not in found:
            raise ValueError(f"URDF has no joint {name}")
    return HeadChain(found[CAMERA_JOINT], found[CAMERA_OPTICAL_JOINT], found[TOF_JOINT], found[TOF_OPTICAL_JOINT])


def tof_joint_origin(chain: HeadChain, extrinsic: ExtrinsicResult) -> Origin:
    """The tof_joint origin (relative to head_link) that realises the calibrated extrinsic.

    T_head_tof = T_head_cam * T_cam_camopt * T_camopt_tofopt * inv(T_tof_tofopt)

    Args:
        chain: Chain read from the URDF.
        extrinsic: Calibrated T(head_camera_optical_frame <- tof_optical_frame).

    Returns:
        The origin to write into tof_joint.
    """
    t_head_tof = (
        to_matrix(chain.camera.origin)
        @ to_matrix(chain.camera_optical.origin)
        @ extrinsic_matrix(extrinsic)
        @ np.linalg.inv(to_matrix(chain.tof_optical.origin))
    )
    return from_matrix(t_head_tof)


def check_drift(chain: HeadChain, extrinsic: ExtrinsicResult) -> Drift:
    """Translation (mm) and rotation (deg) between the URDF's current camopt <- tofopt and the calibration."""
    t_urdf = (
        np.linalg.inv(to_matrix(chain.camera.origin) @ to_matrix(chain.camera_optical.origin))
        @ to_matrix(chain.tof.origin)
        @ to_matrix(chain.tof_optical.origin)
    )
    delta = np.linalg.inv(extrinsic_matrix(extrinsic)) @ t_urdf
    return Drift(
        translation_mm=float(np.linalg.norm(delta[:3, 3]) * MM_PER_M),
        rotation_deg=float(np.degrees(np.linalg.norm(Rotation.from_matrix(delta[:3, :3]).as_rotvec()))),
    )


__all__ = ["Origin", "format_vec", "rewrite_tof_joint"]
