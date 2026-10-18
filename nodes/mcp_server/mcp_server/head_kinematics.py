"""Pan/tilt head kinematics on the head URDF: forward kinematics of every head frame and look-at aiming.

Frames: base_link is the robot body frame (REP 103): x forward, y left, z up, origin on the floor (floor z = 0).
Units: metres and radians throughout.
Sign conventions (head.urdf): ``head_pan`` positive turns the head left (CCW seen from above, about +z);
``head_tilt`` positive pitches it down (about +y of the panned head). pan = tilt = 0 is straight ahead and level.
Optical frames (``head_camera_optical_frame``, ``tof_optical_frame``): z forward along the optical axis, x right,
y down.

The RGB camera does not sit on the pan or tilt axis (mount, tilt-axis height, forward and lateral camera offsets), so
look-at is a small numerical solve on the URDF chain rather than two atan2 calls.
"""

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from mcp_server.config import REPO_ROOT

BASE_LINK = "base_link"
PAN_JOINT = "head_pan"
TILT_JOINT = "head_tilt"
CAMERA_FRAME = "head_camera_optical_frame"
TOF_FRAME = "tof_optical_frame"
DEFAULT_HEAD_URDF = REPO_ROOT / "nodes/web_ui/urdf/head.urdf"
# Pan axis base frame; the offset-free seed aims from the pan axis at the tilt-axis height (head_link).
SEED_FRAME = "head_link"
MIN_CAMERA_DISTANCE_M = 0.15
MAX_RESIDUAL_RAD = math.radians(0.2)
# Solver tolerances: tight enough to land well inside 1 mrad (2 mm at 2 m), far below MAX_RESIDUAL_RAD.
SOLVER_TOLERANCE = 1e-12
MOVABLE_JOINT_TYPES = ("revolute", "continuous")
SUPPORTED_JOINT_TYPES = ("fixed", *MOVABLE_JOINT_TYPES)


class HeadAimError(ValueError):
    """The head cannot aim the RGB optical axis at the point; carries the angles it would need and the limits.

    Attributes:
        pan (float): Pan the aim would need (rad, + left).
        tilt (float): Tilt the aim would need (rad, + down).
        limits (dict[str, tuple[float, float]]): Joint name -> (lower, upper) URDF limits (rad).
    """

    def __init__(self, reason: str, pan: float, tilt: float, limits: dict[str, tuple[float, float]]) -> None:
        """Build the message from the reason, the needed angles and the limits (shown in degrees).

        Args:
            reason (str): Why the aim is refused.
            pan (float): Needed pan (rad).
            tilt (float): Needed tilt (rad).
            limits (dict[str, tuple[float, float]]): Joint limits (rad).
        """
        self.pan = pan
        self.tilt = tilt
        self.limits = limits
        bounds = ", ".join(f"{j} [{math.degrees(lo):.1f}, {math.degrees(hi):.1f}]" for j, (lo, hi) in limits.items())
        super().__init__(
            f"{reason}: needs pan {math.degrees(pan):.1f} deg, tilt {math.degrees(tilt):.1f} deg "
            f"(limits in deg: {bounds})"
        )


@dataclass(frozen=True)
class UrdfJoint:
    """One URDF joint: parent link, fixed origin transform, and (for movable joints) the unit rotation axis."""

    name: str
    parent: str
    origin: np.ndarray
    axis: np.ndarray | None


def origin_transform(element: ET.Element | None) -> np.ndarray:
    """4x4 transform of a URDF ``<origin xyz rpy>`` element (rpy = fixed-axis roll, pitch, yaw).

    Args:
        element (ET.Element | None): The origin element; None means identity.

    Returns:
        np.ndarray: 4x4 homogeneous transform parent <- child.
    """
    transform = np.eye(4)
    if element is None:
        return transform
    xyz = [float(v) for v in element.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in element.get("rpy", "0 0 0").split()]
    transform[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    transform[:3, 3] = xyz
    return transform


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi).

    Args:
        angle (float): Angle (rad).

    Returns:
        float: Wrapped angle (rad).
    """
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class HeadKinematics:
    """Forward kinematics and look-at of the pan/tilt head, parsed from the head URDF.

    Every head link or frame below ``base_link`` is reachable by ``link_pose``; joint angles are URDF angles of
    ``head_pan`` and ``head_tilt`` (rad; pan + left, tilt + down). Other movable joints are held at 0.
    """

    def __init__(self, urdf_path: Path) -> None:
        """Parse the URDF joint tree and the head joint limits.

        Args:
            urdf_path (Path): Head URDF (normally ``nodes/web_ui/urdf/head.urdf``).

        Raises:
            ValueError: When the URDF lacks head_pan / head_tilt limits or uses an unsupported joint type.
        """
        root = ET.parse(urdf_path).getroot()
        self.joints: dict[str, UrdfJoint] = {}  # child link -> joint
        limits: dict[str, tuple[float, float]] = {}
        for joint in root.iter("joint"):
            kind = joint.get("type", "")
            name = joint.attrib["name"]
            if kind not in SUPPORTED_JOINT_TYPES:
                raise ValueError(f"joint {name!r}: unsupported type {kind!r} (supported: {SUPPORTED_JOINT_TYPES})")
            axis = None
            if kind in MOVABLE_JOINT_TYPES:
                axis_element = joint.find("axis")
                axis_xyz = axis_element.get("xyz", "1 0 0") if axis_element is not None else "1 0 0"
                axis = np.array([float(v) for v in axis_xyz.split()])
                axis /= np.linalg.norm(axis)
                limit = joint.find("limit")
                if kind == "revolute" and limit is not None:
                    limits[name] = (float(limit.attrib["lower"]), float(limit.attrib["upper"]))
            child, parent = joint.find("child"), joint.find("parent")
            if child is None or parent is None:
                raise ValueError(f"joint {name!r}: missing parent or child link")
            self.joints[child.attrib["link"]] = UrdfJoint(
                name, parent.attrib["link"], origin_transform(joint.find("origin")), axis
            )
        missing = [j for j in (PAN_JOINT, TILT_JOINT) if j not in limits]
        if missing:
            raise ValueError(f"{urdf_path}: no revolute limits for {missing}")
        self.links = {BASE_LINK, *self.joints}
        self._limits = {PAN_JOINT: limits[PAN_JOINT], TILT_JOINT: limits[TILT_JOINT]}

    @property
    def limits(self) -> dict[str, tuple[float, float]]:
        """Head joint limits from the URDF.

        Returns:
            dict[str, tuple[float, float]]: ``head_pan`` and ``head_tilt`` -> (lower, upper) in rad.
        """
        return dict(self._limits)

    def link_pose(self, pan: float, tilt: float, link: str) -> np.ndarray:
        """Pose of a head link or frame in base_link at the given head angles.

        Args:
            pan (float): head_pan angle (rad, + left).
            tilt (float): head_tilt angle (rad, + down).
            link (str): URDF link name, e.g. ``head_camera_optical_frame``, ``tof_optical_frame``, ``head_link``.

        Returns:
            np.ndarray: 4x4 homogeneous transform base_link <- link (translation in m).

        Raises:
            ValueError: When the link is not in the URDF or not connected to base_link.
        """
        if link not in self.links:
            raise ValueError(f"unknown head link {link!r}; known: {sorted(self.links)}")
        angles = {PAN_JOINT: pan, TILT_JOINT: tilt}
        pose = np.eye(4)
        current = link
        while current != BASE_LINK:
            joint = self.joints.get(current)
            if joint is None:
                raise ValueError(f"head link {link!r} is not connected to {BASE_LINK}")
            step = joint.origin
            if joint.axis is not None:
                motion = np.eye(4)
                motion[:3, :3] = Rotation.from_rotvec(joint.axis * angles.get(joint.name, 0.0)).as_matrix()
                step = step @ motion
            pose = step @ pose
            current = joint.parent
        return pose

    def aim_error(self, pan: float, tilt: float, point_base: np.ndarray) -> float:
        """Angle between the RGB optical axis and the camera-to-point ray.

        Args:
            pan (float): head_pan angle (rad, + left).
            tilt (float): head_tilt angle (rad, + down).
            point_base (np.ndarray): Point in base_link (m), shape (3,).

        Returns:
            float: Angle in rad, 0 when the axis passes through the point, pi when the point is straight behind.
        """
        return float(np.linalg.norm(self.aim_residual(np.array([pan, tilt]), np.asarray(point_base, dtype=float))))

    def aim_residual(self, angles: np.ndarray, point_base: np.ndarray) -> np.ndarray:
        """Least-squares residual of the aim: the off-axis direction of the point scaled to the off-axis angle.

        The point's unit ray in the optical frame is (x, y, z); the residual is (x, y) / |(x, y)| * angle, smooth
        through the solution and with norm equal to the angle between the optical axis and the ray.

        Args:
            angles (np.ndarray): (pan, tilt) in rad.
            point_base (np.ndarray): Point in base_link (m), shape (3,).

        Returns:
            np.ndarray: Residual (2,), rad.
        """
        pose = self.link_pose(float(angles[0]), float(angles[1]), CAMERA_FRAME)
        ray = pose[:3, :3].T @ (point_base - pose[:3, 3])
        ray /= np.linalg.norm(ray)
        off_axis = math.hypot(ray[0], ray[1])
        angle = math.atan2(off_axis, ray[2])
        scale = angle / off_axis if off_axis > 1e-12 else 1.0
        return ray[:2] * scale

    def camera_distance(self, pan: float, tilt: float, point_base: np.ndarray) -> float:
        """Distance from the RGB optical frame origin to the point.

        Args:
            pan (float): head_pan angle (rad).
            tilt (float): head_tilt angle (rad).
            point_base (np.ndarray): Point in base_link (m), shape (3,).

        Returns:
            float: Distance in m.
        """
        return float(np.linalg.norm(point_base - self.link_pose(pan, tilt, CAMERA_FRAME)[:3, 3]))

    def offset_free_seed(self, point_base: np.ndarray) -> tuple[float, float]:
        """Look-at angles as if the camera sat on the pan and tilt axes (the solver seed).

        Args:
            point_base (np.ndarray): Point in base_link (m), shape (3,).

        Returns:
            tuple[float, float]: (pan, tilt) in rad, pan in [-pi, pi).
        """
        delta = point_base - self.link_pose(0.0, 0.0, SEED_FRAME)[:3, 3]
        return math.atan2(delta[1], delta[0]), math.atan2(-delta[2], math.hypot(delta[0], delta[1]))

    def solve(self, point_base: np.ndarray, seed: tuple[float, float], bounded: bool) -> tuple[float, float, float]:
        """Least-squares aim of the optical axis at the point.

        Args:
            point_base (np.ndarray): Point in base_link (m), shape (3,).
            seed (tuple[float, float]): Initial (pan, tilt) in rad.
            bounded (bool): Bound the solve by the URDF limits (seed clipped into them); otherwise free.

        Returns:
            tuple[float, float, float]: (pan, tilt, residual angle) in rad; an unbounded pan is wrapped to [-pi, pi).
        """
        if bounded:
            lower = np.array([self._limits[PAN_JOINT][0], self._limits[TILT_JOINT][0]])
            upper = np.array([self._limits[PAN_JOINT][1], self._limits[TILT_JOINT][1]])
            x0 = np.clip(np.array(seed), lower, upper)
        else:
            lower, upper = np.full(2, -np.inf), np.full(2, np.inf)
            x0 = np.array(seed)
        result = least_squares(
            self.aim_residual,
            x0,
            bounds=(lower, upper),
            args=(point_base,),
            xtol=SOLVER_TOLERANCE,
            ftol=SOLVER_TOLERANCE,
            gtol=SOLVER_TOLERANCE,
        )
        pan, tilt = float(result.x[0]), float(result.x[1])
        if not bounded:
            pan, tilt = wrap_angle(pan), wrap_angle(tilt)
        return pan, tilt, self.aim_error(pan, tilt, point_base)

    def within_limits(self, pan: float, tilt: float) -> bool:
        """Whether both angles lie inside the URDF limits.

        Args:
            pan (float): head_pan angle (rad).
            tilt (float): head_tilt angle (rad).

        Returns:
            bool: True when lower <= angle <= upper for both joints.
        """
        (lo_pan, hi_pan), (lo_tilt, hi_tilt) = self._limits[PAN_JOINT], self._limits[TILT_JOINT]
        return lo_pan <= pan <= hi_pan and lo_tilt <= tilt <= hi_tilt

    def look_at(self, point_base: np.ndarray) -> tuple[float, float]:
        """Head angles for which the +z axis of ``head_camera_optical_frame`` passes through the point.

        Accounts for every URDF offset (mount, tilt-axis height, camera forward and lateral offset): a bounded
        ``scipy.optimize.least_squares`` over (pan, tilt) on the angle between the optical axis and the
        camera-to-point ray, seeded by the offset-free atan2 solution. Never returns clamped angles that miss the
        point: a solve that cannot reach the point within limits is refused.

        Args:
            point_base (np.ndarray): Point in base_link (m; x forward, y left, z up, floor z = 0), shape (3,).

        Returns:
            tuple[float, float]: (pan, tilt) in rad (pan + left, tilt + down), inside the URDF limits.

        Raises:
            HeadAimError: When the needed angles lie outside the limits, the point is closer than 0.15 m to the
                camera, or the solve residual stays above 0.2 deg; carries the needed angles and the limits.
        """
        point = np.asarray(point_base, dtype=float).reshape(3)
        seed = self.offset_free_seed(point)
        if self.camera_distance(*seed, point) < MIN_CAMERA_DISTANCE_M:
            raise HeadAimError(f"point is closer than {MIN_CAMERA_DISTANCE_M} m to the camera", *seed, self.limits)
        pan, tilt, residual = self.solve(point, seed, bounded=True)
        if residual > MAX_RESIDUAL_RAD:
            # No aim found inside the limits: solve freely for the angles the point needs.
            pan, tilt, residual = self.solve(point, seed, bounded=False)
        if residual > MAX_RESIDUAL_RAD:
            reason = (
                f"solve residual {math.degrees(residual):.2f} deg stays above {math.degrees(MAX_RESIDUAL_RAD):.1f} deg"
            )
            raise HeadAimError(reason, pan, tilt, self.limits)
        if self.camera_distance(pan, tilt, point) < MIN_CAMERA_DISTANCE_M:
            raise HeadAimError(f"point is closer than {MIN_CAMERA_DISTANCE_M} m to the camera", pan, tilt, self.limits)
        if not self.within_limits(pan, tilt):
            raise HeadAimError("point lies outside the head limits", pan, tilt, self.limits)
        return pan, tilt
