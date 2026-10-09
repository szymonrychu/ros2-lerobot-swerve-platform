"""Below-surface slow zone for arm motions: pure geometry, no ROS.

'Below ground' is relative to the ROBOT: base_link z = 0 is the plane under the wheels. The effective surface at a
point is the higher of (a) the robot plane shifted to the expected surface height (base_link z = surface_z_m) and (b)
the gravity-level plane through base_link (0, 0, surface_z_m), derived from the robot tilt (IMU roll/pitch, or a
per-call override). A trajectory sample whose checked FK points (fixed jaw tip, tool point, moving jaw tip, wrist and
elbow link origins) come closer than margin_m to that surface is executed at slow_speed_scale of its normal speed. The
guard never blocks a motion: the tracking-error and effort aborts stay the contact safety net.

Also holds the arm mount conversions (arm base frame <-> base_link) and the jaw opening model of the gripper.
"""

import logging
import math
from collections.abc import Sequence
from dataclasses import dataclass, field
from typing import Literal

import numpy as np
from pydantic import BaseModel, ConfigDict, Field

from .config import ArmBaseOffset, FloorGuardSettings
from .ik import ArmKinematics

LOGGER = logging.getLogger("mcp_server.floor_guard")
# Moving jaw pivot and rotation axis in gripper_link (URDF joint 'gripper': origin xyz 0.0202 0.0188 -0.0234, rpy
# 1.5708 0 0, axis z -> the gripper_link -y axis). The rotation sign is resolved against the jaw opening direction.
JAW_PIVOT_IN_GRIPPER_LINK = (0.0202, 0.0188, -0.0234)
JAW_AXIS_IN_GRIPPER_LINK = (0.0, -1.0, 0.0)
JAW_SIGN_PROBE_RAD = 0.1
# Upper end (rad above closed) of the bisection that maps a jaw gap to a gripper angle.
JAW_SEARCH_SPAN_RAD = 2.0
JAW_BISECTION_STEPS = 60
ELBOW_LINK = "lower_arm_link"  # child of elbow_flex: its origin is the elbow axis
WRIST_LINK = "wrist_link"  # child of wrist_flex: its origin is the wrist_flex axis
GRIPPER_LINK = "gripper_link"
TOOL_FRAME_LINK = "gripper_frame_link"
NORMAL_SCALE = 1.0
MAX_TILT_OVERRIDE_DEG = 45.0

TiltSource = Literal["imu", "override", "none"]


class TiltOverrideDeg(BaseModel):
    """Robot tilt (deg) replacing the IMU: roll > 0 = left side up, pitch > 0 = nose down (REP-103)."""

    model_config = ConfigDict(extra="forbid")

    roll: float = Field(default=0.0, ge=-MAX_TILT_OVERRIDE_DEG, le=MAX_TILT_OVERRIDE_DEG)
    pitch: float = Field(default=0.0, ge=-MAX_TILT_OVERRIDE_DEG, le=MAX_TILT_OVERRIDE_DEG)


class FloorOverride(BaseModel):
    """Per-call slow-zone overrides; the only way an agent changes the slow zone (it can never switch it off).

    Attributes:
        surface_z_m: Expected surface height relative to the robot plane (m), e.g. -0.18 for a stair or hole below:
            normal speed is allowed down to that surface and the slow zone starts below it. None = configured value.
        tilt_override_deg: Robot tilt replacing the IMU tilt; None = use the IMU.
    """

    model_config = ConfigDict(extra="forbid")

    surface_z_m: float | None = Field(default=None, ge=-1.0, le=1.0)
    tilt_override_deg: TiltOverrideDeg | None = None


@dataclass(frozen=True)
class Tilt:
    """Robot roll/pitch relative to gravity (rad, REP-103: roll > 0 left side up, pitch > 0 nose down)."""

    roll_rad: float
    pitch_rad: float


@dataclass(frozen=True)
class TiltSample:
    """A tilt measurement with its receive time (monotonic s, same clock as the arm backend)."""

    tilt: Tilt
    stamp: float


def yaw_matrix(yaw: float) -> np.ndarray:
    """Rotation about z.

    Args:
        yaw (float): Angle (rad).

    Returns:
        np.ndarray: 3x3 rotation matrix.
    """
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def arm_to_base_link(point: Sequence[float] | np.ndarray, mount: ArmBaseOffset) -> np.ndarray:
    """Arm base frame point -> robot base_link.

    Args:
        point (Sequence[float] | np.ndarray): (x, y, z) in the arm base frame (m).
        mount (ArmBaseOffset): Arm base pose in base_link.

    Returns:
        np.ndarray: (x, y, z) in base_link (m).
    """
    return yaw_matrix(mount.yaw) @ np.asarray(point, dtype=np.float64) + np.array([mount.x, mount.y, mount.z])


def base_link_to_arm(point: Sequence[float] | np.ndarray, mount: ArmBaseOffset) -> np.ndarray:
    """Robot base_link point -> arm base frame.

    Args:
        point (Sequence[float] | np.ndarray): (x, y, z) in base_link (m).
        mount (ArmBaseOffset): Arm base pose in base_link.

    Returns:
        np.ndarray: (x, y, z) in the arm base frame (m).
    """
    return yaw_matrix(mount.yaw).T @ (np.asarray(point, dtype=np.float64) - np.array([mount.x, mount.y, mount.z]))


def tilt_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> Tilt:
    """Roll and pitch relative to gravity of an IMU orientation (ZYX Euler; yaw is ignored).

    Args:
        qx (float): Quaternion x.
        qy (float): Quaternion y.
        qz (float): Quaternion z.
        qw (float): Quaternion w.

    Returns:
        Tilt: Roll and pitch (rad).
    """
    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx))))
    return Tilt(roll_rad=roll, pitch_rad=pitch)


def monitor_tilt_sample(imu: tuple[float, float, float, float] | None) -> TiltSample | None:
    """Tilt sample from the body monitor's latest IMU record (RobotMonitor.imu).

    Args:
        imu (tuple[float, float, float, float] | None): (monotonic receive time s, roll deg, pitch deg, tilt deg).

    Returns:
        TiltSample | None: Roll/pitch in rad with the receive time, None without IMU data.
    """
    if imu is None:
        return None
    return TiltSample(tilt=Tilt(roll_rad=math.radians(imu[1]), pitch_rad=math.radians(imu[2])), stamp=imu[0])


def level_plane_z(x: float, y: float, tilt: Tilt) -> float:
    """Height (base_link z) at (x, y) of the gravity-level plane through the base_link origin.

    The world 'up' vector in base_link is (-sin p, sin r cos p, cos r cos p); the plane through the origin normal to it
    has z = x tan(p) / cos(r) - y tan(r).

    Args:
        x (float): base_link x (m).
        y (float): base_link y (m).
        tilt (Tilt): Robot roll/pitch.

    Returns:
        float: Plane height (m).
    """
    return x * math.tan(tilt.pitch_rad) / math.cos(tilt.roll_rad) - y * math.tan(tilt.roll_rad)


def effective_surface_z(xy: Sequence[float], surface_z_m: float, tilt: Tilt | None) -> float:
    """Effective surface height at a base_link point: the higher of the robot plane and the level plane.

    Args:
        xy (Sequence[float]): base_link (x, y) (m).
        surface_z_m (float): Expected surface height relative to the robot plane (m).
        tilt (Tilt | None): Robot tilt; None uses the robot plane only.

    Returns:
        float: Surface height in base_link (m).
    """
    if tilt is None:
        return surface_z_m
    return surface_z_m + max(0.0, level_plane_z(float(xy[0]), float(xy[1]), tilt))


@dataclass(frozen=True)
class SurfaceModel:
    """Resolved surface of one motion: expected height, tilt and where the tilt came from."""

    surface_z_m: float
    tilt: Tilt | None
    tilt_source: TiltSource
    mount: ArmBaseOffset

    def clearance(self, point_arm: np.ndarray) -> float:
        """Height of an arm-frame point above the effective surface (negative below it).

        Args:
            point_arm (np.ndarray): (x, y, z) in the arm base frame (m).

        Returns:
            float: Clearance (m).
        """
        p = arm_to_base_link(point_arm, self.mount)
        return float(p[2]) - effective_surface_z(p[:2], self.surface_z_m, self.tilt)

    def describe(self) -> dict[str, object]:
        """JSON summary of the surface.

        Returns:
            dict[str, object]: surface_z_m, tilt_source and tilt_deg (null without a tilt).
        """
        tilt = (
            None
            if self.tilt is None
            else {
                "roll": round(math.degrees(self.tilt.roll_rad), 2),
                "pitch": round(math.degrees(self.tilt.pitch_rad), 2),
            }
        )
        return {"surface_z_m": self.surface_z_m, "tilt_source": self.tilt_source, "tilt_deg": tilt}


def rotate(vector: np.ndarray, axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues rotation of a vector about a unit axis.

    Args:
        vector (np.ndarray): 3-vector.
        axis (np.ndarray): Unit rotation axis.
        angle (float): Angle (rad).

    Returns:
        np.ndarray: Rotated vector.
    """
    c, s = math.cos(angle), math.sin(angle)
    return vector * c + np.cross(axis, vector) * s + axis * float(axis @ vector) * (1.0 - c)


class JawModel:
    """Approximate moving-jaw geometry: the closed moving jaw tip sits on the tool point (fixed jaw inner face) and
    opens by rotating about the URDF gripper joint pivot. Gives the jaw gap (m, across the jaws) for a gripper angle
    and back, and the moving jaw tip position for the floor check."""

    def __init__(self, kin: ArmKinematics, jaw_open_axis: tuple[float, float, float], closed_rad: float) -> None:
        """Derive the closed tip and the opening direction in gripper_link from the URDF chain.

        Args:
            kin (ArmKinematics): Kinematics (tool offset = fixed jaw inner face).
            jaw_open_axis (tuple[float, float, float]): Unit opening direction in gripper_frame_link.
            closed_rad (float): Gripper joint position of a closed jaw (rad).
        """
        frames = kin.link_frames({})
        t_gl_tool = np.linalg.inv(frames[GRIPPER_LINK]) @ frames[TOOL_FRAME_LINK]
        self.closed_rad = closed_rad
        self.closed_tip = t_gl_tool[:3, :3] @ kin.tool_offset + t_gl_tool[:3, 3]
        self.open_axis = t_gl_tool[:3, :3] @ np.array(jaw_open_axis, dtype=np.float64)
        self.pivot = np.array(JAW_PIVOT_IN_GRIPPER_LINK)
        self.axis = np.array(JAW_AXIS_IN_GRIPPER_LINK)
        probe = rotate(self.closed_tip - self.pivot, self.axis, JAW_SIGN_PROBE_RAD) + self.pivot
        self.sign = 1.0 if float((probe - self.closed_tip) @ self.open_axis) > 0.0 else -1.0

    def tip_in_gripper_link(self, angle: float) -> np.ndarray:
        """Moving jaw tip in gripper_link for a gripper angle.

        Args:
            angle (float): Gripper joint position (rad).

        Returns:
            np.ndarray: Tip position (m).
        """
        turned = rotate(self.closed_tip - self.pivot, self.axis, self.sign * (angle - self.closed_rad))
        return turned + self.pivot

    def gap(self, angle: float) -> float:
        """Jaw opening across the jaws (m) for a gripper angle.

        Args:
            angle (float): Gripper joint position (rad).

        Returns:
            float: Distance of the moving jaw tip from the fixed jaw inner face along the opening direction.
        """
        return float((self.tip_in_gripper_link(angle) - self.closed_tip) @ self.open_axis)

    def angle_for_gap(self, gap: float) -> float:
        """Gripper angle giving a jaw gap (bisection; the gap grows monotonically over the useful range).

        Args:
            gap (float): Wanted opening (m), >= 0.

        Returns:
            float: Gripper joint position (rad); the widest-gap angle when the gap is not reachable.
        """
        lo, hi = self.closed_rad, self.closed_rad + JAW_SEARCH_SPAN_RAD
        for _ in range(JAW_BISECTION_STEPS):
            mid = (lo + hi) / 2.0
            if self.gap(mid) < gap:
                lo = mid
            else:
                hi = mid
        return hi

    def moving_tip(self, t_gripper_link: np.ndarray, angle: float) -> np.ndarray:
        """Moving jaw tip in the arm base frame.

        Args:
            t_gripper_link (np.ndarray): 4x4 pose of gripper_link in the arm base frame.
            angle (float): Gripper joint position (rad).

        Returns:
            np.ndarray: Tip position (m).
        """
        return t_gripper_link[:3, :3] @ self.tip_in_gripper_link(angle) + t_gripper_link[:3, 3]


@dataclass
class GuardReport:
    """Per-sample slow-zone verdicts of a trajectory."""

    scales: list[float]
    clearances: list[float]
    lowest_points: list[str]
    surface: SurfaceModel
    slow_speed_scale: float
    margin_m: float
    enabled: bool = True
    notes: list[str] = field(default_factory=list)

    def summary(self) -> dict[str, object] | None:
        """JSON summary for motion results; None when no sample was slowed.

        Returns:
            dict[str, object] | None: slowed/total samples, the lowest clearance and point, speed scale and surface.
        """
        slowed = sum(1 for s in self.scales if s < NORMAL_SCALE)
        if not slowed:
            return None
        lowest = min(range(len(self.clearances)), key=lambda i: self.clearances[i])
        return {
            "slowed_samples": slowed,
            "samples": len(self.scales),
            "speed_scale": self.slow_speed_scale,
            "margin_m": self.margin_m,
            "min_clearance_m": round(self.clearances[lowest], 4),
            "lowest_point": self.lowest_points[lowest],
            **self.surface.describe(),
        }


class FloorGuard:
    """Evaluates the slow zone on sampled joint configurations (one FK pass per sample)."""

    def __init__(
        self,
        kin: ArmKinematics,
        settings: FloorGuardSettings,
        mount: ArmBaseOffset,
        jaw: JawModel,
        gripper_joint: str = "gripper",
    ) -> None:
        """Bind the guard to the arm model.

        Args:
            kin (ArmKinematics): Kinematics on the arm URDF.
            settings (FloorGuardSettings): Margin, slow scale, default surface, IMU age.
            mount (ArmBaseOffset): Arm base pose in base_link.
            jaw (JawModel): Moving jaw model.
            gripper_joint (str): Gripper joint name in the samples.
        """
        self.kin = kin
        self.settings = settings
        self.mount = mount
        self.jaw = jaw
        self.gripper = gripper_joint
        self.imu_missing_logged = False

    def surface(self, override: FloorOverride | None, imu: TiltSample | None, now: float) -> SurfaceModel:
        """Resolve the surface of a motion from the config, the per-call override and the IMU.

        A missing or stale IMU (older than imu_max_age_s) leaves only the robot plane; that is logged once per outage.

        Args:
            override (FloorOverride | None): Per-call overrides.
            imu (TiltSample | None): Latest IMU tilt.
            now (float): Current time on the IMU sample clock (s).

        Returns:
            SurfaceModel: The surface.
        """
        surface_z = self.settings.surface_z_m
        if override is not None and override.surface_z_m is not None:
            surface_z = override.surface_z_m
        if override is not None and override.tilt_override_deg is not None:
            deg = override.tilt_override_deg
            tilt = Tilt(roll_rad=math.radians(deg.roll), pitch_rad=math.radians(deg.pitch))
            return SurfaceModel(surface_z, tilt, "override", self.mount)
        if imu is not None and now - imu.stamp <= self.settings.imu_max_age_s:
            self.imu_missing_logged = False
            return SurfaceModel(surface_z, imu.tilt, "imu", self.mount)
        if not self.imu_missing_logged:
            LOGGER.warning(
                "no IMU tilt newer than %.1f s: the arm slow zone uses the robot plane only",
                self.settings.imu_max_age_s,
            )
            self.imu_missing_logged = True
        return SurfaceModel(surface_z, None, "none", self.mount)

    def checked_points(self, joints: dict[str, float]) -> dict[str, np.ndarray]:
        """Points checked against the surface, in the arm base frame.

        Args:
            joints (dict[str, float]): Joint name -> measured rad (gripper optional, closed when missing).

        Returns:
            dict[str, np.ndarray]: elbow, wrist, jaw_tip (tool frame origin), tool_point (fixed jaw inner face) and
                moving_jaw_tip.
        """
        frames = self.kin.link_frames(joints)
        tool = frames[TOOL_FRAME_LINK]
        return {
            "elbow": frames[ELBOW_LINK][:3, 3],
            "wrist": frames[WRIST_LINK][:3, 3],
            "jaw_tip": tool[:3, 3],
            "tool_point": tool[:3, :3] @ self.kin.tool_offset + tool[:3, 3],
            "moving_jaw_tip": self.jaw.moving_tip(frames[GRIPPER_LINK], joints.get(self.gripper, self.jaw.closed_rad)),
        }

    def evaluate(self, samples: list[dict[str, float]], surface: SurfaceModel) -> GuardReport:
        """Speed scale of every sample: slow_speed_scale when a checked point is closer than margin_m to the surface.

        Args:
            samples (list[dict[str, float]]): Joint configurations (measured space).
            surface (SurfaceModel): Resolved surface.

        Returns:
            GuardReport: Per-sample scales, clearances and lowest points.
        """
        cfg = self.settings
        if not cfg.enabled:
            n = len(samples)
            return GuardReport(
                [NORMAL_SCALE] * n, [math.inf] * n, [""] * n, surface, cfg.slow_speed_scale, cfg.margin_m, False
            )
        scales: list[float] = []
        clearances: list[float] = []
        lowest: list[str] = []
        for joints in samples:
            name, clearance = min(
                ((n, surface.clearance(p)) for n, p in self.checked_points(joints).items()), key=lambda item: item[1]
            )
            clearances.append(clearance)
            lowest.append(name)
            scales.append(cfg.slow_speed_scale if clearance < cfg.margin_m else NORMAL_SCALE)
        return GuardReport(scales, clearances, lowest, surface, cfg.slow_speed_scale, cfg.margin_m)


def step_scales(sample_scales: list[float]) -> list[float]:
    """Speed scale of each trajectory step (between consecutive samples): the slower of its two ends.

    Args:
        sample_scales (list[float]): Per-sample scales, the start sample first.

    Returns:
        list[float]: One scale per step (len(sample_scales) - 1).
    """
    return [min(a, b) for a, b in zip(sample_scales, sample_scales[1:], strict=False)]


def retime(start: dict[str, float], points: list[dict[str, float]], scales: list[float]) -> list[dict[str, float]]:
    """Time-scale trajectory steps for constant-rate streaming: a step with scale s becomes ceil(1/s) linear sub-steps.

    Args:
        start (dict[str, float]): Pose before the first point.
        points (list[dict[str, float]]): Setpoints at the streaming rate (the last one is the goal).
        scales (list[float]): Speed scale per step (one per point), in (0, 1].

    Returns:
        list[dict[str, float]]: Setpoints at the same rate; slowed steps take proportionally more ticks.
    """
    out: list[dict[str, float]] = []
    prev = start
    for point, scale in zip(points, scales, strict=True):
        n = max(1, math.ceil(1.0 / scale - 1e-9))
        for k in range(1, n):
            out.append({j: prev[j] + (point[j] - prev[j]) * k / n for j in point})
        out.append(dict(point))
        prev = point
    return out
