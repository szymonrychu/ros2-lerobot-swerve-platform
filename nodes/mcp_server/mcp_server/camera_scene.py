"""Camera scene geometry for the pixel -> ground tools: calibrated setup, reference frames, ground reports.

Reference frames (documented in the README): the fixed front camera works in ``base_link`` (x forward, y left, z up,
z = 0 is the floor); the gripper camera works in the arm base frame (URDF ``base_link`` of the arm, z = 0 on the arm
mount plane, the floor at ``arm.floor_z_m`` < 0). When ``arm.base_in_base_link`` is configured both frames convert.
"""

import math
from dataclasses import dataclass
from typing import Any

import numpy as np
from ros2_common.camera_geometry import (
    CameraIntrinsics,
    MountPose,
    optical_from_mount,
    project_point_to_pixel,
)
from ros2_common.camera_geometry import pixel_to_ground as geometry_pixel_to_ground

from .config import ArmBaseOffset, IntrinsicsSettings, McpServerConfig
from .ik import ArmKinematics
from .models import BasePose

NOT_CALIBRATED = (
    "camera {camera} not calibrated: set intrinsics and mount in mcp_server config (see README calibration)"
)
FRAME_BASE = "base_link"
FRAME_ARM = "arm_base"
GROUND_METHOD = "pinhole ray through the pixel intersected with the horizontal floor plane (camera_geometry)"
APPROX_NOTE = "approximate intrinsics (hfov only: square pixels, centred principal point, no distortion)"


class CameraNotCalibratedError(ValueError):
    """Raised when a camera lacks intrinsics and/or a mount."""


class PixelError(ValueError):
    """Raised when a pixel has no valid floor point (outside the image, above the horizon or behind the camera)."""


@dataclass(frozen=True)
class CameraSetup:
    """A configured camera: intrinsics, whether they are approximate, the mount (None while uncalibrated)."""

    name: str
    intrinsics: CameraIntrinsics
    approximate: bool
    parent_frame: str
    mount: MountPose | None


def load_intrinsics(settings: IntrinsicsSettings | None) -> tuple[CameraIntrinsics, bool] | None:
    """Build intrinsics from the config.

    Args:
        settings (IntrinsicsSettings | None): Configured source.

    Returns:
        tuple[CameraIntrinsics, bool] | None: (intrinsics, approximate) or None when not configured.
    """
    if settings is None:
        return None
    if settings.calibration_file is not None:
        return CameraIntrinsics.from_calibration_yaml(settings.calibration_file), False
    assert settings.hfov_deg is not None and settings.width is not None and settings.height is not None
    return CameraIntrinsics.from_hfov(settings.width, settings.height, settings.hfov_deg), True


def camera_setup(cfg: McpServerConfig, camera: str, need_mount: bool = True) -> CameraSetup:
    """Resolve a camera's calibration from the config.

    Args:
        cfg (McpServerConfig): Node configuration.
        camera (str): 'gripper' or 'front'.
        need_mount (bool): Require the mount pose (False for capture and the solver, which determine it).

    Returns:
        CameraSetup: The setup.

    Raises:
        CameraNotCalibratedError: When intrinsics (or the mount, if needed) are missing.
    """
    settings = getattr(cfg.cameras, camera)
    loaded = load_intrinsics(settings.intrinsics)
    if loaded is None or (need_mount and settings.mount is None):
        raise CameraNotCalibratedError(NOT_CALIBRATED.format(camera=camera))
    assert settings.parent_frame is not None
    return CameraSetup(camera, loaded[0], loaded[1], settings.parent_frame, settings.mount)


def offset_matrix(offset: ArmBaseOffset) -> np.ndarray:
    """T_base_link_arm_base from the configured arm base offset.

    Args:
        offset (ArmBaseOffset): Arm base pose in base_link.

    Returns:
        np.ndarray: 4x4 transform.
    """
    c, s = math.cos(offset.yaw), math.sin(offset.yaw)
    t = np.eye(4)
    t[:2, :2] = [[c, -s], [s, c]]
    t[:3, 3] = [offset.x, offset.y, offset.z]
    return t


def parent_transform(
    cfg: McpServerConfig, camera: str, kin: ArmKinematics, joints: dict[str, float] | None
) -> np.ndarray:
    """T_reference_parent: the camera's parent link in its reference frame at this moment.

    Args:
        cfg (McpServerConfig): Node configuration.
        camera (str): 'gripper' (parent link pose from the measured joints) or 'front' (identity: base_link).
        kin (ArmKinematics): Arm kinematics on the URDF.
        joints (dict[str, float] | None): Measured arm joints (required for the gripper camera).

    Returns:
        np.ndarray: 4x4 transform.
    """
    if camera == "front":
        return np.eye(4)
    if joints is None:
        raise ValueError("the gripper camera pose needs the measured arm joint positions")
    return kin.link_frame(joints, str(cfg.cameras.gripper.parent_frame))


@dataclass(frozen=True)
class Scene:
    """A camera at one moment: its optical pose in the reference frame plus conversions to and from the other frames.

    Attributes:
        camera: Camera name.
        frame_name: Reference frame name, 'base_link' (front) or 'arm_base' (gripper).
        ground_z: Floor height in the reference frame (m).
        intr: Intrinsics at the working image size.
        approximate: Intrinsics are hfov-based.
        t_ref_optical: 4x4 pose of the optical frame in the reference frame.
        t_base_ref: 4x4 pose of the reference frame in base_link, None when unknown (arm offset not configured).
        t_arm_ref: 4x4 pose of the reference frame in the arm base frame, None when unknown.
    """

    camera: str
    frame_name: str
    ground_z: float
    intr: CameraIntrinsics
    approximate: bool
    t_ref_optical: np.ndarray
    t_base_ref: np.ndarray | None
    t_arm_ref: np.ndarray | None

    def ground_point(self, u: float, v: float) -> np.ndarray | None:
        """Floor point (x, y, ground_z) in the reference frame seen at a pixel.

        Args:
            u (float): Pixel x.
            v (float): Pixel y.

        Returns:
            np.ndarray | None: The point, or None when the ray misses the floor.
        """
        hit = geometry_pixel_to_ground(self.intr, self.t_ref_optical, u, v, self.ground_z)
        if hit is None:
            return None
        return np.array([hit[0], hit[1], self.ground_z])

    def project(self, point: np.ndarray) -> tuple[float, float] | None:
        """Pixel of a reference-frame point, None when behind the camera or off the image.

        Args:
            point (np.ndarray): Point in the reference frame.

        Returns:
            tuple[float, float] | None: (u, v).
        """
        return project_point_to_pixel(self.intr, self.t_ref_optical, point)

    def ref_to_base(self, point: np.ndarray) -> np.ndarray | None:
        """Express a reference-frame point in base_link (None when the arm offset is unknown).

        Args:
            point (np.ndarray): Point in the reference frame.

        Returns:
            np.ndarray | None: Point in base_link.
        """
        return None if self.t_base_ref is None else transform_point(self.t_base_ref, point)

    def base_to_ref(self, point: np.ndarray) -> np.ndarray | None:
        """Express a base_link point in the reference frame (None when the arm offset is unknown).

        Args:
            point (np.ndarray): Point in base_link.

        Returns:
            np.ndarray | None: Point in the reference frame.
        """
        return None if self.t_base_ref is None else transform_point(np.linalg.inv(self.t_base_ref), point)

    def ref_to_arm(self, point: np.ndarray) -> np.ndarray | None:
        """Express a reference-frame point in the arm base frame (None when unknown).

        Args:
            point (np.ndarray): Point in the reference frame.

        Returns:
            np.ndarray | None: Point in the arm base frame.
        """
        return None if self.t_arm_ref is None else transform_point(self.t_arm_ref, point)

    def arm_to_ref(self, point: np.ndarray) -> np.ndarray | None:
        """Express an arm-base-frame point in the reference frame (None when unknown).

        Args:
            point (np.ndarray): Point in the arm base frame.

        Returns:
            np.ndarray | None: Point in the reference frame.
        """
        return None if self.t_arm_ref is None else transform_point(np.linalg.inv(self.t_arm_ref), point)


def transform_point(t: np.ndarray, point: np.ndarray) -> np.ndarray:
    """Apply a 4x4 transform to a 3D point.

    Args:
        t (np.ndarray): 4x4 transform.
        point (np.ndarray): Point (3,).

    Returns:
        np.ndarray: Transformed point (3,).
    """
    return (t @ np.append(np.asarray(point, dtype=np.float64), 1.0))[:3]


def build_scene(setup: CameraSetup, cfg: McpServerConfig, t_ref_parent: np.ndarray) -> Scene:
    """Assemble the scene of a calibrated camera.

    Args:
        setup (CameraSetup): Calibrated camera (mount required).
        cfg (McpServerConfig): Node configuration (arm offset, floor height).
        t_ref_parent (np.ndarray): Pose of the camera's parent link in the reference frame (parent_transform).

    Returns:
        Scene: The scene.
    """
    assert setup.mount is not None
    t_ref_optical = optical_from_mount(t_ref_parent @ setup.mount.to_matrix())
    offset = cfg.arm.base_in_base_link
    t_base_arm = None if offset is None else offset_matrix(offset)
    if setup.name == "front":
        t_base_ref = np.eye(4)
        t_arm_ref = None if t_base_arm is None else np.linalg.inv(t_base_arm)
        return Scene(
            "front", FRAME_BASE, 0.0, setup.intrinsics, setup.approximate, t_ref_optical, t_base_ref, t_arm_ref
        )
    return Scene(
        "gripper",
        FRAME_ARM,
        cfg.arm.floor_z_m,
        setup.intrinsics,
        setup.approximate,
        t_ref_optical,
        t_base_arm,
        np.eye(4),
    )


def base_to_map(pose: BasePose, x: float, y: float) -> tuple[float, float]:
    """Map coordinates of a base_link point for the robot's map pose.

    Args:
        pose (BasePose): map -> base_link pose.
        x (float): Point x in base_link.
        y (float): Point y in base_link.

    Returns:
        tuple[float, float]: (x, y) in the map frame.
    """
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return pose.x + c * x - s * y, pose.y + s * x + c * y


def xyz(point: np.ndarray) -> dict[str, float]:
    """Point as a rounded {x, y, z} dict.

    Args:
        point (np.ndarray): Point (3,).

    Returns:
        dict[str, float]: Metres rounded to 1 mm.
    """
    return {"x": round(float(point[0]), 3), "y": round(float(point[1]), 3), "z": round(float(point[2]), 3)}


def ground_fields(scene: Scene, ground: np.ndarray, pose: BasePose | None) -> dict[str, Any]:
    """Ground coordinates of a floor point in every known frame plus distance and bearing from the robot base.

    The point is given in the scene's reference frame. base_link coordinates (and so the map position) exist for the
    front camera always and for the gripper camera only when ``arm.base_in_base_link`` is configured; otherwise the
    distance and bearing are measured from the arm base origin in the arm base frame.

    Args:
        scene (Scene): The camera scene.
        ground (np.ndarray): Floor point in the reference frame.
        pose (BasePose | None): Robot map pose, None when unknown.

    Returns:
        dict[str, Any]: ground_base_link / ground_arm_base / ground_map / distance_from_base_m / bearing_deg.
    """
    out: dict[str, Any] = {}
    in_base = scene.ref_to_base(ground)
    in_arm = scene.ref_to_arm(ground)
    if in_arm is not None and scene.camera == "gripper":
        out["ground_arm_base"] = xyz(in_arm)
    if in_base is not None:
        out["ground_base_link"] = xyz(in_base)
        origin = in_base
        if pose is not None:
            mx, my = base_to_map(pose, float(in_base[0]), float(in_base[1]))
            out["ground_map"] = {"x": round(mx, 3), "y": round(my, 3)}
    else:
        origin = in_arm if in_arm is not None else ground
    out["distance_from_base_m"] = round(math.hypot(float(origin[0]), float(origin[1])), 3)
    out["bearing_deg"] = round(math.degrees(math.atan2(float(origin[1]), float(origin[0]))), 2)
    return out


def ground_report(scene: Scene, camera: str, u: float, v: float, pose: BasePose | None) -> dict[str, Any]:
    """The pixel_to_ground tool result for one pixel.

    Args:
        scene (Scene): The camera scene.
        camera (str): Camera name.
        u (float): Pixel x.
        v (float): Pixel y.
        pose (BasePose | None): Robot map pose, None when unknown.

    Returns:
        dict[str, Any]: camera, pixel, ground coordinates, distance_from_base_m, bearing_deg, method,
            uncertainty_note.

    Raises:
        PixelError: When the pixel is outside the image or its ray does not hit the floor.
    """
    if not (0.0 <= u < scene.intr.width and 0.0 <= v < scene.intr.height):
        raise PixelError(f"pixel ({u:g}, {v:g}) is outside the image ({scene.intr.width}x{scene.intr.height})")
    ground = scene.ground_point(u, v)
    if ground is None:
        raise PixelError(f"pixel ({u:g}, {v:g}) does not see the floor (above the horizon or behind the camera)")
    notes = [
        "assumes a flat floor and the configured mount; error grows with distance from the camera "
        "(about 1 px of pixel error is several cm far from the camera)"
    ]
    if scene.camera == "gripper":
        notes.append("uses the measured arm joints (servo sag and backlash shift the real camera pose by a few mm)")
    if scene.approximate:
        notes.append(APPROX_NOTE)
    frame_note = (
        f"ground_arm_base is in the arm base frame (floor z = {scene.ground_z:.3f} m)"
        if scene.camera == "gripper"
        else "ground_base_link is in base_link (floor z = 0)"
    )
    return {
        "camera": camera,
        "pixel": {"u": u, "v": v},
        **ground_fields(scene, ground, pose),
        "method": GROUND_METHOD,
        "uncertainty_note": "; ".join([frame_note, *notes]),
    }
