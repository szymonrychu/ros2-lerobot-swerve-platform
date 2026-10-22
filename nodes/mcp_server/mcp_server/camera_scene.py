"""Camera scene geometry for the pixel -> ground tools: calibrated setup, reference frames, ground reports.

Reference frames (documented in the README): the front camera works in ``base_link`` (x forward, y left, z up,
z = 0 is the floor); the gripper camera works in the arm base frame (URDF ``base_link`` of the arm, z = 0 on the arm
mount plane, the floor at ``arm.floor_z_m`` < 0). When ``arm.base_in_base_link`` is configured both frames convert.

The front camera is the head RGB camera and moves with the pan/tilt head: its parent (``cameras.front.parent_frame``,
default ``head_link``) is a link of the head URDF, posed in base_link from the measured ``head_pan`` / ``head_tilt``
angles (``HeadKinematics.link_pose``), just as the gripper camera's parent is posed from the measured arm joints.
"""

import logging
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

import numpy as np
import yaml
from ros2_common.camera_geometry import (
    CameraIntrinsics,
    MountPose,
    optical_from_mount,
    project_point_to_pixel,
)
from ros2_common.camera_geometry import pixel_to_ground as geometry_pixel_to_ground

from .config import ArmBaseOffset, HeadSettings, IntrinsicsSettings, McpServerConfig
from .head_kinematics import HeadKinematics
from .ik import ArmKinematics
from .models import BasePose

NOT_CALIBRATED = (
    "camera {camera} not calibrated: set intrinsics and mount in mcp_server config (see README calibration)"
)
FRAME_BASE = "base_link"
FRAME_ARM = "arm_base"
GROUND_METHOD = (
    "pinhole ray through the pixel intersected with the horizontal plane at floor + surface_height_m (camera_geometry)"
)
ASPECT_TOLERANCE = 0.005  # relative aspect-ratio difference within which a calibration scales to the image size
logger = logging.getLogger(__name__)
# (path, mtime_ns) -> parsed file; an apply rewrites the file, so the next camera call sees the new mtime.
intrinsics_cache: dict[Path, tuple[int, CameraIntrinsics]] = {}
warned: set[tuple[str, int | None]] = set()  # (message key, mtime_ns) already logged: one warning per file state
APPROX_NOTE = "approximate intrinsics (hfov only: square pixels, centred principal point, no distortion)"


class MeasuredHead(Protocol):
    """The pan/tilt head as the camera scene needs it (head.HeadController; a Protocol to avoid an import cycle)."""

    kinematics: HeadKinematics
    settings: HeadSettings

    def measured(self) -> dict[str, float] | None:
        """Measured head angles from a fresh sample (joint name -> rad), None when missing or stale."""
        ...


class CameraNotCalibratedError(ValueError):
    """Raised when a camera lacks intrinsics and/or a mount."""


# Aspect ratio difference (relative) up to which an image size counts as a resize of the calibrated image.
PIXEL_ASPECT_TOLERANCE = 0.02


class PixelError(ValueError):
    """Raised when a pixel has no valid floor point (outside the image, above the horizon or behind the camera)."""


def pixel_scale(image_width: int | None, image_height: int | None, width: int, height: int) -> float | None:
    """Factor from the pixels of a (smaller or larger) image of the same camera to the calibrated image size.

    get_camera_image returns 384x288 by default while the camera tools work in the calibrated size (640x480): a pixel
    picked on the small image must be multiplied by this factor.

    Args:
        image_width (int | None): Width of the image the pixel came from; None = the calibrated size.
        image_height (int | None): Its height; None = the calibrated size.
        width (int): Calibrated image width.
        height (int): Calibrated image height.

    Returns:
        float | None: Scale (calibrated / given); None when no size was given (pixels are calibrated-size pixels).

    Raises:
        PixelError: When only one of the sizes is given or the aspect ratio differs (a crop, not a resize).
    """
    if image_width is None and image_height is None:
        return None
    if image_width is None or image_height is None:
        raise PixelError("give both image_width and image_height (the size of the image the pixel came from)")
    sx, sy = width / image_width, height / image_height
    if abs(sx - sy) > PIXEL_ASPECT_TOLERANCE * max(sx, sy):
        raise PixelError(
            f"image {image_width}x{image_height} does not have the aspect of the calibrated {width}x{height} image "
            "(a crop cannot be scaled): use the calibrated size, e.g. get_annotated_camera_image"
        )
    return (sx + sy) / 2.0


def pixel_input(u: float, v: float, image_width: int, image_height: int, scale: float) -> dict[str, float]:
    """Report of a pixel given in another image size.

    Args:
        u (float): Given pixel x.
        v (float): Given pixel y.
        image_width (int): Given image width.
        image_height (int): Given image height.
        scale (float): Factor to the calibrated size.

    Returns:
        dict[str, float]: {u, v, image_width, image_height, scale}.
    """
    return {"u": u, "v": v, "image_width": image_width, "image_height": image_height, "scale": round(scale, 6)}


@dataclass(frozen=True)
class CameraSetup:
    """A configured camera: intrinsics, whether they are approximate, the mount (None while uncalibrated)."""

    name: str
    intrinsics: CameraIntrinsics
    approximate: bool
    parent_frame: str
    mount: MountPose | None


def warn_once(key: str, stamp: int | None, message: str) -> None:
    """Log a warning once per (key, file state)."""
    if (key, stamp) not in warned:
        warned.add((key, stamp))
        logger.warning(message)


def scale_intrinsics(intr: CameraIntrinsics, width: int, height: int) -> CameraIntrinsics | None:
    """Intrinsics for an image of another size with the same aspect ratio.

    Args:
        intr (CameraIntrinsics): Calibrated intrinsics.
        width (int): Target image width in pixels.
        height (int): Target image height in pixels.

    Returns:
        CameraIntrinsics | None: fx, cx scaled by the width ratio, fy, cy by the height ratio (distortion unchanged);
            the input when the size already matches; None when the aspect ratios differ by more than 0.5 %.
    """
    if (intr.width, intr.height) == (width, height):
        return intr
    if abs((width / height) / (intr.width / intr.height) - 1.0) > ASPECT_TOLERANCE:
        return None
    sx, sy = width / intr.width, height / intr.height
    return intr.model_copy(
        update={
            "width": width,
            "height": height,
            "fx": intr.fx * sx,
            "cx": intr.cx * sx,
            "fy": intr.fy * sy,
            "cy": intr.cy * sy,
        }
    )


def read_calibration(path: Path) -> tuple[int, CameraIntrinsics] | None:
    """Parse a calibration yaml with an mtime cache (re-read when the file changes).

    Args:
        path (Path): camera_calibration yaml.

    Returns:
        tuple[int, CameraIntrinsics] | None: (mtime_ns, intrinsics); None when the file is missing or unreadable
            (warned once per path).
    """
    try:
        stamp = path.stat().st_mtime_ns
    except OSError:
        intrinsics_cache.pop(path, None)
        warn_once(str(path), None, f"camera calibration file {path} is missing: the camera is not calibrated")
        return None
    cached = intrinsics_cache.get(path)
    if cached is not None and cached[0] == stamp:
        return cached
    try:
        entry = (stamp, CameraIntrinsics.from_calibration_yaml(path))
    except (OSError, ValueError, KeyError, IndexError, TypeError, yaml.YAMLError) as exc:
        warn_once(
            str(path), stamp, f"camera calibration file {path} is unreadable ({exc}): the camera is not calibrated"
        )
        return None
    intrinsics_cache[path] = entry
    return entry


def load_intrinsics(settings: IntrinsicsSettings | None) -> tuple[CameraIntrinsics, bool] | None:
    """Build intrinsics from the config; a calibration file is read lazily (mtime cache) at each call.

    Args:
        settings (IntrinsicsSettings | None): Configured source.

    Returns:
        tuple[CameraIntrinsics, bool] | None: (intrinsics, approximate) or None when not configured, when the
            calibration file is missing or unreadable, or when its aspect ratio differs from the image size.
    """
    if settings is None:
        return None
    if settings.calibration_file is not None:
        entry = read_calibration(settings.calibration_file)
        if entry is None:
            return None
        stamp, intr = entry
        if settings.image_width is not None and settings.image_height is not None:
            scaled = scale_intrinsics(intr, settings.image_width, settings.image_height)
            if scaled is None:
                warn_once(
                    f"{settings.calibration_file}:aspect",
                    stamp,
                    f"camera calibration {settings.calibration_file} is {intr.width}x{intr.height}, the image is "
                    f"{settings.image_width}x{settings.image_height}: different aspect ratio, not calibrated",
                )
                return None
            intr = scaled
        return intr, False
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


def head_parent_transform(cfg: McpServerConfig, head: MeasuredHead | None) -> np.ndarray:
    """T_base_link_parent of the front (head) camera at the measured head angles.

    Args:
        cfg (McpServerConfig): Node configuration (cameras.front.parent_frame, head settings).
        head (MeasuredHead | None): Pan/tilt head with its kinematics and measured /head/joint_states.

    Returns:
        np.ndarray: 4x4 pose of cameras.front.parent_frame in base_link.

    Raises:
        ValueError: When there is no head or its joint states are missing or stale (never an identity fallback).
    """
    angles = None if head is None else head.measured()
    if head is None or angles is None:
        raise ValueError(
            f"the front camera pose needs fresh head joint states ({cfg.head.state_topic}): the head camera moves "
            "with the pan/tilt head and its joint states are missing or stale"
        )
    pan, tilt = (angles[name] for name in head.settings.joint_names)
    return head.kinematics.link_pose(pan, tilt, str(cfg.cameras.front.parent_frame))


def parent_transform(
    cfg: McpServerConfig,
    camera: str,
    kin: ArmKinematics,
    joints: dict[str, float] | None,
    head: MeasuredHead | None = None,
) -> np.ndarray:
    """T_reference_parent: the camera's parent link in its reference frame at this moment.

    Args:
        cfg (McpServerConfig): Node configuration.
        camera (str): 'gripper' (parent link pose from the measured arm joints) or 'front' (head link pose in
            base_link from the measured head angles).
        kin (ArmKinematics): Arm kinematics on the URDF.
        joints (dict[str, float] | None): Measured arm joints (required for the gripper camera).
        head (MeasuredHead | None): Pan/tilt head with fresh joint states (required for the front camera).

    Returns:
        np.ndarray: 4x4 transform.

    Raises:
        ValueError: When the needed arm joints or head joint states are missing or stale.
    """
    if camera == "front":
        return head_parent_transform(cfg, head)
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

    def ground_point(self, u: float, v: float, surface_height_m: float = 0.0) -> np.ndarray | None:
        """Point (x, y, ground_z + surface_height_m) in the reference frame seen at a pixel.

        Args:
            u (float): Pixel x.
            v (float): Pixel y.
            surface_height_m (float): Height of the surface the pixel lies on relative to the floor (m, positive
                above, negative below).

        Returns:
            np.ndarray | None: The point, or None when the ray misses the surface plane.
        """
        z = self.ground_z + surface_height_m
        hit = geometry_pixel_to_ground(self.intr, self.t_ref_optical, u, v, z)
        if hit is None:
            return None
        return np.array([hit[0], hit[1], z])

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


def ground_report(
    scene: Scene, camera: str, u: float, v: float, pose: BasePose | None, surface_height_m: float = 0.0
) -> dict[str, Any]:
    """The pixel_to_ground tool result for one pixel.

    Args:
        scene (Scene): The camera scene.
        camera (str): Camera name.
        u (float): Pixel x.
        v (float): Pixel y.
        pose (BasePose | None): Robot map pose, None when unknown.
        surface_height_m (float): Height of the surface the pixel lies on relative to the floor (m).

    Returns:
        dict[str, Any]: camera, pixel, surface_height_m, ground coordinates, distance_from_base_m, bearing_deg,
            method, uncertainty_note.

    Raises:
        PixelError: When the pixel is outside the image or its ray does not hit the floor.
    """
    if not (0.0 <= u < scene.intr.width and 0.0 <= v < scene.intr.height):
        raise PixelError(f"pixel ({u:g}, {v:g}) is outside the image ({scene.intr.width}x{scene.intr.height})")
    ground = scene.ground_point(u, v, surface_height_m)
    if ground is None:
        raise PixelError(
            f"pixel ({u:g}, {v:g}) does not see the surface at floor + {surface_height_m:g} m "
            "(above the horizon or behind the camera)"
        )
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
    if surface_height_m != 0.0:
        frame_note += f"; the pixel was intersected with a surface {surface_height_m:+g} m from the floor"
    return {
        "camera": camera,
        "pixel": {"u": u, "v": v},
        "image": {"width": scene.intr.width, "height": scene.intr.height},
        "surface_height_m": surface_height_m,
        **ground_fields(scene, ground, pose),
        "method": GROUND_METHOD,
        "uncertainty_note": "; ".join([frame_note, *notes]),
    }
