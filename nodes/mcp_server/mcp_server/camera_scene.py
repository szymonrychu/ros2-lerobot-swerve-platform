"""Camera scene geometry for the pixel -> ground tools: calibrated setup, reference frame, ground reports.

The one camera is ``front``, the D435 colour camera on the pan/tilt head. It works in ``base_link`` (x forward, y
left, z up, z = 0 is the floor); arm-frame points (the measured or a planned tool point) convert into it when
``arm.base_in_base_link`` is configured.

The camera moves with the head: its parent (``cameras.front.parent_frame``, default ``d435_color_frame``) is a link
of the head URDF, posed in base_link from the measured ``head_pan`` / ``head_tilt`` angles
(``HeadKinematics.link_pose``).
"""

import logging
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal, Protocol

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
from .models import BasePose

Camera = Literal["front"]
CAMERAS: tuple[Camera, ...] = ("front",)

NOT_CALIBRATED = (
    "camera {camera} not calibrated: set intrinsics and mount in mcp_server config (see README calibration)"
)
FRAME_BASE = "base_link"
GROUND_METHOD = (
    "pinhole ray through the pixel intersected with the horizontal plane at floor + surface_height_m (camera_geometry)"
)
# Calibrated image size shown in tool descriptions while a camera has no readable intrinsics (the D435 colour stream
# before its first CameraInfo).
NOMINAL_IMAGE_SIZE: dict[Camera, tuple[int, int]] = {"front": (640, 360)}
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


@dataclass(frozen=True)
class CameraInfoData:
    """The fields of a camera's latest sensor_msgs/CameraInfo that the camera tools use.

    Attributes:
        width (int): Image width (px).
        height (int): Image height (px).
        k (tuple[float, ...]): Row-major 3x3 camera matrix (9 values).
        d (tuple[float, ...]): Distortion coefficients.
        distortion_model (str): CameraInfo.distortion_model ('plumb_bob' for the D435 colour stream).
        stamp (float): Header stamp (ROS time, s).
    """

    width: int
    height: int
    k: tuple[float, ...]
    d: tuple[float, ...]
    distortion_model: str
    stamp: float


class CameraInfoSource(Protocol):
    """Where the latest CameraInfo of a camera comes from (tool_context.RobotApi, implemented by ros_iface.RosRobot)."""

    def camera_info(self, camera: str) -> CameraInfoData | None:
        """Latest CameraInfo of the camera; None without a camera_info_topic or before a first message."""
        ...


class CameraNotCalibratedError(ValueError):
    """Raised when a camera lacks intrinsics and/or a mount."""


# Aspect ratio difference (relative) up to which an image size counts as a resize of the calibrated image.
PIXEL_ASPECT_TOLERANCE = 0.02


class PixelError(ValueError):
    """Raised when a pixel has no valid floor point (outside the image, above the horizon or behind the camera)."""


def default_picture_size(width: int, height: int, longest_px: int) -> tuple[int, int]:
    """Size of the default get_camera_image picture of a camera: `longest_px` on the longest side, same aspect.

    Args:
        width (int): Calibrated image width.
        height (int): Calibrated image height.
        longest_px (int): Longest side of the default picture (limits.default_camera_px).

    Returns:
        tuple[int, int]: (width, height), e.g. (384, 216) for a 1152x648 camera and 384.
    """
    factor = longest_px / max(width, height)
    return round(width * factor), round(height * factor)


def picture_sizes(cfg: McpServerConfig) -> dict[Camera, tuple[int, int]]:
    """Default get_camera_image picture size of each camera, from its calibrated aspect.

    Args:
        cfg (McpServerConfig): Node configuration (intrinsics, limits.default_camera_px).

    Returns:
        dict[Camera, tuple[int, int]]: Camera name -> (width, height); a camera without readable intrinsics uses its
            nominal frame size (NOMINAL_IMAGE_SIZE) for the aspect.
    """
    sizes: dict[Camera, tuple[int, int]] = {}
    for name in CAMERAS:
        try:
            intr = camera_setup(cfg, name, need_mount=False).intrinsics
            size = (intr.width, intr.height)
        except CameraNotCalibratedError:
            size = NOMINAL_IMAGE_SIZE[name]
        sizes[name] = default_picture_size(*size, cfg.limits.default_camera_px)
    return sizes


def pixel_scale(
    image_width: int | None, image_height: int | None, width: int, height: int, default_px: int | None = None
) -> float | None:
    """Factor from the pixels of a (smaller or larger) image of the same camera to the calibrated image size.

    get_camera_image returns a small picture by default (384x288 for a 4:3 camera, 384x216 for the 16:9 front camera)
    while the camera tools work in the calibrated size: a pixel picked on the small image must be multiplied by this
    factor.

    Args:
        image_width (int | None): Width of the image the pixel came from; None = the calibrated size.
        image_height (int | None): Its height; None = the calibrated size.
        width (int): Calibrated image width.
        height (int): Calibrated image height.
        default_px (int | None): limits.default_camera_px; when given, an aspect error names the camera's default
            picture size.

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
        hint = ""
        if default_px is not None:
            default = default_picture_size(width, height, default_px)
            hint = f"the default get_camera_image picture of this camera is {default[0]}x{default[1]}; "
        raise PixelError(
            f"image {image_width}x{image_height} does not have the aspect of the calibrated {width}x{height} image "
            f"(a crop cannot be scaled): {hint}use that size or the calibrated size, e.g. get_annotated_camera_image"
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


def fit_image_size(
    intr: CameraIntrinsics, settings: IntrinsicsSettings, source: str, stamp: int | None
) -> CameraIntrinsics | None:
    """Scale intrinsics to the configured image size (image_width / image_height), when one is set.

    Args:
        intr (CameraIntrinsics): Intrinsics of the source.
        settings (IntrinsicsSettings): Configured source and image size.
        source (str): Calibration file or CameraInfo topic, for the warning.
        stamp (int | None): Source state (file mtime_ns) the aspect warning is logged once for.

    Returns:
        CameraIntrinsics | None: The (scaled) intrinsics; None when the aspect ratio differs from the image size
            (warned once per source state).
    """
    if settings.image_width is None or settings.image_height is None:
        return intr
    scaled = scale_intrinsics(intr, settings.image_width, settings.image_height)
    if scaled is None:
        warn_once(
            f"{source}:aspect",
            stamp,
            f"camera calibration {source} is {intr.width}x{intr.height}, the image is "
            f"{settings.image_width}x{settings.image_height}: different aspect ratio, not calibrated",
        )
    return scaled


def intrinsics_from_camera_info(topic: str, info: CameraInfoData | None) -> CameraIntrinsics | None:
    """Factory intrinsics from a CameraInfo (CameraIntrinsics.from_camera_info).

    Args:
        topic (str): Configured camera_info_topic, for the warning.
        info (CameraInfoData | None): Latest CameraInfo, None when none was received.

    Returns:
        CameraIntrinsics | None: The intrinsics; None without a CameraInfo or for an invalid one (zero K, an
            unsupported distortion model, a zero image size; warned once per topic).
    """
    if info is None:
        warn_once(f"{topic}:none", None, f"no CameraInfo on {topic} yet: the camera is not calibrated")
        return None
    try:
        if info.width <= 0 or info.height <= 0:
            raise ValueError(f"image size {info.width}x{info.height}")
        return CameraIntrinsics.from_camera_info(info.width, info.height, info.k, info.d, info.distortion_model)
    except ValueError as exc:
        warn_once(f"{topic}:invalid", None, f"CameraInfo on {topic} is invalid ({exc}): the camera is not calibrated")
        return None


def load_intrinsics(
    settings: IntrinsicsSettings | None, camera_info: CameraInfoData | None = None
) -> tuple[CameraIntrinsics, bool] | None:
    """Build intrinsics from the config; a calibration file is read lazily (mtime cache) at each call, a
    camera_info_topic source uses the camera's latest CameraInfo.

    Args:
        settings (IntrinsicsSettings | None): Configured source.
        camera_info (CameraInfoData | None): Latest CameraInfo of the camera (camera_info_topic source); None when
            none was received.

    Returns:
        tuple[CameraIntrinsics, bool] | None: (intrinsics, approximate) or None when not configured, when the
            calibration file is missing or unreadable, when there is no valid CameraInfo, or when the aspect ratio
            differs from the image size.
    """
    if settings is None:
        return None
    if settings.calibration_file is not None:
        entry = read_calibration(settings.calibration_file)
        if entry is None:
            return None
        stamp, intr = entry
        fitted = fit_image_size(intr, settings, str(settings.calibration_file), stamp)
        return None if fitted is None else (fitted, False)
    if settings.camera_info_topic is not None:
        factory = intrinsics_from_camera_info(settings.camera_info_topic, camera_info)
        if factory is None:
            return None
        fitted = fit_image_size(factory, settings, settings.camera_info_topic, None)
        return None if fitted is None else (fitted, False)
    assert settings.hfov_deg is not None and settings.width is not None and settings.height is not None
    return CameraIntrinsics.from_hfov(settings.width, settings.height, settings.hfov_deg), True


def camera_setup(
    cfg: McpServerConfig, camera: Camera, need_mount: bool = True, source: CameraInfoSource | None = None
) -> CameraSetup:
    """Resolve a camera's calibration from the config.

    Args:
        cfg (McpServerConfig): Node configuration.
        camera (Camera): Camera name ('front').
        need_mount (bool): Require the mount pose (False for capture and the solver, which determine it).
        source (CameraInfoSource | None): The robot, asked for the camera's latest CameraInfo when its intrinsics
            come from a camera_info_topic; None when no CameraInfo is available (such a camera is not calibrated).

    Returns:
        CameraSetup: The setup.

    Raises:
        CameraNotCalibratedError: When intrinsics (or the mount, if needed) are missing.
    """
    settings = getattr(cfg.cameras, camera)
    info = None
    if source is not None and settings.intrinsics is not None and settings.intrinsics.camera_info_topic is not None:
        info = source.camera_info(camera)
    loaded = load_intrinsics(settings.intrinsics, info)
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
    return head_parent_transform_at(cfg, head, pan, tilt)


def head_parent_transform_at(cfg: McpServerConfig, head: MeasuredHead, pan: float, tilt: float) -> np.ndarray:
    """T_base_link_parent of the front (head) camera at given head angles (the pose at an image stamp).

    Args:
        cfg (McpServerConfig): Node configuration (cameras.front.parent_frame).
        head (MeasuredHead): Pan/tilt head with its kinematics (the applied chain).
        pan (float): head_pan (rad, + left).
        tilt (float): head_tilt (rad, + down).

    Returns:
        np.ndarray: 4x4 pose of cameras.front.parent_frame in base_link.
    """
    return head.kinematics.link_pose(pan, tilt, str(cfg.cameras.front.parent_frame))


@dataclass(frozen=True)
class Scene:
    """A camera at one moment: its optical pose in the reference frame plus conversions to and from the other frames.

    Attributes:
        camera: Camera name.
        frame_name: Reference frame name ('base_link').
        ground_z: Floor height in the reference frame (m).
        intr: Intrinsics at the working image size.
        approximate: Intrinsics are hfov-based.
        t_ref_optical: 4x4 pose of the optical frame in the reference frame.
        t_base_ref: 4x4 pose of the reference frame in base_link, None when unknown.
        t_arm_ref: 4x4 pose of the reference frame in the arm base frame, None when unknown (arm.base_in_base_link
            not configured).
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


def build_scene(setup: CameraSetup, cfg: McpServerConfig, t_base_parent: np.ndarray) -> Scene:
    """Assemble the base_link scene of a calibrated camera.

    Args:
        setup (CameraSetup): Calibrated camera (mount required).
        cfg (McpServerConfig): Node configuration (arm offset, for arm-frame points).
        t_base_parent (np.ndarray): Pose of the camera's parent link in base_link (head_parent_transform).

    Returns:
        Scene: The scene; t_arm_ref is None while arm.base_in_base_link is not configured.

    Raises:
        ValueError: For a camera that is not one of CAMERAS (only those hang on the head and work in base_link).
    """
    if setup.name not in CAMERAS:
        raise ValueError(f"camera {setup.name!r} has no scene: the camera tools use {', '.join(CAMERAS)}")
    assert setup.mount is not None
    t_ref_optical = optical_from_mount(t_base_parent @ setup.mount.to_matrix())
    offset = cfg.arm.base_in_base_link
    t_arm_ref = None if offset is None else np.linalg.inv(offset_matrix(offset))
    return Scene(setup.name, FRAME_BASE, 0.0, setup.intrinsics, setup.approximate, t_ref_optical, np.eye(4), t_arm_ref)


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
    """Ground coordinates of a floor point in base_link and the map plus distance and bearing from the robot base.

    The point is given in the scene's reference frame; a scene whose frame has no known base_link pose reports only
    the distance and bearing, measured from its own origin.

    Args:
        scene (Scene): The camera scene.
        ground (np.ndarray): Floor point in the reference frame.
        pose (BasePose | None): Robot map pose, None when unknown.

    Returns:
        dict[str, Any]: ground_base_link / ground_map / distance_from_base_m / bearing_deg.
    """
    out: dict[str, Any] = {}
    in_base = scene.ref_to_base(ground)
    origin = ground
    in_arm = scene.ref_to_arm(ground)
    if in_arm is not None:
        out["ground_arm"] = {"frame": "arm", **xyz(in_arm)}
    if in_base is not None:
        out["ground_base_link"] = xyz(in_base)
        origin = in_base
        if pose is not None:
            mx, my = base_to_map(pose, float(in_base[0]), float(in_base[1]))
            out["ground_map"] = {"x": round(mx, 3), "y": round(my, 3)}
    out["distance_from_base_m"] = round(math.hypot(float(origin[0]), float(origin[1])), 3)
    out["bearing_deg"] = round(math.degrees(math.atan2(float(origin[1]), float(origin[0]))), 2)
    return out


def ground_report(
    scene: Scene, camera: Camera, u: float, v: float, pose: BasePose | None, surface_height_m: float = 0.0
) -> dict[str, Any]:
    """The pixel_to_ground tool result for one pixel.

    Args:
        scene (Scene): The camera scene.
        camera (Camera): Camera name.
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
    if scene.approximate:
        notes.append(APPROX_NOTE)
    frame_note = "ground_base_link is in base_link (floor z = 0)"
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
