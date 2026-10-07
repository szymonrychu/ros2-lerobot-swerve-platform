"""Camera geometry: pixel <-> ray <-> ground-plane maths and a mount-pose calibration solver.

Conventions (ROS REP-103):

- ``MountPose`` describes the camera BODY frame (x forward, y left, z up) in ``parent_frame``.
  Orientation is fixed-axis roll/pitch/yaw (rotation about fixed x, then y, then z, like URDF ``rpy``).
- The OPTICAL frame (z forward, x right, y down) is the body frame rotated by the standard REP-103
  camera_link -> optical rotation (roll -pi/2, yaw -pi/2). ``optical_from_mount`` applies it.
- All ``T_*`` arguments are 4x4 homogeneous transforms ``T_a_b`` mapping points in frame b into frame a.
"""

import math
from pathlib import Path

import cv2
import numpy as np
import yaml
from pydantic import BaseModel, Field
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

MOUNT_PARAM_COUNT = 6
MIN_SOLVER_SAMPLES = 3
BEHIND_CAMERA_PENALTY_PX = 1.0e4
MIN_FORWARD_DEPTH = 1.0e-9
PARALLEL_EPS = 1.0e-12
UNDISTORT_CRITERIA = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 100, 1.0e-12)
# REP-103 camera_link (x fwd, y left, z up) -> optical (z fwd, x right, y down): roll -pi/2, yaw -pi/2.
OPTICAL_ROLL = -math.pi / 2
OPTICAL_YAW = -math.pi / 2


class CameraIntrinsics(BaseModel):
    """Pinhole intrinsics with optional plumb_bob distortion.

    Attributes:
        width: Image width in pixels.
        height: Image height in pixels.
        fx: Focal length x in pixels.
        fy: Focal length y in pixels.
        cx: Principal point x in pixels.
        cy: Principal point y in pixels.
        distortion: Plumb_bob coefficients [k1, k2, p1, p2, k3]; empty means no distortion.
    """

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: list[float] = Field(default_factory=list)

    @classmethod
    def from_calibration_yaml(cls, path: str | Path) -> "CameraIntrinsics":
        """Load intrinsics from a ROS camera_info yaml as written by camera_calibration.

        Args:
            path: Path to the yaml file.

        Returns:
            CameraIntrinsics: Parsed intrinsics.
        """
        data = yaml.safe_load(Path(path).read_text())
        k = data["camera_matrix"]["data"]
        dist = data.get("distortion_coefficients", {}).get("data", [])
        return cls(
            width=int(data["image_width"]),
            height=int(data["image_height"]),
            fx=float(k[0]),
            fy=float(k[4]),
            cx=float(k[2]),
            cy=float(k[5]),
            distortion=[float(d) for d in dist],
        )

    @classmethod
    def from_hfov(cls, width: int, height: int, hfov_deg: float) -> "CameraIntrinsics":
        """Approximate intrinsics from a horizontal field of view.

        This is an approximation only: square pixels, principal point at the image centre, no distortion.

        Args:
            width: Image width in pixels.
            height: Image height in pixels.
            hfov_deg: Horizontal field of view in degrees.

        Returns:
            CameraIntrinsics: Approximate intrinsics.
        """
        fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        return cls(width=width, height=height, fx=fx, fy=fx, cx=width / 2.0, cy=height / 2.0)

    def matrix(self) -> np.ndarray:
        """Return the 3x3 camera matrix.

        Returns:
            np.ndarray: [[fx, 0, cx], [0, fy, cy], [0, 0, 1]].
        """
        return np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])


class MountPose(BaseModel):
    """Camera body-frame pose in a parent frame (REP-103 body frame, fixed-axis RPY).

    Attributes:
        parent_frame: Name of the parent frame.
        x: Translation x in meters.
        y: Translation y in meters.
        z: Translation z in meters.
        roll: Rotation about fixed x in radians.
        pitch: Rotation about fixed y in radians.
        yaw: Rotation about fixed z in radians.
    """

    parent_frame: str
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float

    def to_matrix(self) -> np.ndarray:
        """Return T_parent_mount.

        Returns:
            np.ndarray: 4x4 homogeneous transform.
        """
        t = np.eye(4)
        t[:3, :3] = Rotation.from_euler("xyz", [self.roll, self.pitch, self.yaw]).as_matrix()
        t[:3, 3] = [self.x, self.y, self.z]
        return t


class CameraModel(BaseModel):
    """A named camera with optional intrinsics and mount.

    Attributes:
        name: Camera name.
        intrinsics: Intrinsics, if known.
        mount: Mount pose, if known.
    """

    name: str
    intrinsics: CameraIntrinsics | None = None
    mount: MountPose | None = None

    @property
    def calibrated(self) -> bool:
        """True when both intrinsics and mount are set.

        Returns:
            bool: Calibration state.
        """
        return self.intrinsics is not None and self.mount is not None


def optical_from_mount(t_parent_mount: np.ndarray) -> np.ndarray:
    """Convert a body-frame pose into the optical-frame pose.

    The mount pose describes the camera BODY frame (x forward, y left, z up, REP-103); the optical frame
    is rotated by the standard camera_link -> optical rotation (roll -pi/2, yaw -pi/2).
    Composition: T_frame_parent @ T_parent_mount @ T_mount_optical; pass ``T_frame_parent @ T_parent_mount``
    here to get T_frame_optical.

    Args:
        t_parent_mount: 4x4 transform of the body frame in the reference frame.

    Returns:
        np.ndarray: 4x4 transform of the optical frame in the same reference frame.
    """
    t_mount_optical = np.eye(4)
    t_mount_optical[:3, :3] = Rotation.from_euler("xyz", [OPTICAL_ROLL, 0.0, OPTICAL_YAW]).as_matrix()
    return t_parent_mount @ t_mount_optical


def undistort_pixel(intr: CameraIntrinsics, u: float, v: float) -> tuple[float, float]:
    """Convert a pixel to normalized image coordinates, removing distortion if present.

    Args:
        intr: Camera intrinsics.
        u: Pixel x.
        v: Pixel y.

    Returns:
        tuple[float, float]: Normalized (x_n, y_n).
    """
    if intr.distortion:
        pts = np.array([[[u, v]]], dtype=np.float64)
        out = cv2.undistortPointsIter(
            pts, intr.matrix(), np.array(intr.distortion, dtype=np.float64), None, None, UNDISTORT_CRITERIA
        )
        return float(out[0, 0, 0]), float(out[0, 0, 1])
    return (u - intr.cx) / intr.fx, (v - intr.cy) / intr.fy


def pixel_to_ray_camera(intr: CameraIntrinsics, u: float, v: float) -> np.ndarray:
    """Unit ray through a pixel in the camera optical frame (z forward, x right, y down).

    Args:
        intr: Camera intrinsics.
        u: Pixel x.
        v: Pixel y.

    Returns:
        np.ndarray: Unit 3-vector.
    """
    x_n, y_n = undistort_pixel(intr, u, v)
    ray = np.array([x_n, y_n, 1.0])
    return ray / np.linalg.norm(ray)


def camera_ray_in_frame(t_frame_camera_optical: np.ndarray, ray_cam: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Express a camera-frame ray in another frame.

    Args:
        t_frame_camera_optical: 4x4 pose of the optical frame in the target frame.
        ray_cam: Unit ray in the optical frame.

    Returns:
        tuple[np.ndarray, np.ndarray]: (origin3, unit direction3) in the target frame.
    """
    origin = t_frame_camera_optical[:3, 3].copy()
    direction = t_frame_camera_optical[:3, :3] @ ray_cam
    return origin, direction / np.linalg.norm(direction)


def intersect_ground(origin: np.ndarray, direction: np.ndarray, ground_z: float) -> np.ndarray | None:
    """Intersect a ray with the horizontal plane z = ground_z.

    Args:
        origin: Ray origin (3,).
        direction: Ray direction (3,).
        ground_z: Plane height.

    Returns:
        np.ndarray | None: Intersection point (3,), or None if parallel or behind the origin.
    """
    if abs(direction[2]) < PARALLEL_EPS:
        return None
    s = (ground_z - origin[2]) / direction[2]
    if s <= 0.0:
        return None
    return origin + s * direction


def pixel_to_ground(
    intr: CameraIntrinsics, t_frame_optical: np.ndarray, u: float, v: float, ground_z: float
) -> tuple[float, float] | None:
    """Estimate the ground (x, y) seen at a pixel.

    Args:
        intr: Camera intrinsics.
        t_frame_optical: 4x4 pose of the optical frame in the ground frame.
        u: Pixel x.
        v: Pixel y.
        ground_z: Ground plane height in the frame.

    Returns:
        tuple[float, float] | None: (x, y) in the frame, or None when the ray misses the ground.
    """
    origin, direction = camera_ray_in_frame(t_frame_optical, pixel_to_ray_camera(intr, u, v))
    hit = intersect_ground(origin, direction, ground_z)
    if hit is None:
        return None
    return float(hit[0]), float(hit[1])


def project_raw(intr: CameraIntrinsics, t_frame_optical: np.ndarray, point3: np.ndarray) -> tuple[float, float, float]:
    """Project a point ignoring image bounds.

    Args:
        intr: Camera intrinsics.
        t_frame_optical: 4x4 pose of the optical frame in the point's frame.
        point3: Point (3,) in the frame.

    Returns:
        tuple[float, float, float]: (u, v, depth) where depth is z in the optical frame.
    """
    p_cam = np.linalg.inv(t_frame_optical) @ np.append(np.asarray(point3, dtype=np.float64), 1.0)
    depth = float(p_cam[2])
    dist = np.array(intr.distortion, dtype=np.float64) if intr.distortion else np.zeros(5)
    pix, _ = cv2.projectPoints(p_cam[:3].reshape(1, 1, 3), np.zeros(3), np.zeros(3), intr.matrix(), dist)
    return float(pix[0, 0, 0]), float(pix[0, 0, 1]), depth


def project_point_to_pixel(
    intr: CameraIntrinsics, t_frame_optical: np.ndarray, point3: np.ndarray
) -> tuple[float, float] | None:
    """Project a 3D point into the image.

    Args:
        intr: Camera intrinsics.
        t_frame_optical: 4x4 pose of the optical frame in the point's frame.
        point3: Point (3,) in the frame.

    Returns:
        tuple[float, float] | None: (u, v), or None if behind the camera or outside the image.
    """
    u, v, depth = project_raw(intr, t_frame_optical, point3)
    if depth <= MIN_FORWARD_DEPTH:
        return None
    if not (0.0 <= u < intr.width and 0.0 <= v < intr.height):
        return None
    return u, v


def solve_mount_pose(
    samples: list[tuple[np.ndarray, tuple[float, float], tuple[float, float, float]]],
    intr: CameraIntrinsics,
    initial: MountPose,
) -> tuple[MountPose, float]:
    """Fit the 6 mount parameters by minimizing reprojection error.

    Args:
        samples: Each (T_frame_parent 4x4 at capture time, pixel (u, v), ground point (x, y, z) in the frame).
        intr: Camera intrinsics.
        initial: Starting mount pose (its parent_frame is kept).

    Returns:
        tuple[MountPose, float]: (solved mount, RMS Euclidean reprojection error per sample in pixels).

    Raises:
        ValueError: If fewer than 3 samples are given.
    """
    if len(samples) < MIN_SOLVER_SAMPLES:
        raise ValueError(f"need at least {MIN_SOLVER_SAMPLES} samples, got {len(samples)}")

    def residuals(params: np.ndarray) -> np.ndarray:
        mount = initial.model_copy(update=dict(zip(("x", "y", "z", "roll", "pitch", "yaw"), params.tolist())))
        t_parent_optical = optical_from_mount(mount.to_matrix())
        out = []
        for t_frame_parent, (u, v), ground in samples:
            pu, pv, depth = project_raw(intr, np.asarray(t_frame_parent) @ t_parent_optical, np.array(ground))
            if depth <= MIN_FORWARD_DEPTH:
                out.extend([BEHIND_CAMERA_PENALTY_PX, BEHIND_CAMERA_PENALTY_PX])
            else:
                out.extend([pu - u, pv - v])
        return np.array(out)

    x0 = np.array([initial.x, initial.y, initial.z, initial.roll, initial.pitch, initial.yaw])
    result = least_squares(residuals, x0, x_scale=np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]))
    solved = initial.model_copy(update=dict(zip(("x", "y", "z", "roll", "pitch", "yaw"), result.x.tolist())))
    rms = float(math.sqrt(np.mean(result.fun**2) * 2.0))
    return solved, rms
