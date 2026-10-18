"""Shared synthetic scenes for the head RGB-D calibration and validation tests.

A ChArUco board is rendered through a known pinhole (K, optional plumb_bob D) at a known pose for an RGB camera
and a ToF camera related by a known extrinsic, so calibration results can be compared with ground truth.
"""

from dataclasses import dataclass, field

import cv2
import numpy as np

from scripts.head_rgbd_calibration.charuco import make_board
from scripts.head_rgbd_calibration.config import BoardConfig

BOARD_PX_PER_M = 3000.0
MARGIN_PX = 40
SUPERSAMPLE = 2
BACKGROUND_GREY = 120
RGB_SIZE = (576, 324)
TOF_SIZE = (240, 180)
DEFAULT_TOF_OFFSET_M = 0.05
DEFAULT_TOF_YAW_DEG = 1.0
TOF_AMPLITUDE_PEAK = 6000.0
TOF_AMPLITUDE_FLOOR = 300.0
TOF_NOISE_SIGMA = 60.0
RGB_NOISE_SIGMA = 1.5
MM_PER_M = 1000.0
MAX_UINT8 = 255.0
MAX_UINT16 = 65535.0
RENDER_SEED = 0


@dataclass(frozen=True)
class SyntheticCamera:
    """Pinhole camera: size is (width, height), k the 3x3 matrix, d the plumb_bob coefficients (k1 k2 p1 p2 k3)."""

    size: tuple[int, int]
    k: np.ndarray
    d: np.ndarray = field(default_factory=lambda: np.zeros(5))


def make_camera(size: tuple[int, int], focal: float, d: np.ndarray | None = None) -> SyntheticCamera:
    """Camera with fx=fy=focal and the principal point at the image centre."""
    k = np.array([[focal, 0, size[0] / 2], [0, focal, size[1] / 2], [0, 0, 1]], dtype=np.float64)
    return SyntheticCamera(size, k, np.zeros(5) if d is None else np.asarray(d, dtype=np.float64))


def rgb_camera(d: np.ndarray | None = None) -> SyntheticCamera:
    """RGB mapping camera: 576x324, fx=fy=432, cx=288, cy=162."""
    return make_camera(RGB_SIZE, 432.0, d)


def tof_camera(d: np.ndarray | None = None) -> SyntheticCamera:
    """ToF camera: 240x180, fx=fy=191, cx=120, cy=90."""
    return make_camera(TOF_SIZE, 191.0, d)


@dataclass(frozen=True)
class Pose:
    """Rigid transform p_cam = r @ p_board + t (board frame to camera frame)."""

    r: np.ndarray
    t: np.ndarray

    @property
    def rvec(self) -> np.ndarray:
        """Rodrigues vector of r."""
        return cv2.Rodrigues(self.r)[0].reshape(3)


@dataclass(frozen=True)
class Extrinsic:
    """ToF optical frame expressed in the RGB optical frame: p_rgb = r @ p_tof + t."""

    r: np.ndarray
    t: np.ndarray

    def tof_pose(self, rgb_pose: Pose) -> Pose:
        """Board pose in the ToF frame given the board pose in the RGB frame."""
        return Pose(self.r.T @ rgb_pose.r, self.r.T @ (rgb_pose.t - self.t))


def default_extrinsic(offset_m: float = DEFAULT_TOF_OFFSET_M, yaw_deg: float = DEFAULT_TOF_YAW_DEG) -> Extrinsic:
    """ToF offset_m to the left of RGB (optical x is right, so -x) and yawed by yaw_deg about the optical y axis."""
    r = cv2.Rodrigues(np.array([0.0, np.deg2rad(yaw_deg), 0.0]))[0]
    return Extrinsic(r, np.array([-offset_m, 0.0, 0.0]))


def rotation(rx_deg: float, ry_deg: float, rz_deg: float) -> np.ndarray:
    """Rotation matrix Rx @ Ry @ Rz from angles in degrees."""
    rx, ry, rz = (
        cv2.Rodrigues(np.deg2rad(a) * axis)[0] for a, axis in zip((rx_deg, ry_deg, rz_deg), np.eye(3), strict=True)
    )
    return rx @ ry @ rz


def board_size_m(board_cfg: BoardConfig) -> tuple[float, float]:
    """Board (width, height) in metres."""
    mm = board_cfg.square_mm / MM_PER_M
    return board_cfg.squares_x * mm, board_cfg.squares_y * mm


def centred_pose(board_cfg: BoardConfig, r: np.ndarray, centre: np.ndarray) -> Pose:
    """Pose of the board origin (its top-left corner) such that the board centre sits at centre in the camera frame."""
    width, height = board_size_m(board_cfg)
    return Pose(r, np.asarray(centre, dtype=np.float64) - r @ np.array([width / 2, height / 2, 0.0]))


def board_poses(
    count: int,
    camera: SyntheticCamera,
    board_cfg: BoardConfig | None = None,
    seed: int = 1,
    distance_m: tuple[float, float] = (0.5, 1.5),
    max_tilt_deg: float = 40.0,
    lateral_fraction: float = 0.3,
) -> list[Pose]:
    """Board poses spanning distance_m and +-max_tilt_deg tilt.

    Args:
        count: Number of poses.
        camera: Camera the board must stay in front of.
        board_cfg: Board definition (default BoardConfig()).
        seed: Random seed.
        distance_m: (min, max) distance of the board centre along the optical axis.
        max_tilt_deg: Maximum tilt about each of the board x and y axes.
        lateral_fraction: Board centre offset as a fraction of the half field of view at that distance.

    Returns:
        count board poses (board frame to camera frame).
    """
    board_cfg = board_cfg or BoardConfig()
    rng = np.random.default_rng(seed)
    poses = []
    for _ in range(count):
        z = rng.uniform(*distance_m)
        x = rng.uniform(-1, 1) * lateral_fraction * z * camera.k[0, 2] / camera.k[0, 0]
        y = rng.uniform(-1, 1) * lateral_fraction * z * camera.k[1, 2] / camera.k[1, 1]
        tilt_x, tilt_y = rng.uniform(-max_tilt_deg, max_tilt_deg, 2)
        r = rotation(tilt_x, tilt_y, rng.uniform(-20, 20))
        poses.append(centred_pose(board_cfg, r, np.array([x, y, z])))
    return poses


def board_image(board_cfg: BoardConfig) -> np.ndarray:
    """Board texture at BOARD_PX_PER_M with a white margin of MARGIN_PX (uint8)."""
    width, height = board_size_m(board_cfg)
    size = (round(width * BOARD_PX_PER_M), round(height * BOARD_PX_PER_M))
    return make_board(board_cfg).generateImage(
        (size[0] + 2 * MARGIN_PX, size[1] + 2 * MARGIN_PX), marginSize=MARGIN_PX, borderBits=1
    )


def plane_maps(camera: SyntheticCamera, pose: Pose, scale: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Per output pixel (at scale x resolution): board texture coordinates and the plane depth along the optical axis.

    Returns:
        (map_x, map_y, depth_m), each (height*scale, width*scale); depth is NaN where the ray misses the plane.
    """
    width, height = camera.size
    u, v = np.meshgrid((np.arange(width * scale) + 0.5) / scale - 0.5, (np.arange(height * scale) + 0.5) / scale - 0.5)
    pixels = np.stack([u, v], axis=-1).reshape(-1, 1, 2).astype(np.float64)
    norm = cv2.undistortPoints(pixels, camera.k, camera.d).reshape(-1, 2)
    rays = np.concatenate([norm, np.ones((len(norm), 1))], axis=1)
    normal = pose.r[:, 2]
    denom = rays @ normal
    with np.errstate(divide="ignore", invalid="ignore"):
        depth = (normal @ pose.t) / denom
    points = rays * depth[:, None]
    local = (points - pose.t) @ pose.r
    shape = (height * scale, width * scale)
    map_x = (local[:, 0] * BOARD_PX_PER_M + MARGIN_PX - 0.5).reshape(shape).astype(np.float32)
    map_y = (local[:, 1] * BOARD_PX_PER_M + MARGIN_PX - 0.5).reshape(shape).astype(np.float32)
    depth = np.where(depth > 0, depth, np.nan).reshape(shape)
    return map_x, map_y, depth


def render_grey(camera: SyntheticCamera, pose: Pose, board_cfg: BoardConfig | None = None) -> np.ndarray:
    """Board seen by camera as a float image in [0, 1] (anti-aliased by supersampling)."""
    board_cfg = board_cfg or BoardConfig()
    map_x, map_y, depth = plane_maps(camera, pose, SUPERSAMPLE)
    texture = board_image(board_cfg).astype(np.float32) / MAX_UINT8
    grey = cv2.remap(
        texture, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=BACKGROUND_GREY / MAX_UINT8
    )
    grey[np.isnan(depth)] = BACKGROUND_GREY / MAX_UINT8
    return cv2.resize(grey, camera.size, interpolation=cv2.INTER_AREA)


def render_rgb(
    camera: SyntheticCamera, pose: Pose, board_cfg: BoardConfig | None = None, seed: int = RENDER_SEED
) -> np.ndarray:
    """RGB view of the board: uint8 BGR image with gaussian noise."""
    rng = np.random.default_rng(seed)
    grey = render_grey(camera, pose, board_cfg) * MAX_UINT8 + rng.normal(
        0, RGB_NOISE_SIGMA, (camera.size[1], camera.size[0])
    )
    return cv2.cvtColor(np.clip(grey, 0, MAX_UINT8).astype(np.uint8), cv2.COLOR_GRAY2BGR)


def render_tof_amplitude(
    camera: SyntheticCamera, pose: Pose, board_cfg: BoardConfig | None = None, seed: int = RENDER_SEED
) -> np.ndarray:
    """ToF amplitude view of the board: uint16 with gaussian noise (white squares bright, black dark)."""
    rng = np.random.default_rng(seed)
    grey = render_grey(camera, pose, board_cfg)
    amplitude = TOF_AMPLITUDE_FLOOR + grey * (TOF_AMPLITUDE_PEAK - TOF_AMPLITUDE_FLOOR)
    amplitude += rng.normal(0, TOF_NOISE_SIGMA, grey.shape)
    return np.clip(amplitude, 0, MAX_UINT16).astype(np.uint16)


def render_depth(camera: SyntheticCamera, pose: Pose, board_cfg: BoardConfig | None = None) -> np.ndarray:
    """Depth of the board plane as uint16 millimetres (0 outside the board, margin included)."""
    board_cfg = board_cfg or BoardConfig()
    map_x, map_y, depth = plane_maps(camera, pose, 1)
    inside = (
        (map_x >= 0)
        & (map_y >= 0)
        & (map_x <= board_image(board_cfg).shape[1] - 1)
        & (map_y <= board_image(board_cfg).shape[0] - 1)
    )
    millimetres = np.where(inside & np.isfinite(depth), depth * MM_PER_M, 0)
    return np.clip(np.rint(millimetres), 0, MAX_UINT16).astype(np.uint16)
