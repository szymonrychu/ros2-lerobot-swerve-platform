"""Synthetic marker scenes for tests and the bench: QR symbols warped through a pinhole camera."""

import math

import cv2
import numpy as np
import segno

QUIET_MODULES = 4
HIRES_PX_PER_MODULE = 24
SUPERSAMPLE = 3
BACKGROUND_GREY = 128
STREAM_SIZE = (1152, 648)
STREAM_K = np.array([[887.0, 0.0, 576.0], [0.0, 887.0, 324.0], [0.0, 0.0, 1.0]])
# D435 colour stream (design D2 profile 640x360) with its 69 deg horizontal FOV (design D5), square pixels.
D435_COLOR_SIZE = (640, 360)
D435_COLOR_HFOV_DEG = 69.0
D435_COLOR_FOCAL_PX = D435_COLOR_SIZE[0] / 2 / math.tan(math.radians(D435_COLOR_HFOV_DEG / 2))
D435_COLOR_K = np.array(
    [
        [D435_COLOR_FOCAL_PX, 0.0, D435_COLOR_SIZE[0] / 2],
        [0.0, D435_COLOR_FOCAL_PX, D435_COLOR_SIZE[1] / 2],
        [0.0, 0.0, 1.0],
    ]
)


def pose_facing(
    distance_m: float,
    yaw_deg: float = 0.0,
    pitch_deg: float = 0.0,
    roll_deg: float = 0.0,
    offset_m: tuple[float, float] = (0.0, 0.0),
) -> tuple[np.ndarray, np.ndarray]:
    """Pose of a marker (design D3 frame) in the camera frame, facing the camera.

    Args:
        distance_m: Marker centre depth along the camera z axis.
        yaw_deg: Rotation about the marker y axis (applied after roll).
        pitch_deg: Rotation about the marker x axis.
        roll_deg: In-plane rotation about the marker z axis.
        offset_m: Marker centre offset along the camera (x right, y down) axes.

    Returns:
        (rvec, tvec), each shape (3,): rotation vector and translation of the marker frame in the camera frame.
    """
    yaw, pitch, roll = np.radians([yaw_deg, pitch_deg, roll_deg])
    cy, sy, cp, sp, cr, sr = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch), np.cos(roll), np.sin(roll)
    r_yaw = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    r_pitch = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]])
    r_roll = np.array([[cr, -sr, 0], [sr, cr, 0], [0, 0, 1]])
    facing = np.diag([1.0, -1.0, -1.0])  # marker x right, y up, z towards the viewer
    rotation = facing @ r_yaw @ r_pitch @ r_roll
    rvec, _ = cv2.Rodrigues(rotation)
    return rvec.reshape(3), np.array([offset_m[0], offset_m[1], distance_m], dtype=float)


def symbol_matrix(payload: str) -> np.ndarray:
    """Encode a payload at ECC Q (no silent boost) and return the module matrix.

    Args:
        payload: Text to encode.

    Returns:
        (n, n) uint8, 1 = dark module.
    """
    code = segno.make(payload, error="q", micro=False, boost_error=False)
    return np.array(code.matrix, dtype=np.uint8)


def symbol_corners_m(size_mm: float) -> np.ndarray:
    """Symbol corners in the marker frame (top-left, top-right, bottom-right, bottom-left), metres, shape (4, 3)."""
    half = size_mm / 2000.0
    return np.array([[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]])


def project_points(points: np.ndarray, pose: tuple[np.ndarray, np.ndarray], k: np.ndarray) -> np.ndarray:
    """Project marker-frame points to pixels.

    Args:
        points: (N, 3) marker-frame points in metres.
        pose: (rvec, tvec) of the marker frame in the camera frame.
        k: 3x3 intrinsics.

    Returns:
        (N, 2) pixel coordinates.
    """
    pixels, _ = cv2.projectPoints(points.astype(float), np.asarray(pose[0], float), np.asarray(pose[1], float), k, None)
    return pixels.reshape(-1, 2)


def render_marker_scene(
    payloads: list[str],
    sizes_mm: list[float],
    poses: list[tuple[np.ndarray, np.ndarray]],
    K: np.ndarray,  # noqa: N803
    image_size: tuple[int, int],
    noise_px: float = 0.0,
    blur_px: float = 0.0,
    seed: int = 0,
) -> np.ndarray:
    """Render markers into a grey image through a pinhole camera.

    Args:
        payloads: Text of each marker.
        sizes_mm: Symbol edge of each marker in mm (excluding the quiet zone).
        poses: (rvec, tvec) of each marker frame in the camera frame.
        K: 3x3 intrinsics.
        image_size: (width, height) in pixels.
        noise_px: Gaussian noise sigma in grey levels (0 = none).
        blur_px: Gaussian blur sigma in pixels (0 = none).
        seed: RNG seed for the noise.

    Returns:
        Grey uint8 image of shape (height, width) on a mid-grey background.
    """
    width, height = image_size
    big = SUPERSAMPLE
    canvas = np.full((height * big, width * big), BACKGROUND_GREY, dtype=np.uint8)
    k_big = np.asarray(K, dtype=float).copy()
    k_big[:2] *= big
    for payload, size_mm, (rvec, tvec) in zip(payloads, sizes_mm, poses, strict=True):
        matrix = np.pad(symbol_matrix(payload), QUIET_MODULES)
        n_total = matrix.shape[0]
        hires = np.kron(1 - matrix, np.ones((HIRES_PX_PER_MODULE,) * 2, dtype=np.uint8)) * 255
        n_modules = n_total - 2 * QUIET_MODULES
        # hires pixel (u, v) -> marker-frame (x, y) metres, origin at the symbol centre, +y up
        scale = size_mm / 1000.0 / n_modules / HIRES_PX_PER_MODULE
        half_px = n_total * HIRES_PX_PER_MODULE / 2
        to_marker = np.array([[scale, 0, -half_px * scale], [0, -scale, half_px * scale], [0, 0, 1.0]])
        rotation, _ = cv2.Rodrigues(np.asarray(rvec, dtype=float))
        pose_cols = np.column_stack([rotation[:, 0], rotation[:, 1], np.asarray(tvec, dtype=float)])
        homography = k_big @ pose_cols @ to_marker
        dsize = (width * big, height * big)
        warped = cv2.warpPerspective(hires, homography, dsize, flags=cv2.INTER_LINEAR, borderValue=0)
        mask = cv2.warpPerspective(np.full(hires.shape, 255, np.uint8), homography, dsize, flags=cv2.INTER_NEAREST)
        canvas[mask > 0] = warped[mask > 0]
    image = cv2.resize(canvas, (width, height), interpolation=cv2.INTER_AREA)
    if blur_px > 0:
        image = cv2.GaussianBlur(image, (0, 0), blur_px)
    if noise_px > 0:
        noise = np.random.default_rng(seed).normal(0.0, noise_px, image.shape)
        image = np.clip(image.astype(float) + noise, 0, 255).astype(np.uint8)
    return image
