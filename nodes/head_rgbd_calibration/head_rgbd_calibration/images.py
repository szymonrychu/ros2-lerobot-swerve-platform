"""Pose thumbnails: JPEG previews with the detected ChArUco corners drawn."""

import cv2
import numpy as np
from ros2_common.head_rgbd_calibration.charuco import Detection
from ros2_common.head_rgbd_calibration.frames import normalise_amplitude

JPEG_QUALITY = 80
TOF_ENLARGE = 2
CORNER_COLOR_BGR = (0, 255, 0)
PIXEL_CENTRE = 0.5  # detect() reports corners with pixel centres at integer coordinates


def draw_corners(image: np.ndarray, detection: Detection, scale: float) -> np.ndarray:
    """Draw the detected corners on a BGR image that is `scale` times the detection's source image.

    Args:
        image: BGR image to draw on (modified in place).
        detection: Corners in source pixels.
        scale: Ratio of image size to source image size.

    Returns:
        The same image with the corners drawn.
    """
    points = ((detection.points.astype(np.float64) + PIXEL_CENTRE) * scale - PIXEL_CENTRE).astype(np.float32)
    if len(points):
        cv2.aruco.drawDetectedCornersCharuco(
            image, points.reshape(-1, 1, 2), detection.ids.reshape(-1, 1).astype(np.int32), CORNER_COLOR_BGR
        )
    return image


def encode_jpeg(image: np.ndarray) -> bytes:
    """JPEG bytes of a BGR image."""
    ok, buffer = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    if not ok:
        raise ValueError("JPEG encoding failed")
    return buffer.tobytes()


def rgb_thumbnail(image: np.ndarray, detection: Detection, width_px: int) -> bytes:
    """JPEG thumbnail of an RGB frame with its corners drawn.

    Args:
        image: BGR frame (not modified).
        detection: Board corners in the frame's pixels.
        width_px: Thumbnail width; a narrower frame is not enlarged.

    Returns:
        JPEG bytes, scaled to at most width_px wide with the aspect ratio kept.
    """
    height, width = image.shape[:2]
    scale = min(1.0, width_px / width)
    size = (max(1, round(width * scale)), max(1, round(height * scale)))
    thumb = cv2.resize(image, size, interpolation=cv2.INTER_AREA) if scale < 1.0 else image.copy()
    return encode_jpeg(draw_corners(thumb, detection, size[0] / width))


def tof_thumbnail(amplitude: np.ndarray, detection: Detection, percentiles: tuple[float, float]) -> bytes | None:
    """JPEG thumbnail of a ToF amplitude frame: normalised, enlarged 2x, corners drawn.

    Args:
        amplitude: uint16 amplitude frame.
        detection: Board corners in amplitude pixels.
        percentiles: Selection amplitude_percentiles, as normalise_amplitude uses them.

    Returns:
        JPEG bytes, or None for an empty frame (no dynamic range).
    """
    grey = normalise_amplitude(amplitude, percentiles)
    if grey is None:
        return None
    big = cv2.resize(grey, None, fx=TOF_ENLARGE, fy=TOF_ENLARGE, interpolation=cv2.INTER_CUBIC)
    return encode_jpeg(draw_corners(cv2.cvtColor(big, cv2.COLOR_GRAY2BGR), detection, TOF_ENLARGE))
