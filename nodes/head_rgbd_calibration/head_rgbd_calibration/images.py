"""Pose thumbnails: JPEG previews with the detected ChArUco corners drawn."""

import cv2
import numpy as np
from ros2_common.head_rgbd_calibration.charuco import Detection

JPEG_QUALITY = 80
DEPTH_PERCENTILES = (1.0, 99.0)  # valid depth range mapped onto the colour map, robust to stray far pixels
MAX_UINT8 = 255
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


def scaled_thumbnail(image: np.ndarray, detection: Detection, width_px: int, interpolation: int) -> bytes:
    """JPEG of a BGR image scaled to at most width_px wide (aspect kept, never enlarged) with the corners drawn."""
    height, width = image.shape[:2]
    scale = min(1.0, width_px / width)
    size = (max(1, round(width * scale)), max(1, round(height * scale)))
    thumb = cv2.resize(image, size, interpolation=interpolation) if scale < 1.0 else image.copy()
    return encode_jpeg(draw_corners(thumb, detection, size[0] / width))


def rgb_thumbnail(image: np.ndarray, detection: Detection, width_px: int) -> bytes:
    """JPEG thumbnail of an RGB frame with its corners drawn.

    Args:
        image: BGR frame (not modified).
        detection: Board corners in the frame's pixels.
        width_px: Thumbnail width; a narrower frame is not enlarged.

    Returns:
        JPEG bytes, scaled to at most width_px wide with the aspect ratio kept.
    """
    return scaled_thumbnail(image, detection, width_px, cv2.INTER_AREA)


def depth_thumbnail(depth_mm: np.ndarray, detection: Detection, width_px: int) -> bytes | None:
    """JPEG thumbnail of an aligned depth frame: colourised (turbo, near blue), no depth black, colour corners drawn.

    Args:
        depth_mm: uint16 depth registered to the colour frame, millimetres (0 is no depth); not modified.
        detection: Board corners in colour pixels (the aligned depth shares them).
        width_px: Thumbnail width; a narrower frame is not enlarged.

    Returns:
        JPEG bytes scaled as rgb_thumbnail, or None when the frame holds no valid depth.
    """
    valid = depth_mm > 0
    if not valid.any():
        return None
    low, high = np.percentile(depth_mm[valid], DEPTH_PERCENTILES)
    span = max(float(high - low), 1.0)
    grey = np.clip((depth_mm.astype(np.float64) - low) / span * MAX_UINT8, 0, MAX_UINT8).astype(np.uint8)
    colour = cv2.applyColorMap(grey, cv2.COLORMAP_TURBO)
    colour[~valid] = 0
    return scaled_thumbnail(colour, detection, width_px, cv2.INTER_NEAREST)
