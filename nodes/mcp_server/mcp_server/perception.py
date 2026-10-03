"""Perception helpers without ROS: laser scan sector summary, occupancy map stats/crop, image encoding."""

import math
from collections.abc import Sequence

import cv2
import numpy as np

from .models import MapStats, SectorObstacle

# Eight 45 deg sectors counter-clockwise from the robot's front (base_link +x), centred on their direction.
SECTOR_NAMES = ("front", "front_left", "left", "rear_left", "rear", "rear_right", "right", "front_right")
OCCUPIED_THRESHOLD = 65
FREE_THRESHOLD = 25
UNKNOWN_GREY = 160
FREE_GREY = 255
OCCUPIED_GREY = 0
ROBOT_COLOR_BGR = (0, 0, 255)
SUPPORTED_ENCODINGS = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}


class ImageEncodingError(ValueError):
    """Raised for an image buffer that cannot be decoded or converted."""


def summarize_scan(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    laser_x: float,
    laser_y: float,
    laser_yaw: float,
) -> list[SectorObstacle]:
    """Nearest valid return per sector, expressed in base_link.

    Args:
        ranges (Sequence[float]): LaserScan ranges (m).
        angle_min (float): Angle of the first range in the laser frame (rad).
        angle_increment (float): Angle step (rad).
        range_min (float): Minimum valid range (m).
        range_max (float): Maximum valid range (m).
        laser_x (float): Laser origin x in base_link (m).
        laser_y (float): Laser origin y in base_link (m).
        laser_yaw (float): Laser yaw in base_link (rad).

    Returns:
        list[SectorObstacle]: One entry per SECTOR_NAMES item; nearest_m is None when the sector has no return.
    """
    r = np.asarray(ranges, dtype=float)
    angles = angle_min + angle_increment * np.arange(r.size)
    valid = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    r, angles = r[valid], angles[valid]
    c, s = math.cos(laser_yaw), math.sin(laser_yaw)
    lx, ly = r * np.cos(angles), r * np.sin(angles)
    bx = laser_x + c * lx - s * ly
    by = laser_y + s * lx + c * ly
    dist = np.hypot(bx, by)
    bearing = np.arctan2(by, bx)
    width = 2 * math.pi / len(SECTOR_NAMES)
    sector_idx = np.floor(((bearing + width / 2) % (2 * math.pi)) / width).astype(int) % len(SECTOR_NAMES)
    out: list[SectorObstacle] = []
    for i, name in enumerate(SECTOR_NAMES):
        mask = sector_idx == i
        if not mask.any():
            out.append(SectorObstacle(sector=name, nearest_m=None, bearing_rad=None))
            continue
        k = int(np.argmin(np.where(mask, dist, np.inf)))
        out.append(SectorObstacle(sector=name, nearest_m=round(float(dist[k]), 3), bearing_rad=float(bearing[k])))
    return out


def map_stats(data: Sequence[int], width: int, height: int, resolution: float) -> MapStats:
    """Cell counts of an occupancy grid.

    Args:
        data (Sequence[int]): Row-major cells (-1 unknown, 0..100 occupancy).
        width (int): Columns.
        height (int): Rows.
        resolution (float): Cell size (m).

    Returns:
        MapStats: Size and known/occupied/free counts.
    """
    cells = np.asarray(data, dtype=np.int16)
    known = cells >= 0
    return MapStats(
        width=width,
        height=height,
        resolution=resolution,
        width_m=round(width * resolution, 3),
        height_m=round(height * resolution, 3),
        known_cells=int(known.sum()),
        occupied_cells=int((cells >= OCCUPIED_THRESHOLD).sum()),
        free_cells=int((known & (cells <= FREE_THRESHOLD)).sum()),
    )


def fit_size(width: int, height: int, max_px: int) -> tuple[int, int]:
    """Scale (width, height) down so the longer side is at most max_px, keeping the aspect ratio.

    Args:
        width (int): Width.
        height (int): Height.
        max_px (int): Longest side allowed.

    Returns:
        tuple[int, int]: New (width, height); unchanged when already small enough.
    """
    longest = max(width, height)
    if longest <= max_px:
        return width, height
    scale = max_px / longest
    return max(1, round(width * scale)), max(1, round(height * scale))


def render_map_crop(
    data: Sequence[int],
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    radius_m: float,
    max_px: int,
) -> bytes:
    """PNG of the occupancy grid around the robot (north = map +y up), robot drawn as an arrow.

    Args:
        data (Sequence[int]): Row-major cells.
        width (int): Columns.
        height (int): Rows.
        resolution (float): Cell size (m).
        origin_x (float): Map origin x (m), cell (0, 0) corner.
        origin_y (float): Map origin y (m).
        robot_x (float): Robot x in the map frame (m).
        robot_y (float): Robot y in the map frame (m).
        robot_yaw (float): Robot yaw in the map frame (rad).
        radius_m (float): Half size of the square crop (m).
        max_px (int): Longest side of the PNG.

    Returns:
        bytes: PNG image.
    """
    grid = np.asarray(data, dtype=np.int16).reshape(height, width)
    grey = np.full(grid.shape, UNKNOWN_GREY, np.uint8)
    grey[(grid >= 0) & (grid <= FREE_THRESHOLD)] = FREE_GREY
    grey[grid >= OCCUPIED_THRESHOLD] = OCCUPIED_GREY
    half = max(1, int(math.ceil(radius_m / resolution)))
    cx = int(math.floor((robot_x - origin_x) / resolution))
    cy = int(math.floor((robot_y - origin_y) / resolution))
    crop = np.full((2 * half, 2 * half), UNKNOWN_GREY, np.uint8)
    x0, y0 = cx - half, cy - half
    sx0, sy0 = max(x0, 0), max(y0, 0)
    sx1, sy1 = min(x0 + 2 * half, width), min(y0 + 2 * half, height)
    if sx1 > sx0 and sy1 > sy0:
        crop[sy0 - y0 : sy1 - y0, sx0 - x0 : sx1 - x0] = grey[sy0:sy1, sx0:sx1]
    image = cv2.cvtColor(np.flipud(crop), cv2.COLOR_GRAY2BGR)
    w, h = fit_size(image.shape[1], image.shape[0], max_px)
    image = cv2.resize(image, (w, h), interpolation=cv2.INTER_NEAREST)
    scale = w / (2 * half)
    centre = (w // 2, h // 2)
    tip = (int(centre[0] + math.cos(robot_yaw) * 8), int(centre[1] - math.sin(robot_yaw) * 8))
    cv2.circle(image, centre, max(2, int(0.15 / resolution * scale)), ROBOT_COLOR_BGR, 1)
    cv2.arrowedLine(image, centre, tip, ROBOT_COLOR_BGR, 1, tipLength=0.4)
    ok, buf = cv2.imencode(".png", image)
    if not ok:
        raise ImageEncodingError("PNG encoding failed")
    return buf.tobytes()


def encode_jpeg(bgr: np.ndarray, max_px: int, quality: int = 80) -> bytes:
    """Downscale (if needed) and JPEG-encode a BGR image.

    Args:
        bgr (np.ndarray): HxWx3 uint8 image.
        max_px (int): Longest side allowed.
        quality (int): JPEG quality.

    Returns:
        bytes: JPEG data.
    """
    w, h = fit_size(bgr.shape[1], bgr.shape[0], max_px)
    if (w, h) != (bgr.shape[1], bgr.shape[0]):
        bgr = cv2.resize(bgr, (w, h), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise ImageEncodingError("JPEG encoding failed")
    return buf.tobytes()


def image_to_bgr(encoding: str, height: int, width: int, step: int, data: bytes) -> np.ndarray:
    """Convert a sensor_msgs/Image buffer into a BGR array.

    Args:
        encoding (str): ROS image encoding.
        height (int): Rows.
        width (int): Columns.
        step (int): Bytes per row (may include padding).
        data (bytes): Pixel buffer.

    Returns:
        np.ndarray: HxWx3 uint8 BGR image.
    """
    channels = SUPPORTED_ENCODINGS.get(encoding)
    if channels is None:
        raise ImageEncodingError(f"unsupported image encoding {encoding!r} (supported: {sorted(SUPPORTED_ENCODINGS)})")
    if step < width * channels or len(data) < step * height:
        raise ImageEncodingError(f"image buffer too short: {len(data)} bytes for {width}x{height} step {step}")
    rows = np.frombuffer(bytes(data), np.uint8, count=step * height).reshape(height, step)
    pixels = rows[:, : width * channels].reshape(height, width, channels)
    conversions = {
        "rgb8": cv2.COLOR_RGB2BGR,
        "rgba8": cv2.COLOR_RGBA2BGR,
        "bgra8": cv2.COLOR_BGRA2BGR,
        "mono8": cv2.COLOR_GRAY2BGR,
    }
    if encoding == "bgr8":
        return pixels.copy()
    src = pixels[:, :, 0] if channels == 1 else pixels
    return cv2.cvtColor(np.ascontiguousarray(src), conversions[encoding])


def recompress_jpeg(jpeg: bytes, max_px: int, quality: int = 80) -> tuple[bytes, int, int]:
    """Pass a JPEG through when small enough, otherwise decode, downscale and re-encode once.

    Args:
        jpeg (bytes): JPEG data.
        max_px (int): Longest side allowed.
        quality (int): JPEG quality for re-encoding.

    Returns:
        tuple[bytes, int, int]: (JPEG data, width, height).
    """
    image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ImageEncodingError("compressed frame is not a decodable image")
    h, w = image.shape[:2]
    nw, nh = fit_size(w, h, max_px)
    if (nw, nh) == (w, h) and jpeg[:2] == b"\xff\xd8":
        return jpeg, w, h
    return encode_jpeg(image, max_px, quality), nw, nh
