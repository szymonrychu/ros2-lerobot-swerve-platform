"""Perception helpers without ROS: laser scan sector summary, occupancy map stats/crop, image encoding."""

import math
from collections.abc import Sequence

import cv2
import numpy as np

from mcp_core_cpp import camera as core_camera

from .models import MapStats, Obstacles3d, SectorObstacle, TofPoints, TofSector

# Eight 45 deg sectors counter-clockwise from the robot's front (base_link +x), centred on their direction.
SECTOR_NAMES = ("front", "front_left", "left", "rear_left", "rear", "rear_right", "right", "front_right")
TOF_HEIGHT_WINDOW_M = 0.1
# Same bands as the Nav2 STVL layer (nav2_params): obstacle heights above the floor, cliff below, summary range.
TOF_OBSTACLE_BAND_M = (0.10, 0.6)
TOF_CLIFF_MAX_Z_M = -0.10
TOF_SUMMARY_RANGE_M = 2.0
OCCUPIED_THRESHOLD = 65
FREE_THRESHOLD = 25
UNKNOWN_GREY = 160
FREE_GREY = 255
OCCUPIED_GREY = 0
ROBOT_COLOR_BGR = (0, 0, 255)
SUPPORTED_ENCODINGS = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}


class ImageEncodingError(ValueError):
    """Raised for an image buffer that cannot be decoded or converted."""


def scan_points(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    laser_x: float,
    laser_y: float,
    laser_yaw: float,
) -> np.ndarray:
    """Valid laser returns as points in base_link.

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
        np.ndarray: Nx2 (forward, left) points in metres; invalid returns are dropped.
    """
    r = np.asarray(ranges, dtype=float)
    angles = angle_min + angle_increment * np.arange(r.size)
    valid = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    r, angles = r[valid], angles[valid]
    c, s = math.cos(laser_yaw), math.sin(laser_yaw)
    lx, ly = r * np.cos(angles), r * np.sin(angles)
    return np.stack([laser_x + c * lx - s * ly, laser_y + s * lx + c * ly], axis=1)


def sector_indices(bearing: np.ndarray) -> np.ndarray:
    """Index into SECTOR_NAMES of each bearing.

    Args:
        bearing (np.ndarray): Bearings in base_link (rad, 0 = ahead, + = left).

    Returns:
        np.ndarray: Integer sector index per bearing.
    """
    width = 2 * math.pi / len(SECTOR_NAMES)
    return np.floor(((bearing + width / 2) % (2 * math.pi)) / width).astype(int) % len(SECTOR_NAMES)


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
    points = scan_points(ranges, angle_min, angle_increment, range_min, range_max, laser_x, laser_y, laser_yaw)
    bx, by = points[:, 0], points[:, 1]
    dist = np.hypot(bx, by)
    bearing = np.arctan2(by, bx)
    sector_idx = sector_indices(bearing)
    out: list[SectorObstacle] = []
    for i, name in enumerate(SECTOR_NAMES):
        mask = sector_idx == i
        if not mask.any():
            out.append(SectorObstacle(sector=name, nearest_m=None, bearing_rad=None))
            continue
        k = int(np.argmin(np.where(mask, dist, np.inf)))
        out.append(SectorObstacle(sector=name, nearest_m=round(float(dist[k]), 3), bearing_rad=float(bearing[k])))
    return out


def tof_obstacle_sectors(
    points_base: np.ndarray, obstacle_band: tuple[float, float], cliff_max_z: float, max_range_m: float
) -> list[TofSector]:
    """Nearest 3D obstacle and nearest cliff per sector from a ToF cloud in base_link.

    Args:
        points_base (np.ndarray): Nx3 (x forward, y left, z up) points in base_link (m).
        obstacle_band (tuple[float, float]): (min_z, max_z) heights that count as obstacles (m).
        cliff_max_z (float): Points with z below this are cliff points (m).
        max_range_m (float): Points farther than this horizontal distance are ignored (m).

    Returns:
        list[TofSector]: One entry per SECTOR_NAMES item; fields are None when the sector has no such point.
    """
    pts = np.asarray(points_base, dtype=float).reshape(-1, 3)
    pts = pts[np.isfinite(pts).all(axis=1)]
    dist = np.hypot(pts[:, 0], pts[:, 1])
    in_range = dist <= max_range_m
    pts, dist = pts[in_range], dist[in_range]
    z = pts[:, 2]
    bearing = np.arctan2(pts[:, 1], pts[:, 0])
    sector_idx = sector_indices(bearing)
    obstacle = (z >= obstacle_band[0]) & (z <= obstacle_band[1])
    cliff = z < cliff_max_z
    out: list[TofSector] = []
    for i, name in enumerate(SECTOR_NAMES):
        in_sector = sector_idx == i
        obs = in_sector & obstacle
        cliff_dist = dist[in_sector & cliff]
        cliff_m = round(float(cliff_dist.min()), 3) if cliff_dist.size else None
        if not obs.any():
            out.append(
                TofSector(sector=name, nearest_m=None, bearing_rad=None, min_z=None, max_z=None, cliff_m=cliff_m)
            )
            continue
        k = int(np.argmin(np.where(obs, dist, np.inf)))
        near = obs & (dist <= dist[k] + TOF_HEIGHT_WINDOW_M)
        out.append(
            TofSector(
                sector=name,
                nearest_m=round(float(dist[k]), 3),
                bearing_rad=float(bearing[k]),
                min_z=round(float(z[near].min()), 3),
                max_z=round(float(z[near].max()), 3),
                cliff_m=cliff_m,
            )
        )
    return out


def summarize_tof(cloud: TofPoints) -> Obstacles3d:
    """Obstacles3d of a base_link ToF cloud with the STVL bands.

    Args:
        cloud (TofPoints): Cloud in base_link.

    Returns:
        Obstacles3d: Per-sector nearest obstacle and cliff within TOF_SUMMARY_RANGE_M.
    """
    pts = np.asarray(cloud.points, dtype=float).reshape(-1, 3)
    return Obstacles3d(
        sectors=tof_obstacle_sectors(pts, TOF_OBSTACLE_BAND_M, TOF_CLIFF_MAX_Z_M, TOF_SUMMARY_RANGE_M),
        cloud_age_s=round(cloud.age_s, 3),
        frame=cloud.frame,
        point_count=int(np.isfinite(pts).all(axis=1).sum()),
    )


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
    counts = core_camera.map_counts(contiguous(data), OCCUPIED_THRESHOLD, FREE_THRESHOLD)
    return MapStats(
        width=width,
        height=height,
        resolution=resolution,
        width_m=round(width * resolution, 3),
        height_m=round(height * resolution, 3),
        **counts,
    )


def contiguous(data: Sequence[int]) -> Sequence[int]:
    """The grid as the C++ map kernels read it: a numpy array made C-contiguous, anything else unchanged.

    Args:
        data (Sequence[int]): Row-major cells (list, array.array or numpy array).

    Returns:
        Sequence[int]: The same cells.
    """
    return np.ascontiguousarray(data) if isinstance(data, np.ndarray) else data


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
    half = max(1, int(math.ceil(radius_m / resolution)))
    cx = int(math.floor((robot_x - origin_x) / resolution))
    cy = int(math.floor((robot_y - origin_y) / resolution))
    crop = core_camera.map_crop(
        contiguous(data),
        width,
        height,
        cx - half,
        cy - half,
        2 * half,
        OCCUPIED_THRESHOLD,
        FREE_THRESHOLD,
        UNKNOWN_GREY,
        FREE_GREY,
        OCCUPIED_GREY,
    )  # north up
    image = cv2.cvtColor(crop, cv2.COLOR_GRAY2BGR)
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
