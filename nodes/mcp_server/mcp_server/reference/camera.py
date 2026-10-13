"""Python references of the camera kernel family (registry read by tests/regenerate_core_golden.py).

The bodies of ``map_stats``, ``render_map_crop``, ``draw_lidar`` and ``range_color`` are the original mcp_server code
(perception.py, camera_overlay.py) before the port to mcp_core_cpp.camera; only the golden generator runs them. Each
registered kernel wraps one of them in golden terms: plain keyword inputs (arrays, numbers, a dict of intrinsics) and
named outputs (a drawn image becomes the pixels that changed, a PNG becomes its decoded image).
"""

import math
from collections.abc import Mapping, Sequence
from typing import Any

import cv2
import numpy as np
from ros2_common.camera_geometry import CameraIntrinsics

from mcp_server.camera_overlay import LIDAR_COLOR_MAX_RANGE_M, LIDAR_RADIUS_PX, MIN_DEPTH, ipt, project_points
from mcp_server.camera_scene import FRAME_BASE, Scene
from mcp_server.models import MapStats
from mcp_server.perception import (
    FREE_GREY,
    FREE_THRESHOLD,
    OCCUPIED_GREY,
    OCCUPIED_THRESHOLD,
    ROBOT_COLOR_BGR,
    UNKNOWN_GREY,
    ImageEncodingError,
    fit_size,
)
from mcp_server.reference import ReferenceKernel, Tolerance

BENCH_CAMERA = "bench"


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


def range_color(range_m: float, max_range_m: float = LIDAR_COLOR_MAX_RANGE_M) -> tuple[int, int, int]:
    """Colormap color of a lidar range (near = red, far = blue).

    Args:
        range_m (float): Range in metres.
        max_range_m (float): Range mapped to the far end.

    Returns:
        tuple[int, int, int]: BGR color.
    """
    value = int(round(255 * (1.0 - min(max(range_m / max_range_m, 0.0), 1.0))))
    pixel = cv2.applyColorMap(np.array([[value]], dtype=np.uint8), cv2.COLORMAP_JET)[0, 0]
    return int(pixel[0]), int(pixel[1]), int(pixel[2])


def draw_lidar(img: np.ndarray, scene: Scene, points: np.ndarray) -> int:
    """Draw lidar returns as range-coloured dots.

    Args:
        img (np.ndarray): BGR image (modified in place).
        scene (Scene): Camera scene.
        points (np.ndarray): (N, 4) rows (x, y, z, range) in base_link.

    Returns:
        int: Number of dots inside the image.
    """
    if len(points) == 0:
        return 0
    converted = [(scene.base_to_ref(p[:3]), float(p[3])) for p in points]
    keep = [(ref, rng) for ref, rng in converted if ref is not None]
    if not keep:
        return 0
    uv, depth = project_points(scene.intr, scene.t_ref_optical, np.array([ref for ref, _ in keep]))
    shown = 0
    for (u, v), d, (_, rng) in zip(uv, depth, keep, strict=True):
        if d > MIN_DEPTH and 0 <= u < scene.intr.width and 0 <= v < scene.intr.height:
            cv2.circle(img, ipt((u, v)), LIDAR_RADIUS_PX, range_color(rng), -1, cv2.LINE_AA)
            shown += 1
    return shown


def lidar_case(
    image_shape: Sequence[int],
    fill: int,
    points: np.ndarray,
    t_base_ref: np.ndarray | None,
    t_ref_optical: np.ndarray,
    intrinsics: Mapping[str, Any],
) -> tuple[np.ndarray, Scene, np.ndarray]:
    """The draw_lidar arguments of one golden case: a uniform image, the camera scene and the points.

    Args:
        image_shape (Sequence[int]): (height, width, 3) of the image drawn on.
        fill (int): Grey value of every pixel before drawing.
        points (np.ndarray): (N, 4) rows (x, y, z, range) in base_link.
        t_base_ref (np.ndarray | None): 4x4 pose of the reference frame in base_link (None: unknown arm offset).
        t_ref_optical (np.ndarray): 4x4 pose of the optical frame in the reference frame.
        intrinsics (Mapping[str, Any]): CameraIntrinsics fields.

    Returns:
        tuple[np.ndarray, Scene, np.ndarray]: Image, scene and points.
    """
    img = np.full(tuple(image_shape), fill, np.uint8)
    scene = Scene(
        camera=BENCH_CAMERA,
        frame_name=FRAME_BASE,
        ground_z=0.0,
        intr=CameraIntrinsics(**intrinsics),
        approximate=False,
        t_ref_optical=np.asarray(t_ref_optical, dtype=np.float64),
        t_base_ref=None if t_base_ref is None else np.asarray(t_base_ref, dtype=np.float64),
        t_arm_ref=None,
    )
    return img, scene, np.asarray(points, dtype=np.float64).reshape(-1, 4)


def lidar_outputs(img: np.ndarray, fill: int, shown: int) -> dict[str, Any]:
    """Golden outputs of a draw on a uniform image: the dot count and every pixel the dots changed.

    Args:
        img (np.ndarray): BGR image after drawing.
        fill (int): Its grey value before drawing.
        shown (int): draw_lidar's return value.

    Returns:
        dict[str, Any]: ``shown``, ``changed`` (flat pixel indices, int64) and ``values`` ((K, 3) uint8 BGR).
    """
    changed = np.flatnonzero((img != fill).any(axis=2)).astype(np.int64)
    return {"shown": shown, "changed": changed, "values": img.reshape(-1, 3)[changed]}


def png_outputs(png: bytes) -> dict[str, Any]:
    """Golden output of a PNG: its decoded image (PNG is lossless, the encoder's bytes may differ by version).

    Args:
        png (bytes): PNG file.

    Returns:
        dict[str, Any]: ``image`` (uint8 array).
    """
    return {"image": cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_UNCHANGED)}


def draw_lidar_kernel(**case: Any) -> dict[str, Any]:
    """Reference draw_lidar in golden terms.

    Args:
        **case (Any): lidar_case keyword arguments.

    Returns:
        dict[str, Any]: lidar_outputs.
    """
    img, scene, points = lidar_case(**case)
    return lidar_outputs(img, case["fill"], draw_lidar(img, scene, points))


def map_stats_kernel(**case: Any) -> dict[str, Any]:
    """Reference map_stats in golden terms.

    Args:
        **case (Any): map_stats keyword arguments.

    Returns:
        dict[str, Any]: The MapStats fields.
    """
    return map_stats(**case).model_dump()


def render_map_crop_kernel(**case: Any) -> dict[str, Any]:
    """Reference render_map_crop in golden terms.

    Args:
        **case (Any): render_map_crop keyword arguments.

    Returns:
        dict[str, Any]: png_outputs.
    """
    return png_outputs(render_map_crop(**case))


# Kernel name -> reference. Every output is exact: the C++ kernels compute the same integers (counts, crop cells,
# rounded dot centres and colormap indices) and OpenCV still does the drawing, resizing and PNG encoding.
KERNELS: dict[str, ReferenceKernel] = {
    "draw_lidar": ReferenceKernel(
        run=draw_lidar_kernel, inputs="draw_lidar_cases", tolerances={"default": Tolerance()}
    ),
    "map_stats": ReferenceKernel(run=map_stats_kernel, inputs="map_stats_cases", tolerances={"default": Tolerance()}),
    "render_map_crop": ReferenceKernel(
        run=render_map_crop_kernel, inputs="render_map_crop_cases", tolerances={"default": Tolerance()}
    ),
}
