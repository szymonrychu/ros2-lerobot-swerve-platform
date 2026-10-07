"""Overlay drawing for annotated camera images and candidate-point generation (pure numpy/OpenCV, no ROS)."""

import math
from dataclasses import dataclass

import cv2
import numpy as np
from ros2_common.camera_geometry import CameraIntrinsics

from .camera_scene import FRAME_ARM, FRAME_BASE, Scene

# Floor extent drawn per reference frame: (x_min, x_max, y_min, y_max) in metres.
GRID_EXTENT = {FRAME_BASE: (-0.3, 1.5, -0.9, 0.9), FRAME_ARM: (-0.3, 0.7, -0.7, 0.7)}
LINE_SAMPLE_M = 0.04  # spacing of samples along a grid line (curved by lens distortion)
LABEL_MIN_SPACING_PX = 70.0
CIRCLE_SEGMENTS = 96
LIDAR_COLOR_MAX_RANGE_M = 4.0
LIDAR_RADIUS_PX = 3
MARKER_RADIUS_PX = 7
MIN_DEPTH = 1e-9
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.4
COLOR_GRID = (200, 200, 200)
COLOR_REACH = (0, 200, 255)
COLOR_GRIPPER = (0, 255, 0)
COLOR_PLANNED = (255, 0, 255)
COLOR_LABEL = (255, 255, 255)
COLOR_CANDIDATE = (0, 0, 255)
MAX_PROJECTED_PX = 1.0e5  # drop samples that project absurdly far outside the image (cv2 drawing overflow)


@dataclass(frozen=True)
class GridLine:
    """One floor grid line projected to pixels: constant x ('x') or constant y ('y') in the reference frame."""

    axis: str
    value: float
    points: list[tuple[float, float]]


@dataclass(frozen=True)
class GridLabel:
    """A metric label at a visible floor intersection."""

    u: float
    v: float
    text: str


def project_points(intr: CameraIntrinsics, t_ref_optical: np.ndarray, pts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Project many reference-frame points ignoring image bounds (same maths as camera_geometry.project_raw).

    Args:
        intr (CameraIntrinsics): Intrinsics.
        t_ref_optical (np.ndarray): 4x4 optical frame pose in the points' frame.
        pts (np.ndarray): Points (N, 3).

    Returns:
        tuple[np.ndarray, np.ndarray]: Pixels (N, 2) and depths (N,) in the optical frame.
    """
    pts = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
    cam = (np.linalg.inv(t_ref_optical) @ np.hstack([pts, np.ones((len(pts), 1))]).T).T[:, :3]
    depth = cam[:, 2].copy()
    safe = cam.copy()
    safe[:, 2] = np.where(depth > MIN_DEPTH, depth, 1.0)  # keep projectPoints finite for points behind the camera
    dist = np.array(intr.distortion, dtype=np.float64) if intr.distortion else np.zeros(5)
    pix, _ = cv2.projectPoints(safe.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), intr.matrix(), dist)
    return pix.reshape(-1, 2), depth


def visible_runs(uv: np.ndarray, depth: np.ndarray) -> list[list[tuple[float, float]]]:
    """Split projected samples into runs of consecutive in-front-of-camera, finite points.

    Args:
        uv (np.ndarray): Pixels (N, 2).
        depth (np.ndarray): Depths (N,).

    Returns:
        list[list[tuple[float, float]]]: Runs with at least two points.
    """
    runs: list[list[tuple[float, float]]] = []
    current: list[tuple[float, float]] = []
    for (u, v), d in zip(uv, depth, strict=True):
        if d > MIN_DEPTH and abs(u) < MAX_PROJECTED_PX and abs(v) < MAX_PROJECTED_PX:
            current.append((float(u), float(v)))
        else:
            if len(current) >= 2:
                runs.append(current)
            current = []
    if len(current) >= 2:
        runs.append(current)
    return runs


def grid_values(low: float, high: float, step: float) -> list[float]:
    """Multiples of step within [low, high].

    Args:
        low (float): Lower bound.
        high (float): Upper bound.
        step (float): Spacing.

    Returns:
        list[float]: Values (rounded to 1e-6 to avoid float drift).
    """
    first, last = math.ceil(low / step - 1e-9), math.floor(high / step + 1e-9)
    return [round(k * step, 6) for k in range(first, last + 1)]


def grid_polylines(scene: Scene, step: float) -> list[GridLine]:
    """Floor grid lines as pixel polylines, split where they pass behind the camera.

    Args:
        scene (Scene): Camera scene (grid is in its reference frame).
        step (float): Grid spacing in metres.

    Returns:
        list[GridLine]: Visible parts of the lines of constant x and of constant y.
    """
    x0, x1, y0, y1 = GRID_EXTENT[scene.frame_name]
    lines: list[GridLine] = []
    for axis, values, lo, hi in (("x", grid_values(x0, x1, step), y0, y1), ("y", grid_values(y0, y1, step), x0, x1)):
        along = np.linspace(lo, hi, max(2, int((hi - lo) / LINE_SAMPLE_M) + 1))
        for value in values:
            const = np.full_like(along, value)
            xy = np.column_stack([const, along] if axis == "x" else [along, const])
            pts = np.column_stack([xy, np.full(len(along), scene.ground_z)])
            uv, depth = project_points(scene.intr, scene.t_ref_optical, pts)
            lines.extend(GridLine(axis, value, run) for run in visible_runs(uv, depth))
    return lines


def format_metres(value: float) -> str:
    """Compact metre text without trailing zeros.

    Args:
        value (float): Metres.

    Returns:
        str: E.g. '0.5', '-1', '0.25'.
    """
    return f"{round(value, 3):g}"


def grid_labels(scene: Scene, step: float) -> list[GridLabel]:
    """Metric '(x,y)' labels at visible grid intersections, thinned so labels keep a minimum pixel spacing.

    Args:
        scene (Scene): Camera scene.
        step (float): Grid spacing in metres.

    Returns:
        list[GridLabel]: Labels inside the image.
    """
    x0, x1, y0, y1 = GRID_EXTENT[scene.frame_name]
    xs, ys = grid_values(x0, x1, step), grid_values(y0, y1, step)
    pts = np.array([[x, y, scene.ground_z] for x in xs for y in ys])
    uv, depth = project_points(scene.intr, scene.t_ref_optical, pts)
    candidates = [
        (float(u), float(v), f"({format_metres(p[0])},{format_metres(p[1])})")
        for (u, v), d, p in zip(uv, depth, pts, strict=True)
        if d > MIN_DEPTH and 0 <= u < scene.intr.width and 0 <= v < scene.intr.height
    ]
    candidates.sort(key=lambda c: (-c[1], c[0]))  # nearest the camera (image bottom) first
    kept: list[GridLabel] = []
    for u, v, text in candidates:
        if all(math.hypot(u - k.u, v - k.v) >= LABEL_MIN_SPACING_PX for k in kept):
            kept.append(GridLabel(u, v, text))
    return kept


def circle_polyline(scene: Scene, centre_xy: tuple[float, float], radius: float) -> list[tuple[float, float]]:
    """A floor circle as pixel points (only the part in front of the camera).

    Args:
        scene (Scene): Camera scene.
        centre_xy (tuple[float, float]): Circle centre (x, y) in the reference frame.
        radius (float): Radius in metres.

    Returns:
        list[tuple[float, float]]: Pixel points of the visible arcs, concatenated.
    """
    return [pt for run in circle_runs(scene, centre_xy, radius) for pt in run]


def circle_runs(scene: Scene, centre_xy: tuple[float, float], radius: float) -> list[list[tuple[float, float]]]:
    """A floor circle as separate visible pixel arcs.

    Args:
        scene (Scene): Camera scene.
        centre_xy (tuple[float, float]): Circle centre (x, y) in the reference frame.
        radius (float): Radius in metres.

    Returns:
        list[list[tuple[float, float]]]: Visible arcs.
    """
    angles = np.linspace(0.0, 2.0 * math.pi, CIRCLE_SEGMENTS + 1)
    pts = np.column_stack(
        [
            centre_xy[0] + radius * np.cos(angles),
            centre_xy[1] + radius * np.sin(angles),
            np.full(len(angles), scene.ground_z),
        ]
    )
    uv, depth = project_points(scene.intr, scene.t_ref_optical, pts)
    return visible_runs(uv, depth)


def ipt(point: tuple[float, float]) -> tuple[int, int]:
    """Round a pixel coordinate for OpenCV drawing.

    Args:
        point (tuple[float, float]): (u, v).

    Returns:
        tuple[int, int]: Integer pixel.
    """
    return int(round(point[0])), int(round(point[1]))


def draw_polylines(img: np.ndarray, polylines: list[list[tuple[float, float]]], color: tuple[int, int, int]) -> None:
    """Draw polylines in place.

    Args:
        img (np.ndarray): BGR image.
        polylines (list[list[tuple[float, float]]]): Pixel polylines.
        color (tuple[int, int, int]): BGR color.
    """
    for line in polylines:
        pts = np.array([ipt(p) for p in line], dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(img, [pts], False, color, 1, cv2.LINE_AA)


def draw_text(img: np.ndarray, text: str, org: tuple[int, int], color: tuple[int, int, int] = COLOR_LABEL) -> None:
    """Draw outlined text in place so it reads on any background.

    Args:
        img (np.ndarray): BGR image.
        text (str): Text.
        org (tuple[int, int]): Bottom-left pixel of the text.
        color (tuple[int, int, int]): BGR color.
    """
    cv2.putText(img, text, org, FONT, FONT_SCALE, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, org, FONT, FONT_SCALE, color, 1, cv2.LINE_AA)


def draw_grid(img: np.ndarray, scene: Scene, step: float) -> int:
    """Draw the floor grid with metric labels.

    Args:
        img (np.ndarray): BGR image (modified in place).
        scene (Scene): Camera scene.
        step (float): Grid spacing in metres.

    Returns:
        int: Number of labels drawn.
    """
    draw_polylines(img, [ln.points for ln in grid_polylines(scene, step)], COLOR_GRID)
    labels = grid_labels(scene, step)
    for lb in labels:
        draw_text(img, lb.text, (int(lb.u) + 3, int(lb.v) - 3))
    return len(labels)


def draw_marker(
    img: np.ndarray, uv: tuple[float, float], color: tuple[int, int, int], shape: str, label: str | None = None
) -> None:
    """Draw a point marker ('circle' or 'cross') with an optional label.

    Args:
        img (np.ndarray): BGR image (modified in place).
        uv (tuple[float, float]): Pixel position.
        color (tuple[int, int, int]): BGR color.
        shape (str): 'circle' or 'cross'.
        label (str | None): Text beside the marker.
    """
    centre = ipt(uv)
    if shape == "cross":
        cv2.drawMarker(img, centre, color, cv2.MARKER_CROSS, 2 * MARKER_RADIUS_PX, 2, cv2.LINE_AA)
    else:
        cv2.circle(img, centre, MARKER_RADIUS_PX, color, 2, cv2.LINE_AA)
    if label:
        draw_text(img, label, (centre[0] + MARKER_RADIUS_PX + 3, centre[1]), color)


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


def draw_legend(img: np.ndarray, lines: list[str]) -> None:
    """Draw a text legend block in the top-left corner.

    Args:
        img (np.ndarray): BGR image (modified in place).
        lines (list[str]): Legend lines.
    """
    line_h = 16
    width = max(cv2.getTextSize(t, FONT, FONT_SCALE, 1)[0][0] for t in lines) + 10
    overlay = img.copy()
    cv2.rectangle(overlay, (0, 0), (width, line_h * len(lines) + 8), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.55, img, 0.45, 0, img)
    for i, text in enumerate(lines):
        cv2.putText(img, text, (5, line_h * (i + 1)), FONT, FONT_SCALE, COLOR_LABEL, 1, cv2.LINE_AA)


def thin[T](items: list[T], limit: int) -> list[T]:
    """Keep at most limit items, evenly spread and in order.

    Args:
        items (list[T]): Items.
        limit (int): Maximum count.

    Returns:
        list[T]: The kept items.
    """
    if len(items) <= limit:
        return items
    return [items[int(i * len(items) / limit)] for i in range(limit)]


def candidate_pixels(
    width: int, height: int, region: tuple[float, float, float, float] | None, spacing_px: int, max_points: int
) -> list[tuple[float, float]]:
    """Grid of candidate pixels inside a region (cell centres), thinned evenly to max_points.

    Args:
        width (int): Image width in pixels.
        height (int): Image height in pixels.
        region (tuple[float, float, float, float] | None): (u0, v0, u1, v1) or None for the whole image.
        spacing_px (int): Grid spacing.
        max_points (int): Upper bound on returned points.

    Returns:
        list[tuple[float, float]]: Pixels in row-major order (top to bottom, left to right).

    Raises:
        ValueError: When the region is empty or outside the image.
    """
    u0, v0, u1, v1 = region if region is not None else (0.0, 0.0, float(width), float(height))
    if not (0 <= u0 < u1 <= width and 0 <= v0 < v1 <= height):
        raise ValueError(f"region must satisfy 0 <= u0 < u1 <= {width} and 0 <= v0 < v1 <= {height}")
    us = [u0 + spacing_px / 2 + i * spacing_px for i in range(int((u1 - u0) // spacing_px))]
    vs = [v0 + spacing_px / 2 + j * spacing_px for j in range(int((v1 - v0) // spacing_px))]
    return thin([(float(u), float(v)) for v in vs for u in us], max_points)


def draw_candidates(img: np.ndarray, points: list[tuple[int, float, float]]) -> None:
    """Draw numbered candidate dots.

    Args:
        img (np.ndarray): BGR image (modified in place).
        points (list[tuple[int, float, float]]): (n, u, v) per point.
    """
    for n, u, v in points:
        cv2.circle(img, ipt((u, v)), 4, COLOR_CANDIDATE, -1, cv2.LINE_AA)
        cv2.circle(img, ipt((u, v)), 5, (255, 255, 255), 1, cv2.LINE_AA)
        draw_text(img, str(n), (int(u) + 6, int(v) - 4), (0, 255, 255))
