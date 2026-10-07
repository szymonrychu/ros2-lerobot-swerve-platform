"""Robot-up top-down view renderer (OpenCV/numpy only, no ROS).

Conventions: the image is centred on the robot, base_link +x (forward) is image UP and base_link +y (left) is image
LEFT; one pixel is `2 * radius_m / px` metres. Map-frame layers (SLAM map, costmap, plan, POIs, objects) are rotated
into the robot frame with the map -> base_link pose; lidar points are already in base_link.
"""

import math
from collections.abc import Sequence
from dataclasses import dataclass, field

import cv2
import numpy as np

from .perception import FREE_THRESHOLD, OCCUPIED_THRESHOLD, ImageEncodingError

ALL_LAYERS = ("map", "costmap", "lidar", "footprint", "reach", "path", "pois", "objects")
# Layers drawn in the map frame: they need the robot pose to be placed.
POSE_LAYERS = frozenset({"map", "costmap", "path", "pois", "objects"})
ORIENTATION = "robot-up: base_link +x (heading) is image up, +y (left) is image left; robot at the image centre"
SCALE_BAR_M = 0.5
HEADING_EXTRA_M = 0.10  # the heading arrow reaches this far beyond the front edge of the footprint
OUTSIDE = -2  # sample_grid value for cells outside the grid
UNKNOWN_CELL = -1

# Colours are BGR.
COLOR_UNKNOWN = (150, 150, 150)
COLOR_FREE = (240, 240, 240)
COLOR_OCCUPIED = (0, 0, 0)
COLOR_COSTMAP = (0, 140, 255)
COLOR_LIDAR = (0, 200, 0)
COLOR_FOOTPRINT = (255, 0, 0)
COLOR_HEADING = (0, 0, 255)
COLOR_REACH = (0, 170, 170)
COLOR_PATH = (255, 0, 255)
COLOR_POI_OPEN = (0, 128, 255)
COLOR_POI_DONE = (60, 140, 60)
COLOR_POI_CANCELLED = (130, 130, 130)
COLOR_OBJECT = (200, 0, 120)
COLOR_TEXT = (20, 20, 20)
COLOR_LEGEND_BG = (255, 255, 255)
FONT = cv2.FONT_HERSHEY_SIMPLEX
LEGEND_SCALE = 0.4
LABEL_SCALE = 0.35
POI_COLORS = {"open": COLOR_POI_OPEN, "done": COLOR_POI_DONE, "cancelled": COLOR_POI_CANCELLED}
LEGEND_ENTRIES = {
    "map": ("map (white free, black wall, grey unknown)", COLOR_FREE),
    "costmap": ("local costmap", COLOR_COSTMAP),
    "lidar": ("lidar", COLOR_LIDAR),
    "footprint": ("footprint + heading", COLOR_FOOTPRINT),
    "reach": ("arm reach", COLOR_REACH),
    "path": ("Nav2 plan", COLOR_PATH),
    "pois": ("POI (orange open, green done)", COLOR_POI_OPEN),
    "objects": ("remembered object", COLOR_OBJECT),
}


@dataclass(frozen=True)
class GridLayer:
    """An occupancy grid with the pose of its frame in the map frame.

    Cell (row, col) covers [origin + col * resolution, origin + (col + 1) * resolution) along the grid x axis and the
    same along y for the row, in the grid's frame; that frame sits at (frame_x, frame_y, frame_yaw) in the map frame.
    """

    data: np.ndarray  # HxW, -1 unknown, 0..100 occupancy / cost
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0
    frame_x: float = 0.0
    frame_y: float = 0.0
    frame_yaw: float = 0.0
    age_s: float | None = None


@dataclass
class TopdownInputs:
    """Everything get_topdown_view needs from the robot; None means the data is unavailable."""

    pose: tuple[float, float, float] | None  # map -> base_link (x, y, yaw)
    map: GridLayer | None = None
    costmap: GridLayer | None = None
    scan_points: np.ndarray | None = None  # Nx2 in base_link
    plan: np.ndarray | None = None  # Nx2 in the map frame
    ages: dict[str, float] = field(default_factory=dict)  # data age per source (s)
    missing: dict[str, str] = field(default_factory=dict)  # layer -> why it is unavailable


@dataclass(frozen=True)
class TopdownStyle:
    """Robot geometry drawn into the view (m, base_link)."""

    footprint_length_m: float
    footprint_width_m: float
    reach_m: float
    reach_x_m: float = 0.0
    reach_y_m: float = 0.0


@dataclass
class TopdownRender:
    """A rendered view plus which layers made it in."""

    image: np.ndarray
    scale_m_per_px: float
    layers_present: list[str]
    layers_missing: dict[str, str]


def grid_layer(
    data: Sequence[int],
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
    frame_pose: tuple[float, float, float],
    age_s: float | None,
) -> GridLayer:
    """Build a GridLayer from OccupancyGrid message fields.

    Args:
        data (Sequence[int]): Row-major cells.
        width (int): Columns.
        height (int): Rows.
        resolution (float): Cell size (m).
        origin_x (float): Pose of cell (0, 0) in the grid frame, x (m).
        origin_y (float): Same, y (m).
        origin_yaw (float): Same, yaw (rad).
        frame_pose (tuple[float, float, float]): (x, y, yaw) of the grid's frame in the map frame.
        age_s (float | None): Age of the message (s).

    Returns:
        GridLayer: The grid.
    """
    cells = np.asarray(data, dtype=np.int16)
    if cells.size != width * height:
        raise ValueError(f"occupancy grid has {cells.size} cells, expected {width} x {height}")
    return GridLayer(
        data=cells.reshape(height, width),
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        origin_yaw=origin_yaw,
        frame_x=frame_pose[0],
        frame_y=frame_pose[1],
        frame_yaw=frame_pose[2],
        age_s=age_s,
    )


def transform_points(points: np.ndarray, frame_pose: tuple[float, float, float]) -> np.ndarray:
    """Express points given in a frame in the map frame.

    Args:
        points (np.ndarray): Nx2 points in the source frame.
        frame_pose (tuple[float, float, float]): (x, y, yaw) of the source frame in the map frame.

    Returns:
        np.ndarray: Nx2 map-frame points.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    c, s = math.cos(frame_pose[2]), math.sin(frame_pose[2])
    return np.stack(
        [frame_pose[0] + c * pts[:, 0] - s * pts[:, 1], frame_pose[1] + s * pts[:, 0] + c * pts[:, 1]], axis=1
    )


def base_to_px(x_b: float, y_b: float, px: int, scale: float) -> tuple[float, float]:
    """Image coordinates (u right, v down) of a base_link point; the robot is at the image centre.

    Args:
        x_b (float): Forward distance (m).
        y_b (float): Leftward distance (m).
        px (int): Image side (pixels).
        scale (float): Metres per pixel.

    Returns:
        tuple[float, float]: (u, v) in pixels.
    """
    return px / 2 - y_b / scale, px / 2 - x_b / scale


def map_to_base(x: float, y: float, pose: tuple[float, float, float]) -> tuple[float, float]:
    """Express a map-frame point in base_link.

    Args:
        x (float): Map x (m).
        y (float): Map y (m).
        pose (tuple[float, float, float]): Robot (x, y, yaw) in the map frame.

    Returns:
        tuple[float, float]: (forward, left) in metres.
    """
    dx, dy = x - pose[0], y - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return c * dx + s * dy, -s * dx + c * dy


def map_points_to_px(points: np.ndarray, pose: tuple[float, float, float], px: int, scale: float) -> np.ndarray:
    """Vectorised map -> image transform.

    Args:
        points (np.ndarray): Nx2 map-frame points.
        pose (tuple[float, float, float]): Robot (x, y, yaw) in the map frame.
        px (int): Image side.
        scale (float): Metres per pixel.

    Returns:
        np.ndarray: Nx2 float (u, v) pixel coordinates.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    dx, dy = pts[:, 0] - pose[0], pts[:, 1] - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    x_b, y_b = c * dx + s * dy, -s * dx + c * dy
    return np.stack([px / 2 - y_b / scale, px / 2 - x_b / scale], axis=1)


def sample_grid(grid: GridLayer, map_x: np.ndarray, map_y: np.ndarray) -> np.ndarray:
    """Nearest-cell values of a grid at map-frame points.

    Args:
        grid (GridLayer): Grid with its frame pose.
        map_x (np.ndarray): Map-frame x of the sample points (m).
        map_y (np.ndarray): Map-frame y of the sample points (m).

    Returns:
        np.ndarray: int16 cell values, OUTSIDE (-2) where a point lies outside the grid.
    """
    cf, sf = math.cos(grid.frame_yaw), math.sin(grid.frame_yaw)
    x0 = grid.frame_x + cf * grid.origin_x - sf * grid.origin_y
    y0 = grid.frame_y + sf * grid.origin_x + cf * grid.origin_y
    theta = grid.frame_yaw + grid.origin_yaw
    c, s = math.cos(theta), math.sin(theta)
    dx, dy = map_x - x0, map_y - y0
    col = np.floor((c * dx + s * dy) / grid.resolution).astype(np.int64)
    row = np.floor((-s * dx + c * dy) / grid.resolution).astype(np.int64)
    height, width = grid.data.shape
    inside = (col >= 0) & (col < width) & (row >= 0) & (row < height)
    out = np.full(map_x.shape, OUTSIDE, np.int16)
    out[inside] = grid.data[row[inside], col[inside]]
    return out


def pixel_map_coords(pose: tuple[float, float, float], px: int, scale: float) -> tuple[np.ndarray, np.ndarray]:
    """Map-frame coordinates of every pixel centre.

    Args:
        pose (tuple[float, float, float]): Robot (x, y, yaw) in the map frame.
        px (int): Image side.
        scale (float): Metres per pixel.

    Returns:
        tuple[np.ndarray, np.ndarray]: (map_x, map_y), each px x px.
    """
    idx = np.arange(px) + 0.5
    x_b = (px / 2 - idx)[:, None] * scale * np.ones((1, px))  # rows go down: forward decreases
    y_b = (px / 2 - idx)[None, :] * scale * np.ones((px, 1))  # columns go right: left decreases
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return pose[0] + c * x_b - s * y_b, pose[1] + s * x_b + c * y_b


def draw_map(image: np.ndarray, grid: GridLayer, pose: tuple[float, float, float], scale: float) -> None:
    """Paint the SLAM map: white free, black occupied, grey unknown/outside.

    Args:
        image (np.ndarray): BGR canvas (modified in place).
        grid (GridLayer): Occupancy grid.
        pose (tuple[float, float, float]): Robot pose in the map frame.
        scale (float): Metres per pixel.
    """
    values = sample_grid(grid, *pixel_map_coords(pose, image.shape[0], scale))
    image[(values >= 0) & (values <= FREE_THRESHOLD)] = COLOR_FREE
    image[values >= OCCUPIED_THRESHOLD] = COLOR_OCCUPIED
    # Cells between the free and occupied thresholds are uncertain: a mid grey keeps them distinguishable.
    mid = (values > FREE_THRESHOLD) & (values < OCCUPIED_THRESHOLD)
    image[mid] = (120, 120, 120)


def draw_costmap(image: np.ndarray, grid: GridLayer, pose: tuple[float, float, float], scale: float) -> None:
    """Blend the local costmap over the canvas: orange, more opaque for higher cost; free and unknown stay untouched.

    Args:
        image (np.ndarray): BGR canvas (modified in place).
        grid (GridLayer): Costmap.
        pose (tuple[float, float, float]): Robot pose in the map frame.
        scale (float): Metres per pixel.
    """
    values = sample_grid(grid, *pixel_map_coords(pose, image.shape[0], scale))
    mask = values > 0
    alpha = (0.2 + 0.6 * np.clip(values[mask], 0, 100) / 100.0)[:, None]
    blended = image[mask].astype(float) * (1 - alpha) + np.array(COLOR_COSTMAP, float) * alpha
    image[mask] = blended.astype(np.uint8)


def draw_points(image: np.ndarray, uv: np.ndarray, color: tuple[int, int, int]) -> None:
    """Set 2x2 pixel blocks at the given image coordinates (points outside the image are skipped).

    Args:
        image (np.ndarray): BGR canvas (modified in place).
        uv (np.ndarray): Nx2 (u, v) pixel coordinates.
        color (tuple[int, int, int]): BGR colour.
    """
    size = image.shape[0]
    cols = np.floor(uv[:, 0]).astype(np.int64)
    rows = np.floor(uv[:, 1]).astype(np.int64)
    for dr in (0, 1):
        for dc in (0, 1):
            r, c = rows + dr - 0, cols + dc - 0
            ok = (r >= 0) & (r < size) & (c >= 0) & (c < size)
            image[r[ok], c[ok]] = color


def int_pt(u: float, v: float) -> tuple[int, int]:
    """Round image coordinates for OpenCV drawing.

    Args:
        u (float): Column.
        v (float): Row.

    Returns:
        tuple[int, int]: (column, row) integers.
    """
    return int(round(u)), int(round(v))


def draw_label(image: np.ndarray, text: str, u: float, v: float, color: tuple[int, int, int]) -> None:
    """Put a small text label at an image position when it lies inside the image.

    Args:
        image (np.ndarray): BGR canvas (modified in place).
        text (str): Label.
        u (float): Column of the text origin.
        v (float): Row of the text origin.
        color (tuple[int, int, int]): BGR colour.
    """
    size = image.shape[0]
    if 0 <= u < size and 0 <= v < size and text:
        cv2.putText(image, text, int_pt(u, v), FONT, LABEL_SCALE, color, 1, cv2.LINE_AA)


def draw_footprint(image: np.ndarray, style: TopdownStyle, scale: float) -> None:
    """Draw the footprint rectangle (outline) and the heading arrow pointing up from the centre.

    Args:
        image (np.ndarray): BGR canvas (modified in place).
        style (TopdownStyle): Robot geometry.
        scale (float): Metres per pixel.
    """
    px = image.shape[0]
    half_l, half_w = style.footprint_length_m / 2, style.footprint_width_m / 2
    corners = [(half_l, half_w), (half_l, -half_w), (-half_l, -half_w), (-half_l, half_w)]
    poly = np.array([int_pt(*base_to_px(x, y, px, scale)) for x, y in corners], np.int32)
    cv2.polylines(image, [poly], True, COLOR_FOOTPRINT, 1)
    centre = int_pt(*base_to_px(0.0, 0.0, px, scale))
    tip = int_pt(*base_to_px(half_l + HEADING_EXTRA_M, 0.0, px, scale))
    cv2.arrowedLine(image, centre, tip, COLOR_HEADING, 1, tipLength=0.25)


def draw_reach(image: np.ndarray, style: TopdownStyle, scale: float) -> None:
    """Draw the arm reach circle about the arm mount.

    Args:
        image (np.ndarray): BGR canvas (modified in place).
        style (TopdownStyle): Robot geometry.
        scale (float): Metres per pixel.
    """
    centre = int_pt(*base_to_px(style.reach_x_m, style.reach_y_m, image.shape[0], scale))
    cv2.circle(image, centre, max(1, round(style.reach_m / scale)), COLOR_REACH, 1, cv2.LINE_AA)


def draw_path(image: np.ndarray, plan: np.ndarray, pose: tuple[float, float, float], scale: float) -> None:
    """Draw the Nav2 plan as a polyline.

    Args:
        image (np.ndarray): BGR canvas (modified in place).
        plan (np.ndarray): Nx2 map-frame points.
        pose (tuple[float, float, float]): Robot pose in the map frame.
        scale (float): Metres per pixel.
    """
    uv = map_points_to_px(plan, pose, image.shape[0], scale)
    cv2.polylines(image, [np.round(uv).astype(np.int32).reshape(-1, 1, 2)], False, COLOR_PATH, 1)


def draw_pois(image: np.ndarray, pois: Sequence[dict], pose: tuple[float, float, float], scale: float) -> None:
    """Draw POIs: points as circles of their radius, areas as polygons, each with its name.

    Args:
        image (np.ndarray): BGR canvas (modified in place).
        pois (Sequence[dict]): POI JSON objects (map frame).
        pose (tuple[float, float, float]): Robot pose in the map frame.
        scale (float): Metres per pixel.
    """
    px = image.shape[0]
    for poi in pois:
        color = POI_COLORS.get(str(poi.get("status", "open")), COLOR_POI_OPEN)
        polygon = poi.get("polygon") or []
        if poi.get("kind") == "area" and len(polygon) >= 3:
            uv = map_points_to_px(np.array(polygon, float), pose, px, scale)
            cv2.polylines(image, [np.round(uv).astype(np.int32).reshape(-1, 1, 2)], True, color, 2)
        else:
            u, v = map_points_to_px(np.array([[poi["x"], poi["y"]]], float), pose, px, scale)[0]
            radius = max(3, round(float(poi.get("radius_m", 0.2)) / scale))
            cv2.circle(image, int_pt(u, v), radius, color, 2)
            cv2.circle(image, int_pt(u, v), 2, color, -1)
        u, v = map_points_to_px(np.array([[poi["x"], poi["y"]]], float), pose, px, scale)[0]
        draw_label(image, str(poi.get("name", "")), u + 5, v - 5, color)


def draw_objects(image: np.ndarray, objects: Sequence[dict], pose: tuple[float, float, float], scale: float) -> None:
    """Draw remembered objects as filled diamonds with their label.

    Args:
        image (np.ndarray): BGR canvas (modified in place).
        objects (Sequence[dict]): Objects with label, x, y (map frame).
        pose (tuple[float, float, float]): Robot pose in the map frame.
        scale (float): Metres per pixel.
    """
    px = image.shape[0]
    for obj in objects:
        u, v = map_points_to_px(np.array([[obj["x"], obj["y"]]], float), pose, px, scale)[0]
        c = int_pt(u, v)
        diamond = np.array([[c[0], c[1] - 4], [c[0] + 4, c[1]], [c[0], c[1] + 4], [c[0] - 4, c[1]]], np.int32)
        cv2.fillPoly(image, [diamond], COLOR_OBJECT)
        draw_label(image, str(obj.get("label", "")), u + 6, v + 4, COLOR_OBJECT)


def draw_legend(image: np.ndarray, layers: Sequence[str]) -> None:
    """Legend in the top-left corner: a colour swatch and name per drawn layer.

    Args:
        image (np.ndarray): BGR canvas (modified in place).
        layers (Sequence[str]): Layers that were drawn.
    """
    lines = [LEGEND_ENTRIES[name] for name in layers]
    row_h = 14
    width = 8 + max((len(text) for text, _ in lines), default=0) * 7 + 16
    cv2.rectangle(image, (0, 0), (min(width, image.shape[1]), 4 + row_h * len(lines)), COLOR_LEGEND_BG, -1)
    for i, (text, color) in enumerate(lines):
        y = 4 + row_h * i
        cv2.rectangle(image, (3, y + 2), (11, y + 10), color, -1)
        cv2.rectangle(image, (3, y + 2), (11, y + 10), COLOR_TEXT, 1)
        cv2.putText(image, text, (15, y + 10), FONT, LEGEND_SCALE, COLOR_TEXT, 1, cv2.LINE_AA)


def draw_scale_bar(image: np.ndarray, scale: float) -> None:
    """A 0.5 m scale bar in the bottom-left corner (skipped when it would not fit).

    Args:
        image (np.ndarray): BGR canvas (modified in place).
        scale (float): Metres per pixel.
    """
    px = image.shape[0]
    length = round(SCALE_BAR_M / scale)
    if length > px - 24:
        return
    y = px - 10
    cv2.line(image, (10, y), (10 + length, y), COLOR_TEXT, 2)
    cv2.line(image, (10, y - 4), (10, y + 4), COLOR_TEXT, 1)
    cv2.line(image, (10 + length, y - 4), (10 + length, y + 4), COLOR_TEXT, 1)
    cv2.putText(image, f"{SCALE_BAR_M:g} m", (14, y - 6), FONT, LEGEND_SCALE, COLOR_TEXT, 1, cv2.LINE_AA)


def layer_gaps(
    inputs: TopdownInputs, layers: Sequence[str], pois: Sequence[dict] | None, objects: Sequence[dict] | None
) -> dict[str, str]:
    """Reason per requested layer that cannot be drawn.

    Args:
        inputs (TopdownInputs): Robot data.
        layers (Sequence[str]): Requested layers.
        pois (Sequence[dict] | None): POIs, None when unavailable.
        objects (Sequence[dict] | None): Remembered objects, None when unavailable.

    Returns:
        dict[str, str]: Layer -> reason it is missing.
    """
    have = {
        "map": inputs.map is not None,
        "costmap": inputs.costmap is not None,
        "lidar": inputs.scan_points is not None,
        "path": inputs.plan is not None,
        "pois": pois is not None,
        "objects": objects is not None,
        "footprint": True,
        "reach": True,
    }
    gaps: dict[str, str] = {}
    for layer in layers:
        if layer in POSE_LAYERS and inputs.pose is None:
            gaps[layer] = inputs.missing.get(layer) or "robot pose (map->base_link) unavailable"
        elif not have[layer]:
            gaps[layer] = inputs.missing.get(layer) or f"no {layer} data available"
        elif layer in inputs.missing:
            gaps[layer] = inputs.missing[layer]
    return gaps


def render_topdown(
    inputs: TopdownInputs,
    layers: Sequence[str],
    radius_m: float,
    px: int,
    style: TopdownStyle,
    pois: Sequence[dict] | None = None,
    objects: Sequence[dict] | None = None,
) -> TopdownRender:
    """Render the robot-up view. Layers whose data is missing are listed, never fabricated.

    Args:
        inputs (TopdownInputs): Robot data (pose, grids, scan points, plan, ages, known gaps).
        layers (Sequence[str]): Requested layers (subset of ALL_LAYERS).
        radius_m (float): Half size of the view (m).
        px (int): Image side (pixels).
        style (TopdownStyle): Footprint and reach geometry.
        pois (Sequence[dict] | None): POI JSON objects, None when unavailable.
        objects (Sequence[dict] | None): Remembered objects (label, x, y), None when unavailable.

    Returns:
        TopdownRender: Image, scale and the present/missing layers.
    """
    scale = 2 * radius_m / px
    gaps = layer_gaps(inputs, layers, pois, objects)
    drawn = [layer for layer in ALL_LAYERS if layer in layers and layer not in gaps]
    image = np.full((px, px, 3), COLOR_UNKNOWN, np.uint8)
    pose = inputs.pose
    for layer in drawn:
        if layer == "map" and inputs.map is not None and pose is not None:
            draw_map(image, inputs.map, pose, scale)
        elif layer == "costmap" and inputs.costmap is not None and pose is not None:
            draw_costmap(image, inputs.costmap, pose, scale)
        elif layer == "reach":
            draw_reach(image, style, scale)
        elif layer == "path" and inputs.plan is not None and pose is not None:
            draw_path(image, inputs.plan, pose, scale)
        elif layer == "pois" and pois is not None and pose is not None:
            draw_pois(image, pois, pose, scale)
        elif layer == "objects" and objects is not None and pose is not None:
            draw_objects(image, objects, pose, scale)
        elif layer == "lidar" and inputs.scan_points is not None:
            draw_points(image, base_points_to_px(inputs.scan_points, px, scale), COLOR_LIDAR)
        elif layer == "footprint":
            draw_footprint(image, style, scale)
    draw_legend(image, drawn)
    draw_scale_bar(image, scale)
    return TopdownRender(image=image, scale_m_per_px=scale, layers_present=drawn, layers_missing=gaps)


def base_points_to_px(points: np.ndarray, px: int, scale: float) -> np.ndarray:
    """Vectorised base_link -> image transform.

    Args:
        points (np.ndarray): Nx2 (forward, left) points in metres.
        px (int): Image side.
        scale (float): Metres per pixel.

    Returns:
        np.ndarray: Nx2 float (u, v) pixel coordinates.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    return np.stack([px / 2 - pts[:, 1] / scale, px / 2 - pts[:, 0] / scale], axis=1)


def encode_png(image: np.ndarray) -> bytes:
    """PNG-encode a BGR image.

    Args:
        image (np.ndarray): HxWx3 uint8 image.

    Returns:
        bytes: PNG data.
    """
    ok, buf = cv2.imencode(".png", image)
    if not ok:
        raise ImageEncodingError("PNG encoding failed")
    return buf.tobytes()
