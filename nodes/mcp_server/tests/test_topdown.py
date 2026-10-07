"""Tests for mcp_server.topdown: robot-up coordinate transforms, layer placement at pixel level, missing layers."""

import math

import cv2
import numpy as np
import pytest

from mcp_server.perception import scan_points
from mcp_server.topdown import (
    ALL_LAYERS,
    COLOR_FOOTPRINT,
    COLOR_HEADING,
    COLOR_LIDAR,
    COLOR_OBJECT,
    COLOR_PATH,
    GridLayer,
    TopdownInputs,
    TopdownStyle,
    base_to_px,
    encode_png,
    grid_layer,
    map_to_base,
    render_topdown,
    sample_grid,
    transform_points,
)

PX = 400
RADIUS = 2.0
SCALE = 2 * RADIUS / PX  # 0.01 m per pixel
CENTRE = PX / 2
STYLE = TopdownStyle(footprint_length_m=0.47, footprint_width_m=0.386, reach_m=0.41, reach_x_m=0.0, reach_y_m=0.0)


def has_color(image: np.ndarray, color: tuple[int, int, int], u: float, v: float, tol_px: int = 2) -> bool:
    """Whether a pixel of `color` lies within tol_px of (u, v)."""
    u0, v0 = int(round(u)), int(round(v))
    window = image[max(v0 - tol_px, 0) : v0 + tol_px + 1, max(u0 - tol_px, 0) : u0 + tol_px + 1]
    return bool(np.any(np.all(window == np.array(color, np.uint8), axis=-1)))


def render(inputs: TopdownInputs, layers: list[str] | None = None, **kwargs):
    return render_topdown(inputs, layers or list(ALL_LAYERS), RADIUS, PX, STYLE, **kwargs)


def pose_inputs(**kwargs) -> TopdownInputs:
    return TopdownInputs(pose=(0.0, 0.0, 0.0), **kwargs)


def test_base_to_px_forward_is_up_and_left_is_left() -> None:
    assert base_to_px(0.0, 0.0, PX, SCALE) == pytest.approx((CENTRE, CENTRE))
    u, v = base_to_px(1.0, 0.0, PX, SCALE)
    assert (u, v) == pytest.approx((CENTRE, CENTRE - 100))
    u, v = base_to_px(0.0, 0.5, PX, SCALE)
    assert (u, v) == pytest.approx((CENTRE - 50, CENTRE))


def test_map_to_base_rotates_by_robot_yaw() -> None:
    # Robot at (1, 1) facing +y (map north): a map point north of it is straight ahead.
    xb, yb = map_to_base(1.0, 2.0, (1.0, 1.0, math.pi / 2))
    assert (xb, yb) == pytest.approx((1.0, 0.0), abs=1e-9)
    xb, yb = map_to_base(0.0, 1.0, (1.0, 1.0, math.pi / 2))  # west of the robot = to its left
    assert (xb, yb) == pytest.approx((0.0, 1.0), abs=1e-9)


def test_footprint_and_heading_pixels() -> None:
    result = render(pose_inputs(), ["footprint"])
    img = result.image
    half_len, half_wid = 0.47 / 2 / SCALE, 0.386 / 2 / SCALE
    # Front edge centre, rear edge centre, both side edges, all four corners carry the footprint colour.
    for du, dv in [(0, -half_len), (0, half_len), (-half_wid, 0), (half_wid, 0)]:
        assert has_color(img, COLOR_FOOTPRINT, CENTRE + du, CENTRE + dv), (du, dv)
    for du in (-half_wid, half_wid):
        for dv in (-half_len, half_len):
            assert has_color(img, COLOR_FOOTPRINT, CENTRE + du, CENTRE + dv)
    # Nothing of the footprint beyond its outline.
    assert not has_color(img, COLOR_FOOTPRINT, CENTRE, CENTRE - half_len - 8, tol_px=1)
    assert not has_color(img, COLOR_FOOTPRINT, CENTRE + half_wid + 8, CENTRE, tol_px=1)
    # Heading arrow: from the centre straight up (robot-up), past the front edge, none towards the rear.
    assert has_color(img, COLOR_HEADING, CENTRE, CENTRE - 0.15 / SCALE, tol_px=1)
    assert has_color(img, COLOR_HEADING, CENTRE, CENTRE - (0.47 / 2 + 0.08) / SCALE, tol_px=2)
    assert not has_color(img, COLOR_HEADING, CENTRE, CENTRE + 0.15 / SCALE, tol_px=1)
    assert not has_color(img, COLOR_HEADING, CENTRE + 0.15 / SCALE, CENTRE, tol_px=1)


def test_scale_metadata_and_image_size() -> None:
    result = render(pose_inputs(), ["footprint"])
    assert result.image.shape == (PX, PX, 3)
    assert result.scale_m_per_px == pytest.approx(SCALE)


def test_lidar_points_are_placed_in_base_frame() -> None:
    pts = np.array([[1.0, 0.0], [0.0, 0.5], [-0.5, -0.5]])
    result = render(TopdownInputs(pose=None, scan_points=pts), ["lidar"])
    assert has_color(result.image, COLOR_LIDAR, CENTRE, CENTRE - 100, tol_px=1)  # 1 m ahead
    assert has_color(result.image, COLOR_LIDAR, CENTRE - 50, CENTRE, tol_px=1)  # 0.5 m left
    assert has_color(result.image, COLOR_LIDAR, CENTRE + 50, CENTRE + 50, tol_px=1)  # behind, to the right
    assert result.layers_present == ["lidar"]


def test_map_layer_is_cropped_robot_up() -> None:
    # 10 x 10 m map, 0.05 m cells, origin (-5, -5), free everywhere except a wall strip north of the robot at y = 1.0.
    size = 200
    data = np.zeros((size, size), np.int16)
    row = int((1.0 + 5.0) / 0.05)
    data[row : row + 2, :] = 100
    grid = GridLayer(data=data, resolution=0.05, origin_x=-5.0, origin_y=-5.0)
    # Robot at the origin facing east (yaw 0): the wall (north) is on the robot's left.
    east = render(TopdownInputs(pose=(0.0, 0.0, 0.0), map=grid), ["map"]).image
    assert tuple(east[int(CENTRE), int(CENTRE - 105)]) == (0, 0, 0)  # 1.05 m left of centre: occupied
    assert tuple(east[int(CENTRE), int(CENTRE)]) != (0, 0, 0)
    # Facing north (yaw pi/2): the same wall is straight ahead, i.e. above the centre.
    north = render(TopdownInputs(pose=(0.0, 0.0, math.pi / 2), map=grid), ["map"]).image
    assert tuple(north[int(CENTRE - 105), int(CENTRE)]) == (0, 0, 0)
    assert tuple(north[int(CENTRE + 105), int(CENTRE)]) != (0, 0, 0)


def test_sample_grid_honours_frame_offset_and_rotation() -> None:
    data = np.full((10, 10), -1, np.int16)
    data[2, 3] = 77  # row 2, column 3
    # Grid frame sits at (10, 20) rotated 90 deg in the map frame; grid origin (1, 2) inside that frame.
    grid = GridLayer(
        data=data, resolution=0.5, origin_x=1.0, origin_y=2.0, frame_x=10.0, frame_y=20.0, frame_yaw=math.pi / 2
    )
    # Cell (col 3, row 2) centre in the grid frame: (1 + 3.5 * 0.5, 2 + 2.5 * 0.5) = (2.75, 3.25).
    # Rotated 90 deg + offset: map x = 10 - 3.25, map y = 20 + 2.75.
    values = sample_grid(grid, np.array([10 - 3.25, 0.0]), np.array([20 + 2.75, 0.0]))
    assert values[0] == 77
    assert values[1] == -2  # outside the grid


def test_costmap_overlay_tints_lethal_cells_only() -> None:
    data = np.zeros((40, 40), np.int16)
    data[10:12, 20:22] = 100
    costmap = GridLayer(data=data, resolution=0.1, origin_x=-2.0, origin_y=-2.0)
    base = render(pose_inputs(), ["map"]).image
    img = render(pose_inputs(costmap=costmap), ["costmap"]).image
    assert base.shape == img.shape
    lethal_u, lethal_v = base_to_px(0.0, 0.0, PX, SCALE)
    # Lethal cell: x in [0.0, 0.2] at row 10 -> y_map = -2 + 1.0 = -1.0 .. -0.8 ; x_map = 0.0 .. 0.2.
    x_b, y_b = 0.1, -0.9
    u, v = base_to_px(x_b, y_b, PX, SCALE)
    assert not np.array_equal(img[int(v), int(u)], img[int(lethal_v), int(lethal_u)])
    assert np.array_equal(img[int(CENTRE), int(CENTRE) - 150], img[int(CENTRE), int(CENTRE) - 160])  # free: untouched


def test_path_and_object_layers_follow_the_robot_pose() -> None:
    inputs = TopdownInputs(pose=(5.0, 5.0, math.pi / 2), plan=np.array([[5.0, 5.5], [5.0, 6.5]]))
    objects = [{"label": "cup", "x": 4.0, "y": 5.0}]  # 1 m behind-left? robot faces north: west = left
    result = render(inputs, ["path", "objects"], objects=objects)
    assert has_color(result.image, COLOR_PATH, CENTRE, CENTRE - 100, tol_px=1)  # path 1 m straight ahead
    assert has_color(result.image, COLOR_OBJECT, CENTRE - 100, CENTRE, tol_px=3)  # object 1 m to the left
    assert result.layers_present == ["path", "objects"]


def test_pois_point_and_area_are_drawn_with_status_colours() -> None:
    pois = [
        {"id": "a", "kind": "point", "name": "dock", "x": 1.0, "y": 0.0, "radius_m": 0.2, "status": "open"},
        {
            "id": "b",
            "kind": "area",
            "name": "rug",
            "x": 0.0,
            "y": 1.0,
            "polygon": [[-0.3, 0.7], [0.3, 0.7], [0.3, 1.3], [-0.3, 1.3]],
            "status": "done",
        },
    ]
    result = render(pose_inputs(), ["pois"], pois=pois)
    img = result.image
    blank = render(pose_inputs(), ["pois"], pois=[]).image
    assert not np.array_equal(
        img[int(CENTRE) - 100 - 3 : int(CENTRE) - 100 + 3, int(CENTRE) - 3 : int(CENTRE) + 3],
        blank[int(CENTRE) - 100 - 3 : int(CENTRE) - 100 + 3, int(CENTRE) - 3 : int(CENTRE) + 3],
    )
    # Area polygon corner: (0.3, 1.3) map -> ahead 0.3 m, left 1.3 m.
    assert not np.array_equal(img, blank)
    assert result.layers_present == ["pois"]


def test_missing_layers_are_listed_with_reason_and_never_drawn() -> None:
    inputs = TopdownInputs(pose=(0.0, 0.0, 0.0), missing={"costmap": "no /local_costmap/costmap message within 1.5 s"})
    result = render(inputs)  # all layers, only the pose is known
    assert set(result.layers_present) == {"footprint", "reach"}
    assert set(result.layers_missing) == set(ALL_LAYERS) - {"footprint", "reach"}
    assert "1.5 s" in result.layers_missing["costmap"]
    assert result.layers_missing["map"]
    only_fp = render(inputs, ["footprint"])
    clean = render(inputs, ["footprint", "reach", "lidar"])
    assert only_fp.layers_missing == {}
    assert "lidar" in clean.layers_missing


def test_without_pose_map_frame_layers_are_missing_but_robot_layers_render() -> None:
    result = render(TopdownInputs(pose=None, scan_points=np.array([[1.0, 0.0]])))
    assert set(result.layers_present) == {"footprint", "reach", "lidar"}
    for layer in ("map", "costmap", "path", "pois", "objects"):
        assert "pose" in result.layers_missing[layer]


def test_encode_png_roundtrip() -> None:
    img = render(pose_inputs(), ["footprint"]).image
    decoded = cv2.imdecode(np.frombuffer(encode_png(img), np.uint8), cv2.IMREAD_COLOR)
    assert np.array_equal(decoded, img)


def test_scan_points_transform_into_base_link_and_drop_invalid_returns() -> None:
    # Laser 0.1 m ahead of base_link, yawed 90 deg left: a return straight ahead of the laser is to base_link's left.
    pts = scan_points([1.0, float("inf"), 0.01, 99.0], 0.0, 0.1, 0.05, 10.0, 0.1, 0.0, math.pi / 2)
    assert pts.shape == (1, 2)
    assert pts[0] == pytest.approx([0.1, 1.0])


def test_grid_layer_from_message_fields_and_frame_pose() -> None:
    grid = grid_layer([0, 100, -1, 50], 2, 2, 0.5, 1.0, 2.0, 0.0, (10.0, 20.0, math.pi / 2), 0.4)
    assert grid.data.shape == (2, 2) and grid.data[0, 1] == 100
    assert (grid.frame_x, grid.frame_y, grid.frame_yaw) == (10.0, 20.0, math.pi / 2)
    assert grid.age_s == 0.4
    with pytest.raises(ValueError, match="cells"):
        grid_layer([0, 1, 2], 2, 2, 0.5, 0.0, 0.0, 0.0, (0.0, 0.0, 0.0), None)


def test_transform_points_into_map_frame() -> None:
    pts = transform_points(np.array([[1.0, 0.0], [0.0, 1.0]]), (10.0, 20.0, math.pi / 2))
    assert pts == pytest.approx(np.array([[10.0, 21.0], [9.0, 20.0]]))
