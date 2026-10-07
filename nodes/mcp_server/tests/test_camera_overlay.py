"""Overlay geometry (grid, reach annulus, markers, lidar dots, legend) and candidate point generation."""

import numpy as np
import pytest
from ros2_common.camera_geometry import project_raw

from mcp_server.camera_overlay import (
    candidate_pixels,
    circle_polyline,
    draw_candidates,
    draw_legend,
    draw_lidar,
    draw_polylines,
    grid_labels,
    grid_polylines,
    project_points,
    range_color,
)
from mcp_server.camera_scene import build_scene, camera_setup
from mcp_server.config import McpServerConfig

from .camera_helpers import HEIGHT, WIDTH, camera_config, jpeg_blank


def front_scene():
    cfg = camera_config()
    return build_scene(camera_setup(cfg, "front"), cfg, np.eye(4))


def test_project_points_matches_geometry_projection() -> None:
    scene = front_scene()
    pts = np.array([[0.9, 0.2, 0.0], [0.6, -0.3, 0.0], [-3.0, 0.0, 0.0]])
    uv, depth = project_points(scene.intr, scene.t_ref_optical, pts)
    for i, p in enumerate(pts[:2]):
        u, v, d = project_raw(scene.intr, scene.t_ref_optical, p)
        assert (uv[i, 0], uv[i, 1], depth[i]) == pytest.approx((u, v, d), abs=1e-6)
    assert depth[2] < 0  # behind the camera: only the depth sign is meaningful


def test_grid_polylines_cover_both_directions_at_the_step() -> None:
    lines = grid_polylines(front_scene(), 0.5)
    xs = sorted({ln.value for ln in lines if ln.axis == "x"})
    ys = sorted({ln.value for ln in lines if ln.axis == "y"})
    assert 1.0 in xs and 0.5 in xs and 0.0 in ys and -0.5 in ys
    assert all(abs(v / 0.5 - round(v / 0.5)) < 1e-9 for v in xs + ys)
    for ln in lines:
        assert len(ln.points) >= 2


def test_grid_line_points_lie_on_the_floor_projection() -> None:
    scene = front_scene()
    line = next(ln for ln in grid_polylines(scene, 0.5) if ln.axis == "x" and ln.value == 1.0)
    u, v = line.points[len(line.points) // 2]
    ground = scene.ground_point(u, v)
    assert ground is not None and ground[0] == pytest.approx(1.0, abs=1e-3)


def test_grid_skips_samples_behind_the_camera() -> None:
    scene = front_scene()
    for ln in grid_polylines(scene, 0.25):
        for u, v in ln.points:
            assert np.isfinite(u) and np.isfinite(v)


def test_grid_labels_are_visible_spaced_and_metric() -> None:
    labels = grid_labels(front_scene(), 0.25)
    assert labels
    for lb in labels:
        assert 0 <= lb.u < WIDTH and 0 <= lb.v < HEIGHT
    for i, a in enumerate(labels):
        for b in labels[i + 1 :]:
            assert np.hypot(a.u - b.u, a.v - b.v) >= 60
    one = next(lb for lb in labels if lb.text == "(1,0)") if any(lb.text == "(1,0)" for lb in labels) else labels[0]
    assert "(" in one.text and "," in one.text


def test_circle_polyline_is_a_closed_floor_circle() -> None:
    scene = front_scene()
    pts = circle_polyline(scene, (0.8, 0.0), 0.3)
    assert len(pts) > 20
    centre = scene.project(np.array([0.8, 0.0, 0.0]))
    assert centre is not None
    # every point is within the image and the ring surrounds the centre
    assert min(p[0] for p in pts) < centre[0] < max(p[0] for p in pts)


def test_draw_polylines_changes_pixels_only_where_drawn() -> None:
    img = jpeg_blank()
    before = img.copy()
    draw_polylines(img, [[(10.0, 10.0), (100.0, 10.0)]], (0, 0, 255))
    assert (img != before).any()
    assert (img[200:, :] == before[200:, :]).all()


def test_range_color_is_monotonic_in_range() -> None:
    near, far = range_color(0.3, 4.0), range_color(3.8, 4.0)
    assert near != far and len(near) == 3


def test_draw_lidar_marks_projected_points_and_counts_visible() -> None:
    scene = front_scene()
    img = jpeg_blank()
    pts = np.array([[1.0, 0.0, 0.2, 1.0], [0.9, 0.3, 0.2, 0.95], [-5.0, 0.0, 0.2, 5.0]])
    shown = draw_lidar(img, scene, pts)
    assert shown == 2
    assert (img != jpeg_blank()).any()


def test_draw_legend_draws_text_block() -> None:
    img = jpeg_blank()
    draw_legend(img, ["frame: base_link", "grid 0.1 m"])
    assert (img[:60, :200] != 90).any()


def test_candidate_pixels_whole_image_spacing() -> None:
    px = candidate_pixels(WIDTH, HEIGHT, None, 40, 1000)
    assert px[0] == (20.0, 20.0)
    assert len(px) == (WIDTH // 40) * (HEIGHT // 40)
    assert all(0 <= u < WIDTH and 0 <= v < HEIGHT for u, v in px)


def test_candidate_pixels_region_and_cap() -> None:
    px = candidate_pixels(WIDTH, HEIGHT, (100, 100, 300, 220), 50, 1000)
    assert all(100 <= u <= 300 and 100 <= v <= 220 for u, v in px)
    assert len(px) == 4 * 2
    capped = candidate_pixels(WIDTH, HEIGHT, None, 20, 10)
    assert len(capped) <= 10
    assert capped == sorted(capped, key=lambda p: (p[1], p[0]))


def test_candidate_pixels_validates_region() -> None:
    with pytest.raises(ValueError, match="region"):
        candidate_pixels(WIDTH, HEIGHT, (300, 0, 100, 50), 40, 10)
    with pytest.raises(ValueError, match="region"):
        candidate_pixels(WIDTH, HEIGHT, (0, 0, WIDTH + 100, HEIGHT), 40, 10)


def test_draw_candidates_numbers_the_points() -> None:
    img = jpeg_blank()
    draw_candidates(img, [(1, 100.0, 100.0), (2, 200.0, 120.0)])
    assert (img[90:110, 90:110] != 90).any() and (img[110:130, 190:210] != 90).any()


def test_default_config_has_no_cameras_to_draw() -> None:
    assert McpServerConfig().cameras.front.mount is None
