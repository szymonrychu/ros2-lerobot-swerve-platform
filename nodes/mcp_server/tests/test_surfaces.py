"""Tests for mcp_server.surfaces: the multi-surface model (half-plane and convex polygon regions relative to the robot
floor), local height lookup, step faces between regions and the edge-aware point/capsule clearance."""

import math

import numpy as np
import pytest
from pydantic import ValidationError

from mcp_server.config import ArmBaseOffset
from mcp_server.surfaces import HalfPlaneEdge, SurfaceRegion, Terrain

MOUNT = ArmBaseOffset(x=0.06, y=-0.05, z=0.10, yaw=0.0)
# 'Stair down 10 cm starting 28 cm in front of the robot': the region x >= 0.28 (base_link) is 0.10 below the floor.
STAIR = SurfaceRegion(
    name="stair",
    frame="base_link",
    height_m=-0.10,
    edge=HalfPlaneEdge(point=(0.28, 0.0), direction=(0.0, -1.0)),  # left of -y is +x
)
TABLE = SurfaceRegion(
    name="table", frame="base_link", height_m=0.15, polygon=[(0.4, -0.1), (0.6, -0.1), (0.6, 0.1), (0.4, 0.1)]
)


def test_region_needs_exactly_one_shape_and_a_convex_polygon() -> None:
    with pytest.raises(ValidationError, match="exactly one"):
        SurfaceRegion(height_m=0.1)
    with pytest.raises(ValidationError, match="exactly one"):
        SurfaceRegion(height_m=0.1, edge=STAIR.edge, polygon=TABLE.polygon)
    with pytest.raises(ValidationError, match="convex"):
        SurfaceRegion(height_m=0.1, polygon=[(0, 0), (1, 0), (0.2, 0.2), (0, 1)])
    with pytest.raises(ValidationError, match="direction"):
        HalfPlaneEdge(point=(0.0, 0.0), direction=(0.0, 0.0))
    clockwise = SurfaceRegion(height_m=0.1, polygon=[(0, 0), (0, 1), (1, 1), (1, 0)])  # either winding is accepted
    assert Terrain([clockwise], 0.0, MOUNT).height_at((0.5, 0.5)) == pytest.approx(0.1)


def test_height_lookup_uses_the_regions_and_the_default_outside() -> None:
    terrain = Terrain([STAIR], 0.0, MOUNT)
    assert terrain.height_at((0.1, 0.0)) == 0.0
    assert terrain.height_at((0.3, 0.5)) == pytest.approx(-0.10)
    assert terrain.region_at((0.3, 0.0)) == "stair" and terrain.region_at((0.1, 0.0)) is None
    assert Terrain([STAIR], -0.02, MOUNT).height_at((0.1, 0.0)) == pytest.approx(-0.02)


def test_later_regions_win_so_a_hole_can_cut_into_a_table() -> None:
    hole = SurfaceRegion(
        name="hole", frame="base_link", height_m=-0.2, polygon=[(0.45, -0.02), (0.5, -0.02), (0.5, 0.02)]
    )
    terrain = Terrain([TABLE, hole], 0.0, MOUNT)
    assert terrain.height_at((0.55, 0.0)) == pytest.approx(0.15)
    assert terrain.height_at((0.49, 0.0)) == pytest.approx(-0.2)


def test_arm_frame_regions_are_converted_with_the_mount() -> None:
    yawed = ArmBaseOffset(x=0.1, y=0.0, z=0.1, yaw=math.pi / 2)
    region = SurfaceRegion(frame="arm", height_m=-0.1, edge=HalfPlaneEdge(point=(0.2, 0.0), direction=(0.0, -1.0)))
    terrain = Terrain([region], 0.0, yawed)
    # arm x 0.2 is base_link y 0.2 (yaw 90 deg) shifted by the mount x 0.1
    assert terrain.height_at((0.1, 0.25)) == pytest.approx(-0.1)
    assert terrain.height_at((0.1, 0.15)) == 0.0


def test_vertical_clearance_above_the_local_surface() -> None:
    terrain = Terrain([STAIR], 0.0, MOUNT)
    c = terrain.clearance(np.array([0.6, 0.0, -0.05]))
    assert c.value == pytest.approx(0.05) and c.feature == "surface 'stair'"
    c = terrain.clearance(np.array([0.0, 0.0, -0.01]))
    assert c.value == pytest.approx(-0.01) and c.feature == "robot floor"


def test_step_face_is_a_vertical_wall_between_the_two_heights() -> None:
    terrain = Terrain([STAIR], 0.0, MOUNT)
    # beside the wall, below the upper surface: horizontal distance to the face
    c = terrain.clearance(np.array([0.30, 0.0, -0.05]))
    assert c.value == pytest.approx(0.02) and c.feature == "step edge of 'stair' (0.10 m step)"
    # above the step: distance to the upper corner, not to the lower surface
    c = terrain.clearance(np.array([0.29, 0.0, 0.005]))
    assert c.value == pytest.approx(math.hypot(0.01, 0.005)) and "step edge" in c.feature
    # well above the step and beside it: the corner is still nearer than the stair below
    c = terrain.clearance(np.array([0.30, 0.0, 0.01]))
    assert c.value == pytest.approx(math.hypot(0.02, 0.01)) and "step edge" in c.feature


def test_polygon_faces_and_no_face_between_equal_heights() -> None:
    terrain = Terrain([TABLE], 0.0, MOUNT)
    c = terrain.clearance(np.array([0.38, 0.0, 0.05]))  # 2 cm in front of the table's near face
    assert c.value == pytest.approx(0.02) and c.feature == "step edge of 'table' (0.15 m step)"
    flush = SurfaceRegion(name="mat", frame="base_link", height_m=0.0, polygon=TABLE.polygon)
    assert Terrain([flush], 0.0, MOUNT).faces == []


def test_lift_raises_surfaces_and_faces() -> None:
    terrain = Terrain([STAIR], 0.0, MOUNT)
    c = terrain.clearance(np.array([0.6, 0.0, -0.05]), lift=lambda x, y: 0.01)
    assert c.value == pytest.approx(0.04)


def test_capsule_clearance_is_the_sampled_minimum_minus_the_radius() -> None:
    terrain = Terrain([STAIR], 0.0, MOUNT)
    a, b = np.array([0.20, 0.0, 0.10]), np.array([0.26, 0.0, -0.05])
    c = terrain.capsule_clearance(a, b, 0.01)
    assert c.value == pytest.approx(-0.06, abs=1e-3)  # the far end is 5 cm below the floor, minus the radius
    c = terrain.capsule_clearance(np.array([0.30, 0.0, -0.02]), np.array([0.40, 0.0, -0.02]), 0.005)
    assert c.value == pytest.approx(0.015) and "step edge" in c.feature


def test_steps_crossed_along_a_horizontal_line() -> None:
    terrain = Terrain([STAIR, TABLE], 0.0, MOUNT)
    crossings = terrain.steps_between((0.0, 0.0), (0.7, 0.0))
    assert [(round(x.at_m, 3), x.name, round(x.drop_m, 3)) for x in crossings] == [
        (0.28, "stair", 0.1),
        (0.4, "table", -0.25),
        (0.6, "table", 0.25),
    ]
    assert terrain.steps_between((0.0, 0.0), (0.2, 0.0)) == []


def test_describe_is_json_ready() -> None:
    out = Terrain([STAIR], 0.0, MOUNT).describe()
    assert out == [
        {"name": "stair", "height_m": -0.1, "kind": "edge", "points_base_link": [[0.28, 0.0]], "direction": [0.0, -1.0]}
    ]


def test_clearance_of_many_points_is_the_minimum_of_the_single_queries() -> None:
    terrain = Terrain([STAIR, TABLE], 0.0, MOUNT)
    rng = np.random.default_rng(3)
    points = rng.uniform([0.0, -0.2, -0.15], [0.7, 0.2, 0.2], size=(40, 3))
    singles = [terrain.clearance(p, lift=lambda x, y: 0.01 * x) for p in points]
    best = min(singles, key=lambda c: c.value)
    many = terrain.clearance_many(points, lift=lambda x, y: 0.01 * x)
    assert many.value == pytest.approx(best.value) and many.feature == best.feature
    assert terrain.clearance_many(points[:1]).value == pytest.approx(terrain.clearance(points[0]).value)
