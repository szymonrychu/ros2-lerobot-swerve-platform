"""Tests for mcp_server.body_model: exact signed distances of points and capsules to the body obstacles, and the body
config (mcp_server.config BodySettings / BodyCheckSettings)."""

import math
from pathlib import Path

import numpy as np
import pytest
import yaml
from pydantic import ValidationError

from mcp_server.body_model import BodyModel, BoxObstacle, CylinderObstacle, capsule_signed_distance
from mcp_server.config import REPO_ROOT, BodyBox, BodyCheckSettings, BodyCylinder, BodySettings, McpServerConfig

UNIT_BOX = BoxObstacle("box", np.array([0.0, 0.0, 0.0]), np.array([1.0, 1.0, 1.0]))
UNIT_CYL = CylinderObstacle("cyl", (0.0, 0.0), 1.0, 0.0, 1.0)
TOL = 1e-9


def pts(*points: tuple[float, float, float]) -> np.ndarray:
    return np.array(points, dtype=np.float64)


@pytest.mark.parametrize(
    ("point", "expected"),
    [
        ((2.0, 0.5, 0.5), 1.0),  # outside a face
        ((2.0, 2.0, 0.5), math.sqrt(2.0)),  # outside an edge
        ((2.0, 2.0, 2.0), math.sqrt(3.0)),  # outside a corner
        ((0.5, 0.5, 0.5), -0.5),  # centre: penetration depth to the nearest face
        ((0.9, 0.5, 0.5), -0.1),  # inside near a face
        ((1.0, 0.5, 0.5), 0.0),  # on the surface
    ],
)
def test_box_point_signed_distance(point: tuple[float, float, float], expected: float) -> None:
    assert UNIT_BOX.signed_distance(pts(point))[0] == pytest.approx(expected, abs=TOL)


@pytest.mark.parametrize(
    ("point", "expected"),
    [
        ((3.0, 0.0, 0.5), 2.0),  # beside the mantle
        ((0.0, 0.0, 3.0), 2.0),  # above the top disc
        ((2.0, 0.0, 2.0), math.sqrt(2.0)),  # outside the rim
        ((0.0, 0.0, 0.5), -0.5),  # on the axis, half way up: nearest are the discs
        ((0.9, 0.0, 0.5), -0.1),  # just inside the mantle
        ((0.0, 0.6, -0.5), 0.5),  # under the bottom disc
    ],
)
def test_cylinder_point_signed_distance(point: tuple[float, float, float], expected: float) -> None:
    assert UNIT_CYL.signed_distance(pts(point))[0] == pytest.approx(expected, abs=TOL)


@pytest.mark.parametrize(
    ("a", "b", "radius", "expected"),
    [
        ((2.0, -1.0, 0.5), (2.0, 2.0, 0.5), 0.1, 0.9),  # parallel to a face
        ((-1.0, 0.5, 0.5), (2.0, 0.5, 0.5), 0.1, -0.6),  # straight through the centre
        ((1.5, 0.5, 2.0), (1.5, 0.5, -1.0), 0.2, 0.3),  # vertical past a face
        ((2.0, 3.0, 0.5), (3.0, 2.0, 0.5), 0.0, math.sqrt(2.0) * 1.5),  # diagonal outside the x=y=1 edge
        ((2.0, 0.5, 0.5), (3.0, 0.5, 0.5), 0.05, 0.95),  # nearest point is the segment end
        ((1.5, 1.5, 1.5), (1.5, 1.5, 1.5), 0.1, math.sqrt(0.75) - 0.1),  # degenerate segment = a sphere
    ],
)
def test_capsule_box(
    a: tuple[float, float, float], b: tuple[float, float, float], radius: float, expected: float
) -> None:
    got = capsule_signed_distance(UNIT_BOX, pts(a), pts(b), radius)
    assert got[0] == pytest.approx(expected, abs=1e-7)


@pytest.mark.parametrize(
    ("a", "b", "radius", "expected"),
    [
        ((-2.0, 0.0, 2.0), (2.0, 0.0, 2.0), 0.1, 0.9),  # horizontal above the top disc
        ((1.5, 0.0, -1.0), (1.5, 0.0, 3.0), 0.2, 0.3),  # vertical beside the mantle
        ((-2.0, 0.0, 0.5), (2.0, 0.0, 0.5), 0.0, -0.5),  # through the axis
        ((3.0, 0.0, 0.5), (5.0, 0.0, 0.5), 0.5, 1.5),  # nearest point is the segment end
        ((2.0, 0.0, 3.0), (2.0, 0.0, 3.0), 0.0, math.sqrt(5.0)),  # point: outside the rim
    ],
)
def test_capsule_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float], radius: float, expected: float
) -> None:
    got = capsule_signed_distance(UNIT_CYL, pts(a), pts(b), radius)
    assert got[0] == pytest.approx(expected, abs=1e-7)


def test_capsule_distance_is_vectorised() -> None:
    a = pts((2.0, -1.0, 0.5), (-1.0, 0.5, 0.5))
    b = pts((2.0, 2.0, 0.5), (2.0, 0.5, 0.5))
    got = capsule_signed_distance(UNIT_BOX, a, b, 0.1)
    assert got == pytest.approx([0.9, -0.6], abs=1e-7)


def test_default_body_is_the_h_shape() -> None:
    body = BodyModel.from_settings(McpServerConfig().body)
    names = [o.name for o in body.obstacles]
    assert names == [
        "frame_left",
        "frame_right",
        "crossbar",
        "wheel_fl",
        "wheel_fr",
        "wheel_rl",
        "wheel_rr",
        "lidar",
    ]
    lidar = body.by_name("lidar")
    assert isinstance(lidar, CylinderObstacle)
    assert lidar.center_xy == (0.158, 0.06) and lidar.z_max == pytest.approx(0.21) and lidar.radius == 0.074
    crossbar = body.by_name("crossbar")
    assert isinstance(crossbar, BoxObstacle)
    assert crossbar.lo == pytest.approx([-0.15, -0.1158, 0.06]) and crossbar.hi == pytest.approx([0.10, 0.1158, 0.10])
    frame = body.by_name("frame_right")
    assert frame.lo == pytest.approx([-0.2125, -0.15080, 0.12]) and frame.hi == pytest.approx([0.2125, -0.11580, 0.18])
    wheel = body.by_name("wheel_fl")
    assert wheel.center_xy == (0.1525, 0.1333) and wheel.radius == 0.0625  # 120 x 35 mm wheel steering: half-diagonal


def test_describe_lists_dimensions() -> None:
    out = BodyModel.from_settings(McpServerConfig().body).describe()
    lidar = next(o for o in out if o["name"] == "lidar")
    assert lidar == {
        "name": "lidar",
        "shape": "vertical_cylinder",
        "center_xy_m": [0.158, 0.06],
        "radius_m": 0.074,
        "z_min_m": 0.1,
        "z_max_m": 0.21,
    }
    crossbar = next(o for o in out if o["name"] == "crossbar")
    assert crossbar["shape"] == "box" and crossbar["size_m"] == pytest.approx([0.25, 0.2316, 0.04])


@pytest.mark.parametrize("size", [(0.0, 0.1, 0.1), (0.1, -0.1, 0.1)])
def test_box_size_must_be_positive(size: tuple[float, float, float]) -> None:
    with pytest.raises(ValidationError):
        BodyBox(name="b", center=(0.0, 0.0, 0.0), size=size)


def test_cylinder_needs_positive_radius_and_height() -> None:
    with pytest.raises(ValidationError):
        BodyCylinder(name="c", x=0.0, y=0.0, radius=0.0, z_min=0.0, z_max=0.1)
    with pytest.raises(ValidationError):
        BodyCylinder(name="c", x=0.0, y=0.0, radius=0.05, z_min=0.1, z_max=0.1)


def test_obstacle_names_unique() -> None:
    box = BodyBox(name="dup", center=(0.0, 0.0, 0.0), size=(0.1, 0.1, 0.1))
    cyl = BodyCylinder(name="dup", x=0.0, y=0.0, radius=0.05, z_min=0.0, z_max=0.1)
    with pytest.raises(ValidationError):
        BodySettings(boxes=[box], cylinders=[cyl])


def test_hard_margin_below_soft_margin() -> None:
    with pytest.raises(ValidationError):
        BodyCheckSettings(soft_margin_m=0.02, hard_margin_m=0.02)
    settings = BodyCheckSettings()
    assert (settings.enabled, settings.soft_margin_m, settings.hard_margin_m, settings.slow_speed_scale) == (
        True,
        0.03,
        0.01,
        0.3,
    )


def test_client_yml_body_block_validates() -> None:
    doc = yaml.safe_load((REPO_ROOT / "ansible/group_vars/client.yml").read_text())
    node = next(n for n in doc["ros2_nodes"] if n["name"] == "mcp_server")
    cfg = McpServerConfig.model_validate(yaml.safe_load(node["config"]))
    # same obstacles as the defaults (compared at 0.1 mm: the defaults are computed from the swerve constants)
    assert BodyModel.from_settings(cfg.body).describe() == BodyModel.from_settings(McpServerConfig().body).describe()
    # The robot keeps the reach envelope on disk; every other body_check value is the default.
    assert cfg.body_check.reach_cache_file == Path("/var/lib/ros2/arm/reach_envelope.json")
    assert cfg.body_check.model_copy(update={"reach_cache_file": None}) == BodyCheckSettings()
