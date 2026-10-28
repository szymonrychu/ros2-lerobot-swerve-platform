"""The web_ui 3D obstacles layer colours D435 points by the same height bands as the Nav2 depth sources.

The frontend constants (`OBSTACLE_MIN_Z`, `OBSTACLE_MAX_Z`, `CLIFF_MAX_Z` in
`nodes/web_ui/frontend/src/map3d/obstaclePoints.ts`) and `nodes/nav2_bringup/config/nav2_params.yaml` are maintained
separately; these tests read both files (the TypeScript constants by regex, no build) and fail when they drift.
"""

import re
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
NAV2_PARAMS = REPO_ROOT / "nodes" / "nav2_bringup" / "config" / "nav2_params.yaml"
OBSTACLE_POINTS_TS = REPO_ROOT / "nodes" / "web_ui" / "frontend" / "src" / "map3d" / "obstaclePoints.ts"
CONST_RE = re.compile(r"^export const ([A-Z_]+) = (-?\d+(?:\.\d+)?)$", re.MULTILINE)
BAND_CONSTANTS = ("OBSTACLE_MIN_Z", "OBSTACLE_MAX_Z", "CLIFF_MAX_Z")


def frontend_constants() -> dict[str, float]:
    """Numeric exported constants of obstaclePoints.ts.

    Returns:
        dict[str, float]: constant name to value, for every `export const NAME = <number>` line.
    """
    return {name: float(value) for name, value in CONST_RE.findall(OBSTACLE_POINTS_TS.read_text())}


def nav2_params() -> dict:
    """Parsed nav2_params.yaml.

    Returns:
        dict: the whole YAML document.
    """
    return yaml.safe_load(NAV2_PARAMS.read_text())


@pytest.fixture(scope="module")
def bands() -> dict[str, float]:
    """Frontend band constants.

    Returns:
        dict[str, float]: the three band constants by name.
    """
    consts = frontend_constants()
    missing = [name for name in BAND_CONSTANTS if name not in consts]
    assert not missing, f"{OBSTACLE_POINTS_TS} lacks numeric exports {missing}"
    return {name: consts[name] for name in BAND_CONSTANTS}


@pytest.fixture(scope="module")
def voxel_layer() -> dict:
    """The local costmap depth_voxel_layer parameters.

    Returns:
        dict: depth_voxel_layer of local_costmap.
    """
    return nav2_params()["local_costmap"]["local_costmap"]["ros__parameters"]["depth_voxel_layer"]


@pytest.fixture(scope="module")
def collision_monitor() -> dict:
    """The collision_monitor parameters.

    Returns:
        dict: collision_monitor ros__parameters.
    """
    return nav2_params()["collision_monitor"]["ros__parameters"]


def test_obstacle_band_equals_depth_mark(bands: dict[str, float], voxel_layer: dict) -> None:
    """OBSTACLE_MIN_Z / OBSTACLE_MAX_Z equal the depth_mark marking band."""
    mark = voxel_layer["depth_mark"]
    assert bands["OBSTACLE_MIN_Z"] == pytest.approx(mark["min_obstacle_height"])
    assert bands["OBSTACLE_MAX_Z"] == pytest.approx(mark["max_obstacle_height"])


def test_cliff_bound_equals_depth_cliff_mark(bands: dict[str, float], voxel_layer: dict) -> None:
    """CLIFF_MAX_Z equals the top of the depth_cliff_mark band."""
    assert bands["CLIFF_MAX_Z"] == pytest.approx(voxel_layer["depth_cliff_mark"]["max_obstacle_height"])


def test_obstacle_band_equals_collision_depth_obstacles(bands: dict[str, float], collision_monitor: dict) -> None:
    """OBSTACLE_MIN_Z / OBSTACLE_MAX_Z equal the collision_monitor depth_obstacles band."""
    source = collision_monitor["depth_obstacles"]
    assert bands["OBSTACLE_MIN_Z"] == pytest.approx(source["min_height"])
    assert bands["OBSTACLE_MAX_Z"] == pytest.approx(source["max_height"])


def test_cliff_bound_equals_collision_depth_cliff(bands: dict[str, float], collision_monitor: dict) -> None:
    """CLIFF_MAX_Z equals the top of the collision_monitor depth_cliff band."""
    assert bands["CLIFF_MAX_Z"] == pytest.approx(collision_monitor["depth_cliff"]["max_height"])
