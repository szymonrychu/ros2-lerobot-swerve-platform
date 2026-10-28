"""Static invariants of the D435 spatio-temporal voxel layer (STVL) in the Nav2 local costmap (design D6).

The head D435 cloud (/head/d435/points_filtered: d435_obstacle_cloud point_cloud_xyz -> laser_filter self filter) marks
3D obstacles and below-floor cliff points in a `depth_voxel_layer` of the local costmap only; the lidar `obstacle_layer`
keeps its /scan_filtered source in both costmaps (U5). No ROS is available on the dev machine, so the YAML is read
directly.

Re-derived for the D435 (the ToF values are no longer accurate): the clearing frustum starts at the cloud min_depth
0.15 m (the D435 min-Z at 640x360) and lies inside the D435 depth FOV of 87 x 58 deg minus the datasheet tolerance
(+-3 deg horizontal, +-1 deg vertical) instead of the ToF 70 deg diagonal; the frustum FOV is a datasheet number, not a
derivation from a diagonal, so the ToF diagonal-to-4:3 conversion test has no D435 counterpart. The mark ranges stay
2 m, inside the cloud's 3 m max_depth. The height bands are kept (10 cm over and under the floor, 0.6 m robot height):
the D435 depth RMS error at the 2 m range is under 2 % (about 4 cm), so the floor stays outside both bands.
"""

import ast
import math
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
NAV2_PARAMS = REPO_ROOT / "nodes" / "nav2_bringup" / "config" / "nav2_params.yaml"
NAV2_README = REPO_ROOT / "nodes" / "nav2_bringup" / "README.md"
TESTS_README = REPO_ROOT / "tests" / "README.md"
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
CLOUD_PARAMS = REPO_ROOT / "nodes" / "d435_obstacle_cloud" / "config" / "point_cloud_xyz.yaml"
CLOUD_NODE = "d435_obstacle_cloud"
LASER_FILTER_PARAMS = REPO_ROOT / "nodes" / "laser_filter" / "config" / "footprint_filter.yaml"

STVL_PLUGIN = "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
STVL_APT_PACKAGE = "ros-jazzy-spatio-temporal-voxel-layer"
LAYER = "depth_voxel_layer"
GROUND_LAYER = "ground_voxel_layer"
D435_TOPIC = "/head/d435/points_filtered"
FILTERED_SCAN = "/scan_filtered"
LOCAL_PLUGINS = [
    "obstacle_layer",
    "depth_voxel_layer",
    "ground_voxel_layer",
    "keepout_filter",
    "ground_cost_filter",
    "inflation_layer",
]
GLOBAL_PLUGINS = ["static_layer", "obstacle_layer", "keepout_filter", "inflation_layer"]
MARK_SOURCES = ("depth_mark", "depth_cliff_mark")
CLEAR_SOURCE = "depth_clear"
README_HEADING = "## D435 obstacle layer"
PARKING_PHRASES = ("to tune", "starting value", "later", "follow-up", "todo", "phase 2", "out of scope")
# Robot height with the head (design D3): anything higher is an overhang the robot passes under.
ROBOT_HEIGHT_M = 0.6
# Floor z = 0 in odom; a cliff band must start clearly below the D435 depth noise around the floor.
CLIFF_MAX_HEIGHT_M = -0.05
# D435 depth FOV (Intel datasheet, design D5: full 87 x 58 deg at 640x360) and its unit-to-unit tolerance.
D435_HFOV_DEG = 87.0
D435_VFOV_DEG = 58.0
D435_HFOV_TOLERANCE_DEG = 3.0
D435_VFOV_TOLERANCE_DEG = 1.0

EXPECTED_LAYER = {
    "plugin": STVL_PLUGIN,
    "enabled": True,
    "voxel_decay": 15.0,
    "decay_model": 0,
    "voxel_size": 0.05,
    "track_unknown_space": False,
    "mark_threshold": 0,
    "update_footprint_enabled": False,
    "combination_method": 1,
    "publish_voxel_map": False,
    "transform_tolerance": 0.3,
    "mapping_mode": False,
    "observation_sources": "depth_mark depth_cliff_mark depth_clear",
}
MARK_COMMON = {
    "data_type": "PointCloud2",
    "topic": D435_TOPIC,
    "marking": True,
    "clearing": False,
    "obstacle_range": 2.0,
    "filter": "voxel",
    "voxel_min_points": 0,
    "observation_persistence": 0.0,
    "clear_after_reading": True,
    "expected_update_rate": 0.0,
    "inf_is_valid": False,
}
EXPECTED_SOURCES = {
    "depth_mark": {**MARK_COMMON, "min_obstacle_height": 0.10, "max_obstacle_height": 0.6},
    "depth_cliff_mark": {**MARK_COMMON, "min_obstacle_height": -1.0, "max_obstacle_height": -0.10},
    "depth_clear": {
        "data_type": "PointCloud2",
        "topic": D435_TOPIC,
        "marking": False,
        "clearing": True,
        "model_type": 0,
        "min_z": 0.15,
        "max_z": 3.0,
        "horizontal_fov_angle": 1.45,
        "vertical_fov_angle": 0.98,
        "decay_acceleration": 1.0,
    },
}


def nav2() -> dict:
    """Load the repo Nav2 params.

    Returns:
        dict: Parsed nav2_params.yaml.
    """
    return yaml.safe_load(NAV2_PARAMS.read_text())


def costmap_params(costmap: str) -> dict:
    """ros__parameters of one costmap.

    Args:
        costmap: `global_costmap` or `local_costmap`.

    Returns:
        dict: The costmap parameters.
    """
    return nav2()[costmap][costmap]["ros__parameters"]


def depth_layer() -> dict:
    """The depth_voxel_layer block of the local costmap.

    Returns:
        dict: Layer parameters.
    """
    return costmap_params("local_costmap")[LAYER]


def d435_min_fov_rad() -> tuple[float, float]:
    """Smallest horizontal and vertical D435 depth FOV a unit can have (datasheet value minus its tolerance).

    Returns:
        tuple[float, float]: (horizontal, vertical) FOV in radians.
    """
    horizontal = math.radians(D435_HFOV_DEG - D435_HFOV_TOLERANCE_DEG)
    vertical = math.radians(D435_VFOV_DEG - D435_VFOV_TOLERANCE_DEG)
    return horizontal, vertical


def cloud_max_depth() -> float:
    """max_depth of the d435_obstacle_cloud point_cloud_xyz params.

    Returns:
        float: Farthest point the cloud contains (m).
    """
    return yaml.safe_load(CLOUD_PARAMS.read_text())[CLOUD_NODE]["ros__parameters"]["max_depth"]


def lines_before(key: str, text: str) -> list[str]:
    """Return the lines from the `<key>:` line inside the depth_voxel_layer block back to the previous non-comment line.

    Args:
        key: Parameter name.
        text: The depth_voxel_layer block text.

    Returns:
        list[str]: The key line plus the comment lines directly above it.
    """
    lines = text.splitlines()
    index = next(i for i, line in enumerate(lines) if line.strip().startswith(f"{key}:"))
    block = [lines[index]]
    for line in reversed(lines[:index]):
        if not line.strip().startswith("#"):
            break
        block.append(line)
    return block


def layer_text() -> str:
    """Raw text of the depth_voxel_layer block in nav2_params.yaml (to check comments).

    Returns:
        str: Lines from the `depth_voxel_layer:` key up to the next sibling key of the same indent.
    """
    lines = NAV2_PARAMS.read_text().splitlines()
    start = next(i for i, line in enumerate(lines) if line.strip() == f"{LAYER}:")
    indent = len(lines[start]) - len(lines[start].lstrip())
    end = start + 1
    while end < len(lines):
        line = lines[end]
        stripped = line.strip()
        if stripped and not stripped.startswith("#") and len(line) - len(line.lstrip()) <= indent:
            break
        end += 1
    # Comments directly above the key belong to the layer too.
    head = start
    while head > 0 and lines[head - 1].strip().startswith("#"):
        head -= 1
    return "\n".join(lines[head:end])


def test_local_costmap_has_the_depth_layer_after_the_lidar_layer() -> None:
    assert costmap_params("local_costmap")["plugins"] == LOCAL_PLUGINS


@pytest.mark.parametrize(("key", "value"), sorted(EXPECTED_LAYER.items()))
def test_depth_voxel_layer_settings(key: str, value: object) -> None:
    actual = depth_layer()[key]
    assert type(actual) is type(value), key
    assert actual == value


def test_stvl_footprint_clearing_is_disabled() -> None:
    """STVL clears the footprint with setConvexPolygonCost (convex fill like the ObstacleLayer) and would erase
    obstacles in the H notches."""
    assert depth_layer()["update_footprint_enabled"] is False
    assert "setConvexPolygonCost" in "\n".join(lines_before("update_footprint_enabled", layer_text()))


@pytest.mark.parametrize("source", sorted(EXPECTED_SOURCES))
def test_depth_sources_settings(source: str) -> None:
    actual = depth_layer()[source]
    for key, value in EXPECTED_SOURCES[source].items():
        assert type(actual[key]) is type(value), f"{source}.{key}"
        assert actual[key] == value, f"{source}.{key}"


def test_mark_and_clear_sources_are_split() -> None:
    layer = depth_layer()
    assert layer["observation_sources"].split() == [*MARK_SOURCES, CLEAR_SOURCE]
    for name in MARK_SOURCES:
        assert layer[name]["marking"] is True and layer[name]["clearing"] is False
    assert layer[CLEAR_SOURCE]["marking"] is False and layer[CLEAR_SOURCE]["clearing"] is True
    # model_type 0: depth camera frustum (1 would be a 3D lidar).
    assert layer[CLEAR_SOURCE]["model_type"] == 0


def test_obstacle_band_is_above_the_floor_and_below_the_robot() -> None:
    mark = depth_layer()["depth_mark"]
    assert 0.0 < mark["min_obstacle_height"] < mark["max_obstacle_height"] <= ROBOT_HEIGHT_M


def test_mcp_obstacle_band_equals_depth_mark() -> None:
    perception = REPO_ROOT / "nodes" / "mcp_server" / "mcp_server" / "perception.py"
    band = None
    for node in ast.parse(perception.read_text()).body:
        names = [t.id for t in node.targets if isinstance(t, ast.Name)] if isinstance(node, ast.Assign) else []
        if "TOF_OBSTACLE_BAND_M" in names:
            band = ast.literal_eval(node.value)
    assert band is not None, "TOF_OBSTACLE_BAND_M not found in perception.py"
    mark = depth_layer()["depth_mark"]
    assert tuple(band) == (mark["min_obstacle_height"], mark["max_obstacle_height"])


def test_cliff_band_is_strictly_below_the_floor_noise() -> None:
    cliff = depth_layer()["depth_cliff_mark"]
    assert cliff["min_obstacle_height"] < cliff["max_obstacle_height"] < CLIFF_MAX_HEIGHT_M


def test_bands_do_not_overlap() -> None:
    layer = depth_layer()
    assert layer["depth_cliff_mark"]["max_obstacle_height"] < layer["depth_mark"]["min_obstacle_height"]


def test_ranges_stay_inside_the_cloud_max_depth() -> None:
    layer = depth_layer()
    max_depth = cloud_max_depth()
    assert layer[CLEAR_SOURCE]["max_z"] <= max_depth
    for name in MARK_SOURCES:
        assert layer[name]["obstacle_range"] <= max_depth
    # The clearing frustum starts where the cloud starts (point_cloud_xyz min_depth).
    min_depth = yaml.safe_load(CLOUD_PARAMS.read_text())[CLOUD_NODE]["ros__parameters"]["min_depth"]
    assert layer[CLEAR_SOURCE]["min_z"] >= min_depth


def test_clearing_fov_is_inside_the_d435_fov() -> None:
    """Inside the smallest FOV a D435 unit has (87 - 3 x 58 - 1 deg), so clearing never reaches outside the view."""
    horizontal, vertical = d435_min_fov_rad()
    clear = depth_layer()[CLEAR_SOURCE]
    assert 0.0 < clear["horizontal_fov_angle"] < horizontal
    assert 0.0 < clear["vertical_fov_angle"] < vertical


def test_depth_layer_reads_the_d435_laser_filter_output() -> None:
    """laser_filter self-filters the D435 cloud (laser-filter-on-the-d435-clo); every source of the enabled layer reads
    its output."""
    laser_filter = yaml.safe_load(LASER_FILTER_PARAMS.read_text())
    assert laser_filter["cloud_filtered_topic"] == D435_TOPIC
    layer = depth_layer()
    assert {layer[source]["topic"] for source in (*MARK_SOURCES, CLEAR_SOURCE)} == {D435_TOPIC}
    assert layer["enabled"] is True


@pytest.mark.parametrize("costmap", ["global_costmap", "local_costmap"])
def test_lidar_obstacle_layer_keeps_the_scan_source(costmap: str) -> None:
    params = costmap_params(costmap)
    assert "obstacle_layer" in params["plugins"]
    obstacle = params["obstacle_layer"]
    assert obstacle["plugin"] == "nav2_costmap_2d::ObstacleLayer"
    assert obstacle["enabled"] is True
    assert obstacle["observation_sources"].split() == ["scan"]
    scan = obstacle["scan"]
    assert scan["topic"] == FILTERED_SCAN
    assert scan["data_type"] == "LaserScan"
    assert scan["marking"] is True and scan["clearing"] is True


def test_no_stvl_source_uses_the_lidar() -> None:
    """STVL is not meant for planar lidars; the lidar stays on the ObstacleLayer."""
    layer = depth_layer()
    for name in layer["observation_sources"].split():
        assert layer[name]["topic"] != FILTERED_SCAN
        assert layer[name]["data_type"] != "LaserScan"
        assert "scan" not in layer[name]["topic"]


def test_global_costmap_has_no_depth_voxels() -> None:
    """Depth voxels live only in the local costmap (user decision 2026-10-18, Q3)."""
    params = costmap_params("global_costmap")
    assert params["plugins"] == GLOBAL_PLUGINS
    assert LAYER not in params
    for key, value in params.items():
        if isinstance(value, dict):
            assert value.get("plugin") != STVL_PLUGIN, key
    assert D435_TOPIC not in yaml.safe_dump(params)


def test_only_one_stvl_layer_in_the_local_costmap() -> None:
    params = costmap_params("local_costmap")
    stvl = [key for key, value in params.items() if isinstance(value, dict) and value.get("plugin") == STVL_PLUGIN]
    assert stvl == [LAYER, GROUND_LAYER]


@pytest.mark.parametrize(
    ("key", "reason"),
    [
        ("voxel_decay", "decay"),
        ("update_footprint_enabled", "notch"),
        ("combination_method", "max"),
        ("transform_tolerance", "TF"),
        ("min_obstacle_height", "floor"),
        ("max_obstacle_height", "robot height"),
        ("obstacle_range", "max_depth"),
        ("horizontal_fov_angle", "87 deg"),
        ("vertical_fov_angle", "58 deg"),
        ("min_z", "min-Z"),
        ("expected_update_rate", "collision monitor"),
    ],
)
def test_layer_values_are_commented_with_their_reason(key: str, reason: str) -> None:
    text = "\n".join(lines_before(key, layer_text()))
    assert "#" in text, f"{key} has no comment"
    assert reason.lower() in text.lower(), f"{key} comment lacks '{reason}'"


def test_nav2_bringup_installs_stvl() -> None:
    client = yaml.safe_load(CLIENT_VARS.read_text())
    assert STVL_APT_PACKAGE in client["ros2_node_type_defaults"]["nav2_bringup"]["apt_packages"]
    lines = CLIENT_VARS.read_text().splitlines()
    index = next(i for i, line in enumerate(lines) if line.strip() == f"- {STVL_APT_PACKAGE}")
    assert lines[index - 1].strip().startswith("#"), "the STVL apt package has no comment"


def test_readme_documents_the_depth_obstacle_layer() -> None:
    text = NAV2_README.read_text()
    assert README_HEADING in text
    section = text.split(README_HEADING, 1)[1].split("\n## ", 1)[0]
    for phrase in (
        D435_TOPIC,
        "d435_obstacle_cloud",
        "point_cloud_xyz",
        "laser_filter",
        LAYER,
        "voxel_decay",
        "update_footprint_enabled",
        "lidar raytracing never clears depth voxels",
        "only decays depth voxels",
        "87 x 58 deg",
        "Local costmap only",
        "void",
    ):
        assert phrase in section, phrase
    for stale in ("/tof", "tof_obstacle_cloud", "tof_voxel_layer", "arducam"):
        assert stale not in section, stale


def test_readme_depth_section_has_no_parking_wording() -> None:
    section = NAV2_README.read_text().split(README_HEADING, 1)[1].split("\n## ", 1)[0].lower()
    found = [phrase for phrase in PARKING_PHRASES if phrase in section]
    assert not found, f"parking wording in the D435 obstacle layer section: {found}"


def test_tests_readme_documents_this_file() -> None:
    text = TESTS_README.read_text()
    assert "### test_nav_depth_voxel_layer.py" in text
    section = text.split("### test_nav_depth_voxel_layer.py", 1)[1].split("\n### ", 1)[0]
    tree = ast.parse(Path(__file__).read_text())
    names = [n.name for n in tree.body if isinstance(n, ast.FunctionDef) and n.name.startswith("test_")]
    missing = [name for name in names if f"`{name}`" not in section]
    assert not missing, f"undocumented in tests/README.md: {missing}"
