"""Static invariants of the ToF spatio-temporal voxel layer (STVL) in the Nav2 local costmap.

The head ToF cloud (/tof/points_filtered: point_cloud_xyz -> laser_filter self filter) marks 3D obstacles and
below-floor cliff points in a `tof_voxel_layer` of the local costmap only; the lidar `obstacle_layer` keeps its
/scan_filtered source in both costmaps (U5). No ROS is available on the dev machine, so the YAML is read directly.
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
CLOUD_PARAMS = REPO_ROOT / "nodes" / "tof_obstacle_cloud" / "config" / "point_cloud_xyz.yaml"
LASER_FILTER_PARAMS = REPO_ROOT / "nodes" / "laser_filter" / "config" / "footprint_filter.yaml"

STVL_PLUGIN = "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
STVL_APT_PACKAGE = "ros-jazzy-spatio-temporal-voxel-layer"
LAYER = "tof_voxel_layer"
GROUND_LAYER = "ground_voxel_layer"
TOF_TOPIC = "/tof/points_filtered"
FILTERED_SCAN = "/scan_filtered"
LOCAL_PLUGINS = [
    "obstacle_layer",
    "tof_voxel_layer",
    "ground_voxel_layer",
    "keepout_filter",
    "ground_cost_filter",
    "inflation_layer",
]
GLOBAL_PLUGINS = ["static_layer", "obstacle_layer", "keepout_filter", "inflation_layer"]
MARK_SOURCES = ("tof_mark", "tof_cliff_mark")
CLEAR_SOURCE = "tof_clear"
PARKING_PHRASES = ("to tune", "starting value", "later", "follow-up", "todo", "phase 2", "out of scope")
# Robot height with the head (design D3): anything higher is an overhang the robot passes under.
ROBOT_HEIGHT_M = 0.6
# Floor z = 0 in odom; a cliff band must start clearly below the ToF depth noise around the floor.
CLIFF_MAX_HEIGHT_M = -0.05
# Arducam ToF spec: 70 deg diagonal FOV on a 240 x 180 (4:3) sensor.
TOF_DIAGONAL_FOV_DEG = 70.0
TOF_WIDTH_PX = 240
TOF_HEIGHT_PX = 180

EXPECTED_LAYER = {
    "plugin": STVL_PLUGIN,
    "enabled": False,
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
    "observation_sources": "tof_mark tof_cliff_mark tof_clear",
}
MARK_COMMON = {
    "data_type": "PointCloud2",
    "topic": TOF_TOPIC,
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
    "tof_mark": {**MARK_COMMON, "min_obstacle_height": 0.10, "max_obstacle_height": 0.6},
    "tof_cliff_mark": {**MARK_COMMON, "min_obstacle_height": -1.0, "max_obstacle_height": -0.10},
    "tof_clear": {
        "data_type": "PointCloud2",
        "topic": TOF_TOPIC,
        "marking": False,
        "clearing": True,
        "model_type": 0,
        "min_z": 0.2,
        "max_z": 3.0,
        "horizontal_fov_angle": 1.0,
        "vertical_fov_angle": 0.78,
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


def tof_layer() -> dict:
    """The tof_voxel_layer block of the local costmap.

    Returns:
        dict: Layer parameters.
    """
    return costmap_params("local_costmap")[LAYER]


def tof_fov_rad() -> tuple[float, float]:
    """Horizontal and vertical ToF FOV from the diagonal FOV and the sensor aspect (pinhole model).

    Returns:
        tuple[float, float]: (horizontal, vertical) FOV in radians.
    """
    half_diag = math.tan(math.radians(TOF_DIAGONAL_FOV_DEG) / 2.0)
    diag_px = math.hypot(TOF_WIDTH_PX, TOF_HEIGHT_PX)
    horizontal = 2.0 * math.atan(half_diag * TOF_WIDTH_PX / diag_px)
    vertical = 2.0 * math.atan(half_diag * TOF_HEIGHT_PX / diag_px)
    return horizontal, vertical


def cloud_max_depth() -> float:
    """max_depth of the tof_obstacle_cloud point_cloud_xyz params.

    Returns:
        float: Farthest point the cloud contains (m).
    """
    return yaml.safe_load(CLOUD_PARAMS.read_text())["tof_obstacle_cloud"]["ros__parameters"]["max_depth"]


def lines_before(key: str, text: str) -> list[str]:
    """Return the lines from the `<key>:` line inside the tof_voxel_layer block back to the previous non-comment line.

    Args:
        key: Parameter name.
        text: The tof_voxel_layer block text.

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
    """Raw text of the tof_voxel_layer block in nav2_params.yaml (to check comments).

    Returns:
        str: Lines from the `tof_voxel_layer:` key up to the next sibling key of the same indent.
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


def test_local_costmap_has_the_tof_layer_after_the_lidar_layer() -> None:
    assert costmap_params("local_costmap")["plugins"] == LOCAL_PLUGINS


@pytest.mark.parametrize(("key", "value"), sorted(EXPECTED_LAYER.items()))
def test_tof_voxel_layer_settings(key: str, value: object) -> None:
    actual = tof_layer()[key]
    assert type(actual) is type(value), key
    assert actual == value


def test_stvl_footprint_clearing_is_disabled() -> None:
    """STVL clears the footprint with setConvexPolygonCost (convex fill like the ObstacleLayer) and would erase
    obstacles in the H notches."""
    assert tof_layer()["update_footprint_enabled"] is False
    assert "setConvexPolygonCost" in "\n".join(lines_before("update_footprint_enabled", layer_text()))


@pytest.mark.parametrize("source", sorted(EXPECTED_SOURCES))
def test_tof_sources_settings(source: str) -> None:
    actual = tof_layer()[source]
    for key, value in EXPECTED_SOURCES[source].items():
        assert type(actual[key]) is type(value), f"{source}.{key}"
        assert actual[key] == value, f"{source}.{key}"


def test_mark_and_clear_sources_are_split() -> None:
    layer = tof_layer()
    assert layer["observation_sources"].split() == [*MARK_SOURCES, CLEAR_SOURCE]
    for name in MARK_SOURCES:
        assert layer[name]["marking"] is True and layer[name]["clearing"] is False
    assert layer[CLEAR_SOURCE]["marking"] is False and layer[CLEAR_SOURCE]["clearing"] is True
    # model_type 0: depth camera frustum (1 would be a 3D lidar).
    assert layer[CLEAR_SOURCE]["model_type"] == 0


def test_obstacle_band_is_above_the_floor_and_below_the_robot() -> None:
    mark = tof_layer()["tof_mark"]
    assert 0.0 < mark["min_obstacle_height"] < mark["max_obstacle_height"] <= ROBOT_HEIGHT_M


def test_mcp_tof_obstacle_band_equals_tof_mark() -> None:
    perception = REPO_ROOT / "nodes" / "mcp_server" / "mcp_server" / "perception.py"
    band = None
    for node in ast.parse(perception.read_text()).body:
        names = [t.id for t in node.targets if isinstance(t, ast.Name)] if isinstance(node, ast.Assign) else []
        if "TOF_OBSTACLE_BAND_M" in names:
            band = ast.literal_eval(node.value)
    assert band is not None, "TOF_OBSTACLE_BAND_M not found in perception.py"
    mark = tof_layer()["tof_mark"]
    assert tuple(band) == (mark["min_obstacle_height"], mark["max_obstacle_height"])


def test_cliff_band_is_strictly_below_the_floor_noise() -> None:
    cliff = tof_layer()["tof_cliff_mark"]
    assert cliff["min_obstacle_height"] < cliff["max_obstacle_height"] < CLIFF_MAX_HEIGHT_M


def test_bands_do_not_overlap() -> None:
    layer = tof_layer()
    assert layer["tof_cliff_mark"]["max_obstacle_height"] < layer["tof_mark"]["min_obstacle_height"]


def test_ranges_stay_inside_the_cloud_max_depth() -> None:
    layer = tof_layer()
    max_depth = cloud_max_depth()
    assert layer[CLEAR_SOURCE]["max_z"] <= max_depth
    for name in MARK_SOURCES:
        assert layer[name]["obstacle_range"] <= max_depth
    # The clearing frustum starts where the cloud starts (point_cloud_xyz min_depth).
    min_depth = yaml.safe_load(CLOUD_PARAMS.read_text())["tof_obstacle_cloud"]["ros__parameters"]["min_depth"]
    assert layer[CLEAR_SOURCE]["min_z"] >= min_depth


def test_clearing_fov_is_inside_the_tof_fov() -> None:
    horizontal, vertical = tof_fov_rad()
    clear = tof_layer()[CLEAR_SOURCE]
    assert 0.0 < clear["horizontal_fov_angle"] < horizontal
    assert 0.0 < clear["vertical_fov_angle"] < vertical


def test_tof_fov_constants_match_the_design() -> None:
    """70 deg diagonal at 4:3 is 58.5 x 45.6 deg (design D3)."""
    horizontal, vertical = tof_fov_rad()
    assert math.degrees(horizontal) == pytest.approx(58.5, abs=0.1)
    assert math.degrees(vertical) == pytest.approx(45.6, abs=0.1)


def test_tof_topic_is_the_laser_filter_output() -> None:
    laser_filter = yaml.safe_load(LASER_FILTER_PARAMS.read_text())
    assert laser_filter["cloud_filtered_topic"] == TOF_TOPIC


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
    layer = tof_layer()
    for name in layer["observation_sources"].split():
        assert layer[name]["topic"] != FILTERED_SCAN
        assert layer[name]["data_type"] != "LaserScan"
        assert "scan" not in layer[name]["topic"]


def test_global_costmap_has_no_tof_voxels() -> None:
    """ToF voxels live only in the local costmap (user decision 2026-10-18, Q3)."""
    params = costmap_params("global_costmap")
    assert params["plugins"] == GLOBAL_PLUGINS
    assert LAYER not in params
    for key, value in params.items():
        if isinstance(value, dict):
            assert value.get("plugin") != STVL_PLUGIN, key
    assert TOF_TOPIC not in yaml.safe_dump(params)


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
        ("horizontal_fov_angle", "70 deg"),
        ("vertical_fov_angle", "70 deg"),
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


def test_readme_documents_the_tof_obstacle_layer() -> None:
    text = NAV2_README.read_text()
    assert "## ToF obstacle layer" in text
    section = text.split("## ToF obstacle layer", 1)[1].split("\n## ", 1)[0]
    for phrase in (
        TOF_TOPIC,
        "point_cloud_xyz",
        "laser_filter",
        LAYER,
        "voxel_decay",
        "update_footprint_enabled",
        "lidar raytracing never clears ToF voxels",
        "only decays ToF voxels",
        "Local costmap only",
        "void",
    ):
        assert phrase in section, phrase


def test_readme_tof_section_has_no_parking_wording() -> None:
    section = NAV2_README.read_text().split("## ToF obstacle layer", 1)[1].split("\n## ", 1)[0].lower()
    found = [phrase for phrase in PARKING_PHRASES if phrase in section]
    assert not found, f"parking wording in the ToF obstacle layer section: {found}"


def test_tests_readme_documents_this_file() -> None:
    text = TESTS_README.read_text()
    assert "### test_nav_tof_voxel_layer.py" in text
    section = text.split("### test_nav_tof_voxel_layer.py", 1)[1].split("\n### ", 1)[0]
    tree = ast.parse(Path(__file__).read_text())
    names = [n.name for n in tree.body if isinstance(n, ast.FunctionDef) and n.name.startswith("test_")]
    missing = [name for name in names if f"`{name}`" not in section]
    assert not missing, f"undocumented in tests/README.md: {missing}"
