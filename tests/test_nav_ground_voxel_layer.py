"""Static invariants of the ground voxel layer and the ground cost filter in the Nav2 local costmap (D8, D13, D15).

The ground estimator (nodes/ground_estimator_cpp) publishes plane-relative obstacle and drop points; the local costmap
marks them in `ground_voxel_layer` next to the fixed-band `tof_voxel_layer`, and `ground_cost_filter` costs the
costmap_zones ground warning mask. The collision monitor and CliffStop keep the fixed ToF bands. No ROS is needed: the
YAML is read directly.
"""

import ast
import re
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
NAV2_PARAMS = REPO_ROOT / "nodes" / "nav2_bringup" / "config" / "nav2_params.yaml"
NAV2_README = REPO_ROOT / "nodes" / "nav2_bringup" / "README.md"
TESTS_README = REPO_ROOT / "tests" / "README.md"
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
GROUND_CONFIG = REPO_ROOT / "nodes" / "ground_estimator_cpp" / "config" / "ground_estimator.yaml"
COSTMAP_ZONES_CONFIG = REPO_ROOT / "nodes" / "costmap_zones" / "costmap_zones" / "config.py"

STVL_PLUGIN = "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
LAYER = "ground_voxel_layer"
TOF_LAYER = "tof_voxel_layer"
FILTER = "ground_cost_filter"
LOCAL_PLUGINS = [
    "obstacle_layer",
    "tof_voxel_layer",
    "ground_voxel_layer",
    "keepout_filter",
    "ground_cost_filter",
    "inflation_layer",
]
GLOBAL_PLUGINS = ["static_layer", "obstacle_layer", "keepout_filter", "inflation_layer"]
GROUND_SOURCES = ["ground_obstacle_mark", "ground_drop_mark", "ground_clear"]
INSCRIBED_COST = 253
LETHAL_COST = 254
README_HEADING = "## Ground layer (plane-relative ToF marks)"
PARKING_PHRASES = ("to tune", "starting value", "later", "follow-up", "todo", "phase 2", "out of scope")
LAYER_KEYS = (
    "plugin",
    "enabled",
    "voxel_decay",
    "decay_model",
    "voxel_size",
    "track_unknown_space",
    "mark_threshold",
    "update_footprint_enabled",
    "combination_method",
    "publish_voxel_map",
    "transform_tolerance",
    "mapping_mode",
)
MARK_COMMON = {
    "data_type": "PointCloud2",
    "marking": True,
    "clearing": False,
    "min_obstacle_height": -1.5,
    "max_obstacle_height": 1.5,
    "obstacle_range": 2.0,
    "filter": "voxel",
    "voxel_min_points": 0,
    "observation_persistence": 0.0,
    "clear_after_reading": True,
    "expected_update_rate": 0.0,
    "inf_is_valid": False,
}
CLEAR_KEYS = (
    "data_type",
    "topic",
    "marking",
    "clearing",
    "model_type",
    "min_z",
    "max_z",
    "horizontal_fov_angle",
    "vertical_fov_angle",
    "decay_acceleration",
)


def nav2() -> dict[str, Any]:
    """Load the repo Nav2 params.

    Returns:
        dict[str, Any]: Parsed nav2_params.yaml.
    """
    return yaml.safe_load(NAV2_PARAMS.read_text())


def costmap_params(costmap: str) -> dict[str, Any]:
    """ros__parameters of one costmap.

    Args:
        costmap (str): `global_costmap` or `local_costmap`.

    Returns:
        dict[str, Any]: The costmap parameters.
    """
    return nav2()[costmap][costmap]["ros__parameters"]


def ground_layer() -> dict[str, Any]:
    """The ground_voxel_layer block of the local costmap.

    Returns:
        dict[str, Any]: Layer parameters.
    """
    return costmap_params("local_costmap")[LAYER]


def block_text(key: str) -> str:
    """Raw text of a local costmap block in nav2_params.yaml (to check comments).

    Args:
        key (str): Block key such as `ground_voxel_layer`.

    Returns:
        str: Lines from the key up to the next sibling key of the same indent.
    """
    lines = NAV2_PARAMS.read_text().splitlines()
    start = next(i for i, line in enumerate(lines) if line.strip() == f"{key}:")
    indent = len(lines[start]) - len(lines[start].lstrip())
    end = start + 1
    while end < len(lines):
        line = lines[end]
        if line.strip() and not line.strip().startswith("#") and len(line) - len(line.lstrip()) <= indent:
            break
        end += 1
    return "\n".join(lines[start:end])


def ground_output_namespace() -> str:
    """Return the ground_estimator_cpp `output_namespace` and check client.yml and the package config agree.

    Returns:
        str: The namespace, e.g. `/ground`.
    """
    package = yaml.safe_load(GROUND_CONFIG.read_text())["output_namespace"]
    client = yaml.safe_load(CLIENT_VARS.read_text())
    entry = next(n for n in client["ros2_nodes"] if n["name"] == "ground_estimator_cpp")
    assert yaml.safe_load(entry["config"])["output_namespace"] == package
    return package


def readme_section(text: str, heading: str) -> str:
    """Return a README section body.

    Args:
        text (str): README text.
        heading (str): Section heading line.

    Returns:
        str: The section body (subsections included).
    """
    assert heading in text, heading
    return text.split(heading, 1)[1].split("\n## ", 1)[0]


def test_local_plugin_order_puts_ground_cost_filter_between_keepout_and_inflation() -> None:
    assert costmap_params("local_costmap")["plugins"] == LOCAL_PLUGINS


def test_ground_cost_filter_is_a_keepout_filter_on_the_ground_cost_info_topic() -> None:
    block = costmap_params("local_costmap")[FILTER]
    assert block["plugin"] == "nav2_costmap_2d::KeepoutFilter"
    assert block["enabled"] is True
    assert block["filter_info_topic"] == "/costmap_filter_info/ground_cost"
    assert block["transform_tolerance"] == 0.2
    assert block["override_lethal_cost"] is False
    assert FILTER not in costmap_params("local_costmap").get("filters", [])


def test_ground_cost_filter_info_topic_equals_the_costmap_zones_default() -> None:
    default = re.search(
        r'ground_cost_info_topic:\s*str\s*=\s*Field\(default="([^"]+)"', COSTMAP_ZONES_CONFIG.read_text()
    )
    assert default is not None
    assert costmap_params("local_costmap")[FILTER]["filter_info_topic"] == default.group(1)


def test_ground_warning_mask_cost_stays_below_inscribed() -> None:
    text = COSTMAP_ZONES_CONFIG.read_text()
    match = re.search(r"^DEFAULT_GROUND_WARNING_MASK_VALUE\s*=\s*(\d+)", text, re.MULTILINE)
    assert match is not None
    cost = int(match.group(1)) * LETHAL_COST // 100
    assert 0 < cost < INSCRIBED_COST


def test_local_costmap_keeps_exactly_one_speed_filter() -> None:
    params = costmap_params("local_costmap")
    assert params["filters"] == ["speed_filter"]
    assert params["speed_filter"]["plugin"] == "nav2_costmap_2d::SpeedFilter"
    assert params["speed_filter"]["filter_info_topic"] == "/costmap_filter_info/speed"
    speed_filters = [
        k for k, v in params.items() if isinstance(v, dict) and v.get("plugin", "").endswith("SpeedFilter")
    ]
    assert speed_filters == ["speed_filter"]


def test_ground_layer_settings_equal_the_tof_layer_settings() -> None:
    local = costmap_params("local_costmap")
    ground = local[LAYER]
    assert ground["plugin"] == STVL_PLUGIN
    # Off by default since 2026-10-22: live, its 15 s decay piled up the estimator's flickering lethal points into
    # dozens of false lethal cells that left Nav2 no path; back on once the live ground acceptance passes.
    assert ground["enabled"] is False
    for key in (k for k in LAYER_KEYS if k != "enabled"):
        assert type(ground[key]) is type(local[TOF_LAYER][key]), key
        assert ground[key] == local[TOF_LAYER][key], key
    assert ground["voxel_decay"] == 15.0 and ground["voxel_size"] == 0.05 and ground["combination_method"] == 1


def test_ground_layer_sources_topics_and_pass_through_limits() -> None:
    layer = ground_layer()
    assert layer["observation_sources"].split() == GROUND_SOURCES
    namespace = ground_output_namespace()
    for name, suffix in (("ground_obstacle_mark", "/obstacle_points"), ("ground_drop_mark", "/drop_points")):
        source = layer[name]
        assert source["topic"] == f"{namespace}{suffix}", name
        for key, value in MARK_COMMON.items():
            assert type(source[key]) is type(value), f"{name}.{key}"
            assert source[key] == value, f"{name}.{key}"


def test_ground_clear_equals_the_tof_clear_frustum() -> None:
    local = costmap_params("local_costmap")
    clear = local[LAYER]["ground_clear"]
    assert clear["topic"] == "/tof/points_filtered"
    for key in CLEAR_KEYS:
        assert clear[key] == local[TOF_LAYER]["tof_clear"][key], key


def test_global_costmap_is_untouched() -> None:
    params = costmap_params("global_costmap")
    assert params["plugins"] == GLOBAL_PLUGINS
    text = yaml.dump(params)
    assert "ground" not in text
    assert STVL_PLUGIN not in text


def test_collision_monitor_sources_are_unchanged_and_free_of_ground_topics() -> None:
    monitor = nav2()["collision_monitor"]["ros__parameters"]
    assert monitor["observation_sources"] == ["scan", "tof_obstacles", "tof_cliff"]
    for name in monitor["observation_sources"]:
        assert not monitor[name]["topic"].startswith("/ground/"), name
    assert monitor["CliffStop"]["sources_names"] == ["tof_cliff"]
    assert "/ground/" not in yaml.dump(monitor)


def test_tof_voxel_layer_is_pinned() -> None:
    layer = costmap_params("local_costmap")[TOF_LAYER]
    assert layer["observation_sources"] == "tof_mark tof_cliff_mark tof_clear"
    assert (layer["tof_mark"]["min_obstacle_height"], layer["tof_mark"]["max_obstacle_height"]) == (0.10, 0.6)
    assert (layer["tof_cliff_mark"]["min_obstacle_height"], layer["tof_cliff_mark"]["max_obstacle_height"]) == (
        -1.0,
        -0.10,
    )


def test_ground_values_are_commented_with_their_reasons() -> None:
    text = block_text(LAYER)
    for key in ("enabled", "min_obstacle_height", "max_obstacle_height", "obstacle_range", "clear_after_reading"):
        lines = text.splitlines()
        indexes = [i for i, line in enumerate(lines) if line.strip().startswith(f"{key}:")]
        assert indexes, key
        for index in indexes:
            assert lines[index - 1].strip().startswith("#") or "#" in lines[index], f"{key} has no comment"
    assert "pass-through" in text.lower()
    filter_text = block_text(FILTER)
    assert "override_lethal_cost" in filter_text and "#" in filter_text
    header = NAV2_PARAMS.read_text()
    assert "ros2 param set /local_costmap/local_costmap ground_voxel_layer.enabled false" in header
    assert "alongside the fixed bands" in header
    assert "D-A1" in header


def test_readme_documents_the_ground_layer() -> None:
    section = readme_section(NAV2_README.read_text(), README_HEADING)
    for phrase in (
        "/ground/obstacle_points",
        "/ground/drop_points",
        "pass-through",
        "-1.5",
        "1.5",
        "collision monitor",
        "CliffStop",
        "one third of the wheel diameter",
        "max slope",
        "ground_cost_filter",
        "speed_limit",
        "ros2 param set /local_costmap/local_costmap ground_voxel_layer.enabled false",
        "Switch-over criteria",
        "within 1 cm",
        "3 cm sill",
        "5 cm sill",
        "10 cm drop",
        "5 min",
        "CPU",
        "tof_mark",
        "tof_cliff_mark",
    ):
        assert phrase in section, phrase


def test_readme_ground_section_has_no_parking_wording() -> None:
    section = readme_section(NAV2_README.read_text(), README_HEADING).lower()
    found = [phrase for phrase in PARKING_PHRASES if phrase in section]
    assert not found, found


def test_tests_readme_documents_this_file() -> None:
    text = TESTS_README.read_text()
    assert "### test_nav_ground_voxel_layer.py" in text
    section = text.split("### test_nav_ground_voxel_layer.py", 1)[1].split("\n### ", 1)[0]
    tree = ast.parse(Path(__file__).read_text())
    names = [n.name for n in tree.body if isinstance(n, ast.FunctionDef) and n.name.startswith("test_")]
    missing = [name for name in names if f"`{name}`" not in section]
    assert not missing, f"undocumented in tests/README.md: {missing}"
