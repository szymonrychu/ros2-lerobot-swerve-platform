"""Nav2 head inputs on the RealSense D435 (design D6): the lidar-only state of 2026-10-24 ends with the D435.

The head 3D input of the local costmap is the self-filtered D435 cloud (d435_obstacle_cloud point_cloud_xyz ->
laser_filter): `depth_voxel_layer` is enabled and every one of its sources reads `/head/d435/points_filtered`. The
collision monitor reads the same cloud: StopBox checks `scan` and `depth_obstacles`, CliffStop is enabled on
`depth_cliff`. `ground_voxel_layer` and `ground_cost_filter` are enabled on the ground estimator outputs. No line of
nav2_params.yaml names a `/tof` topic or keeps the lidar-only disable comment any more. No ROS is available on the dev
machine, so the YAML is read directly.
"""

import subprocess
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
NAV2_PARAMS = REPO_ROOT / "nodes" / "nav2_bringup" / "config" / "nav2_params.yaml"
NAV2_README = REPO_ROOT / "nodes" / "nav2_bringup" / "README.md"
LASER_FILTER_PARAMS = REPO_ROOT / "nodes" / "laser_filter" / "config" / "footprint_filter.yaml"
GROUND_DEFAULTS = REPO_ROOT / "nodes" / "ground_estimator_cpp" / "config" / "ground_estimator.yaml"
TESTS_README = REPO_ROOT / "tests" / "README.md"
LIDAR_ONLY_TEST = "tests/test_nav_lidar_only.py"
D435_TOPIC = "/head/d435/points_filtered"
LAYER = "depth_voxel_layer"
LIDAR_ONLY_COMMENT = "disabled (user 2026-10-24): lidar-only"
LIDAR_ONLY_MARKER = "disabled (user 2026-10-24)"
README_INPUTS_HEADING = "## D435 inputs"
DEPTH_COLLISION_SOURCES = ("depth_obstacles", "depth_cliff")


def local_costmap() -> dict[str, Any]:
    """ros__parameters of the local costmap in nav2_params.yaml.

    Returns:
        dict[str, Any]: The local costmap parameters.
    """
    return yaml.safe_load(NAV2_PARAMS.read_text())["local_costmap"]["local_costmap"]["ros__parameters"]


def collision_monitor() -> dict[str, Any]:
    """ros__parameters of the collision_monitor in nav2_params.yaml.

    Returns:
        dict[str, Any]: The collision_monitor parameters.
    """
    return yaml.safe_load(NAV2_PARAMS.read_text())["collision_monitor"]["ros__parameters"]


def layer_block_lines(name: str) -> list[str]:
    """Raw lines of one local costmap layer block (the `<name>:` key up to the next sibling key).

    Args:
        name: Layer key, e.g. `depth_voxel_layer`.

    Returns:
        list[str]: The block lines, the key line included.
    """
    lines = NAV2_PARAMS.read_text().splitlines()
    start = next(i for i, line in enumerate(lines) if line.strip() == f"{name}:")
    indent = len(lines[start]) - len(lines[start].lstrip())
    end = start + 1
    while end < len(lines):
        stripped = lines[end].strip()
        if stripped and not stripped.startswith("#") and len(lines[end]) - len(lines[end].lstrip()) <= indent:
            break
        end += 1
    return lines[start:end]


def test_depth_voxel_layer_reads_the_d435_cloud() -> None:
    params = local_costmap()
    assert LAYER in params["plugins"]
    layer = params[LAYER]
    assert layer["enabled"] is True
    sources = layer["observation_sources"].split()
    assert sources == ["depth_mark", "depth_cliff_mark", "depth_clear"]
    assert {layer[source]["topic"] for source in sources} == {D435_TOPIC}
    # The laser_filter cloud self filter publishes exactly this topic.
    assert yaml.safe_load(LASER_FILTER_PARAMS.read_text())["cloud_filtered_topic"] == D435_TOPIC
    assert not [line for line in layer_block_lines(LAYER) if LIDAR_ONLY_COMMENT in line]
    text = NAV2_PARAMS.read_text()
    assert "/tof" not in text, [line.strip() for line in text.splitlines() if "/tof" in line]
    assert "tof_voxel_layer" not in text


def test_collision_sources_read_the_d435_cloud() -> None:
    """StopBox reads the lidar and the D435 obstacle band, CliffStop is enabled on the D435 below-floor band, and both
    D435 sources are enabled on the laser_filter D435 cloud (the ToF names are gone)."""
    monitor = collision_monitor()
    assert monitor["observation_sources"] == ["scan", *DEPTH_COLLISION_SOURCES]
    for name in DEPTH_COLLISION_SOURCES:
        assert monitor[name]["type"] == "pointcloud", name
        assert monitor[name]["topic"] == D435_TOPIC, name
        assert monitor[name]["enabled"] is True, name
    assert monitor["StopBox"]["enabled"] is True
    assert monitor["StopBox"]["sources_names"] == ["scan", "depth_obstacles"]
    assert monitor["CliffStop"]["enabled"] is True
    assert monitor["CliffStop"]["sources_names"] == ["depth_cliff"]
    text = NAV2_PARAMS.read_text()
    for stale in ("tof_obstacles", "tof_cliff"):
        assert stale not in text, [line.strip() for line in text.splitlines() if stale in line]


def test_fixed_bands_derive_from_the_ground_limits() -> None:
    """Task T3: the fixed D435 bands (depth_voxel_layer marks and the collision monitor sources, user decision D-A1) come
    from the one ground estimator config: the obstacle band ends at the robot height (an overhang is never marked) and
    the cliff band at the drop band depth; both start beyond the step limits, so a climbable step or step down never
    marks the raw bands (the raw cloud has no fitted plane, the 6 cm margin over the step is its noise and head-tilt
    allowance). The two bands are equal in the costmap and the monitor."""
    ground = yaml.safe_load(GROUND_DEFAULTS.read_text())
    trav = ground["traversability"]
    step_up = 2 * trav["wheel_radius_m"] * trav["max_step_up_fraction"]
    step_down = 2 * trav["wheel_radius_m"] * trav["max_step_down_fraction"]
    layer = local_costmap()[LAYER]
    monitor = collision_monitor()
    mark, cliff = layer["depth_mark"], layer["depth_cliff_mark"]
    assert mark["max_obstacle_height"] == monitor["depth_obstacles"]["max_height"] == trav["robot_height_m"]
    assert ground["bands"]["obstacle_max_m"] <= trav["robot_height_m"]
    assert mark["min_obstacle_height"] == monitor["depth_obstacles"]["min_height"] > step_up
    assert cliff["min_obstacle_height"] == monitor["depth_cliff"]["min_height"] == -ground["bands"]["drop_max_m"]
    assert cliff["max_obstacle_height"] == monitor["depth_cliff"]["max_height"] < -step_down


def test_no_line_keeps_the_lidar_only_comment() -> None:
    """Every line commented `disabled (user 2026-10-24): lidar-only ...` is re-enabled (design D6): no line of
    nav2_params.yaml keeps the comment, and the nav2_bringup README D435 inputs section no longer lists a layer as still
    disabled; it states the ground layer and the ground keepout filter are enabled."""
    lines = NAV2_PARAMS.read_text().splitlines()
    stale = [line.strip() for line in lines if LIDAR_ONLY_MARKER in line or "lidar-only" in line]
    assert not stale, stale
    readme = NAV2_README.read_text()
    assert README_INPUTS_HEADING in readme
    section = readme.split(README_INPUTS_HEADING, 1)[1].split("\n## ", 1)[0]
    assert "Still disabled" not in section
    assert "`ground_voxel_layer` and `ground_cost_filter` are enabled" in section


def test_lidar_only_pin_is_retired() -> None:
    """tests/test_nav_lidar_only.py pinned the lidar-only state that ends with the D435 inputs: it is deleted, not
    tracked by git, and no longer documented in tests/README.md."""
    assert not (REPO_ROOT / LIDAR_ONLY_TEST).exists()
    tracked = subprocess.run(
        ["git", "ls-files", "--", LIDAR_ONLY_TEST], cwd=REPO_ROOT, capture_output=True, text=True, check=True
    ).stdout
    assert tracked == ""
    assert "test_nav_lidar_only" not in TESTS_README.read_text()


def test_tests_readme_documents_this_file() -> None:
    text = TESTS_README.read_text()
    assert "### test_nav_d435_inputs.py" in text
    section = text.split("### test_nav_d435_inputs.py", 1)[1].split("\n### ", 1)[0]
    for name in (
        "test_depth_voxel_layer_reads_the_d435_cloud",
        "test_collision_sources_read_the_d435_cloud",
        "test_fixed_bands_derive_from_the_ground_limits",
        "test_no_line_keeps_the_lidar_only_comment",
        "test_lidar_only_pin_is_retired",
        "test_tests_readme_documents_this_file",
    ):
        assert f"`{name}`" in section, name
