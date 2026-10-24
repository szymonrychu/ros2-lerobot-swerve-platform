"""Lidar-only path planning (user 2026-10-24): no enabled Nav2 costmap layer and no enabled collision-monitor source or
polygon reads the head ToF / ground estimator / RGB-D topics. The definitions stay in nav2_params.yaml (disabled) so a
RealSense D435 source can be switched in at the same places."""

from pathlib import Path
from typing import Any

import pytest
import yaml

NAV2_PARAMS = Path(__file__).resolve().parent.parent / "nodes" / "nav2_bringup" / "config" / "nav2_params.yaml"
HEAD_PREFIXES = ("/tof", "/ground", "/head_rgbd")
LIDAR_TOPIC = "/scan_filtered"
COSTMAPS = ("local_costmap", "global_costmap")
DISABLED_COMMENT = "disabled (user 2026-10-24): lidar-only path planning until the RealSense D435 replaces the head ToF"


def nav2() -> dict[str, Any]:
    """Load nav2_params.yaml.

    Returns:
        dict[str, Any]: The parsed parameter file.
    """
    return yaml.safe_load(NAV2_PARAMS.read_text())


def is_head_topic(topic: str) -> bool:
    """Whether a topic belongs to the head ToF / ground estimator / RGB-D family.

    Args:
        topic: Topic name.

    Returns:
        bool: True for /tof*, /ground*, /head_rgbd*.
    """
    return topic.startswith(HEAD_PREFIXES)


def layer_topics(layer: dict[str, Any]) -> list[str]:
    """Topics of an observation-source layer (STVL / obstacle layer).

    Args:
        layer: The layer parameter block.

    Returns:
        list[str]: Every observation source topic.
    """
    return [layer[s]["topic"] for s in layer.get("observation_sources", "").split()]


@pytest.mark.parametrize("costmap", COSTMAPS)
def test_no_enabled_costmap_layer_reads_a_head_topic(costmap: str) -> None:
    params = nav2()[costmap][costmap]["ros__parameters"]
    for name in params["plugins"]:
        layer = params[name]
        if layer.get("enabled", True) is False:
            continue
        for topic in layer_topics(layer):
            assert not is_head_topic(topic), f"{costmap}.{name} reads {topic}"
        assert "ground_cost" not in layer.get("filter_info_topic", ""), f"{costmap}.{name} is the ground cost filter"


def test_head_layers_are_defined_but_disabled() -> None:
    local = nav2()["local_costmap"]["local_costmap"]["ros__parameters"]
    for name in ("tof_voxel_layer", "ground_voxel_layer", "ground_cost_filter"):
        assert name in local["plugins"], name
        assert local[name]["enabled"] is False, name


def test_enabled_collision_monitor_sources_and_polygons_read_only_the_lidar() -> None:
    cm = nav2()["collision_monitor"]["ros__parameters"]
    for name in cm["observation_sources"]:
        if cm[name]["topic"] == LIDAR_TOPIC:
            assert cm[name]["enabled"] is True
        else:
            assert is_head_topic(cm[name]["topic"]), name
            assert cm[name]["enabled"] is False, name
    for name in cm["polygons"]:
        if cm[name]["enabled"] is False:
            continue
        for source in cm[name]["sources_names"]:
            assert cm[source]["enabled"] is True and not is_head_topic(cm[source]["topic"]), f"{name}.{source}"
    assert cm["StopBox"]["sources_names"] == ["scan"]
    assert cm["CliffStop"]["enabled"] is False


def test_every_disabled_line_carries_the_lidar_only_comment() -> None:
    text = NAV2_PARAMS.read_text()
    assert text.count(DISABLED_COMMENT) >= 7, "tof layer, ground layer, ground filter, StopBox, CliffStop, 2 sources"
