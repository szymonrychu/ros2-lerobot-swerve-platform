"""Collision monitor stale-source fail-safe pin (design D55, approval R1).

The 2026-10-22 16:30:35 "invalid source" stop (nodes/nav2_bringup/README.md, "Invalid source stop during the final
turn") must not be cured by weakening the fail-safe (user decision 2026-10-18): the node `source_timeout` stays 1.0 s
with no per-source override, and every source a polygon reads stays enabled, so an enabled source with no fresh data
for 1 s stops the robot. The pin holds for any source (the lidar `scan` and the head D435 `depth_obstacles` /
`depth_cliff`): the only polygon allowed to name a disabled source is one that is itself disabled and reads disabled
sources only. No ROS is available on the dev machine, so the shipped YAML is read directly.
"""

from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
NAV2_PARAMS = REPO_ROOT / "nodes" / "nav2_bringup" / "config" / "nav2_params.yaml"
SOURCE_TIMEOUT_S = 1.0


def collision_monitor_params() -> dict[str, Any]:
    """Load the shipped collision_monitor ros__parameters.

    Returns:
        dict[str, Any]: The collision_monitor parameter block of nav2_params.yaml.
    """
    return yaml.safe_load(NAV2_PARAMS.read_text())["collision_monitor"]["ros__parameters"]


def polygon_sources(params: dict[str, Any], polygon: str) -> list[str]:
    """Sources one polygon reads (nav2 1.3.13: all observation_sources when sources_names is absent).

    Args:
        params (dict[str, Any]): collision_monitor ros__parameters.
        polygon (str): Polygon name from `polygons`.

    Returns:
        list[str]: The observation source names the polygon checks.
    """
    return list(params[polygon].get("sources_names", params["observation_sources"]))


def test_fail_safe_stale_source_stop_kept() -> None:
    """source_timeout is 1.0 s with no per-source override, and every source a polygon relies on is enabled."""
    params = collision_monitor_params()
    sources: list[str] = params["observation_sources"]
    assert params["source_timeout"] == SOURCE_TIMEOUT_S
    for name in sources:
        assert "source_timeout" not in params[name], name
    guarded = []
    for polygon in params["polygons"]:
        names = polygon_sources(params, polygon)
        assert names, polygon
        assert set(names) <= set(sources), polygon
        enabled = {name: params[name]["enabled"] is True for name in names}
        if all(enabled.values()):
            if params[polygon]["enabled"] is True:
                guarded.append(polygon)
            continue
        assert params[polygon]["enabled"] is False and not any(enabled.values()), (polygon, enabled)
    assert guarded, "no enabled polygon reads only enabled sources"
