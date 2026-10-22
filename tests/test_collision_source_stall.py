"""Collision monitor stale-source fail-safe pin (design D55, approval R1).

The 2026-10-22 16:30:35 "invalid source" stop (nodes/nav2_bringup/README.md, "Invalid source stop during the final
turn") must not be cured by weakening the fail-safe (user decision 2026-10-18): every collision_monitor observation
source stays enabled and the node `source_timeout` stays 1.0 s with no per-source override. No ROS is available on the
dev machine, so the shipped YAML is read directly.
"""

from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
NAV2_PARAMS = REPO_ROOT / "nodes" / "nav2_bringup" / "config" / "nav2_params.yaml"
SOURCE_TIMEOUT_S = 1.0
OBSERVATION_SOURCES = ["scan", "tof_obstacles", "tof_cliff"]


def collision_monitor_params() -> dict[str, Any]:
    """Load the shipped collision_monitor ros__parameters.

    Returns:
        dict[str, Any]: The collision_monitor parameter block of nav2_params.yaml.
    """
    return yaml.safe_load(NAV2_PARAMS.read_text())["collision_monitor"]["ros__parameters"]


def test_fail_safe_stale_source_stop_kept() -> None:
    """Every observation source is enabled, the node source_timeout is 1.0 s and no source overrides it."""
    params = collision_monitor_params()
    assert params["source_timeout"] == SOURCE_TIMEOUT_S
    assert params["observation_sources"] == OBSERVATION_SOURCES
    for name in OBSERVATION_SOURCES:
        source = params[name]
        assert source["enabled"] is True, name
        assert "source_timeout" not in source, name
