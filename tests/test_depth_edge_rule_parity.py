"""One depth edge ("veil") rule in C++ and Python: shared/ros2_common_cpp/test/data/depth_edge_mask_parity.yaml.

The fixture's `defaults` are the rule's numbers; the C++ DepthEdgeMaskParams defaults, the Python constants of
ros2_common.depth_edge_mask, head_rgbd_cpp's edge_mask (repo config and client.yml) and mcp_server's camera_arm edge
settings (code defaults and client.yml) must equal them, and the Python rule must give every fixture mask (the C++ rule
is held to the same masks by shared/ros2_common_cpp/test/test_depth_mask.cpp).
"""

import re
import sys
from pathlib import Path
from typing import Any

import numpy as np
import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
FIXTURE = REPO_ROOT / "shared" / "ros2_common_cpp" / "test" / "data" / "depth_edge_mask_parity.yaml"
CPP_HEADER = REPO_ROOT / "shared" / "ros2_common_cpp" / "include" / "ros2_common_cpp" / "depth_mask.hpp"
HEAD_RGBD_CONFIG = REPO_ROOT / "nodes" / "head_rgbd_cpp" / "config" / "head_rgbd.yaml"
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
M_PER_MM = np.float32(0.001)

sys.path[:0] = [str(REPO_ROOT / "nodes" / "mcp_server"), str(REPO_ROOT / "shared")]
try:
    from mcp_server.config import CameraArmSettings
    from ros2_common import depth_edge_mask as rule
finally:
    del sys.path[:2]


def fixture() -> dict[str, Any]:
    """Load the parity fixture.

    Returns:
        dict[str, Any]: defaults and cases.
    """
    return yaml.safe_load(FIXTURE.read_text())


def node_config(name: str) -> dict[str, Any]:
    """Parse a node's `config: |` block of group_vars/client.yml.

    Args:
        name (str): ros2_nodes entry name.

    Returns:
        dict[str, Any]: The node config.
    """
    entry = next(n for n in yaml.safe_load(CLIENT_VARS.read_text())["ros2_nodes"] if n["name"] == name)
    return yaml.safe_load(entry["config"])


def cpp_default(field: str) -> float:
    """A default of DepthEdgeMaskParams read from the C++ header.

    Args:
        field (str): Member name.

    Returns:
        float: Its default value.
    """
    match = re.search(rf"\b(?:int|double)\s+{field}\s*=\s*([0-9.]+)\s*;", CPP_HEADER.read_text())
    assert match, field
    return float(match.group(1))


@pytest.mark.parametrize("case", fixture()["cases"], ids=lambda c: c["name"])
def test_python_rule_gives_every_fixture_mask(case: dict[str, Any]) -> None:
    depth_m = np.asarray(case["depth_mm"], dtype=np.uint16).astype(np.float32) * M_PER_MM
    p = case["params"]
    removed = rule.depth_edge_mask(depth_m, p["window_px"], p["abs_m"], p["rel"], p["grow_px"])
    assert removed.dtype == bool
    np.testing.assert_array_equal(removed.astype(int), np.asarray(case["removed"]))


def test_python_constants_are_the_fixture_defaults() -> None:
    d = fixture()["defaults"]
    assert (rule.WINDOW_PX, rule.ABS_M, rule.REL, rule.GROW_PX) == (
        d["window_px"],
        d["abs_m"],
        d["rel"],
        d["grow_px"],
    )


def test_cpp_defaults_are_the_fixture_defaults() -> None:
    d = fixture()["defaults"]
    for field in ("window_px", "abs_m", "rel", "grow_px"):
        assert cpp_default(field) == d[field], field


def test_head_rgbd_edge_mask_is_the_fixture_defaults() -> None:
    d = fixture()["defaults"]
    for cfg in (yaml.safe_load(HEAD_RGBD_CONFIG.read_text())["edge_mask"], node_config("head_rgbd_cpp")["edge_mask"]):
        assert {k: cfg[k] for k in d} == d


def test_mcp_camera_arm_edge_settings_are_the_fixture_defaults() -> None:
    d = fixture()["defaults"]
    for cfg in (CameraArmSettings().model_dump(), node_config("mcp_server")["camera_arm"]):
        assert (cfg["edge_kernel_px"], cfg["edge_abs_m"], cfg["edge_rel"]) == (d["window_px"], d["abs_m"], d["rel"])


def test_the_rule_is_one_sided_the_nearer_side_keeps_its_edge() -> None:
    depth = np.full((5, 8), 0.5, dtype=np.float32)
    depth[:, 4:] = 0.9
    removed = rule.depth_edge_mask(depth)
    assert removed[:, 4].all() and not removed[:, :4].any() and not removed[:, 5:].any()


def test_invalid_parameters_are_refused_as_in_cpp() -> None:
    depth = np.full((5, 5), 1.0, dtype=np.float32)
    for args in ((4,), (1,), (3, -0.01), (3, 0.01, -0.1), (3, 0.0, 0.0), (3, 0.01, 0.025, -1)):
        with pytest.raises(ValueError):
            rule.depth_edge_mask(depth, *args)
