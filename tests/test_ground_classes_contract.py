"""One traversability class contract (task T3): ground_estimator_cpp's GroundClass enum and cost table (C++) equal
shared/ros2_common/ground_classes.py, the class limits come from the one ground estimator config (repo defaults and
client.yml), and the Python decoder reads the C++ wire layout of /ground/classified_points."""

import math
import re
import struct
import sys
from pathlib import Path
from typing import Any

import numpy as np
import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
NODE_DIR = REPO_ROOT / "nodes" / "ground_estimator_cpp"
HEADER = NODE_DIR / "include" / "ground_estimator_cpp" / "ground_classes.hpp"
SOURCE = NODE_DIR / "src" / "core" / "ground_classes.cpp"
DEFAULTS = NODE_DIR / "config" / "ground_estimator.yaml"
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
MAX_STEP_M = 0.04  # user decision 2026-10-29: 0.04 m up and down
MAX_SLOPE_DEG = 15.0  # user decision 2026-10-29: in every direction
FLOAT32 = 7
UINT8 = 2

sys.path.insert(0, str(REPO_ROOT / "shared"))
try:
    from ros2_common import ground_classes as gc
finally:
    sys.path.remove(str(REPO_ROOT / "shared"))


def cpp_enum() -> dict[str, int]:
    """The GroundClass enumerators of the C++ header.

    Returns:
        dict[str, int]: name -> value.
    """
    body = re.search(r"enum class GroundClass : std::uint8_t\s*\{(.*?)\};", HEADER.read_text(), re.S)
    assert body, "GroundClass enum not found"
    return {m.group(1): int(m.group(2)) for m in re.finditer(r"(\w+)\s*=\s*(\d+)", body.group(1))}


def cpp_switch(function: str, result_type: str) -> dict[str, str]:
    """The case -> return pairs of a switch over GroundClass in the C++ source.

    Args:
        function (str): Function name.
        result_type (str): Prefix of the returned values ("" for string literals).

    Returns:
        dict[str, str]: class name -> returned name.
    """
    text = SOURCE.read_text()
    body = re.search(rf"\b{function}\(GroundClass c\)\s*\{{(.*?)\n\}}", text, re.S)
    assert body, function
    value = rf"{result_type}::(\w+)" if result_type else r'"(\w+)"'
    return dict(re.findall(rf"case GroundClass::(\w+):\s*return {value};", body.group(1)))


def ground_entry() -> dict[str, Any]:
    """The ground_estimator_cpp config block of group_vars/client.yml.

    Returns:
        dict[str, Any]: The parsed config.
    """
    nodes = yaml.safe_load(CLIENT_VARS.read_text())["ros2_nodes"]
    entry = next(n for n in nodes if n["name"] == "ground_estimator_cpp")
    return yaml.safe_load(entry["config"])


def test_enum_values_and_names_equal_cpp() -> None:
    assert cpp_enum() == {c.name: c.value for c in gc.GroundClass}
    assert cpp_switch("to_string", "") == {c.name: c.name for c in gc.GroundClass}
    assert gc.CLASS_COUNT == len(cpp_enum()) == 9


def test_costs_equal_cpp() -> None:
    assert cpp_switch("cost_of", "ClassCost") == {c.name: cost for c, cost in gc.COSTS.items()}
    assert gc.LETHAL_CLASSES == {gc.GroundClass.obstacle, gc.GroundClass.wall, gc.GroundClass.drop}
    assert gc.COSTS[gc.GroundClass.overhang] == "ignored"
    assert gc.COSTS[gc.GroundClass.unknown] == "unknown"


@pytest.mark.parametrize("source", ["repo", "client"])
def test_limits_are_the_user_decisions(source: str) -> None:
    config = yaml.safe_load(DEFAULTS.read_text()) if source == "repo" else ground_entry()
    limits = gc.limits_from_ground_config(config)
    assert math.isclose(limits.max_step_up_m, MAX_STEP_M, abs_tol=1e-4)
    assert math.isclose(limits.max_step_down_m, MAX_STEP_M, abs_tol=1e-4)
    assert limits.max_slope_deg == MAX_SLOPE_DEG
    assert limits.floor_band_m < limits.max_step_up_m
    assert max(limits.max_step_up_m, limits.max_step_down_m) <= limits.chassis_clearance_m
    assert config["bands"]["obstacle_max_m"] <= limits.robot_height_m
    assert limits.wall_min_height_m > limits.max_step_up_m


def test_client_limits_equal_the_repo_defaults() -> None:
    assert gc.limits_from_ground_config(ground_entry()) == gc.limits_from_ground_config(
        yaml.safe_load(DEFAULTS.read_text())
    )


def test_decoder_reads_the_cpp_layout() -> None:
    # The layout of ground_estimator_cpp::to_classified_msg (test_ground_classes.cpp ClassifiedCloudLayout).
    fields = [("x", 0), ("y", 4), ("z", 8), ("class", 12)]
    rows = [(1.0, 2.0, 3.0, 7), (-1.0, 0.5, 0.25, 8), (float("nan"), 0.0, 0.0, 1), (0.0, 0.0, 0.0, 200)]
    data = b"".join(struct.pack("<fffB3x", *r) for r in rows)
    xyz, classes = gc.decode_classified(data, len(rows), gc.CLASSIFIED_POINT_STEP, fields)
    np.testing.assert_allclose(xyz, [[1.0, 2.0, 3.0], [-1.0, 0.5, 0.25]])
    assert classes.tolist() == [gc.GroundClass.drop, gc.GroundClass.overhang]
    with pytest.raises(ValueError, match="class"):
        gc.decode_classified(data, len(rows), gc.CLASSIFIED_POINT_STEP, fields[:3])


def test_cpp_layout_constants_equal_python() -> None:
    text = SOURCE.read_text()
    assert int(re.search(r"POINT_STEP = (\d+);", text).group(1)) == gc.CLASSIFIED_POINT_STEP
    assert int(re.search(r"CLASS_OFFSET = (\d+);", text).group(1)) == 12
    assert 'cls.name = "class";' in text and gc.CLASS_FIELD == "class"


def test_class_name_never_guesses() -> None:
    assert gc.class_name(6) == "wall"
    assert gc.class_name(9) == "unknown"
    assert gc.class_name(-1) == "unknown"
