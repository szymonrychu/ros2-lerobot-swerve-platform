"""The ground_estimator_cpp README documents every topic, state key, config key and metric of the node."""

import re
from pathlib import Path
from typing import Any

import pytest
import yaml

NODE = Path(__file__).resolve().parent.parent / "nodes" / "ground_estimator_cpp"
README = NODE / "README.md"
REQUIRED_SECTIONS = (
    "## Topics",
    "## State",
    "## Labels",
    "## Obstacle or warning",
    "## Configuration",
    "## Metrics",
    "## Plane tracking",
    "## No placeholder data",
)
INPUT_TOPICS = ("/tof/points_filtered", "/imu/data", "/head_rgbd/points")
OUTPUT_TOPICS = (
    "/ground/obstacle_points",
    "/ground/drop_points",
    "/ground/labelled_points",
    "/ground/state",
    "/ground/warning_areas",
)


def flatten(data: dict[str, Any], prefix: str = "") -> list[str]:
    """Dotted key paths of the leaves of a nested mapping.

    :param data: parsed YAML mapping.
    :param prefix: dotted prefix of the mapping.
    :return: leaf key paths.
    """
    keys: list[str] = []
    for key, value in data.items():
        path = f"{prefix}{key}"
        if isinstance(value, dict):
            keys.extend(flatten(value, f"{path}."))
        else:
            keys.append(path)
    return keys


def state_keys() -> list[str]:
    """STATE_KEYS parsed from include/ground_estimator_cpp/state.hpp.

    :return: the state document keys.
    """
    text = (NODE / "include" / "ground_estimator_cpp" / "state.hpp").read_text()
    block = re.search(r"STATE_KEYS\s*=\s*\{(.*?)\};", text, re.S)
    assert block, "STATE_KEYS not found"
    return re.findall(r'"(\w+)"', block.group(1))


def metric_names() -> list[str]:
    """Every ground_* metric name string of the node sources.

    :return: the metric names.
    """
    names: set[str] = set()
    for path in (NODE / "src" / "ground_node.cpp", NODE / "include" / "ground_estimator_cpp" / "ground_node.hpp"):
        names.update(re.findall(r'"(ground_[a-z_]+)"', path.read_text()))
    return sorted(names)


@pytest.fixture(scope="module")
def readme() -> str:
    """README text."""
    return README.read_text()


def test_readme_has_every_required_section(readme: str) -> None:
    for heading in REQUIRED_SECTIONS:
        assert re.search(rf"^{re.escape(heading)}\s*$", readme, re.M), heading


def test_readme_names_every_topic(readme: str) -> None:
    for topic in (*INPUT_TOPICS, *OUTPUT_TOPICS):
        assert f"`{topic}`" in readme, topic
    assert "base_link" in readme
    assert "plane-relative" in readme


def test_readme_names_every_config_key(readme: str) -> None:
    shipped = yaml.safe_load((NODE / "config" / "ground_estimator.yaml").read_text())
    keys = flatten(shipped)
    assert keys
    for key in keys:
        assert f"`{key}`" in readme, key


def test_readme_names_every_state_key(readme: str) -> None:
    keys = state_keys()
    assert len(keys) >= 18
    for key in keys:
        assert f"`{key}`" in readme, key


def test_readme_names_every_metric(readme: str) -> None:
    names = metric_names()
    assert len(names) >= 12
    for name in names:
        assert f"`{name}`" in readme, name


def test_readme_documents_labels_and_the_obstacle_rule(readme: str) -> None:
    labels = readme.split("## Labels", 1)[1].split("\n## ", 1)[0]
    for value in range(5):
        assert re.search(rf"\|\s*{value}\s*\|", labels), value
    rule = readme.split("## Obstacle or warning", 1)[1].split("\n## ", 1)[0]
    for phrase in ("one third", "wheel diameter", "max_slope_deg", "costmap_zones", "slow-down"):
        assert phrase in rule, phrase


def test_readme_documents_robustness(readme: str) -> None:
    assert re.search(r"^## Robustness\s*$", readme, re.M)
    section = readme.split("## Robustness", 1)[1].split("\n## ", 1)[0]
    for phrase in (
        "max_spread_m",
        "lethal_min_points",
        "slope_min_cells",
        "hysteresis",
        "enter_frames",
        "roll_error_deg",
        "15 deg",
    ):
        assert phrase in section, phrase
