"""head_state_publisher applies an override tof_joint origin from the robot state dir (design D8)."""

import ast
import difflib
import importlib.util
import json
import logging
import sys
from pathlib import Path
from types import ModuleType

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
LAUNCH_DIR = REPO_ROOT / "nodes" / "head_state_publisher" / "launch"
HEAD_DESCRIPTION = LAUNCH_DIR / "head_description.py"
LAUNCH_FILE = LAUNCH_DIR / "head_state_publisher.launch.py"
URDF = REPO_ROOT / "nodes" / "web_ui" / "urdf" / "head.urdf"
URDF_TEXT = REPO_ROOT / "shared" / "ros2_common" / "head_rgbd_calibration" / "urdf_text.py"
APPLIED_PATH = "/var/lib/ros2/head_rgbd_calibration/applied/tof_joint.json"


def load_head_description() -> ModuleType:
    """Load head_description.py by path, as the launch file does."""
    spec = importlib.util.spec_from_file_location("head_description_under_test", HEAD_DESCRIPTION)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def description() -> ModuleType:
    return load_head_description()


def build(description: ModuleType, override: Path) -> str:
    return description.robot_description(URDF, override, URDF_TEXT)


def test_override_rewrites_only_tof_joint(description: ModuleType, tmp_path: Path) -> None:
    """The tof_joint origin carries the JSON values; every other line is unchanged."""
    override = tmp_path / "tof_joint.json"
    override.write_text(json.dumps({"xyz": [0.01, 0.02, 0.03], "rpy": [0.1, 0.2, 0.3], "note": "test note"}))
    original = URDF.read_text()
    result = build(description, override)
    assert result != original
    assert "0.010000000 0.020000000 0.030000000" in result
    assert "0.100000000 0.200000000 0.300000000" in result
    diff = [
        line[1:]
        for line in difflib.unified_diff(original.splitlines(), result.splitlines(), lineterm="", n=0)
        if line[:1] in "+-" and line[:3] not in ("+++", "---")
    ]
    assert diff
    assert all("origin" in line or "<!--" in line for line in diff)


def test_missing_override_keeps_repo_urdf(description: ModuleType, tmp_path: Path) -> None:
    assert build(description, tmp_path / "absent.json") == URDF.read_text()


@pytest.mark.parametrize(
    "content",
    ["{not json", json.dumps({"xyz": [1, 2], "rpy": [0, 0, 0], "note": "x"}), json.dumps([1, 2, 3])],
)
def test_invalid_override_ignored_with_warning(
    description: ModuleType, tmp_path: Path, caplog: pytest.LogCaptureFixture, content: str
) -> None:
    override = tmp_path / "tof_joint.json"
    override.write_text(content)
    with caplog.at_level(logging.WARNING):
        result = build(description, override)
    assert result == URDF.read_text()
    assert any(r.levelno == logging.WARNING and str(override) in r.getMessage() for r in caplog.records)


def test_head_description_is_stdlib_only() -> None:
    tree = ast.parse(HEAD_DESCRIPTION.read_text())
    roots = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            roots.update(a.name.split(".")[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom):
            assert node.level == 0
            roots.add(node.module.split(".")[0])
    assert roots <= set(sys.stdlib_module_names) | {"__future__"}


def test_launch_file_references_override_pieces() -> None:
    source = LAUNCH_FILE.read_text()
    assert "head_description" in source
    assert APPLIED_PATH in source
    assert "urdf_text.py" in source
