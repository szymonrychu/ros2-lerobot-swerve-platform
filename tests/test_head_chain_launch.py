"""head_state_publisher publishes the applied head chain and tof_joint (design D7, D8)."""

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
CHAIN_XYZ = {
    "head_mount_joint": "0.111 0.222 0.333",
    "head_tilt": "0.0 0.0 0.0444",
    "head_camera_joint": "0.05 -0.025 0.06",
}
CHAIN_RPY = {"head_mount_joint": "0.01 0.02 0.0", "head_tilt": "0.0 0.0 0.0", "head_camera_joint": "0.0 0.03 0.0"}
TOF = {"xyz": [0.01, 0.02, 0.03], "rpy": [0.1, 0.2, 0.3], "note": "tof"}


def load(path: Path, name: str) -> ModuleType:
    """Load a module by file path."""
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def chain_json() -> str:
    joints = {
        n: {"xyz": [float(v) for v in CHAIN_XYZ[n].split()], "rpy": [float(v) for v in CHAIN_RPY[n].split()]}
        for n in CHAIN_XYZ
    }
    return json.dumps({"note": "chain", "joints": joints})


@pytest.fixture
def description() -> ModuleType:
    return load(HEAD_DESCRIPTION, "head_description_chain_test")


def build(description: ModuleType, applied: Path) -> str:
    return description.robot_description(URDF, applied, URDF_TEXT)


def test_chain_file_changes_three_joint_origins(description: ModuleType, tmp_path: Path) -> None:
    (tmp_path / "head_chain.json").write_text(chain_json())
    result = build(description, tmp_path)
    assert result != URDF.read_text()
    assert "0.111000000 0.222000000 0.333000000" in result
    assert "0.044400000" in result
    assert "0.050000000 -0.025000000 0.060000000" in result


def test_chain_and_tof_together(description: ModuleType, tmp_path: Path) -> None:
    (tmp_path / "head_chain.json").write_text(chain_json())
    (tmp_path / "tof_joint.json").write_text(json.dumps(TOF))
    result = build(description, tmp_path)
    assert "0.111000000 0.222000000 0.333000000" in result
    assert "0.010000000 0.020000000 0.030000000" in result


def test_invalid_chain_warns_and_tof_still_applies(
    description: ModuleType, tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    (tmp_path / "head_chain.json").write_text("{not json")
    (tmp_path / "tof_joint.json").write_text(json.dumps(TOF))
    with caplog.at_level(logging.WARNING):
        result = build(description, tmp_path)
    assert "0.111000000" not in result
    assert "0.010000000 0.020000000 0.030000000" in result
    assert any(r.levelno == logging.WARNING and "head_chain.json" in r.getMessage() for r in caplog.records)


def test_unloadable_urdf_text_returns_urdf(
    description: ModuleType, tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    (tmp_path / "head_chain.json").write_text(chain_json())
    with caplog.at_level(logging.WARNING):
        result = description.robot_description(URDF, tmp_path, tmp_path / "missing.py")
    assert result == URDF.read_text()
    assert any(r.levelno == logging.WARNING for r in caplog.records)


def test_launch_passes_overridden_text(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    """generate_launch_description hands the overridden URDF to robot_state_publisher."""
    (tmp_path / "head_chain.json").write_text(chain_json())
    captured: dict = {}

    class FakeNode:
        def __init__(self, **kwargs: object) -> None:
            captured.update(kwargs)

    class FakeDescription:
        def __init__(self, entities: list) -> None:
            self.entities = entities

    launch_mod = ModuleType("launch")
    launch_mod.LaunchDescription = FakeDescription
    ros_mod = ModuleType("launch_ros")
    actions_mod = ModuleType("launch_ros.actions")
    actions_mod.Node = FakeNode
    monkeypatch.setitem(sys.modules, "launch", launch_mod)
    monkeypatch.setitem(sys.modules, "launch_ros", ros_mod)
    monkeypatch.setitem(sys.modules, "launch_ros.actions", actions_mod)
    launch = load(LAUNCH_FILE, "head_launch_under_test")
    monkeypatch.setattr(launch, "APPLIED_DIR", tmp_path)
    launch.generate_launch_description()
    text = captured["parameters"][0]["robot_description"]
    assert "0.111000000 0.222000000 0.333000000" in text
