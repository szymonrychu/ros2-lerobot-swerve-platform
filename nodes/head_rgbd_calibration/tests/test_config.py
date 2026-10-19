"""Tests for the head_rgbd_calibration node config."""

from pathlib import Path

import pytest
import yaml
from pydantic import ValidationError

from head_rgbd_calibration.config import CONFIG_ENV, NodeConfig, load_config

NODE_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = NODE_DIR.parents[1]
SHIPPED = NODE_DIR / "config" / "head_rgbd_calibration.yaml"
TOOL_YAML = REPO_ROOT / "scripts" / "head_rgbd_calibrate.yaml"


def test_defaults_equal_shipped_yaml() -> None:
    assert load_config(SHIPPED) == NodeConfig()
    assert yaml.safe_load(SHIPPED.read_text()) == NodeConfig().model_dump(mode="json")


def test_empty_file_gives_defaults(tmp_path: Path) -> None:
    path = tmp_path / "c.yaml"
    path.write_text("")
    assert load_config(path) == NodeConfig()


def test_env_name() -> None:
    assert CONFIG_ENV == "HEAD_RGBD_CALIBRATION_CONFIG"


def test_unknown_key_rejected_naming_key(tmp_path: Path) -> None:
    path = tmp_path / "c.yaml"
    path.write_text("bogus_key: 1\n")
    with pytest.raises(ValidationError, match="bogus_key"):
        load_config(path)


def test_nested_unknown_key_rejected(tmp_path: Path) -> None:
    path = tmp_path / "c.yaml"
    path.write_text("topics:\n  nope_topic: /x\n")
    with pytest.raises(ValidationError, match="nope_topic"):
        load_config(path)


@pytest.mark.parametrize(
    "text",
    [
        "max_capture_s: 0\n",
        "compute_timeout_s: -1\n",
        "compute_nice: 20\n",
        "compute_nice: -1\n",
        "qos:\n  tof_depth: 0\n",
        "qos:\n  rgb_depth: 0\n",
        "coverage:\n  cols: 2\n",
        "coverage:\n  cols: 7\n",
        "coverage:\n  rows: 1\n",
        "coverage:\n  rows: 5\n",
    ],
)
def test_out_of_range_rejected(tmp_path: Path, text: str) -> None:
    path = tmp_path / "c.yaml"
    path.write_text(text)
    with pytest.raises(ValidationError):
        load_config(path)


def test_core_blocks_equal_calibrate_tool_yaml() -> None:
    tool = yaml.safe_load(TOOL_YAML.read_text())
    cfg = NodeConfig()
    assert cfg.board.model_dump(mode="json") == {**tool["board"]}
    assert cfg.selection.model_dump(mode="json") == {**tool["selection"]}
    assert cfg.limits.model_dump(mode="json") == tool["limits"]
