"""Tests for the head_rgbd_calibration node config."""

from pathlib import Path

import pytest
import yaml
from pydantic import ValidationError
from ros2_common.head_rgbd_calibration.config import FloorFitConfig

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


def test_floor_defaults_and_tilt_range() -> None:
    floor = NodeConfig().floor
    assert floor.joint_states_topic == "/head/joint_states"
    assert floor.command_topic == "/head/joint_commands"
    assert floor.cloud_topic == "/tof/points"
    assert floor.command_frame_id == "head_rgbd_calibration"
    assert floor.pan_deg == [-30, -15, 0, 15, 30, 45]
    assert floor.tilt_deg == [20, 24, 28]
    assert floor.reach_tol_deg == 1.0
    assert floor.still_tol_deg == 0.2
    assert floor.still_window_s == 0.5
    assert floor.move_timeout_s == 6.0
    assert floor.settle_s == 0.5
    assert floor.clouds_per_pose == 10
    assert floor.min_clouds_per_pose == 5
    assert floor.cloud_timeout_s == 3.0
    assert floor.max_stamp_dt_s == 0.1
    assert floor.fit == FloorFitConfig()


@pytest.mark.parametrize(
    "text",
    [
        "floor:\n  tilt_deg: [12, 31]\n",
        "floor:\n  tilt_deg: [-30.5]\n",
        "floor:\n  tilt_deg: []\n",
        "floor:\n  pan_deg: []\n",
        "floor:\n  pan_deg: [95]\n",
        "floor:\n  clouds_per_pose: 0\n",
        "floor:\n  min_clouds_per_pose: 0\n",
        "floor:\n  clouds_per_pose: 4\n  min_clouds_per_pose: 5\n",
        "floor:\n  move_timeout_s: 0\n",
        "floor:\n  bogus: 1\n",
    ],
)
def test_floor_out_of_range_rejected(tmp_path: Path, text: str) -> None:
    path = tmp_path / "c.yaml"
    path.write_text(text)
    with pytest.raises(ValidationError):
        load_config(path)


def test_floor_tilt_range_edges_accepted(tmp_path: Path) -> None:
    path = tmp_path / "c.yaml"
    path.write_text("floor:\n  tilt_deg: [-30, 30]\n")
    assert load_config(path).floor.tilt_deg == [-30, 30]


def test_still_capture_defaults() -> None:
    cfg = NodeConfig()
    assert cfg.topics.head_joint_states == "/head/joint_states"
    assert cfg.topics.arm_joint_states == "/follower/joint_states"
    assert cfg.still.model_dump() == {
        "window_s": 0.6,
        "rgb_mad": 2.0,
        "joint_still_rad": 0.002,
        "max_joint_dt_s": 0.1,
        "timeout_s": 6.0,
    }
    assert cfg.arm_urdf_file == "/opt/ros2-lerobot-swerve-platform/nodes/web_ui/urdf/so101_arm.urdf"
    assert cfg.fit.board == cfg.board


@pytest.mark.parametrize("key", ["window_s", "rgb_mad", "joint_still_rad", "max_joint_dt_s", "timeout_s"])
def test_still_values_must_be_positive(key: str) -> None:
    with pytest.raises(ValidationError, match=key):
        NodeConfig.model_validate({"still": {key: 0}})


def test_fit_board_mismatch_refused_naming_both() -> None:
    with pytest.raises(ValidationError, match=r"fit\.board.*board"):
        NodeConfig.model_validate({"fit": {"board": {"squares_x": 7}}})
    cfg = NodeConfig.model_validate({"board": {"squares_x": 7}, "fit": {"board": {"squares_x": 7}}})
    assert cfg.fit.board.squares_x == 7
