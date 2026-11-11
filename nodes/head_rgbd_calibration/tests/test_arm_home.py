"""The stored arm home (mcp_server's home.yaml) the board park drives to."""

import math
from pathlib import Path

import pytest
import yaml
from ros2_common.head_rgbd_calibration.config import ARM_JOINT_NAMES

from head_rgbd_calibration.arm_home import ArmHomeError, load_arm_home

CALIBRATED_HOME: dict[str, float] = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -math.pi / 2,
    "elbow_flex": math.pi / 2,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
    "gripper": -0.14419419406125028,
}


def write_home(path: Path, joints: object) -> Path:
    """Write a home file as mcp_server's home_store does.

    Args:
        path (Path): Target file.
        joints (object): The `joints` mapping.

    Returns:
        Path: The written file.
    """
    path.write_text(yaml.safe_dump({"joints": joints}, sort_keys=False))
    return path


def test_the_five_arm_joints_are_loaded_and_the_gripper_is_dropped(tmp_path: Path) -> None:
    home = load_arm_home(write_home(tmp_path / "home.yaml", CALIBRATED_HOME))
    assert list(home) == list(ARM_JOINT_NAMES)
    assert home == {name: CALIBRATED_HOME[name] for name in ARM_JOINT_NAMES}
    assert all(isinstance(value, float) for value in home.values())


def test_integer_angles_are_floats(tmp_path: Path) -> None:
    home = load_arm_home(write_home(tmp_path / "home.yaml", dict.fromkeys(ARM_JOINT_NAMES, 0)))
    assert home == dict.fromkeys(ARM_JOINT_NAMES, 0.0)


def test_a_missing_file_asks_to_store_a_home(tmp_path: Path) -> None:
    path = tmp_path / "home.yaml"
    with pytest.raises(ArmHomeError) as refused:
        load_arm_home(path)
    assert str(refused.value) == (
        f"no stored arm home at {path}: store the arm home (Calibrate > General, Set home) first"
    )


def test_unparsable_yaml_names_the_file(tmp_path: Path) -> None:
    path = tmp_path / "home.yaml"
    path.write_text("joints: {shoulder_pan: [0.0\n")
    with pytest.raises(ArmHomeError, match=f"cannot parse {path}"):
        load_arm_home(path)


@pytest.mark.parametrize("text", ["- 1\n- 2\n", "other: 1\n", "joints: 0.5\n", ""])
def test_no_joints_mapping_names_the_file(tmp_path: Path, text: str) -> None:
    path = tmp_path / "home.yaml"
    path.write_text(text)
    with pytest.raises(ArmHomeError, match=f"invalid arm home {path}: no `joints` mapping"):
        load_arm_home(path)


def test_a_missing_arm_joint_is_named(tmp_path: Path) -> None:
    joints = {k: v for k, v in CALIBRATED_HOME.items() if k != "wrist_roll"}
    path = write_home(tmp_path / "home.yaml", joints)
    with pytest.raises(ArmHomeError, match=f"invalid arm home {path}: missing joint wrist_roll"):
        load_arm_home(path)


@pytest.mark.parametrize("value", [float("nan"), float("inf"), "fast", None, True])
def test_a_non_finite_or_non_number_angle_is_refused(tmp_path: Path, value: object) -> None:
    path = write_home(tmp_path / "home.yaml", {**CALIBRATED_HOME, "elbow_flex": value})
    with pytest.raises(ArmHomeError, match=f"invalid arm home {path}: .*elbow_flex"):
        load_arm_home(path)
