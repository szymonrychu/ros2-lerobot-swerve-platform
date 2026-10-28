"""Tests for the head_rgbd_calibration node config."""

from pathlib import Path

import pytest
import yaml
from pydantic import ValidationError
from ros2_common.head_rgbd_calibration.arm_pose_check import PoseCheckConfig
from ros2_common.head_rgbd_calibration.config import ARM_JOINT_NAMES, FloorFitConfig

from head_rgbd_calibration.config import CONFIG_ENV, ArmSequenceConfig, NodeConfig, load_config

NODE_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = NODE_DIR.parents[1]
SHIPPED = NODE_DIR / "config" / "head_rgbd_calibration.yaml"
CLIENT_YML = REPO_ROOT / "ansible" / "group_vars" / "client.yml"


def test_defaults_equal_shipped_yaml() -> None:
    assert load_config(SHIPPED) == NodeConfig()
    assert yaml.safe_load(SHIPPED.read_text()) == NodeConfig().model_dump(mode="json")


def test_client_yml_arm_sequence_and_board_mount_equal_defaults() -> None:
    """The deployed config block carries the shipped arm sequence and the nominal board mount, key for key."""
    nodes = yaml.safe_load(CLIENT_YML.read_text())["ros2_nodes"]
    block = yaml.safe_load(next(n for n in nodes if n["name"] == "head_rgbd_calibration")["config"])
    defaults = NodeConfig().model_dump(mode="json")
    assert block["arm_sequence"] == defaults["arm_sequence"]
    assert block["fit"]["board_mount"] == defaults["fit"]["board_mount"]
    deployed = NodeConfig.model_validate(block)
    assert deployed.arm_sequence == NodeConfig().arm_sequence
    assert deployed.fit.board_mount == NodeConfig().fit.board_mount


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


def test_robot_defaults() -> None:
    robot = NodeConfig().robot
    assert robot.mcp_url == "http://127.0.0.1:18200/mcp"
    assert robot.token_file == "/etc/ros2/mcp_server/token"
    assert robot.board_park_token_file == "/etc/ros2/mcp_server/board_park_token"  # its own token, not the shared one
    assert robot.call_timeout_s == 60.0
    assert robot.move_timeout_s == 120.0


@pytest.mark.parametrize("key", ["call_timeout_s", "move_timeout_s"])
def test_robot_timeouts_must_be_positive(key: str) -> None:
    with pytest.raises(ValidationError):
        NodeConfig.model_validate({"robot": {key: 0}})


def test_arm_sequence_defaults() -> None:
    seq = NodeConfig().arm_sequence
    assert set(seq.base_pose) == set(ARM_JOINT_NAMES)
    assert 0.0 <= seq.gripper_rad <= seq.pose_check.max_roll_gripper_rad
    assert seq.max_roll_delta_rad == 1.2
    assert seq.max_flex_delta_rad == 0.4
    assert seq.max_big_delta_rad == 0.35
    assert seq.speed_scale == 0.1  # half the 0.2 of the jaw-mount set: the board on a stick is fragile
    assert load_config(SHIPPED).arm_sequence.speed_scale == 0.1
    assert seq.settle_s == 1.0
    assert seq.confirm_timeout_s == 600.0
    assert seq.head_offsets_deg == [(0, 0), (10, 0), (-10, 0), (0, 6)]
    assert seq.pose_check == PoseCheckConfig()
    assert seq.provisional_margin_m == 0.05
    assert len(seq.tier0_poses) >= 8 and len(seq.tier1_poses) >= 8 and len(seq.tier2_poses) >= 12
    assert ArmSequenceConfig() == seq


def test_arm_sequence_board_margin() -> None:
    seq = NodeConfig().arm_sequence
    assert seq.board_margin_m(nominal_mount=True) == pytest.approx(0.03 + 0.05)
    assert seq.board_margin_m(nominal_mount=False) == pytest.approx(0.03)


@pytest.mark.parametrize("speed", [0.0, -0.1, 0.31])
def test_arm_sequence_speed_scale_range(speed: float) -> None:
    with pytest.raises(ValidationError, match="speed_scale"):
        NodeConfig.model_validate({"arm_sequence": {"speed_scale": speed}})
    assert NodeConfig.model_validate({"arm_sequence": {"speed_scale": 0.3}}).arm_sequence.speed_scale == 0.3


def sequence_with(**changes: object) -> dict:
    """The shipped arm_sequence block (JSON form) with top-level keys replaced."""
    return {**NodeConfig().arm_sequence.model_dump(mode="json"), **changes}


def changed_pose(tier: int, index: int, **joints: float) -> dict:
    """The shipped arm_sequence block with one pose of a tier changed (joints named, None removes the joint)."""
    data = NodeConfig().arm_sequence.model_dump(mode="json")
    pose = {**data[f"tier{tier}_poses"][index], **joints}
    data[f"tier{tier}_poses"][index] = {name: value for name, value in pose.items() if value is not None}
    return data


def base(joint: str) -> float:
    return NodeConfig().arm_sequence.base_pose[joint]


@pytest.mark.parametrize(
    ("data", "pattern"),
    [
        (lambda: changed_pose(0, 1, gripper=0.3), r"tier 0 pose 1\b.*gripper"),
        (lambda: changed_pose(1, 2, elbow_flex=None), r"tier 1 pose 2\b.*elbow_flex"),
        (lambda: changed_pose(0, 3, wrist_flex=base("wrist_flex") + 0.1), r"tier 0 pose 3\b.*wrist_flex"),
        (lambda: changed_pose(0, 0, wrist_roll=base("wrist_roll") + 1.25), r"tier 0 pose 0\b.*wrist_roll"),
        (lambda: changed_pose(1, 4, elbow_flex=base("elbow_flex") + 0.1), r"tier 1 pose 4\b.*elbow_flex"),
        (lambda: changed_pose(1, 0, wrist_flex=base("wrist_flex") - 0.45), r"tier 1 pose 0\b.*wrist_flex"),
        (lambda: changed_pose(1, 5, wrist_roll=base("wrist_roll") - 1.3), r"tier 1 pose 5\b.*wrist_roll"),
        (lambda: changed_pose(2, 3, shoulder_pan=base("shoulder_pan") + 0.36), r"tier 2 pose 3\b.*shoulder_pan"),
        (lambda: changed_pose(2, 6, shoulder_lift=base("shoulder_lift") - 0.4), r"tier 2 pose 6\b.*shoulder_lift"),
        (lambda: changed_pose(2, 7, elbow_flex=base("elbow_flex") + 0.5), r"tier 2 pose 7\b.*elbow_flex"),
        (lambda: changed_pose(2, 8, wrist_flex=base("wrist_flex") + 0.5), r"tier 2 pose 8\b.*wrist_flex"),
        (lambda: changed_pose(2, 9, wrist_roll=base("wrist_roll") + 1.5), r"tier 2 pose 9\b.*wrist_roll"),
        (lambda: changed_pose(2, 0, bogus_joint=0.0), r"tier 2 pose 0\b.*bogus_joint"),
        (
            lambda: sequence_with(base_pose={**NodeConfig().arm_sequence.base_pose, "gripper": 0.1}),
            r"base_pose.*gripper",
        ),
        (
            lambda: sequence_with(
                base_pose={k: v for k, v in NodeConfig().arm_sequence.base_pose.items() if k != "wrist_roll"}
            ),
            r"base_pose.*wrist_roll",
        ),
        (lambda: sequence_with(gripper_rad=0.81), r"gripper_rad.*max_roll_gripper_rad"),
        (lambda: sequence_with(head_offsets_deg=[]), r"head_offsets_deg"),
    ],
    ids=[
        "gripper-in-pose",
        "missing-joint",
        "tier0-moves-flex",
        "tier0-roll-too-far",
        "tier1-moves-elbow",
        "tier1-flex-too-far",
        "tier1-roll-too-far",
        "tier2-pan-too-far",
        "tier2-lift-too-far",
        "tier2-elbow-too-far",
        "tier2-flex-too-far",
        "tier2-roll-too-far",
        "unknown-joint",
        "base-gripper",
        "base-missing-joint",
        "gripper-too-open",
        "no-head-offsets",
    ],
)
def test_arm_sequence_validation_names_pose(data, pattern: str) -> None:
    with pytest.raises(ValidationError, match=pattern):
        NodeConfig.model_validate({"arm_sequence": data()})


def test_arm_sequence_error_names_every_bad_pose() -> None:
    data = changed_pose(0, 1, wrist_flex=base("wrist_flex") + 0.1)
    data["tier2_poses"][2]["shoulder_pan"] = base("shoulder_pan") + 0.5
    with pytest.raises(ValidationError) as err:
        NodeConfig.model_validate({"arm_sequence": data})
    message = str(err.value)
    assert "tier 0 pose 1" in message and "wrist_flex" in message
    assert "tier 2 pose 2" in message and "shoulder_pan" in message


def test_fit_board_mount_default_is_the_measured_mount() -> None:
    mount = NodeConfig().fit.board_mount
    assert mount.xyz_init != (0.0, 0.0, 0.0)
    assert mount == load_config(SHIPPED).fit.board_mount
    assert mount.outline_margin_m == 0.01
    assert "board_mount: # measured by the user 2026-10-23, not fit-solved" in SHIPPED.read_text()


def test_park_pose_is_the_parked_arm_and_sag_margin_defaults() -> None:
    parked = yaml.safe_load((REPO_ROOT / "scripts" / "gen_nav_footprint.yaml").read_text())["parked_arm_joints"]
    seq = NodeConfig().arm_sequence
    assert seq.park_pose == {name: parked[name] for name in ARM_JOINT_NAMES}
    assert seq.park_board_sag_margin_m == 0.01
    assert load_config(SHIPPED).arm_sequence.park_board_sag_margin_m == 0.01


def test_park_pose_names_every_arm_joint_and_sag_margin_is_not_negative() -> None:
    pose = {k: v for k, v in NodeConfig().arm_sequence.park_pose.items() if k != "wrist_roll"}
    with pytest.raises(ValueError, match="park_pose: missing joint wrist_roll"):
        NodeConfig.model_validate({"arm_sequence": sequence_with(park_pose=pose)})
    with pytest.raises(ValueError):
        NodeConfig.model_validate({"arm_sequence": sequence_with(park_board_sag_margin_m=-0.001)})


def test_board_record_max_age_defaults_to_two_hours_and_must_be_positive() -> None:
    assert NodeConfig().arm_sequence.board_record_max_age_s == 7200.0
    assert load_config(SHIPPED).arm_sequence.board_record_max_age_s == 7200.0
    with pytest.raises(ValueError):
        NodeConfig.model_validate({"arm_sequence": sequence_with(board_record_max_age_s=0)})
