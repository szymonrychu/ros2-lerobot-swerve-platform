"""Tests for the head_rgbd_calibration node config."""

import json
from pathlib import Path

import pytest
import yaml
from pydantic import ValidationError
from ros2_common.head_rgbd_calibration.arm_pose_check import PoseCheckConfig
from ros2_common.head_rgbd_calibration.config import (
    ARM_BOARD,
    ARM_JOINT_NAMES,
    FULL_BOARD,
    FloorFitConfig,
    GroundBoardFitConfig,
)

from head_rgbd_calibration.config import CONFIG_ENV, ArmSequenceConfig, NodeConfig, load_config

NODE_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = NODE_DIR.parents[1]
SHIPPED = NODE_DIR / "config" / "head_rgbd_calibration.yaml"
ARM_JSON = REPO_ROOT / "markers" / "charuco-arm-6x5-28mm.json"
CLIENT_YML = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
MARKERS_YAML = REPO_ROOT / "scripts" / "markers_generate.yaml"
# Keys of the retired ToF calibration, with their last defaults.
RETIRED_SELECTION_KEYS = {
    "still_tof_mad": 0.01,
    "max_pair_dt_s": 0.05,
    "amplitude_percentiles": [1, 99],
    "tof_upscale": 2,
}
RETIRED_LIMIT_KEYS = {
    "rgb_max_rms_px": 0.5,
    "tof_max_rms_px": 0.3,
    "stereo_max_rms_px": 1.0,
    "tof_zero_distortion_rms_px": 0.3,
}

# The pose check body is not written in any config: its default loads shared/arm_collision robot_body.yaml (the one
# body definition), so the key-for-key comparisons leave it out.
LOADED_BODY = {"arm_sequence": {"pose_check": {"body"}}}


def test_defaults_equal_shipped_yaml() -> None:
    assert load_config(SHIPPED) == NodeConfig()
    assert yaml.safe_load(SHIPPED.read_text()) == NodeConfig().model_dump(mode="json", exclude=LOADED_BODY)


def client_block() -> dict:
    """The head_rgbd_calibration config block of ansible/group_vars/client.yml."""
    nodes = yaml.safe_load(CLIENT_YML.read_text())["ros2_nodes"]
    return yaml.safe_load(next(n for n in nodes if n["name"] == "head_rgbd_calibration")["config"])


def test_client_block_equals_the_defaults() -> None:
    """The deployed config block shows every key at its default, key for key."""
    block = client_block()
    assert block == NodeConfig().model_dump(mode="json", exclude=LOADED_BODY)
    assert NodeConfig.model_validate(block) == NodeConfig()


def test_defaults_are_the_d435() -> None:
    cfg = NodeConfig()
    assert cfg.topics.model_dump() == {
        "rgb": "/head/d435/color/image_raw/compressed",
        "depth": "/head/d435/aligned_depth_to_color/image_raw",
        "rgb_info": "/head/d435/color/camera_info",
        "head_joint_states": "/head/joint_states",
        "arm_joint_states": "/follower/joint_states",
    }
    assert cfg.qos.model_dump() == {"depth": 10, "rgb": 5}
    assert cfg.camera_frame == "d435_color_optical_frame"
    assert cfg.floor.cloud_topic == "/head/d435/points"
    assert cfg.floor.sensor_frame == "d435_depth_optical_frame"
    assert cfg.selection.model_dump() == {
        "still_window_s": 0.5,
        "still_rgb_mad": 2.0,
        "still_depth_mad": 0.01,
        "skip_start_s": 2.0,
    }
    assert cfg.limits.model_dump() == {
        "plane_max_bias_mm": 10.0,
        "plane_max_spread_mm": 50.0,
        "min_views": 12,
        "intrinsics_max_rms_px": 0.5,
        "intrinsics_max_focal_diff_pct": 1.0,
        "intrinsics_max_principal_diff_px": 5.0,
    }
    assert not {"head_camera_config_file", "default_lens_position"} & set(cfg.model_dump())
    # An apply restarts the D435 consumers of the applied head chain only (design D10).
    assert cfg.restart_units == ["ros2-head_rgbd_cpp.service", "ros2-head_state_publisher.service"]


def key_path(data: dict) -> list[str]:
    """The keys from the top of a one-key-per-level mapping down to its leaf key."""
    section, value = next(iter(data.items()))
    return [section, next(iter(value))] if isinstance(value, dict) else [section]


@pytest.mark.parametrize(
    "data",
    [
        {"topics": {"amplitude": "/tof/amplitude/image_raw"}},
        {"topics": {"tof_info": "/tof/camera_info"}},
        {"qos": {"tof_depth": 10}},
        {"qos": {"rgb_depth": 5}},
        {"head_camera_config_file": "/etc/ros2/head_camera_cpp/config.yaml"},
        {"default_lens_position": 1.0},
        *({"selection": {key: value}} for key, value in RETIRED_SELECTION_KEYS.items()),
        *({"limits": {key: value}} for key, value in RETIRED_LIMIT_KEYS.items()),
    ],
    ids=lambda data: ".".join(key_path(data)),
)
def test_tof_keys_are_rejected(data: dict) -> None:
    """ToF, amplitude, ToF-era fit and focus-lock keys are refused, naming the key."""
    with pytest.raises(ValidationError, match=key_path(data)[-1]):
        NodeConfig.model_validate(data)


def test_client_yml_arm_sequence_and_board_mount_equal_defaults() -> None:
    """The deployed config block carries the shipped arm sequence and the nominal board mount, key for key."""
    block = client_block()
    defaults = NodeConfig().model_dump(mode="json", exclude=LOADED_BODY)
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
        "qos:\n  depth: 0\n",
        "qos:\n  rgb: 0\n",
        "selection:\n  still_depth_mad: -0.1\n",
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
    assert floor.cloud_topic == "/head/d435/points"
    assert floor.sensor_frame == "d435_depth_optical_frame"
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
        "floor:\n  tilt_deg: [12, 45.5]\n",
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
    path.write_text("floor:\n  tilt_deg: [-30, 45]\n")
    assert load_config(path).floor.tilt_deg == [-30, 45]


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
    assert cfg.fit.board == ARM_BOARD and cfg.board == FULL_BOARD


@pytest.mark.parametrize("key", ["window_s", "rgb_mad", "joint_still_rad", "max_joint_dt_s", "timeout_s"])
def test_still_values_must_be_positive(key: str) -> None:
    with pytest.raises(ValidationError, match=key):
        NodeConfig.model_validate({"still": {key: 0}})


def test_board_roles_default_full_and_arm() -> None:
    cfg = NodeConfig()
    assert cfg.board == FULL_BOARD and cfg.fit.board == ARM_BOARD and cfg.fit.plate == ARM_BOARD


def test_board_id_ranges_must_not_overlap() -> None:
    clash = ARM_BOARD.model_copy(update={"ids_offset": 5})
    with pytest.raises(ValidationError, match="charuco-6x5-40mm.*charuco-arm-6x5-28mm"):
        NodeConfig.model_validate({"fit": {"board": clash.model_dump()}})
    with pytest.raises(ValidationError, match="charuco-6x5-40mm.*charuco-arm-6x5-28mm"):
        NodeConfig.model_validate({"fit": {"plate": clash.model_dump()}})


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
    assert 0.0 <= seq.insert_gripper_rad <= seq.pose_check.max_roll_gripper_rad
    assert seq.max_roll_delta_rad == 1.2
    assert seq.max_flex_delta_rad == 0.4
    assert seq.max_big_delta_rad == 0.35
    assert seq.speed_scale == 0.1  # half the 0.2 of the jaw-mount set: the board on a stick is fragile
    assert load_config(SHIPPED).arm_sequence.speed_scale == 0.1
    assert seq.settle_s == 1.0
    assert seq.confirm_timeout_s == 600.0
    assert seq.head_offsets_deg == [(0, 0), (10, 0), (-10, 0), (0, -3)]
    assert seq.pose_check == PoseCheckConfig()
    assert seq.provisional_margin_m == 0.05
    assert len(seq.tier0_poses) >= 8 and len(seq.tier1_poses) >= 8 and len(seq.tier2_poses) >= 12
    assert ArmSequenceConfig() == seq


def test_arm_sequence_board_margin() -> None:
    seq = NodeConfig().arm_sequence
    assert seq.board_margin_m(nominal_mount=True) == pytest.approx(0.005 + 0.05)
    assert seq.board_margin_m(nominal_mount=False) == pytest.approx(0.005)


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
        (lambda: sequence_with(insert_gripper_rad=0.81), r"insert_gripper_rad.*max_roll_gripper_rad"),
        (lambda: sequence_with(head_offsets_deg=[]), r"head_offsets_deg"),
        (lambda: sequence_with(handle_squeeze_rad=0.08), r"handle_squeeze_rad.*below handle_contact_rad"),
        (lambda: sequence_with(handle_squeeze_rad=0.0), r"handle_squeeze_rad"),
        (
            lambda: sequence_with(handle_squeeze_rad=0.03, grip_converge_tolerance_rad=0.03),
            r"handle_squeeze_rad.*below grip_converge_tolerance_rad",
        ),
        (lambda: sequence_with(grip_converge_tolerance_rad=0.0), r"grip_converge_tolerance_rad"),
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
        "squeeze-past-contact",
        "squeeze-zero",
        "squeeze-at-converge-tolerance",
        "converge-tolerance-zero",
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


def test_fit_mounts_are_the_cad_seed() -> None:
    meta, fit = json.loads(ARM_JSON.read_text()), NodeConfig().fit
    assert fit.board_mount.xyz_init == pytest.approx(meta["board_mount"]["xyz_init"], abs=1e-6)
    assert fit.board_mount.rpy_init == pytest.approx(meta["board_mount"]["rpy_init"], abs=1e-6)
    assert fit.plate_mount.rpy == fit.plate_mount.rpy_init == fit.board_mount.rpy_init
    assert fit.plate_mount.xyz_init == fit.board_mount.xyz_init
    assert fit.plate_mount.tool_in_board_m == pytest.approx(meta["handle_in_board"]["tool_point_m"], abs=1e-6)
    assert fit.board_mount.outline_margin_m == 0.01
    assert fit.board_mount == load_config(SHIPPED).fit.board_mount
    assert "CAD seed from the handle" in SHIPPED.read_text()


def test_board_handle_boxes_are_the_handle_cad() -> None:
    """The post, its gusset and the tab of the edge handle metadata (user m540, handle_envelope_mm), boxes in the board
    frame (pattern at z = 0, back at z = T): on the pattern side (z < 0) and off the plate outline past its bottom
    edge; the post box rails-wide along board y, from the nominal bed face to the taller rail top along board x, from
    the tab top up neck plus grip; the tab box from the plate back to the tab top."""
    meta = json.loads(ARM_JSON.read_text())
    envelope = meta["handle_envelope_mm"]
    handle = next(b for b in yaml.safe_load(MARKERS_YAML.read_text())["boards"] if b["name"] == ARM_BOARD.name)[
        "handle"
    ]
    boxes = NodeConfig().fit.board_mount.handle_boxes_mm
    assert boxes == load_config(SHIPPED).fit.board_mount.handle_boxes_mm
    expected = [envelope["post"], envelope["gusset"], envelope["tab"]]
    assert len(boxes) == len(expected)
    for box, want in zip(boxes, expected, strict=True):
        assert [*box[0], *box[1]] == pytest.approx([*want[0], *want[1]], abs=1e-6)
        assert box[0][2] < 0.0  # the pattern side
        assert box[0][1] >= ARM_BOARD.squares_y * ARM_BOARD.square_mm  # past the bottom edge of the squares
    post, _, tab = boxes
    (x0, y0, z0), (x1, y1, z1) = post
    assert y1 - y0 == pytest.approx(handle["channel_end_mm"] + 2 * handle["rail_width_mm"], abs=1e-6)
    top = max(handle["thickness_tip_mm"], handle["thickness_end_mm"]) + handle["rail_height_mm"]
    assert x1 - x0 == pytest.approx(top + handle["clamp_clearance_mm"], abs=1e-6)
    tab_top = ARM_BOARD.thickness_mm - handle["tab_thickness_mm"]
    assert z1 == pytest.approx(tab_top, abs=1e-6)
    assert z1 - z0 == pytest.approx(handle["neck_mm"] + handle["length_mm"], abs=1e-6)
    assert tab[0][2] == pytest.approx(tab_top, abs=1e-6) and tab[1][2] == pytest.approx(ARM_BOARD.thickness_mm)


def test_arm_sequence_grip_defaults_come_from_the_handle() -> None:
    meta, seq = json.loads(ARM_JSON.read_text()), NodeConfig().arm_sequence
    assert seq.handle_contact_rad == pytest.approx(meta["grip"]["contact_rad"], abs=1e-4)
    assert seq.grip_rad == pytest.approx(meta["grip"]["grip_rad"], abs=1e-4)
    assert seq.handle_squeeze_rad == 0.02
    assert seq.grip_converge_tolerance_rad == 0.03
    assert seq.board_mass_g == pytest.approx(meta["mass_g"], abs=0.5)
    assert seq.insert_gripper_rad == 0.4


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


def test_ground_board_defaults() -> None:
    ground = NodeConfig().ground_board
    assert ground.pan_offsets_deg == [-24, -12, 0, 12, 24]
    assert ground.tilt_offsets_deg == [-12, -6, 0, 6, 12]
    assert ground.min_tilt_deg == 0.0
    assert ground.max_tilt_deg == 45.0
    assert ground.image_margin_px == 10.0
    assert ground.frames_per_pose == 3
    assert ground.min_frames_per_pose == 2
    assert ground.min_corners == 12
    assert ground.home_min_corners == 16
    assert ground.detect_upscale == 2
    assert ground.frame_timeout_s == 3.0
    assert ground.max_placements == 3
    assert ground.placement_wait_s == 1800.0
    assert ground.fit == GroundBoardFitConfig()
    # The home pose is mcp_server's head home (config.head home_pan_rad 0, home_tilt_rad 0.5236).
    assert (ground.fit.home_pan_deg, ground.fit.home_tilt_deg) == (0.0, 30.0)
    verify = ground.verify
    assert verify.state_topic == "/ground/state"
    assert verify.poses_deg == [(0.0, 30.0), (40.0, 20.0), (-40.0, 25.0)]
    assert verify.settle_s == 3.0
    assert verify.window_s == 15.0
    assert verify.min_samples == 10
    assert verify.sample_timeout_s == 60.0
    # docs/runbooks/head-floor-fit.md acceptance checks.
    assert verify.max_height_m == 0.01
    assert verify.max_plane_vs_base_deg == 0.6
    assert verify.max_obstacle_points == 10
    assert verify.max_drop_points == 0


@pytest.mark.parametrize(
    "text",
    [
        "ground_board:\n  pan_offsets_deg: []\n",
        "ground_board:\n  tilt_offsets_deg: []\n",
        "ground_board:\n  max_tilt_deg: 46\n",
        "ground_board:\n  min_tilt_deg: -31\n",
        "ground_board:\n  min_tilt_deg: 30\n  max_tilt_deg: 20\n",
        "ground_board:\n  frames_per_pose: 0\n",
        "ground_board:\n  frames_per_pose: 2\n  min_frames_per_pose: 3\n",
        "ground_board:\n  min_corners: 5\n",
        "ground_board:\n  home_min_corners: 21\n",
        "ground_board:\n  detect_upscale: 0\n",
        "ground_board:\n  max_placements: 0\n",
        "ground_board:\n  verify:\n    poses_deg: []\n",
        "ground_board:\n  verify:\n    poses_deg: [[0, 50]]\n",
        "ground_board:\n  verify:\n    poses_deg: [[95, 20]]\n",
        "ground_board:\n  verify:\n    min_samples: 0\n",
        "ground_board:\n  fit:\n    bounds:\n      max_mount_move_mm: 0\n",
        "ground_board:\n  fit:\n    home_tilt_deg: 50\n",
        "ground_board:\n  fit:\n    home_pan_deg: 95\n",
        "ground_board:\n  bogus: 1\n",
    ],
)
def test_ground_board_out_of_range_rejected(tmp_path: Path, text: str) -> None:
    path = tmp_path / "c.yaml"
    path.write_text(text)
    with pytest.raises(ValidationError):
        load_config(path)
