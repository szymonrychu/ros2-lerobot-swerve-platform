"""Tests of the FK pose check of the arm with the board on the gripper (head_rgbd_calibration/arm_pose_check.py)."""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml
from ros2_common.head_rgbd_calibration.arm_pose_check import (
    BodyConfig,
    BoxObstacle,
    CylinderObstacle,
    PoseCheckConfig,
    PoseChecker,
    head_envelope,
)
from ros2_common.head_rgbd_calibration.chain import UrdfChain, arm_in_robot_base, pose_from_xyz_rpy
from ros2_common.head_rgbd_calibration.charuco import board_outline
from ros2_common.head_rgbd_calibration.config import ARM_JOINT_NAMES, BoardMountConfig, HeadArmFitConfig
from ros2_common.head_rgbd_calibration.urdf_text import Origin, rewrite_joint_origin
from scipy.optimize import brentq

ROOT = Path(__file__).resolve().parents[1]
URDF_DIR = ROOT / "nodes" / "web_ui" / "urdf"
HEAD_URDF = (URDF_DIR / "head.urdf").read_text()
ARM_URDF = (URDF_DIR / "so101_arm.urdf").read_text()
CLIENT_YML = ROOT / "ansible" / "group_vars" / "client.yml"
HEAD_LINKS = ("head_post", "head_pan_base", "head_pan_link", "head_link")
HEAD_POINT_LINKS = ("head_camera_link", "tof_link")
WIDENED_TILT = (-0.5236, 1.0472)
TEST_SAMPLE_DEG = 5.0  # finer than the envelope's own 15 deg grid
RIM_POINTS = 180
EPS_M = 1e-9
# Board on the gripper for the scenarios: in the gripper_link x/y plane 12 cm beyond gripper_link along -z (2 cm past
# the tool point), centred on the gripper y axis and reaching up along gripper x (up when the gripper is level).
TEST_MOUNT = BoardMountConfig(xyz_init=(0.0, -0.10, -0.12), rpy_init=(0.0, 0.0, 0.0), outline_margin_m=0.01)
ZERO = dict.fromkeys(ARM_JOINT_NAMES, 0.0)
# Level gripper, tool 0.45 m forward of base_link, at 0.12 m and 0.20 m above the floor (solved by FK on the
# shipped arm URDF with the default joint offsets; test_roll_rules pins the heights).
LEVEL_TOOL_12CM = {**ZERO, "shoulder_lift": 1.0475, "elbow_flex": 0.3074, "wrist_flex": -1.3122}
LEVEL_TOOL_20CM = {**ZERO, "shoulder_lift": 0.4565, "elbow_flex": 0.5573, "wrist_flex": -0.971}
PATH_FRACTION = "at path fraction "
# The test board turned upside down under the gripper (hanging below it when the gripper is level).
HANGING_MOUNT = BoardMountConfig(xyz_init=(0.0, 0.10, -0.12), rpy_init=(0.0, 0.0, math.pi), outline_margin_m=0.01)
FOREARM_PAN_RAD = -1.4  # shoulder_pan turning the level forearm through the head pan axis


def fit_with(mount: BoardMountConfig = TEST_MOUNT, offsets: dict[str, float] | None = None) -> HeadArmFitConfig:
    """Default fit config with the given board mount (and joint offsets)."""
    fit = HeadArmFitConfig(board_mount=mount)
    if offsets is not None:
        fit = fit.model_copy(update={"joint_offsets_rad": offsets})
    return fit


def checker(fit: HeadArmFitConfig | None = None, cfg: PoseCheckConfig | None = None) -> PoseChecker:
    return PoseChecker(ARM_URDF, HEAD_URDF, fit or fit_with(), cfg or PoseCheckConfig())


def arm_pose(link: str, joints: dict[str, float], fit: HeadArmFitConfig) -> np.ndarray:
    """4x4 pose of an arm link in base_link, FK straight from the URDF (independent of the checker)."""
    chain = UrdfChain.from_text(ARM_URDF)
    base = arm_in_robot_base(fit.base_in_base_link.model_dump())
    return base @ chain.transform(link, joints, offsets=fit.joint_offsets_rad)


def mount_with_corner_at(joints: dict[str, float], point: np.ndarray) -> BoardMountConfig:
    """Board mount (no rotation) that puts the first board outline corner at `point` (base_link) for `joints`."""
    fit = fit_with()
    corner = board_outline(fit.board, TEST_MOUNT.outline_margin_m)[0]
    gripper = arm_pose("gripper_link", joints, fit)
    local = np.linalg.solve(gripper, np.append(point, 1.0))[:3] - corner
    return BoardMountConfig(xyz_init=tuple(float(v) for v in local), rpy_init=(0.0, 0.0, 0.0), outline_margin_m=0.01)


def rules(entries: list[str]) -> set[str]:
    return {entry.split(":", 1)[0] for entry in entries}


def with_tilt_limits(urdf: str, lower: float, upper: float) -> str:
    root = ET.fromstring(urdf)
    limit = next(j for j in root.findall("joint") if j.get("name") == "head_tilt").find("limit")
    limit.set("lower", str(lower))
    limit.set("upper", str(upper))
    return ET.tostring(root, encoding="unicode")


def visual_points(urdf: str) -> dict[str, list[np.ndarray]]:
    """Points of every head visual in its link frame: box corners and dense cylinder rims (the test's own parser)."""
    out: dict[str, list[np.ndarray]] = {}
    for link in ET.fromstring(urdf).findall("link"):
        for visual in link.findall("visual"):
            origin = visual.find("origin")
            xyz = tuple(float(v) for v in origin.get("xyz").split())
            rpy = tuple(float(v) for v in origin.get("rpy").split())
            pose = pose_from_xyz_rpy(xyz, rpy)
            geometry = visual.find("geometry")
            box, cyl = geometry.find("box"), geometry.find("cylinder")
            if box is not None:
                half = np.array([float(v) for v in box.get("size").split()]) / 2
                local = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half
            else:
                radius, length = float(cyl.get("radius")), float(cyl.get("length"))
                angles = np.linspace(0.0, 2 * np.pi, RIM_POINTS, endpoint=False)
                ring = np.stack([radius * np.cos(angles), radius * np.sin(angles)], axis=1)
                local = np.vstack([np.column_stack([ring, np.full(len(ring), z)]) for z in (-length / 2, length / 2)])
            placed = (pose @ np.column_stack([local, np.ones(len(local))]).T).T[:, :3]
            out.setdefault(link.get("name"), []).append(placed)
    return out


def head_points(urdf: str, pan: float, tilt: float) -> dict[str, np.ndarray]:
    """Head visual points (and the camera and ToF link origins) in base_link at one pan/tilt."""
    chain = UrdfChain.from_text(urdf)
    positions = {"head_pan": pan, "head_tilt": tilt}
    out = {}
    for link, parts in visual_points(urdf).items():
        pose = chain.transform(link, positions)
        local = np.vstack(parts)
        out[link] = (pose @ np.column_stack([local, np.ones(len(local))]).T).T[:, :3]
    for link in HEAD_POINT_LINKS:
        out[link] = chain.transform(link, positions)[None, :3, 3]
    return out


def joint_samples(urdf: str, joint: str) -> np.ndarray:
    lower, upper = UrdfChain.from_text(urdf).limits(joint)
    count = math.ceil(math.degrees(upper - lower) / TEST_SAMPLE_DEG)
    return np.linspace(lower, upper, count + 1)


def assert_contains_head(urdf: str, envelope: CylinderObstacle) -> None:
    pans, tilts = joint_samples(urdf, "head_pan"), joint_samples(urdf, "head_tilt")
    assert len(tilts) > 2 and len(pans) > 2
    for pan in pans:
        for tilt in tilts:
            points = head_points(urdf, float(pan), float(tilt))
            assert set(HEAD_LINKS + HEAD_POINT_LINKS) <= set(points)
            for link, pts in points.items():
                radial = np.hypot(pts[:, 0] - envelope.x, pts[:, 1] - envelope.y)
                assert radial.max() <= envelope.radius + EPS_M, (link, pan, tilt, radial.max(), envelope.radius)
                assert pts[:, 2].max() <= envelope.z_max + EPS_M, (link, pan, tilt)


def test_body_defaults_equal_mcp_server_body() -> None:
    nodes = yaml.safe_load(CLIENT_YML.read_text())["ros2_nodes"]
    node = next(n for n in nodes if n["name"] == "mcp_server")
    body = yaml.safe_load(node["config"])["body"]
    assert BodyConfig.model_validate(body) == BodyConfig()
    assert PoseCheckConfig().body == BodyConfig()
    assert [b.name for b in BodyConfig().boxes] == ["frame_left", "frame_right", "crossbar"]
    assert [c.name for c in BodyConfig().cylinders] == ["wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr", "lidar"]


@pytest.mark.parametrize(
    "cfg",
    [PoseCheckConfig(), PoseCheckConfig(head_radius_m=0.01, head_height_m=0.0)],
    ids=["defaults", "grown"],
)
def test_head_envelope_contains_head_visuals(cfg: PoseCheckConfig) -> None:
    mount = UrdfChain.from_text(HEAD_URDF).origin("head_mount_joint")[0]
    crossbar = next(b for b in cfg.body.boxes if b.name == "crossbar")
    envelope = head_envelope(HEAD_URDF, cfg)
    assert (envelope.x, envelope.y) == pytest.approx(mount[:2])
    assert envelope.z_min == pytest.approx(crossbar.center[2] + crossbar.size[2] / 2)
    assert envelope.z_max >= mount[2] + cfg.head_height_m - EPS_M
    assert envelope.radius >= cfg.head_radius_m - EPS_M
    assert_contains_head(HEAD_URDF, envelope)
    if cfg == PoseCheckConfig():
        # The defaults come from the shipped head URDF: nothing has to grow there.
        assert envelope.radius == pytest.approx(cfg.head_radius_m)
        assert envelope.z_max == pytest.approx(mount[2] + cfg.head_height_m)

    widened = with_tilt_limits(HEAD_URDF, *WIDENED_TILT)
    assert UrdfChain.from_text(widened).limits("head_tilt") == pytest.approx(WIDENED_TILT)
    assert_contains_head(widened, head_envelope(widened, cfg))

    moved_xyz = (mount[0] - 0.03, mount[1] - 0.05, mount[2] + 0.02)
    moved = rewrite_joint_origin(HEAD_URDF, "head_mount_joint", Origin(moved_xyz, (0.01, -0.02, 0.05)), "test")
    moved_envelope = head_envelope(moved, cfg)
    assert (moved_envelope.x, moved_envelope.y) == pytest.approx(moved_xyz[:2])
    assert moved_envelope.z_max >= moved_xyz[2] + cfg.head_height_m - EPS_M
    assert_contains_head(moved, moved_envelope)


def test_pose_violations_are_named() -> None:
    clear = checker()
    assert clear.check_pose(ZERO, 0.3, rolling=False) == []

    low = dict(ZERO)

    def tool_z(elbow: float) -> float:
        return float(arm_pose("gripper_frame_link", {**ZERO, "elbow_flex": elbow}, fit_with())[2, 3]) - 0.05

    low["elbow_flex"] = brentq(tool_z, 0.5, 1.5)
    assert arm_pose("gripper_frame_link", low, fit_with())[2, 3] == pytest.approx(0.05, abs=1e-6)
    entries = clear.check_pose(low, 0.3, rolling=False)
    tool = [e for e in entries if e.startswith("min_tool_height_m")]
    assert tool and "tool" in tool[0] and "50.0 mm" in tool[0], entries

    lidar = next(c for c in PoseCheckConfig().body.cylinders if c.name == "lidar")
    centre = np.array([lidar.x, lidar.y, (lidar.z_min + lidar.z_max) / 2])
    entries = checker(fit_with(mount_with_corner_at(ZERO, centre))).check_pose(ZERO, 0.3, rolling=False)
    board = [e for e in entries if e.startswith("board_clearance") and "lidar" in e]
    assert board and " mm" in board[0], entries

    through_head = {**ZERO, "shoulder_pan": FOREARM_PAN_RAD}
    entries = clear.check_pose(through_head, 0.3, rolling=False)
    head = [e for e in entries if e.startswith("link_clearance") and "forearm" in e and "head" in e]
    assert head and " mm" in head[0], entries

    fit = fit_with()
    elbow = arm_pose("lower_arm_link", ZERO, fit)[:3, 3]
    wrist = arm_pose("wrist_link", ZERO, fit)[:3, 3]
    on_forearm = checker(fit_with(mount_with_corner_at(ZERO, (elbow + wrist) / 2)))
    entries = on_forearm.check_pose(ZERO, 0.3, rolling=False)
    board_link = [e for e in entries if e.startswith("board_link_clearance") and "forearm" in e]
    assert board_link and " mm" in board_link[0], entries
    assert all(not e.startswith("board_link_clearance") for e in clear.check_pose(ZERO, 0.3, rolling=False))

    # The upper arm stands on the crossbar: never checked against it, but against any other box in the same place.
    tall = BoxObstacle(name="crossbar", center=(0.1, -0.05, 0.12), size=(0.1, 0.1, 0.2))
    other = tall.model_copy(update={"name": "frame_tall"})
    base = PoseCheckConfig().body
    without_crossbar = [b for b in base.boxes if b.name != "crossbar"]
    for box, boxes, reported in ((tall, [*without_crossbar, tall], False), (other, [*base.boxes, other], True)):
        cfg = PoseCheckConfig(body=base.model_copy(update={"boxes": boxes}))
        upper = [e for e in checker(cfg=cfg).check_pose(ZERO, 0.3, rolling=False) if "upper arm to" in e]
        assert bool([e for e in upper if box.name in e]) == reported, upper


def test_roll_rules() -> None:
    fit = fit_with()
    assert arm_pose("gripper_frame_link", LEVEL_TOOL_12CM, fit)[2, 3] == pytest.approx(0.12, abs=1e-3)
    assert arm_pose("gripper_frame_link", LEVEL_TOOL_20CM, fit)[2, 3] == pytest.approx(0.20, abs=1e-3)
    check = checker(fit)

    def rolled(pose: dict[str, float]) -> dict[str, float]:
        return {**pose, "wrist_roll": pose["wrist_roll"] + 0.3}

    entries = check.check_path(LEVEL_TOOL_20CM, rolled(LEVEL_TOOL_20CM), 1.2)
    assert "max_roll_gripper_rad" in rules(entries), entries
    entries = check.check_path(LEVEL_TOOL_12CM, rolled(LEVEL_TOOL_12CM), 0.5)
    assert "roll_min_tool_height_m" in rules(entries), entries
    assert "max_roll_gripper_rad" not in rules(entries)
    assert check.check_path(LEVEL_TOOL_20CM, rolled(LEVEL_TOOL_20CM), 0.5) == []

    assert check.check_pose(LEVEL_TOOL_12CM, 1.2, rolling=False) == []
    assert "max_roll_gripper_rad" in rules(check.check_pose(LEVEL_TOOL_20CM, 1.2, rolling=True))
    assert "roll_min_tool_height_m" in rules(check.check_pose(LEVEL_TOOL_12CM, 0.5, rolling=True))
    assert check.check_pose(LEVEL_TOOL_20CM, 0.5, rolling=True) == []
    # Any wrist_roll difference between start and target is a roll, however small.
    entries = check.check_path(LEVEL_TOOL_12CM, {**LEVEL_TOOL_12CM, "wrist_roll": 0.05}, 1.2)
    assert {"max_roll_gripper_rad", "roll_min_tool_height_m"} <= set(rules(entries)), entries
    assert check.check_path(LEVEL_TOOL_12CM, LEVEL_TOOL_12CM, 1.2) == []


def test_stretched_arm_and_limits() -> None:
    stretched = {**ZERO, "shoulder_lift": 1.9, "elbow_flex": -0.2}
    entries = checker().check_pose(stretched, 0.3, rolling=False)
    assert "stretched_shoulder_lift_max_rad" in rules(entries), entries
    # The same pose with offsets that keep it inside the URDF limits: the stretched rule alone refuses it.
    offsets = {**HeadArmFitConfig().joint_offsets_rad, "shoulder_lift": -0.2}
    entries = checker(fit_with(offsets=offsets)).check_pose(stretched, 0.3, rolling=False)
    assert "stretched_shoulder_lift_max_rad" in rules(entries), entries
    assert not any(e.startswith("joint_limits") and "shoulder_lift" in e for e in entries)
    folded = {**stretched, "elbow_flex": 0.2}
    assert "stretched_shoulder_lift_max_rad" not in rules(checker().check_pose(folded, 0.3, rolling=False))

    over = {**ZERO, "wrist_flex": 1.7}  # urdf 1.7988 > 1.65806
    entries = checker().check_pose(over, 0.3, rolling=False)
    limits = [e for e in entries if e.startswith("joint_limits")]
    assert limits and "wrist_flex" in limits[0], entries
    entries = checker().check_pose(ZERO, 1.9, rolling=False)  # gripper upper limit 1.74533
    assert any(e.startswith("joint_limits") and "gripper" in e for e in entries), entries
    with pytest.raises(ValueError, match="elbow_flex"):
        checker().check_pose({k: v for k, v in ZERO.items() if k != "elbow_flex"}, 0.3, rolling=False)


def test_path_check_catches_mid_path_contact() -> None:
    check = checker()
    start = {**LEVEL_TOOL_20CM, "shoulder_pan": 0.2}
    target = {**LEVEL_TOOL_20CM, "shoulder_pan": -0.3, "wrist_roll": 0.3}
    assert check.check_pose(start, 0.3, rolling=False) == []
    assert check.check_pose(target, 0.3, rolling=False) == []
    assert check.check_path(start, target, 0.3) == []

    # The board hangs below the level gripper: rolled to either side it lies flat clear of the floor, but the
    # joint-linear roll from one side to the other swings it through straight down, into the floor.
    hanging = checker(fit_with(HANGING_MOUNT))
    side_a = {**LEVEL_TOOL_20CM, "wrist_roll": 1.6}
    side_b = {**LEVEL_TOOL_20CM, "wrist_roll": -1.6}
    assert hanging.check_pose(side_a, 0.3, rolling=True) == []
    assert hanging.check_pose(side_b, 0.3, rolling=True) == []
    entries = hanging.check_path(side_a, side_b, 0.3)
    floor = [e for e in entries if e.startswith("min_board_height_m")]
    assert floor and len(floor) == len(entries), entries
    fractions = [float(e.rsplit(PATH_FRACTION, 1)[1]) for e in floor]
    assert all(0.0 < f < 1.0 for f in fractions), floor
    assert 0.5 in fractions
    # Samples are spaced so no joint steps more than path_step_rad: 3.2 rad of wrist_roll in 64 steps.
    steps = math.ceil(3.2 / PoseCheckConfig().path_step_rad)
    assert all(abs(f * steps - round(f * steps)) < 0.07 for f in fractions)
    assert len(set(fractions)) > 1


def test_board_margin_applies_to_board_rules_only() -> None:
    fit = fit_with(mount_with_corner_at(ZERO, np.array([0.40, -0.05, 0.35])))
    plain = checker(fit)
    assert plain.board_margin_m == PoseCheckConfig().margin_m
    board_clear = float(plain.obstacles.distances(plain.board_points(ZERO)).min())
    assert board_clear > PoseCheckConfig().margin_m
    base_entries = plain.check_pose(ZERO, 0.3, rolling=False)
    assert not [e for e in base_entries if e.startswith("board_")], base_entries

    wide = PoseChecker(ARM_URDF, HEAD_URDF, fit, PoseCheckConfig(), board_margin_m=board_clear + 0.01)
    entries = wide.check_pose(ZERO, 0.3, rolling=False)
    board = [e for e in entries if e.startswith("board_clearance")]
    assert board and all(f"board_margin_m {(board_clear + 0.01) * 1000:.1f} mm" in e for e in board), entries
    assert [e for e in entries if not e.startswith("board_")] == base_entries
    with pytest.raises(ValueError, match="board_margin_m"):
        PoseChecker(ARM_URDF, HEAD_URDF, fit, PoseCheckConfig(), board_margin_m=-0.01)
