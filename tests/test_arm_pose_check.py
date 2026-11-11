"""Tests of the FK pose check of the arm with the board on the gripper (head_rgbd_calibration/arm_pose_check.py)."""

import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml
from pydantic import ValidationError
from ros2_common.camera_geometry import CameraIntrinsics
from ros2_common.head_rgbd_calibration.arm_pose_check import (
    FOV_RULE,
    HEAD_CLEARANCE_RULE,
    HEAD_REGION_STEP_RAD,
    MIN_Z_RULE,
    BodyConfig,
    BoxObstacle,
    CylinderObstacle,
    HeadRegion,
    ObstacleSet,
    OrientedBox,
    PoseCheckConfig,
    PoseChecker,
    board_points_local,
    handle_box_points_local,
    head_envelope,
    head_region_boxes,
    head_shapes,
    mounted,
    point_segment_distances,
)
from ros2_common.head_rgbd_calibration.board_lattice import box_lattice, lattice_miss_bound, plate_lattice
from ros2_common.head_rgbd_calibration.chain import (
    UrdfChain,
    arm_in_robot_base,
    pose_from_xyz_rpy,
    xyz_rpy_from_pose,
)
from ros2_common.head_rgbd_calibration.charuco import board_corners_3d, board_outline
from ros2_common.head_rgbd_calibration.config import ARM_JOINT_NAMES, BoardMountConfig, HeadArmFitConfig
from ros2_common.head_rgbd_calibration.head_arm_fit import aim_head, board_centre
from ros2_common.head_rgbd_calibration.urdf_text import Origin, rewrite_joint_origin
from scipy.optimize import brentq

ROOT = Path(__file__).resolve().parents[1]
URDF_DIR = ROOT / "nodes" / "web_ui" / "urdf"
HEAD_URDF = (URDF_DIR / "head.urdf").read_text()
# The robot body mcp_server checks against (the single definition in the shared arm_collision package).
ROBOT_BODY_YAML = ROOT / "shared" / "arm_collision" / "arm_collision" / "robot_body.yaml"
ARM_URDF = (URDF_DIR / "so101_arm.urdf").read_text()
HEAD_LINKS = ("head_post", "head_pan_base", "head_pan_link", "head_link")
HEAD_POINT_LINKS = ("d435_depth_optical_frame", "d435_color_optical_frame")  # D435 frames without a visual
WIDENED_TILT = (-0.5236, 0.7854)  # -30..+45 deg, the reprinted mount's range (runbook head-arm-calibration Hardware)
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
COLOR_FRAME = "d435_color_optical_frame"
# A very wide 640x360 camera (145 deg across) for the min-Z cases, and the nominal D435 colour at 640x360 (69 deg
# across, the datasheet colour FOV, as nodes/marker_detector/marker_detector/synthetic.py) for the FOV cases.
WIDE_CAMERA = CameraIntrinsics(width=640, height=360, fx=100.0, fy=100.0, cx=320.0, cy=180.0)
D435_COLOR = CameraIntrinsics.from_hfov(640, 360, 69.0)
SPACING_M = PoseCheckConfig().board_spacing_m
OLD_GRID = 5  # the former fixed plate lattice: 5 points per plate edge, about 47 x 40 mm cells
OLD_BOX_POINTS = 3  # the former fixed handle box lattice: 3 points per axis (50 mm apart along the 100 mm tab)
CLOSE_M = 0.002  # an obstacle this close to the board, well inside the 5 mm margin
# The edge handle's tab (board frame, mm, the CAD box): 100 mm along the plate edge, 10 mm thick.
TAB_BOX_MM = ((34.0, 145.0, -6.0), (134.0, 200.3, 4.0))
ELBOW_GAP_M = 0.04  # the plate starts this far past the elbow along the forearm: the upper arm keeps clear of it
PROBE_SIZE_M = 0.0005  # a body box small enough to sit in the middle of one lattice cell
PROBE_GAP_M = 0.0045  # probe to plate back: inside the 5 mm margin, beyond it from every lattice point


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


def mount_facing_camera(
    joints: dict[str, float], depth_m: float, lateral_m: float = 0.0, pan: float = 0.0, tilt: float = 0.0
) -> BoardMountConfig:
    """Board mount that puts the board fronto-parallel to the D435 colour camera at a head pose: its corner grid
    centred `lateral_m` along the optical x axis, every corner at depth `depth_m` (FK and head chain straight from the
    URDFs, independent of the checker)."""
    fit = fit_with()
    board_in_camera = np.eye(4)
    board_in_camera[:3, 3] = np.array([lateral_m, 0.0, depth_m]) - board_centre(fit.board)
    head = UrdfChain.from_text(HEAD_URDF)
    camera = head.transform(COLOR_FRAME, {"head_pan": pan, "head_tilt": tilt})
    mount = np.linalg.solve(arm_pose("gripper_link", joints, fit), camera @ board_in_camera)
    xyz, rpy = xyz_rpy_from_pose(mount)
    return BoardMountConfig(xyz_init=xyz, rpy_init=rpy, outline_margin_m=0.01)


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
    body = yaml.safe_load(ROBOT_BODY_YAML.read_text())
    assert BodyConfig.model_validate(body) == BodyConfig()
    assert PoseCheckConfig().body == BodyConfig()
    assert [b.name for b in BodyConfig().boxes] == ["frame_left", "frame_right", "crossbar", "head_arm"]
    assert [c.name for c in BodyConfig().cylinders] == [
        "wheel_fl",
        "wheel_fr",
        "wheel_rl",
        "wheel_rr",
        "lidar",
        "head_post",
        "head",
    ]


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


def test_board_beyond_d435_min_z() -> None:
    """Every board corner keeps d435_min_z_m (150 mm at 640x360, design D2) plus d435_min_z_margin_m (50 mm) of depth
    from d435_color_optical_frame; a nearer board is named with its nearest corner depth."""
    cfg = PoseCheckConfig()
    assert (cfg.d435_min_z_m, cfg.d435_min_z_margin_m) == (0.15, 0.05)
    for depth, inside in ((0.19, True), (0.199, True), (0.201, False), (0.45, False)):
        entries = checker(fit_with(mount_facing_camera(ZERO, depth))).check_view(ZERO, 0.0, 0.0, WIDE_CAMERA)
        assert (MIN_Z_RULE in rules(entries)) == inside, (depth, entries)
        assert FOV_RULE not in rules(entries), entries
    entries = checker(fit_with(mount_facing_camera(ZERO, 0.19))).check_view(ZERO, 0.0, 0.0, WIDE_CAMERA)
    assert entries == [f"{MIN_Z_RULE}: board corner depth 190.0 mm < 200.0 mm (min-Z 150.0 mm + margin 50.0 mm)"]
    # The depth is taken at the head pose of the view: a board 0.45 m out on the optical axis at pan 0.4 rad passes
    # there, and with the head turned to pan -1.2 rad it lies beside the camera, inside min-Z.
    turned = checker(fit_with(mount_facing_camera(ZERO, 0.45, pan=0.4, tilt=0.2)))
    assert turned.check_view(ZERO, 0.4, 0.2, WIDE_CAMERA) == []
    assert MIN_Z_RULE in rules(turned.check_view(ZERO, -1.2, 0.2, WIDE_CAMERA))
    # A board behind the camera is inside min-Z and outside the image.
    behind = checker(fit_with(mount_facing_camera(ZERO, -0.3))).check_view(ZERO, 0.0, 0.0, WIDE_CAMERA)
    assert rules(behind) == {MIN_Z_RULE, FOV_RULE}, behind
    # The limit is the config: no margin lets 0.19 m pass.
    relaxed = PoseCheckConfig(d435_min_z_margin_m=0.0)
    near = checker(fit_with(mount_facing_camera(ZERO, 0.19)), relaxed)
    assert near.check_view(ZERO, 0.0, 0.0, WIDE_CAMERA) == []


def test_board_outside_the_colour_fov() -> None:
    """Every board corner projects inside the colour image of the camera_info; the arm pose does not matter."""
    for joints in (ZERO, LEVEL_TOOL_20CM):
        centred = checker(fit_with(mount_facing_camera(joints, 0.45)))
        assert centred.check_view(joints, 0.0, 0.0, D435_COLOR) == []
        # 0.27 m to the side (the 28 mm ARM board, corner columns 0.028 m apart): the far column of 4 corners (0.326 m
        # out) passes the 34.5 deg half-angle (0.31 m at 0.45 m), the next one (0.298 m) does not.
        aside = checker(fit_with(mount_facing_camera(joints, 0.45, lateral_m=0.27)))
        entries = aside.check_view(joints, 0.0, 0.0, D435_COLOR)
        assert rules(entries) == {FOV_RULE}, entries
        assert entries[0].startswith(f"{FOV_RULE}: 4 of 20 board corners outside the 640x360 colour image")
        assert aside.check_view(joints, 0.0, 0.0, WIDE_CAMERA) == []
    # Turning the head away by 30 deg moves the centred board out of the image.
    centred = checker(fit_with(mount_facing_camera(ZERO, 0.45)))
    assert rules(centred.check_view(ZERO, math.radians(30.0), 0.0, D435_COLOR)) == {FOV_RULE}
    # Another camera frame of the head URDF is honoured.
    depth_frame = centred.check_view(ZERO, 0.0, 0.0, D435_COLOR, camera_frame="d435_depth_optical_frame")
    assert depth_frame == []
    assert len(board_corners_3d(fit_with().board)) == 20


def test_board_points_span_the_board_thickness_behind_the_printed_face() -> None:
    fit = fit_with(BoardMountConfig(xyz_init=(0.0, 0.0, 0.0), rpy_init=(0.0, 0.0, 0.0), outline_margin_m=0.01))
    assert fit.board.thickness_mm == 4.0
    local = board_points_local(fit, SPACING_M)
    assert local[:, 2].min() == 0.0 and local[:, 2].max() == pytest.approx(0.004)
    front, back = local[np.isclose(local[:, 2], 0.0)], local[np.isclose(local[:, 2], 0.004)]
    assert len(front) == len(back) == len(local) // 2
    assert np.allclose(back[:, :2], front[:, :2])
    outline = board_outline(fit.board, 0.01)
    assert front[:, :2].min(axis=0) == pytest.approx(outline.min(axis=0)[:2])
    assert front[:, :2].max(axis=0) == pytest.approx(outline.max(axis=0)[:2])


def test_thicker_board_has_less_clearance_than_a_thin_one() -> None:
    mount = mount_with_corner_at(ZERO, np.array([0.40, -0.05, 0.35]))
    thin = fit_with(mount)
    thin.board.thickness_mm = 1.0
    thick = fit_with(mount)
    thick.board.thickness_mm = 40.0
    clear = lambda f: float(checker(f).obstacles.distances(checker(f).board_points(ZERO)).min())  # noqa: E731
    assert clear(thick) < clear(thin)


# A rib-like box behind the plate back and a block off the plate edge (board frame, mm), for the handle box cases.
HANDLE_BOXES_MM = [((70.0, 70.0, 4.0), (76.0, 150.0, 25.0)), ((70.0, 150.0, 0.0), (100.0, 180.0, 30.0))]


def test_handle_boxes_are_board_points_of_the_checker() -> None:
    flat = BoardMountConfig(xyz_init=(0.0, 0.0, 0.0), rpy_init=(0.0, 0.0, 0.0), handle_boxes_mm=HANDLE_BOXES_MM)
    fit = fit_with(flat)
    local = handle_box_points_local(fit, SPACING_M)
    per_box = [
        len(box_lattice(np.array(low) / 1000.0, np.array(high) / 1000.0, SPACING_M)) for low, high in HANDLE_BOXES_MM
    ]
    assert len(local) == sum(per_box)
    for low, high in HANDLE_BOXES_MM:  # every box corner is a checked point
        for corner in np.array(np.meshgrid(*zip(low, high, strict=True))).reshape(3, -1).T / 1000.0:
            assert np.isclose(local, corner).all(axis=1).any(), corner
    assert len(handle_box_points_local(fit_with(flat.model_copy(update={"handle_boxes_mm": []})), SPACING_M)) == 0
    boxed_mount = TEST_MOUNT.model_copy(update={"handle_boxes_mm": HANDLE_BOXES_MM})
    plain, boxed = checker(fit_with()), checker(fit_with(boxed_mount))
    assert len(boxed.board_points(ZERO)) == len(plain.board_points(ZERO)) + sum(per_box)
    assert np.allclose(boxed.board_points(ZERO)[: len(plain.board_points(ZERO))], plain.board_points(ZERO))


def test_a_handle_box_near_the_floor_fails_the_board_height() -> None:
    """The plate alone keeps min_board_height_m; a 10 mm box 0.25 m below the board origin (base_link -z) does not."""
    mount = mount_with_corner_at(LEVEL_TOOL_20CM, np.array([0.45, -0.05, 0.20]))
    fit = fit_with(mount)
    assert not [e for e in checker(fit).check_pose(LEVEL_TOOL_20CM, 0.0, False) if e.startswith("min_board_height_m")]
    board = arm_pose("gripper_link", LEVEL_TOOL_20CM, fit) @ pose_from_xyz_rpy(mount.xyz_init, mount.rpy_init)
    down_mm = board[:3, :3].T @ np.array([0.0, 0.0, -250.0])
    box = (tuple(float(v) for v in down_mm), tuple(float(v) + 10.0 for v in down_mm))
    reaching = checker(fit_with(mount.model_copy(update={"handle_boxes_mm": [box]})))
    entries = reaching.check_pose(LEVEL_TOOL_20CM, 0.0, False)
    assert any(e.startswith("min_board_height_m") for e in entries), entries


def test_handle_boxes_need_min_below_max() -> None:
    with pytest.raises(ValidationError, match="handle_boxes_mm"):
        BoardMountConfig(handle_boxes_mm=[((0.0, 0.0, 4.0), (10.0, 10.0, 4.0))])


def old_grid_points(fit: HeadArmFitConfig) -> np.ndarray:
    """The former plate lattice (outline corners plus OLD_GRID x OLD_GRID points on both faces), board frame (m)."""
    outline = board_outline(fit.board, fit.board_mount.outline_margin_m)
    xs = np.linspace(outline[:, 0].min(), outline[:, 0].max(), OLD_GRID)
    ys = np.linspace(outline[:, 1].min(), outline[:, 1].max(), OLD_GRID)
    front = np.vstack([outline, [[x, y, 0.0] for y in ys for x in xs]])
    return np.vstack([front, front + [0.0, 0.0, fit.board.thickness_mm / 1000.0]])


def old_box_points(low_mm: tuple[float, ...], high_mm: tuple[float, ...]) -> np.ndarray:
    """The former OLD_BOX_POINTS^3 lattice over a handle box, board frame (m)."""
    axes = [np.linspace(lo, hi, OLD_BOX_POINTS) / 1000.0 for lo, hi in zip(low_mm, high_mm, strict=True)]
    return np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, 3)


def old_cell_centre(fit: HeadArmFitConfig) -> np.ndarray:
    """Board-frame (m) centre of a middle cell of the former plate lattice, on the plate back."""
    outline = board_outline(fit.board, fit.board_mount.outline_margin_m)
    low, high = outline.min(axis=0), outline.max(axis=0)
    cell = (high[:2] - low[:2]) / (OLD_GRID - 1)
    return np.array([*(low[:2] + 1.5 * cell), fit.board.thickness_mm / 1000.0])


def board_in_base(fit: HeadArmFitConfig, joints: dict[str, float]) -> np.ndarray:
    """4x4 board pose in base_link (FK straight from the URDF)."""
    mount = fit.board_mount
    return arm_pose("gripper_link", joints, fit) @ pose_from_xyz_rpy(mount.xyz_init, mount.rpy_init)


def forearm_mount(joints: dict[str, float], line_point: np.ndarray, line_axis: np.ndarray) -> BoardMountConfig:
    """A board mount that runs the forearm axis (elbow -> wrist) at `joints` through `line_point` along `line_axis`
    (board frame, m and unit vector), its midpoint at `line_point` (FK straight from the URDF)."""
    fit = fit_with()
    elbow, wrist = forearm_segment(joints, fit)
    along = (wrist - elbow) / np.linalg.norm(wrist - elbow)
    side = np.cross(along, [0.0, 0.0, 1.0])
    side /= np.linalg.norm(side)
    a = np.asarray(line_axis, dtype=float)
    b = np.cross(a, [0.0, 1.0, 0.0] if abs(a[1]) < 0.9 else [1.0, 0.0, 0.0])
    b /= np.linalg.norm(b)
    world = np.column_stack([along, side, np.cross(along, side)])
    local = np.column_stack([a, b, np.cross(a, b)])
    board = np.eye(4)
    board[:3, :3] = world @ local.T
    board[:3, 3] = (elbow + wrist) / 2.0 - board[:3, :3] @ line_point
    xyz, rpy = xyz_rpy_from_pose(np.linalg.solve(arm_pose("gripper_link", joints, fit), board))
    return BoardMountConfig(xyz_init=xyz, rpy_init=rpy, outline_margin_m=0.01)


def forearm_segment(joints: dict[str, float], fit: HeadArmFitConfig) -> tuple[np.ndarray, np.ndarray]:
    """Elbow and wrist positions in base_link (FK straight from the URDF)."""
    return arm_pose("lower_arm_link", joints, fit)[:3, 3], arm_pose("wrist_link", joints, fit)[:3, 3]


def test_board_spacing_never_exceeds_the_margin() -> None:
    """The lattice spacing bounds the clearance a point check can miss (half a cell diagonal), so it may not exceed
    the smallest board margin, margin_m (the margin once the mount is fit-solved)."""
    cfg = PoseCheckConfig()
    assert cfg.board_spacing_m <= cfg.margin_m
    with pytest.raises(ValidationError, match="board_spacing_m"):
        PoseCheckConfig(margin_m=0.004, board_spacing_m=0.005)
    with pytest.raises(ValidationError, match="board_spacing_m"):
        PoseCheckConfig(board_spacing_m=0.0005)


def test_a_body_box_between_the_old_plate_lattice_points_is_caught() -> None:
    """A 4 mm body box 2 mm behind the plate back, at the centre of a cell of the former 5 x 5 lattice: every former
    point keeps more than the margin from it, the margin-spaced lattice does not."""
    joints, fit = ZERO, fit_with(mount_with_corner_at(ZERO, np.array([0.40, -0.05, 0.35])))
    board = board_in_base(fit, joints)
    centre_local = old_cell_centre(fit) + [0.0, 0.0, CLOSE_M + 0.002]
    centre = (board @ np.append(centre_local, 1.0))[:3]
    probe = BoxObstacle(name="probe", center=tuple(centre), size=(0.004, 0.004, 0.004))
    base = PoseCheckConfig().body
    cfg = PoseCheckConfig(body=base.model_copy(update={"boxes": [*base.boxes, probe]}))
    old = old_grid_points(fit) @ board[:3, :3].T + board[:3, 3]
    assert np.abs(old - centre).max(axis=1).min() - 0.002 > cfg.margin_m  # the former lattice kept the margin
    entries = checker(fit, cfg).check_pose(joints, 0.3, rolling=False)
    assert [e for e in entries if e.startswith("board_clearance: board to probe")], entries


def test_a_forearm_passing_between_the_old_plate_lattice_points_is_caught() -> None:
    """The forearm axis along board x, parallel to the plate back and link_radius_m + 2 mm behind it, between two
    former lattice rows: the former points keep more than the margin, the margin-spaced lattice reports it. The plate
    starts past the elbow, so the forearm is the only link it is reported against."""
    cfg = PoseCheckConfig()
    point = old_cell_centre(fit_with()) + [0.0, 0.0, cfg.link_radius_m + CLOSE_M]
    elbow, wrist = forearm_segment(ZERO, fit_with())
    # The plate starts ELBOW_GAP_M past the elbow and runs past the wrist, so only the forearm passes over it.
    point[0] = float(board_outline(fit_with().board, 0.01)[:, 0].min()) + np.linalg.norm(wrist - elbow) / 2.0
    point[0] -= ELBOW_GAP_M
    placed = fit_with(forearm_mount(ZERO, point, np.array([1.0, 0.0, 0.0])))
    board = board_in_base(placed, ZERO)
    elbow, wrist = forearm_segment(ZERO, placed)
    old = old_grid_points(placed) @ board[:3, :3].T + board[:3, 3]
    assert point_segment_distances(old, elbow, wrist).min() - cfg.link_radius_m > cfg.margin_m
    entries = checker(placed, cfg).check_pose(ZERO, 0.3, rolling=False)
    assert [e for e in entries if e.startswith("board_link_clearance: board to forearm")], entries
    assert not [e for e in entries if e.startswith("board_link_clearance: board to upper arm")], entries


def test_a_forearm_passing_between_the_old_tab_lattice_points_is_caught() -> None:
    """The edge handle's 100 mm tab was sampled 50 mm apart: the forearm axis along board z (through the tab's
    thickness), link_radius_m + 2 mm beyond the tab's far face (board y 200.3 mm) and midway between two former
    columns (x 34 and 84 mm), keeps more than the margin from every former point and is reported with the
    margin-spaced lattice."""
    cfg = PoseCheckConfig()
    low, high = TAB_BOX_MM
    point = np.array(
        [(low[0] + (high[0] - low[0]) / 4.0) / 1000.0, high[1] / 1000.0 + cfg.link_radius_m + CLOSE_M, 0.0]
    )
    mount = forearm_mount(ZERO, point, np.array([0.0, 0.0, 1.0]))
    placed = fit_with(mount.model_copy(update={"handle_boxes_mm": [TAB_BOX_MM]}))
    board = board_in_base(placed, ZERO)
    elbow, wrist = forearm_segment(ZERO, placed)
    old = np.vstack([old_grid_points(placed), old_box_points(low, high)]) @ board[:3, :3].T + board[:3, 3]
    assert point_segment_distances(old, elbow, wrist).min() - cfg.link_radius_m > cfg.margin_m
    entries = checker(placed, cfg).check_pose(ZERO, 0.3, rolling=False)
    assert [e for e in entries if e.startswith("board_link_clearance: board to forearm")], entries


def handle_box_at(fit: HeadArmFitConfig, joints: dict[str, float], point: np.ndarray) -> BoardMountConfig:
    """The fit's mount with one 10 mm handle box centred on a base_link point."""
    board = board_in_base(fit, joints)
    centre_mm = np.linalg.solve(board, np.append(point, 1.0))[:3] * 1000.0
    box = (tuple(float(v) - 5.0 for v in centre_mm), tuple(float(v) + 5.0 for v in centre_mm))
    return fit.board_mount.model_copy(update={"handle_boxes_mm": [box]})


def test_a_handle_box_inside_a_body_part_is_refused() -> None:
    """The plate alone clears the body; a handle box in the lidar is refused (not only the floor rule)."""
    fit = fit_with(mount_with_corner_at(ZERO, np.array([0.40, -0.05, 0.35])))
    assert not [e for e in checker(fit).check_pose(ZERO, 0.3, rolling=False) if e.startswith("board_")]
    lidar = next(c for c in PoseCheckConfig().body.cylinders if c.name == "lidar")
    centre = np.array([lidar.x, lidar.y, (lidar.z_min + lidar.z_max) / 2])
    entries = checker(fit_with(handle_box_at(fit, ZERO, centre))).check_pose(ZERO, 0.3, rolling=False)
    assert [e for e in entries if e.startswith("board_clearance: board to lidar")], entries


def test_a_handle_box_inside_the_head_envelope_is_refused() -> None:
    """The plate alone clears the head; a handle box in the head envelope is refused."""
    fit = fit_with(mount_with_corner_at(ZERO, np.array([0.40, -0.05, 0.35])))
    plain = checker(fit)
    assert not [e for e in plain.check_pose(ZERO, 0.3, rolling=False) if e.startswith("board_")]
    head = plain.envelope
    centre = np.array([head.x, head.y, (head.z_min + head.z_max) / 2])
    entries = checker(fit_with(handle_box_at(fit, ZERO, centre))).check_pose(ZERO, 0.3, rolling=False)
    assert [e for e in entries if e.startswith(f"board_clearance: board to {head.name}")], entries


def test_a_body_box_within_the_margin_of_the_plate_but_beyond_it_from_every_lattice_point_is_refused() -> None:
    """A small body box PROBE_GAP_M (4.5 mm) behind the plate back, over the middle of one lattice cell: every lattice
    point keeps more than margin_m from it, the plate itself does not. The board threshold adds the lattice miss
    bound (spacing * sqrt(3) / 2), so the configured margin is kept to the true plate surface."""
    fit = fit_with(mount_with_corner_at(ZERO, np.array([0.40, -0.05, 0.35])))
    cfg = PoseCheckConfig()
    plate, thickness = fit.board, fit.board.thickness_mm / 1000.0
    local = plate_lattice(
        plate.squares_x * plate.square_mm / 1000.0,
        plate.squares_y * plate.square_mm / 1000.0,
        fit.board_mount.outline_margin_m,
        thickness,
        cfg.board_spacing_m,
    )
    xs, ys = np.unique(local[:, 0].round(9)), np.unique(local[:, 1].round(9))
    cell = np.array([xs[len(xs) // 2 : len(xs) // 2 + 2].mean(), ys[len(ys) // 2 : len(ys) // 2 + 2].mean()])
    board = board_in_base(fit, ZERO)
    centre_local = np.array([*cell, thickness + PROBE_GAP_M + PROBE_SIZE_M / 2.0])
    centre = (board @ np.append(centre_local, 1.0))[:3]
    probe = BoxObstacle(name="probe", center=tuple(centre), size=(PROBE_SIZE_M,) * 3)
    probed = PoseCheckConfig(body=cfg.body.model_copy(update={"boxes": [*cfg.body.boxes, probe]}))
    plain = checker(fit, probed)
    distances = plain.obstacles.distances(plain.board_points(ZERO))
    lattice_clear = float(distances[:, plain.obstacles.names.index("probe")].min())
    assert PROBE_GAP_M < cfg.margin_m < lattice_clear < cfg.margin_m + lattice_miss_bound(cfg.board_spacing_m)
    entries = plain.check_pose(ZERO, 0.3, rolling=False)
    assert [e for e in entries if e.startswith("board_clearance: board to probe")], entries


# The head with its pan axis off the mount (the 2026-11-11 ground board fit, rounded): head_pan origin xy, the tilt
# origin xy moved back over the mount, d435_joint x. The camera sits on a ~36 mm lever about the pan axis, so the
# full-range envelope about the mount grows from 75 to 138 mm radius.
PAN_AXIS_XY = (-0.0319, -0.0159)
D435_X_M = 0.0378
SHIPPED_HEAD_LINKS = {"head_post", "head_pan_base", "head_pan_link", "head_link", "d435_link"}
REGION = HeadRegion(pan_rad=(-0.3, 0.25), tilt_rad=(-0.2, 0.4))
DENSE_STEP_RAD = math.radians(0.5)
CUBE = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)
# Near the head: the envelope of the pan-axis head refuses the upper arm (3.5 mm < 5 mm margin, found by a grid search
# over shoulder_pan/shoulder_lift on the shipped arm and the default fit); the real head at the aim keeps far more.
NEAR_HEAD = {**ZERO, "shoulder_pan": -0.05, "shoulder_lift": 0.3, "wrist_flex": -0.5, "wrist_roll": -0.3}
OBSTACLE_ENTRY = re.compile(r"^(\w+: .+? to .+?) clearance ")


def pan_axis_head() -> str:
    """The shipped head with the pan axis moved off the mount, as the 2026-11-11 fit (see PAN_AXIS_XY)."""
    urdf = rewrite_joint_origin(HEAD_URDF, "head_pan", Origin((*PAN_AXIS_XY, 0.0), (0.0, 0.0, 0.0)), "test")
    tilt_xyz = (-PAN_AXIS_XY[0], -PAN_AXIS_XY[1], 0.04)
    urdf = rewrite_joint_origin(urdf, "head_tilt", Origin(tilt_xyz, (0.0, 0.0, 0.0)), "test")
    return rewrite_joint_origin(urdf, "d435_joint", Origin((D435_X_M, 0.0, 0.0), (0.0, 0.0, 0.0)), "test")


def grid_link_poses(chain: UrdfChain, pans: np.ndarray, tilts: np.ndarray) -> dict[str, np.ndarray]:
    """Pose of every head link with geometry at every (pan, tilt) of the grid, (len(pans) * len(tilts), 4, 4).

    Composed from the chain's own transforms: pose(pan, tilt) = pan_child(pan) @ inv(pan_child(0)) @ pose(0, tilt)
    for the links below head_pan (the joints between head_pan and head_tilt are fixed)."""
    pan_child = chain.joints["head_pan"].child
    at_pan = np.array([chain.transform(pan_child, {"head_pan": float(p)}) for p in pans])
    turn = at_pan @ np.linalg.inv(chain.transform(pan_child, {"head_pan": 0.0}))
    out = {}
    for link in SHIPPED_HEAD_LINKS:
        names = chain.joint_names(link)
        at_tilt = np.array([chain.transform(link, {"head_pan": 0.0, "head_tilt": float(t)}) for t in tilts])
        moved = (
            turn[:, None] @ at_tilt[None]
            if "head_pan" in names
            else np.broadcast_to(at_tilt, (len(pans), *at_tilt.shape))
        )
        out[link] = np.ascontiguousarray(moved).reshape(-1, 4, 4)
    return out


def test_oriented_box_distance_matches_hand_values() -> None:
    pose = np.eye(4)
    pose[:3, :3] = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])  # 90 deg about z
    pose[:3, 3] = (1.0, 0.0, 0.0)
    box = OrientedBox(name="probe", pose=pose, half=np.array([0.1, 0.05, 0.05]))
    obstacles = ObstacleSet([], [], [box])
    assert obstacles.names == ["probe"]
    points = np.array([[1.0, 0.15, 0.0], [1.06, 0.0, 0.0], [1.0, 0.0, 0.0]])
    assert obstacles.distances(points)[:, 0] == pytest.approx([0.05, 0.01, -0.05], abs=1e-9)
    # The segment distance agrees with the point distance at the nearest end.
    seg = obstacles.segment_distances(np.array([[1.0, 0.15, 0.0]]), np.array([[1.0, 0.3, 0.0]]))
    assert seg[0, 0] == pytest.approx(0.05, abs=1e-9)


def test_head_shapes_of_the_shipped_head() -> None:
    shapes = head_shapes(HEAD_URDF)
    assert {s.link for s in shapes} == SHIPPED_HEAD_LINKS
    d435 = next(s for s in shapes if s.link == "d435_link")
    assert d435.half == pytest.approx((0.045, 0.0125, 0.0125))
    pan = next(s for s in shapes if s.link == "head_pan_link")
    assert pan.half == pytest.approx((0.02, 0.02, 0.02))  # cylinder r 0.02 length 0.04 as its circumscribing box
    assert pan.local[:3, 3] == pytest.approx((0.0, 0.0, 0.02))


def test_head_shapes_prefer_collision_and_refuse_meshes() -> None:
    both = """<robot name="r"><link name="a">
      <visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry><box size="0.4 0.4 0.4"/></geometry></visual>
      <collision><origin xyz="0.1 0 0" rpy="0 0 0"/><geometry><box size="0.02 0.04 0.06"/></geometry></collision>
      </link><link name="b"><visual><geometry><sphere radius="0.03"/></geometry></visual></link><link name="c"/>
      </robot>"""
    shapes = head_shapes(both)
    assert [s.link for s in shapes] == ["a", "b"]
    assert shapes[0].half == pytest.approx((0.01, 0.02, 0.03))
    assert shapes[0].local[:3, 3] == pytest.approx((0.1, 0.0, 0.0))
    assert shapes[1].half == pytest.approx((0.03, 0.03, 0.03))
    mesh = """<robot name="r"><link name="a"><visual><geometry><mesh filename="a.stl"/></geometry></visual></link>
      </robot>"""
    with pytest.raises(ValueError, match="mesh"):
        head_shapes(mesh)


@pytest.mark.parametrize("urdf", [HEAD_URDF, pan_axis_head()], ids=["shipped", "pan-axis"])
def test_head_region_boxes_contain_the_head_between_samples(urdf: str) -> None:
    chain = UrdfChain.from_text(urdf)
    shapes = head_shapes(urdf)
    boxes = head_region_boxes(chain, shapes, REGION)
    assert {b.name for b in boxes} == {f"head {link}" for link in SHIPPED_HEAD_LINKS}
    per_link = {
        link: ObstacleSet([], [], [b for b in boxes if b.name == f"head {link}"]) for link in SHIPPED_HEAD_LINKS
    }
    # A dense grid, off the region's own HEAD_REGION_STEP_RAD samples: the head between samples is inside too.
    assert DENSE_STEP_RAD < HEAD_REGION_STEP_RAD / 4
    pans = np.arange(REGION.pan_rad[0], REGION.pan_rad[1] + 1e-12, DENSE_STEP_RAD)
    tilts = np.arange(REGION.tilt_rad[0], REGION.tilt_rad[1] + 1e-12, DENSE_STEP_RAD)
    poses = grid_link_poses(chain, pans, tilts)
    for shape in shapes:
        placed = poses[shape.link] @ shape.local  # (G, 4, 4)
        corners = np.einsum("gij,cj->gci", placed[:, :3, :3], CUBE * shape.half) + placed[:, None, :3, 3]
        distances = per_link[shape.link].distances(corners)  # (G, 8, boxes)
        inside = (distances.max(axis=1) <= EPS_M).any(axis=1)  # every corner inside one box of the link
        assert inside.all(), (shape.link, np.flatnonzero(~inside)[:5])


def test_head_region_needs_ordered_ranges() -> None:
    with pytest.raises(ValueError, match="pan_rad"):
        HeadRegion(pan_rad=(0.2, 0.1), tilt_rad=(0.0, 0.0))
    with pytest.raises(ValueError, match="tilt_rad"):
        HeadRegion(pan_rad=(0.0, 0.0), tilt_rad=(0.1, -0.1))
    # A degenerate region is one head pose: one box per shape, no inflation.
    held = HeadRegion(pan_rad=(0.1, 0.1), tilt_rad=(0.2, 0.2))
    chain = UrdfChain.from_text(HEAD_URDF)
    shapes = head_shapes(HEAD_URDF)
    boxes = head_region_boxes(chain, shapes, held)
    assert len(boxes) == len(shapes)
    for shape, box in zip(shapes, boxes, strict=True):
        assert box.half == pytest.approx(shape.half)
        assert box.pose == pytest.approx(chain.transform(shape.link, {"head_pan": 0.1, "head_tilt": 0.2}) @ shape.local)


def test_head_band_and_span_clip_to_the_urdf_limits() -> None:
    check = checker()
    band = math.radians(PoseCheckConfig().head_band_deg)
    pan_hi = UrdfChain.from_text(HEAD_URDF).limits("head_pan")[1]
    edge = check.head_band(1.5, 0.0)
    assert edge.pan_rad == pytest.approx((1.5 - band, pan_hi))
    assert edge.tilt_rad == pytest.approx((-band, band))
    span = check.head_span((0.0, 0.0), (0.2, -0.1))
    assert span.pan_rad == pytest.approx((-band, 0.2 + band))
    assert span.tilt_rad == pytest.approx((-0.1 - band, band))
    tilt_lo = UrdfChain.from_text(HEAD_URDF).limits("head_tilt")[0]
    assert check.head_span((0.0, -0.4), (0.0, 0.0)).tilt_rad == pytest.approx((tilt_lo, band))


def test_head_band_deg_is_validated() -> None:
    assert PoseCheckConfig().head_band_deg == 15.0
    for bad in (0.0, 46.0):
        with pytest.raises(ValidationError, match="head_band_deg"):
            PoseCheckConfig(head_band_deg=bad)


def test_no_region_keeps_the_envelope_obstacles() -> None:
    check = checker()
    assert check.obstacles.names[-1] == check.envelope.name == "head"
    assert check.obstacles.names[:-1] == [b.name for b in BodyConfig().boxes] + [c.name for c in BodyConfig().cylinders]
    through_head = {**ZERO, "shoulder_pan": FOREARM_PAN_RAD}
    for pose in (ZERO, LEVEL_TOOL_20CM, through_head, NEAR_HEAD):
        assert check.check_pose(pose, 0.3, rolling=False) == check.check_pose(pose, 0.3, rolling=False, head=None)
    assert check.check_path(ZERO, through_head, 0.3) == check.check_path(ZERO, through_head, 0.3, head=None)
    entries = check.check_head_move(through_head, 0.3, None)
    assert entries and all(e.startswith(f"{HEAD_CLEARANCE_RULE}: ") and " to head clearance " in e for e in entries)


def test_band_passes_where_the_envelope_refuses() -> None:
    check = PoseChecker(ARM_URDF, pan_axis_head(), fit_with(), PoseCheckConfig())
    assert check.envelope.radius > 0.13  # the camera lever grows the full-range envelope
    refused = check.check_pose(NEAR_HEAD, 0.3, rolling=False)
    assert refused and all(
        e.startswith("link_clearance: ") and e.split(" clearance ")[0].endswith(" to head") for e in refused
    ), refused
    gripper = check.link_poses(NEAR_HEAD)["gripper_link"]
    centre = mounted(check.fit, board_centre(check.fit.board)[None, :]) @ gripper[:3, :3].T + gripper[:3, 3]
    pan, tilt = aim_head(check.head, centre[0])
    band = check.head_band(pan, tilt)
    assert check.check_pose(NEAR_HEAD, 0.3, rolling=False, head=band) == []
    assert check.check_head_move(NEAR_HEAD, 0.3, band) == []
    assert check.check_path(NEAR_HEAD, {**NEAR_HEAD, "shoulder_pan": 0.0}, 0.3, head=band) == []


def test_region_check_refuses_an_arm_in_the_head() -> None:
    check = checker()
    through_head = {**ZERO, "shoulder_pan": FOREARM_PAN_RAD}
    entries = check.check_pose(through_head, 0.3, rolling=False, head=check.head_band(0.0, 0.0))
    assert any(
        e.startswith(("link_clearance: forearm to head head_link", "link_clearance: forearm to head d435_link"))
        for e in entries
    ), entries
    moves = check.check_head_move(through_head, 0.3, check.head_span((0.0, 0.0), (0.2, 0.0)))
    assert moves and all(e.startswith(f"{HEAD_CLEARANCE_RULE}: ") for e in moves), moves
    assert any(" to head head_link clearance " in e or " to head d435_link clearance " in e for e in moves), moves
    # The body's static head cylinder is not a head-move obstacle.
    assert not [e for e in moves if " to head clearance " in e], moves
    # The board in the head is a head-move refusal too, against the board threshold.
    fit = fit_with(mount_with_corner_at(ZERO, np.array([0.40, -0.05, 0.35])))
    centre = PoseChecker(ARM_URDF, HEAD_URDF, fit, PoseCheckConfig()).head.transform(
        "head_link", {"head_pan": 0.0, "head_tilt": 0.0}
    )
    boarded = checker(fit_with(handle_box_at(fit, ZERO, centre[:3, 3])))
    board = [e for e in boarded.check_head_move(ZERO, 0.3, boarded.head_band(0.0, 0.0)) if " board to " in e]
    assert board and all(
        e.startswith(f"{HEAD_CLEARANCE_RULE}: board to head ") and "board_margin_m" in e for e in board
    ), board


def test_clearances_are_reported_once_per_obstacle_name() -> None:
    check = checker()
    through_head = {**ZERO, "shoulder_pan": FOREARM_PAN_RAD}
    for head in (None, check.head_band(0.0, 0.0), check.head_span((-0.5, -0.3), (0.5, 0.6))):
        for entries in (
            check.check_pose(through_head, 0.3, rolling=False, head=head),
            check.check_head_move(through_head, 0.3, head),
        ):
            prefixes = [m.group(1) for e in entries if (m := OBSTACLE_ENTRY.match(e))]
            assert prefixes and len(prefixes) == len(set(prefixes)), entries


def test_region_obstacle_sets_are_cached_per_region() -> None:
    check = checker()
    band = check.head_band(0.1, 0.2)
    assert check.region_obstacles(band) is check.region_obstacles(check.head_band(0.1, 0.2))
    assert check.region_obstacles(band) is not check.region_obstacles(check.head_band(0.0, 0.2))
    names = check.region_obstacles(band).names
    body = [b.name for b in BodyConfig().boxes] + [c.name for c in BodyConfig().cylinders]
    assert names[: len(body)] == body and "head" in body  # the body's static head cylinder stays
    assert set(names[len(body) :]) == {f"head {link}" for link in SHIPPED_HEAD_LINKS}


def test_worst_point_clearances_are_exact_below_the_cutoff() -> None:
    """The broad phase only replaces distances it proves are at or above the cutoff: below it every name's value is
    the exact smallest distance, at or above it the value is a lower bound not below the cutoff."""
    check = checker()
    obstacles = check.region_obstacles(check.head_span((-0.5, -0.3), (0.5, 0.6)))
    near_head = {**ZERO, "shoulder_pan": FOREARM_PAN_RAD}
    for joints in (ZERO, near_head, LEVEL_TOOL_20CM):
        points = check.board_points(joints)
        exact = obstacles.worst_by_name(obstacles.distances(points).min(axis=0))
        for cutoff in (0.0, 0.01, 0.05, 0.2):
            fast = obstacles.worst_point_clearances(points, cutoff)
            below = exact < cutoff
            assert fast[below] == pytest.approx(exact[below], abs=1e-12), (joints, cutoff)
            assert (fast[~below] >= cutoff).all() and (fast[~below] <= exact[~below] + 1e-12).all(), (joints, cutoff)
    # The oriented distances agree with the paired form the capsule search uses.
    points = check.board_points(near_head)[:50]
    paired = obstacles.paired_distances(np.broadcast_to(points[:, None, :], (50, len(obstacles.names), 3)))
    assert paired == pytest.approx(obstacles.distances(points), abs=1e-12)
