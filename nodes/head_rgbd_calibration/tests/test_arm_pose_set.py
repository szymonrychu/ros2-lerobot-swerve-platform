"""The shipped head-from-arm arm pose set: safe by FK, the board in view, coverage and a passing synthetic fit.

Everything runs on nodes/web_ui/urdf/so101_arm.urdf and nodes/web_ui/urdf/head.urdf, both as shipped and with the
applied floor-fit chain of 2026-10-22 as overrides (`tests/head_arm_synthetic.py::applied_head_urdf`). The poses are
stored as measured (follower) angles, the space `move_arm_joints` takes; every FK applies `urdf = measured + offset`
with `fit.joint_offsets_rad`.

The home pose the sequence starts from is `HOME_POSE`: `scripts/gen_nav_footprint.yaml` `parked_arm_joints`, the
repo record of the home that mcp_server's `arm.home_file` (/var/lib/ros2/arm/home.yaml) holds and `arm_home` parks
at. The board mount is the shipped jaw-geometry estimate (design D47): the safety tests use the board margin
`margin_m + provisional_margin_m` and the mount shifted by +-1 cm and turned by +-5 deg about each gripper axis.
"""

import importlib.util
import math
import sys
import xml.etree.ElementTree as ET
from collections.abc import Iterable, Iterator
from functools import cache
from pathlib import Path
from types import ModuleType

import cv2
import numpy as np
import pytest
import yaml
from ros2_common.head_rgbd_calibration.arm_pose_check import PoseChecker
from ros2_common.head_rgbd_calibration.chain import UrdfChain, pose_from_xyz_rpy, xyz_rpy_from_pose
from ros2_common.head_rgbd_calibration.charuco import board_corners_3d
from ros2_common.head_rgbd_calibration.config import ARM_JOINT_NAMES, BoardMountConfig, HeadArmFitConfig
from ros2_common.head_rgbd_calibration.floor_fit import PARAM_NAMES as FLOOR_PARAMS
from ros2_common.head_rgbd_calibration.head_arm_fit import (
    aim_head,
    board_centre,
    camera_pose,
    fit_head_from_arm,
    limit_checks,
    start_params,
)

from head_rgbd_calibration.config import ArmSequenceConfig, NodeConfig, load_config

NODE_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = NODE_DIR.parents[1]
SHIPPED_YAML = NODE_DIR / "config" / "head_rgbd_calibration.yaml"
SYNTHETIC_FILE = REPO_ROOT / "tests" / "head_arm_synthetic.py"
URDF_DIR = REPO_ROOT / "nodes" / "web_ui" / "urdf"
ARM_PY = REPO_ROOT / "nodes" / "mcp_server" / "mcp_server" / "arm.py"
FOOTPRINT_YAML = REPO_ROOT / "scripts" / "gen_nav_footprint.yaml"
MEASURED_SPACE_PHRASE = (
    "Joint name -> target rad in measured (follower) joint space, as reported by get_arm_state (clamped to URDF "
    "limits minus margin, applied in URDF space: urdf = measured + offset)"
)
WIDENED_TILT_RAD = (-0.5236, 1.0472)  # -30..+60 deg, the reprinted mount's range (head-tilt-range-widened)
CLIENT_YML = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
MOUNT_ESTIMATE_NOTE = "jaw-geometry estimate, not a measurement"
MOUNT_SHIFT_M = 0.01
MOUNT_TURN_DEG = 5.0
MIN_VISIBLE_CORNERS = 12
IMAGE_MARGIN_PX = 10.0
MIN_GOOD_OFFSETS = 3
MIN_CAMERA_DISTANCE_M = 0.25
MAX_CAMERA_DISTANCE_M = 0.65
BASE_MIN_DISTANCE_M = 0.30
BASE_MAX_DISTANCE_M = 0.50
MAX_TRANSIT_RAD = 0.7  # largest single-joint change between consecutive poses of the sequence
NOISE_PX = 0.3
SEED = 7
MIN_TIER0 = 8
MIN_TIER1 = 8
MIN_TIER2 = 12
WRIST_ROLL = "wrist_roll"
WRIST_FLEX = "wrist_flex"
BIG_JOINTS = ("elbow_flex", "shoulder_lift", "shoulder_pan")
PAN, TILT = "head_pan", "head_tilt"
EPS = 1e-9


def load_synthetic() -> ModuleType:
    """Import tests/head_arm_synthetic.py of the repo root (the node has its own tests package)."""
    name = "head_arm_synthetic_root"
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, SYNTHETIC_FILE)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


SYN = load_synthetic()
CFG = load_config(SHIPPED_YAML)
SEQ: ArmSequenceConfig = CFG.arm_sequence
FIT: HeadArmFitConfig = CFG.fit
ARM_URDF = (URDF_DIR / "so101_arm.urdf").read_text()
SHIPPED_HEAD = (URDF_DIR / "head.urdf").read_text()
APPLIED_HEAD = SYN.applied_head_urdf()
HOME_POSE: dict[str, float] = {
    name: float(value)
    for name, value in yaml.safe_load(FOOTPRINT_YAML.read_text())["parked_arm_joints"].items()
    if name in ARM_JOINT_NAMES
}


def with_tilt_limits(urdf: str, lower: float, upper: float) -> str:
    """The head URDF with other head_tilt limits."""
    root = ET.fromstring(urdf)
    limit = next(j for j in root.findall("joint") if j.get("name") == TILT).find("limit")
    limit.set("lower", str(lower))
    limit.set("upper", str(upper))
    return ET.tostring(root, encoding="unicode")


HEADS = {"shipped": SHIPPED_HEAD, "applied": APPLIED_HEAD}
SAFETY_HEADS = {
    **HEADS,
    "shipped-tilt-60": with_tilt_limits(SHIPPED_HEAD, *WIDENED_TILT_RAD),
    "applied-tilt-60": with_tilt_limits(APPLIED_HEAD, *WIDENED_TILT_RAD),
}


def set_poses() -> list[dict[str, float]]:
    """Tier 0, tier 1 and tier 2 poses in sequence order."""
    return [*SEQ.tier0_poses, *SEQ.tier1_poses, *SEQ.tier2_poses]


def sequence() -> list[tuple[str, dict[str, float]]]:
    """(name, pose) of the whole sequence: home, base, tier 0, tier 1, tier 2, base."""
    named = [("home", HOME_POSE), ("base", SEQ.base_pose)]
    for tier in (0, 1, 2):
        named += [(f"tier {tier} pose {i}", p) for i, p in enumerate(getattr(SEQ, f"tier{tier}_poses"))]
    return [*named, ("base (return)", SEQ.base_pose)]


def band_mounts() -> Iterator[tuple[str, BoardMountConfig]]:
    """The nominal mount, then shifted by +-MOUNT_SHIFT_M along and turned by +-MOUNT_TURN_DEG (about the board
    centre) about each gripper_link axis."""
    nominal = FIT.board_mount
    yield "nominal", nominal
    pose = pose_from_xyz_rpy(nominal.xyz_init, nominal.rpy_init)
    centre = (pose @ np.append(board_centre(FIT.board), 1.0))[:3]
    for axis, label in enumerate("xyz"):
        for sign in (-1.0, 1.0):
            shifted = pose.copy()
            shifted[axis, 3] += sign * MOUNT_SHIFT_M
            turn = np.eye(4)
            turn[:3, :3] = cv2.Rodrigues(np.eye(3)[axis] * sign * math.radians(MOUNT_TURN_DEG))[0]
            to_centre, from_centre = np.eye(4), np.eye(4)
            to_centre[:3, 3], from_centre[:3, 3] = centre, -centre
            turned = to_centre @ turn @ from_centre @ pose
            for kind, moved in (("shift", shifted), ("turn", turned)):
                xyz, rpy = xyz_rpy_from_pose(moved)
                yield (
                    f"{kind} {'+' if sign > 0 else '-'}{label}",
                    nominal.model_copy(update={"xyz_init": xyz, "rpy_init": rpy}),
                )


def checkers(head_urdf: str, mounts: Iterable[tuple[str, BoardMountConfig]]) -> Iterator[tuple[str, PoseChecker]]:
    """A pose checker per board mount, with the extra board margin of the estimated mount."""
    for label, mount in mounts:
        fit = FIT.model_copy(update={"board_mount": mount})
        yield label, PoseChecker(ARM_URDF, head_urdf, fit, SEQ.pose_check, SEQ.board_margin_m(nominal_mount=True))


def board_centre_in_base(checker: PoseChecker, pose: dict[str, float]) -> np.ndarray:
    """Centre of the board (nominal mount) in base_link at an arm pose (m)."""
    mount = pose_from_xyz_rpy(FIT.board_mount.xyz_init, FIT.board_mount.rpy_init)
    board = checker.link_poses(pose)["gripper_link"] @ mount
    return (board @ np.append(board_centre(FIT.board), 1.0))[:3]


@cache
def aimed_views(head_name: str) -> list[tuple[float, float, list[tuple[int, float]]]]:
    """Per set pose: aimed (pan, tilt) rad and per head offset (visible corners, camera-to-board-centre distance).

    The head is aimed by `aim_head` at the board centre (nominal mount) on the given chain, whose limits clamp it;
    an offset whose pan or tilt leaves the URDF limits is skipped at run time and counts 0 corners. A corner counts
    when it lies in front of the camera, inside the 576x324 image with IMAGE_MARGIN_PX to spare, and the board front
    faces the camera (`MIN_FACING_COS`, as the synthetic scenes).
    """
    chain = UrdfChain.from_text(HEADS[head_name])
    checker = PoseChecker(ARM_URDF, HEADS[head_name], FIT, SEQ.pose_check)
    mount = pose_from_xyz_rpy(FIT.board_mount.xyz_init, FIT.board_mount.rpy_init)
    corners = np.column_stack([board_corners_3d(FIT.board), np.ones(len(board_corners_3d(FIT.board)))])
    pan_limits, tilt_limits = chain.limits(PAN), chain.limits(TILT)
    width, height = SYN.IMAGE_SIZE
    out = []
    for pose in set_poses():
        board = checker.link_poses(pose)["gripper_link"] @ mount
        centre = board_centre_in_base(checker, pose)
        pan0, tilt0 = aim_head(chain, centre)
        views = []
        for dpan, dtilt in SEQ.head_offsets_deg:
            pan, tilt = pan0 + math.radians(dpan), tilt0 + math.radians(dtilt)
            if not (pan_limits[0] <= pan <= pan_limits[1] and tilt_limits[0] <= tilt <= tilt_limits[1]):
                views.append((0, math.nan))
                continue
            camera = camera_pose(chain, {PAN: pan, TILT: tilt})
            in_camera = np.linalg.solve(camera, board)
            points = (in_camera @ corners.T).T[:, :3]
            pixels = (SYN.SYNTH_CAMERA_MATRIX @ points.T).T
            pixels = pixels[:, :2] / pixels[:, 2:3]
            inside = (
                (points[:, 2] > 0)
                & (pixels[:, 0] >= IMAGE_MARGIN_PX)
                & (pixels[:, 0] <= width - 1 - IMAGE_MARGIN_PX)
                & (pixels[:, 1] >= IMAGE_MARGIN_PX)
                & (pixels[:, 1] <= height - 1 - IMAGE_MARGIN_PX)
            )
            facing = in_camera[2, 2] >= SYN.MIN_FACING_COS
            views.append((int(inside.sum()) if facing else 0, float(np.linalg.norm(centre - camera[:3, 3]))))
        out.append((pan0, tilt0, views))
    return out


def test_target_joint_convention() -> None:
    docstring = " ".join(ARM_PY.read_text().split())
    assert MEASURED_SPACE_PHRASE in docstring


def test_home_pose_is_the_parked_arm() -> None:
    assert set(HOME_POSE) == set(ARM_JOINT_NAMES)
    assert "/var/lib/ros2/arm/home.yaml" in FOOTPRINT_YAML.read_text()


def test_board_mount_is_the_nominal_jaw_mount() -> None:
    """The shipped mount: the board normal to the tool axis, its centre on it 2 cm past the URDF tool point."""
    tool_xyz, _ = UrdfChain.from_text(ARM_URDF).origin("gripper_frame_joint")
    assert tool_xyz == pytest.approx(SYN.TOOL_POINT, abs=EPS)
    xyz, rpy = SYN.nominal_board_mount(FIT)
    assert FIT.board_mount.xyz_init == pytest.approx(xyz, abs=1e-4)
    assert FIT.board_mount.rpy_init == pytest.approx(rpy, abs=1e-6)
    assert NodeConfig().fit.board_mount == FIT.board_mount
    for path in (SHIPPED_YAML, CLIENT_YML):
        text = path.read_text()
        mount_block = text[text.index("  board_mount:") : text.index("  limits: # acceptance limits of the fit")]
        assert MOUNT_ESTIMATE_NOTE in mount_block, path


def test_tier_rules_hold_for_shipped_set() -> None:
    assert ArmSequenceConfig.model_validate(SEQ.model_dump()) == SEQ
    assert len(SEQ.tier0_poses) >= MIN_TIER0
    assert len(SEQ.tier1_poses) >= MIN_TIER1
    assert len(SEQ.tier2_poses) >= MIN_TIER2
    assert SEQ.gripper_rad <= SEQ.pose_check.max_roll_gripper_rad
    base = SEQ.base_pose
    for tier, allowed in ((0, {WRIST_ROLL}), (1, {WRIST_ROLL, WRIST_FLEX}), (2, {WRIST_ROLL, WRIST_FLEX, *BIG_JOINTS})):
        for index, pose in enumerate(getattr(SEQ, f"tier{tier}_poses")):
            where = f"tier {tier} pose {index}"
            assert set(pose) == set(ARM_JOINT_NAMES), where
            delta = {name: pose[name] - base[name] for name in ARM_JOINT_NAMES}
            moved = {name for name, value in delta.items() if abs(value) > EPS}
            assert moved <= allowed, (where, moved)
            assert abs(delta[WRIST_ROLL]) <= SEQ.max_roll_delta_rad + EPS, where
            assert abs(delta[WRIST_FLEX]) <= SEQ.max_flex_delta_rad + EPS, where
            assert all(abs(delta[name]) <= SEQ.max_big_delta_rad + EPS for name in BIG_JOINTS), where
            # Each tier adds motion: tier 1 moves wrist_flex, tier 2 at least one of the big joints.
            assert tier != 0 or moved == {WRIST_ROLL}, where
            assert tier != 1 or WRIST_FLEX in moved, where
            assert tier != 2 or moved & set(BIG_JOINTS), where


def test_transits_are_short() -> None:
    steps = sequence()[1:]
    for (name_a, a), (name_b, b) in zip(steps, steps[1:], strict=False):
        largest = max(abs(b[n] - a[n]) for n in ARM_JOINT_NAMES)
        assert largest <= MAX_TRANSIT_RAD, (name_a, name_b, largest)


def sequence_failures(head_urdf: str, mounts: Iterable[tuple[str, BoardMountConfig]]) -> list[str]:
    """Safety-check entries of every pose (rolling where wrist_roll changes) and transit of the sequence, per mount.

    :param head_urdf: head URDF text, its joint limits give the head envelope
    :param mounts: (label, board mount) pairs to check the sequence with
    :return: one "<mount> <step>: <entry>" line per failed check; empty when everything is safe
    """
    steps = sequence()
    roll_change = SEQ.pose_check.roll_min_change_rad
    failures = []
    for mount, checker in checkers(head_urdf, mounts):
        for (name_a, a), (name_b, b) in zip(steps, steps[1:], strict=False):
            rolling = abs(b[WRIST_ROLL] - a[WRIST_ROLL]) > roll_change
            failures += [f"{mount} {name_b}: {e}" for e in checker.check_pose(b, SEQ.gripper_rad, rolling)]
            failures += [f"{mount} {name_a} -> {name_b}: {e}" for e in checker.check_path(a, b, SEQ.gripper_rad)]
    return failures


@pytest.mark.parametrize("head_name", list(SAFETY_HEADS))
def test_every_pose_and_transit_is_safe(head_name: str) -> None:
    """The shipped (estimated) mount with margin_m + provisional_margin_m."""
    failures = sequence_failures(SAFETY_HEADS[head_name], [("nominal", FIT.board_mount)])
    assert failures == [], "\n".join(failures[:20])


@pytest.mark.parametrize("head_name", list(SAFETY_HEADS))
def test_safe_under_board_mount_uncertainty(head_name: str) -> None:
    """The mount shifted by +-1 cm along and turned by +-5 deg about each gripper_link axis, so a measured mount in
    that band keeps every pose and transit safe."""
    mounts = list(band_mounts())
    assert len(mounts) == 1 + 2 * 2 * 3
    failures = sequence_failures(SAFETY_HEADS[head_name], mounts[1:])
    assert failures == [], "\n".join(failures[:20])


def test_safety_check_refuses_a_pose_the_tier_rules_allow() -> None:
    """The tier bounds alone do not make a pose safe: shoulder_pan at +max_big_delta_rad swings the upper arm over
    the right frame rail, which the safety test catches."""
    pose = {**SEQ.base_pose, "shoulder_pan": SEQ.base_pose["shoulder_pan"] + SEQ.max_big_delta_rad}
    ArmSequenceConfig.model_validate({**SEQ.model_dump(), "tier2_poses": [*SEQ.tier2_poses, pose]})
    _, checker = next(checkers(SHIPPED_HEAD, [("nominal", FIT.board_mount)]))
    entries = checker.check_pose(pose, SEQ.gripper_rad, rolling=False)
    assert any(e.startswith("link_clearance: upper arm to frame_right") for e in entries), entries


def test_base_pose_puts_the_board_in_front_of_the_head() -> None:
    body = SEQ.pose_check.body
    body_front = max([box.center[0] + box.size[0] / 2 for box in body.boxes] + [c.x + c.radius for c in body.cylinders])
    body_top = max([box.center[2] + box.size[2] / 2 for box in body.boxes] + [c.z_max for c in body.cylinders])
    for head_name, head_urdf in HEADS.items():
        checker = PoseChecker(ARM_URDF, head_urdf, FIT, SEQ.pose_check)
        board = checker.board_points(SEQ.base_pose)
        centre = board_centre_in_base(checker, SEQ.base_pose)
        # Forward: the whole board ahead of the body and the head envelope; lifted: its centre above the body top.
        assert board[:, 0].min() > body_front + SEQ.pose_check.margin_m, head_name
        assert board[:, 0].min() > checker.envelope.x + checker.envelope.radius, head_name
        assert centre[2] > body_top + SEQ.pose_check.margin_m, head_name
        chain = UrdfChain.from_text(head_urdf)
        pan, tilt = aim_head(chain, centre)
        distance = float(np.linalg.norm(centre - camera_pose(chain, {PAN: pan, TILT: tilt})[:3, 3]))
        assert BASE_MIN_DISTANCE_M <= distance <= BASE_MAX_DISTANCE_M, (head_name, distance)
        assert checker.check_pose(SEQ.base_pose, SEQ.gripper_rad, rolling=False) == [], head_name


@pytest.mark.parametrize("head_name", list(HEADS))
def test_board_visible_from_aimed_head(head_name: str) -> None:
    for index, (_, _, views) in enumerate(aimed_views(head_name)):
        good = [n >= MIN_VISIBLE_CORNERS and MIN_CAMERA_DISTANCE_M <= d <= MAX_CAMERA_DISTANCE_M for n, d in views]
        assert sum(good) >= MIN_GOOD_OFFSETS, (index, views)


@pytest.mark.parametrize("head_name", list(HEADS))
def test_coverage_limits(head_name: str) -> None:
    aimed = aimed_views(head_name)
    pan_span = math.degrees(float(np.ptp([pan for pan, _, _ in aimed])))
    tilt_span = math.degrees(float(np.ptp([tilt for _, tilt, _ in aimed])))
    assert pan_span >= FIT.limits.min_pan_span_deg, pan_span
    assert tilt_span >= FIT.limits.min_tilt_span_deg, tilt_span
    assert len(set_poses()) >= FIT.limits.min_arm_poses


@pytest.mark.parametrize("head_name", list(HEADS))
def test_synthetic_fit_on_shipped_set_passes(head_name: str) -> None:
    head_urdf = HEADS[head_name]
    start = start_params(UrdfChain.from_text(head_urdf))
    floor = {name: start[name] for name in FLOOR_PARAMS}
    if head_name == "applied":
        assert floor == pytest.approx(SYN.APPLIED_FLOOR)
    xyz, rpy = SYN.nominal_board_mount(FIT)
    truth = SYN.HeadArmTruth(origins=SYN.perturbed_origins(head_urdf, SYN.TRUTH_DELTAS), board_xyz=xyz, board_rpy=rpy)
    start_xyz, start_rpy = SYN.perturbed_board_mount(
        FIT.board_mount.xyz_init, FIT.board_mount.rpy_init, SYN.MOUNT_INIT_SHIFT_M, SYN.MOUNT_INIT_TURN_DEG
    )
    cfg = FIT.model_copy(update={"board_mount": BoardMountConfig(xyz_init=start_xyz, rpy_init=start_rpy)})
    tiers = [0] * len(SEQ.tier0_poses) + [1] * len(SEQ.tier1_poses) + [2] * len(SEQ.tier2_poses)
    views = SYN.arm_board_scene(
        truth, head_urdf, ARM_URDF, set_poses(), SEQ.head_offsets_deg, NOISE_PX, SEED, cfg, tiers
    )
    result = fit_head_from_arm(views, SYN.SYNTH_CAMERA_MATRIX, SYN.SYNTH_DIST, head_urdf, ARM_URDF, cfg, floor)
    failed = [check for check in limit_checks(result, cfg.limits) if not check[3]]
    assert result.passed and failed == [], failed
