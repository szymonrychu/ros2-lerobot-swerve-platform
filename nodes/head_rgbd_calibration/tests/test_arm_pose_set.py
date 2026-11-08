"""The shipped head-from-arm arm pose set: safe by FK, the board in the D435's range, coverage and a passing synthetic
fit.

Everything runs on nodes/web_ui/urdf/so101_arm.urdf and nodes/web_ui/urdf/head.urdf, both as shipped and with the
applied floor-fit chain of 2026-10-22 as overrides (`tests/head_arm_synthetic.py::synth_head_urdf`). The poses are
stored as measured (follower) angles, the space `move_arm_joints` takes; every FK applies `urdf = measured + offset`
with `fit.joint_offsets_rad`.

The home pose the sequence starts from is `HOME_POSE`: `scripts/gen_nav_footprint.yaml` `parked_arm_joints`, the
repo record of the home that mcp_server's `arm.home_file` (/var/lib/ros2/arm/home.yaml) holds and `arm_home` parks
at. The board is the ARM board on its paddle handle, mounted at the handle CAD seed (markers/charuco-arm-6x5-28mm.json),
not yet fit-solved (design D47): the safety tests check the plate plus its ribs and handle (`board_mount.back_boxes_mm`)
with the board margin `margin_m + provisional_margin_m` and the mount shifted by +-1 cm and turned by +-5 deg about each
gripper axis. The camera is the head D435 colour camera (`d435_color_optical_frame`) with the nominal 640x360
intrinsics (69 deg across, the datasheet colour FOV); the factory camera_info replaces them at run time.
"""

import importlib.util
import json
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
from ros2_common.camera_geometry import CameraIntrinsics
from ros2_common.head_rgbd_calibration.arm_pose_check import FOV_RULE, MIN_Z_RULE, PoseChecker
from ros2_common.head_rgbd_calibration.chain import UrdfChain, pose_from_xyz_rpy, xyz_rpy_from_pose
from ros2_common.head_rgbd_calibration.charuco import board_corners_3d
from ros2_common.head_rgbd_calibration.config import ARM_BOARD, ARM_JOINT_NAMES, BoardMountConfig, HeadArmFitConfig
from ros2_common.head_rgbd_calibration.floor_fit import PARAM_NAMES as FLOOR_PARAMS
from ros2_common.head_rgbd_calibration.head_arm_fit import (
    aim_head,
    board_centre,
    camera_pose,
    fit_head_from_arm,
    limit_checks,
    start_params,
)

from head_rgbd_calibration.config import ArmSequenceConfig, load_config

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
WIDENED_TILT_RAD = (-0.5236, 0.7854)  # -30..+45 deg, the reprinted mount's range (head-tilt-range-widened)
CLIENT_YML = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
MOUNT_SHIFT_M = 0.01
MOUNT_TURN_DEG = 5.0
MIN_VISIBLE_CORNERS = 12
IMAGE_MARGIN_PX = 10.0
MIN_GOOD_OFFSETS = 3
MIN_OFFSET_POSE_SHARE = 0.75  # each head offset keeps its view on at least this share of the shipped poses
COVERAGE_MARGIN = 1.5  # the kept D435 views meet every coverage limit times this
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
D435_COLOR = CameraIntrinsics.from_hfov(640, 360, 69.0)


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
APPLIED_HEAD = SYN.synth_head_urdf()
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
    "shipped-tilt-45": with_tilt_limits(SHIPPED_HEAD, *WIDENED_TILT_RAD),
    "applied-tilt-45": with_tilt_limits(APPLIED_HEAD, *WIDENED_TILT_RAD),
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
    when it lies in front of the camera, inside the nominal D435 640x360 colour image with IMAGE_MARGIN_PX to spare,
    and the board front faces the camera (`MIN_FACING_COS`, as the synthetic scenes).
    """
    chain = UrdfChain.from_text(HEADS[head_name])
    checker = PoseChecker(ARM_URDF, HEADS[head_name], FIT, SEQ.pose_check)
    mount = pose_from_xyz_rpy(FIT.board_mount.xyz_init, FIT.board_mount.rpy_init)
    corners = np.column_stack([board_corners_3d(FIT.board), np.ones(len(board_corners_3d(FIT.board)))])
    pan_limits, tilt_limits = chain.limits(PAN), chain.limits(TILT)
    width, height = D435_COLOR.width, D435_COLOR.height
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
            pixels = (D435_COLOR.matrix() @ points.T).T
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
    nodes = yaml.safe_load(CLIENT_YML.read_text())["ros2_nodes"]
    home_file = yaml.safe_load(next(n for n in nodes if n["name"] == "mcp_server")["config"])["arm"]["home_file"]
    assert home_file == "/var/lib/ros2/arm/home.yaml"
    assert home_file in FOOTPRINT_YAML.read_text()


def test_board_mount_is_the_handle_cad_mount() -> None:
    """The ARM board on its paddle handle in the closed gripper: the CAD seed of the handle metadata, the board normal
    along the jaw opening axis and board y against the tool direction; both yamls name the handle CAD."""
    meta = json.loads((REPO_ROOT / "markers" / "charuco-arm-6x5-28mm.json").read_text())
    assert FIT.board == ARM_BOARD
    assert FIT.board_mount.xyz_init == pytest.approx(meta["board_mount"]["xyz_init"], abs=1e-6)
    mount = pose_from_xyz_rpy(FIT.board_mount.xyz_init, FIT.board_mount.rpy_init)
    assert mount[:3, 2] == pytest.approx([1.0, 0.0, 0.0], abs=1e-6)  # board z along the jaw opening axis
    assert mount[:3, 1] == pytest.approx([0.0, 0.0, 1.0], abs=1e-6)  # board y against the tool direction
    for path in (SHIPPED_YAML, CLIENT_YML):
        text = path.read_text()
        block = text[text.index("  board_mount:") : text.index("  limits: # acceptance limits of the fit")]
        assert "charuco-arm-6x5-28mm.json" in block and "measured by the user" not in block, path


def test_tier_rules_hold_for_shipped_set() -> None:
    assert ArmSequenceConfig.model_validate(SEQ.model_dump()) == SEQ
    assert len(SEQ.tier0_poses) >= MIN_TIER0
    assert len(SEQ.tier1_poses) >= MIN_TIER1
    assert len(SEQ.tier2_poses) >= MIN_TIER2
    assert SEQ.insert_gripper_rad <= SEQ.pose_check.max_roll_gripper_rad
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
    failures = []
    for mount, checker in checkers(head_urdf, mounts):
        for (name_a, a), (name_b, b) in zip(steps, steps[1:], strict=False):
            rolling = b[WRIST_ROLL] != a[WRIST_ROLL]
            # The base move opens the gripper for the handle; every later move holds it closed on the handle.
            gripper = SEQ.insert_gripper_rad if name_a == "home" else SEQ.grip_rad
            failures += [f"{mount} {name_b}: {e}" for e in checker.check_pose(b, gripper, rolling)]
            failures += [f"{mount} {name_a} -> {name_b}: {e}" for e in checker.check_path(a, b, gripper)]
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
    """The tier bounds alone do not make a pose safe: shoulder_lift and elbow_flex both at +max_big_delta_rad lower
    the board below the floor, which the safety test catches. (At the 5 mm body keep-away no tier-2 corner reaches
    a body part: the pan swing that once came within 30 mm of the right frame rail now passes.)"""
    pose = {
        **SEQ.base_pose,
        "shoulder_lift": SEQ.base_pose["shoulder_lift"] + SEQ.max_big_delta_rad,
        "elbow_flex": SEQ.base_pose["elbow_flex"] + SEQ.max_big_delta_rad,
    }
    ArmSequenceConfig.model_validate({**SEQ.model_dump(), "tier2_poses": [*SEQ.tier2_poses, pose]})
    _, checker = next(checkers(SHIPPED_HEAD, [("nominal", FIT.board_mount)]))
    entries = checker.check_pose(pose, SEQ.grip_rad, rolling=False)
    assert any(e.startswith("min_board_height_m") for e in entries), entries


def test_tier2_shoulder_pan_stays_in_the_clear_band() -> None:
    """At margin_m 0.03 the upper arm comes within 30 mm of frame_right above pan ~0.25 rad and the lidar and head
    envelope below pan ~-0.5, so every shipped shoulder_pan stays inside [-0.5, 0.2] and the base pan keeps the
    +-max_big_delta_rad tier-2 swing from reaching either edge. The shipped 5 mm margin is smaller: the band stays
    clear."""
    pans = [pose["shoulder_pan"] for pose in [SEQ.base_pose, *SEQ.tier2_poses]]
    assert SEQ.pose_check.margin_m <= 0.03
    assert -0.5 <= min(pans) and max(pans) <= 0.2, pans


def test_base_pose_puts_the_board_in_front_of_the_head() -> None:
    # The floor-level body (the head post, head arm and head envelope stand above it; the board is checked ahead of the head envelope below).
    boxes = [b for b in SEQ.pose_check.body.boxes if not b.name.startswith("head")]
    cylinders = [c for c in SEQ.pose_check.body.cylinders if not c.name.startswith("head")]
    body_front = max([box.center[0] + box.size[0] / 2 for box in boxes] + [c.x + c.radius for c in cylinders])
    body_top = max([box.center[2] + box.size[2] / 2 for box in boxes] + [c.z_max for c in cylinders])
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
        assert checker.check_pose(SEQ.base_pose, SEQ.grip_rad, rolling=False) == [], head_name


@pytest.mark.parametrize("head_name", list(HEADS))
def test_board_visible_from_aimed_head(head_name: str) -> None:
    for index, (_, _, views) in enumerate(aimed_views(head_name)):
        good = [n >= MIN_VISIBLE_CORNERS and MIN_CAMERA_DISTANCE_M <= d <= MAX_CAMERA_DISTANCE_M for n, d in views]
        assert sum(good) >= MIN_GOOD_OFFSETS, (index, views)


@cache
def d435_views(head_name: str) -> list[tuple[int, int, float, float, list[str]]]:
    """(pose index, head offset index, pan rad, tilt rad, `PoseChecker.check_view` entries) of every view inside the
    head limits, the head aimed at the board centre as the sequence does, with the nominal D435 colour intrinsics."""
    chain = UrdfChain.from_text(HEADS[head_name])
    checker = PoseChecker(ARM_URDF, HEADS[head_name], FIT, SEQ.pose_check)
    pan_limits, tilt_limits = chain.limits(PAN), chain.limits(TILT)
    out = []
    for index, pose in enumerate(set_poses()):
        pan0, tilt0 = aim_head(chain, board_centre_in_base(checker, pose))
        for offset, (dpan, dtilt) in enumerate(SEQ.head_offsets_deg):
            pan, tilt = pan0 + math.radians(dpan), tilt0 + math.radians(dtilt)
            if pan_limits[0] <= pan <= pan_limits[1] and tilt_limits[0] <= tilt <= tilt_limits[1]:
                out.append((index, offset, pan, tilt, checker.check_view(pose, pan, tilt, D435_COLOR)))
    return out


def pose_names() -> list[str]:
    """Name (tier <t> pose <i>) of every set pose, in `set_poses` order."""
    return [f"tier {tier} pose {i}" for tier in (0, 1, 2) for i in range(len(getattr(SEQ, f"tier{tier}_poses")))]


@pytest.mark.parametrize("head_name", list(HEADS))
def test_shipped_views_keep_the_board_in_the_d435_range(head_name: str) -> None:
    """No shipped view puts a board corner inside the D435 min-Z (0.15 m + 0.05 m margin), and the views the colour
    FOV rule keeps (the sequence skips the others) meet the fit's coverage limits times COVERAGE_MARGIN in every
    tier."""
    views = d435_views(head_name)
    assert [v for v in views if any(e.startswith(MIN_Z_RULE) for e in v[4])] == []
    assert all(e.startswith(FOV_RULE) for v in views for e in v[4])
    kept = [v for v in views if not v[4]]
    poses = sorted({index for index, *_ in kept})
    held = set(poses[FIT.limits.holdout_every - 1 :: FIT.limits.holdout_every])
    limits = FIT.limits
    assert len(poses) >= COVERAGE_MARGIN * limits.min_arm_poses, poses
    assert sum(1 for index, *_ in kept if index not in held) >= COVERAGE_MARGIN * limits.min_views
    assert math.degrees(float(np.ptp([v[2] for v in kept]))) >= COVERAGE_MARGIN * limits.min_pan_span_deg
    assert math.degrees(float(np.ptp([v[3] for v in kept]))) >= COVERAGE_MARGIN * limits.min_tilt_span_deg
    tiers = [0] * len(SEQ.tier0_poses) + [1] * len(SEQ.tier1_poses) + [2] * len(SEQ.tier2_poses)
    assert {tiers[index] for index in poses} == {0, 1, 2}


@pytest.mark.parametrize("head_name", list(HEADS))
def test_every_pose_keeps_the_minimum_d435_views(head_name: str) -> None:
    """Every shipped pose keeps at least MIN_GOOD_OFFSETS views the D435 min-Z and colour FOV rule passes (nominal
    intrinsics, `PoseChecker.check_view` as the sequencer runs it), so the sequencer visits every pose. Tier 1 pose 8
    at wrist_flex -0.9 / wrist_roll -0.6 filled more than the 360 px image height and lost all four views."""
    usable = [0] * len(set_poses())
    for index, *_, entries in d435_views(head_name):
        usable[index] += not entries
    short = {name: count for name, count in zip(pose_names(), usable, strict=True) if count < MIN_GOOD_OFFSETS}
    assert short == {}, short


@pytest.mark.parametrize("head_name", list(HEADS))
def test_every_head_offset_is_usable_on_most_poses(head_name: str) -> None:
    """Each head offset keeps its view on at least MIN_OFFSET_POSE_SHARE of the shipped poses under the D435 rule: the
    former +6 deg tilt offset pushed the board out of the 42 deg tall colour image on 26 of 33 poses
    (shipped head)."""
    poses = len(set_poses())
    for offset, degrees in enumerate(SEQ.head_offsets_deg):
        kept = sum(1 for _, view_offset, *_, entries in d435_views(head_name) if view_offset == offset and not entries)
        assert kept >= MIN_OFFSET_POSE_SHARE * poses, (degrees, kept, poses)


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
    truth = SYN.synth_truth(head_urdf, cfg=FIT, mount=(FIT.board_mount.xyz_init, FIT.board_mount.rpy_init))
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
