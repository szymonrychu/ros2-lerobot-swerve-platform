"""The shipped head-from-arm arm pose set: safe by FK, the board in the D435's range, coverage and a passing synthetic
fit.

Everything runs on nodes/web_ui/urdf/so101_arm.urdf and nodes/web_ui/urdf/head.urdf, both as shipped and with the
applied floor-fit chain of 2026-10-22 as overrides (`tests/head_arm_synthetic.py::synth_head_urdf`). The poses are
stored as measured (follower) angles, the space `move_arm_joints` takes; every FK applies `urdf = measured + offset`
with `fit.joint_offsets_rad`.

The home pose the sequence starts from is `HOME_POSE`: `scripts/gen_nav_footprint.yaml` `parked_arm_joints`, the
repo record of the home that mcp_server's `arm.home_file` (/var/lib/ros2/arm/home.yaml) holds and `arm_home` parks
at. The board is the ARM board on its edge handle (a tab off the plate's bottom edge with the grip post on the pattern
side), mounted at the handle CAD seed (markers/charuco-arm-6x5-28mm.json), not yet fit-solved (design D47): the safety
tests check the plate plus the handle's post, gusset and tab (`board_mount.handle_boxes_mm`) with the board margin
`margin_m + provisional_margin_m` (plus the lattice miss bound) and the mount shifted by +-1 cm and turned by +-5 deg
about each gripper axis. The camera is the head D435 colour camera (`d435_color_optical_frame`) with the nominal 640x360
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
from ros2_common.head_rgbd_calibration.arm_pose_check import (
    FOV_RULE,
    HEAD_CLEARANCE_RULE,
    MIN_Z_RULE,
    HeadRegion,
    PoseChecker,
    head_envelope,
)
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
LIVE_HEAD_FIXTURE = Path(__file__).resolve().parent / "fixtures" / "head_2026-11-11.urdf"  # live applied head
# Pan-axis fit of the ground board fit 2026-11-11 (head_pan origin xy, m) and its d435_joint x (m).
PAN_AXIS_DX_M, PAN_AXIS_DY_M = -0.031938113, -0.015897602
PAN_AXIS_D435_X_M = 0.037844510
HEAD_PAN_JOINT, HEAD_TILT_JOINT, D435_JOINT = "head_pan", "head_tilt", "d435_joint"
LIVE_REFUSAL = (
    "link_clearance: upper arm to head clearance"  # main's envelope refusal of tier 1 pose 8 -> tier 2 pose 0
)
CONTACT_PAN_DEG = -85.0  # held pan near the real head's contact zone of the tier 2 poses (.logs/tier2-path-debug)
CONTACT_SPAN_PAN_RAD = -1.48  # head move target pan into that zone
HEAD_OBSTACLE_PREFIX = "head "  # region obstacles are named "head <link>"
HEADROOM_PROBE_M = 0.005  # extra link margin that refuses today's full-range legs (3.2 mm headroom over margin_m)


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


def joint_origin(root: ET.Element, name: str) -> ET.Element:
    """The <origin> element of a URDF joint.

    :param root: parsed URDF root
    :param name: joint name
    :return: the joint's origin element
    """
    return next(j for j in root.findall("joint") if j.get("name") == name).find("origin")


def pan_axis_head(urdf: str, dx_m: float, dy_m: float, d435_x_m: float) -> str:
    """The head URDF with the pan axis moved by (dx, dy) in head_pan_base, the head itself kept in place.

    head_pan origin xy becomes (dx, dy) and head_tilt origin xy moves by (-dx, -dy), so at pan 0 the tilt axis stays
    where it was and only the pan axis moves (the pan-axis fit of `max_pan_axis_delta_mm`); d435_joint x is set.

    :param urdf: head URDF text
    :param dx_m: head_pan origin x (m)
    :param dy_m: head_pan origin y (m)
    :param d435_x_m: d435_joint origin x (m)
    :return: the modified head URDF text
    """
    root = ET.fromstring(urdf)
    pan = joint_origin(root, HEAD_PAN_JOINT)
    pan_xyz = [float(v) for v in pan.get("xyz").split()]
    pan.set("xyz", f"{dx_m} {dy_m} {pan_xyz[2]}")
    tilt = joint_origin(root, HEAD_TILT_JOINT)
    tilt_xyz = [float(v) for v in tilt.get("xyz").split()]
    tilt.set("xyz", f"{tilt_xyz[0] - dx_m} {tilt_xyz[1] - dy_m} {tilt_xyz[2]}")
    d435 = joint_origin(root, D435_JOINT)
    d435_xyz = [float(v) for v in d435.get("xyz").split()]
    d435.set("xyz", f"{d435_x_m} {d435_xyz[1]} {d435_xyz[2]}")
    return ET.tostring(root, encoding="unicode")


def centred_pan_axis_head(urdf: str) -> str:
    """The head URDF with the pan axis put back on the mount, the head itself kept in place.

    head_pan origin xy becomes (0, 0) and head_tilt origin xy moves by the old head_pan origin xy, so at pan 0 the tilt
    axis stays where it was: the base `pan_axis_head` expects (zero pan axis offset).

    :param urdf: head URDF text
    :return: the modified head URDF text
    """
    root = ET.fromstring(urdf)
    pan = joint_origin(root, HEAD_PAN_JOINT)
    pan_xyz = [float(v) for v in pan.get("xyz").split()]
    pan.set("xyz", f"0 0 {pan_xyz[2]}")
    tilt = joint_origin(root, HEAD_TILT_JOINT)
    tilt_xyz = [float(v) for v in tilt.get("xyz").split()]
    tilt.set("xyz", f"{tilt_xyz[0] + pan_xyz[0]} {tilt_xyz[1] + pan_xyz[1]} {tilt_xyz[2]}")
    return ET.tostring(root, encoding="unicode")


LIVE_HEAD = LIVE_HEAD_FIXTURE.read_text()
LIVE_CENTRED_HEAD = centred_pan_axis_head(LIVE_HEAD)
PAN_AXIS_MAX_SCALE = FIT.limits.max_pan_axis_delta_mm / 1000.0 / math.hypot(PAN_AXIS_DX_M, PAN_AXIS_DY_M)
# The fit's largest accepted pan-axis delta along today's direction, on the live mount (decision T-114 #369: the
# full-range envelope refuses it, the held-head band model passes it).
MAX_PAN_AXIS_HEAD = pan_axis_head(
    LIVE_CENTRED_HEAD, PAN_AXIS_DX_M * PAN_AXIS_MAX_SCALE, PAN_AXIS_DY_M * PAN_AXIS_MAX_SCALE, PAN_AXIS_D435_X_M
)
LIVE_HEAD_NAME = "live-20261111"
BAND_HEADS = {**SAFETY_HEADS, LIVE_HEAD_NAME: LIVE_HEAD}


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
    """The ARM board on its edge handle in the closed gripper (user m540, decision 297): the CAD seed of the handle
    metadata, board z (into the plate, away from the printed face) along the tool direction so the pattern faces back
    toward the wrist and the head, and board x (the long edges) along the jaw opening axis; both yamls name the handle
    CAD."""
    meta = json.loads((REPO_ROOT / "markers" / "charuco-arm-6x5-28mm.json").read_text())
    assert FIT.board == ARM_BOARD
    assert FIT.board_mount.xyz_init == pytest.approx(meta["board_mount"]["xyz_init"], abs=1e-6)
    assert FIT.board_mount.rpy_init == pytest.approx(meta["board_mount"]["rpy_init"], abs=1e-6)
    mount = pose_from_xyz_rpy(FIT.board_mount.xyz_init, FIT.board_mount.rpy_init)
    assert mount[:3, 2] == pytest.approx([0.0, 0.0, -1.0], abs=1e-6)  # board z along the tool direction
    assert mount[:3, 0] == pytest.approx([1.0, 0.0, 0.0], abs=1e-6)  # board x along the jaw opening axis
    handle = meta["handle_in_board"]
    assert mount[:3, :3] @ handle["tool_axis"] == pytest.approx([0.0, 0.0, -1.0], abs=1e-6)
    assert mount[:3, :3] @ handle["open_axis"] == pytest.approx([1.0, 0.0, 0.0], abs=1e-6)
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


@cache
def cached_checker(head_urdf: str, mount_json: str, sag_m: float = 0.0, extra_margin_m: float = 0.0) -> PoseChecker:
    """One pose checker per head, board mount, board sag and link margin, reused by every check on them (region
    obstacle sets are cached per checker).

    :param head_urdf: head URDF text
    :param mount_json: the board mount, `BoardMountConfig.model_dump_json()`
    :param sag_m: board sag margin (`park_board_sag_margin_m`) added to the board margin and to `min_board_height_m` (0 for the sequence)
    :param extra_margin_m: added to `pose_check.margin_m` (0 for the shipped link margin)
    :return: the pose checker, board margin `board_margin_m(nominal_mount=True) + sag_m`
    """
    fit = FIT.model_copy(update={"board_mount": BoardMountConfig.model_validate_json(mount_json)})
    pose_check = SEQ.pose_check.model_copy(
        update={
            "min_board_height_m": SEQ.pose_check.min_board_height_m + sag_m,
            "margin_m": SEQ.pose_check.margin_m + extra_margin_m,
        }
    )
    return PoseChecker(ARM_URDF, head_urdf, fit, pose_check, SEQ.board_margin_m(nominal_mount=True) + sag_m)


def nominal_checker(head_urdf: str) -> PoseChecker:
    """The cached checker of a head at the nominal board mount."""
    return cached_checker(head_urdf, FIT.board_mount.model_dump_json())


@cache
def candidate_aims(head_urdf: str) -> dict[str, list[tuple[float, float]]]:
    """Per tier pose name ("tier <t> pose <i>"): the (pan, tilt) rad the sequencer may command there.

    `aim_head` at the nominal board centre (clamped to the head limits) plus each `SEQ.head_offsets_deg`; an offset
    that leaves the head URDF limits is skipped before it is commanded, so it is not a candidate.

    :param head_urdf: head URDF text
    :return: candidate aims per pose name, in `pose_names` order
    """
    chain = UrdfChain.from_text(head_urdf)
    checker = nominal_checker(head_urdf)
    pan_limits, tilt_limits = chain.limits(PAN), chain.limits(TILT)
    out = {}
    for name, pose in zip(pose_names(), set_poses(), strict=True):
        pan0, tilt0 = aim_head(chain, board_centre_in_base(checker, pose))
        aims = [(pan0 + math.radians(dpan), tilt0 + math.radians(dtilt)) for dpan, dtilt in SEQ.head_offsets_deg]
        out[name] = [
            (pan, tilt)
            for pan, tilt in aims
            if pan_limits[0] <= pan <= pan_limits[1] and tilt_limits[0] <= tilt <= tilt_limits[1]
        ]
    return out


def aims_region(checker: PoseChecker, aims: list[tuple[float, float]]) -> HeadRegion:
    """The bounding (pan, tilt) rectangle of head aims, +-`head_band_deg`, the band clipped to the head limits.

    :param checker: pose checker of the head
    :param aims: (pan, tilt) rad, at least one
    :return: the region
    """
    pans, tilts = [pan for pan, _ in aims], [tilt for _, tilt in aims]
    return checker.head_span((min(pans), min(tilts)), (max(pans), max(tilts)))


def sequencer_model_failures(
    head_urdf: str, mounts: Iterable[tuple[str, BoardMountConfig]]
) -> tuple[list[str], list[str]]:
    """Safety-check entries of the sequence under the sequencer's head model, per mount (spec "Which check uses which
    model"), split by whether the head is held.

    Not held (full-range envelope): the base pose, home -> base, tier 0 pose 0, base -> tier 0 pose 0 and the first
    head move at tier 0 pose 0. Held: every later transit prev -> pose (tier poses and the return to base) holds the
    head inside prev's aims, `check_pose(pose)` and `check_path(prev, pose)` with the band around the bounding
    rectangle of prev's aims; at each later tier pose the head moves from prev's aims to this pose's aims,
    `check_head_move` with the band around the rectangle of both.

    :param head_urdf: head URDF text
    :param mounts: (label, board mount) pairs to check the sequence with
    :return: (held, not held) lists of "<mount> <step>: <entry>" lines per failed check; both empty when safe
    """
    steps = sequence()
    aims = candidate_aims(head_urdf)
    held_failures: list[str] = []
    unheld_failures: list[str] = []
    for mount, board_mount in mounts:
        checker = cached_checker(head_urdf, board_mount.model_dump_json())
        for (name_a, a), (name_b, b) in zip(steps, steps[1:], strict=False):
            rolling = b[WRIST_ROLL] != a[WRIST_ROLL]
            gripper = SEQ.insert_gripper_rad if name_a == "home" else SEQ.grip_rad
            held = aims.get(name_a)
            region = None if held is None else aims_region(checker, held)
            failures = unheld_failures if held is None else held_failures
            failures += [f"{mount} {name_b}: {e}" for e in checker.check_pose(b, gripper, rolling, region)]
            failures += [f"{mount} {name_a} -> {name_b}: {e}" for e in checker.check_path(a, b, gripper, region)]
            if name_b in aims:
                moves = None if held is None else aims_region(checker, [*held, *aims[name_b]])
                entries = checker.check_head_move(b, SEQ.grip_rad, moves)
                failures += [f"{mount} {name_b} head move: {e}" for e in entries]
    return held_failures, unheld_failures


def full_range_legs(head_urdf: str, extra_margin_m: float = 0.0) -> dict[str, list[str]]:
    """Envelope check entries of the legs that never hold the head, nominal mount: home -> base, base -> tier 0
    pose 0, and base -> the stored home (`HOME_POSE`) with the board sag margin (as `check_board_home_path`).

    :param head_urdf: head URDF text
    :param extra_margin_m: added to `pose_check.margin_m` (0 for the shipped link margin)
    :return: entries per leg name; empty lists when every leg passes
    """
    mount_json = FIT.board_mount.model_dump_json()
    checker = cached_checker(head_urdf, mount_json, extra_margin_m=extra_margin_m)
    home = cached_checker(head_urdf, mount_json, SEQ.park_board_sag_margin_m, extra_margin_m)
    return {
        "home -> base": checker.check_path(HOME_POSE, SEQ.base_pose, SEQ.insert_gripper_rad),
        "base -> tier 0 pose 0": checker.check_path(SEQ.base_pose, SEQ.tier0_poses[0], SEQ.grip_rad),
        "base -> home": home.check_path(SEQ.base_pose, HOME_POSE, SEQ.grip_rad),
    }


def live_refusal_path() -> tuple[PoseChecker, dict[str, float], dict[str, float]]:
    """The live head checker (nominal mount) and the transit main refused on it: tier 1 pose 8 -> tier 2 pose 0."""
    return nominal_checker(LIVE_HEAD), SEQ.tier1_poses[-1], SEQ.tier2_poses[0]


def test_live_refusal_reproduced_by_the_envelope() -> None:
    """Main's full-range envelope refuses the live transit tier 1 pose 8 -> tier 2 pose 0 (.logs/tier2-path-debug)."""
    checker, start, target = live_refusal_path()
    entries = checker.check_path(start, target, SEQ.grip_rad)
    assert any(e.startswith(LIVE_REFUSAL) for e in entries), entries


def test_live_refusal_passes_with_the_real_head_band() -> None:
    """With the head held inside tier 1 pose 8's aims (+-band) the real head shapes leave the transit clear."""
    checker, start, target = live_refusal_path()
    region = aims_region(checker, candidate_aims(LIVE_HEAD)[f"tier 1 pose {len(SEQ.tier1_poses) - 1}"])
    assert checker.check_path(start, target, SEQ.grip_rad, head=region) == []


@pytest.mark.parametrize("head_name", list(BAND_HEADS))
def test_sequence_is_safe_under_the_sequencer_head_model(head_name: str) -> None:
    """Every pose, transit and head move with the nominal mount under the sequencer's head model."""
    held, unheld = sequencer_model_failures(BAND_HEADS[head_name], [("nominal", FIT.board_mount)])
    assert held + unheld == [], "\n".join((held + unheld)[:20])


@pytest.mark.parametrize("head_name", list(BAND_HEADS))
def test_sequencer_head_model_safe_under_board_mount_uncertainty(head_name: str) -> None:
    """The sequencer's head model with the mount shifted by +-1 cm and turned by +-5 deg about each gripper axis."""
    held, unheld = sequencer_model_failures(BAND_HEADS[head_name], list(band_mounts())[1:])
    assert held + unheld == [], "\n".join((held + unheld)[:20])


def test_head_band_model_still_refuses_a_colliding_transit() -> None:
    """The real head band is no blanket pass: held near pan -85 deg the head is in the way of tier 2 pose 1, and a
    head move from the aim into that zone is refused."""
    head_urdf = LIVE_HEAD
    checker = nominal_checker(head_urdf)
    pose = SEQ.tier2_poses[1]
    pan, tilt = aim_head(UrdfChain.from_text(head_urdf), board_centre_in_base(checker, pose))
    band = checker.head_band(math.radians(CONTACT_PAN_DEG), tilt)
    entries = checker.check_path(SEQ.tier2_poses[0], pose, SEQ.grip_rad, head=band)
    head_entries = [e for e in entries if e.startswith("link_clearance") and f" to {HEAD_OBSTACLE_PREFIX}" in e]
    assert head_entries, entries
    span = checker.head_span((pan, tilt), (CONTACT_SPAN_PAN_RAD, tilt))
    moves = checker.check_head_move(pose, SEQ.grip_rad, span)
    assert moves and all(e.startswith(HEAD_CLEARANCE_RULE) for e in moves), moves


def test_pan_axis_head_rebuilds_the_live_head() -> None:
    """The live head re-centred plus today's pan-axis fit is the live head again: the construction of the max variant
    is pinned (same joint origins, same full-range envelope)."""
    rebuilt = pan_axis_head(LIVE_CENTRED_HEAD, PAN_AXIS_DX_M, PAN_AXIS_DY_M, PAN_AXIS_D435_X_M)
    live, built = ET.fromstring(LIVE_HEAD), ET.fromstring(rebuilt)
    for joint in (HEAD_PAN_JOINT, HEAD_TILT_JOINT, D435_JOINT):
        expected = [float(v) for v in joint_origin(live, joint).get("xyz").split()]
        got = [float(v) for v in joint_origin(built, joint).get("xyz").split()]
        assert got == pytest.approx(expected, abs=EPS), joint
    live_envelope, built_envelope = head_envelope(LIVE_HEAD, SEQ.pose_check), head_envelope(rebuilt, SEQ.pose_check)
    assert built_envelope.radius == pytest.approx(live_envelope.radius, abs=EPS)
    assert (built_envelope.x, built_envelope.y) == pytest.approx((live_envelope.x, live_envelope.y), abs=EPS)


def test_live_head_full_range_legs_pass_with_today_headroom() -> None:
    """Today's live head (35.66 mm pan-axis delta) passes the legs that use the full-range envelope (home -> base, base
    -> tier 0 pose 0, base -> home with the board sag) with less than `HEADROOM_PROBE_M` to spare (8.2 mm worst
    clearance, 3.2 mm over margin_m): a refit that moves the pan axis ~2 mm further out blocks the sequence start."""
    legs = full_range_legs(LIVE_HEAD)
    assert all(entries == [] for entries in legs.values()), legs
    probed = full_range_legs(LIVE_HEAD, HEADROOM_PROBE_M)
    assert all(entries for entries in probed.values()), probed


def test_max_pan_axis_head_held_band_passes_and_unheld_envelope_moves_fail_safe() -> None:
    """At the fit's largest pan-axis delta (`max_pan_axis_delta_mm`, live mount, today's direction) every check with
    the head held passes under the nominal mount and the board mount band, while every move before the head is held
    (base pose, home -> base, base -> tier 0 pose 0, the first head move, base -> home) is refused by the full-range
    envelope: the known ~38 mm limit (decision T-114 #369), fail safe."""
    mounts = list(band_mounts())
    held, unheld = sequencer_model_failures(MAX_PAN_AXIS_HEAD, mounts)
    assert held == [], "\n".join(held[:20])
    for mount, _ in mounts:
        for step in ("base: ", "home -> base: ", "base -> tier 0 pose 0: ", "tier 0 pose 0 head move: "):
            assert any(e.startswith(f"{mount} {step}") for e in unheld), (mount, step)
    assert full_range_legs(MAX_PAN_AXIS_HEAD)["base -> home"], "board Home not refused"
