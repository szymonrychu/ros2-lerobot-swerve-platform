"""Tests for scripts/markers/handle.py and the ARM board flow of write_charuco: right-angle grip post with gussets on
the board back (decision 296), face-down print, seat in the SO-101 jaws (URDF meshes), metadata JSON and the shipped
outputs."""

import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import pytest
import trimesh
import yaml
from manifold3d import Manifold
from pydantic import ValidationError
from ros2_common.head_rgbd_calibration.chain import pose_from_xyz_rpy

from scripts.markers.charuco import (
    CharucoSpec,
    back_envelope_mm,
    board_config,
    board_metadata,
    make_charuco_board,
    print_bodies,
    write_board_metadata,
    write_charuco,
)
from scripts.markers.config import load_generator_config
from scripts.markers.handle import (
    SWEEP_STEP_RAD,
    HandleSpec,
    board_in_mesh,
    face_down,
    gusset_solid,
    handle_solid,
    nominal_solid,
    post_in_mesh,
    post_solid,
    printed_contact_rad,
    seat_crop,
    shoulder_top_mm,
)
from scripts.markers.jaw_fit import (
    CONTACT_HI_RAD,
    OPEN_AXIS,
    TOOL_AXIS,
    WIDTH_AXIS,
    clearance_mm,
    load_gripper,
    moving_jaw,
    nearest_distance_mm,
    penetration_mm,
    surface_points,
)
from scripts.markers.mesh import box, print_violations, printed_solid, to_manifold, to_trimesh

REPO_ROOT = Path(__file__).resolve().parents[1]
SHIPPED_YAML = REPO_ROOT / "scripts" / "markers_generate.yaml"
ARM_URDF = REPO_ROOT / "nodes" / "web_ui" / "urdf" / "so101_arm.urdf"
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
ARM_NAME = "charuco-arm-6x5-28mm"
CONFIG = load_generator_config(SHIPPED_YAML)
ARM = next(board for board in CONFIG.boards if board.name == ARM_NAME)
GRIPPER = load_gripper(ARM_URDF)
MM_PER_M = 1000.0
VIEW_PX_PER_MM = 10.0
INNER_CORNERS = 20
CONTACT_TOL_MM = 0.05
FLOAT_TOL_MM = 1e-9  # JSON pose round trip (see test_seated_handle_touches_both_jaws_without_penetration)
HANDLE_CUT_MM = 0.5  # mesh z above the plate back (T) from which the printed solid is the post and its gussets
INSERT_GRIPPER_RAD = 0.4  # the sequencer's opening for sliding the handle in (Task 8 DEFAULT_INSERT_GRIPPER_RAD)
MIN_INSERT_CLEARANCE_MM = 6.0  # 3 mm per side with the handle centred between the open jaws
LAND_TOL_MM = 0.4
MIN_LAND_SPREAD_MM = 15.0
PROFILE_STEP_MM = 0.5
LAND_SAMPLES = 200000  # moving jaw surface samples, ~10 per mm2 of face over the channel
CLAMP_CLEARANCE_MM = 0.15  # user decision 282: print allowance on each clamp face
ALLOWANCE_TOL_MM = 0.02
COLUMN_MM = 0.2  # half-size of the vertical probe column through the handle
VOLUME_REL_TOL = 0.02
# Decision 296: the in-line paddle's grip is reused unchanged, only turned to stand on the board back, so the contact
# angles of the in-line handle (Task 6c, 8f0c0a55) stay; the crop now holds the post instead of a plate sliver.
IN_LINE_GRIP = {"contact_rad": 0.07027905529357185, "modelled_contact_rad": 0.06620181229285478}
GRIP_INVARIANT_TOL_RAD = 1e-4
MIN_PLATE_CLEARANCE_MM = 5.0  # jaws to the plate back over the insert opening and the seat sweep
MIN_GUSSET_CLEARANCE_MM = 5.0  # jaws to the gussets over the insert opening and the seat sweep
GUSSET_SLOPE_LIMIT_DEG = 45.0  # gusset top face at most this far from vertical in print orientation
MASS_RANGE_G = (150.0, 200.0)  # plausible board mass: the 157 g plate and post plus the gussets
AXIS_TOL = 1e-9
PROBE_MM = 0.5  # column offsets inside the post and the gussets
JAW_SAMPLES = 5000  # jaw surface samples for the jaw-to-gusset distance (plus every jaw vertex)
TOOL_POINT_TOL_MM = 2.0  # the tool point lies on the fixed jaw face plane, just off the printed bed face
CROSS_STEP_RAD = 0.05  # gripper angle step of the jaw-to-gusset clearance sweep (penetration uses SWEEP_STEP_RAD)


@pytest.fixture(scope="module")
def arm_files(tmp_path_factory: pytest.TempPathFactory) -> dict[str, Path]:
    """Write the shipped ARM board once per module.

    Args:
        tmp_path_factory (pytest.TempPathFactory): pytest temp dirs.

    Returns:
        dict[str, Path]: Written paths keyed by suffix ("-base.stl", "-modules.stl", ".json", ...).
    """
    stem = tmp_path_factory.mktemp("arm") / ARM.name
    paths = write_charuco(ARM, CONFIG.stl, stem, GRIPPER)
    return {path.name[len(ARM.name) :]: path for path in paths}


@pytest.fixture(scope="module")
def arm_bodies(arm_files: dict[str, Path]) -> tuple[trimesh.Trimesh, trimesh.Trimesh]:
    """The ARM base and module bodies as write_charuco builds them (print_bodies), checked against the written STLs.

    The bodies come from print_bodies, not from the STLs read back: checker cells touch diagonally, and an STL
    keeps no vertex identity, so trimesh merges those corners into shared edges and the read-back bodies are not
    watertight (see test_flush_inlay_board_is_flat_and_watertight). The STLs hold the same triangles.

    Args:
        arm_files (dict[str, Path]): Written ARM files.

    Returns:
        tuple[trimesh.Trimesh, trimesh.Trimesh]: (base, modules) in print coordinates (mm).
    """
    bodies = print_bodies(ARM, CONFIG.stl)
    for body, suffix in zip(bodies, ("-base.stl", "-modules.stl"), strict=True):
        written = trimesh.load(arm_files[suffix])
        assert written.bounds == pytest.approx(body.bounds, abs=1e-4)
        assert written.volume == pytest.approx(body.volume, rel=1e-6)
    return bodies


@pytest.fixture(scope="module")
def arm_metadata(arm_files: dict[str, Path]) -> dict[str, Any]:
    """The written ARM metadata JSON.

    Args:
        arm_files (dict[str, Path]): Written ARM files.

    Returns:
        dict[str, Any]: Parsed JSON.
    """
    return json.loads(arm_files[".json"].read_text())


def mesh_in_gripper_link(meta: dict[str, Any]) -> np.ndarray:
    """Print-mesh pose in gripper_link from the JSON handle_in_board.

    Args:
        meta (dict[str, Any]): Board metadata.

    Returns:
        np.ndarray: 4x4 transform, mm.
    """
    hib = meta["handle_in_board"]
    board_in_gripper = np.linalg.inv(pose_from_xyz_rpy(hib["gripper_link_xyz_m"], hib["gripper_link_rpy_rad"]))
    board_in_gripper[:3, 3] *= MM_PER_M
    return board_in_gripper @ np.linalg.inv(
        board_in_mesh(ARM.plate_mm, ARM.margin_mm, ARM.pattern_face, ARM.thickness_mm)
    )


def seated_handle(
    meta: dict[str, Any], bodies: tuple[trimesh.Trimesh, trimesh.Trimesh], nominal: bool = True
) -> trimesh.Trimesh:
    """The handle part of the nominal (as grown after printing) or the printed solid moved into gripper_link with
    handle_in_board.

    The solid is cut at mesh z = T + HANDLE_CUT_MM (capped, watertight): the post and its gussets, so surface samples
    are dense where the jaws are.

    Args:
        meta (dict[str, Any]): Board metadata.
        bodies (tuple[trimesh.Trimesh, trimesh.Trimesh]): (base, modules).
        nominal (bool): True for the nominal solid (clamp faces grown back by clamp_clearance_mm), False for the
            printed solid as exported.

    Returns:
        trimesh.Trimesh: The seated handle (mm, gripper_link).
    """
    solid = printed_solid(*bodies)
    if nominal:
        assert ARM.handle is not None
        solid = nominal_solid(solid, ARM.handle, ARM.plate_mm, ARM.thickness_mm)
    return post_crop(solid).apply_transform(mesh_in_gripper_link(meta))


def post_crop(solid: trimesh.Trimesh) -> trimesh.Trimesh:
    """The solid above mesh z = T + HANDLE_CUT_MM (the post and its gussets), capped.

    Args:
        solid (trimesh.Trimesh): Watertight solid in mesh coordinates (mm).

    Returns:
        trimesh.Trimesh: The capped crop (mm).
    """
    origin = (0.0, 0.0, ARM.thickness_mm + HANDLE_CUT_MM)
    return trimesh.intersections.slice_mesh_plane(solid, (0.0, 0.0, 1.0), origin, cap=True)


def handle_frame(meta: dict[str, Any]) -> np.ndarray:
    """Pose of the handle frame (x across the post, y along it toward the plate, z from the bed face) in gripper_link.

    Args:
        meta (dict[str, Any]): Board metadata.

    Returns:
        np.ndarray: 4x4 transform, mm.
    """
    assert ARM.handle is not None
    return mesh_in_gripper_link(meta) @ post_in_mesh(ARM.handle, ARM.plate_mm, ARM.thickness_mm)


def bottom_view(modules: trimesh.Trimesh, spec: CharucoSpec) -> np.ndarray:
    """Rasterise the module faces on the bed (normal z < -0.5 at z = 0) as seen from below, dark on white.

    Seen from below with mesh +y up, image right is mesh -x: pixel (W - x, H - y) * VIEW_PX_PER_MM.

    Args:
        modules (trimesh.Trimesh): Module body in print coordinates.
        spec (CharucoSpec): The board.

    Returns:
        np.ndarray: (H, W) uint8 image.
    """
    width_mm, height_mm = spec.plate_mm
    width, height = round(width_mm * VIEW_PX_PER_MM), round(height_mm * VIEW_PX_PER_MM)
    image = np.full((height, width), 255, dtype=np.uint8)
    for face, normal in zip(modules.faces, modules.face_normals, strict=True):
        pts = modules.vertices[face]
        if normal[2] < -0.5 and np.allclose(pts[:, 2], 0.0, atol=1e-4):
            xy = np.stack([(width_mm - pts[:, 0]) * VIEW_PX_PER_MM, (height_mm - pts[:, 1]) * VIEW_PX_PER_MM], axis=1)
            cv2.fillConvexPoly(image, np.round(xy).astype(np.int32), 0)
    return image


def view_to_mesh_xy(corners: np.ndarray, spec: CharucoSpec) -> np.ndarray:
    """Bottom-view pixels back to mesh (x, y) in mm.

    Args:
        corners (np.ndarray): (N, 2) pixel coordinates.
        spec (CharucoSpec): The board.

    Returns:
        np.ndarray: (N, 2) mesh coordinates (mm).
    """
    width_mm, height_mm = spec.plate_mm
    return np.stack([width_mm - corners[:, 0] / VIEW_PX_PER_MM, height_mm - corners[:, 1] / VIEW_PX_PER_MM], axis=1)


def detect(image: np.ndarray, spec: CharucoSpec) -> tuple[np.ndarray, np.ndarray]:
    """Run CharucoDetector.

    Args:
        image (np.ndarray): Grey image.
        spec (CharucoSpec): The board.

    Returns:
        tuple[np.ndarray, np.ndarray]: (corners (N, 2) px, ids (N,)).
    """
    corners, ids, _, _ = cv2.aruco.CharucoDetector(make_charuco_board(spec)).detectBoard(image)
    assert corners is not None and ids is not None
    return corners.reshape(-1, 2), ids.reshape(-1)


def bare_plate_top(base: trimesh.Trimesh) -> float:
    """Highest base vertex over the plate outside the post and gusset footprints (the bare plate back).

    Args:
        base (trimesh.Trimesh): Base body.

    Returns:
        float: z (mm).
    """
    h = ARM.handle
    assert h is not None
    lo, hi = to_trimesh(
        post_solid(h, ARM.plate_mm, ARM.thickness_mm) + gusset_solid(h, ARM.plate_mm, ARM.thickness_mm)
    ).bounds
    v = base.vertices
    outside = np.any((v[:, :2] < lo[:2] - PROBE_MM) | (v[:, :2] > hi[:2] + PROBE_MM), axis=1)
    return float(v[outside][:, 2].max())


def flatten(value: Any, prefix: str = "") -> dict[str, Any]:
    """Nested dicts and lists to a flat {dotted.key: leaf} dict.

    Args:
        value (Any): JSON value.
        prefix (str): Key prefix.

    Returns:
        dict[str, Any]: Leaves by dotted path.
    """
    if isinstance(value, dict):
        return {k: v for key, item in value.items() for k, v in flatten(item, f"{prefix}{key}.").items()}
    if isinstance(value, list):
        return {k: v for i, item in enumerate(value) for k, v in flatten(item, f"{prefix}{i}.").items()}
    return {prefix.rstrip("."): value}


def client_node(name: str) -> dict[str, Any]:
    """Parsed config of a client node in ansible/group_vars/client.yml.

    Args:
        name (str): ros2_nodes name.

    Returns:
        dict[str, Any]: The node's config YAML.
    """
    nodes = yaml.safe_load(CLIENT_VARS.read_text())["ros2_nodes"]
    return yaml.safe_load(next(n for n in nodes if n["name"] == name)["config"])


@pytest.mark.parametrize(
    ("update", "field"),
    [
        ({"length_mm": 24.0}, "length_mm"),
        ({"length_mm": 41.0}, "length_mm"),
        ({"clearance_mm": 0.5}, "clearance_mm"),
        ({"channel_tip_mm": 13.0}, "channel_tip_mm"),
        ({"channel_end_mm": 1.0}, "channel_end_mm"),
        ({"thickness_end_mm": 1.0}, "thickness_end_mm"),
        ({"squeeze_rad": 0.03}, "squeeze_rad"),
        ({"clamp_clearance_mm": -0.01}, "clamp_clearance_mm"),
        ({"clamp_clearance_mm": 0.31}, "clamp_clearance_mm"),
        ({"gusset_height_mm": 0.0}, "gusset_height_mm"),
        ({"gusset_height_mm": 37.0}, "gusset_height_mm"),  # taller than the post (neck plus grip)
        ({"gusset_reach_mm": 0.0}, "gusset_reach_mm"),
        ({"gusset_reach_mm": 36.1}, "gusset_reach_mm"),  # flatter than 45 deg from vertical
        ({"gusset_thickness_mm": 0.0}, "gusset_thickness_mm"),
        ({"gusset_thickness_mm": 24.8}, "gusset_thickness_mm"),  # thicker than the post section
        ({"rib_width_mm": 4.0}, "rib_width_mm"),  # the twin ribs are gone (decision 296)
        ({"bogus": 1}, "bogus"),
    ],
)
def test_handle_spec_refuses_out_of_range(update: dict[str, Any], field: str) -> None:
    assert ARM.handle is not None
    with pytest.raises(ValidationError, match=field):
        HandleSpec.model_validate({**ARM.handle.model_dump(), **update})


def test_handle_spec_extent_and_width() -> None:
    h = ARM.handle
    assert h is not None
    assert h.extent_mm == h.neck_mm + h.length_mm
    assert h.width_mm == h.channel_end_mm + 2 * h.rail_width_mm


def test_pattern_face_is_required_and_arm_prints_face_down() -> None:
    spec = ARM.model_dump()
    del spec["pattern_face"]
    with pytest.raises(ValidationError, match="pattern_face"):
        CharucoSpec.model_validate(spec)
    assert ARM.pattern_face == "bottom"
    assert next(b for b in CONFIG.boards if b.name != ARM_NAME).pattern_face == "top"


def test_charuco_spec_refuses_gussets_off_the_plate() -> None:
    assert ARM.handle is not None
    spec = ARM.model_dump()
    reach = ARM.plate_mm[0] / 2 - ARM.handle.width_mm / 2 + 1.0
    spec["handle"] = {**spec["handle"], "gusset_reach_mm": reach, "gusset_height_mm": reach, "neck_mm": reach}
    with pytest.raises(ValidationError, match="gusset_reach_mm"):
        CharucoSpec.model_validate(spec)


def test_charuco_spec_needs_the_pattern_face_down_for_a_post() -> None:
    with pytest.raises(ValidationError, match="pattern_face"):
        CharucoSpec.model_validate({**ARM.model_dump(), "pattern_face": "top"})


def test_face_down_turns_about_the_plate_y_axis() -> None:
    turned = to_trimesh(face_down(box(1.0, 2.0, 0.0, 3.0, 5.0, 1.0), (10.0, 20.0), 4.0))
    assert turned.bounds == pytest.approx(np.array([[7.0, 2.0, 3.0], [9.0, 5.0, 4.0]]), abs=1e-5)


def test_board_in_mesh_bottom_is_the_face_down_top() -> None:
    w, h = ARM.plate_mm
    m, t = ARM.margin_mm, ARM.thickness_mm
    top = board_in_mesh(ARM.plate_mm, m, "top", t)
    bottom = board_in_mesh(ARM.plate_mm, m, "bottom", t)
    assert top[:3, :3] == pytest.approx(np.diag([1.0, -1.0, -1.0]))
    assert top[:3, 3] == pytest.approx([m, h - m, t])
    turn = np.diag([-1.0, 1.0, -1.0, 1.0])
    turn[:3, 3] = [w, 0.0, t]
    assert bottom == pytest.approx(turn @ top)


def test_handle_solid_is_the_grip_in_the_handle_frame() -> None:
    h = ARM.handle
    assert h is not None
    solid = to_trimesh(handle_solid(h))
    expected = [
        [-h.width_mm / 2, -h.extent_mm, 0.0],
        [h.width_mm / 2, 0.0, h.thickness_end_mm + h.rail_height_mm],
    ]
    assert solid.bounds == pytest.approx(np.array(expected), abs=1e-3)


def test_post_in_mesh_stands_the_handle_on_the_plate_centre() -> None:
    """Handle y (toward the plate) is mesh -z, the free end points away from the plate back (mesh z = T); the
    footprint centre (x = 0, z = shoulder top / 2) lands on the plate centre; handle z (OPEN_AXIS) is mesh +y."""
    h = ARM.handle
    assert h is not None
    pose = post_in_mesh(h, ARM.plate_mm, ARM.thickness_mm)
    assert pose[:3, :3] == pytest.approx(np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]]))
    assert np.linalg.det(pose[:3, :3]) == pytest.approx(1.0)
    centre = pose @ np.array([0.0, 0.0, shoulder_top_mm(h) / 2, 1.0])
    assert centre[:3] == pytest.approx([ARM.plate_mm[0] / 2, ARM.plate_mm[1] / 2, ARM.thickness_mm])
    free_end = pose @ np.array([0.0, -h.extent_mm, 0.0, 1.0])
    assert free_end[2] == pytest.approx(ARM.thickness_mm + h.extent_mm)


def test_post_solid_alone_is_printable_and_stands_on_the_back() -> None:
    """The post prints standing on the plate back: its tapered walls lean a few degrees, never past 45 deg."""
    h = ARM.handle
    assert h is not None
    solid = to_trimesh(post_solid(h, ARM.plate_mm, ARM.thickness_mm))
    assert print_violations(solid) == []
    cx, cy, t = ARM.plate_mm[0] / 2, ARM.plate_mm[1] / 2, ARM.thickness_mm
    low_y = cy - shoulder_top_mm(h) / 2
    expected = [
        [cx - h.width_mm / 2, low_y, t],
        [cx + h.width_mm / 2, low_y + h.thickness_end_mm + h.rail_height_mm, t + h.extent_mm],
    ]
    assert solid.bounds == pytest.approx(np.array(expected), abs=1e-3)
    assert solid.bounds[0][2] == pytest.approx(t, abs=1e-3)


def test_arm_board_union_is_watertight_and_overhang_free(arm_bodies: tuple[trimesh.Trimesh, trimesh.Trimesh]) -> None:
    base, modules = arm_bodies
    assert print_violations(printed_solid(base, modules)) == []


def test_face_down_pattern_reads_from_below(arm_bodies: tuple[trimesh.Trimesh, trimesh.Trimesh]) -> None:
    _, modules = arm_bodies
    assert modules.bounds[0][2] == pytest.approx(0.0, abs=1e-4)
    corners, ids = detect(bottom_view(modules, ARM), ARM)
    assert len(ids) == INNER_CORNERS
    board = board_in_mesh(ARM.plate_mm, ARM.margin_mm, "bottom", ARM.thickness_mm)
    expected = (board[:3, :3] @ (make_charuco_board(ARM).getChessboardCorners().T * MM_PER_M)).T + board[:3, 3]
    for corner, ident in zip(view_to_mesh_xy(corners, ARM), ids, strict=True):
        assert np.linalg.norm(corner - expected[ident][:2]) < 0.2


def test_handle_dimensions_follow_the_spec(arm_bodies: tuple[trimesh.Trimesh, trimesh.Trimesh]) -> None:
    """The board stays inside its plate outline; the post rises neck plus grip above the back, perpendicular to the
    plate at its centre; the rest of the back is bare."""
    base, _ = arm_bodies
    h = ARM.handle
    assert h is not None
    t = ARM.thickness_mm
    assert base.bounds[0] == pytest.approx([0.0, 0.0, 0.0], abs=1e-3)
    assert base.bounds[1] == pytest.approx([*ARM.plate_mm, t + h.extent_mm], abs=1e-3)
    assert column_z(to_manifold(base), ARM.plate_mm[0] / 2, ARM.plate_mm[1] / 2)[1] == pytest.approx(
        t + h.extent_mm, abs=1e-3
    )
    web_low = (ARM.plate_mm[1] - h.gusset_thickness_mm) / 2 - PROBE_MM  # mesh y below the gusset webs
    for depth in (PROBE_MM, h.extent_mm / 2, h.extent_mm - PROBE_MM):  # the post is straight, centred at every level
        level = box(0.0, 0.0, t + depth - COLUMN_MM, ARM.plate_mm[0], web_low, t + depth + COLUMN_MM)
        lo, hi = to_trimesh(to_manifold(base) ^ level).bounds
        assert [lo[0], hi[0]] == pytest.approx([(ARM.plate_mm[0] - s * h.width_mm) / 2 for s in (1, -1)], abs=1e-3)
        assert lo[1] == pytest.approx((ARM.plate_mm[1] - shoulder_top_mm(h)) / 2, abs=1e-3)
    assert bare_plate_top(base) == pytest.approx(t, abs=1e-3)


def test_arm_board_is_one_piece(arm_bodies: tuple[trimesh.Trimesh, trimesh.Trimesh]) -> None:
    assert len(to_manifold(arm_bodies[0]).decompose()) == 1


def test_seated_handle_touches_both_jaws_without_penetration(
    arm_metadata: dict[str, Any], arm_bodies: tuple[trimesh.Trimesh, trimesh.Trimesh]
) -> None:
    """Both directions of penetration (penetration_mm only tests `other` inside `solid`) and contact on both jaws,
    on the nominal surfaces (the printed solid with its clamp faces grown back, user decision 283).

    contact_angle_rad bisects to the angle where the jaw penetration is CONTACT_TOL_MM to float precision, and the
    pose read back from the JSON (rpy of +-pi/2, cos(pi/2) = 6e-17) moves the handle by ~1e-14 mm, so the
    penetration bound carries FLOAT_TOL_MM.
    """
    handle = seated_handle(arm_metadata, arm_bodies)
    moving = moving_jaw(GRIPPER, arm_metadata["grip"]["contact_rad"])
    for jaw in (GRIPPER.fixed, moving):
        assert penetration_mm(handle, jaw) <= CONTACT_TOL_MM + FLOAT_TOL_MM
        assert penetration_mm(jaw, handle) <= CONTACT_TOL_MM + FLOAT_TOL_MM
        assert clearance_mm(handle, jaw) <= CONTACT_TOL_MM
    assert arm_metadata["grip"]["grip_rad"] == pytest.approx(
        arm_metadata["grip"]["contact_rad"] - ARM.handle.squeeze_rad  # type: ignore[union-attr]
    )


def test_contact_angle_is_near_parallel(arm_metadata: dict[str, Any]) -> None:
    assert 0.0 < arm_metadata["grip"]["contact_rad"] < np.radians(10.0)


def test_top_plane_bears_on_the_moving_jaw_lands(arm_metadata: dict[str, Any]) -> None:
    """The SO-101 jaw faces are stepped (2 mm lands), so the sloped top plane cannot follow the gap at every depth.

    It touches the land ends instead: the moving jaw surface over the channel (at the contact angle, in mesh
    coordinates) never dips below the top plane beyond LAND_TOL_MM and comes within LAND_TOL_MM of it at grip
    depths at least MIN_LAND_SPREAD_MM apart, so the moving jaw bears along the grip, not at one line. The margin
    is taken per PROFILE_STEP_MM depth bin from the jaw surface across the channel, not from mid-plane rays
    (jaw_gap_mm): the moving finger has a hole on its mid-plane about 28 mm behind the tip. Coordinates are the handle
    frame (`post_in_mesh`).
    """
    h = ARM.handle
    assert h is not None
    jaw = moving_jaw(GRIPPER, arm_metadata["grip"]["contact_rad"])
    points = trimesh.transform_points(surface_points(jaw, LAND_SAMPLES), np.linalg.inv(handle_frame(arm_metadata)))
    grip_depth = -h.neck_mm - points[:, 1]  # behind the shoulder face (handle y = -neck)
    over_channel = np.abs(points[:, 0]) <= h.channel_tip_mm / 2
    keep = over_channel & (grip_depth >= 0.0) & (grip_depth <= h.length_mm)
    points, grip_depth = points[keep], grip_depth[keep]
    # nominal top plane: the nominal bed face lies clamp_clearance_mm below the printed one (handle z = -c)
    top = (
        h.thickness_tip_mm + (h.thickness_end_mm - h.thickness_tip_mm) * grip_depth / h.length_mm
    ) - h.clamp_clearance_mm
    bins = np.floor(grip_depth / PROFILE_STEP_MM).astype(int)
    depths = np.unique(bins)
    margins_mm = np.array([float((points[bins == b, 2] - top[bins == b]).min()) for b in depths])
    assert margins_mm.min() >= -LAND_TOL_MM
    touching = depths[margins_mm <= LAND_TOL_MM] * PROFILE_STEP_MM
    assert touching.max() - touching.min() >= MIN_LAND_SPREAD_MM


def test_insert_opening_clears_the_handle(
    arm_metadata: dict[str, Any], arm_bodies: tuple[trimesh.Trimesh, trimesh.Trimesh]
) -> None:
    handle = seated_handle(arm_metadata, arm_bodies)
    assert clearance_mm(handle, moving_jaw(GRIPPER, INSERT_GRIPPER_RAD)) >= MIN_INSERT_CLEARANCE_MM


def test_metadata_mass_and_mount(
    arm_metadata: dict[str, Any], arm_bodies: tuple[trimesh.Trimesh, trimesh.Trimesh]
) -> None:
    base, modules = arm_bodies
    meta = arm_metadata
    assert meta["name"] == ARM.name
    assert meta["plate_mm"] == list(ARM.plate_mm)
    assert meta["volume_cm3"] == pytest.approx((base.volume + modules.volume) / 1000.0, rel=1e-6)
    assert meta["mass_g"] == pytest.approx((base.volume + modules.volume) / 1000.0 * 1.24, rel=1e-6)
    assert meta["board"] == board_config(ARM).model_dump()
    inverse = np.linalg.inv(pose_from_xyz_rpy(meta["board_mount"]["xyz_init"], meta["board_mount"]["rpy_init"]))
    assert inverse[:3, 3] == pytest.approx(meta["handle_in_board"]["gripper_link_xyz_m"], abs=1e-9)
    hib = pose_from_xyz_rpy(
        meta["handle_in_board"]["gripper_link_xyz_m"], meta["handle_in_board"]["gripper_link_rpy_rad"]
    )
    assert hib[:3, :3] == pytest.approx(inverse[:3, :3], abs=1e-9)


def test_tool_point_is_the_tool_offset_in_the_board_frame(arm_metadata: dict[str, Any]) -> None:
    h = ARM.handle
    assert h is not None
    hib = arm_metadata["handle_in_board"]
    gripper_in_board = pose_from_xyz_rpy(hib["gripper_link_xyz_m"], hib["gripper_link_rpy_rad"])
    tool = GRIPPER.tool.copy()
    tool[:3, 3] /= MM_PER_M
    expected = gripper_in_board @ tool @ np.array([*h.tool_offset_m, 1.0])
    assert hib["tool_point_m"] == pytest.approx(expected[:3], abs=1e-9)


def test_write_board_metadata_sorted_and_indented(tmp_path: Path) -> None:
    path = write_board_metadata({"b": 1, "a": {"d": 2.5, "c": [1, 2]}}, tmp_path / "board")
    assert path == tmp_path / "board.json"
    assert path.read_text() == json.dumps({"a": {"c": [1, 2], "d": 2.5}, "b": 1}, sort_keys=True, indent=2) + "\n"


def test_write_charuco_needs_the_gripper_for_a_handle(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="gripper"):
        write_charuco(ARM, CONFIG.stl, tmp_path / ARM.name, None)
    assert not list(tmp_path.iterdir())


def test_board_metadata_seat_is_reproducible(
    arm_metadata: dict[str, Any], arm_bodies: tuple[trimesh.Trimesh, trimesh.Trimesh]
) -> None:
    """board_metadata called again on the print bodies gives the JSON write_charuco wrote."""
    again = board_metadata(ARM, *arm_bodies, GRIPPER)
    assert flatten(again) == pytest.approx(flatten(arm_metadata), abs=1e-6)


def test_shipped_arm_outputs_are_current(arm_metadata: dict[str, Any]) -> None:
    shipped = json.loads((REPO_ROOT / "markers" / f"{ARM_NAME}.json").read_text())
    assert flatten(shipped) == pytest.approx(flatten(arm_metadata), abs=1e-6)


def test_tool_offset_is_the_mcp_server_tool_point() -> None:
    mcp = client_node("mcp_server")["arm"]["tool_offset_m"]
    assert ARM.handle is not None
    assert tuple(ARM.handle.tool_offset_m) == (mcp["x"], mcp["y"], mcp["z"])


def test_handle_solid_is_a_manifold() -> None:
    assert ARM.handle is not None
    assert isinstance(handle_solid(ARM.handle), Manifold)
    assert isinstance(post_solid(ARM.handle, ARM.plate_mm, ARM.thickness_mm), Manifold)


def column_z(solid: Manifold, x: float, y: float) -> tuple[float, float]:
    """Lowest and highest z of a solid inside a thin vertical column.

    Args:
        solid (Manifold): The solid (mm).
        x (float): Column centre x (mm).
        y (float): Column centre y (mm).

    Returns:
        tuple[float, float]: (z min, z max) of the material in the column (mm).
    """
    column = box(x - COLUMN_MM, y - COLUMN_MM, -10.0, x + COLUMN_MM, y + COLUMN_MM, 100.0)
    lo, hi = to_trimesh(solid ^ column).bounds
    return float(lo[2]), float(hi[2])


def test_shipped_clamp_clearance_is_the_user_allowance() -> None:
    assert ARM.handle is not None
    assert ARM.handle.clamp_clearance_mm == CLAMP_CLEARANCE_MM
    assert HandleSpec.model_fields["clamp_clearance_mm"].default == CLAMP_CLEARANCE_MM


def test_printed_grip_is_thinner_at_the_clamp_faces_only() -> None:
    """The printed grip loses clamp_clearance_mm at the bed face and at the channel floor; the rails, the channel
    walls and the shoulder block are the nominal ones."""
    h = ARM.handle
    assert h is not None
    c = h.clamp_clearance_mm
    printed = handle_solid(h)
    nominal = handle_solid(h, nominal=True)
    centre = 0.0  # handle frame: the grip is centred on x = 0
    for depth in (2.0, h.length_mm / 2, h.length_mm - 2.0):
        y = -h.neck_mm - depth
        p_lo, p_hi = column_z(printed, centre, y)
        n_lo, n_hi = column_z(nominal, centre, y)
        assert p_lo == pytest.approx(0.0, abs=1e-3)
        assert p_lo - n_lo == pytest.approx(c, abs=ALLOWANCE_TOL_MM)
        assert n_hi - p_hi == pytest.approx(c, abs=ALLOWANCE_TOL_MM)
        assert (n_hi - n_lo) - (p_hi - p_lo) == pytest.approx(2 * c, abs=ALLOWANCE_TOL_MM)
        rail_x = centre + h.channel_end_mm / 2 + h.rail_width_mm / 2
        assert column_z(printed, rail_x, y)[1] == pytest.approx(column_z(nominal, rail_x, y)[1], abs=1e-3)
    assert (printed - nominal).volume() == pytest.approx(0.0, abs=1e-3)
    shoulder = box(centre - h.width_mm, -h.neck_mm, -10.0, centre + h.width_mm, 0.0, 100.0)
    assert (printed ^ shoulder).volume() == pytest.approx((nominal ^ shoulder).volume(), abs=1e-3)
    channel_area = (h.channel_tip_mm + h.channel_end_mm) / 2 * h.length_mm
    expected = c * h.width_mm * h.length_mm + c * channel_area
    assert (nominal - printed).volume() == pytest.approx(expected, rel=VOLUME_REL_TOL)
    above = box(centre - h.width_mm, -h.extent_mm - 1.0, h.thickness_end_mm, centre + h.width_mm, 0.0, 100.0)
    assert (printed ^ above).volume() == pytest.approx((nominal ^ above).volume(), abs=1e-3)


def test_nominal_solid_adds_the_clamp_allowance(arm_bodies: tuple[trimesh.Trimesh, trimesh.Trimesh]) -> None:
    h = ARM.handle
    assert h is not None
    solid = printed_solid(*arm_bodies)
    grown = nominal_solid(solid, h, ARM.plate_mm, ARM.thickness_mm)
    allowance = post_solid(h, ARM.plate_mm, ARM.thickness_mm, nominal=True) - post_solid(
        h, ARM.plate_mm, ARM.thickness_mm
    )
    assert grown.is_watertight
    assert grown.volume == pytest.approx(solid.volume + allowance.volume(), rel=1e-6)
    bed_face_y = (ARM.plate_mm[1] - shoulder_top_mm(h)) / 2  # mesh y of the printed bed face (handle z = 0)
    lo_y = to_trimesh(to_manifold(grown) ^ box(0.0, 0.0, ARM.thickness_mm + h.neck_mm + 1.0, *ARM.plate_mm, 100.0))
    assert lo_y.bounds[0][1] == pytest.approx(bed_face_y - h.clamp_clearance_mm, abs=1e-3)
    assert (to_manifold(solid) - to_manifold(grown)).volume() == pytest.approx(0.0, abs=1e-3)


def test_printed_handle_clears_both_jaws_at_the_nominal_contact(
    arm_metadata: dict[str, Any], arm_bodies: tuple[trimesh.Trimesh, trimesh.Trimesh]
) -> None:
    """At the nominal seat and contact angle the printed (thinner) handle stands off each jaw by clamp_clearance_mm.

    On the moving jaw the nominal contact itself is found to within CONTACT_TOL_MM of penetration, so the printed
    gap there lies between c - CONTACT_TOL_MM and c (plus ALLOWANCE_TOL_MM either way).
    """
    c = ARM.handle.clamp_clearance_mm  # type: ignore[union-attr]
    handle = seated_handle(arm_metadata, arm_bodies, nominal=False)
    moving = moving_jaw(GRIPPER, arm_metadata["grip"]["contact_rad"])
    assert clearance_mm(handle, GRIPPER.fixed) == pytest.approx(c, abs=ALLOWANCE_TOL_MM)
    assert c - CONTACT_TOL_MM - ALLOWANCE_TOL_MM <= clearance_mm(handle, moving) <= c + ALLOWANCE_TOL_MM
    for jaw in (GRIPPER.fixed, moving):
        assert penetration_mm(handle, jaw) == 0.0
        assert penetration_mm(jaw, handle) == 0.0


def test_squeeze_closes_past_the_modelled_contact_on_an_exact_print(arm_metadata: dict[str, Any]) -> None:
    """An exact (not grown) print is 2 c thinner: pressed on the fixed jaw, the moving jaw first touches it at
    modelled_contact_rad, further closed (a smaller angle) than the nominal contact. The commanded grip still closes
    past it."""
    grip = arm_metadata["grip"]
    assert grip["clearance_clamp_mm"] == ARM.handle.clamp_clearance_mm  # type: ignore[union-attr]
    assert grip["grip_rad"] < grip["modelled_contact_rad"] < grip["contact_rad"]


def test_printed_contact_rad_seats_the_print_on_the_fixed_jaw(
    arm_metadata: dict[str, Any], arm_bodies: tuple[trimesh.Trimesh, trimesh.Trimesh]
) -> None:
    h = ARM.handle
    assert h is not None
    solid = printed_solid(*arm_bodies)
    contact = printed_contact_rad(GRIPPER, solid, h, mesh_in_gripper_link(arm_metadata), ARM.thickness_mm)
    assert contact == pytest.approx(arm_metadata["grip"]["modelled_contact_rad"], abs=1e-6)
    pressed = mesh_in_gripper_link(arm_metadata)
    pressed[:3, 3] -= np.asarray(OPEN_AXIS) * h.clamp_clearance_mm  # bed face onto the fixed jaw
    handle = post_crop(solid).apply_transform(pressed)
    assert clearance_mm(handle, GRIPPER.fixed) <= CONTACT_TOL_MM
    assert penetration_mm(handle, GRIPPER.fixed) <= CONTACT_TOL_MM


def gusset_sides(meta: dict[str, Any]) -> list[np.ndarray]:
    """Centroid offsets of the two gussets from the post axis in gripper_link (mm).

    Args:
        meta (dict[str, Any]): Board metadata.

    Returns:
        list[np.ndarray]: One (3,) offset per gusset.
    """
    h = ARM.handle
    assert h is not None
    pose = mesh_in_gripper_link(meta)
    axis = pose @ post_in_mesh(h, ARM.plate_mm, ARM.thickness_mm) @ np.array([0.0, 0.0, shoulder_top_mm(h) / 2, 1.0])
    parts = gusset_solid(h, ARM.plate_mm, ARM.thickness_mm).decompose()
    return [trimesh.transform_points([to_trimesh(part).centroid], pose)[0] - axis[:3] for part in parts]


def test_gussets_brace_the_post_rail_walls_within_45_deg() -> None:
    """Two triangular webs, one off each rail outer wall: gusset_height_mm up the wall, gusset_reach_mm along the
    plate back, gusset_thickness_mm thick around the post axis. Side walls vertical, the top face up-facing and at
    most 45 deg from vertical, the bottom on the plate back."""
    h = ARM.handle
    assert h is not None
    t, (w, height) = ARM.thickness_mm, ARM.plate_mm
    gussets = gusset_solid(h, ARM.plate_mm, t)
    parts = sorted(gussets.decompose(), key=lambda part: to_trimesh(part).bounds[0][0])
    assert len(parts) == 2
    half = h.width_mm / 2
    web = (height / 2 - h.gusset_thickness_mm / 2, height / 2 + h.gusset_thickness_mm / 2)
    expected = [
        [[w / 2 - half - h.gusset_reach_mm, web[0], t], [w / 2 - half, web[1], t + h.gusset_height_mm]],
        [[w / 2 + half, web[0], t], [w / 2 + half + h.gusset_reach_mm, web[1], t + h.gusset_height_mm]],
    ]
    limit = np.sin(np.radians(GUSSET_SLOPE_LIMIT_DEG)) + 1e-6
    for part, bounds in zip(parts, expected, strict=True):
        mesh = to_trimesh(part)
        assert mesh.bounds == pytest.approx(np.array(bounds), abs=1e-3)
        assert part.volume() == pytest.approx(
            h.gusset_height_mm * h.gusset_reach_mm / 2 * h.gusset_thickness_mm, rel=1e-6
        )
        nz = mesh.face_normals[:, 2]
        bottom = (nz < -0.5) & np.isclose(mesh.triangles_center[:, 2], t, atol=1e-3)
        vertical = np.abs(nz) < 1e-4
        sloped = ~bottom & ~vertical
        assert sloped.any() and np.all((nz[sloped] > 0.0) & (nz[sloped] <= limit))
        assert np.all(bottom | vertical | sloped)
        side = vertical & (np.abs(mesh.face_normals[:, 1]) > 1.0 - 1e-6)  # walls facing the jaw-closing direction
        assert side.sum() >= 2
    assert h.gusset_reach_mm / h.gusset_height_mm <= np.tan(np.radians(GUSSET_SLOPE_LIMIT_DEG)) + 1e-9


def test_gussets_sit_on_the_sides_the_jaws_do_not_occupy(arm_metadata: dict[str, Any]) -> None:
    """The jaws close along OPEN_AXIS; the gussets stand off the post along WIDTH_AXIS (the rail outer walls)."""
    for offset in gusset_sides(arm_metadata):
        along_width = abs(float(offset @ np.asarray(WIDTH_AXIS)))
        assert along_width > ARM.handle.width_mm / 2  # type: ignore[union-attr]
        assert abs(float(offset @ np.asarray(OPEN_AXIS))) < 1e-3


def test_gussets_join_the_post_and_plate_into_one_print(arm_bodies: tuple[trimesh.Trimesh, trimesh.Trimesh]) -> None:
    h = ARM.handle
    assert h is not None
    base = to_manifold(arm_bodies[0])
    gussets = gusset_solid(h, ARM.plate_mm, ARM.thickness_mm)
    assert (gussets - base).volume() == pytest.approx(0.0, abs=1e-3)
    assert (gussets ^ post_solid(h, ARM.plate_mm, ARM.thickness_mm)).volume() == pytest.approx(0.0, abs=1e-3)
    cx, cy, t = ARM.plate_mm[0] / 2, ARM.plate_mm[1] / 2, ARM.thickness_mm
    for sign in (-1.0, 1.0):
        x = cx + sign * (h.width_mm / 2 + PROBE_MM)  # just off the rail wall: the gusset top is near its full height
        top = t + h.gusset_height_mm * (1.0 - (PROBE_MM - COLUMN_MM) / h.gusset_reach_mm)  # the column's inner edge
        assert column_z(base, x, cy)[1] == pytest.approx(top, abs=1e-3)


def test_jaws_clear_the_plate_and_gussets_at_insert_and_over_the_seat_sweep(arm_metadata: dict[str, Any]) -> None:
    """At the insert opening, the grip angle and every seat-sweep angle (contact to CONTACT_HI_RAD), and for the fixed
    jaw: no jaw vertex within MIN_PLATE_CLEARANCE_MM of the plate back (a plane: the closest point is a vertex), no
    penetration into the gussets either way, and jaw surface points at least MIN_GUSSET_CLEARANCE_MM from them
    (memory-safe nearest_distance_mm from the jaw points onto the small gusset mesh, never closest_point over a
    jaw)."""
    h = ARM.handle
    assert h is not None
    pose = mesh_in_gripper_link(arm_metadata)
    to_mesh = np.linalg.inv(pose)
    gussets = to_trimesh(gusset_solid(h, ARM.plate_mm, ARM.thickness_mm)).apply_transform(pose)
    grip = arm_metadata["grip"]
    sweep = [float(a) for a in np.arange(grip["contact_rad"], CONTACT_HI_RAD, SWEEP_STEP_RAD)]
    angles = [INSERT_GRIPPER_RAD, grip["grip_rad"], *sweep, CONTACT_HI_RAD]
    cross = {INSERT_GRIPPER_RAD, grip["grip_rad"], *sweep[:: round(CROSS_STEP_RAD / SWEEP_STEP_RAD)], CONTACT_HI_RAD}
    jaws = [(f"moving {a:.3f}", moving_jaw(GRIPPER, a), a in cross) for a in angles] + [("fixed", GRIPPER.fixed, True)]
    for name, jaw, full in jaws:
        lift = float(trimesh.transform_points(jaw.vertices, to_mesh)[:, 2].min()) - ARM.thickness_mm
        assert lift >= MIN_PLATE_CLEARANCE_MM, name
        assert penetration_mm(gussets, jaw) == 0.0, name
        assert penetration_mm(jaw, gussets) == 0.0, name
        if full:
            assert nearest_distance_mm(gussets, surface_points(jaw, JAW_SAMPLES)) >= MIN_GUSSET_CLEARANCE_MM, name


def test_seat_crop_keeps_the_post_and_gussets_only(arm_bodies: tuple[trimesh.Trimesh, trimesh.Trimesh]) -> None:
    h = ARM.handle
    assert h is not None
    crop = seat_crop(printed_solid(*arm_bodies), ARM.thickness_mm)
    assert crop.is_watertight
    expected = (post_solid(h, ARM.plate_mm, ARM.thickness_mm) + gusset_solid(h, ARM.plate_mm, ARM.thickness_mm)) ^ box(
        0.0, 0.0, ARM.thickness_mm + 0.5, *ARM.plate_mm, 100.0
    )
    assert crop.volume == pytest.approx(expected.volume(), rel=1e-4)
    assert crop.bounds[0][2] == pytest.approx(ARM.thickness_mm + 0.5, abs=1e-3)


def test_grip_angles_are_the_in_line_handle_ones(arm_metadata: dict[str, Any]) -> None:
    """The grip is reused unchanged and only turned: the nominal and modelled contact angles are those of the in-line
    handle, and grip_rad < modelled_contact_rad < contact_rad as in Task 6b."""
    grip = arm_metadata["grip"]
    for key, value in IN_LINE_GRIP.items():
        assert grip[key] == pytest.approx(value, abs=GRIP_INVARIANT_TOL_RAD)
    assert grip["grip_rad"] < grip["modelled_contact_rad"] < grip["contact_rad"]


def test_post_axis_is_the_tool_axis_and_jaws_close_along_board_y(arm_metadata: dict[str, Any]) -> None:
    """The board normal lies along the tool axis (the jaw tips point at the plate back, board -z), the jaws close
    along the board y axis; both recorded in handle_in_board."""
    hib = arm_metadata["handle_in_board"]
    rotation = pose_from_xyz_rpy(hib["gripper_link_xyz_m"], hib["gripper_link_rpy_rad"])[:3, :3]
    assert rotation @ np.asarray(TOOL_AXIS) == pytest.approx([0.0, 0.0, -1.0], abs=AXIS_TOL)
    assert rotation @ np.asarray(OPEN_AXIS) == pytest.approx([0.0, -1.0, 0.0], abs=AXIS_TOL)
    assert hib["tool_axis"] == pytest.approx([0.0, 0.0, -1.0], abs=AXIS_TOL)
    assert hib["open_axis"] == pytest.approx([0.0, -1.0, 0.0], abs=AXIS_TOL)
    # the tool point lies on the post, between the plate centre and the free end
    h = ARM.handle
    assert h is not None
    margin, (w, height) = ARM.margin_mm, ARM.plate_mm
    point_mm = np.asarray(hib["tool_point_m"]) * MM_PER_M
    assert abs(point_mm[0] - (w / 2 - margin)) <= h.width_mm / 2
    assert abs(point_mm[1] - (height / 2 - margin)) <= shoulder_top_mm(h) / 2 + TOOL_POINT_TOL_MM
    assert ARM.thickness_mm < point_mm[2] < ARM.thickness_mm + h.extent_mm


def test_board_mass_and_centre_of_mass(
    arm_metadata: dict[str, Any], arm_bodies: tuple[trimesh.Trimesh, trimesh.Trimesh]
) -> None:
    """Mass in a plausible range, equal to the bare face-down plate plus the post and gussets; the centre of mass in
    the board frame (m) sits over the plate centre, a little behind the plate."""
    h = ARM.handle
    assert h is not None
    bare = ARM.model_copy(update={"handle": None})
    plate_cm3 = sum(body.volume for body in print_bodies(bare, CONFIG.stl)) / 1000.0
    post_cm3 = (
        post_solid(h, ARM.plate_mm, ARM.thickness_mm) + gusset_solid(h, ARM.plate_mm, ARM.thickness_mm)
    ).volume()
    assert arm_metadata["mass_g"] == pytest.approx((plate_cm3 + post_cm3 / 1000.0) * h.density_g_per_cm3, rel=1e-4)
    assert MASS_RANGE_G[0] <= arm_metadata["mass_g"] <= MASS_RANGE_G[1]
    solid = printed_solid(*arm_bodies)
    mesh_to_board = np.linalg.inv(board_in_mesh(ARM.plate_mm, ARM.margin_mm, ARM.pattern_face, ARM.thickness_mm))
    expected = trimesh.transform_points([solid.center_mass], mesh_to_board)[0] / MM_PER_M
    com = np.asarray(arm_metadata["centre_of_mass_m"])
    assert com == pytest.approx(expected, abs=1e-6)
    centre = (np.asarray(ARM.plate_mm) / 2 - ARM.margin_mm) / MM_PER_M
    assert com[:2] == pytest.approx(centre, abs=0.003)
    assert ARM.thickness_mm / 2 / MM_PER_M < com[2] < (ARM.thickness_mm + h.extent_mm) / MM_PER_M


def test_back_envelope_boxes_hold_the_post_and_gussets(
    arm_metadata: dict[str, Any], arm_bodies: tuple[trimesh.Trimesh, trimesh.Trimesh]
) -> None:
    """back_envelope_mm: the post box (nominal bed face included) and one box per gusset in the OpenCV board frame,
    each from the plate back (z = T) up; together they hold all material behind the plate back."""
    h = ARM.handle
    assert h is not None
    envelope = back_envelope_mm(ARM)
    assert flatten(arm_metadata["back_envelope_mm"]) == pytest.approx(flatten(envelope), abs=1e-6)
    t = ARM.thickness_mm
    assert envelope["height_mm"] == pytest.approx(h.extent_mm)
    boxes = [envelope["post"], *envelope["gussets"]]
    assert len(envelope["gussets"]) == 2
    for low, high in boxes:
        assert low[2] == pytest.approx(t)
        assert 0.0 <= low[0] and high[0] <= ARM.plate_mm[0] and -ARM.margin_mm <= low[1]
    assert envelope["post"][1][2] == pytest.approx(t + h.extent_mm)
    mesh_to_board = np.linalg.inv(board_in_mesh(ARM.plate_mm, ARM.margin_mm, ARM.pattern_face, t))
    grown = nominal_solid(printed_solid(*arm_bodies), h, ARM.plate_mm, t)
    behind = trimesh.transform_points(post_crop(grown).vertices, mesh_to_board)
    inside = np.zeros(len(behind), dtype=bool)
    for low, high in boxes:
        inside |= np.all((behind >= np.asarray(low) - 1e-3) & (behind <= np.asarray(high) + 1e-3), axis=1)
    assert inside.all()


def test_shoulder_top_follows_the_spec() -> None:
    h = ARM.handle
    assert h is not None
    assert shoulder_top_mm(h) == h.thickness_tip_mm + h.rail_height_mm
