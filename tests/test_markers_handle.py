"""Tests for scripts/markers/handle.py and the ARM board flow of write_charuco: edge handle (a tab off the plate edge
with the grip post on the pattern side and one gusset, user m540), back-down print with the pattern up, seat in the
SO-101 jaws (URDF meshes), facing, occlusion, tab stiffness, metadata JSON and the shipped outputs."""

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
    board_config,
    board_metadata,
    handle_envelope_mm,
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
    handle_body,
    handle_solid,
    nominal_solid,
    post_in_mesh,
    post_solid,
    printed_contact_rad,
    seat_crop,
    shoulder_top_mm,
    stiffness,
    tab_solid,
)
from scripts.markers.jaw_fit import (
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
HANDLE_CUT_MM = 0.5  # mesh z above the tab top from which the printed solid is the post and its gusset
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
# The in-line paddle's grip is reused unchanged, only turned to stand on the tab (decision 296, user m540), so the
# contact angles of the in-line handle (Task 6c, 8f0c0a55) stay; the crop holds the post instead of a plate sliver.
IN_LINE_GRIP = {"contact_rad": 0.07027905529357185, "modelled_contact_rad": 0.06620181229285478}
GRIP_INVARIANT_TOL_RAD = 1e-4
MIN_TAB_CLEARANCE_MM = 5.0  # jaws to the tab top (above the pattern face) over the insert opening and the sweep
MIN_GUSSET_CLEARANCE_MM = 5.0  # jaws to the gusset over the insert opening and the seat sweep
GUSSET_SLOPE_LIMIT_DEG = 45.0  # gusset top face at most this far from vertical in print orientation
MASS_RANGE_G = (180.0, 230.0)  # plausible board mass: the ~133 g plate plus the tab, the post and the gusset
AXIS_TOL = 1e-9
PROBE_MM = 0.5  # column offsets inside the post and the gusset
JAW_SAMPLES = 5000  # jaw surface samples for the jaw-to-gusset distance (plus every jaw vertex)
TOOL_POINT_TOL_MM = 2.0  # the tool point lies on the fixed jaw face plane, just off the printed bed face
MIN_OCCLUSION_GAP_MM = 10.0  # gripper footprint (projected along the pattern normal) to the ChArUco square area
MAX_TAB_DEFLECTION_MM = 0.1  # board centre sag under 1 g from the tab (ruling 5)
# main's measured stick mount (head_rgbd_calibration NOMINAL_BOARD_XYZ / RPY, FULL 6x5 40 mm board): the board centre
# 0.12 m off the tool axis, the plane 2 cm past the tool point, the long edges level at the parked arm
STICK_XYZ_M = (-0.1665, 0.1938, -0.1181)
STICK_RPY_RAD = (3.141593, 0.0, 0.185886)
STICK_CENTRE_M = (0.12, 0.10, 0.0)  # FULL pattern centre in its board frame
MAX_MOUNT_SHIFT_MM = 36.0  # the post sits on the edge middle: the centre moves along the plate edge only
MAX_MOUNT_TURN_DEG = 11.0  # axis-aligned jaws: the stick mount's 10.65 deg yaw remains
PLANE_TOL_MM = 1.0
CROSS_STEP_RAD = 0.05  # gripper angle step of the jaw-to-gusset clearance sweep (penetration uses SWEEP_STEP_RAD)
FACING_DOT_MAX = -0.99  # n . t of the pattern normal and the tool approach axis (user m540)


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

    The solid is cut at the tab top + HANDLE_CUT_MM (capped, watertight): the post and its gusset, so surface samples
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
        solid = nominal_solid(solid, ARM.handle, ARM.plate_mm)
    return post_crop(solid).apply_transform(mesh_in_gripper_link(meta))


def post_crop(solid: trimesh.Trimesh) -> trimesh.Trimesh:
    """The solid above the tab top + HANDLE_CUT_MM (the post and its gusset), capped.

    Args:
        solid (trimesh.Trimesh): Watertight solid in mesh coordinates (mm).

    Returns:
        trimesh.Trimesh: The capped crop (mm).
    """
    assert ARM.handle is not None
    origin = (0.0, 0.0, ARM.handle.tab_thickness_mm + HANDLE_CUT_MM)
    return trimesh.intersections.slice_mesh_plane(solid, (0.0, 0.0, 1.0), origin, cap=True)


def handle_frame(meta: dict[str, Any]) -> np.ndarray:
    """Pose of the handle frame (x across the post, y along it toward the tab, z from the bed face) in gripper_link.

    Args:
        meta (dict[str, Any]): Board metadata.

    Returns:
        np.ndarray: 4x4 transform, mm.
    """
    assert ARM.handle is not None
    return mesh_in_gripper_link(meta) @ post_in_mesh(ARM.handle, ARM.plate_mm)


def top_view(modules: trimesh.Trimesh, spec: CharucoSpec) -> np.ndarray:
    """Rasterise the module faces on the pattern face (normal z > 0.5 at z = T) as seen from above, dark on white.

    Seen from above with mesh +y up, image right is mesh +x: pixel (x, H - y) * VIEW_PX_PER_MM.

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
        if normal[2] > 0.5 and np.allclose(pts[:, 2], spec.thickness_mm, atol=1e-4):
            xy = np.stack([pts[:, 0] * VIEW_PX_PER_MM, (height_mm - pts[:, 1]) * VIEW_PX_PER_MM], axis=1)
            cv2.fillConvexPoly(image, np.round(xy).astype(np.int32), 0)
    return image


def view_to_mesh_xy(corners: np.ndarray, spec: CharucoSpec) -> np.ndarray:
    """Top-view pixels back to mesh (x, y) in mm.

    Args:
        corners (np.ndarray): (N, 2) pixel coordinates.
        spec (CharucoSpec): The board.

    Returns:
        np.ndarray: (N, 2) mesh coordinates (mm).
    """
    height_mm = spec.plate_mm[1]
    return np.stack([corners[:, 0] / VIEW_PX_PER_MM, height_mm - corners[:, 1] / VIEW_PX_PER_MM], axis=1)


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
    """Highest base vertex over the plate (mesh y > PROBE_MM, off the tab): the pattern face, nothing above it.

    Args:
        base (trimesh.Trimesh): Base body.

    Returns:
        float: z (mm).
    """
    v = base.vertices
    return float(v[v[:, 1] > PROBE_MM][:, 2].max())


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
        ({"gusset_height_mm": 25.0}, "gusset_reach_mm"),  # reach 30 on 25: flatter than 45 deg from vertical
        ({"gusset_reach_mm": 30.8}, "gusset_reach_mm"),  # past the plate edge (post_offset 43 - 12.3)
        ({"gusset_thickness_mm": 0.0}, "gusset_thickness_mm"),
        ({"gusset_thickness_mm": 24.8}, "gusset_thickness_mm"),  # thicker than the post section
        ({"tab_width_mm": 24.6}, "tab_width_mm"),  # narrower than the post footprint (24.7)
        ({"tab_root_width_mm": 39.0}, "tab_root_width_mm"),  # narrower at the plate than at the far end
        ({"tab_thickness_mm": 0.0}, "tab_thickness_mm"),
        ({"post_offset_mm": 0.0}, "post_offset_mm"),
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


def test_pattern_face_is_required_and_arm_prints_back_down() -> None:
    spec = ARM.model_dump()
    del spec["pattern_face"]
    with pytest.raises(ValidationError, match="pattern_face"):
        CharucoSpec.model_validate(spec)
    assert ARM.pattern_face == "top"
    assert next(b for b in CONFIG.boards if b.name != ARM_NAME).pattern_face == "top"


@pytest.mark.parametrize(
    ("update", "field"),
    [
        ({"tab_thickness_mm": 4.0}, "tab_thickness_mm"),  # not thicker than the plate
        ({"tab_root_width_mm": 178.5}, "tab_root_width_mm"),  # wider than the plate edge
        ({"post_offset_mm": 100.0, "gusset_reach_mm": 30.0}, "bed_mm"),  # plate 150 + tab 112.3 > 256
    ],
)
def test_charuco_spec_refuses_a_tab_off_the_plate_or_bed(update: dict[str, Any], field: str) -> None:
    spec = ARM.model_dump()
    spec["handle"] = {**spec["handle"], **update}
    with pytest.raises(ValidationError, match=field):
        CharucoSpec.model_validate(spec)


def test_charuco_spec_needs_the_pattern_face_up_for_an_edge_handle() -> None:
    with pytest.raises(ValidationError, match="pattern_face"):
        CharucoSpec.model_validate({**ARM.model_dump(), "pattern_face": "bottom"})


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


def test_post_in_mesh_stands_the_handle_on_the_tab() -> None:
    """Handle y (toward the tab) is mesh -z, the free end points up from the tab top (mesh z = tab_thickness_mm);
    handle x is mesh -y (the plate lies at handle -x), handle z (OPEN_AXIS) mesh +x along the plate edge; the
    footprint centre (x = 0, z = shoulder top / 2) lands post_offset_mm off the middle of the plate edge y = 0."""
    h = ARM.handle
    assert h is not None
    pose = post_in_mesh(h, ARM.plate_mm)
    assert pose[:3, :3] == pytest.approx(np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]))
    assert np.linalg.det(pose[:3, :3]) == pytest.approx(1.0)
    centre = pose @ np.array([0.0, 0.0, shoulder_top_mm(h) / 2, 1.0])
    assert centre[:3] == pytest.approx([ARM.plate_mm[0] / 2, -h.post_offset_mm, h.tab_thickness_mm])
    free_end = pose @ np.array([0.0, -h.extent_mm, 0.0, 1.0])
    assert free_end[2] == pytest.approx(h.tab_thickness_mm + h.extent_mm)


def test_post_solid_alone_is_printable_and_stands_on_the_tab() -> None:
    """The post prints standing on the tab top: its tapered walls lean a few degrees, never past 45 deg."""
    h = ARM.handle
    assert h is not None
    solid = to_trimesh(post_solid(h, ARM.plate_mm))
    assert print_violations(solid) == []
    low_x = ARM.plate_mm[0] / 2 - shoulder_top_mm(h) / 2
    expected = [
        [low_x, -h.post_offset_mm - h.width_mm / 2, h.tab_thickness_mm],
        [
            low_x + h.thickness_end_mm + h.rail_height_mm,
            -h.post_offset_mm + h.width_mm / 2,
            h.tab_thickness_mm + h.extent_mm,
        ],
    ]
    assert solid.bounds == pytest.approx(np.array(expected), abs=1e-3)


def test_tab_solid_is_a_trapezoid_off_the_middle_of_the_plate_edge() -> None:
    h = ARM.handle
    assert h is not None
    w = ARM.plate_mm[0]
    tab = tab_solid(h, ARM.plate_mm)
    expected = [
        [(w - h.tab_root_width_mm) / 2, -h.tab_length_mm, 0.0],
        [(w + h.tab_root_width_mm) / 2, 0.0, h.tab_thickness_mm],
    ]
    assert to_trimesh(tab).bounds == pytest.approx(np.array(expected), abs=1e-3)
    area = (h.tab_root_width_mm + h.tab_width_mm) / 2 * h.tab_length_mm
    assert tab.volume() == pytest.approx(area * h.tab_thickness_mm, rel=1e-6)
    far = to_trimesh(tab ^ box(0.0, -h.tab_length_mm, 0.0, w, -h.tab_length_mm + COLUMN_MM, 100.0)).bounds
    flare = (h.tab_root_width_mm - h.tab_width_mm) / h.tab_length_mm * COLUMN_MM  # the slice is COLUMN_MM deep
    assert far[1][0] - far[0][0] == pytest.approx(h.tab_width_mm + flare, abs=1e-3)
    assert print_violations(to_trimesh(tab)) == []
    assert h.tab_length_mm == pytest.approx(h.post_offset_mm + h.width_mm / 2)


def test_arm_board_union_is_watertight_and_overhang_free(arm_bodies: tuple[trimesh.Trimesh, trimesh.Trimesh]) -> None:
    base, modules = arm_bodies
    assert print_violations(printed_solid(base, modules)) == []


def test_pattern_reads_from_above_in_the_top_layers(arm_bodies: tuple[trimesh.Trimesh, trimesh.Trimesh]) -> None:
    """The board prints back down: the modules are inlaid in the top layers (from T - inlay up to the pattern face
    at z = T) and the pattern reads from above, at the corners of the top board frame."""
    _, modules = arm_bodies
    assert modules.bounds[0][2] == pytest.approx(ARM.thickness_mm - ARM.inlay_mm, abs=1e-4)
    assert modules.bounds[1][2] == pytest.approx(ARM.thickness_mm, abs=1e-4)
    corners, ids = detect(top_view(modules, ARM), ARM)
    assert len(ids) == INNER_CORNERS
    board = board_in_mesh(ARM.plate_mm, ARM.margin_mm, "top", ARM.thickness_mm)
    expected = (board[:3, :3] @ (make_charuco_board(ARM).getChessboardCorners().T * MM_PER_M)).T + board[:3, 3]
    for corner, ident in zip(view_to_mesh_xy(corners, ARM), ids, strict=True):
        assert np.linalg.norm(corner - expected[ident][:2]) < 0.2


def test_bed_side_is_flat(arm_bodies: tuple[trimesh.Trimesh, trimesh.Trimesh]) -> None:
    """The back of the plate and of the tab lie on the bed: the whole bed footprint is one flat face at z = 0."""
    h = ARM.handle
    assert h is not None
    base = arm_bodies[0]
    bed = (base.face_normals[:, 2] < -0.5) & np.isclose(base.triangles_center[:, 2], 0.0, atol=1e-4)
    steep = base.face_normals[:, 2] < -0.5  # the post's tapered faces lean ~7 deg, every other face is up or vertical
    assert np.all(bed[steep])
    tab_area = (h.tab_root_width_mm + h.tab_width_mm) / 2 * h.tab_length_mm
    assert float(base.area_faces[bed].sum()) == pytest.approx(ARM.plate_mm[0] * ARM.plate_mm[1] + tab_area, rel=1e-6)


def test_handle_dimensions_follow_the_spec(arm_bodies: tuple[trimesh.Trimesh, trimesh.Trimesh]) -> None:
    """The plate keeps its outline with the tab off its edge y = 0; the post rises neck plus grip above the tab top,
    perpendicular to the plate, post_offset_mm out from the edge; the plate itself is bare up to the pattern face."""
    base, _ = arm_bodies
    h = ARM.handle
    assert h is not None
    top = h.tab_thickness_mm + h.extent_mm
    assert base.bounds[0] == pytest.approx([0.0, -h.tab_length_mm, 0.0], abs=1e-3)
    assert base.bounds[1] == pytest.approx([ARM.plate_mm[0], ARM.plate_mm[1], top], abs=1e-3)
    centre_x = ARM.plate_mm[0] / 2
    assert column_z(to_manifold(base), centre_x, -h.post_offset_mm)[1] == pytest.approx(top, abs=1e-3)
    web_low = centre_x - h.gusset_thickness_mm / 2 - PROBE_MM  # mesh x below the gusset web
    for depth in (PROBE_MM, h.extent_mm / 2, h.extent_mm - PROBE_MM):  # the post is straight at every level
        z = h.tab_thickness_mm + depth
        level = box(0.0, -h.tab_length_mm - 1.0, z - COLUMN_MM, web_low, 0.0, z + COLUMN_MM)
        lo, hi = to_trimesh(to_manifold(base) ^ level).bounds
        assert [lo[1], hi[1]] == pytest.approx([-h.post_offset_mm - s * h.width_mm / 2 for s in (1, -1)], abs=1e-3)
        assert lo[0] == pytest.approx(centre_x - shoulder_top_mm(h) / 2, abs=1e-3)
    assert bare_plate_top(base) == pytest.approx(ARM.thickness_mm, abs=1e-3)


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
    assert isinstance(post_solid(ARM.handle, ARM.plate_mm), Manifold)
    assert isinstance(handle_body(ARM.handle, ARM.plate_mm), Manifold)


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
    grown = nominal_solid(solid, h, ARM.plate_mm)
    allowance = post_solid(h, ARM.plate_mm, nominal=True) - post_solid(h, ARM.plate_mm)
    assert grown.is_watertight
    assert grown.volume == pytest.approx(solid.volume + allowance.volume(), rel=1e-6)
    bed_face_x = (ARM.plate_mm[0] - shoulder_top_mm(h)) / 2  # mesh x of the printed bed face (handle z = 0)
    grip = box(0.0, -h.tab_length_mm, h.tab_thickness_mm + h.neck_mm + 1.0, ARM.plate_mm[0], 0.0, 100.0)
    lo_x = to_trimesh(to_manifold(grown) ^ grip)
    assert lo_x.bounds[0][0] == pytest.approx(bed_face_x - h.clamp_clearance_mm, abs=1e-3)
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
    contact = printed_contact_rad(GRIPPER, solid, h, mesh_in_gripper_link(arm_metadata))
    assert contact == pytest.approx(arm_metadata["grip"]["modelled_contact_rad"], abs=1e-6)
    pressed = mesh_in_gripper_link(arm_metadata)
    pressed[:3, 3] -= np.asarray(OPEN_AXIS) * h.clamp_clearance_mm  # bed face onto the fixed jaw
    handle = post_crop(solid).apply_transform(pressed)
    assert clearance_mm(handle, GRIPPER.fixed) <= CONTACT_TOL_MM
    assert penetration_mm(handle, GRIPPER.fixed) <= CONTACT_TOL_MM


def gusset_offset(meta: dict[str, Any]) -> np.ndarray:
    """Centroid offset of the gusset from the post axis in gripper_link (mm).

    Args:
        meta (dict[str, Any]): Board metadata.

    Returns:
        np.ndarray: (3,) offset.
    """
    h = ARM.handle
    assert h is not None
    pose = mesh_in_gripper_link(meta)
    axis = pose @ post_in_mesh(h, ARM.plate_mm) @ np.array([0.0, 0.0, shoulder_top_mm(h) / 2, 1.0])
    centroid = to_trimesh(gusset_solid(h, ARM.plate_mm)).centroid
    return trimesh.transform_points([centroid], pose)[0] - axis[:3]


def test_gusset_braces_the_post_toward_the_plate_within_45_deg() -> None:
    """One triangular web off the rail outer wall facing the plate: gusset_height_mm up the wall, gusset_reach_mm
    along the tab toward the plate (ending before the plate edge), gusset_thickness_mm thick around the post axis.
    Side walls vertical, the top face up-facing and at most 45 deg from vertical, the bottom on the tab top."""
    h = ARM.handle
    assert h is not None
    gusset = gusset_solid(h, ARM.plate_mm)
    assert len(gusset.decompose()) == 1
    near_wall = -h.post_offset_mm + h.width_mm / 2  # mesh y of the rail wall facing the plate
    cx, base_z = ARM.plate_mm[0] / 2, h.tab_thickness_mm
    expected = [
        [cx - h.gusset_thickness_mm / 2, near_wall, base_z],
        [cx + h.gusset_thickness_mm / 2, near_wall + h.gusset_reach_mm, base_z + h.gusset_height_mm],
    ]
    mesh = to_trimesh(gusset)
    assert mesh.bounds == pytest.approx(np.array(expected), abs=1e-3)
    assert mesh.bounds[1][1] <= 0.0  # on the tab, off the plate (mesh y >= 0)
    assert gusset.volume() == pytest.approx(
        h.gusset_height_mm * h.gusset_reach_mm / 2 * h.gusset_thickness_mm, rel=1e-6
    )
    limit = np.sin(np.radians(GUSSET_SLOPE_LIMIT_DEG)) + 1e-6
    nz = mesh.face_normals[:, 2]
    bottom = (nz < -0.5) & np.isclose(mesh.triangles_center[:, 2], base_z, atol=1e-3)
    vertical = np.abs(nz) < 1e-4
    sloped = ~bottom & ~vertical
    assert sloped.any() and np.all((nz[sloped] > 0.0) & (nz[sloped] <= limit))
    side = vertical & (np.abs(mesh.face_normals[:, 0]) > 1.0 - 1e-6)  # walls facing the jaw-closing direction
    assert side.sum() >= 2
    assert h.gusset_reach_mm / h.gusset_height_mm <= np.tan(np.radians(GUSSET_SLOPE_LIMIT_DEG)) + 1e-9


def test_gusset_sits_on_the_plate_side_the_jaws_do_not_occupy(arm_metadata: dict[str, Any]) -> None:
    """The jaws close along OPEN_AXIS; the gusset stands off the post along +WIDTH_AXIS, toward the plate."""
    offset = gusset_offset(arm_metadata)
    assert float(offset @ np.asarray(WIDTH_AXIS)) > ARM.handle.width_mm / 2  # type: ignore[union-attr]
    assert abs(float(offset @ np.asarray(OPEN_AXIS))) < 1e-3


def test_gusset_and_tab_join_the_post_and_plate_into_one_print(
    arm_bodies: tuple[trimesh.Trimesh, trimesh.Trimesh],
) -> None:
    h = ARM.handle
    assert h is not None
    base = to_manifold(arm_bodies[0])
    for part in (gusset_solid(h, ARM.plate_mm), tab_solid(h, ARM.plate_mm), post_solid(h, ARM.plate_mm)):
        assert (part - base).volume() <= 1e-6 * part.volume()  # float32 round-off of the base only
    assert (gusset_solid(h, ARM.plate_mm) ^ post_solid(h, ARM.plate_mm)).volume() == pytest.approx(0.0, abs=1e-3)
    y = -h.post_offset_mm + h.width_mm / 2 + PROBE_MM  # just off the rail wall: the gusset top is near its full height
    top = h.tab_thickness_mm + h.gusset_height_mm * (1.0 - (PROBE_MM - COLUMN_MM) / h.gusset_reach_mm)
    assert column_z(base, ARM.plate_mm[0] / 2, y)[1] == pytest.approx(top, abs=1e-3)
    assert column_z(base, ARM.plate_mm[0] / 2, -PROBE_MM)[1] == pytest.approx(h.tab_thickness_mm, abs=1e-3)


def test_jaws_clear_the_tab_and_gusset_at_insert_and_over_the_whole_gripper_range(
    arm_metadata: dict[str, Any],
) -> None:
    """At the insert opening, the grip angle and every angle from the contact to the URDF upper limit, and for the
    fixed jaw: no jaw vertex within MIN_TAB_CLEARANCE_MM of the tab top (a plane above the pattern face and the plate:
    the closest point is a vertex), no penetration into the gusset either way, and jaw surface points at least
    MIN_GUSSET_CLEARANCE_MM from it (memory-safe nearest_distance_mm from the jaw points onto the small gusset mesh,
    never closest_point over a jaw)."""
    h = ARM.handle
    assert h is not None
    pose = mesh_in_gripper_link(arm_metadata)
    to_mesh = np.linalg.inv(pose)
    gusset = to_trimesh(gusset_solid(h, ARM.plate_mm)).apply_transform(pose)
    grip = arm_metadata["grip"]
    sweep = [float(a) for a in np.arange(grip["contact_rad"], GRIPPER.upper_rad, SWEEP_STEP_RAD)]
    angles = [INSERT_GRIPPER_RAD, grip["grip_rad"], *sweep, GRIPPER.upper_rad]
    cross = {INSERT_GRIPPER_RAD, grip["grip_rad"], *sweep[:: round(CROSS_STEP_RAD / SWEEP_STEP_RAD)], GRIPPER.upper_rad}
    jaws = [(f"moving {a:.3f}", moving_jaw(GRIPPER, a), a in cross) for a in angles] + [("fixed", GRIPPER.fixed, True)]
    for name, jaw, full in jaws:
        lift = float(trimesh.transform_points(jaw.vertices, to_mesh)[:, 2].min()) - h.tab_thickness_mm
        assert lift >= MIN_TAB_CLEARANCE_MM, name
        assert penetration_mm(gusset, jaw) == 0.0, name
        assert penetration_mm(jaw, gusset) == 0.0, name
        if full:
            assert nearest_distance_mm(gusset, surface_points(jaw, JAW_SAMPLES)) >= MIN_GUSSET_CLEARANCE_MM, name


def test_seat_crop_keeps_the_post_and_gusset_only(arm_bodies: tuple[trimesh.Trimesh, trimesh.Trimesh]) -> None:
    h = ARM.handle
    assert h is not None
    crop = seat_crop(printed_solid(*arm_bodies), h)
    assert crop.is_watertight
    cut = h.tab_thickness_mm + HANDLE_CUT_MM
    above = box(0.0, -h.tab_length_mm - 1.0, cut, ARM.plate_mm[0], ARM.plate_mm[1], 100.0)
    expected = (post_solid(h, ARM.plate_mm) + gusset_solid(h, ARM.plate_mm)) ^ above
    assert crop.volume == pytest.approx(expected.volume(), rel=1e-4)
    assert crop.bounds[0][2] == pytest.approx(cut, abs=1e-3)


def test_grip_angles_are_the_in_line_handle_ones(arm_metadata: dict[str, Any]) -> None:
    """The grip is reused unchanged and only turned: the nominal and modelled contact angles are those of the in-line
    handle, and grip_rad < modelled_contact_rad < contact_rad as in Task 6b."""
    grip = arm_metadata["grip"]
    for key, value in IN_LINE_GRIP.items():
        assert grip[key] == pytest.approx(value, abs=GRIP_INVARIANT_TOL_RAD)
    assert grip["grip_rad"] < grip["modelled_contact_rad"] < grip["contact_rad"]


def test_post_axis_is_the_tool_axis_and_jaws_close_along_board_x(arm_metadata: dict[str, Any]) -> None:
    """The tool axis is board +z (the jaw tips point from the pattern side through the tab), the jaws close along the
    board x axis (the plate edge); both recorded in handle_in_board. The tool point lies on the post."""
    hib = arm_metadata["handle_in_board"]
    rotation = pose_from_xyz_rpy(hib["gripper_link_xyz_m"], hib["gripper_link_rpy_rad"])[:3, :3]
    assert rotation @ np.asarray(TOOL_AXIS) == pytest.approx([0.0, 0.0, 1.0], abs=AXIS_TOL)
    assert rotation @ np.asarray(OPEN_AXIS) == pytest.approx([1.0, 0.0, 0.0], abs=AXIS_TOL)
    assert hib["tool_axis"] == pytest.approx([0.0, 0.0, 1.0], abs=AXIS_TOL)
    assert hib["open_axis"] == pytest.approx([1.0, 0.0, 0.0], abs=AXIS_TOL)
    h = ARM.handle
    assert h is not None
    margin, (w, height), t = ARM.margin_mm, ARM.plate_mm, ARM.thickness_mm
    point_mm = np.asarray(hib["tool_point_m"]) * MM_PER_M
    assert abs(point_mm[0] - (w / 2 - margin)) <= shoulder_top_mm(h) / 2 + TOOL_POINT_TOL_MM
    assert abs(point_mm[1] - (height - margin + h.post_offset_mm)) <= h.width_mm / 2
    assert -(h.tab_thickness_mm - t + h.extent_mm) < point_mm[2] < -(h.tab_thickness_mm - t)


def test_board_mass_and_centre_of_mass(
    arm_metadata: dict[str, Any], arm_bodies: tuple[trimesh.Trimesh, trimesh.Trimesh]
) -> None:
    """Mass in a plausible range, equal to the bare back-down plate plus the tab, the post and the gusset; the centre
    of mass in the board frame (m) on the board x centre line, between the plate centre and the post along board y,
    within the handle height of the pattern face."""
    h = ARM.handle
    assert h is not None
    bare = ARM.model_copy(update={"handle": None})
    plate_cm3 = sum(body.volume for body in print_bodies(bare, CONFIG.stl)) / 1000.0
    handle_cm3 = handle_body(h, ARM.plate_mm).volume() / 1000.0
    assert arm_metadata["mass_g"] == pytest.approx((plate_cm3 + handle_cm3) * h.density_g_per_cm3, rel=1e-4)
    assert MASS_RANGE_G[0] <= arm_metadata["mass_g"] <= MASS_RANGE_G[1]
    solid = printed_solid(*arm_bodies)
    mesh_to_board = np.linalg.inv(board_in_mesh(ARM.plate_mm, ARM.margin_mm, ARM.pattern_face, ARM.thickness_mm))
    expected = trimesh.transform_points([solid.center_mass], mesh_to_board)[0] / MM_PER_M
    com = np.asarray(arm_metadata["centre_of_mass_m"])
    assert com == pytest.approx(expected, abs=1e-6)
    centre = (np.asarray(ARM.plate_mm) / 2 - ARM.margin_mm) / MM_PER_M
    assert com[0] == pytest.approx(centre[0], abs=0.003)
    assert centre[1] < com[1] < (ARM.plate_mm[1] - ARM.margin_mm + h.post_offset_mm) / MM_PER_M
    assert -(h.tab_thickness_mm - ARM.thickness_mm + h.extent_mm) / MM_PER_M < com[2] < ARM.thickness_mm / MM_PER_M


def test_handle_envelope_boxes_hold_the_tab_post_and_gusset(
    arm_metadata: dict[str, Any], arm_bodies: tuple[trimesh.Trimesh, trimesh.Trimesh]
) -> None:
    """handle_envelope_mm: the post box (nominal bed face included), the gusset box and the tab box in the OpenCV
    board frame (pattern at z = 0, back at z = T); the handle lies on the pattern side (z < 0) and outside the plate
    outline (y > plate height - margin), and the boxes hold all material outside the plate slab."""
    h = ARM.handle
    assert h is not None
    envelope = handle_envelope_mm(ARM)
    assert flatten(arm_metadata["handle_envelope_mm"]) == pytest.approx(flatten(envelope), abs=1e-6)
    t, (w, height), m = ARM.thickness_mm, ARM.plate_mm, ARM.margin_mm
    assert envelope["height_mm"] == pytest.approx(h.tab_thickness_mm - t + h.extent_mm)
    boxes = [envelope["post"], envelope["gusset"], envelope["tab"]]
    edge = height - m  # board y of the plate edge the tab leaves
    for low, high in boxes:
        assert low[1] >= edge - 1e-6 and low[2] < 0.0
        assert 0.0 <= low[0] + m and high[0] + m <= w
    assert envelope["post"][0][2] == pytest.approx(-envelope["height_mm"])
    assert envelope["tab"][1][2] == pytest.approx(t)
    mesh_to_board = np.linalg.inv(board_in_mesh(ARM.plate_mm, m, ARM.pattern_face, t))
    grown = nominal_solid(printed_solid(*arm_bodies), h, ARM.plate_mm)
    v = grown.vertices
    outside = (v[:, 1] < -1e-6) | (v[:, 2] > t + 1e-6)  # off the plate slab x in [0, W], y in [0, H], z in [0, T]
    assert outside.sum() > 0
    points = trimesh.transform_points(v[outside], mesh_to_board)
    inside = np.zeros(len(points), dtype=bool)
    for low, high in boxes:
        inside |= np.all((points >= np.asarray(low) - 1e-3) & (points <= np.asarray(high) + 1e-3), axis=1)
    assert inside.all()


def test_shoulder_top_follows_the_spec() -> None:
    h = ARM.handle
    assert h is not None
    assert shoulder_top_mm(h) == h.thickness_tip_mm + h.rail_height_mm


def test_pattern_faces_back_toward_the_wrist(arm_metadata: dict[str, Any]) -> None:
    """User m540: the gripper holds the edge handle from the pattern side, so the pattern's outward normal n (board -z
    in gripper_link) points back against the tool approach axis t (TOOL_AXIS, wrist toward the finger tips)."""
    mount = pose_from_xyz_rpy(arm_metadata["board_mount"]["xyz_init"], arm_metadata["board_mount"]["rpy_init"])
    normal = -mount[:3, 2]
    assert float(normal @ np.asarray(TOOL_AXIS)) < FACING_DOT_MAX
    assert arm_metadata["handle_in_board"]["tool_axis"][2] > -FACING_DOT_MAX  # the tool axis is board +z


def test_gripper_does_not_cover_the_squares_seen_along_the_pattern_normal(arm_metadata: dict[str, Any]) -> None:
    """Projected along the pattern normal onto the board plane (board z dropped), the fixed jaw (servo housing
    included) and the moving jaw at the grip, the insert opening and the URDF upper limit lie beyond the ChArUco
    square area (board y > squares_y x square_mm, the tab side) by at least MIN_OCCLUSION_GAP_MM. A linear bound over
    a mesh is reached at a vertex, so the vertices give the exact gap."""
    hib = arm_metadata["handle_in_board"]
    gripper_in_board = pose_from_xyz_rpy(hib["gripper_link_xyz_m"], hib["gripper_link_rpy_rad"])
    gripper_in_board[:3, 3] *= MM_PER_M
    squares_y = ARM.squares_y * ARM.square_mm
    grip = arm_metadata["grip"]["grip_rad"]
    jaws = [GRIPPER.fixed, *(moving_jaw(GRIPPER, a) for a in (grip, INSERT_GRIPPER_RAD, GRIPPER.upper_rad))]
    for jaw in jaws:
        in_board = trimesh.transform_points(jaw.vertices, gripper_in_board)
        assert float(in_board[:, 1].min()) - squares_y >= MIN_OCCLUSION_GAP_MM
        assert float(in_board[:, 2].max()) < 0.0  # the whole gripper is on the pattern side


def test_board_mount_is_near_the_measured_stick_mount(arm_metadata: dict[str, Any]) -> None:
    """Ruling 2: the board plane lies in main's measured stick mount plane (2 cm past the tool point, the same
    normal), the board centre about 0.12 m off the tool axis on the same side (gripper_link +y), the long edges along
    OPEN_AXIS; the centre moves at most MAX_MOUNT_SHIFT_MM (along the plate edge) and the board turns at most
    MAX_MOUNT_TURN_DEG (the stick mount's yaw about the normal)."""
    stick = pose_from_xyz_rpy(STICK_XYZ_M, STICK_RPY_RAD)
    mount = pose_from_xyz_rpy(arm_metadata["board_mount"]["xyz_init"], arm_metadata["board_mount"]["rpy_init"])
    stick_centre = (stick @ np.array([*STICK_CENTRE_M, 1.0]))[:3]
    centre = (mount @ np.array([*((np.asarray(ARM.plate_mm) / 2 - ARM.margin_mm) / MM_PER_M), 0.0, 1.0]))[:3]
    assert float(np.linalg.norm(centre - stick_centre)) * MM_PER_M <= MAX_MOUNT_SHIFT_MM
    assert (centre[2] - stick_centre[2]) * MM_PER_M == pytest.approx(0.0, abs=PLANE_TOL_MM)
    assert mount[:3, 2] == pytest.approx(stick[:3, 2], abs=1e-6)
    turn = np.degrees(np.arccos((np.trace(stick[:3, :3].T @ mount[:3, :3]) - 1.0) / 2.0))
    assert turn <= MAX_MOUNT_TURN_DEG
    assert mount[:3, 0] == pytest.approx(OPEN_AXIS, abs=1e-6)  # the long edges along the jaw-closing direction
    tool_axis_point = GRIPPER.tool[:3, 3] / MM_PER_M
    assert (centre - tool_axis_point)[1] == pytest.approx(0.12, abs=0.005)


def test_stiffness_tab_share_is_the_closed_form_cantilever() -> None:
    """For a rectangular tab (root width = far width) the tab share of the centre sag is the closed-form cantilever
    P / (E I) (L^3 / 3 + e L^2 + e^2 L), L the tab span from the post wall to the plate edge, e = H / 2; the root
    stress is JERK_G P (L + e) (t / 2) / I."""
    h = ARM.handle
    assert h is not None
    square = h.model_copy(update={"tab_root_width_mm": h.tab_width_mm})
    got = stiffness(square, ARM.plate_mm, ARM.thickness_mm, 200.0)
    load = 0.2 * 9.81
    span, e = h.post_offset_mm - h.width_mm / 2, ARM.plate_mm[1] / 2
    inertia = h.tab_width_mm * h.tab_thickness_mm**3 / 12.0
    expected = load / (got["modulus_mpa"] * inertia) * (span**3 / 3 + e * span**2 + e**2 * span)
    assert got["tab_deflection_mm"] == pytest.approx(expected, rel=1e-4)
    assert got["root_stress_mpa"] == pytest.approx(10.0 * load * (span + e) * h.tab_thickness_mm / 2 / inertia)
    assert got["centre_deflection_mm"] == pytest.approx(got["tab_deflection_mm"] + got["plate_deflection_mm"])
    assert got["plate_deflection_mm"] > 0.0


def test_tab_holds_the_board_centre_within_the_deflection_limit(arm_metadata: dict[str, Any]) -> None:
    """Ruling 5: under 1 g normal to the plate the tab sags the board centre by at most MAX_TAB_DEFLECTION_MM; the
    JSON records the estimate at the board mass."""
    h = ARM.handle
    assert h is not None
    expected = stiffness(h, ARM.plate_mm, ARM.thickness_mm, arm_metadata["mass_g"])
    assert flatten(arm_metadata["stiffness"]) == pytest.approx(flatten(expected), abs=1e-9)
    assert arm_metadata["stiffness"]["tab_deflection_mm"] <= MAX_TAB_DEFLECTION_MM
