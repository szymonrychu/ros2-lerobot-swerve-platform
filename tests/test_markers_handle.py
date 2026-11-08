"""Tests for scripts/markers/handle.py and the ARM board flow of write_charuco: paddle handle, face-down print,
seat in the SO-101 jaws (URDF meshes), metadata JSON and the shipped outputs."""

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
    handle_solid,
    nominal_solid,
    printed_contact_rad,
    rib_solid,
    rib_x_ranges_mm,
    shoulder_top_mm,
)
from scripts.markers.jaw_fit import (
    CONTACT_HI_RAD,
    OPEN_AXIS,
    clearance_mm,
    load_gripper,
    moving_jaw,
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
HANDLE_CUT_MM = 0.5  # mesh y below which the printed solid is the handle (plate starts at y = 0)
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
# Task 6c (user decision 292): twin ribs on the board back. The jaw region must stay exactly as at 96322bd1.
BASE_GRIP = {"contact_rad": 0.07027905529357185, "modelled_contact_rad": 0.06620181229285478}
BASE_BOARD_MOUNT = {
    "xyz_init": [-0.007749999618530183, -0.08405085943012967, -0.2551131187031419],
    "rpy_init": [1.5707963267948966, 0.0, 1.5707963267948966],
}
BASE_TOOL_POINT_M = [0.07823273843012968, 0.15838571604954718, -0.0011500040964920074]
BASE_MASS_G = 156.86370096027835  # the unribbed board at 96322bd1
RIB_MASS_RANGE_G = (5.0, 12.0)  # the decision's "about +8 g"
INVARIANT_TOL = 1e-6
RIB_PROBE_MM = 0.5  # column and slab offsets off the rib ends


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

    The solid is cut at mesh y = HANDLE_CUT_MM (capped, watertight), so surface samples are dense where the jaws are.

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
    handle = trimesh.intersections.slice_mesh_plane(solid, (0.0, -1.0, 0.0), (0.0, HANDLE_CUT_MM, 0.0), cap=True)
    return handle.apply_transform(mesh_in_gripper_link(meta))


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


def plate_region_top(base: trimesh.Trimesh, ribs: bool = True) -> float:
    """Highest base vertex over the plate (mesh y >= HANDLE_CUT_MM), with or without the ribs.

    Args:
        base (trimesh.Trimesh): Base body.
        ribs (bool): True counts every vertex over the plate; False only those past the rib ends (mesh y beyond
            rib_length_mm + HANDLE_CUT_MM), i.e. the bare plate back.

    Returns:
        float: z (mm).
    """
    assert ARM.handle is not None
    start = HANDLE_CUT_MM if ribs else ARM.handle.rib_length_mm + HANDLE_CUT_MM
    return float(base.vertices[base.vertices[:, 1] >= start][:, 2].max())


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
        ({"rib_width_mm": 0.0}, "rib_width_mm"),
        ({"rib_width_mm": 11.0}, "rib_width_mm"),
        ({"rib_length_mm": 0.0}, "rib_length_mm"),
        ({"rib_end_height_mm": 0.0}, "rib_end_height_mm"),
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


def test_bed_check_includes_the_handle_extent() -> None:
    assert ARM.handle is not None
    tight = ARM.plate_mm[1] + ARM.handle.extent_mm - 1.0
    with pytest.raises(ValidationError, match="handle"):
        CharucoSpec.model_validate({**ARM.model_dump(), "bed_mm": tight})


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


def test_handle_solid_alone_is_printable_and_sized() -> None:
    h = ARM.handle
    assert h is not None
    solid = to_trimesh(handle_solid(h, ARM.plate_mm))
    assert print_violations(solid) == []
    centre = ARM.plate_mm[0] / 2
    expected = [
        [centre - h.width_mm / 2, -h.extent_mm, 0.0],
        [centre + h.width_mm / 2, 0.0, h.thickness_end_mm + h.rail_height_mm],
    ]
    assert solid.bounds == pytest.approx(np.array(expected), abs=1e-3)


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
    base, _ = arm_bodies
    h = ARM.handle
    assert h is not None
    assert base.bounds[0][1] == pytest.approx(-h.extent_mm, abs=1e-3)
    assert base.bounds[1][2] == pytest.approx(h.thickness_end_mm + h.rail_height_mm, abs=1e-3)
    assert plate_region_top(base, ribs=False) == pytest.approx(ARM.thickness_mm, abs=1e-3)


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
    (jaw_gap_mm): the moving finger has a hole on its mid-plane about 28 mm behind the tip.
    """
    h = ARM.handle
    assert h is not None
    jaw = moving_jaw(GRIPPER, arm_metadata["grip"]["contact_rad"])
    points = trimesh.transform_points(
        surface_points(jaw, LAND_SAMPLES), np.linalg.inv(mesh_in_gripper_link(arm_metadata))
    )
    grip_depth = -h.neck_mm - points[:, 1]  # behind the shoulder face (mesh y = -neck)
    over_channel = np.abs(points[:, 0] - ARM.plate_mm[0] / 2) <= h.channel_tip_mm / 2
    keep = over_channel & (grip_depth >= 0.0) & (grip_depth <= h.length_mm)
    points, grip_depth = points[keep], grip_depth[keep]
    # nominal top plane: the nominal bed face lies clamp_clearance_mm below the printed one (mesh z = -c)
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
    assert isinstance(handle_solid(ARM.handle, ARM.plate_mm), Manifold)


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
    printed = handle_solid(h, ARM.plate_mm)
    nominal = handle_solid(h, ARM.plate_mm, nominal=True)
    centre = ARM.plate_mm[0] / 2
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
    allowance = handle_solid(h, ARM.plate_mm, nominal=True) - handle_solid(h, ARM.plate_mm)
    assert grown.is_watertight
    assert grown.volume == pytest.approx(solid.volume + allowance.volume(), rel=1e-6)
    assert grown.bounds[0][2] == pytest.approx(-h.clamp_clearance_mm, abs=1e-3)
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
    handle = trimesh.intersections.slice_mesh_plane(solid, (0.0, -1.0, 0.0), (0.0, HANDLE_CUT_MM, 0.0), cap=True)
    handle.apply_transform(pressed)
    assert clearance_mm(handle, GRIPPER.fixed) <= CONTACT_TOL_MM
    assert penetration_mm(handle, GRIPPER.fixed) <= CONTACT_TOL_MM


def rib_tops_mm() -> tuple[float, float]:
    """Rib top z (print coordinates) at the plate edge and at the rib end.

    Returns:
        tuple[float, float]: (shoulder top = thickness_tip_mm + rail_height_mm, thickness_mm + rib_end_height_mm).
    """
    h = ARM.handle
    assert h is not None
    return h.thickness_tip_mm + h.rail_height_mm, ARM.thickness_mm + h.rib_end_height_mm


def rib_top_at_mm(y: float) -> float:
    """Rib top z at mesh y on the linear taper.

    Args:
        y (float): Mesh y in [0, rib_length_mm] (mm).

    Returns:
        float: z (mm).
    """
    assert ARM.handle is not None
    edge, end = rib_tops_mm()
    return edge + (end - edge) * y / ARM.handle.rib_length_mm


def expected_rib_x_ranges_mm() -> list[tuple[float, float]]:
    """The two rib x ranges, flush with the handle outer side walls.

    Returns:
        list[tuple[float, float]]: [(x0, x1) of the low-x rib, (x0, x1) of the high-x rib] (mm).
    """
    h = ARM.handle
    assert h is not None
    centre, half = ARM.plate_mm[0] / 2, h.width_mm / 2
    return [(centre - half, centre - half + h.rib_width_mm), (centre + half - h.rib_width_mm, centre + half)]


def test_rib_solid_tapers_from_the_shoulder_top_to_the_end_height() -> None:
    h = ARM.handle
    assert h is not None
    ribs = rib_solid(h, ARM.plate_mm, ARM.thickness_mm)
    edge, _ = rib_tops_mm()
    (lo_x0, _), (_, hi_x1) = expected_rib_x_ranges_mm()
    expected = [[lo_x0, 0.0, ARM.thickness_mm], [hi_x1, h.rib_length_mm, edge]]
    assert to_trimesh(ribs).bounds == pytest.approx(np.array(expected), abs=1e-3)
    for x0, x1 in expected_rib_x_ranges_mm():
        for y in (RIB_PROBE_MM, h.rib_length_mm - RIB_PROBE_MM):
            lo, hi = column_z(ribs, (x0 + x1) / 2, y)
            assert lo == pytest.approx(ARM.thickness_mm, abs=1e-3)
            assert hi == pytest.approx(rib_top_at_mm(y - COLUMN_MM), abs=1e-3)


def test_ribs_continue_the_handle_side_walls_as_two_full_height_webs(
    arm_bodies: tuple[trimesh.Trimesh, trimesh.Trimesh],
) -> None:
    """Cross-section of the printed base at the plate edge, above the plate back: two webs of rib_width_mm at the
    shoulder height, flush with the handle outer walls, nothing between them; at rib_length_mm the rib top is
    thickness_mm + rib_end_height_mm."""
    h = ARM.handle
    assert h is not None
    base = to_manifold(arm_bodies[0])
    width, t = ARM.plate_mm[0], ARM.thickness_mm
    web_top, web_bottom = rib_top_at_mm(RIB_PROBE_MM), t + RIB_PROBE_MM  # clear of the plate back face
    shoulder = to_trimesh(base ^ box(0.0, -h.neck_mm, web_bottom, width, 0.0, web_top)).bounds
    halves = (  # from COLUMN_MM: clear of the shoulder face at y = 0
        box(0.0, COLUMN_MM, web_bottom, width / 2, RIB_PROBE_MM, web_top),
        box(width / 2, COLUMN_MM, web_bottom, width, RIB_PROBE_MM, web_top),
    )
    for (x0, x1), half in zip(expected_rib_x_ranges_mm(), halves, strict=True):
        web = base ^ half
        assert to_trimesh(web).bounds[:, 0] == pytest.approx([x0, x1], abs=1e-3)
        assert web.volume() == pytest.approx(
            h.rib_width_mm * (RIB_PROBE_MM - COLUMN_MM) * (web_top - web_bottom), rel=1e-4
        )
    (lo_x0, _), (_, hi_x1) = expected_rib_x_ranges_mm()
    assert shoulder[:, 0] == pytest.approx([lo_x0, hi_x1], abs=1e-3)
    for x0, x1 in expected_rib_x_ranges_mm():
        _, end_top = column_z(base, (x0 + x1) / 2, h.rib_length_mm - COLUMN_MM)
        assert end_top == pytest.approx(rib_top_at_mm(h.rib_length_mm - 2 * COLUMN_MM), abs=1e-3)
        _, past = column_z(base, (x0 + x1) / 2, h.rib_length_mm + RIB_PROBE_MM)
        assert past == pytest.approx(t, abs=1e-3)
    _, between = column_z(base, width / 2, RIB_PROBE_MM)
    assert between == pytest.approx(t, abs=1e-3)


def test_ribs_lie_inside_the_plate_and_off_the_pattern_face(
    arm_bodies: tuple[trimesh.Trimesh, trimesh.Trimesh],
) -> None:
    h = ARM.handle
    assert h is not None
    base, modules = arm_bodies
    ribs = rib_solid(h, ARM.plate_mm, ARM.thickness_mm)
    lo, hi = to_trimesh(ribs).bounds
    assert lo[0] >= 0.0 and lo[1] >= 0.0 and hi[0] <= ARM.plate_mm[0] and hi[1] <= ARM.plate_mm[1]
    assert lo[2] == pytest.approx(ARM.thickness_mm, abs=1e-3)
    assert (ribs ^ to_manifold(modules)).volume() == pytest.approx(0.0, abs=1e-6)
    for x0, x1 in expected_rib_x_ranges_mm():
        below = box(x0, 0.0, -1.0, x1, h.rib_length_mm, ARM.thickness_mm)
        filled = (to_manifold(base) ^ below).volume() + (to_manifold(modules) ^ below).volume()
        assert filled == pytest.approx((x1 - x0) * h.rib_length_mm * ARM.thickness_mm, rel=1e-6)


def test_jaw_region_is_unchanged_by_the_ribs(arm_metadata: dict[str, Any]) -> None:
    """Seat pose, contact_rad, modelled_contact_rad, grip_rad and the CAD mount seed are the 96322bd1 values."""
    grip = arm_metadata["grip"]
    for key, value in BASE_GRIP.items():
        assert grip[key] == pytest.approx(value, abs=INVARIANT_TOL)
    assert grip["grip_rad"] == pytest.approx(BASE_GRIP["contact_rad"] - grip["squeeze_rad"], abs=INVARIANT_TOL)
    assert flatten(arm_metadata["board_mount"]) == pytest.approx(flatten(BASE_BOARD_MOUNT), abs=INVARIANT_TOL)
    assert arm_metadata["handle_in_board"]["tool_point_m"] == pytest.approx(BASE_TOOL_POINT_M, abs=INVARIANT_TOL)


def test_ribs_clear_the_jaws_at_insert_and_over_the_seat_sweep(arm_metadata: dict[str, Any]) -> None:
    """The moving finger at the insert opening and at every shoulder-sweep angle (contact to CONTACT_HI_RAD), and
    the fixed jaw, stay out of the ribs."""
    h = ARM.handle
    assert h is not None
    ribs = to_trimesh(rib_solid(h, ARM.plate_mm, ARM.thickness_mm)).apply_transform(mesh_in_gripper_link(arm_metadata))
    angles = [INSERT_GRIPPER_RAD, *np.arange(arm_metadata["grip"]["contact_rad"], CONTACT_HI_RAD, SWEEP_STEP_RAD)]
    for angle in angles:
        assert penetration_mm(ribs, moving_jaw(GRIPPER, float(angle))) == 0.0
    assert penetration_mm(ribs, GRIPPER.fixed) == 0.0
    assert clearance_mm(ribs, moving_jaw(GRIPPER, INSERT_GRIPPER_RAD)) > 0.0


def test_ribs_add_about_eight_grams(
    arm_metadata: dict[str, Any], arm_bodies: tuple[trimesh.Trimesh, trimesh.Trimesh]
) -> None:
    h = ARM.handle
    assert h is not None
    added = arm_metadata["mass_g"] - BASE_MASS_G
    assert RIB_MASS_RANGE_G[0] <= added <= RIB_MASS_RANGE_G[1]
    rib_g = rib_solid(h, ARM.plate_mm, ARM.thickness_mm).volume() / 1000.0 * h.density_g_per_cm3
    assert added == pytest.approx(rib_g, rel=1e-3)


def test_back_envelope_is_the_rib_height_and_footprint_in_the_board_frame(
    arm_metadata: dict[str, Any], arm_bodies: tuple[trimesh.Trimesh, trimesh.Trimesh]
) -> None:
    h = ARM.handle
    assert h is not None
    envelope = back_envelope_mm(ARM)
    assert flatten(arm_metadata["back_envelope_mm"]) == pytest.approx(flatten(envelope), abs=1e-6)
    edge, _ = rib_tops_mm()
    t = ARM.thickness_mm
    base = to_manifold(arm_bodies[0])
    # the tallest material over the plate is the rib top at the plate edge
    tallest = max(column_z(base, (x0 + x1) / 2, COLUMN_MM)[1] for x0, x1 in expected_rib_x_ranges_mm())
    assert envelope["height_mm"] == pytest.approx(tallest - t, abs=1e-3)
    assert plate_region_top(arm_bodies[0]) - t <= envelope["height_mm"]
    assert envelope["height_mm"] == pytest.approx(edge - t, abs=1e-9)
    assert envelope["end_height_mm"] == h.rib_end_height_mm
    mesh_to_board = np.linalg.inv(board_in_mesh(ARM.plate_mm, ARM.margin_mm, ARM.pattern_face, t))
    expected = []
    for x0, x1 in expected_rib_x_ranges_mm():
        corners = trimesh.transform_points(np.array([[x0, 0.0, t], [x1, h.rib_length_mm, edge]]), mesh_to_board)
        expected.append([corners.min(axis=0).tolist(), corners.max(axis=0).tolist()])
    expected.sort()
    assert flatten(envelope["ribs"]) == pytest.approx(flatten(expected), abs=1e-9)
    for low, high in envelope["ribs"]:
        assert low[2] == pytest.approx(t) and high[2] == pytest.approx(edge)
        assert 0.0 <= low[1] and high[1] <= ARM.plate_mm[1] - ARM.margin_mm


@pytest.mark.parametrize(
    ("update", "field"),
    [
        ({"handle": {"rib_length_mm": 151.0}}, "rib_length_mm"),
        ({"handle": {"rib_end_height_mm": 21.0}}, "rib_end_height_mm"),
        ({"pattern_face": "top"}, "pattern_face"),
    ],
)
def test_charuco_spec_refuses_ribs_off_the_plate_back(update: dict[str, Any], field: str) -> None:
    spec = ARM.model_dump()
    spec["handle"] = {**spec["handle"], **update.get("handle", {})}
    spec.update({k: v for k, v in update.items() if k != "handle"})
    with pytest.raises(ValidationError, match=field):
        CharucoSpec.model_validate(spec)


def test_shoulder_top_and_rib_ranges_follow_the_spec() -> None:
    h = ARM.handle
    assert h is not None
    assert shoulder_top_mm(h) == h.thickness_tip_mm + h.rail_height_mm
    assert rib_x_ranges_mm(h, ARM.plate_mm) == pytest.approx(expected_rib_x_ranges_mm())
    (lo_x0, lo_x1), (hi_x0, hi_x1) = rib_x_ranges_mm(h, ARM.plate_mm)
    assert lo_x1 - lo_x0 == pytest.approx(h.rib_width_mm) and hi_x1 - hi_x0 == pytest.approx(h.rib_width_mm)
    assert hi_x1 - lo_x0 == pytest.approx(h.width_mm)
