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
    board_config,
    board_metadata,
    make_charuco_board,
    print_bodies,
    write_board_metadata,
    write_charuco,
)
from scripts.markers.config import load_generator_config
from scripts.markers.handle import HandleSpec, board_in_mesh, face_down, handle_solid
from scripts.markers.jaw_fit import (
    clearance_mm,
    load_gripper,
    moving_jaw,
    penetration_mm,
    surface_points,
)
from scripts.markers.mesh import box, print_violations, printed_solid, to_trimesh

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


def seated_handle(meta: dict[str, Any], bodies: tuple[trimesh.Trimesh, trimesh.Trimesh]) -> trimesh.Trimesh:
    """The handle part of the printed solid moved into gripper_link with handle_in_board.

    The solid is cut at mesh y = HANDLE_CUT_MM (capped, watertight), so surface samples are dense where the jaws are.

    Args:
        meta (dict[str, Any]): Board metadata.
        bodies (tuple[trimesh.Trimesh, trimesh.Trimesh]): (base, modules).

    Returns:
        trimesh.Trimesh: The seated handle (mm, gripper_link).
    """
    solid = printed_solid(*bodies)
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


def plate_region_top(base: trimesh.Trimesh) -> float:
    """Highest base vertex over the plate (mesh y >= HANDLE_CUT_MM).

    Args:
        base (trimesh.Trimesh): Base body.

    Returns:
        float: z (mm).
    """
    return float(base.vertices[base.vertices[:, 1] >= HANDLE_CUT_MM][:, 2].max())


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
    assert plate_region_top(base) == pytest.approx(ARM.thickness_mm, abs=1e-3)


def test_seated_handle_touches_both_jaws_without_penetration(
    arm_metadata: dict[str, Any], arm_bodies: tuple[trimesh.Trimesh, trimesh.Trimesh]
) -> None:
    """Both directions of penetration (penetration_mm only tests `other` inside `solid`) and contact on both jaws.

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
    top = h.thickness_tip_mm + (h.thickness_end_mm - h.thickness_tip_mm) * grip_depth / h.length_mm
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
