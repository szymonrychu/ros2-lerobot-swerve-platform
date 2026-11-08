"""ChArUco calibration board: spec, OpenCV board, exact-pixel dark mask, PDF, two-colour STL/3MF output and, for a
board with a handle, the metadata JSON (mass, centre of mass, seat in the gripper, grip angles, handle envelope, tab
stiffness)."""

import json
import logging
from math import isclose
from pathlib import Path
from typing import Any, Literal, Self

import cv2
import numpy as np
import trimesh
from pydantic import Field, field_validator, model_validator
from ros2_common.head_rgbd_calibration.chain import xyz_rpy_from_pose
from ros2_common.head_rgbd_calibration.config import BoardConfig

from scripts.markers.handle import (
    HandleSpec,
    board_in_mesh,
    face_down,
    handle_body,
    post_in_mesh,
    printed_contact_rad,
    seat,
    shoulder_top_mm,
    stiffness,
)
from scripts.markers.jaw_fit import OPEN_AXIS, TOOL_AXIS, Gripper
from scripts.markers.mesh import mask_bodies, print_violations, printed_solid, to_manifold, to_trimesh, write_bodies
from scripts.markers.models import OUTPUT_FORMATS, PageSpec, StlOptions, StrictModel
from scripts.markers.pdf import draw_mask_pdf, page_mm

LOGGER = logging.getLogger(__name__)
MM_PER_M = 1000.0
MIN_THICKNESS_MM = 4.0
PX_PER_MM_STEP = 0.05
PX_PER_MM_MAX = 20.0
EXACT_TOL = 1e-6
DARK_BELOW = 128
PDF_TOP_MM = 2.5
LABEL_BELOW_BOARD_MM = 5.0
NAME_PATTERN = r"^[a-z0-9][a-z0-9-]*$"
MESH_SUFFIX = {"stl": ("-base.stl", "-modules.stl"), "3mf": (".3mf",)}
MM3_PER_CM3 = 1000.0
JSON_INDENT = 2


def board_config(spec: "CharucoSpec") -> BoardConfig:
    """The calibration-side board definition of a generator spec.

    Args:
        spec: Generator board spec (CharucoSpec).

    Returns:
        The BoardConfig with the fields both sides share.
    """
    return BoardConfig.model_validate(spec.model_dump(include=set(BoardConfig.model_fields)))


class CharucoSpec(StrictModel):
    """ChArUco board definition shared by the generator and the calibration tool.

    name: file stem and board id (lower case, digits, hyphens). ids_offset: first ArUco marker id; the board uses
    squares_x * squares_y // 2 consecutive ids from it, so boards sharing a dictionary can have disjoint ids.
    squares_x / squares_y: chessboard squares (squares_y odd). square_mm / marker_mm: square and ArUco marker edge.
    dictionary: any cv2.aruco.DICT_* name. margin_mm: plate border around the board. thickness_mm: STL base
    thickness (>= 4). relief_mm: module height above the base. inlay_mm: module depth sunk into the base (flush
    inlay when relief_mm is 0). bed_mm: print bed edge the plate (with the handle) must fit. page: PDF media.
    outputs: subset of pdf/stl/3mf. pattern_face: "top" prints the pattern face up (back on the bed, the inlays in
    the top layers), "bottom" face down on the bed (the smooth bed face carries the pattern). handle: optional edge
    handle for the gripper (user m540): a tab off the middle of the plate edge with the grip post standing on it on
    the pattern side; it needs pattern_face "top" (the board prints back down, the post rises from the tab), a tab
    thicker than the plate and no wider than it, plate plus tab within bed_mm, and a board with a handle also writes
    <stem>.json.
    """

    name: str = Field(pattern=NAME_PATTERN)
    squares_x: int = Field(ge=2)
    squares_y: int = Field(ge=2)
    square_mm: float = Field(gt=0)
    marker_mm: float = Field(gt=0)
    dictionary: str
    ids_offset: int = Field(ge=0)
    margin_mm: float = Field(default=5.0, ge=0)
    thickness_mm: float = Field(default=MIN_THICKNESS_MM, ge=MIN_THICKNESS_MM)
    relief_mm: float = Field(default=0.0, ge=0)
    inlay_mm: float = Field(default=1.2, ge=0)
    bed_mm: float = Field(default=256.0, gt=0)
    page: PageSpec = Field(default_factory=lambda: PageSpec(size="A4", orientation="landscape"))
    outputs: list[Literal["pdf", "stl", "3mf"]] = Field(default_factory=lambda: list(OUTPUT_FORMATS), min_length=1)
    pattern_face: Literal["top", "bottom"]
    handle: HandleSpec | None = None

    @field_validator("dictionary")
    @classmethod
    def check_dictionary(cls, value: str) -> str:
        """The dictionary is a cv2.aruco.DICT_* name."""
        if not value.startswith("DICT_") or not hasattr(cv2.aruco, value):
            raise ValueError(f"dictionary {value!r} is not a cv2.aruco.DICT_* name")
        return value

    @field_validator("squares_y")
    @classmethod
    def check_odd_rows(cls, value: int) -> int:
        """An even row count flips the legacy corner layout, so it is refused."""
        if value % 2 == 0:
            raise ValueError(f"squares_y {value} must be odd (an even row count changes the legacy board layout)")
        return value

    @model_validator(mode="after")
    def check_ids(self) -> Self:
        """The marker ids ids_offset .. ids_offset + squares_x * squares_y // 2 - 1 exist in the dictionary."""
        available = len(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.dictionary)).bytesList)
        if self.ids_offset + self.squares_x * self.squares_y // 2 > available:
            raise ValueError(
                f"ids_offset {self.ids_offset} + {self.squares_x * self.squares_y // 2} markers "
                f"exceeds the {available} ids of {self.dictionary}"
            )
        return self

    @model_validator(mode="after")
    def check_geometry(self) -> Self:
        """marker_mm < square_mm, modules have height, inlay < thickness, plate fits bed_mm, a handle needs a
        back-down (pattern up) board, its tab thicker than the plate and no wider, plate plus tab within bed_mm."""
        if self.relief_mm + self.inlay_mm <= 0:
            raise ValueError("relief_mm + inlay_mm must be positive")
        if self.inlay_mm >= self.thickness_mm:
            raise ValueError(f"inlay_mm {self.inlay_mm} must be below thickness_mm {self.thickness_mm}")
        if self.marker_mm >= self.square_mm:
            raise ValueError(f"marker_mm {self.marker_mm} must be below square_mm {self.square_mm}")
        for field, squares in (("squares_x", self.squares_x), ("squares_y", self.squares_y)):
            plate = squares * self.square_mm + 2 * self.margin_mm
            if plate > self.bed_mm:
                raise ValueError(f"{field} {squares}: plate {plate:g} mm exceeds bed_mm {self.bed_mm:g}")
        if self.pattern_face == "bottom" and self.relief_mm > 0:
            raise ValueError(f"relief_mm {self.relief_mm} must be 0 for pattern_face bottom (modules flush on the bed)")
        if self.handle is None:
            return self
        if self.pattern_face != "top":
            raise ValueError(
                "a handle needs pattern_face top (the board prints back down, the post rises from the tab)"
            )
        if self.handle.tab_thickness_mm <= self.thickness_mm:
            raise ValueError(
                f"tab_thickness_mm {self.handle.tab_thickness_mm:g} must exceed thickness_mm {self.thickness_mm:g}"
            )
        if self.handle.tab_root_width_mm > self.plate_mm[0]:
            raise ValueError(
                f"tab_root_width_mm {self.handle.tab_root_width_mm:g} exceeds the plate edge {self.plate_mm[0]:g} mm"
            )
        length = self.plate_mm[1] + self.handle.tab_length_mm
        if length > self.bed_mm:
            raise ValueError(f"plate plus tab_length_mm {length:g} mm exceeds bed_mm {self.bed_mm:g}")
        return self

    @property
    def plate_mm(self) -> tuple[float, float]:
        """Plate (width, height) in mm."""
        return (
            self.squares_x * self.square_mm + 2 * self.margin_mm,
            self.squares_y * self.square_mm + 2 * self.margin_mm,
        )

    @property
    def marker_ids(self) -> range:
        """ArUco marker ids used by the board."""
        return range(self.ids_offset, self.ids_offset + self.squares_x * self.squares_y // 2)

    @property
    def marker_cell_mm(self) -> float:
        """Edge of one ArUco marker cell (marker bits plus the 1-cell border) in mm."""
        size = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.dictionary)).markerSize
        return self.marker_mm / (size + 2)


def make_charuco_board(spec: CharucoSpec) -> cv2.aruco.CharucoBoard:
    """Build the OpenCV board (lengths in metres).

    Args:
        spec: The board definition.

    Returns:
        The cv2.aruco.CharucoBoard.
    """
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, spec.dictionary))
    return cv2.aruco.CharucoBoard(
        (spec.squares_x, spec.squares_y),
        spec.square_mm / MM_PER_M,
        spec.marker_mm / MM_PER_M,
        dictionary,
        np.arange(spec.marker_ids.start, spec.marker_ids.stop, dtype=np.int32),
    )


def is_whole(value: float) -> bool:
    """Whether value is an integer within EXACT_TOL."""
    return isclose(value, round(value), abs_tol=EXACT_TOL)


def exact_px_per_mm(spec: CharucoSpec) -> float:
    """Smallest resolution where squares, marker cells and the square-to-marker gap are whole pixels.

    Args:
        spec: The board definition.

    Returns:
        Pixels per mm.

    Raises:
        ValueError: when no resolution up to PX_PER_MM_MAX is exact.
    """
    lengths = (spec.square_mm, spec.marker_mm, spec.marker_cell_mm, (spec.square_mm - spec.marker_mm) / 2)
    steps = int(PX_PER_MM_MAX / PX_PER_MM_STEP)
    for step in range(1, steps + 1):
        ppm = step * PX_PER_MM_STEP
        if all(is_whole(length * ppm) for length in lengths):
            return ppm
    raise ValueError("no exact raster resolution for this square and marker size")


def board_mask(spec: CharucoSpec) -> tuple[np.ndarray, float]:
    """Dark mask of the board and its cell size.

    Args:
        spec: The board definition.

    Returns:
        (mask, cell_mm): bool array, True = dark, row 0 on top; cell_mm is the edge of one mask pixel.
    """
    ppm = exact_px_per_mm(spec)
    width, height = round(spec.squares_x * spec.square_mm * ppm), round(spec.squares_y * spec.square_mm * ppm)
    assert is_whole(spec.squares_x * spec.square_mm * ppm) and is_whole(spec.squares_y * spec.square_mm * ppm)
    image = make_charuco_board(spec).generateImage((width, height), marginSize=0, borderBits=1)
    assert image.shape == (height, width), f"board image {image.shape} differs from the exact size {(height, width)}"
    return image < DARK_BELOW, 1.0 / ppm


def handle_envelope_mm(spec: CharucoSpec) -> dict[str, Any]:
    """Material of a board with a handle beyond its plate: the post, its gusset and the tab, in the OpenCV board frame.

    The board frame has the pattern at z = 0 and the back at z = T; the handle lies on the pattern side (z < 0) and
    outside the plate outline (y > plate height - margin).

    Args:
        spec (CharucoSpec): The board (with a handle).

    Returns:
        dict[str, Any]: height_mm (how far the post top stands above the pattern face), post (the post box: rails
        width, from the nominal bed face to the taller rail top, from the tab top to the free end), gusset (the box
        around the gusset web) and tab (the tab box, from the plate back to the tab top); each box is [[x, y, z] min,
        [x, y, z] max] in mm.

    Raises:
        ValueError: When the spec has no handle.
    """
    handle = spec.handle
    if handle is None:
        raise ValueError(f"board {spec.name} has no handle")
    mesh_to_board = np.linalg.inv(board_in_mesh(spec.plate_mm, spec.margin_mm, spec.pattern_face, spec.thickness_mm))
    to_board = mesh_to_board @ post_in_mesh(handle, spec.plate_mm)
    half, top = handle.width_mm / 2, max(handle.thickness_tip_mm, handle.thickness_end_mm) + handle.rail_height_mm
    web = (shoulder_top_mm(handle) - handle.gusset_thickness_mm) / 2  # handle z of the gusset web faces: web, top - web
    corners = {
        "post": ([[-half, -handle.extent_mm, -handle.clamp_clearance_mm], [half, 0.0, top]], to_board),
        "gusset": (
            [
                [-half - handle.gusset_reach_mm, -handle.gusset_height_mm, web],
                [-half, 0.0, web + handle.gusset_thickness_mm],
            ],
            to_board,
        ),
        "tab": (
            [
                [(spec.plate_mm[0] - handle.tab_root_width_mm) / 2, -handle.tab_length_mm, 0.0],
                [(spec.plate_mm[0] + handle.tab_root_width_mm) / 2, 0.0, handle.tab_thickness_mm],
            ],
            mesh_to_board,
        ),
    }
    boxes: dict[str, Any] = {}
    for name, (pair, transform) in corners.items():
        points = trimesh.transform_points(np.array(pair), transform)
        boxes[name] = [[float(v) for v in points.min(axis=0)], [float(v) for v in points.max(axis=0)]]
    return {"height_mm": handle.tab_thickness_mm - spec.thickness_mm + handle.extent_mm, **boxes}


def board_metadata(
    spec: CharucoSpec, base: trimesh.Trimesh, modules: trimesh.Trimesh, gripper: Gripper
) -> dict[str, Any]:
    """Mass, seat and grip of a board with a handle, from its print bodies and the gripper meshes.

    Args:
        spec (CharucoSpec): The board (with a handle).
        base (trimesh.Trimesh): Base body in print coordinates (handle included).
        modules (trimesh.Trimesh): Module body in print coordinates.
        gripper (Gripper): Loaded SO-101 gripper.

    Returns:
        dict[str, Any]: name, board (BoardConfig dump), plate_mm, volume_cm3, mass_g, centre_of_mass_m (board
        frame, m); handle_in_board (gripper_link_xyz_m and gripper_link_rpy_rad: gripper_link in the board frame;
        tool_point_m: the handle tool_offset_m point in the board frame; tool_axis and open_axis: gripper_link
        TOOL_AXIS and OPEN_AXIS in the board frame, the post axis and the jaw-closing direction); board_mount
        (xyz_init, rpy_init: the board frame in gripper_link, m and rad); grip (contact_rad: on the nominal handle
        surfaces, squeeze_rad, grip_rad = contact_rad - squeeze_rad, clearance_clamp_mm: the print allowance on each
        clamp face, modelled_contact_rad: the contact on the printed, thinner solid pressed on the fixed jaw);
        handle_envelope_mm (`handle_envelope_mm`); stiffness (`stiffness` at mass_g).

    Raises:
        ValueError: When the spec has no handle or the seat fails.
    """
    if spec.handle is None:
        raise ValueError(f"board {spec.name} has no handle")
    solid = printed_solid(base, modules)
    mesh_pose, contact = seat(gripper, solid, spec.handle, spec.plate_mm)
    modelled = printed_contact_rad(gripper, solid, spec.handle, mesh_pose)
    board_in = board_in_mesh(spec.plate_mm, spec.margin_mm, spec.pattern_face, spec.thickness_mm)
    centre_of_mass = trimesh.transform_points([solid.center_mass], np.linalg.inv(board_in))[0] / MM_PER_M
    board_pose = mesh_pose @ board_in
    board_pose[:3, 3] /= MM_PER_M
    gripper_in_board = np.linalg.inv(board_pose)
    tool = gripper.tool.copy()
    tool[:3, 3] /= MM_PER_M
    tool_point = gripper_in_board @ tool @ np.array([*spec.handle.tool_offset_m, 1.0])
    gripper_xyz, gripper_rpy = xyz_rpy_from_pose(gripper_in_board)
    board_xyz, board_rpy = xyz_rpy_from_pose(board_pose)
    volume = (base.volume + modules.volume) / MM3_PER_CM3
    mass = float(volume * spec.handle.density_g_per_cm3)
    return {
        "name": spec.name,
        "board": board_config(spec).model_dump(),
        "plate_mm": list(spec.plate_mm),
        "volume_cm3": float(volume),
        "mass_g": mass,
        "handle_in_board": {
            "gripper_link_xyz_m": list(gripper_xyz),
            "gripper_link_rpy_rad": list(gripper_rpy),
            "tool_point_m": [float(v) for v in tool_point[:3]],
            "tool_axis": [float(v) for v in gripper_in_board[:3, :3] @ np.asarray(TOOL_AXIS)],
            "open_axis": [float(v) for v in gripper_in_board[:3, :3] @ np.asarray(OPEN_AXIS)],
        },
        "centre_of_mass_m": [float(v) for v in centre_of_mass],
        "handle_envelope_mm": handle_envelope_mm(spec),
        "stiffness": stiffness(spec.handle, spec.plate_mm, spec.thickness_mm, mass),
        "board_mount": {"xyz_init": list(board_xyz), "rpy_init": list(board_rpy)},
        "grip": {
            "contact_rad": float(contact),
            "squeeze_rad": spec.handle.squeeze_rad,
            "grip_rad": float(contact - spec.handle.squeeze_rad),
            "clearance_clamp_mm": spec.handle.clamp_clearance_mm,
            "modelled_contact_rad": float(modelled),
        },
    }


def write_board_metadata(meta: dict[str, Any], stem: Path) -> Path:
    """Write the board metadata as <stem>.json (sorted keys, indent 2, trailing newline).

    Args:
        meta (dict[str, Any]): The metadata (`board_metadata`).
        stem (Path): Output path without suffix.

    Returns:
        Path: The written file.
    """
    path = stem.parent / f"{stem.name}.json"
    path.write_text(json.dumps(meta, sort_keys=True, indent=JSON_INDENT) + "\n")
    return path


def print_bodies(spec: CharucoSpec, opts: StlOptions) -> tuple[trimesh.Trimesh, trimesh.Trimesh]:
    """The print bodies of a board: mask bodies built face up (back on the bed, the inlays in the top layers), turned
    face down for pattern_face "bottom", the handle (tab, post and gusset) unioned into the base.

    Args:
        spec (CharucoSpec): The board.
        opts (StlOptions): 3D print options (min_module_mm and warn_module_mm are checked against the marker cell).

    Returns:
        tuple[trimesh.Trimesh, trimesh.Trimesh]: (base, modules) in print coordinates, bed at z = 0.

    Raises:
        ValueError: When the marker cell is below opts.min_module_mm.
    """
    mask, cell_mm = board_mask(spec)
    if spec.marker_cell_mm < opts.min_module_mm:
        raise ValueError(f"marker cell {spec.marker_cell_mm:.2f} mm is below min_module_mm {opts.min_module_mm}")
    mesh_opts = opts.model_copy(
        update={
            "base_thickness_mm": spec.thickness_mm,
            "relief_mm": spec.relief_mm,
            "inlay_mm": spec.inlay_mm,
            "min_module_mm": min(opts.min_module_mm, cell_mm),
            "warn_module_mm": min(opts.warn_module_mm, cell_mm),
        }
    )
    base, modules = mask_bodies(mask, cell_mm, spec.plate_mm, (spec.margin_mm, spec.margin_mm), mesh_opts)
    if spec.pattern_face == "top" and spec.handle is None:
        return base, modules
    solids = [to_manifold(base), to_manifold(modules)]
    if spec.pattern_face == "bottom":
        solids = [face_down(solid, spec.plate_mm, spec.thickness_mm) for solid in solids]
    if spec.handle is not None:
        solids[0] = solids[0] + handle_body(spec.handle, spec.plate_mm)
    return to_trimesh(solids[0]), to_trimesh(solids[1])


def write_charuco(spec: CharucoSpec, opts: StlOptions, stem: Path, gripper: Gripper | None) -> list[Path]:
    """Write the requested outputs for the board; nothing is written when any check fails.

    PDF: <stem>.pdf at 100 % scale. Meshes: <stem>-base.stl, <stem>-modules.stl, <stem>.3mf. A board with a handle
    also writes <stem>.json (`board_metadata`), whatever its outputs.

    Args:
        spec (CharucoSpec): The board definition.
        opts (StlOptions): 3D print options (min_module_mm and warn_module_mm are checked against the marker cell).
        stem (Path): Output path without suffix.
        gripper (Gripper | None): Loaded SO-101 gripper; needed when the spec has a handle.

    Returns:
        list[Path]: The written paths.

    Raises:
        ValueError: when the spec has a handle and no gripper is given, the marker cell is below opts.min_module_mm,
            the printed solid is not watertight or has overhang faces steeper than 45 deg, or the handle cannot be
            seated.
    """
    if spec.handle is not None and gripper is None:
        raise ValueError(f"board {spec.name} has a handle: the gripper is needed to seat it")
    wanted = [suffix for fmt in ("stl", "3mf") if fmt in spec.outputs for suffix in MESH_SUFFIX[fmt]]
    bodies = None
    meta = None
    if wanted or spec.handle is not None:
        bodies = print_bodies(spec, opts)
        violations = print_violations(printed_solid(*bodies))
        if violations:
            raise ValueError(f"board {spec.name} is not printable: {'; '.join(violations)}")
        if spec.handle is not None and gripper is not None:
            meta = board_metadata(spec, bodies[0], bodies[1], gripper)
    written: list[Path] = []
    stem.parent.mkdir(parents=True, exist_ok=True)
    if "pdf" in spec.outputs:
        mask, cell_mm = board_mask(spec)
        width_mm, height_mm = spec.squares_x * spec.square_mm, spec.squares_y * spec.square_mm
        label = (
            f"ChArUco {spec.squares_x}x{spec.squares_y} {spec.dictionary} square {spec.square_mm:g} mm "
            f"marker {spec.marker_mm:g} mm - Print at 100 %"
        )
        origin_x = (page_mm(spec.page)[0] - width_mm) / 2
        pdf_path = stem.with_suffix(".pdf")
        draw_mask_pdf(
            mask,
            cell_mm,
            spec.page,
            pdf_path,
            (origin_x, PDF_TOP_MM),
            label,
            label_y_mm=PDF_TOP_MM + height_mm + LABEL_BELOW_BOARD_MM,
        )
        written.append(pdf_path)
    if wanted and bodies is not None:
        for path in write_bodies(bodies[0], bodies[1], stem):
            if any(path.name.endswith(suffix) for suffix in wanted):
                written.append(path)
            else:
                path.unlink()
    if meta is not None:
        written.append(write_board_metadata(meta, stem))
    return written
