"""Two-colour print bodies (white base, black modules) from a boolean mask, written as STL and Bambu 3MF."""

import io
import logging
import math
import zipfile
from pathlib import Path
from xml.etree import ElementTree

import numpy as np
import trimesh
from manifold3d import Manifold, Mesh, OpType

from scripts.markers.models import StlOptions
from scripts.markers.symbol import Symbol

LOGGER = logging.getLogger(__name__)
SETTINGS_PATH = "Metadata/model_settings.config"
BASE_NAME = "base"
MODULES_NAME = "modules"
EXTRUDERS = {BASE_NAME: 1, MODULES_NAME: 2}
CUT_OVERSHOOT_MM = 1.0
# Half width of the square column joining two dark cells that touch only at a corner (see pinch_corners): far below
# the print resolution, far above the float32 resolution of STL/3MF coordinates (about 1.5e-5 mm at 250 mm). A power
# of two, so the column sides land exactly on x +- half (box() builds a cube and translates it: a non-dyadic width
# leaves 1-ulp slivers that collapse into coincident vertices in float32).
PINCH_BRIDGE_HALF_MM = 2.0**-7
OVERHANG_LIMIT_DEG = 45.0
BED_TOL_MM = 0.05
NORMAL_TOL = 1e-4  # manifold3d stores float32 vertices, so normals at board coordinates are off by ~1e-5


def box(x0: float, y0: float, z0: float, x1: float, y1: float, z1: float) -> Manifold:
    """Axis-aligned solid box between two corners, in mm."""
    return Manifold.cube((x1 - x0, y1 - y0, z1 - z0)).translate((x0, y0, z0))


def to_trimesh(solid: Manifold) -> trimesh.Trimesh:
    """Convert a manifold3d solid to a watertight trimesh."""
    mesh = solid.to_mesh()
    return trimesh.Trimesh(
        vertices=np.asarray(mesh.vert_properties, dtype=np.float64)[:, :3],
        faces=np.asarray(mesh.tri_verts, dtype=np.int64),
        process=False,
    )


def to_manifold(mesh: trimesh.Trimesh) -> Manifold:
    """Convert a watertight trimesh to a manifold3d solid (float32 vertices).

    Args:
        mesh: Watertight mesh.

    Returns:
        The manifold3d solid.
    """
    return Manifold(
        Mesh(
            vert_properties=np.asarray(mesh.vertices, dtype=np.float32),
            tri_verts=np.asarray(mesh.faces, dtype=np.uint32),
        )
    )


def printed_solid(base: trimesh.Trimesh, modules: trimesh.Trimesh) -> trimesh.Trimesh:
    """Union of the base and module bodies, which is what the printer builds.

    Args:
        base: Base body.
        modules: Module body in the same coordinates.

    Returns:
        The watertight union mesh.
    """
    return to_trimesh(to_manifold(base) + to_manifold(modules))


def print_violations(mesh: trimesh.Trimesh) -> list[str]:
    """List the reasons a mesh cannot be printed bed-down without supports.

    Args:
        mesh: The printed solid, bed at its lowest z.

    Returns:
        Empty when printable; "not watertight" and/or "N overhang faces (A mm2) below 45 deg" otherwise. Overhang
        faces have normal z below -(sin(OVERHANG_LIMIT_DEG) + NORMAL_TOL) and a centre above the bed by more than
        BED_TOL_MM.
    """
    violations: list[str] = []
    if not mesh.is_watertight:
        violations.append("not watertight")
    steep = mesh.face_normals[:, 2] < -(math.sin(math.radians(OVERHANG_LIMIT_DEG)) + NORMAL_TOL)
    raised = mesh.triangles_center[:, 2] > mesh.bounds[0][2] + BED_TOL_MM
    bad = steep & raised
    if bad.any():
        area = float(mesh.area_faces[bad].sum())
        violations.append(f"{int(bad.sum())} overhang faces ({area:.1f} mm2) below {OVERHANG_LIMIT_DEG:g} deg")
    return violations


def dark_runs(mask: np.ndarray) -> list[tuple[int, int, int]]:
    """Return (row, first_col, end_col_exclusive) for every horizontal run of dark cells.

    Args:
        mask: 2D bool array, True = dark.

    Returns:
        List of runs, top row first.
    """
    runs: list[tuple[int, int, int]] = []
    for row, line in enumerate(mask):
        padded = np.concatenate([[False], line, [False]]).astype(np.int8)
        edges = np.flatnonzero(np.diff(padded))
        runs.extend((row, int(start), int(end)) for start, end in zip(edges[::2], edges[1::2], strict=True))
    return runs


def mask_bodies(
    mask: np.ndarray,
    cell_mm: float,
    plate_mm: tuple[float, float],
    origin_mm: tuple[float, float],
    opts: StlOptions,
) -> tuple[trimesh.Trimesh, trimesh.Trimesh]:
    """Build the base plate and the module body for a dark-cell mask.

    The plate spans (0, 0) to plate_mm in x, y and z from 0 to base_thickness_mm. mask row 0 is the top (highest y)
    row, as in an image viewed from above; origin_mm is the plate position of the mask's lower-left corner.

    Two dark cells that touch only at a corner (`pinch_corners`) are joined by a PINCH_BRIDGE_HALF_MM square column
    in the modules, cut from the base the same way, so neither body pinches along that edge: the written STL and 3MF
    bodies stay watertight after a reader merges coincident vertices.

    Args:
        mask: 2D bool array, True = dark cell (printed as a module).
        cell_mm: Cell edge length in mm.
        plate_mm: Plate (width, height) in mm.
        origin_mm: (x, y) of the mask's lower-left corner in the plate frame.
        opts: 3D print options.

    Returns:
        (base, modules) watertight meshes whose intersection volume is zero.

    Raises:
        ValueError: when the cell is below min_module_mm (naming the size), relief + inlay is not positive, or the
            inlay is not thinner than the base.
    """
    if cell_mm < opts.min_module_mm:
        raise ValueError(f"module size {cell_mm:.2f} mm is below the printable minimum {opts.min_module_mm} mm")
    if opts.relief_mm + opts.inlay_mm <= 0:
        raise ValueError("relief_mm + inlay_mm must be positive, otherwise the modules have no height")
    if opts.inlay_mm >= opts.base_thickness_mm:
        raise ValueError(f"inlay_mm {opts.inlay_mm} must be below base_thickness_mm {opts.base_thickness_mm}")
    if cell_mm < opts.warn_module_mm:
        LOGGER.warning("module size %.2f mm is below %s mm and may print poorly", cell_mm, opts.warn_module_mm)

    top = opts.base_thickness_mm
    z_low = top - opts.inlay_mm
    rows = mask.shape[0]
    ox, oy = origin_mm

    def footprint(z0: float, z1: float) -> Manifold:
        boxes = [
            box(ox + c0 * cell_mm, oy + (rows - 1 - r) * cell_mm, z0, ox + c1 * cell_mm, oy + (rows - r) * cell_mm, z1)
            for r, c0, c1 in dark_runs(mask)
        ]
        bridge = PINCH_BRIDGE_HALF_MM
        for r, c in pinch_corners(mask):
            x, y = ox + c * cell_mm, oy + (rows - r) * cell_mm
            boxes.append(box(x - bridge, y - bridge, z0, x + bridge, y + bridge, z1))
        return Manifold.batch_boolean(boxes, OpType.Add)

    modules = footprint(z_low, top + opts.relief_mm)
    base = box(0.0, 0.0, 0.0, plate_mm[0], plate_mm[1], top)
    if opts.inlay_mm > 0:
        base = base - footprint(z_low, top + CUT_OVERSHOOT_MM)
    return to_trimesh(base), to_trimesh(modules)


def marker_bodies(symbol: Symbol, opts: StlOptions) -> tuple[trimesh.Trimesh, trimesh.Trimesh]:
    """Build the base and module bodies for a QR symbol, centred on a plate with quiet zone and margin.

    Args:
        symbol: The QR symbol.
        opts: 3D print options.

    Returns:
        (base, modules) meshes; plate side = size_mm + 2 * quiet zone + 2 * margin_mm.
    """
    quiet_mm = symbol.quiet_zone_modules * symbol.module_mm
    side = symbol.size_mm + 2 * quiet_mm + 2 * opts.margin_mm
    offset = opts.margin_mm + quiet_mm
    return mask_bodies(symbol.matrix, symbol.module_mm, (side, side), (offset, offset), opts)


def settings_xml(ids: dict[str, int]) -> str:
    """Return the minimal Bambu Studio model_settings.config assigning an extruder to each named object.

    Args:
        ids: Object name to 3MF object id.

    Returns:
        The XML text.
    """
    root = ElementTree.Element("config")
    for name, object_id in ids.items():
        obj = ElementTree.SubElement(root, "object", id=str(object_id))
        ElementTree.SubElement(obj, "metadata", key="name", value=name)
        ElementTree.SubElement(obj, "metadata", key="extruder", value=str(EXTRUDERS[name]))
    return ElementTree.tostring(root, encoding="unicode", xml_declaration=False)


def pinch_corners(mask: np.ndarray) -> list[tuple[int, int]]:
    """Grid corners where two dark cells touch only along the corner edge (a diagonal pair in a 2x2 block).

    Both bodies pinch there: the modules of the two cells and the base around them share that edge only. The bodies are
    still manifold in memory (the coincident vertices are separate), but STL and 3MF readers merge coincident
    vertices, which leaves the edge in four faces and the written body not watertight.

    Args:
        mask (np.ndarray): 2D bool array, True = dark cell; row 0 is the top row.

    Returns:
        list[tuple[int, int]]: (row, col) of each pinch corner, where corner (r, c) is the top-left corner of cell
            (r, c), in row-major order.
    """
    padded = np.pad(np.asarray(mask, dtype=bool), 1, constant_values=False)
    top_left, top_right = padded[:-1, :-1], padded[:-1, 1:]
    bottom_left, bottom_right = padded[1:, :-1], padded[1:, 1:]
    diagonal = (top_left == bottom_right) & (top_right == bottom_left) & (top_left != top_right)
    return [(int(r), int(c)) for r, c in np.argwhere(diagonal)]


def weld(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """The mesh as STL and 3MF readers see it: float32 vertices, coincident ones merged, collapsed faces dropped.

    manifold3d booleans can leave two vertices that coincide in float32 joined by zero-area faces: closed by index in
    memory, but a reader merging coincident vertices gets degenerate faces and 3-face edges. Welding before the export
    writes the closed mesh a reader rebuilds. Only faces with a repeated vertex are dropped (zero area, so the volume
    and the bounds stay).

    Args:
        mesh (trimesh.Trimesh): A body, closed by index.

    Returns:
        trimesh.Trimesh: The welded body (float64 vertices holding float32 values).
    """
    vertices = np.asarray(mesh.vertices, dtype=np.float32)
    unique, inverse = np.unique(vertices, axis=0, return_inverse=True)
    faces = inverse.reshape(-1)[np.asarray(mesh.faces)]
    kept = (faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])
    welded = trimesh.Trimesh(vertices=unique.astype(np.float64), faces=faces[kept], process=False)
    welded.remove_unreferenced_vertices()
    return welded


def write_bodies(base: trimesh.Trimesh, modules: trimesh.Trimesh, stem: Path) -> list[Path]:
    """Write <stem>-base.stl, <stem>-modules.stl and <stem>.3mf (two objects, extruders 1 and 2).

    Both bodies are welded first (`weld`), so each written body reads back closed.

    Args:
        base: Base body.
        modules: Module body in the same coordinates.
        stem: Output path without suffix.

    Returns:
        The three written paths in that order.
    """
    base, modules = weld(base), weld(modules)
    stem.parent.mkdir(parents=True, exist_ok=True)
    base_path = stem.parent / f"{stem.name}-base.stl"
    modules_path = stem.parent / f"{stem.name}-modules.stl"
    threemf_path = stem.parent / f"{stem.name}.3mf"
    base.export(base_path)
    modules.export(modules_path)

    scene = trimesh.Scene()
    scene.add_geometry(base, node_name=BASE_NAME, geom_name=BASE_NAME)
    scene.add_geometry(modules, node_name=MODULES_NAME, geom_name=MODULES_NAME)
    raw = scene.export(file_type="3mf")
    with zipfile.ZipFile(io.BytesIO(raw)) as src:
        model = ElementTree.fromstring(src.read("3D/3dmodel.model"))
        ids = {
            obj.get("name"): int(obj.get("id"))
            for obj in model.iter("{http://schemas.microsoft.com/3dmanufacturing/core/2015/02}object")
        }
        with zipfile.ZipFile(threemf_path, "w", zipfile.ZIP_DEFLATED) as dst:
            for item in src.infolist():
                dst.writestr(item, src.read(item.filename))
            dst.writestr(SETTINGS_PATH, settings_xml(ids))
    return [base_path, modules_path, threemf_path]


def read_bodies(stem: Path) -> dict[str, tuple[trimesh.Trimesh, trimesh.Trimesh]]:
    """Read back the bodies `write_bodies` wrote, as a slicer reads them (coincident vertices merged).

    Args:
        stem (Path): Output path without suffix, as given to `write_bodies`.

    Returns:
        dict[str, tuple[trimesh.Trimesh, trimesh.Trimesh]]: "stl": (base, modules) from <stem>-base.stl and
            <stem>-modules.stl; "3mf": (base, modules) objects of <stem>.3mf.
    """
    scene = trimesh.load(stem.parent / f"{stem.name}.3mf", file_type="3mf")
    return {
        "stl": (
            trimesh.load_mesh(stem.parent / f"{stem.name}-base.stl"),
            trimesh.load_mesh(stem.parent / f"{stem.name}-modules.stl"),
        ),
        "3mf": (scene.geometry[BASE_NAME], scene.geometry[MODULES_NAME]),
    }
