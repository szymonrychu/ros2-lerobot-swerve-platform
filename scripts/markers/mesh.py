"""Two-colour print bodies (white base, black modules) from a boolean mask, written as STL and Bambu 3MF."""

import io
import logging
import zipfile
from pathlib import Path
from xml.etree import ElementTree

import numpy as np
import trimesh
from manifold3d import Manifold, OpType

from scripts.markers.config import StlOptions
from scripts.markers.symbol import Symbol

LOGGER = logging.getLogger(__name__)
SETTINGS_PATH = "Metadata/model_settings.config"
BASE_NAME = "base"
MODULES_NAME = "modules"
EXTRUDERS = {BASE_NAME: 1, MODULES_NAME: 2}
CUT_OVERSHOOT_MM = 1.0


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


def write_bodies(base: trimesh.Trimesh, modules: trimesh.Trimesh, stem: Path) -> list[Path]:
    """Write <stem>-base.stl, <stem>-modules.stl and <stem>.3mf (two objects, extruders 1 and 2).

    Args:
        base: Base body.
        modules: Module body in the same coordinates.
        stem: Output path without suffix.

    Returns:
        The three written paths in that order.
    """
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
