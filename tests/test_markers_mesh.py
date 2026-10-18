"""Tests for scripts/markers/mesh.py: watertight two-body meshes, footprint, heights, plate size, 3MF structure."""

import logging
import zipfile
from pathlib import Path
from xml.etree import ElementTree

import numpy as np
import pytest
import trimesh
import zxingcpp
from manifold3d import Manifold, Mesh

from scripts.markers.config import MarkerSpec, StlOptions, SymbolDefaults
from scripts.markers.mesh import marker_bodies, mask_bodies, write_bodies
from scripts.markers.symbol import Symbol, build_symbol, render_raster

TOL = 1e-6


def make_symbol(size: float = 150.0, quiet: int = 4, name: str = "DOCK-1") -> Symbol:
    """Build a CHARGER symbol of the given edge size."""
    spec = MarkerSpec(kind_code="CHARGER", name=name, size_mm=size)
    return build_symbol(spec, SymbolDefaults(quiet_zone_modules=quiet))


def to_manifold(mesh: trimesh.Trimesh) -> Manifold:
    """Convert a trimesh to a manifold3d solid."""
    return Manifold(
        Mesh(
            vert_properties=np.asarray(mesh.vertices, dtype=np.float32),
            tri_verts=np.asarray(mesh.faces, dtype=np.uint32),
        )
    )


def cell_centres(symbol: Symbol, opts: StlOptions) -> tuple[np.ndarray, float]:
    """Return (rows, cols, 2) xy centres of the matrix cells in the plate frame, and the plate side."""
    cell = symbol.module_mm
    plate = symbol.size_mm + 2 * symbol.quiet_zone_modules * cell + 2 * opts.margin_mm
    origin = opts.margin_mm + symbol.quiet_zone_modules * cell
    n = symbol.modules
    xs = origin + (np.arange(n) + 0.5) * cell
    ys = origin + (n - 1 - np.arange(n) + 0.5) * cell
    grid = np.stack(np.meshgrid(xs, ys), axis=-1)
    return grid, plate


def sample_footprint(modules: trimesh.Trimesh, symbol: Symbol, opts: StlOptions, z: float) -> np.ndarray:
    """Sample the module body containment at every cell centre at height z."""
    grid, _ = cell_centres(symbol, opts)
    pts = np.concatenate([grid.reshape(-1, 2), np.full((grid.shape[0] * grid.shape[1], 1), z)], axis=1)
    return modules.contains(pts).reshape(grid.shape[:2])


@pytest.fixture(scope="module")
def v2_bodies() -> tuple[Symbol, StlOptions, trimesh.Trimesh, trimesh.Trimesh]:
    """A 50 mm v2 marker with default options."""
    symbol = make_symbol(50.0)
    opts = StlOptions()
    base, modules = marker_bodies(symbol, opts)
    return symbol, opts, base, modules


def test_bodies_watertight_and_disjoint(v2_bodies) -> None:
    _, _, base, modules = v2_bodies
    assert base.is_watertight and modules.is_watertight
    overlap = to_manifold(base) ^ to_manifold(modules)
    assert overlap.volume() < TOL


def test_bodies_disjoint_with_inlay() -> None:
    symbol = make_symbol(50.0)
    opts = StlOptions(inlay_mm=0.6, relief_mm=0.0)
    base, modules = marker_bodies(symbol, opts)
    assert base.is_watertight and modules.is_watertight
    assert (to_manifold(base) ^ to_manifold(modules)).volume() < TOL


def test_module_footprint_matches_matrix(v2_bodies) -> None:
    symbol, opts, _, modules = v2_bodies
    z = opts.base_thickness_mm + opts.relief_mm / 2
    assert np.array_equal(sample_footprint(modules, symbol, opts, z), symbol.matrix)


def test_no_module_material_in_quiet_zone(v2_bodies) -> None:
    symbol, opts, _, modules = v2_bodies
    cell = symbol.module_mm
    lo = opts.margin_mm + symbol.quiet_zone_modules * cell
    hi = lo + symbol.size_mm
    assert modules.bounds[0][0] >= lo - TOL and modules.bounds[0][1] >= lo - TOL
    assert modules.bounds[1][0] <= hi + TOL and modules.bounds[1][1] <= hi + TOL


def test_heights(v2_bodies) -> None:
    _, opts, base, modules = v2_bodies
    assert base.bounds[0][2] == pytest.approx(0.0)
    assert base.bounds[1][2] == pytest.approx(opts.base_thickness_mm)
    assert modules.bounds[0][2] == pytest.approx(opts.base_thickness_mm)
    assert modules.bounds[1][2] == pytest.approx(opts.base_thickness_mm + opts.relief_mm)


def test_flush_inlay_heights_and_pockets() -> None:
    symbol = make_symbol(50.0)
    opts = StlOptions(inlay_mm=0.6, relief_mm=0.0)
    base, modules = marker_bodies(symbol, opts)
    assert base.bounds[1][2] == pytest.approx(opts.base_thickness_mm)
    assert modules.bounds[1][2] == pytest.approx(opts.base_thickness_mm)
    assert modules.bounds[0][2] == pytest.approx(opts.base_thickness_mm - 0.6)
    plain_base, _ = marker_bodies(symbol, StlOptions(inlay_mm=0.0))
    assert base.volume < plain_base.volume - 1.0
    assert np.array_equal(sample_footprint(modules, symbol, opts, opts.base_thickness_mm - 0.3), symbol.matrix)


def test_plate_size_150mm_v2() -> None:
    symbol = make_symbol(150.0, quiet=4)
    base, _ = marker_bodies(symbol, StlOptions(margin_mm=3.0))
    cell = 150.0 / symbol.modules
    expected = 150.0 + 2 * 4 * cell + 2 * 3.0
    assert expected == pytest.approx(204.0, abs=0.01)
    extents = base.bounds[1] - base.bounds[0]
    assert extents[0] == pytest.approx(204.0, abs=0.01)
    assert extents[1] == pytest.approx(204.0, abs=0.01)


def test_module_too_small_raises_naming_size() -> None:
    symbol = make_symbol(30.0, name="DOCK-NUMBER-ONE-ABC")
    assert symbol.version == 3
    with pytest.raises(ValueError, match=r"1\.03"):
        marker_bodies(symbol, StlOptions())


def test_module_size_warning(caplog: pytest.LogCaptureFixture) -> None:
    with caplog.at_level(logging.WARNING):
        marker_bodies(make_symbol(50.0), StlOptions())
    assert not caplog.records
    with caplog.at_level(logging.WARNING):
        marker_bodies(make_symbol(45.0), StlOptions())
    assert any("1.8" in r.getMessage() for r in caplog.records)


def test_invalid_heights_raise() -> None:
    symbol = make_symbol(50.0)
    with pytest.raises(ValueError, match="relief"):
        marker_bodies(symbol, StlOptions(relief_mm=0.0, inlay_mm=0.0))
    with pytest.raises(ValueError, match="inlay"):
        marker_bodies(symbol, StlOptions(inlay_mm=2.0, base_thickness_mm=2.0))


def test_write_bodies_3mf_structure(tmp_path: Path, v2_bodies) -> None:
    _, _, base, modules = v2_bodies
    paths = write_bodies(base, modules, tmp_path / "marker")
    assert [p.name for p in paths] == ["marker-base.stl", "marker-modules.stl", "marker.3mf"]
    assert all(p.exists() for p in paths)
    scene = trimesh.load(paths[2], file_type="3mf")
    assert sorted(scene.geometry) == ["base", "modules"]
    stl_base = trimesh.load(paths[0])
    stl_modules = trimesh.load(paths[1])
    assert np.allclose(stl_base.bounds, base.bounds, atol=1e-4)
    assert np.allclose(stl_modules.bounds, modules.bounds, atol=1e-4)
    assert np.allclose(scene.geometry["modules"].bounds, modules.bounds, atol=1e-4)
    with zipfile.ZipFile(paths[2]) as zf:
        config = zf.read("Metadata/model_settings.config").decode()
    root = ElementTree.fromstring(config)
    found = {}
    for obj in root.iter("object"):
        meta = {m.get("key"): m.get("value") for m in obj.iter("metadata")}
        found[meta["name"]] = meta["extruder"]
    assert found == {"base": "1", "modules": "2"}


def test_render_then_decode(v2_bodies) -> None:
    symbol, opts, _, modules = v2_bodies
    z = opts.base_thickness_mm + opts.relief_mm / 2
    mask = sample_footprint(modules, symbol, opts, z)
    padded = np.pad(mask, symbol.quiet_zone_modules, constant_values=False)
    raster = np.where(np.kron(padded, np.ones((8, 8), dtype=bool)), 0, 255).astype(np.uint8)
    assert np.array_equal(raster, render_raster(symbol, 8))
    results = zxingcpp.read_barcodes(raster)
    assert [r.text for r in results] == [symbol.payload_text]


def test_mask_bodies_unions_adjacent_cells_and_orients_rows() -> None:
    mask = np.array([[True, True, False], [False, False, False], [False, False, True]])
    opts = StlOptions()
    base, modules = mask_bodies(mask, 2.0, (10.0, 10.0), (2.0, 2.0), opts)
    assert modules.volume == pytest.approx(3 * 4.0 * opts.relief_mm)
    assert modules.bounds[0][:2] == pytest.approx([2.0, 2.0])
    assert modules.bounds[1][:2] == pytest.approx([8.0, 8.0])
    assert modules.contains([[3.0, 7.0, 2.3], [7.0 - 0.5, 3.0, 2.3], [3.0, 3.0, 2.3]]).tolist() == [True, True, False]
    assert base.volume == pytest.approx(10 * 10 * opts.base_thickness_mm)
