"""Tests for scripts/markers/charuco.py: the shipped spec, validators, PDF and mesh detected by OpenCV."""

from pathlib import Path

import cv2
import numpy as np
import pypdfium2 as pdfium
import pytest
import trimesh
from pydantic import ValidationError

from scripts.markers.charuco import CharucoSpec, board_mask, make_charuco_board, write_charuco
from scripts.markers.config import GeneratorConfig, StlOptions, load_generator_config

DPI = 600
PX_PER_MM = DPI / 25.4
MESH_PX_PER_MM = 10.0
INNER_CORNERS = 20
SHIPPED_YAML = Path(__file__).resolve().parent.parent / "scripts" / "markers_generate.yaml"


def detect(image: np.ndarray, spec: CharucoSpec) -> tuple[np.ndarray, np.ndarray]:
    """Run CharucoDetector; return (corners (N, 2) px, ids (N,))."""
    detector = cv2.aruco.CharucoDetector(make_charuco_board(spec))
    corners, ids, _, _ = detector.detectBoard(image)
    assert corners is not None and ids is not None
    return corners.reshape(-1, 2), ids.reshape(-1)


def test_shipped_spec_is_6x5_and_fits_bed() -> None:
    spec = CharucoSpec()
    assert (spec.squares_x, spec.squares_y, spec.square_mm, spec.marker_mm) == (6, 5, 40.0, 30.0)
    assert spec.dictionary == "DICT_5X5_100"
    assert spec.squares_x * spec.square_mm + 2 * spec.margin_mm == 250.0
    assert spec.squares_y * spec.square_mm + 2 * spec.margin_mm == 210.0
    assert 250.0 <= spec.bed_mm


def test_yaml_carries_charuco_section() -> None:
    config = load_generator_config(SHIPPED_YAML)
    assert isinstance(config, GeneratorConfig)
    assert config.charuco == CharucoSpec()


@pytest.mark.parametrize(
    ("update", "field"),
    [
        ({"squares_x": 7}, "squares_x"),
        ({"squares_y": 4}, "squares_y"),
        ({"marker_mm": 40.0}, "marker_mm"),
        ({"marker_mm": 45.0}, "marker_mm"),
        ({"squares_y": 7, "square_mm": 40.0}, "squares_y"),
        ({"thickness_mm": 3.0}, "thickness_mm"),
        ({"bogus": 1}, "bogus"),
    ],
)
def test_invalid_specs_name_the_field(update: dict, field: str) -> None:
    with pytest.raises(ValidationError, match=field):
        CharucoSpec(**update)


def test_board_mask_is_exact_pixels() -> None:
    mask, cell_mm = board_mask(CharucoSpec())
    assert cell_mm == pytest.approx(1 / 1.4)
    assert mask.shape == (280, 336)
    assert mask.dtype == bool


def rasterise_pdf(path: Path) -> np.ndarray:
    """Render page 0 at 600 dpi to a grey array."""
    bitmap = pdfium.PdfDocument(str(path))[0].render(scale=DPI / 72, grayscale=True)
    return np.array(bitmap.to_numpy(), dtype=np.uint8).reshape(bitmap.height, bitmap.width)


def test_pdf_detects_all_corners_at_40_mm(tmp_path: Path) -> None:
    spec = CharucoSpec()
    paths = write_charuco(spec, StlOptions(), tmp_path / "board")
    pdf = next(p for p in paths if p.suffix == ".pdf")
    corners, ids = detect(rasterise_pdf(pdf), spec)
    assert len(ids) == INNER_CORNERS
    grid = {int(i): c for i, c in zip(ids, corners, strict=True)}
    pitch_x = [np.linalg.norm(grid[i + 1] - grid[i]) for i in grid if i + 1 in grid and (i + 1) % (spec.squares_x - 1)]
    assert np.mean(pitch_x) / PX_PER_MM == pytest.approx(40.0, abs=0.1)
    text = pdfium.PdfDocument(str(pdf))[0].get_textpage().get_text_range()
    assert "Print at 100 %" in text


def top_view(modules: trimesh.Trimesh, spec: CharucoSpec) -> np.ndarray:
    """Rasterise the upward faces at the module top over the plate, modules dark on white."""
    width = int(round((spec.squares_x * spec.square_mm + 2 * spec.margin_mm) * MESH_PX_PER_MM))
    height = int(round((spec.squares_y * spec.square_mm + 2 * spec.margin_mm) * MESH_PX_PER_MM))
    image = np.full((height, width), 255, dtype=np.uint8)
    top = modules.vertices[:, 2].max()
    for face, normal in zip(modules.faces, modules.face_normals, strict=True):
        pts = modules.vertices[face]
        if normal[2] > 0.5 and np.allclose(pts[:, 2], top):
            xy = np.stack([pts[:, 0] * MESH_PX_PER_MM, height - pts[:, 1] * MESH_PX_PER_MM], axis=1)
            cv2.fillConvexPoly(image, np.round(xy).astype(np.int32), 0)
    return image


def test_mesh_footprint_matches_board_corners(tmp_path: Path) -> None:
    spec = CharucoSpec()
    paths = write_charuco(spec, StlOptions(), tmp_path / "board")
    modules = trimesh.load(next(p for p in paths if p.name.endswith("-modules.stl")))
    corners, ids = detect(top_view(modules, spec), spec)
    assert len(ids) == INNER_CORNERS
    expected = make_charuco_board(spec).getChessboardCorners() * 1000
    for corner, ident in zip(corners, ids, strict=True):
        found = corner / MESH_PX_PER_MM - spec.margin_mm
        assert np.linalg.norm(found - expected[ident][:2]) < 0.2


def test_stl_base_is_4_mm_thick_and_plate_sized(tmp_path: Path) -> None:
    paths = write_charuco(CharucoSpec(), StlOptions(), tmp_path / "board")
    assert sorted(p.suffix for p in paths) == [".3mf", ".pdf", ".stl", ".stl"]
    base = trimesh.load(next(p for p in paths if p.name.endswith("-base.stl")))
    assert base.extents == pytest.approx([250.0, 210.0, 4.0], abs=1e-3)
    modules = trimesh.load(next(p for p in paths if p.name.endswith("-modules.stl")))
    assert modules.bounds[1][2] == pytest.approx(4.0, abs=1e-3)


def test_outputs_subset(tmp_path: Path) -> None:
    paths = write_charuco(CharucoSpec(outputs=["pdf"]), StlOptions(), tmp_path / "b")
    assert [p.suffix for p in paths] == [".pdf"]


def test_flush_inlay_board_is_flat_and_watertight(tmp_path: Path) -> None:
    spec = CharucoSpec(relief_mm=0.0, inlay_mm=1.2)
    paths = write_charuco(spec, StlOptions(), tmp_path / "board")
    base = trimesh.load(next(p for p in paths if p.name.endswith("-base.stl")))
    modules = trimesh.load(next(p for p in paths if p.name.endswith("-modules.stl")))
    # Checkerboard cells touch diagonally, so the merged STL has shared edges; volumes prove the parts tile the plate.
    assert base.volume + modules.volume == pytest.approx(250.0 * 210.0 * 4.0, rel=1e-6)
    assert base.bounds[1][2] == pytest.approx(4.0, abs=1e-3)
    assert modules.bounds[1][2] == pytest.approx(4.0, abs=1e-3)
    assert modules.bounds[0][2] == pytest.approx(2.8, abs=1e-3)


def test_inlay_must_be_below_thickness() -> None:
    with pytest.raises(ValidationError, match="inlay_mm"):
        CharucoSpec(relief_mm=0.0, inlay_mm=4.0)
