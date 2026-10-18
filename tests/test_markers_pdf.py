"""Tests for scripts/markers/pdf.py: exact-size marker PDF measured on a 600 dpi raster of the page."""

from pathlib import Path

import numpy as np
import pypdfium2 as pdfium
import pytest
import zxingcpp

from scripts.markers.config import MarkerSpec, PageSpec, SymbolDefaults
from scripts.markers.pdf import MM_TO_PT, PdfLayout, draw_mask_pdf, page_size_pt, write_marker_pdf
from scripts.markers.symbol import Symbol, build_symbol

DPI = 600
PX_PER_MM = DPI / 25.4
DARK_BELOW = 100
LABEL = "CHARGER DOCK-1"


def make_symbol(size: float = 100.0, name: str = "DOCK-1") -> Symbol:
    """Build a symbol for a CHARGER marker."""
    spec = MarkerSpec(kind_code="CHARGER", name=name, size_mm=size)
    return build_symbol(spec, SymbolDefaults())


def rasterise(path: Path) -> np.ndarray:
    """Render page 0 at 600 dpi to a uint8 grey array."""
    pdf = pdfium.PdfDocument(str(path))
    bitmap = pdf[0].render(scale=DPI / 72, grayscale=True)
    return np.array(bitmap.to_numpy(), dtype=np.uint8).reshape(bitmap.height, bitmap.width)


def px(mm: float) -> int:
    """Millimetres to raster pixels."""
    return round(mm * PX_PER_MM)


def dark_extent_mm(crop: np.ndarray) -> tuple[float, float]:
    """Return the (width, height) in mm of the dark bounding box of a crop."""
    ys, xs = np.nonzero(crop < DARK_BELOW)
    return (xs.max() - xs.min() + 1) / PX_PER_MM, (ys.max() - ys.min() + 1) / PX_PER_MM


def symbol_crop(raster: np.ndarray, layout: PdfLayout, quiet_mm: float) -> np.ndarray:
    """Crop the symbol plus its quiet zone (cut square and label excluded)."""
    x0, y0 = layout.origin_mm
    return raster[
        px(y0 - quiet_mm) : px(y0 + layout.edge_mm + quiet_mm), px(x0 - quiet_mm) : px(x0 + layout.edge_mm + quiet_mm)
    ]


@pytest.mark.parametrize(
    ("page", "expected"),
    [
        (PageSpec(), (595.28, 841.89)),
        (PageSpec(size="A3"), (841.89, 1190.55)),
        (PageSpec(orientation="landscape"), (841.89, 595.28)),
        (PageSpec(width_mm=100, height_mm=200), (100 * MM_TO_PT, 200 * MM_TO_PT)),
        (PageSpec(size="LETTER"), (612.0, 792.0)),
    ],
)
def test_page_size_pt(page: PageSpec, expected: tuple[float, float]) -> None:
    width, height = page_size_pt(page)
    assert width == pytest.approx(expected[0], abs=0.02)
    assert height == pytest.approx(expected[1], abs=0.02)


def test_pdf_mediabox_equals_media(tmp_path: Path) -> None:
    path = tmp_path / "m.pdf"
    write_marker_pdf(make_symbol(), PageSpec(), path, LABEL)
    pdf = pdfium.PdfDocument(str(path))
    assert len(pdf) == 1
    assert pdf[0].get_mediabox() == pytest.approx((0, 0, 595.28, 841.89), abs=0.02)


@pytest.mark.parametrize("size", [60.0, 100.0, 150.0])
def test_symbol_edge_measured_on_raster(tmp_path: Path, size: float) -> None:
    symbol = make_symbol(size)
    path = tmp_path / "m.pdf"
    layout = write_marker_pdf(symbol, PageSpec(), path, LABEL)
    assert layout.edge_mm == pytest.approx(size)
    width, height = dark_extent_mm(symbol_crop(rasterise(path), layout, quiet_mm=0.5))
    assert width == pytest.approx(size, abs=0.1)
    assert height == pytest.approx(size, abs=0.1)


def test_quiet_zone_is_white(tmp_path: Path) -> None:
    symbol = make_symbol(100.0)
    path = tmp_path / "m.pdf"
    layout = write_marker_pdf(symbol, PageSpec(), path, LABEL)
    raster = rasterise(path)
    quiet_mm = symbol.quiet_zone_modules * symbol.module_mm
    x0, y0 = layout.origin_mm
    window = raster[
        px(y0 - quiet_mm) : px(y0 + layout.edge_mm + quiet_mm), px(x0 - quiet_mm) : px(x0 + layout.edge_mm + quiet_mm)
    ]
    margin = px(quiet_mm) - 1
    inner = window[margin : window.shape[0] - margin, margin : window.shape[1] - margin]
    ring = window.copy()
    ring[margin : window.shape[0] - margin, margin : window.shape[1] - margin] = 255
    assert ring.min() > 250
    assert inner.min() < DARK_BELOW


def test_cut_square_is_grey_outside_quiet_zone(tmp_path: Path) -> None:
    symbol = make_symbol(100.0)
    path = tmp_path / "m.pdf"
    layout = write_marker_pdf(symbol, PageSpec(), path, LABEL)
    raster = rasterise(path)
    quiet_mm = symbol.quiet_zone_modules * symbol.module_mm
    x0, y0 = layout.origin_mm
    row = raster[px(y0 + layout.edge_mm / 2), px(x0 - quiet_mm - 2) : px(x0 - quiet_mm)]
    assert row.min() < 250
    assert row.min() >= DARK_BELOW


def test_ruler_measures_100_mm(tmp_path: Path) -> None:
    path = tmp_path / "m.pdf"
    layout = write_marker_pdf(make_symbol(), PageSpec(), path, LABEL)
    raster = rasterise(path)
    ruler_x, ruler_y = layout.ruler_origin_mm
    band = raster[px(ruler_y) : px(ruler_y + 1), :]
    xs = np.nonzero(band.min(axis=0) < DARK_BELOW)[0]
    assert (xs.max() - xs.min() + 1) / PX_PER_MM == pytest.approx(100.0, abs=0.1)
    assert xs.min() / PX_PER_MM == pytest.approx(ruler_x, abs=0.1)


def test_page_text_has_payload_and_scale_note(tmp_path: Path) -> None:
    symbol = make_symbol()
    path = tmp_path / "m.pdf"
    write_marker_pdf(symbol, PageSpec(), path, LABEL)
    text = pdfium.PdfDocument(str(path))[0].get_textpage().get_text_range()
    assert symbol.payload_text in text
    assert "Print at 100 %" in text
    assert "CHARGER DOCK-1" in text
    assert "100 mm" in text


@pytest.mark.parametrize("size", [60.0, 100.0, 150.0])
def test_raster_decodes_to_payload(tmp_path: Path, size: float) -> None:
    symbol = make_symbol(size)
    path = tmp_path / "m.pdf"
    write_marker_pdf(symbol, PageSpec(), path, LABEL)
    results = zxingcpp.read_barcodes(rasterise(path))
    assert [r.text for r in results] == [symbol.payload_text]


def test_oversize_marker_raises_does_not_fit(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match=r"does not fit.*needs.*mm.*available.*mm"):
        write_marker_pdf(make_symbol(200.0), PageSpec(), tmp_path / "m.pdf", LABEL)


def test_draw_mask_pdf_draws_cells_at_origin(tmp_path: Path) -> None:
    mask = np.zeros((4, 6), dtype=bool)
    mask[0, :] = True
    mask[2, 1:3] = True
    path = tmp_path / "mask.pdf"
    draw_mask_pdf(mask, 10.0, PageSpec(), path, (20.0, 30.0), "MASK")
    raster = rasterise(path)
    crop = raster[px(29) : px(30 + 40 + 1), px(19) : px(20 + 60 + 1)]
    ys, xs = np.nonzero(crop < DARK_BELOW)
    assert (xs.max() - xs.min() + 1) / PX_PER_MM == pytest.approx(60.0, abs=0.1)
    assert raster[px(30 + 25), px(20 + 15)] < DARK_BELOW
    assert raster[px(30 + 15), px(20 + 15)] > 250
    assert "MASK" in pdfium.PdfDocument(str(path))[0].get_textpage().get_text_range()
