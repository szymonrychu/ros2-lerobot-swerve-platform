"""Exact-size marker PDFs (reportlab): vector modules, quiet zone, cut square, label and a 100 mm ruler."""

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from reportlab.pdfgen.canvas import Canvas

from scripts.markers.models import PageSpec
from scripts.markers.symbol import Symbol

MM_TO_PT = 72 / 25.4
PAGE_SIZES_MM = {"A4": (210.0, 297.0), "A3": (297.0, 420.0), "LETTER": (215.9, 279.4)}
TOP_MARGIN_MM = 8.0
SIDE_MARGIN_MM = 5.0
BOTTOM_MARGIN_MM = 8.0
CUT_OFFSET_MM = 1.0
CUT_GREY = 0.6
CUT_WIDTH_PT = 0.3
LABEL_GAP_MM = 6.0
LINE_PITCH_MM = 5.0
FONT = "Helvetica"
FONT_BOLD = "Helvetica-Bold"
LABEL_PT = 11
TEXT_PT = 8
RULER_MM = 100.0
RULER_BAR_MM = 1.5
RULER_TICK_MM = 10.0
RULER_TICK_LEN_MM = 3.0
RULER_BLOCK_MM = 14.0
SCALE_NOTE = "Print at 100 % (actual size), no fit to page; mount on a rigid plate"


@dataclass(frozen=True)
class PdfLayout:
    """Where things landed, in mm from the page top-left.

    origin_mm: symbol top-left (finder outer edge, quiet zone excluded). edge_mm: symbol edge. ruler_origin_mm:
    left end of the ruler bar's top edge (x, y). page_mm: (width, height) of the media.
    """

    origin_mm: tuple[float, float]
    edge_mm: float
    ruler_origin_mm: tuple[float, float]
    page_mm: tuple[float, float]


def page_mm(page: PageSpec) -> tuple[float, float]:
    """Media size in mm.

    Args:
        page: Named or custom page; orientation swaps a named page.

    Returns:
        (width, height) in mm.
    """
    if page.width_mm is not None and page.height_mm is not None:
        return page.width_mm, page.height_mm
    width, height = PAGE_SIZES_MM[page.size]
    return (height, width) if page.orientation == "landscape" else (width, height)


def page_size_pt(page: PageSpec) -> tuple[float, float]:
    """Media size in PDF points.

    Args:
        page: The page.

    Returns:
        (width, height) in points.
    """
    width, height = page_mm(page)
    return width * MM_TO_PT, height * MM_TO_PT


def draw_runs(
    canvas: Canvas, mask: np.ndarray, cell_mm: float, origin_mm: tuple[float, float], height_pt: float
) -> None:
    """Fill the True cells of a mask as vector rectangles, merging horizontal runs.

    Args:
        canvas: Target canvas (black fill is set here).
        mask: 2D bool array, True = dark.
        cell_mm: Cell edge in mm.
        origin_mm: Top-left of the mask from the page top-left, mm.
        height_pt: Page height in points (PDF y axis points up).
    """
    canvas.setFillGray(0)
    cell_pt = cell_mm * MM_TO_PT
    x0_pt = origin_mm[0] * MM_TO_PT
    y0_pt = height_pt - origin_mm[1] * MM_TO_PT
    for row_index, row in enumerate(np.asarray(mask, dtype=bool)):
        padded = np.concatenate(([False], row, [False]))
        edges = np.flatnonzero(padded[1:] != padded[:-1])
        for start, stop in zip(edges[::2], edges[1::2], strict=True):
            canvas.rect(
                x0_pt + start * cell_pt,
                y0_pt - (row_index + 1) * cell_pt,
                (stop - start) * cell_pt,
                cell_pt,
                stroke=0,
                fill=1,
            )


def draw_text(canvas: Canvas, text: str, x_mm: float, y_mm: float, height_pt: float, font: str, size_pt: int) -> None:
    """Draw one text line with its baseline y_mm below the page top.

    Args:
        canvas: Target canvas.
        text: The line.
        x_mm: Left edge, mm.
        y_mm: Baseline from the page top, mm.
        height_pt: Page height in points.
        font: Font name.
        size_pt: Font size.
    """
    canvas.setFillGray(0)
    canvas.setFont(font, size_pt)
    canvas.drawString(x_mm * MM_TO_PT, height_pt - y_mm * MM_TO_PT, text)


def draw_ruler(canvas: Canvas, x_mm: float, y_mm: float, height_pt: float) -> None:
    """Draw the 100 mm ruler bar with 10 mm ticks and the scale note beneath it.

    Args:
        canvas: Target canvas.
        x_mm: Left end of the bar, mm.
        y_mm: Top edge of the bar from the page top, mm.
        height_pt: Page height in points.
    """
    canvas.setFillGray(0)
    canvas.rect(
        x_mm * MM_TO_PT,
        height_pt - (y_mm + RULER_BAR_MM) * MM_TO_PT,
        RULER_MM * MM_TO_PT,
        RULER_BAR_MM * MM_TO_PT,
        stroke=0,
        fill=1,
    )
    tick_w = 0.3 * MM_TO_PT
    for index in range(int(RULER_MM / RULER_TICK_MM) + 1):
        tick_x = x_mm + index * RULER_TICK_MM
        left = min(max(tick_x * MM_TO_PT - tick_w / 2, x_mm * MM_TO_PT), (x_mm + RULER_MM) * MM_TO_PT - tick_w)
        canvas.rect(
            left,
            height_pt - (y_mm + RULER_BAR_MM + RULER_TICK_LEN_MM) * MM_TO_PT,
            tick_w,
            RULER_TICK_LEN_MM * MM_TO_PT,
            stroke=0,
            fill=1,
        )
    draw_text(canvas, "0", x_mm, y_mm + RULER_BAR_MM + RULER_TICK_LEN_MM + 3, height_pt, FONT, TEXT_PT)
    draw_text(
        canvas, "100 mm", x_mm + RULER_MM - 10, y_mm + RULER_BAR_MM + RULER_TICK_LEN_MM + 3, height_pt, FONT, TEXT_PT
    )
    draw_text(canvas, SCALE_NOTE, x_mm, y_mm + RULER_BAR_MM + RULER_TICK_LEN_MM + 8, height_pt, FONT_BOLD, TEXT_PT)


def write_marker_pdf(symbol: Symbol, page: PageSpec, path: Path, label: str) -> PdfLayout:
    """Write a one-page PDF whose MediaBox is the media and whose symbol is exactly symbol.size_mm wide.

    Args:
        symbol: The QR symbol.
        page: The media.
        path: Output file.
        label: Heading line (kind and name); size and payload lines are added.

    Returns:
        The PdfLayout in mm from the page top-left.

    Raises:
        ValueError: naming the needed and available size when symbol, quiet zone, label and ruler do not fit.
    """
    width_mm, height_mm = page_mm(page)
    quiet_mm = symbol.quiet_zone_modules * symbol.module_mm
    block_mm = symbol.size_mm + 2 * (quiet_mm + CUT_OFFSET_MM)
    label_lines = [*label.splitlines(), f"Size {symbol.size_mm:g} mm (symbol edge, quiet zone excluded)"]
    label_lines.append(symbol.payload_text)
    needed_w = max(block_mm, RULER_MM) + 2 * SIDE_MARGIN_MM
    needed_h = (
        TOP_MARGIN_MM + block_mm + LABEL_GAP_MM + LINE_PITCH_MM * len(label_lines) + RULER_BLOCK_MM + BOTTOM_MARGIN_MM
    )
    if needed_w > width_mm or needed_h > height_mm:
        raise ValueError(
            f"marker {label!r} does not fit: needs {needed_w:.1f} x {needed_h:.1f} mm, "
            f"available {width_mm:.1f} x {height_mm:.1f} mm"
        )
    width_pt, height_pt = page_size_pt(page)
    canvas = Canvas(str(path), pagesize=(width_pt, height_pt))
    origin = ((width_mm - symbol.size_mm) / 2, TOP_MARGIN_MM + CUT_OFFSET_MM + quiet_mm)
    draw_runs(canvas, symbol.matrix, symbol.module_mm, origin, height_pt)
    cut_mm = symbol.size_mm + 2 * (quiet_mm + CUT_OFFSET_MM)
    canvas.setStrokeGray(CUT_GREY)
    canvas.setLineWidth(CUT_WIDTH_PT)
    canvas.rect(
        (origin[0] - quiet_mm - CUT_OFFSET_MM) * MM_TO_PT,
        height_pt - (TOP_MARGIN_MM + cut_mm) * MM_TO_PT,
        cut_mm * MM_TO_PT,
        cut_mm * MM_TO_PT,
        stroke=1,
        fill=0,
    )
    text_x = max(SIDE_MARGIN_MM, (width_mm - max(block_mm, RULER_MM)) / 2)
    line_y = TOP_MARGIN_MM + block_mm + LABEL_GAP_MM
    for index, line in enumerate(label_lines):
        is_head = index == 0
        draw_text(
            canvas,
            line,
            text_x,
            line_y + index * LINE_PITCH_MM,
            height_pt,
            FONT_BOLD if is_head else FONT,
            LABEL_PT if is_head else TEXT_PT,
        )
    ruler = (text_x, line_y + LINE_PITCH_MM * len(label_lines))
    draw_ruler(canvas, ruler[0], ruler[1], height_pt)
    canvas.showPage()
    canvas.save()
    return PdfLayout(origin_mm=origin, edge_mm=symbol.size_mm, ruler_origin_mm=ruler, page_mm=(width_mm, height_mm))


def draw_mask_pdf(
    mask: np.ndarray,
    cell_mm: float,
    page: PageSpec,
    path: Path,
    origin_mm: tuple[float, float],
    label: str,
    label_y_mm: float | None = None,
) -> None:
    """Write a one-page PDF with a boolean mask drawn as vector cells at origin_mm.

    Args:
        mask: 2D bool array, True = dark.
        cell_mm: Cell edge in mm.
        page: The media.
        path: Output file.
        origin_mm: Mask top-left from the page top-left, mm.
        label: Text line drawn at the page bottom margin.
        label_y_mm: Label baseline from the page top; default is BOTTOM_MARGIN_MM above the page bottom.
    """
    width_pt, height_pt = page_size_pt(page)
    canvas = Canvas(str(path), pagesize=(width_pt, height_pt))
    draw_runs(canvas, mask, cell_mm, origin_mm, height_pt)
    height_mm = height_pt / MM_TO_PT
    baseline_mm = height_mm - BOTTOM_MARGIN_MM if label_y_mm is None else label_y_mm
    draw_text(canvas, label, SIDE_MARGIN_MM, baseline_mm, height_pt, FONT, TEXT_PT)
    canvas.showPage()
    canvas.save()
