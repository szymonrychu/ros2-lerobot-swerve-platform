---
title: Marker PDF output
status: todo
repo: ros2-lerobot-sverve-platform
value: "A marker prints on paper at exactly its encoded size, with a ruler to check the printer did not rescale it."
files: [scripts/markers/pdf.py, tests/test_markers_pdf.py, pyproject.toml, uv.lock]
verify: "uv run pytest tests/test_markers_pdf.py -q && uv run poe lint && uv run ruff check scripts/markers && uv run ruff format --check scripts/markers"
depends_on: [marker-qr-symbol]
owns: [markers-pdf]
uses: [markers-config, markers-symbol]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Marker PDF output

## Requirement

- Root dev dependencies `reportlab>=4,<5` and `pypdfium2>=4,<6`; `uv.lock` regenerated.
- `scripts/markers/pdf.py`:
  - `MM_TO_PT = 72 / 25.4` at the top; `page_size_pt(page: PageSpec) -> tuple[float, float]`.
  - `write_marker_pdf(symbol: Symbol, page: PageSpec, path: Path, label: str) -> PdfLayout`: one page whose MediaBox equals the media (no scaling), the symbol centred horizontally in the upper part, dark modules drawn as filled vector rectangles (horizontal runs merged), quiet zone left white, a thin grey cut square 1 mm outside the quiet zone, the label (kind, name, size in mm, payload text), a 100 mm ruler bar with 10 mm ticks and the line `Print at 100 % (actual size), no fit to page; mount on a rigid plate`; raises naming the needed and available size when the symbol, quiet zone, label and ruler do not fit. `PdfLayout` reports the symbol origin and edge in mm (used by the ChArUco task too).
  - `draw_mask_pdf(mask: np.ndarray, cell_mm: float, page: PageSpec, path: Path, origin_mm: tuple[float, float], label: str) -> None`: the same vector run drawing for any boolean mask (the ChArUco task reuses it).
- Tests first in `tests/test_markers_pdf.py`, seen failing (helper in the test: rasterise page 0 with pypdfium2 at 600 dpi to a grey array):
  - the page size in points equals A4 (595.28 x 841.89) and A3/landscape/custom variants;
  - the measured symbol edge (dark bounding box of the finders in the raster) equals `size_mm` within 0.1 mm for 60, 100 and 150 mm markers;
  - the quiet zone around the symbol is white for at least `quiet_zone_modules` modules;
  - the ruler bar measures 100 mm within 0.1 mm;
  - the page text (pypdfium2 text extraction) contains the payload text and `Print at 100 %`;
  - zxing-cpp decodes the raster to the payload text (render-then-decode round trip);
  - a 200 mm marker on A4 portrait with label raises the does-not-fit error.

## Acceptance conditions

- a1: the PDF page equals the media and the printed symbol edge equals the encoded size within 0.1 mm [evidence: test: tests/test_markers_pdf.py]
- a2: the quiet zone stays white and the ruler measures 100 mm [evidence: test: tests/test_markers_pdf.py]
- a3: the label carries the payload and the print-at-100-percent instruction [evidence: test: tests/test_markers_pdf.py]
- a4: the rasterised PDF decodes back to the payload [evidence: test: tests/test_markers_pdf.py]
- a5: a marker that does not fit the page is refused [evidence: test: tests/test_markers_pdf.py]

## Files

- `scripts/markers/pdf.py`
- `tests/test_markers_pdf.py`
- `pyproject.toml`
- `uv.lock`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
