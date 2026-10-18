---
title: Charuco board output
status: done
repo: ros2-lerobot-sverve-platform
value: "The head RGB-D calibration board prints from the same tool, on paper or the Bambu bed, with one definition the calibration tool also uses."
files: [scripts/markers/charuco.py, scripts/markers/config.py, scripts/markers_generate.yaml, tests/test_markers_charuco.py]
verify: "uv run pytest tests/test_markers_charuco.py tests/test_markers_symbol.py -q && uv run poe lint && uv run ruff check scripts/markers && uv run ruff format --check scripts/markers"
depends_on: [marker-stl-3mf-output]
owns: [charuco-spec]
uses: [markers-config, markers-pdf, markers-mesh]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/marker-and-calibration-board--charuco-board-output
branch: marker-and-calibration-board--charuco-board-output
verified_head: 821d49d1042311778618e240c71ac68d4824cce9
run: marker-and-calibration-board
provides: [markers-models, charuco-spec]
acs_for_audit: []
process_gaps: []
---

# Charuco board output

## Requirement

- `scripts/markers/charuco.py` (design D7):
  - `class CharucoSpec(BaseModel)` (extra forbidden): `squares_x` 6, `squares_y` 5, `square_mm` 40.0, `marker_mm` 30.0, `dictionary` `DICT_5X5_100` (any `cv2.aruco.DICT_*` name), `margin_mm` 5.0, `thickness_mm` 4.0 (>= 4), `relief_mm` 0.6, `bed_mm` 256.0, `page` (A4 landscape), `outputs` (pdf/stl/3mf); validators refuse `marker_mm >= square_mm`, an even `squares_y` (legacy layout change), and a plate (`squares * square_mm + 2 * margin_mm`) larger than `bed_mm` in either direction, each naming the field.
  - `make_charuco_board(spec) -> cv2.aruco.CharucoBoard` (`cv2.aruco.CharucoBoard((squares_x, squares_y), square_mm / 1000, marker_mm / 1000, dictionary)`).
  - `board_mask(spec) -> tuple[np.ndarray, float]`: boolean dark mask of the board and its cell size in mm, from `board.generateImage` at a resolution where squares, marker cells and the square-to-marker gap are whole pixels (for 40/30 mm and a 7-cell 5x5 marker: 1.4 px/mm); asserts the resolution is exact.
  - `write_charuco(spec, opts: StlOptions, stem: Path) -> list[Path]`: PDF via `draw_mask_pdf` (label with the spec and `Print at 100 %`), STL/3MF via `mask_bodies` with the plate `(squares_x * square_mm + 2 * margin_mm, squares_y * square_mm + 2 * margin_mm)`, `thickness_mm` as base thickness and `relief_mm`, then `write_bodies`.
- `scripts/markers/config.py`: the `charuco` slot becomes `CharucoSpec | None`; `scripts/markers_generate.yaml` gains the `charuco:` section with the defaults above.
- Tests first in `tests/test_markers_charuco.py`, seen failing: the shipped spec is 6x5, 40/30 mm, DICT_5X5_100 and its plate is 250 x 210 mm (<= 256); 7x5 at 40 mm is refused (280 + 10 > 256), an even row count and `marker_mm >= square_mm` are refused; the PDF rasterised at 600 dpi is detected by `cv2.aruco.CharucoDetector` with all 20 inner corners and the measured square size is 40 mm within 0.1 mm; the module body footprint rasterised (top view) is detected with all 20 corners whose positions match `board.getChessboardCorners()` within 0.2 mm; the STL base is 4 mm thick.

## Acceptance conditions

- a1: the shipped ChArUco spec is 6x5 squares of 40 mm with 30 mm DICT_5X5_100 markers and a plate that fits a 256 mm bed [evidence: test: tests/test_markers_charuco.py]
- a2: specs that exceed the bed, have even rows or oversized markers are refused [evidence: test: tests/test_markers_charuco.py]
- a3: the ChArUco PDF is detected with all corners at 40 mm squares [evidence: test: tests/test_markers_charuco.py]
- a4: the ChArUco STL module footprint is detected with corners at the board geometry [evidence: test: tests/test_markers_charuco.py]

## Files

- `scripts/markers/charuco.py`
- `scripts/markers/config.py`
- `scripts/markers_generate.yaml`
- `tests/test_markers_charuco.py`

## Interfaces

- `scripts/markers/charuco.py`: `CharucoSpec`, `make_charuco_board`, `board_mask`, `write_charuco` (part `charuco-spec`); the `charuco` section of `scripts/markers_generate.yaml` is the single board definition (the head RGB-D fusion epic's `rgbd-calibration-frames` asserts its yaml `board` equals it).

## Expected outcome

One config section defines the calibration board; the generator prints it as PDF and two-colour STL/3MF that OpenCV detects.

## Steps

- Write `tests/test_markers_charuco.py`; run `uv run pytest tests/test_markers_charuco.py -q`; see it fail.
- Implement charuco.py, the config slot and the yaml section; rerun until green; run the verify command.
- Commit `feat(scripts): ChArUco calibration board output`.

## Notes

- The fusion epic's 7x5 x 40 mm board (280 mm) did not fit the 256 mm bed; it now uses this 6x5 spec (same square and marker sizes, so ToF detectability is unchanged).
- opencv-python-headless >= 4.9 in the root dev env has `cv2.aruco.CharucoDetector`; no contrib wheel.
