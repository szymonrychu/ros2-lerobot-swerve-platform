# Marker and calibration board generator implementation plan

## Agreed rung

Rung 1 of plan.md, migrated from the epic format: the steps are the epic's tasks.

## Approach

Migrated from the epic format: one step per task file, in dependency order, each with its status, files, verify, verified head and every body section.

## Chunk 1: migrated tasks (model: sonnet)

### S1: Marker payload codec

- Status: done
- Files: `shared/ros2_common/marker_payload.py`, `tests/test_shared_marker_payload.py`, `shared/README.md`
- Verify: `uv run pytest tests/test_shared_marker_payload.py -q && uv run poe lint`
- Verified head: c72ec5cb52aa8c31a7273dc834a95e3202aa9aa2

#### Requirement

- `shared/ros2_common/marker_payload.py` (pydantic v2, no ROS, constants at the top), the format of design D2/D3:
  - Constants: `PAYLOAD_PREFIX = "P"`, `SUPPORTED_MAJOR = 1`, `ALPHANUMERIC = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ $%*+-./:"`, `KNOWN_KINDS = {"BASE": "base", "CHARGER": "charger", "DROPOFF": "drop_off", "BUTTON": "button", "SWITCH": "switch", "TOOLSTATION": "tool_station"}`, `MIN_SIZE_MM = 20.0`, `MAX_SIZE_MM = 1000.0`.
  - `class MarkerPayload(BaseModel)`: `version_major: int`, `version_minor: int | None`, `kind_code: str` (`[A-Z0-9-]{1,16}`), `name: str` (`[A-Z0-9-]{1,20}`), `size_mm: float` (20..1000, at most one decimal), `params: dict[str, str]` (keys `[A-Z0-9]{1,8}` not in P/K/N/S, values `[A-Z0-9 $%*+.-]{0,24}`); properties `poi_kind` (KNOWN_KINDS value, else `kind_code.lower()`), `key` (`f"{poi_kind}/{name}"`).
  - `encode_payload(payload: MarkerPayload) -> str`: `P<major>[.<minor>]/K:<K>/N:<N>/S:<S>` then params sorted by key as `/<KEY>:<VAL>`; `S` printed without a trailing `.0` (`150`, `42.5`); the result only uses ALPHANUMERIC characters.
  - `decode_payload(text: str) -> MarkerPayload`: raises `PayloadError(ValueError)` with a message naming the problem for: a missing or malformed `P` segment, major != 1 (`unsupported payload version 2`), a missing K, N or S, a duplicate key, an empty segment, a character outside ALPHANUMERIC, an out-of-range or malformed S. Unknown keys become params. `decode_payload(encode_payload(p)) == p` for every valid p.
  - `class MarkerInfo(BaseModel)` (extra forbidden): `kind: str`, `id: str`, `key: str`, `size_mm: float`, `params: dict[str, str]`, `payload: str`, `version: int`, `pose_samples: int >= 0`; `marker_info(payload: MarkerPayload, pose_samples: int = 1) -> MarkerInfo` builds it (payload = canonical text).
- Tests first in `tests/test_shared_marker_payload.py` (import as `shared.ros2_common.marker_payload`), seen failing: round trip for every known kind and for params; `P1/K:CHARGER/N:DOCK-1/S:150` decodes to kind charger, key `charger/DOCK-1`, size 150.0; canonical order (params given out of order encode sorted); `P1.3/...` accepted, `P2/...` rejected naming version 2; each malformed case above raises PayloadError with its reason; unknown kind `K:LAMP` maps to `lamp`; lower-case input rejected (outside the alphanumeric set); MarkerInfo built from a payload carries the canonical text.
- `shared/README.md`: a `marker_payload` section with the grammar, the kind table, the size definition (symbol edge excluding the quiet zone, design D3) and the marker frame axes.

#### Acceptance conditions

- a1: encode and decode round-trip every known kind and params in canonical order [evidence: test: tests/test_shared_marker_payload.py]
- a2: unknown major versions and every malformed payload are rejected with a named reason [evidence: test: tests/test_shared_marker_payload.py]
- a3: unknown kinds map to their lower-case POI kind and the key is kind/name [evidence: test: tests/test_shared_marker_payload.py]
- a4: MarkerInfo is built from a decoded payload [evidence: symbol: shared/ros2_common/marker_payload.py:marker_info]
- a5: shared README documents the grammar, kinds, size and frame [evidence: command: grep -q 'marker_payload' shared/README.md && grep -q 'quiet zone' shared/README.md]

#### Files

- `shared/ros2_common/marker_payload.py`
- `tests/test_shared_marker_payload.py`
- `shared/README.md`

#### Interfaces

- `shared/ros2_common/marker_payload.py`: `MarkerPayload`, `PayloadError`, `encode_payload`, `decode_payload`, `MarkerInfo`, `marker_info`, `KNOWN_KINDS` (part `marker-payload`).

#### Expected outcome

`uv run pytest tests/test_shared_marker_payload.py -q` passes; any component can turn a QR text into a validated payload and back.

#### Steps

- Write `tests/test_shared_marker_payload.py`; run `uv run pytest tests/test_shared_marker_payload.py -q`; see it fail on the missing module.
- Implement `shared/ros2_common/marker_payload.py`; rerun until green; `uv run poe lint`.
- Update `shared/README.md`; commit `feat(shared): marker payload codec`.

#### Notes

- Precedent for a shared pydantic model used by poi_store: `shared/ros2_common/photos.py:PhotoSelector`.
- No regex-free hand parser games: a split on `/` then `:` (first colon) is enough; validation lives in the pydantic fields.
- Do not add a POI `category`; the agent-skills epic adds it later (design D11).

#### Ruling

Accepted by the orchestrator: verified (32 passed, lint clean); the gate-repeat was an AC judgement the triage itself declined.

### S2: Marker QR symbol

- Status: done
- Files: `scripts/markers/__init__.py`, `scripts/markers/config.py`, `scripts/markers/symbol.py`, `scripts/markers_generate.yaml`, `tests/test_markers_symbol.py`, `pyproject.toml`, `uv.lock`
- Verify: `uv run pytest tests/test_markers_symbol.py -q && uv run poe lint && uv run ruff check scripts/markers && uv run ruff format --check scripts/markers`
- Verified head: 77ca54c06173fceecb52f7a5311de077d96a2f02

#### Requirement

- Root dev dependencies (pinned majors): `segno>=1.6,<2`, `zxing-cpp>=2.2,<4`; `uv.lock` regenerated with `uv lock`.
- `scripts/markers/config.py`: pydantic models (extra forbidden, errors name the key) and `load_generator_config(path: Path) -> GeneratorConfig`:
  - `GeneratorConfig`: `output_dir` (default `.logs/markers`), `defaults: SymbolDefaults` (`ecc` Literal "m"/"q" default "q", `min_version` 2, `max_version` 3 (2..4), `quiet_zone_modules` 4 (>= 2), `page` (PageSpec: `size` Literal A4/A3/LETTER or `width_mm`/`height_mm`, `orientation` portrait/landscape)), `stl: StlOptions` (`base_thickness_mm` 2.0, `relief_mm` 0.6, `inlay_mm` 0.0, `margin_mm` 3.0, `min_module_mm` 1.5, `warn_module_mm` 2.0), `markers: list[MarkerSpec]`, and an optional `charuco` slot typed `dict[str, Any] | None` that the charuco-board-output task replaces with its model.
  - `MarkerSpec`: `kind_code`, `name`, `size_mm`, `params` (dict), `outputs` (subset of pdf/stl/3mf, default all three), optional `page`; `payload()` returns the `MarkerPayload` (codec validation applies, so an invalid name fails at config load naming the marker).
- `scripts/markers/symbol.py`: `build_symbol(spec: MarkerSpec, defaults: SymbolDefaults) -> Symbol` with segno `make_qr(text, mode="alphanumeric", error=ecc, boost_error=False, version=<smallest fitting in min..max>)`; `Symbol` holds `payload_text`, `version`, `ecc`, `matrix: np.ndarray[bool]` (modules only, no quiet zone, True = dark), `modules` (17 + 4 * version), `module_mm` (size_mm / modules), `size_mm`, `quiet_zone_modules`; a payload too long for `max_version` raises naming the length and the capacity; `render_raster(symbol, px_per_module: int) -> np.ndarray` (uint8 image with quiet zone) for tests and previews.
- `scripts/markers_generate.yaml`: the shipped config with `output_dir`, `defaults`, `stl`, and one marker per known kind (BASE HOME 150, CHARGER DOCK-1 150, DROPOFF BIN-1 100, BUTTON LIGHT-1 60, SWITCH FAN-1 60, TOOLSTATION TS-1 100).
- Tests first in `tests/test_markers_symbol.py` (import `scripts.markers.*`), seen failing: the shipped yaml loads with six markers covering the six known kinds; an unknown key and an invalid marker name fail naming them; `CHARGER DOCK-1` builds version 2, ECC Q (asserted on the segno object, not just the config) and module_mm 6.0 for 150 mm; a payload that only fits version 3 picks 3; a payload over the v3-Q capacity raises naming the capacity; `ecc: m` gives M; `render_raster` of each shipped marker decodes with `zxingcpp.read_barcodes` to the exact payload text.

#### Acceptance conditions

- a1: the shipped generator config loads and covers base, charger, drop-off, button, switch and tool station [evidence: test: tests/test_markers_symbol.py]
- a2: version and error correction come from the config with no silent boost [evidence: test: tests/test_markers_symbol.py]
- a3: an over-long payload is refused naming the capacity [evidence: test: tests/test_markers_symbol.py]
- a4: a rendered symbol decodes back to its payload with zxing-cpp [evidence: test: tests/test_markers_symbol.py]
- a5: segno and zxing-cpp are pinned root dev dependencies [evidence: command: grep -q 'segno' pyproject.toml && grep -q 'zxing-cpp' pyproject.toml && grep -q 'name = "segno"' uv.lock]

#### Files

- `scripts/markers/__init__.py`
- `scripts/markers/config.py`
- `scripts/markers/symbol.py`
- `scripts/markers_generate.yaml`
- `tests/test_markers_symbol.py`
- `pyproject.toml`
- `uv.lock`

#### Interfaces

- `scripts/markers/config.py`: `GeneratorConfig`, `SymbolDefaults`, `PageSpec`, `StlOptions`, `MarkerSpec`, `load_generator_config` (part `markers-config`).
- `scripts/markers/symbol.py`: `Symbol`, `build_symbol`, `render_raster` (part `markers-symbol`).

#### Expected outcome

Each configured marker yields a validated QR matrix with known version, ECC and module size.

#### Steps

- `uv add --group dev 'segno>=1.6,<2' 'zxing-cpp>=2.2,<4'`.
- Write `tests/test_markers_symbol.py`; run `uv run pytest tests/test_markers_symbol.py -q`; see it fail.
- Implement config.py, symbol.py and the yaml; rerun until green; run the verify command.
- Commit `feat(scripts): marker generator config and QR symbol`.

#### Notes

- segno's `boost_error` defaults to True and silently raises the EC level (research fix-as-ac).
- Root tests import scripts as `scripts.<module>` (see tests/test_nav_footprint.py); `scripts/markers/` is a regular package.
- Config file over CLI (CLAUDE.md): the only CLI argument later is the config path.

### S3: Marker PDF output

- Status: done
- Files: `scripts/markers/pdf.py`, `tests/test_markers_pdf.py`, `pyproject.toml`, `uv.lock`
- Verify: `uv run pytest tests/test_markers_pdf.py -q && uv run poe lint && uv run ruff check scripts/markers && uv run ruff format --check scripts/markers`
- Verified head: 2969d62410d8e3dcf09a78b7cdc8aeb4d4917295

#### Requirement

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

#### Acceptance conditions

- a1: the PDF page equals the media and the printed symbol edge equals the encoded size within 0.1 mm [evidence: test: tests/test_markers_pdf.py]
- a2: the quiet zone stays white and the ruler measures 100 mm [evidence: test: tests/test_markers_pdf.py]
- a3: the label carries the payload and the print-at-100-percent instruction [evidence: test: tests/test_markers_pdf.py]
- a4: the rasterised PDF decodes back to the payload [evidence: test: tests/test_markers_pdf.py]
- a5: a marker that does not fit the page is refused [evidence: test: tests/test_markers_pdf.py]

#### Files

- `scripts/markers/pdf.py`
- `tests/test_markers_pdf.py`
- `pyproject.toml`
- `uv.lock`

#### Interfaces

- `scripts/markers/pdf.py`: `write_marker_pdf`, `draw_mask_pdf`, `page_size_pt`, `PdfLayout`, `MM_TO_PT` (part `markers-pdf`).

#### Expected outcome

A printed PDF marker has exactly its encoded edge length and decodes to its payload.

#### Steps

- `uv add --group dev 'reportlab>=4,<5' 'pypdfium2>=4,<6'`.
- Write `tests/test_markers_pdf.py`; run `uv run pytest tests/test_markers_pdf.py -q`; see it fail.
- Implement pdf.py; rerun until green; run the verify command.
- Commit `feat(scripts): exact-size marker PDF`.

#### Notes

- Measure the raster, not the layout numbers (review focus): the test must catch a wrong points-per-mm factor.
- Printers rescale with "fit to page"; the ruler is how the user checks (research fix-as-ac print scaling).

### S4: Marker STL 3MF output

- Status: done
- Files: `scripts/markers/mesh.py`, `tests/test_markers_mesh.py`, `pyproject.toml`, `uv.lock`
- Verify: `uv run pytest tests/test_markers_mesh.py -q && uv run poe lint && uv run ruff check scripts/markers && uv run ruff format --check scripts/markers`
- Verified head: 002cd0a9f3de3f0e26c21c7b512fd3ded5e8bbc6

#### Requirement

- Root dev dependencies `trimesh>=4,<5` and `manifold3d>=3,<4`; `uv.lock` regenerated.
- `scripts/markers/mesh.py` (design D6):
  - `mask_bodies(mask: np.ndarray, cell_mm: float, plate_mm: tuple[float, float], origin_mm: tuple[float, float], opts: StlOptions) -> tuple[trimesh.Trimesh, trimesh.Trimesh]`: base body = plate `plate_mm` x `base_thickness_mm` (z from 0) with pockets of depth `inlay_mm` under every dark cell when `inlay_mm > 0`; module body = the dark cells (adjacent cells unioned) from z `base_thickness_mm - inlay_mm` to `base_thickness_mm + relief_mm`; both watertight; their intersection volume is zero. Raises when `relief_mm + inlay_mm <= 0`, when `inlay_mm >= base_thickness_mm`, or when `cell_mm < opts.min_module_mm` (names the module size); logs a warning (standard `logging`) below `warn_module_mm`.
  - `marker_bodies(symbol: Symbol, opts: StlOptions) -> tuple[Trimesh, Trimesh]`: plate side = size_mm + 2 * quiet zone + 2 * `margin_mm`, symbol centred.
  - `write_bodies(base, modules, stem: Path) -> list[Path]`: `<stem>-base.stl`, `<stem>-modules.stl` (same coordinates) and `<stem>.3mf` with two objects named `base` and `modules`, plus `Metadata/model_settings.config` in the 3MF zip assigning extruder 1 to `base` and 2 to `modules` (Bambu Studio per-object filament).
- Tests first in `tests/test_markers_mesh.py`, seen failing:
  - both bodies watertight, intersection volume < 1e-6 mm3 (manifold boolean);
  - module footprint: sampling the module body's top at every cell centre (ray or point containment at z = base_thickness + relief/2) reproduces `symbol.matrix` exactly, and no module material exists in the quiet zone;
  - heights: base top at `base_thickness_mm`, module top at `base_thickness_mm + relief_mm`; with `inlay_mm: 0.6, relief_mm: 0` the module top is coplanar with the base top and the base has pockets;
  - plate size: 150 mm v2 marker with quiet zone 4 and margin 3 gives a 204 mm plate (150 + 2*24 + 2*3), within 0.01 mm;
  - a 30 mm v3 marker (1.03 mm modules) raises naming the module size; a 50 mm v2 marker (2.0 mm) passes, a 45 mm one (1.8 mm) warns;
  - the 3MF reloads with trimesh as two geometries named base and modules, and its zip has `Metadata/model_settings.config` naming both with extruders 1 and 2;
  - render-then-decode: a top-view raster of the module footprint (cells of the body top, with quiet zone) decodes with zxing-cpp to the payload.

#### Acceptance conditions

- a1: base and module bodies are watertight and disjoint [evidence: test: tests/test_markers_mesh.py]
- a2: the module footprint reproduces the QR matrix and the heights follow base, relief and inlay [evidence: test: tests/test_markers_mesh.py]
- a3: modules under 1.5 mm are refused and under 2 mm warned [evidence: test: tests/test_markers_mesh.py]
- a4: the 3MF holds named base and modules objects with Bambu extruder settings [evidence: test: tests/test_markers_mesh.py]
- a5: the printed module footprint decodes back to the payload [evidence: test: tests/test_markers_mesh.py]

#### Files

- `scripts/markers/mesh.py`
- `tests/test_markers_mesh.py`
- `pyproject.toml`
- `uv.lock`

#### Interfaces

- `scripts/markers/mesh.py`: `mask_bodies`, `marker_bodies`, `write_bodies` (part `markers-mesh`).

#### Expected outcome

Each marker gets a base STL, a module STL and a two-object 3MF that load as one two-colour print.

#### Steps

- `uv add --group dev 'trimesh>=4,<5' 'manifold3d>=3,<4'`.
- Write `tests/test_markers_mesh.py`; run `uv run pytest tests/test_markers_mesh.py -q`; see it fail.
- Implement mesh.py; rerun until green; run the verify command.
- Commit `feat(scripts): two-colour marker STL and 3MF`.

#### Notes

- Raised modules are the default because the user chose them (U5); the flush inlay is a config option.
- Bambu Studio ignores 3MF colours; filaments come from `model_settings.config` (research). Keep the XML minimal: `<config><object id=... ><metadata key="name" value="base"/><metadata key="extruder" value="1"/></object>...</config>`; the user confirms it in Bambu Studio during the runbook.

### S5: Charuco board output

- Status: done
- Files: `scripts/markers/charuco.py`, `scripts/markers/config.py`, `scripts/markers_generate.yaml`, `tests/test_markers_charuco.py`
- Verify: `uv run pytest tests/test_markers_charuco.py tests/test_markers_symbol.py -q && uv run poe lint && uv run ruff check scripts/markers && uv run ruff format --check scripts/markers`
- Verified head: 821d49d1042311778618e240c71ac68d4824cce9

#### Requirement

- `scripts/markers/charuco.py` (design D7):
  - `class CharucoSpec(BaseModel)` (extra forbidden): `squares_x` 6, `squares_y` 5, `square_mm` 40.0, `marker_mm` 30.0, `dictionary` `DICT_5X5_100` (any `cv2.aruco.DICT_*` name), `margin_mm` 5.0, `thickness_mm` 4.0 (>= 4), `relief_mm` 0.6, `bed_mm` 256.0, `page` (A4 landscape), `outputs` (pdf/stl/3mf); validators refuse `marker_mm >= square_mm`, an even `squares_y` (legacy layout change), and a plate (`squares * square_mm + 2 * margin_mm`) larger than `bed_mm` in either direction, each naming the field.
  - `make_charuco_board(spec) -> cv2.aruco.CharucoBoard` (`cv2.aruco.CharucoBoard((squares_x, squares_y), square_mm / 1000, marker_mm / 1000, dictionary)`).
  - `board_mask(spec) -> tuple[np.ndarray, float]`: boolean dark mask of the board and its cell size in mm, from `board.generateImage` at a resolution where squares, marker cells and the square-to-marker gap are whole pixels (for 40/30 mm and a 7-cell 5x5 marker: 1.4 px/mm); asserts the resolution is exact.
  - `write_charuco(spec, opts: StlOptions, stem: Path) -> list[Path]`: PDF via `draw_mask_pdf` (label with the spec and `Print at 100 %`), STL/3MF via `mask_bodies` with the plate `(squares_x * square_mm + 2 * margin_mm, squares_y * square_mm + 2 * margin_mm)`, `thickness_mm` as base thickness and `relief_mm`, then `write_bodies`.
- `scripts/markers/config.py`: the `charuco` slot becomes `CharucoSpec | None`; `scripts/markers_generate.yaml` gains the `charuco:` section with the defaults above.
- Tests first in `tests/test_markers_charuco.py`, seen failing: the shipped spec is 6x5, 40/30 mm, DICT_5X5_100 and its plate is 250 x 210 mm (<= 256); 7x5 at 40 mm is refused (280 + 10 > 256), an even row count and `marker_mm >= square_mm` are refused; the PDF rasterised at 600 dpi is detected by `cv2.aruco.CharucoDetector` with all 20 inner corners and the measured square size is 40 mm within 0.1 mm; the module body footprint rasterised (top view) is detected with all 20 corners whose positions match `board.getChessboardCorners()` within 0.2 mm; the STL base is 4 mm thick.

#### Acceptance conditions

- a1: the shipped ChArUco spec is 6x5 squares of 40 mm with 30 mm DICT_5X5_100 markers and a plate that fits a 256 mm bed [evidence: test: tests/test_markers_charuco.py]
- a2: specs that exceed the bed, have even rows or oversized markers are refused [evidence: test: tests/test_markers_charuco.py]
- a3: the ChArUco PDF is detected with all corners at 40 mm squares [evidence: test: tests/test_markers_charuco.py]
- a4: the ChArUco STL module footprint is detected with corners at the board geometry [evidence: test: tests/test_markers_charuco.py]

#### Files

- `scripts/markers/charuco.py`
- `scripts/markers/config.py`
- `scripts/markers_generate.yaml`
- `tests/test_markers_charuco.py`

#### Interfaces

- `scripts/markers/charuco.py`: `CharucoSpec`, `make_charuco_board`, `board_mask`, `write_charuco` (part `charuco-spec`); the `charuco` section of `scripts/markers_generate.yaml` is the single board definition (the head RGB-D fusion epic's `rgbd-calibration-frames` asserts its yaml `board` equals it).

#### Expected outcome

One config section defines the calibration board; the generator prints it as PDF and two-colour STL/3MF that OpenCV detects.

#### Steps

- Write `tests/test_markers_charuco.py`; run `uv run pytest tests/test_markers_charuco.py -q`; see it fail.
- Implement charuco.py, the config slot and the yaml section; rerun until green; run the verify command.
- Commit `feat(scripts): ChArUco calibration board output`.

#### Notes

- The fusion epic's 7x5 x 40 mm board (280 mm) did not fit the 256 mm bed; it now uses this 6x5 spec (same square and marker sizes, so ToF detectability is unchanged).
- opencv-python-headless >= 4.9 in the root dev env has `cv2.aruco.CharucoDetector`; no contrib wheel.

### S6: Markers generate CLI

- Status: done
- Files: `scripts/markers_generate.py`, `scripts/markers_generate.yaml`, `tests/test_markers_generate_cli.py`, `scripts/README.md`, `tests/README.md`
- Verify: `uv run pytest tests/test_markers_generate_cli.py -q && uv run poe lint && uv run ruff check scripts/markers && uv run ruff format --check scripts/markers && uv run ruff check scripts/markers_generate.py && uv run ruff format --check scripts/markers_generate.py`
- Verified head: 093dbaa5ff59dcf76d2aac716ab57e4011ad667d

#### Requirement

- `scripts/markers_generate.py`: `main(argv: list[str] | None = None) -> int`, run as `uv run python -m scripts.markers_generate [config]`; the only argument is an optional config path (default `scripts/markers_generate.yaml`). For each marker: build the symbol, write the requested outputs into `output_dir` with the stem `<kind>-<name>` (lower case), e.g. `charger-dock-1.pdf`, `charger-dock-1-base.stl`, `charger-dock-1-modules.stl`, `charger-dock-1.3mf`; then the ChArUco outputs with stem `charuco-6x5-40mm` when `charuco` is set; print one line per written file and a summary. Exit 1 with the pydantic error (naming the key) on an invalid config, and exit 1 without writing that marker's files when a marker cannot be produced (too small for FDM, does not fit the page), naming it, after producing the others. A relative `output_dir` resolves against the repo root.
- Tests first in `tests/test_markers_generate_cli.py`, seen failing: a tmp config with two markers and the charuco section writes exactly the expected file set into a tmp dir and returns 0; an unknown key returns 1 and writes nothing; a too-small STL marker returns 1, names it, and the other marker's files exist; the shipped `scripts/markers_generate.yaml` runs end to end into a tmp `output_dir` (override by writing a tmp copy) and every PDF decodes (rasterised) to its payload.
- `tests/README.md`: entries for every root test file of this epic (test_shared_marker_payload.py, test_markers_symbol.py, test_markers_pdf.py, test_markers_mesh.py, test_markers_charuco.py, test_markers_generate_cli.py), each with what it covers.
- `scripts/README.md`: a `markers_generate.py` entry: purpose, config keys, outputs per marker and for the board, printing guidance (PDF at 100 % on a rigid plate; STL: load both STLs or the 3MF, white for base and black for modules), the size definition and the minimum module edge.

#### Acceptance conditions

- a1: the CLI writes PDF, two STLs and a 3MF per marker plus the ChArUco outputs from one config [evidence: test: tests/test_markers_generate_cli.py]
- a2: an invalid config exits 1 naming the key and writes nothing [evidence: test: tests/test_markers_generate_cli.py]
- a3: a marker that cannot be produced is named and skipped while the others are written [evidence: test: tests/test_markers_generate_cli.py]
- a4: scripts README documents the generator and printing [evidence: command: grep -q 'markers_generate.py' scripts/README.md]
- a5: tests README lists every root test file of the generator [evidence: command: for f in test_shared_marker_payload test_markers_symbol test_markers_pdf test_markers_mesh test_markers_charuco test_markers_generate_cli; do grep -q "$f" tests/README.md || exit 1; done]

#### Files

- `scripts/markers_generate.py`
- `scripts/markers_generate.yaml`
- `tests/test_markers_generate_cli.py`
- `scripts/README.md`
- `tests/README.md`

#### Interfaces

- `scripts/markers_generate.py:main`; config `scripts/markers_generate.yaml`.

#### Expected outcome

`uv run python -m scripts.markers_generate` fills `.logs/markers/` with print files for every configured marker and the board.

#### Steps

- Write `tests/test_markers_generate_cli.py`; run `uv run pytest tests/test_markers_generate_cli.py -q`; see it fail.
- Implement the entry point; rerun until green; run the verify command.
- Update scripts/README.md; commit `feat(scripts): markers_generate CLI`.

#### Notes

- `.logs/` is gitignored: generated print files are never committed.

## Files to delete

none recorded in the epic format

## Final verify

none recorded in the epic format

## After merge

none
