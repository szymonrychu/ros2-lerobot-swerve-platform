---
title: Marker QR symbol
status: todo
repo: ros2-lerobot-sverve-platform
value: "A marker spec becomes a QR matrix with the right version, error correction and physical module size, checked before anything is printed."
files: [scripts/markers/__init__.py, scripts/markers/config.py, scripts/markers/symbol.py, scripts/markers_generate.yaml, tests/test_markers_symbol.py, pyproject.toml, uv.lock]
verify: "uv run pytest tests/test_markers_symbol.py -q && uv run poe lint && uv run ruff check scripts/markers && uv run ruff format --check scripts/markers"
depends_on: [marker-payload-codec]
owns: [markers-config, markers-symbol]
uses: [marker-payload]
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

# Marker QR symbol

## Requirement

- Root dev dependencies (pinned majors): `segno>=1.6,<2`, `zxing-cpp>=2.2,<4`; `uv.lock` regenerated with `uv lock`.
- `scripts/markers/config.py`: pydantic models (extra forbidden, errors name the key) and `load_generator_config(path: Path) -> GeneratorConfig`:
  - `GeneratorConfig`: `output_dir` (default `.logs/markers`), `defaults: SymbolDefaults` (`ecc` Literal "m"/"q" default "q", `min_version` 2, `max_version` 3 (2..4), `quiet_zone_modules` 4 (>= 2), `page` (PageSpec: `size` Literal A4/A3/LETTER or `width_mm`/`height_mm`, `orientation` portrait/landscape)), `stl: StlOptions` (`base_thickness_mm` 2.0, `relief_mm` 0.6, `inlay_mm` 0.0, `margin_mm` 3.0, `min_module_mm` 1.5, `warn_module_mm` 2.0), `markers: list[MarkerSpec]`, and an optional `charuco` slot typed `dict[str, Any] | None` that the charuco-board-output task replaces with its model.
  - `MarkerSpec`: `kind_code`, `name`, `size_mm`, `params` (dict), `outputs` (subset of pdf/stl/3mf, default all three), optional `page`; `payload()` returns the `MarkerPayload` (codec validation applies, so an invalid name fails at config load naming the marker).
- `scripts/markers/symbol.py`: `build_symbol(spec: MarkerSpec, defaults: SymbolDefaults) -> Symbol` with segno `make_qr(text, mode="alphanumeric", error=ecc, boost_error=False, version=<smallest fitting in min..max>)`; `Symbol` holds `payload_text`, `version`, `ecc`, `matrix: np.ndarray[bool]` (modules only, no quiet zone, True = dark), `modules` (17 + 4 * version), `module_mm` (size_mm / modules), `size_mm`, `quiet_zone_modules`; a payload too long for `max_version` raises naming the length and the capacity; `render_raster(symbol, px_per_module: int) -> np.ndarray` (uint8 image with quiet zone) for tests and previews.
- `scripts/markers_generate.yaml`: the shipped config with `output_dir`, `defaults`, `stl`, and one marker per known kind (BASE HOME 150, CHARGER DOCK-1 150, DROPOFF BIN-1 100, BUTTON LIGHT-1 60, SWITCH FAN-1 60, TOOLSTATION TS-1 100).
- Tests first in `tests/test_markers_symbol.py` (import `scripts.markers.*`), seen failing: the shipped yaml loads with six markers covering the six known kinds; an unknown key and an invalid marker name fail naming them; `CHARGER DOCK-1` builds version 2, ECC Q (asserted on the segno object, not just the config) and module_mm 6.0 for 150 mm; a payload that only fits version 3 picks 3; a payload over the v3-Q capacity raises naming the capacity; `ecc: m` gives M; `render_raster` of each shipped marker decodes with `zxingcpp.read_barcodes` to the exact payload text.

## Acceptance conditions

- a1: the shipped generator config loads and covers base, charger, drop-off, button, switch and tool station [evidence: test: tests/test_markers_symbol.py]
- a2: version and error correction come from the config with no silent boost [evidence: test: tests/test_markers_symbol.py]
- a3: an over-long payload is refused naming the capacity [evidence: test: tests/test_markers_symbol.py]
- a4: a rendered symbol decodes back to its payload with zxing-cpp [evidence: test: tests/test_markers_symbol.py]
- a5: segno and zxing-cpp are pinned root dev dependencies [evidence: command: grep -q 'segno' pyproject.toml && grep -q 'zxing-cpp' pyproject.toml && grep -q 'name = "segno"' uv.lock]

## Files

- `scripts/markers/__init__.py`
- `scripts/markers/config.py`
- `scripts/markers/symbol.py`
- `scripts/markers_generate.yaml`
- `tests/test_markers_symbol.py`
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
