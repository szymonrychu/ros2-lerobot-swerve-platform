---
title: Marker STL 3MF output
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "A marker prints in two colours on the Bambu AMS: a white base and black raised modules as separate parts."
files: [scripts/markers/mesh.py, tests/test_markers_mesh.py, pyproject.toml, uv.lock]
verify: "uv run pytest tests/test_markers_mesh.py -q && uv run poe lint && uv run ruff check scripts/markers && uv run ruff format --check scripts/markers"
depends_on: [marker-pdf-output]
owns: [markers-mesh]
uses: [markers-config, markers-symbol]
model: sonnet
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/marker-and-calibration-board--marker-stl-3mf-output
branch: marker-and-calibration-board--marker-stl-3mf-output
verified_head: ""
run: ""
provides: []
---

# Marker STL 3MF output

## Requirement

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

## Acceptance conditions

- a1: base and module bodies are watertight and disjoint [evidence: test: tests/test_markers_mesh.py]
- a2: the module footprint reproduces the QR matrix and the heights follow base, relief and inlay [evidence: test: tests/test_markers_mesh.py]
- a3: modules under 1.5 mm are refused and under 2 mm warned [evidence: test: tests/test_markers_mesh.py]
- a4: the 3MF holds named base and modules objects with Bambu extruder settings [evidence: test: tests/test_markers_mesh.py]
- a5: the printed module footprint decodes back to the payload [evidence: test: tests/test_markers_mesh.py]

## Files

- `scripts/markers/mesh.py`
- `tests/test_markers_mesh.py`
- `pyproject.toml`
- `uv.lock`

## Interfaces

- `scripts/markers/mesh.py`: `mask_bodies`, `marker_bodies`, `write_bodies` (part `markers-mesh`).

## Expected outcome

Each marker gets a base STL, a module STL and a two-object 3MF that load as one two-colour print.

## Steps

- `uv add --group dev 'trimesh>=4,<5' 'manifold3d>=3,<4'`.
- Write `tests/test_markers_mesh.py`; run `uv run pytest tests/test_markers_mesh.py -q`; see it fail.
- Implement mesh.py; rerun until green; run the verify command.
- Commit `feat(scripts): two-colour marker STL and 3MF`.

## Notes

- Raised modules are the default because the user chose them (U5); the flush inlay is a config option.
- Bambu Studio ignores 3MF colours; filaments come from `model_settings.config` (research). Keep the XML minimal: `<config><object id=... ><metadata key="name" value="base"/><metadata key="extruder" value="1"/></object>...</config>`; the user confirms it in Bambu Studio during the runbook.
