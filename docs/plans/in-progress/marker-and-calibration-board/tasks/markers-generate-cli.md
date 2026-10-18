---
title: Markers generate CLI
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "One command, driven by one config file, writes every marker and the calibration board ready to print."
files: [scripts/markers_generate.py, scripts/markers_generate.yaml, tests/test_markers_generate_cli.py, scripts/README.md, tests/README.md]
verify: "uv run pytest tests/test_markers_generate_cli.py -q && uv run poe lint && uv run ruff check scripts/markers && uv run ruff format --check scripts/markers && uv run ruff check scripts/markers_generate.py && uv run ruff format --check scripts/markers_generate.py"
depends_on: [charuco-board-output]
owns: []
uses: [markers-config, markers-symbol, markers-pdf, markers-mesh, charuco-spec]
model: sonnet
estimate: 45
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/marker-and-calibration-board--markers-generate-cli
branch: marker-and-calibration-board--markers-generate-cli
verified_head: ""
run: ""
provides: []
---

# Markers generate CLI

## Requirement

- `scripts/markers_generate.py`: `main(argv: list[str] | None = None) -> int`, run as `uv run python -m scripts.markers_generate [config]`; the only argument is an optional config path (default `scripts/markers_generate.yaml`). For each marker: build the symbol, write the requested outputs into `output_dir` with the stem `<kind>-<name>` (lower case), e.g. `charger-dock-1.pdf`, `charger-dock-1-base.stl`, `charger-dock-1-modules.stl`, `charger-dock-1.3mf`; then the ChArUco outputs with stem `charuco-6x5-40mm` when `charuco` is set; print one line per written file and a summary. Exit 1 with the pydantic error (naming the key) on an invalid config, and exit 1 without writing that marker's files when a marker cannot be produced (too small for FDM, does not fit the page), naming it, after producing the others. A relative `output_dir` resolves against the repo root.
- Tests first in `tests/test_markers_generate_cli.py`, seen failing: a tmp config with two markers and the charuco section writes exactly the expected file set into a tmp dir and returns 0; an unknown key returns 1 and writes nothing; a too-small STL marker returns 1, names it, and the other marker's files exist; the shipped `scripts/markers_generate.yaml` runs end to end into a tmp `output_dir` (override by writing a tmp copy) and every PDF decodes (rasterised) to its payload.
- `tests/README.md`: entries for every root test file of this epic (test_shared_marker_payload.py, test_markers_symbol.py, test_markers_pdf.py, test_markers_mesh.py, test_markers_charuco.py, test_markers_generate_cli.py), each with what it covers.
- `scripts/README.md`: a `markers_generate.py` entry: purpose, config keys, outputs per marker and for the board, printing guidance (PDF at 100 % on a rigid plate; STL: load both STLs or the 3MF, white for base and black for modules), the size definition and the minimum module edge.

## Acceptance conditions

- a1: the CLI writes PDF, two STLs and a 3MF per marker plus the ChArUco outputs from one config [evidence: test: tests/test_markers_generate_cli.py]
- a2: an invalid config exits 1 naming the key and writes nothing [evidence: test: tests/test_markers_generate_cli.py]
- a3: a marker that cannot be produced is named and skipped while the others are written [evidence: test: tests/test_markers_generate_cli.py]
- a4: scripts README documents the generator and printing [evidence: command: grep -q 'markers_generate.py' scripts/README.md]
- a5: tests README lists every root test file of the generator [evidence: command: for f in test_shared_marker_payload test_markers_symbol test_markers_pdf test_markers_mesh test_markers_charuco test_markers_generate_cli; do grep -q "$f" tests/README.md || exit 1; done]

## Files

- `scripts/markers_generate.py`
- `scripts/markers_generate.yaml`
- `tests/test_markers_generate_cli.py`
- `scripts/README.md`
- `tests/README.md`

## Interfaces

- `scripts/markers_generate.py:main`; config `scripts/markers_generate.yaml`.

## Expected outcome

`uv run python -m scripts.markers_generate` fills `.logs/markers/` with print files for every configured marker and the board.

## Steps

- Write `tests/test_markers_generate_cli.py`; run `uv run pytest tests/test_markers_generate_cli.py -q`; see it fail.
- Implement the entry point; rerun until green; run the verify command.
- Update scripts/README.md; commit `feat(scripts): markers_generate CLI`.

## Notes

- `.logs/` is gitignored: generated print files are never committed.
