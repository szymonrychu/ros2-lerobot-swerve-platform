---
title: Gripper plate marker
status: todo
repo: ros2-lerobot-sverve-platform
value: "The user can print the gripper plate the Arm calibration needs, with ids that never clash with the floor board."
files: [scripts/markers/charuco.py, scripts/markers/models.py, scripts/markers_generate.py, scripts/markers_generate.yaml, tests/test_markers_charuco.py, tests/test_markers_generate_cli.py]
verify: "uv run --frozen pytest tests/test_markers_charuco.py tests/test_markers_generate_cli.py tests/test_markers_mesh.py tests/test_markers_pdf.py -q && uv run poe lint"
depends_on: [calib-config-and-view-types]
owns: []
uses: [head-arm-calib-config]
model: ""
estimate: 50
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Gripper plate marker

## Requirement

The marker generator outputs the gripper ChArUco plate next to the 6x5 board (design D10, approval A4).

- `scripts/markers/charuco.py`: `CharucoSpec.first_marker_id: int = 0` (ge 0); `make_charuco_board(spec)` uses ids starting at it (the 6x5 board output stays byte-identical).
- `scripts/markers/models.py`: the generator config gains `charuco_plate: CharucoSpec | None = None` (omit to skip).
- `scripts/markers_generate.yaml`: a `charuco_plate:` entry: squares 5 x 3, square 18 mm, marker 14 mm, DICT_5X5_100, first_marker_id 15, margin 4 mm, thickness 4 mm, inlay 1.2 mm, outputs pdf, stl, 3mf; a comment says it clips to the fixed jaw for Arm calibrate and that the ids must not overlap the board's 0-14.
- `scripts/markers_generate.py`: writes the plate under the stem `charuco-plate-5x3-18mm` beside the board; a plate failure is reported as `charuco_plate` and the exit code counts it like the board.
- `tests/test_markers_charuco.py`: the shipped plate spec equals `ros2_common.head_rgbd_calibration.config.PLATE_BOARD` (squares, square_mm, marker_mm, dictionary, first_marker_id); the plate PDF rendered at 100 % detects all 8 corners with `make_board(PLATE_BOARD)` and none with `make_board(BoardConfig())`; the plate STL is 4 mm thick and plate-sized; the board outputs are unchanged.
- `tests/test_markers_generate_cli.py`: one CLI run writes both the board and the plate stems.
- Existing marker tests pass.

## Acceptance conditions

- a1: the shipped plate spec equals the core PLATE_BOARD [evidence: test:tests/test_markers_charuco.py::test_shipped_plate_spec_equals_core_plate_board]
- a2: the plate PDF decodes only with the plate board [evidence: test:tests/test_markers_charuco.py::test_plate_pdf_detects_only_with_plate_board]
- a3: the plate STL is 4 mm thick and plate-sized [evidence: test:tests/test_markers_charuco.py::test_plate_stl_is_4_mm_thick]
- a4: the CLI writes the board and the plate [evidence: test:tests/test_markers_generate_cli.py::test_cli_writes_board_and_plate]
- a5: existing marker tests pass [evidence: command:uv run --frozen pytest tests/test_markers_charuco.py tests/test_markers_generate_cli.py tests/test_markers_mesh.py tests/test_markers_pdf.py tests/test_markers_symbol.py -q]
- a6: root lint passes [evidence: command:uv run poe lint]

## Files

- `scripts/markers/charuco.py`
- `scripts/markers/models.py`
- `scripts/markers_generate.py`
- `scripts/markers_generate.yaml`
- `tests/test_markers_charuco.py`
- `tests/test_markers_generate_cli.py`

## Interfaces

- `scripts/markers/charuco.py::CharucoSpec.first_marker_id`; generator config key `charuco_plate`; output stem `charuco-plate-5x3-18mm`.

## Expected outcome

The user can print the gripper plate the Arm calibration needs, with ids that never clash with the floor board.

## Steps

- Tests first (red), implement, run the verify command.
- Run `uv run --frozen pytest tests/test_markers_charuco.py tests/test_markers_generate_cli.py tests/test_markers_mesh.py tests/test_markers_pdf.py -q && uv run poe lint`; expect every test passed and lint clean.

## Notes

- Design: the epic design.md (`ticket.py path head-and-arm-calibration-fro design`) (decisions D1-D12, review focus).

