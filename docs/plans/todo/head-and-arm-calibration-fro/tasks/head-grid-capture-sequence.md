---
title: Head grid capture sequence
status: todo
repo: ros2-lerobot-sverve-platform
value: "Head calibrate drives the pan/tilt grid by itself; the user only moves the board between spots."
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/sequencer.py, nodes/head_rgbd_calibration/head_rgbd_calibration/config.py, nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml, nodes/head_rgbd_calibration/tests/test_sequencer_head.py, nodes/head_rgbd_calibration/tests/test_config.py]
verify: "cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_sequencer_head.py tests/test_config.py -q && uv run poe lint"
depends_on: [robot-mcp-client]
owns: [calib-sequencer]
uses: [calib-still-capture, calib-robot-client]
model: ""
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Head grid capture sequence

## Requirement

The Head calibrate sequence: pan/tilt grid per board spot through mcp_server (owns `calib-sequencer`; design D6, D9).

- `config.py` and the shipped YAML: `head_sequence: HeadSequenceConfig`: `pan_deg` [-45, -30, -15, 0, 15, 30, 45], `tilt_deg` [35, 45, 55, 60] (+ is down), `settle_s` 0.8, `max_spots` 4 (ge 3), `spot_hints` ["front 0.6 m", "left 0.7 m", "right 0.7 m", "front 1.0 m"] (length >= max_spots, validated).
- New `head_rgbd_calibration/sequencer.py`: `Sequencer(capture: CaptureController, robot: RobotClient, cfg: NodeConfig, sleep: Callable[[float], None] = time.sleep)`; one sequence at a time in a worker thread; `Busy` when a sequence or a capture runs.
  - `start_head(session_id: str | None) -> str`: `capture.start_still_session("head_chain", session_id)`, spot 0, then for each (tilt, pan) grid point: `set_head_angles`, `sleep(settle_s)`, `capture_still(spot)`; counts captured and skipped (with the outcome reason per point). At the end of a spot: `head_home()`, status `spot N done - place the board at <hint> and press Next spot`.
  - `next_spot()`: runs the grid for spot + 1; refused (Busy or ValueError with the reason) while running or past `max_spots`.
  - `stop()`: cancels after the current step, `head_home()`, `capture.stop()`.
  - A robot tool error (`is_error`) stops the sequence: status `stopped: <tool> failed: <text>`, `head_home()` attempted, capture stopped. A refused angle (outside the head limits) counts as skipped with the tool text, not as an error.
  - `state() -> {running, kind, session_id, spot, step, total, captured, skipped, status, error}`.
- New `tests/test_sequencer_head.py` (fake RobotClient and fake capture): call order per grid point; skipped points with reasons; next_spot advances the spot and refuses past max_spots; stop mid-grid homes the head and stops the capture; tool error path; Busy; state keys. `tests/test_config.py`: head_sequence defaults and the hints validation.

## Acceptance conditions

- a1: each grid point moves the head, settles and captures a still [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer_head.py::test_grid_moves_settles_and_captures]
- a2: next_spot advances and refuses past max_spots [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer_head.py::test_next_spot_advances_and_is_bounded]
- a3: stop homes the head and stops the capture [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer_head.py::test_stop_homes_head_and_stops_capture]
- a4: a tool error stops the sequence and names the tool [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer_head.py::test_tool_error_stops_sequence]
- a5: config and sequencer tests pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_sequencer_head.py tests/test_config.py -q]
- a6: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run poe lint]

## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/sequencer.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py`
- `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`
- `nodes/head_rgbd_calibration/tests/test_sequencer_head.py`
- `nodes/head_rgbd_calibration/tests/test_config.py`

## Interfaces

- `sequencer.py::Sequencer`, `start_head`, `next_spot`, `stop`, `state`, `Busy` (part `calib-sequencer`); config `head_sequence`.

## Expected outcome

Head calibrate drives the pan/tilt grid by itself; the user only moves the board between spots.

## Steps

- Tests first (red), implement, verify.
- Run `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_sequencer_head.py tests/test_config.py -q && uv run poe lint`; expect every test passed and lint clean.

## Notes

- Design: the epic design.md (`ticket.py path head-and-arm-calibration-fro design`) (decisions D1-D12, review focus).

