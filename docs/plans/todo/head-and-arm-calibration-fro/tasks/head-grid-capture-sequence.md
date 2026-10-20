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

The Head calibrate sequence: pan/tilt grid per board spot through mcp_server (owns `calib-sequencer`; design D6, D9). Board moves between spots are done by hand by the user; the base never drives.

- `config.py` and the shipped YAML: `head_sequence: HeadSequenceConfig`: `pan_deg` [-45, -30, -15, 0, 15, 30, 45], `tilt_deg` [35, 45, 55, 60] (+ is down), `settle_s` 0.8, `max_spots` 4 (ge 3), `spot_hints` ["front 0.6 m", "left 0.7 m", "right 0.7 m", "front 1.0 m"] (length >= max_spots, validated).
- New `head_rgbd_calibration/sequencer.py`: `Sequencer(capture: CaptureController, robot: RobotClient, cfg: NodeConfig, sleep: Callable[[float], None] = time.sleep)`; one sequence at a time in a worker thread; `Busy` when a sequence or a capture runs.
  - Lease (mcp_server `head_tools.py` LEASE_NOTE: the head tools need the arm or the base lease, and releasing the controlling lease sends the head home): each spot run starts with `robot.take_arm()` (the head sequence never moves the arm) and ends with `robot.release_arm(keep_position=True)` (no arm park; the release homes the head). The release runs on every exit path: spot finished, `stop()`, a tool error and an exception. No separate `head_home()` call.
  - `start_head(session_id: str | None) -> str`: `capture.start_still_session("head_chain", session_id)`, spot 0, take the lease, then for each (tilt, pan) grid point: `set_head_angles`, `sleep(settle_s)`, `capture_still(spot)`; counts captured and skipped (with the outcome reason per point). At the end of a spot: release the lease, status `spot N done - place the board at <hint> and press Next spot`. The capture session stays open between spots.
  - `next_spot()`: takes the lease again and runs the grid for spot + 1; refused (Busy or ValueError with the reason) while running or past `max_spots`.
  - `stop()`: cancels after the current step, releases the lease, `capture.stop()`.
  - A robot tool error (`is_error`) stops the sequence: status `stopped: <tool> failed: <text>`, lease released, capture stopped. A refused angle (outside the head limits) counts as skipped with the tool text, not as an error. A failed `take_arm` stops before any head command.
  - `state() -> {running, kind, session_id, spot, step, total, captured, skipped, status, error}`.
- New `tests/test_sequencer_head.py` (fake RobotClient and fake capture): call order per grid point inside take/release; the release (keep_position true) on spot end, stop, tool error and exception, and no `head_home` call; no head command when `take_arm` fails; skipped points with reasons; next_spot re-takes the lease, advances the spot and refuses past max_spots; Busy; state keys. `tests/test_config.py`: head_sequence defaults and the hints validation.

## Acceptance conditions

- a1: each grid point moves the head, settles and captures a still while the arm lease is held [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer_head.py::test_grid_moves_settles_and_captures]
- a2: the lease is released without parking the arm on spot end, stop, tool error and exception [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer_head.py::test_lease_released_on_every_exit]
- a3: a failed take_control sends no head command [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer_head.py::test_no_head_command_without_lease]
- a4: next_spot re-takes the lease, advances and refuses past max_spots [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer_head.py::test_next_spot_advances_and_is_bounded]
- a5: a tool error stops the sequence and names the tool [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer_head.py::test_tool_error_stops_sequence]
- a6: config and sequencer tests pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_sequencer_head.py tests/test_config.py -q]
- a7: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run poe lint]

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

- 2026-10-20 amended: builder check: head tools need a lease, three arm tiers with confirms, gripper set once, mcp-token via supplementary_groups, optional drive check
