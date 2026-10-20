---
title: Sequence HTTP API
status: todo
repo: ros2-lerobot-sverve-platform
value: "The web UI can start, advance and stop both calibrations and read their results through one pinned API."
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/app.py, nodes/head_rgbd_calibration/head_rgbd_calibration/__main__.py, nodes/head_rgbd_calibration/tests/test_app.py, nodes/head_rgbd_calibration/tests/test_main.py, nodes/head_rgbd_calibration/tests/contract.py, nodes/head_rgbd_calibration/tests/fixtures/api/state.json, nodes/head_rgbd_calibration/tests/fixtures/api/result_head_chain.json, nodes/head_rgbd_calibration/tests/fixtures/api/result_arm.json]
verify: "cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint"
depends_on: [arm-pose-capture-sequence, arm-offsets-compute-apply]
owns: [head-arm-calib-api]
uses: [calib-sequencer, calib-robot-client, calib-head-result-files]
model: ""
estimate: 75
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Sequence HTTP API

## Requirement

HTTP routes for the Head and Arm sequences and results (owns the `head-arm-calib-api` contract; design D9). Same error body `{ok: false, message}` and status mapping as today.

- `app.py`:
  - `POST /api/sequence/head/start` body `{session_id?: str}` -> the state body; 409 while a capture, a compute or a sequence runs.
  - `POST /api/sequence/head/next-spot` -> state; 409 while running, without a head sequence, or past `max_spots`.
  - `POST /api/sequence/arm/start` body `{session_id?: str, allow_wrist_elbow: bool = false, allow_big_joints: bool = false}` -> state; 409 when no head chain is applied (`applied.json` kind `head_chain` or `arm`) or busy; 422 for `allow_big_joints` without tier 1 allowed now or already captured in the session.
  - `POST /api/sequence/stop` -> state (also when idle).
  - `GET /api/state` (and every state body) adds `sequence` (`Sequencer.state()`); the `applied` record carries `kind` and `head_session`.
  - `POST /api/sessions/{sid}/compute` body optional `{floor_session?: str}`; minimum views per kind (rgbd `limits.min_views`, head_chain `fit.head_limits.min_views`, arm `fit.arm_limits.min_views`); 409 while a sequence runs.
  - `GET /api/sessions` items and `GET /api/sessions/{sid}` include `kind`; `GET /api/sessions/{sid}/results/{rid}` adds `snippet` (the snippet text for arm results, null otherwise).
- `__main__.py`: builds `RobotClient` and `Sequencer`; shutdown stops the sequence first (arm released), then the capture, the job and the node.
- Fixtures in `tests/fixtures/api/`: `state.json` gains `sequence`; new `result_head_chain.json` and `result_arm.json`; `tests/contract.py` checks them like the others.
- `tests/test_app.py`: every new route with a fake sequencer: bodies match the fixtures (contract), 409 cases, both tier flags passed through, big without tier 1 refused, arm start refused without an applied head chain, compute min views per kind, snippet field. `tests/test_main.py`: shutdown order.
- The whole node suite passes.

## Acceptance conditions

- a1: the sequence routes answer the state body matching the fixture [evidence: test:nodes/head_rgbd_calibration/tests/test_app.py::test_sequence_routes_answer_state_contract]
- a2: arm start is refused without an applied head chain [evidence: test:nodes/head_rgbd_calibration/tests/test_app.py::test_arm_start_needs_applied_head_chain]
- a3: the tier flags reach the sequencer only when sent true, and big joints without tier 1 are refused [evidence: test:nodes/head_rgbd_calibration/tests/test_app.py::test_tier_flags_passed_through]
- a4: arm results carry the snippet [evidence: test:nodes/head_rgbd_calibration/tests/test_app.py::test_arm_result_has_snippet]
- a5: shutdown stops the sequence before the capture [evidence: test:nodes/head_rgbd_calibration/tests/test_main.py::test_shutdown_stops_sequence_first]
- a6: the whole node suite passes [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q]
- a7: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run poe lint]

## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/app.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/__main__.py`
- `nodes/head_rgbd_calibration/tests/test_app.py`
- `nodes/head_rgbd_calibration/tests/test_main.py`
- `nodes/head_rgbd_calibration/tests/contract.py`
- `nodes/head_rgbd_calibration/tests/fixtures/api/state.json`
- `nodes/head_rgbd_calibration/tests/fixtures/api/result_head_chain.json`
- `nodes/head_rgbd_calibration/tests/fixtures/api/result_arm.json`

## Interfaces

- Routes `POST /api/sequence/head/start`, `/api/sequence/head/next-spot`, `/api/sequence/arm/start`, `/api/sequence/stop`; `sequence` in every state body; `snippet` in results; fixtures `state.json`, `result_head_chain.json`, `result_arm.json` (contract `head-arm-calib-api`).

## Expected outcome

The web UI can start, advance and stop both calibrations and read their results through one pinned API.

## Steps

- tests and fixtures first (red), implement, verify.
- Run `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint`; expect every test passed and lint clean.

## Notes

- Design: the epic design.md (`ticket.py path head-and-arm-calibration-fro design`) (decisions D1-D12, review focus).

- 2026-10-20 amended: builder check: head tools need a lease, three arm tiers with confirms, gripper set once, mcp-token via supplementary_groups, optional drive check
