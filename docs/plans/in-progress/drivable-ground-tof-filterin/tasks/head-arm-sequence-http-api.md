---
title: Head arm sequence HTTP API
status: blocked
repo: ros2-lerobot-sverve-platform
value: "The web UI can drive the guided head-from-arm calibration, with every confirm and refusal enforced by the node, not only by the browser."
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/head_arm_api.py, nodes/head_rgbd_calibration/head_rgbd_calibration/__main__.py, nodes/head_rgbd_calibration/tests/test_head_arm_api.py, nodes/head_rgbd_calibration/tests/test_main.py, nodes/head_rgbd_calibration/tests/contract.py, nodes/head_rgbd_calibration/tests/fixtures/api/head_arm_state.json, nodes/head_rgbd_calibration/tests/fixtures/api/head_arm_result.json, nodes/head_rgbd_calibration/tests/fixtures/api/head_arm_sessions.json]
verify: "cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint"
depends_on: [head-arm-pose-capture-sequen, head-arm-compute-and-apply]
owns: [head-arm-calib-api]
uses: [calib-sequencer, calib-robot-client, calib-head-arm-results]
model: ""
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/drivable-ground-tof-filterin--head-arm-sequence-http-api
branch: drivable-ground-tof-filterin--head-arm-sequence-http-api
verified_head: unknown
run: drivable-ground-tof-filterin
provides: []
acs_for_audit: []
process_gaps: []
---

# Head arm sequence HTTP API

## Requirement

HTTP routes for the guided head-from-arm calibration (owns the `head-arm-calib-api` contract; design D36, D38;
approvals H2, H3). Same error body `{ok: false, message}` and status mapping as `floor_api.py` (InvalidId and NotFound
404, Busy and NotApplicable 409, a refused step 400).

- New `head_rgbd_calibration/head_arm_api.py`, `head_arm_router(sequencer, capture, sweep, job, store, cfg, run) ->
  APIRouter` with prefix `/api/head-arm`:
  - `GET /state` -> `{sequence: Sequencer.state(), applied: floor_results.applied_head_chain(cfg)}`.
  - `POST /sequence/base` body `{confirm: bool}` -> state; 400 without `confirm: true`; 409 while a capture, a compute,
    a floor sweep or a sequence runs.
  - `POST /sequence/board-attached` -> state; 409 outside phase `waiting_board`.
  - `POST /sequence/big-joints` body `{confirm: bool}` -> state; 409 outside phase `waiting_big_joints`.
  - `POST /sequence/stop` -> state (also when idle).
  - `POST /park` body `{board_removed: bool}` -> state; 400 without `board_removed: true`; 409 while running.
  - `GET /sessions` (the `head_arm` sessions with `{id, created, views, arm_poses, results}`), `POST
    /sessions/{sid}/compute` (409 while a sequence runs), `GET /sessions/{sid}/results/{rid}`, `POST
    /sessions/{sid}/results/{rid}/apply` (409 for a failed result).
- `__main__.py`: builds `RobotClient` and `Sequencer` (head URDF `floor_results.base_urdf`, arm URDF
  `cfg.arm_urdf_file`); the floor sweep's `blocked` also names a running sequence and the sequencer's `blocked` names a
  running capture, compute or floor sweep; includes the router; shutdown stops the sequence first (the arm holds), then
  the capture, the sweep, the job and the node.
- Fixtures in `tests/fixtures/api/`: new `head_arm_state.json`, `head_arm_result.json`, `head_arm_sessions.json`;
  `tests/contract.py` checks them like the floor fixtures.
- New `tests/test_head_arm_api.py` with a fake sequencer: every route, bodies matching the fixtures (contract), the
  400 and 409 cases, `confirm`/`board_removed` passed through only when true, compute and apply through
  `head_arm_results`. `tests/test_main.py`: wiring and shutdown order.
- The whole node suite passes.

## Acceptance conditions

- a1: route bodies match the contract fixtures [evidence: test:nodes/head_rgbd_calibration/tests/test_head_arm_api.py::test_bodies_match_fixtures]
- a2: base move and park are refused without their confirms [evidence: test:nodes/head_rgbd_calibration/tests/test_head_arm_api.py::test_confirms_required]
- a3: busy and wrong-phase requests answer 409 [evidence: test:nodes/head_rgbd_calibration/tests/test_head_arm_api.py::test_busy_and_phase_conflicts]
- a4: a failed result cannot be applied over HTTP [evidence: test:nodes/head_rgbd_calibration/tests/test_head_arm_api.py::test_apply_failed_result_conflict]
- a5: shutdown stops the sequence first [evidence: test:nodes/head_rgbd_calibration/tests/test_main.py::test_shutdown_stops_sequence_first]
- a6: the node suite passes [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q]
- a7: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run poe lint]

## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/head_arm_api.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/__main__.py`
- `nodes/head_rgbd_calibration/tests/test_head_arm_api.py`
- `nodes/head_rgbd_calibration/tests/test_main.py`
- `nodes/head_rgbd_calibration/tests/contract.py`
- `nodes/head_rgbd_calibration/tests/fixtures/api/head_arm_state.json`
- `nodes/head_rgbd_calibration/tests/fixtures/api/head_arm_result.json`
- `nodes/head_rgbd_calibration/tests/fixtures/api/head_arm_sessions.json`

## Interfaces

- `/api/head-arm/*` routes and the three fixtures (part `head-arm-calib-api`); the web UI proxies them under
  `/api/calibration/rgbd/` like the floor routes.

## Expected outcome

The web UI can drive the guided head-from-arm calibration, with every confirm and refusal enforced by the node, not
only by the browser.

## Steps

- Write the fixtures and `tests/test_head_arm_api.py` first; run them: red.
- Implement the router and the wiring; run `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint`;
  expect every test passed and lint clean.

## Notes

- Model it on `floor_api.py`; keep the floor routes unchanged.
- Design: `ticket.py path drivable-ground-tof-filterin design` (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).

- 2026-10-22 run drivable-ground-tof-filterin: dependency-failed on head-arm-pose-capture-sequen
