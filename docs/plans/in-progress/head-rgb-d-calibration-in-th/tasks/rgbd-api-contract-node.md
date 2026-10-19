---
title: RGBD API contract node
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "Start, Stop and Apply work end to end: the node's responses match one documented contract pinned by JSON fixtures, and result.json carries its limits."
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/app.py, nodes/head_rgbd_calibration/head_rgbd_calibration/apply.py, nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py, nodes/head_rgbd_calibration/tests/test_app.py, nodes/head_rgbd_calibration/tests/test_compute.py, nodes/head_rgbd_calibration/tests/fixtures/api, nodes/head_rgbd_calibration/README.md, shared/ros2_common/head_rgbd_calibration/config.py]
verify: "(cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/ -q && uv run --frozen poe lint) && uv run --frozen pytest tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calibrate_cli.py -q && uv run poe lint"
depends_on: [jetson-retirement-allowlist]
owns: [rgbd-api-fixtures]
uses: []
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/head-rgb-d-calibration-in-th--rgbd-api-contract-node
branch: head-rgb-d-calibration-in-th--rgbd-api-contract-node
verified_head: ""
run: ""
provides: []
---

# RGBD API contract node

## Requirement

Whole-epic review found the node's HTTP responses and the frontend client disagree, which breaks the main operator flow on the robot. The node owns the API contract; this task fixes the node side and pins the contract as literal JSON examples that both sides test against.

- One response shape per route, documented in `nodes/head_rgbd_calibration/README.md`: `POST /api/capture/start` and `POST /api/capture/stop` return the same nested body as `GET /api/state` (`{capture, compute, applied}`), so the frontend can call `onState` with it. `POST /api/sessions/{sid}/results/{rid}/apply` returns the applied record plus the per-unit restart state (`units`: the same list `GET /api/applied` returns), or the frontend gets the units another documented way; pick one and make node, fixture and README agree. The applied record (`applied.json` and every response embedding it) uses one set of key names everywhere (today the node writes `session`/`result` while the TS type says `session_id`/`result_id`); choose the node's names and document them.
- `result.json` (compute.py) carries a `limits` object with every acceptance limit used to judge the result, including a plane depth bias limit: add `plane_max_bias_mm: float = 15.0` (gt 0) to `LimitsConfig` in `shared/ros2_common/head_rgbd_calibration/config.py` and to `failed_limits` (the methodology and runbook already gate on plane bias <= 15 mm), with the YAML default in `scripts/head_rgbd_calibrate.yaml`.
- Contract fixtures: literal JSON example bodies for state, capture start, capture stop, apply, applied, result under `nodes/head_rgbd_calibration/tests/fixtures/api/*.json`; node tests assert the real responses (TestClient with the existing fakes) have exactly the fixture's keys and value types (a structural match helper, not byte equality of stamps).
- Tests first, seen failing: start/stop return the nested state; apply response matches its fixture; result.json has limits including plane_max_bias_mm; failed_limits flags a plane bias over the limit.

## Acceptance conditions

- a1: capture start and stop return the nested {capture, compute, applied} state matching tests/fixtures/api/capture_start.json and capture_stop.json [evidence: test: nodes/head_rgbd_calibration/tests/test_app.py::test_capture_start_and_stop_return_the_state_fixture]
- a2: apply and applied responses match their fixtures, one set of applied-record key names [evidence: test: nodes/head_rgbd_calibration/tests/test_app.py::test_apply_and_applied_match_the_fixtures]
- a3: result.json carries every limit including plane_max_bias_mm [evidence: test: nodes/head_rgbd_calibration/tests/test_compute.py::test_result_carries_the_limits]
- a4: a plane bias over plane_max_bias_mm fails the limits [evidence: test: tests/test_head_rgbd_calib_core.py::test_failed_limits_flags_plane_bias]

## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/app.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/apply.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py`
- `nodes/head_rgbd_calibration/tests/test_app.py`
- `nodes/head_rgbd_calibration/tests/test_compute.py`
- `nodes/head_rgbd_calibration/tests/fixtures/api`
- `nodes/head_rgbd_calibration/README.md`
- `shared/ros2_common/head_rgbd_calibration/config.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

Start, Stop, Apply and applied responses of the node match the documented contract and the JSON fixtures; result.json carries all limits including plane_max_bias_mm.

## Steps

- Write the fixtures and the failing tests (start/stop nested state, apply/applied fixtures, result limits, plane bias limit); run the verify command and see them fail.
- Change app.py/apply.py/compute.py/config.py to the contract; update the node README; run the verify command (all pass) and commit `fix(head_rgbd_calibration): one API contract pinned by JSON fixtures`.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
