---
title: Floor fit HTTP API
status: todo
repo: ros2-lerobot-sverve-platform
value: "Compute, review and apply a floor fit from the Calibrate page; Apply puts the fitted chain into TF through head_chain.json."
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/floor_results.py, nodes/head_rgbd_calibration/head_rgbd_calibration/floor_api.py, nodes/head_rgbd_calibration/head_rgbd_calibration/__main__.py, nodes/head_rgbd_calibration/tests/test_floor_results.py, nodes/head_rgbd_calibration/tests/test_floor_api.py, nodes/head_rgbd_calibration/tests/test_main.py, nodes/head_rgbd_calibration/tests/fixtures/api/floor_state.json, nodes/head_rgbd_calibration/tests/fixtures/api/floor_result.json]
verify: "cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint"
depends_on: [floor-sweep-capture, head-state-publisher-chain]
owns: [head-floor-fit-api, head-chain-applied-record]
uses: [floor-sweep, head-floor-fit, head-chain-override]
model: ""
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/head-chain-floor-plane-fit-a--floor-fit-http-api
branch: head-chain-floor-plane-fit-a--floor-fit-http-api
verified_head: ""
run: head-chain-floor-plane-fit-a
provides: []
acs_for_audit: []
process_gaps: []
---

# Floor fit HTTP API

## Requirement

Compute, apply and the HTTP routes of the head floor fit (owns `head-floor-fit-api` and `head-chain-applied-record`; design D6, D7, D8). Same error body `{ok: false, message}` and status mapping as `app.py`.

- New `head_rgbd_calibration/floor_results.py`:
  - `base_urdf(cfg) -> str`: `cfg.head_urdf_file` text with `urdf_text.apply_overrides(text, <state_dir>/applied)` (the applied tof_joint and any applied head chain), each warning logged.
  - `compute_floor(cfg, sid, now) -> dict`: reads `floor_fit/<sid>/observations.json`, runs `fit_head_chain_floor(observations, base_urdf(cfg), cfg.floor.fit)`, writes `floor_fit/<sid>/results/<rid>/result.json` (`{id, session, created, passed, failed_limits, params, height_rms_mm, normal_rms_deg, poses, skipped}`) and, only when passed, `head_chain.json` from `head_chain_json` with every joint of the currently applied head chain plus the new `head_mount_joint` and `head_tilt` origins (note: date, height RMS, normal RMS). Synchronous (milliseconds; no child process).
  - `apply_floor(cfg, sid, rid, now, run) -> dict`: refuses (NotApplicable, 409) a result without `passed: true` or when `applied/applied.json` is absent (no RGB-D calibration applied, so the ToF pose the fit assumed is not the deployed one); copies `head_chain.json` into `applied/` (temp file + `os.replace`), then rewrites `applied/applied.json` atomically with every existing key unchanged plus `head_chain: {session, result, applied_at}` (the change starts the apply path unit, which restarts head_state_publisher); returns `{head_chain: {...}, units}` with `apply.unit_state` per `cfg.restart_units` against the head chain `applied_at`.
  - `applied_head_chain(cfg) -> dict | None`: `{session, result, note, joints}` read from `applied/head_chain.json` itself (so an RGB-D apply that rewrites applied.json does not hide it); None when absent or invalid.
- New `head_rgbd_calibration/floor_api.py`: `floor_router(sweep, capture, job, cfg, run) -> APIRouter`:
  - `GET /api/floor/state` -> `{sweep: sweep.state(), applied: applied_head_chain(cfg)}`.
  - `POST /api/floor/sweep/start` -> state; 409 while an RGB-D capture, a compute or a sweep runs.
  - `POST /api/floor/sweep/stop` -> state (also when idle).
  - `GET /api/floor/sessions` -> `[{id, created, observations, skipped, results}]`.
  - `POST /api/floor/sessions/{sid}/compute` -> the result body; 409 while a sweep runs; 404 for an unknown or malformed id (`store.InvalidId` rules for ids).
  - `GET /api/floor/sessions/{sid}/results/{rid}` -> result.json; `POST .../apply` -> the apply body.
- `__main__.py`: builds `FloorSweep` (urdf_text = `base_urdf(cfg)`), `app.include_router(floor_router(...))` after `build_app`; shutdown stops the sweep first (head restored), then the capture, the job and the node.
- Fixtures `tests/fixtures/api/floor_state.json` and `tests/fixtures/api/floor_result.json` (checked with `tests/contract.py` like the others; the web UI tests import them).
- New `tests/test_floor_results.py` (synthetic observations from `tests/head_floor_synthetic.py` loaded by path, tmp state dir): a passing compute writes result.json and head_chain.json with the fitted joints and the previously applied joints kept; a failing compute writes no head_chain.json; apply copies the file, keeps every applied.json key and adds `head_chain`; apply refused for a failed result and without applied.json; `applied_head_chain` survives an applied.json rewrite without the key.
- New `tests/test_floor_api.py` (fake sweep, capture and job): every route body matches its fixture; 409 cases; ids validated. `tests/test_main.py`: shutdown order with the sweep first.
- The whole node suite passes.

## Acceptance conditions

- a1: a passing compute writes result.json and head_chain.json keeping the applied joints [evidence: test:nodes/head_rgbd_calibration/tests/test_floor_results.py::test_compute_writes_result_and_head_chain]
- a2: a failing compute writes no head_chain.json [evidence: test:nodes/head_rgbd_calibration/tests/test_floor_results.py::test_failed_compute_writes_no_head_chain]
- a3: apply copies head_chain.json and adds the head_chain record keeping every applied.json key [evidence: test:nodes/head_rgbd_calibration/tests/test_floor_results.py::test_apply_keeps_applied_keys_and_adds_record]
- a4: apply is refused for a failed result and without an applied RGB-D calibration [evidence: test:nodes/head_rgbd_calibration/tests/test_floor_results.py::test_apply_refusals]
- a5: route bodies match the floor fixtures and busy cases answer 409 [evidence: test:nodes/head_rgbd_calibration/tests/test_floor_api.py::test_routes_match_fixtures_and_conflicts]
- a6: shutdown stops the sweep first [evidence: test:nodes/head_rgbd_calibration/tests/test_main.py::test_shutdown_stops_floor_sweep_first]
- a7: the node suite and lint pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint]

## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/floor_results.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/floor_api.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/__main__.py`
- `nodes/head_rgbd_calibration/tests/test_floor_results.py`
- `nodes/head_rgbd_calibration/tests/test_floor_api.py`
- `nodes/head_rgbd_calibration/tests/test_main.py`
- `nodes/head_rgbd_calibration/tests/fixtures/api/floor_state.json`
- `nodes/head_rgbd_calibration/tests/fixtures/api/floor_result.json`

## Interfaces

- `floor_results.py::base_urdf`, `compute_floor`, `apply_floor`, `applied_head_chain` (contract `head-chain-applied-record`).
- `floor_api.py::floor_router(sweep, capture, job, cfg, run) -> APIRouter`; the routes of the design's Floor fit API table; fixtures `floor_state.json`, `floor_result.json` (contract `head-floor-fit-api`).

## Expected outcome

The Calibrate page can start a sweep, compute a result and apply a passing one; after Apply, `applied/head_chain.json` exists and applied.json changed, so head_state_publisher restarts on the fitted chain.

## Steps

- Write `tests/test_floor_results.py`, `tests/test_floor_api.py`, the fixtures and the shutdown test first; run `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_floor_results.py tests/test_floor_api.py tests/test_main.py -q` and see them fail.
- Implement `floor_results.py`, `floor_api.py`, then the `__main__.py` wiring.
- Run `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint`; expect every test passed and lint clean.

## Notes

- Design D6, D7. Do not edit `app.py`, `store.py` or `compute.py` (in-progress per-pose-quality-hints-in-th); the router is included in `__main__.py`.
- Reuse `apply.unit_state` and `store.write_json_atomic`; load `tests/head_floor_synthetic.py` by path as `tests/pose_factory.py` loads `tests/rgbd_synthetic.py`.

- 2026-10-21 run head-chain-floor-plane-fit-a: dependency-failed on floor-sweep-capture
