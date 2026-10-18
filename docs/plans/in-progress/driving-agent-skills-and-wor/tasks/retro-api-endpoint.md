---
title: Retro API endpoint
status: in-progress
repo: ros2-lerobot-sverve-platform
value: Retrospectives can be read over HTTP by the web UI and by scripts.
files: [nodes/claude_agent/claude_agent/api.py, nodes/claude_agent/claude_agent/__main__.py, nodes/claude_agent/tests/test_retro_api.py, nodes/claude_agent/README.md]
verify: "cd nodes/claude_agent && uv run --frozen pytest tests/test_retro_api.py tests/test_api.py tests/test_main.py -q && uv run --frozen poe lint"
depends_on: [retro-followup-and-feed, retro-store-and-tool]
owns: [retro-api]
uses: [retro-store]
model: sonnet
estimate: 35
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Retro API endpoint

## Requirement

- `api.py` (D10, U4): `GET /api/retrospectives?limit=50&before=<unix s>` -> `{"retrospectives": [<Retrospective as JSON>...], "total": <count>}` newest first; limit 1..500 (422 outside), before optional float; the runner protocol exposes the store (`retrospectives(limit, before)`, `retrospective_count()`); no session or busy state needed (works while an instruction runs).
- `__main__.py`: wiring unchanged except what the store needs (state_dir path).
- `README.md`: the API section lists the endpoint and its JSON shape (the contract the web UI and the script use).
- Tests first in new `tests/test_retro_api.py` (FastAPI TestClient, fake runner), seen failing: list newest first; limit and before; bounds 422; empty store -> [] and total 0; works while busy.

## Acceptance conditions

- a1: GET /api/retrospectives lists newest first with limit, before and total [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_retro_api.py -q -k list]
- a2: limit bounds are enforced and an empty store returns an empty list [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_retro_api.py -q -k 'bounds or empty']
- a3: the endpoint works while an instruction runs [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_retro_api.py -q -k busy]
- a4: README documents the endpoint [evidence: command: grep -q '/api/retrospectives' nodes/claude_agent/README.md]
- a5: existing api and main tests and lint pass [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_api.py tests/test_main.py -q && uv run --frozen poe lint]

## Files

- `nodes/claude_agent/claude_agent/api.py`
- `nodes/claude_agent/claude_agent/__main__.py`
- `nodes/claude_agent/tests/test_retro_api.py`
- `nodes/claude_agent/README.md`

## Interfaces

- `GET /api/retrospectives` -> `{retrospectives: Retrospective[], total: int}` (contract `retro-api`).

## Expected outcome

The retrospective store is readable over the claude_agent API.

## Steps

- Write `tests/test_retro_api.py`; run it; expect failures.
- Implement; run the verify command; commit `feat(claude_agent): retrospectives API`.

## Notes

- Precedent: `GET /api/retry_outcomes` in api.py.
