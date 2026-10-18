---
title: Web UI POI clear route
status: todo
repo: ros2-lerobot-sverve-platform
value: The web UI can clear POIs in one request with the same filters poi_store supports.
files: [nodes/web_ui/web_ui/server.py, nodes/web_ui/tests/test_poi_clear.py, nodes/web_ui/README.md]
verify: "cd nodes/web_ui && uv run --frozen pytest tests/test_poi_clear.py -q && uv run --frozen pytest -q && uv run --frozen poe lint"
depends_on: [web-ui-named-maps-backend, poi-store-clear-filters]
owns: []
uses: [poi-clear-filters]
model: sonnet
estimate: 40
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Web UI POI clear route

## Requirement

`POST /api/poi?tab=` accepts `{"op": "clear", "created_by": "agent"|"user"|null, "kinds": [...]|null,
"statuses": [...]|null, "keep_zones": bool}` (design D7, D12).

- Validation: `created_by` in agent/user/null, `kinds` subset of point/area/object, `statuses` subset of
  open/done/cancelled, `keep_zones` bool (default false); anything else 422 with the reason. The command forwarded to
  poi_store contains only the given fields plus `op`.
- Same unavailable/timeout handling as the other POI ops; not battery-guarded (not motion).
- The error message for an unknown op lists clear among the accepted ops.
- `nodes/web_ui/tests/test_poi_clear.py` (test first): forwards each filter; rejects bad values; poi_store down 503;
  timeout 504; the result passes through (`{"removed": n}`).
- README POI route section documents clear.

## Acceptance conditions

- a1: clear forwards the given filters to poi_store and returns its result [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_poi_clear.py -q]
- a2: invalid clear filters answer 422 [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_poi_clear.py -q -k invalid]
- a3: README documents the clear op [evidence: command: grep -q '"clear"' nodes/web_ui/README.md]

## Files

- `nodes/web_ui/web_ui/server.py`
- `nodes/web_ui/tests/test_poi_clear.py`
- `nodes/web_ui/README.md`

## Interfaces

- `POST /api/poi` op `clear` in `nodes/web_ui/web_ui/server.py:poi_command`.

## Expected outcome

The POI clear filters reach poi_store from the browser API with validation.

## Steps

- Write the test(s) named in the requirement first; run `cd nodes/web_ui && uv run --frozen pytest tests/test_poi_clear.py -q` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `cd nodes/web_ui && uv run --frozen pytest tests/test_poi_clear.py -q && uv run --frozen pytest -q && uv run --frozen poe lint`; expect every test passed and lint clean.
- Commit: `feat(web_ui): POI clear route`.

## Notes

poi_store validates again; the route rejects bad values early with 422 so the UI gets a clear message.
- Servo safety (CLAUDE.md): this task moves no servo.
