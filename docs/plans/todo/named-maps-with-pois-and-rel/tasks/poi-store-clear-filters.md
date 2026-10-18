---
title: POI store clear filters
status: todo
repo: ros2-lerobot-sverve-platform
value: "POIs can be cleared in one step: all of them, or only one creator's or one kind's, optionally keeping keep-out and slow zones."
files: [nodes/poi_store/poi_store/models.py, nodes/poi_store/poi_store/store.py, nodes/poi_store/tests/test_clear_filters.py, nodes/poi_store/README.md]
verify: "cd nodes/poi_store && uv run --frozen pytest tests/test_clear_filters.py -q && uv run --frozen pytest -q && uv run --frozen poe lint"
depends_on: []
owns: [poi-clear-filters]
uses: []
model: sonnet
estimate: 50
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# POI store clear filters

## Requirement

Extend poi_store's `clear` op (design D7), backward compatible with claude_agent's New-session clear.

- `Command.created_by` stays `"agent" | "user" | None`, but `clear` no longer requires it: absent or null clears every
  creator. New optional fields: `kinds: list["point" | "area" | "object"] | None` (None: every kind; [] deletes
  nothing) and `keep_zones: bool = False` (true keeps areas whose `zone` is `keepout` or `slow`, any status).
  `statuses` keeps its meaning. A POI is cleared only when it matches every given filter.
- `PoiStore.op_clear(created_by, statuses, kinds, keep_zones)`: one commit (revision +1) only when something was
  removed; the result stays `{"removed": n}`; photos are never cleared; cleared POIs are stripped from photo links;
  a failed write rolls back as today.
- `nodes/poi_store/tests/test_clear_filters.py` (test first, seen failing): `test_clear_without_created_by_removes_every_creator`,
  `test_clear_kinds_filter`, `test_clear_keep_zones_keeps_keepout_and_slow_areas`,
  `test_clear_filters_combine`, `test_clear_agent_only_unchanged` (the claude_agent New-session payload removes the same
  POIs as before), `test_clear_nothing_matching_keeps_revision`.
- README `/poi/command` table and the `clear` bullet document the new fields.

## Acceptance conditions

- a1: clear without created_by removes POIs of both creators and leaves photos [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_clear_filters.py::test_clear_without_created_by_removes_every_creator -q]
- a2: kinds limits the clear to those kinds [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_clear_filters.py::test_clear_kinds_filter -q]
- a3: keep_zones keeps keepout and slow areas [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_clear_filters.py::test_clear_keep_zones_keeps_keepout_and_slow_areas -q]
- a4: the existing agent New-session payload behaves as before [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_clear_filters.py::test_clear_agent_only_unchanged -q]
- a5: a clear that removes nothing does not move the revision [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_clear_filters.py::test_clear_nothing_matching_keeps_revision -q]
- a6: the whole poi_store suite and lint pass [evidence: command: cd nodes/poi_store && uv run --frozen pytest -q && uv run --frozen poe lint]
- a7: README documents the clear filters [evidence: command: grep -q keep_zones nodes/poi_store/README.md]

## Files

- `nodes/poi_store/poi_store/models.py`
- `nodes/poi_store/poi_store/store.py`
- `nodes/poi_store/tests/test_clear_filters.py`
- `nodes/poi_store/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
