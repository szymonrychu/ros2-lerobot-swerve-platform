---
title: POI store clear filters
status: done
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
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/named-maps-with-pois-and-rel--poi-store-clear-filters
branch: named-maps-with-pois-and-rel--poi-store-clear-filters
verified_head: 1b383009a4bbfe8d7e3f77653048aac96c50b4ed
run: named-maps-with-pois-and-rel
provides: []
acs_for_audit: []
process_gaps: []
---

# POI store clear filters

## Requirement

Extend poi_store's `clear` op (design D7), backward compatible with claude_agent's New-session clear.

- `Command.created_by` stays `"agent" | "user" | None`, but `clear` no longer requires it: absent or null clears every
  creator. New optional fields: `kinds: list["point" | "area" | "object"] | None` (None: every kind; [] deletes
  nothing) and `keep_zones: bool = False` (true keeps areas whose `zone` is `keepout` or `slow`, any status).
  `statuses` keeps its meaning. New optional `ids: list[str] | None` (None: no id filter; [] deletes nothing; unknown ids are ignored): this is the batch delete-by-ids operation, ONE implementation shared with the skills epic's POI list "delete marked" (U10). A POI is cleared only when it matches every given filter.
- `PoiStore.op_clear(created_by, statuses, kinds, keep_zones, ids)`: one commit (revision +1) only when something was
  removed; the result stays `{"removed": n}`; photos are never cleared; cleared POIs are stripped from photo links;
  a failed write rolls back as today.
- `nodes/poi_store/tests/test_clear_filters.py` (test first, seen failing): `test_clear_without_created_by_removes_every_creator`,
  `test_clear_kinds_filter`, `test_clear_keep_zones_keeps_keepout_and_slow_areas`,
  `test_clear_filters_combine`, `test_clear_ids_removes_only_listed`, `test_clear_ids_combine_with_keep_zones`, `test_clear_ids_empty_or_unknown_removes_nothing`, `test_clear_agent_only_unchanged` (the claude_agent New-session payload removes the same
  POIs as before), `test_clear_nothing_matching_keeps_revision`.
- README `/poi/command` table and the `clear` bullet document the new fields.

## Acceptance conditions

- a1: clear without created_by removes POIs of both creators and leaves photos [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_clear_filters.py::test_clear_without_created_by_removes_every_creator -q]
- a2: kinds limits the clear to those kinds [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_clear_filters.py::test_clear_kinds_filter -q]
- a3: keep_zones keeps keepout and slow areas [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_clear_filters.py::test_clear_keep_zones_keeps_keepout_and_slow_areas -q]
- a4: the existing agent New-session payload behaves as before [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_clear_filters.py::test_clear_agent_only_unchanged -q]
- a5: a clear that removes nothing does not move the revision [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_clear_filters.py::test_clear_nothing_matching_keeps_revision -q]
- a8: ids deletes exactly the listed existing POIs, ANDed with the other filters, and empty or unknown ids remove nothing [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_clear_filters.py -q -k clear_ids]
- a6: the whole poi_store suite and lint pass [evidence: command: cd nodes/poi_store && uv run --frozen pytest -q && uv run --frozen poe lint]
- a7: README documents the clear filters [evidence: command: grep -q keep_zones nodes/poi_store/README.md]

## Files

- `nodes/poi_store/poi_store/models.py`
- `nodes/poi_store/poi_store/store.py`
- `nodes/poi_store/tests/test_clear_filters.py`
- `nodes/poi_store/README.md`

## Interfaces

- `nodes/poi_store/poi_store/models.py:Command` fields `kinds`, `keep_zones`, `ids`; `created_by` optional for clear.
- `nodes/poi_store/poi_store/store.py:PoiStore.op_clear(created_by, statuses, kinds, keep_zones, ids)`.

## Expected outcome

`{"op": "clear"}` with any combination of creator, kinds, statuses and keep_zones removes exactly the matching POIs; the New-session agent clear is unchanged.

## Steps

- Write the test(s) named in the requirement first; run `cd nodes/poi_store && uv run --frozen pytest tests/test_clear_filters.py -q` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `cd nodes/poi_store && uv run --frozen pytest tests/test_clear_filters.py -q && uv run --frozen pytest -q && uv run --frozen poe lint`; expect every test passed and lint clean.
- Commit: `feat(poi_store): filtered clear (all creators, kinds, keep zones)`.

## Notes

OWNER of the shared POI delete operation (contract `poi-clear-filters`, now including `ids`). The epic driving-agent-skills-and-wor lists it in `uses`: its `poi-store-clear-by-tag` adds `tag` and `keep_keepout` terms to the same `op_clear` (every filter an independent AND term; whichever lands second rebases and keeps both) and its web UI "delete marked" posts `ids` through this operation. Do not build a second delete path.

claude_agent's `clear_agent_pois` sends `created_by: agent` with optional statuses; mcp_server's admin route forwards it. The printable-markers epic also edits models.py later (Poi fields); keep the change inside `Command` and `op_clear`.
- Servo safety (CLAUDE.md): this task moves no servo.
