---
title: POI store replace op
status: done
repo: ros2-lerobot-sverve-platform
value: "Loading a named map can swap in that map's POIs atomically, so the POIs always belong to the map that is loaded."
files: [nodes/poi_store/poi_store/models.py, nodes/poi_store/poi_store/store.py, nodes/poi_store/tests/test_replace.py, nodes/poi_store/README.md]
verify: "cd nodes/poi_store && uv run --frozen pytest tests/test_replace.py -q && uv run --frozen pytest -q && uv run --frozen poe lint"
depends_on: [poi-store-clear-filters]
owns: [poi-replace-op]
uses: []
model: sonnet
estimate: 55
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/named-maps-with-pois-and-rel--poi-store-replace-op
branch: named-maps-with-pois-and-rel--poi-store-replace-op
verified_head: 8ff9cf61ce62f438ce7de1c6fee488aa282ba191
run: named-maps-with-pois-and-rel
provides: []
acs_for_audit: []
process_gaps: []
---

# POI store replace op

## Requirement

Add a `replace` op to poi_store (design D8): `{"op": "replace", "request_id": ..., "pois": [ {...}, ... ]}`.

- `Command.op` gains `"replace"`; `pois: list[dict] | None` is required for replace (error names it) and ignored for
  other ops.
- Every entry is validated through `Poi` before anything changes; each must carry a valid `id`; duplicate ids are
  rejected; one invalid entry rejects the whole command (`ok: false` naming the index and reason) with the store and
  revision untouched. Kind `photo` entries are rejected (photos are never part of a map bundle).
- On success the non-photo POI set becomes exactly the given POIs (ids, `created_at`, `updated_at`, `created_by` kept
  as given), the revision increments by one (never reset to the bundle's value), the file is written atomically, and
  photo links to POI ids no longer present are stripped. The result is `{"replaced": n, "removed": m}`.
- A failed write rolls memory and revision back (as `apply` does today) and answers `ok: false`.
- `nodes/poi_store/tests/test_replace.py` (test first): `test_replace_swaps_the_set_and_keeps_ids`,
  `test_replace_rejects_any_invalid_entry_atomically`, `test_replace_rejects_duplicate_ids`,
  `test_replace_rejects_photo_kind`, `test_replace_increments_revision`, `test_replace_strips_stale_photo_links`,
  `test_replace_write_failure_rolls_back`.
- README documents `replace` in the `/poi/command` table and bullets.

## Acceptance conditions

- a1: replace swaps the POI set keeping ids and timestamps [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_replace.py::test_replace_swaps_the_set_and_keeps_ids -q]
- a2: one invalid entry rejects the whole replace and leaves the store and revision untouched [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_replace.py::test_replace_rejects_any_invalid_entry_atomically -q]
- a3: duplicate ids and photo entries are rejected [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_replace.py::test_replace_rejects_duplicate_ids tests/test_replace.py::test_replace_rejects_photo_kind -q]
- a4: the revision only increases [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_replace.py::test_replace_increments_revision -q]
- a5: photo links to removed POIs are stripped [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_replace.py::test_replace_strips_stale_photo_links -q]
- a6: a failed write rolls back [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_replace.py::test_replace_write_failure_rolls_back -q]
- a7: README documents replace [evidence: command: grep -q '"replace"' nodes/poi_store/README.md]

## Files

- `nodes/poi_store/poi_store/models.py`
- `nodes/poi_store/poi_store/store.py`
- `nodes/poi_store/tests/test_replace.py`
- `nodes/poi_store/README.md`

## Interfaces

- `Command.op` `"replace"`, `Command.pois`.
- `PoiStore.op_replace(pois) -> Result` with result `{"replaced": n, "removed": m}`.

## Expected outcome

A replace swaps the whole non-photo POI set atomically or changes nothing; costmap_zones and every `/poi/list` reader see one new revision.

## Steps

- Write the test(s) named in the requirement first; run `cd nodes/poi_store && uv run --frozen pytest tests/test_replace.py -q` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `cd nodes/poi_store && uv run --frozen pytest tests/test_replace.py -q && uv run --frozen pytest -q && uv run --frozen poe lint`; expect every test passed and lint clean.
- Commit: `feat(poi_store): replace op for loading a map's POIs`.

## Notes

Reuse `apply`'s rollback (pois_before / revision_before) and `strip_photo_links`. The node republishes `/poi/list` because the revision moved; no node.py change is needed.
- Servo safety (CLAUDE.md): this task moves no servo.
