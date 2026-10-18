---
title: Map clear live and discard
status: todo
repo: ros2-lerobot-sverve-platform
value: Reset map and an unrecoverable move remove the live POIs and photos and leave saved maps alone
files: [nodes/map_manager/map_manager/manager.py, nodes/map_manager/map_manager/node.py, nodes/map_manager/tests/test_clear_live.py, nodes/map_manager/tests/test_manager.py, nodes/map_manager/README.md]
verify: "cd nodes/map_manager && uv run --frozen pytest tests/test_clear_live.py tests/test_manager.py -q && uv run --frozen poe lint"
depends_on: [map-bundle-photos]
owns: []
uses: [map-contract-photos-clear-live, poi-command-photo-ops]
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

# Map clear live and discard

## Requirement

D6, D8, D9: map_manager empties the live POIs and photos on request (web UI Reset map) and on the kidnap discard, and clears the live owner both times.

- `nodes/map_manager/map_manager/manager.py`:
  - `handle` dispatches the new `clear_live` op (shared contract) to `clear_live(command)`: read the live counts from the latched listing (non-photo and photo entries), `poi.replace([])`, then clear and persist `active_map` (only when the replace succeeded); result ok with message "live set cleared: removed N POIs and M photos; saved maps kept" and `data` `{"pois_removed", "photos_removed", "active_map": None}`; a failed or unanswered replace gives ok=false naming poi_store's reason and keeps `active_map`. It is refused like other ops while one runs.
  - `discard_live_pois` returns POI and photo counts (from the replace data); relocalise `data` gains `discarded_photos`; a successful discard clears and persists `active_map`; the message names both counts. Saved bundles are never touched.
- `nodes/map_manager/map_manager/node.py`: no new topic; the command parser accepts the new op through `MapCommand`.
- Tests first, seen failing, in new `nodes/map_manager/tests/test_clear_live.py`: clear_live empties live POIs and photos, reports counts and clears the persisted owner so a later delete of the former owner leaves the (new) live set alone; a failed replace keeps the owner and is not ok; bundles on disk are unchanged; a relocalise with `discard_on_fail` that fails removes live photos, reports `discarded_photos` and clears the owner; a recovered relocalise touches nothing. Update affected relocalise tests in `tests/test_manager.py`.
- `nodes/map_manager/README.md`: the clear_live op and the discard counts.

## Acceptance conditions

- a1: clear_live empties live POIs and photos, reports counts and clears the owner [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_clear_live.py -q -k clear_live]
- a2: the kidnap discard removes live photos, reports discarded_photos and clears the owner [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_clear_live.py -q -k discard]
- a3: saved bundles are untouched by clear_live and discard [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_clear_live.py -q -k bundles]
- a4: map_manager suite and lint pass [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests -q && uv run --frozen poe lint]
- a5: README documents clear_live [evidence: command: grep -q 'clear_live' nodes/map_manager/README.md]

## Files

- `nodes/map_manager/map_manager/manager.py`
- `nodes/map_manager/map_manager/node.py`
- `nodes/map_manager/tests/test_clear_live.py`
- `nodes/map_manager/tests/test_manager.py`
- `nodes/map_manager/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
