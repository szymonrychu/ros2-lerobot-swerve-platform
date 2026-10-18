---
title: POI cleanups delete photos
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "Clearing or deleting POIs removes the photos in that set, records and image files, and says how many"
files: [nodes/poi_store/poi_store/models.py, nodes/poi_store/poi_store/store.py, nodes/poi_store/tests/test_clear_filters.py, nodes/poi_store/tests/test_photo_cleanup.py, nodes/poi_store/README.md]
verify: "cd nodes/poi_store && uv run --frozen pytest tests/test_photo_cleanup.py tests/test_clear_filters.py -q && uv run --frozen poe lint"
depends_on: [poi-list-includes-photos]
owns: [poi-clear-photo-kind]
uses: [poi-list-photo-entries]
model: ""
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/poi-fixes-photos-as-pois-and--poi-cleanups-delete-photos
branch: poi-fixes-photos-as-pois-and--poi-cleanups-delete-photos
verified_head: ""
run: ""
provides: []
---

# POI cleanups delete photos

## Requirement

D7, D9: every POI cleanup deletes the photo POIs in its set (record and JPEG) and reports what it removed.

- `nodes/poi_store/poi_store/models.py`: `Command.kinds` accepts "photo" (`list[Literal["point", "area", "object", "photo"]] | None`); docstrings no longer say photos are never cleared.
- `nodes/poi_store/poi_store/store.py` `op_clear`: photos are matched by the same filters as POIs: `kinds` None matches every kind including photos, `kinds` containing "photo" matches photos, `created_by` and `statuses` compare the photo record's fields, `ids` matches photo ids, `keep_zones` never keeps a photo. Matched photos are removed with their JPEGs in one PhotoStore operation (reuse `PhotoStore.remove` or `delete_many` with an ids selector), the non-photo POIs as today, then link strips for the removed POIs; the revision moves once when anything was removed. Result data `{"removed": total, "pois": n_pois, "photos": n_photos}`, message "cleared N POIs and M photos". A photo store disk failure leaves the POIs untouched and returns ok=false with the reason (photo removal runs before the POI commit, or the POI change is rolled back).
- `op_delete` keeps deleting a photo id with its JPEG; deleting a non-photo POI keeps the linked photos and strips the link (unchanged, now pinned by a test).
- Tests first, seen failing: extend `nodes/poi_store/tests/test_clear_filters.py` and add `nodes/poi_store/tests/test_photo_cleanup.py`: a clear with no filters removes POIs and photos and their JPEGs and reports both counts; `kinds=["photo"]` removes only photos; `kinds=["point"]` keeps photos; `created_by="user"` removes only user photos and POIs; `statuses=["done"]` matches photo status; `ids` with a mix of POI and photo ids removes exactly those (the delete-marked path); `keep_zones=True` still removes photos; deleting a non-photo POI keeps its linked photo and its JPEG; single delete of a photo removes the JPEG; a clear that matches nothing does not bump the revision; a simulated photo store `OSError` leaves POIs and revision unchanged.
- `nodes/poi_store/README.md`: the clear op documents the photo kind, the filters on photos and the result counts.

## Acceptance conditions

- a1: a clear removes the matching photos with their JPEGs and reports POI and photo counts [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_photo_cleanup.py -q -k clear]
- a2: the ids filter deletes marked photos and POIs exactly [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_photo_cleanup.py -q -k ids]
- a3: deleting a non-photo POI keeps its linked photo; deleting a photo removes its JPEG [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_photo_cleanup.py -q -k delete]
- a4: existing clear filter tests pass with the photo kind added [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_clear_filters.py -q]
- a5: poi_store suite and lint pass [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests -q && uv run --frozen poe lint]
- a6: README documents clearing photos [evidence: command: grep -qi 'kinds.*photo' nodes/poi_store/README.md]

## Files

- `nodes/poi_store/poi_store/models.py`
- `nodes/poi_store/poi_store/store.py`
- `nodes/poi_store/tests/test_clear_filters.py`
- `nodes/poi_store/tests/test_photo_cleanup.py`
- `nodes/poi_store/README.md`

## Interfaces

- `nodes/poi_store/poi_store/models.py:Command.kinds: list[Literal["point", "area", "object", "photo"]] | None` (part `poi-clear-photo-kind`).
- `nodes/poi_store/poi_store/store.py:PoiStore.op_clear` result data `{"removed", "pois", "photos"}`.
- Kept: `op_delete` semantics for photo ids and for non-photo POIs (link strip only).

## Expected outcome

A clear (including the delete-marked ids path and "clear all") removes the matching photos with their JPEGs and reports POI and photo counts; deleting an ordinary POI never deletes a photo.

## Steps

- Write `nodes/poi_store/tests/test_photo_cleanup.py` and extend `tests/test_clear_filters.py`; run `cd nodes/poi_store && uv run --frozen pytest tests/test_photo_cleanup.py tests/test_clear_filters.py -q`; see them fail.
- Implement the photo matching and removal in `op_clear`, the model literal and the result data; rerun until green.
- Run `cd nodes/poi_store && uv run --frozen pytest tests -q && uv run --frozen poe lint`.
- Update the README (remove "photos are never cleared"); commit `feat(poi_store): clear deletes the photos it matches`.

## Notes

- Photo records carry `created_by` and `status` (`LookupRecord`), so the filters apply directly.
- Remove photos through PhotoStore in one index rewrite (`remove(ids)` exists) so a clear of many photos does not rewrite the index per photo.
- The driving-agent delete-marked dialog posts `{op: "clear", ids: [...]}`; photo ids in that list must delete photos (orchestrator note).
- Decision D7.
